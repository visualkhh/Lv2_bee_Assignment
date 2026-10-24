"""Shared pieces of set_center and sin_test: joint list, URDF limits, config, sine profile, `q` key, base node."""
import math
import select
import signal
import sys
import termios
import threading
import time
import tty
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_left_joint']  # arm_controller joint order
LINEAR = {'gripper_left_joint'}  # config in m, everything else in deg
ARM = [j for j in JOINTS if j not in LINEAR]
CONTROLLER = '/arm_controller'

SHARE = Path(get_package_share_directory('sine_test'))
DEFAULT_CONFIG = str(SHARE / 'config' / 'sine.yaml')
# install/sine_test/share/sine_test -> real_test/ros2_ws -> real_test/logs (no resolve(): --symlink-install)
DEFAULT_OUT_DIR = str(SHARE.parents[3].parent / 'logs')


class Quit(Exception):
    """`q` pressed (or Ctrl+C): stop where the arm is and exit normally."""


def dur(t):
    sec, nsec = divmod(int(round(t * 1e9)), 1_000_000_000)  # 6.9999999999 s -> 7 s, never nanosec = 1e9
    return Duration(sec=sec, nanosec=nsec)


def limits_from_urdf(text):
    """{joint: (lower, upper, velocity)} for JOINTS."""
    out = {}
    for j in ET.fromstring(text).iter('joint'):
        lim = j.find('limit')
        if j.get('name') in JOINTS and lim is not None:
            out[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')), float(lim.get('velocity')))
    return out


def center(joint, spec, lim):
    """Test center in SI units: base + offset. Base is 0 rad for the arm (servo upright, 2048) and the middle of
    the URDF range for the gripper (Isaac Gain Tuner definition)."""
    unit = 1.0 if joint in LINEAR else math.pi / 180
    lo, hi, _ = lim
    base = (lo + hi) / 2 if joint in LINEAR else 0.0
    return base + spec.get('offset', 0.0) * unit


def move_time(cfg, lim, delta):
    """Slow move: at least move_s, at most 0.3 x the URDF velocity on average."""
    return max(cfg['move_s'], abs(delta) / (0.3 * lim[2]))


class SineTest:
    """One joint's sine: command as a function of time since the trajectory start."""

    def __init__(self, center, amp, period, cycles, ramp_cycles):
        self.c, self.a, self.w = center, amp, 2 * math.pi / period
        self.tr = ramp_cycles * period
        self.total = cycles * period + 2 * self.tr

    def env(self, t):  # 0 -> 1 -> 0 with zero slope at both ends
        if t <= 0 or t >= self.total: return 0.0, 0.0
        k = math.pi / self.tr
        if t < self.tr: return 0.5 * (1 - math.cos(k * t)), 0.5 * k * math.sin(k * t)
        if t > self.total - self.tr:
            u = self.total - t
            return 0.5 * (1 - math.cos(k * u)), -0.5 * k * math.sin(k * u)
        return 1.0, 0.0

    def __call__(self, t):
        e, de = self.env(t)
        s, ds = math.sin(self.w * t), self.w * math.cos(self.w * t)
        return self.c + self.a * e * s, self.a * (de * s + e * ds)


class KeyWatcher:
    """Reads single keys from the terminal in a thread; `pressed` turns True on `q`. Ctrl+C still raises
    KeyboardInterrupt (cbreak keeps ISIG). Without a tty (e.g. under ros2 launch) it does nothing."""

    def __init__(self):
        self.pressed = False
        self.fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        self.saved = None
        self.alive = True
        if self.fd is None: return
        self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def loop(self):
        while self.alive:
            if select.select([self.fd], [], [], 0.1)[0] and sys.stdin.read(1) in ('q', 'Q'):
                self.pressed = True

    def close(self):
        self.alive = False
        if self.saved is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
            self.saved = None


class ArmNode(Node):
    """Config + URDF limits + latest /joint_states + latest JTC reference, and spin helpers that honor `q`."""

    def __init__(self, name, config=DEFAULT_CONFIG):
        super().__init__(name)
        self.cfg_path = self.declare_parameter('config', config).value
        self.cfg = yaml.safe_load(Path(self.cfg_path).read_text())
        self.state = None      # {joint: (pos, vel, effort)} from /joint_states
        self.state_msg = None
        self.reference = None  # {joint: pos} — what arm_controller is commanding now
        self.urdf = None
        self.keys = None
        self.on_state = None   # optional callback(msg) for logging
        self.create_subscription(JointState, '/joint_states', self._joint_states, 50)
        self.create_subscription(JointTrajectoryControllerState, CONTROLLER + '/controller_state', self._ctrl_state, 10)
        self.create_subscription(String, '/robot_description', lambda m: setattr(self, 'urdf', m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _joint_states(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in JOINTS): return
        g = lambda arr, i: arr[i] if i < len(arr) else float('nan')
        self.state = {j: (msg.position[idx[j]], g(msg.velocity, idx[j]), g(msg.effort, idx[j])) for j in JOINTS}
        self.state_msg = msg
        if self.on_state: self.on_state(msg)

    def _ctrl_state(self, msg):
        if len(msg.reference.positions) == len(msg.joint_names):
            self.reference = dict(zip(msg.joint_names, msg.reference.positions))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_until(self, cond, timeout, what):
        """Spin until cond() — raises Quit on `q`, RuntimeError on timeout."""
        end = time.monotonic() + timeout
        while rclpy.ok() and not cond():
            if self.keys and self.keys.pressed: raise Quit()
            if time.monotonic() > end: raise RuntimeError(f'timeout waiting for {what}')
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin_for(self, seconds):
        """Spin for a while — raises Quit on `q`."""
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            if self.keys and self.keys.pressed: raise Quit()
            rclpy.spin_once(self, timeout_sec=0.02)

    def stopping(self):
        """Called once on `q` / Ctrl+C: ignore further Ctrl+C (ros2 launch and the terminal both send SIGINT)
        and spin a little for a fresh state, without honoring `q` again."""
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        for _ in range(5): rclpy.spin_once(self, timeout_sec=0.01)

    def wait_ready(self):
        """URDF limits, a joint state and a JTC reference. Returns the commanded pose {joint: pos}."""
        self.spin_until(lambda: self.urdf is not None, 10.0, '/robot_description')
        self.lim = limits_from_urdf(self.urdf)
        self.spin_until(lambda: self.state is not None, 10.0, '/joint_states')
        self.spin_until(lambda: self.reference is not None and all(j in self.reference for j in JOINTS), 10.0,
                        CONTROLLER + '/controller_state (is arm_controller active?)')
        return {j: self.reference[j] for j in JOINTS}

    def fmt(self, joint, value):
        return f'{value * 1000:.2f} mm' if joint in LINEAR else f'{math.degrees(value):.2f} deg'


def run_main(node_cls):
    """rclpy init without its SIGINT handler (Ctrl+C -> KeyboardInterrupt, context stays up for cleanup),
    `q` key watcher for the node's lifetime, terminal restored on exit."""
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = node_cls()
    node.keys = KeyWatcher()
    code = 0
    try:
        code = node.run() or 0
    finally:
        node.keys.close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    sys.exit(code)
