"""Sine response test on the real beebot arm.

For each enabled joint in the config (one at a time, the others hold the start pose):
  move to the test center -> sine with fade-in/out -> settle -> move back to the start pose.
The whole sine is sent as one FollowJointTrajectory goal (JTC replays it on its 100 Hz loop), while
/joint_states (position, velocity, effort = servo current [A]) is logged next to the analytic command.

Output: <out_dir>/<YYYYmmdd_HHMMSS>/log.csv, meta.yaml, summary.yaml (Peak / RMS like Isaac Gain Tuner).
Ctrl+C cancels the running goal (JTC stops where it is) and still writes what was logged.
"""
import csv
import math
import signal
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_left_joint']
LINEAR = {'gripper_left_joint'}  # config in m, everything else in deg
ACTION = '/arm_controller/follow_joint_trajectory'
START_DELAY = 0.3  # s between sending a goal and its header stamp, so the start time is known exactly


def dur(t):
    return Duration(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))


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


def limits_from_urdf(text):
    root = ET.fromstring(text)
    out = {}
    for j in root.iter('joint'):
        lim = j.find('limit')
        if j.get('name') in JOINTS and lim is not None:
            out[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')), float(lim.get('velocity')))
    return out


class Runner(Node):
    def __init__(self):
        super().__init__('sine_test')
        self.cfg_path = self.declare_parameter('config', '').value
        self.out_root = Path(self.declare_parameter('out_dir', 'sine_test_logs').value)
        self.mock = bool(self.declare_parameter('mock_hardware', False).value)
        self.cfg = yaml.safe_load(open(self.cfg_path))
        self.state = None          # latest JointState as {name: (pos, vel, eff)}
        self.recording = None      # (joint, SineTest, t0) while a sine is logged
        self.rows = []
        self.urdf = None
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 50)
        self.create_subscription(String, '/robot_description', lambda m: setattr(self, 'urdf', m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)
        self.goal_handle = None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_joint_states(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in JOINTS): return
        g = lambda arr, i: arr[i] if i < len(arr) else float('nan')
        self.state = {j: (msg.position[idx[j]], g(msg.velocity, idx[j]), g(msg.effort, idx[j])) for j in JOINTS}
        if self.recording:
            joint, sine, t0 = self.recording
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - t0
            if t < 0: return
            cp, cv = sine(t)
            p, v, e = self.state[joint]
            self.rows.append([f'{t:.4f}', joint, f'{cp:.6f}', f'{cv:.6f}', f'{p:.6f}', f'{v:.6f}', f'{e:.4f}'])

    def spin_until(self, cond, timeout, what):
        end = time.monotonic() + timeout
        while rclpy.ok() and not cond():
            if time.monotonic() > end: raise RuntimeError(f'timeout waiting for {what}')
            rclpy.spin_once(self, timeout_sec=0.05)

    def send(self, points, t0):
        """points: list of (t, {joint: (pos, vel)}) relative to t0 (sim/ROS time, seconds). Blocks until done."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        sec = int(t0)
        goal.trajectory.header.stamp.sec, goal.trajectory.header.stamp.nanosec = sec, int((t0 - sec) * 1e9)
        for t, q in points:
            goal.trajectory.points.append(JointTrajectoryPoint(
                positions=[q[j][0] for j in JOINTS], velocities=[q[j][1] for j in JOINTS], time_from_start=dur(t)))
        fut = self.client.send_goal_async(goal)
        self.spin_until(fut.done, 5.0, 'goal acceptance')
        self.goal_handle = fut.result()
        if not self.goal_handle.accepted: raise RuntimeError('JTC rejected the trajectory')
        res = self.goal_handle.get_result_async()
        self.spin_until(res.done, points[-1][0] + START_DELAY + 10.0, 'trajectory end')
        self.goal_handle = None
        r = res.result().result
        if r.error_code != 0: self.get_logger().warning(f'JTC result {r.error_code}: {r.error_string}')

    def hold(self, pose, overrides=None):
        q = {j: (pose[j], 0.0) for j in JOINTS}
        q.update({j: (p, 0.0) for j, p in (overrides or {}).items()})
        return q

    def move(self, start, target, joint):
        """Slow single-segment move of one joint (others hold start)."""
        lo, hi, vmax = self.lim[joint]
        t = max(self.cfg['move_s'], abs(target - start[joint]) / (0.3 * vmax))
        self.send([(t, self.hold(start, {joint: target}))], self.now() + START_DELAY)

    def plan(self, joint, spec, start):
        """Build the SineTest in SI units and check limits / speed. Returns (sine, problem or None)."""
        lin = joint in LINEAR
        unit = 1.0 if lin else math.pi / 180
        lo, hi, vmax = self.lim[joint]
        # Same as Isaac Gain Tuner: center = middle of the URDF range + offset, amplitude % of half the range
        center = spec['center'] * unit if 'center' in spec else (lo + hi) / 2 + spec.get('offset', 0.0) * unit
        amp = spec['amplitude_pct'] / 100 * (hi - lo) / 2 if 'amplitude_pct' in spec else spec['amplitude'] * unit
        sine = SineTest(center, amp, spec['period'], spec['cycles'], self.cfg['ramp_cycles'])
        m = self.cfg['limit_margin']['m'] if lin else math.radians(self.cfg['limit_margin']['deg'])
        if center - amp < lo + m or center + amp > hi - m:
            return sine, f'swing [{center - amp:.4f}, {center + amp:.4f}] outside limits [{lo + m:.4f}, {hi - m:.4f}] (margin incl.)'
        peak_v = max(abs(sine(i * 0.001)[1]) for i in range(int(sine.total * 1000)))
        if peak_v > self.cfg['speed_margin'] * vmax:
            return sine, f'peak speed {peak_v:.3f} > {self.cfg["speed_margin"]} x limit {vmax}'
        return sine, None

    def run_joint(self, joint, spec, start):
        sine, problem = self.plan(joint, spec, start)
        if problem:
            self.get_logger().error(f'{joint}: skipped — {problem}')
            return None
        self.get_logger().info(f'{joint}: center {sine.c:.4f}, amplitude {sine.a:.4f}, period {spec["period"]} s, '
                               f'{spec["cycles"]} cycles (+fade), {sine.total:.1f} s')
        if abs(sine.c - start[joint]) > 1e-4: self.move(start, sine.c, joint)
        dt = 1.0 / self.cfg['sample_hz']
        n = int(math.ceil(sine.total / dt))
        pts = []
        for i in range(1, n + 1):
            t = min(i * dt, sine.total)
            p, v = sine(t)
            pts.append((t, self.hold(start, {joint: p}) | {joint: (p, v)}))
        settle = self.cfg['settle_s']
        pts.append((sine.total + settle, self.hold(start, {joint: sine.c})))
        t0 = self.now() + START_DELAY
        first = len(self.rows)
        self.recording = (joint, sine, t0)
        try:
            self.send(pts, t0)
        finally:
            self.recording = None
        # back to the start pose only after a normal finish — on Ctrl+C the arm stays where it stopped
        if abs(sine.c - start[joint]) > 1e-4: self.move(self.hold_pose(start, sine.c, joint), start[joint], joint)
        return self.metrics(joint, self.rows[first:], sine.total)

    @staticmethod
    def hold_pose(start, value, joint):
        return {**start, joint: value}

    def metrics(self, joint, rows, t_end):
        err = [float(r[4]) - float(r[2]) for r in rows if 0 <= float(r[0]) <= t_end]
        if not err: return None
        k = 1000.0 if joint in LINEAR else 180 / math.pi
        unit = 'mm' if joint in LINEAR else 'deg'
        peak, rms = max(abs(e) for e in err) * k, math.sqrt(sum(e * e for e in err) / len(err)) * k
        cur = [abs(float(r[6])) for r in rows if not math.isnan(float(r[6]))]
        return {'peak_error': round(peak, 4), 'rms_error': round(rms, 4), 'unit': unit, 'samples': len(err),
                'max_abs_current_A': round(max(cur), 4) if cur else None}

    def run(self):
        self.get_logger().info(f'config {self.cfg_path}, mock_hardware={self.mock}')
        self.spin_until(lambda: self.urdf is not None, 10.0, '/robot_description')
        self.lim = limits_from_urdf(self.urdf)
        self.spin_until(lambda: self.state is not None, 10.0, '/joint_states')
        self.spin_until(lambda: self.client.wait_for_server(timeout_sec=0.1), 10.0, ACTION)
        start = {j: self.state[j][0] for j in JOINTS}
        out = self.out_root / time.strftime('%Y%m%d_%H%M%S')
        out.mkdir(parents=True, exist_ok=True)
        meta = {'config': self.cfg, 'config_path': self.cfg_path, 'mock_hardware': self.mock,
                'start_pose': start, 'limits': {j: list(v) for j, v in self.lim.items()},
                'columns': 't [s since sine start], joint, cmd_pos, cmd_vel, pos, vel (rad, rad/s | m, m/s), effort [A]'}
        yaml.safe_dump(meta, open(out / 'meta.yaml', 'w'), sort_keys=False)
        summary = {}
        try:
            for joint in JOINTS:
                spec = self.cfg['joints'].get(joint, {})
                if spec.get('test'):
                    summary[joint] = self.run_joint(joint, spec, start)
        except KeyboardInterrupt:
            signal.signal(signal.SIGINT, signal.SIG_IGN)  # terminal Ctrl+C and ros2 launch both send SIGINT — finish cleanup
            self.get_logger().warning('interrupted — cancelling goal')
            if self.goal_handle is not None:
                fut = self.goal_handle.cancel_goal_async()
                end = time.monotonic() + 2.0
                while not fut.done() and time.monotonic() < end: rclpy.spin_once(self, timeout_sec=0.05)
        finally:
            with open(out / 'log.csv', 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t', 'joint', 'cmd_pos', 'cmd_vel', 'pos', 'vel', 'effort_A'])
                w.writerows(self.rows)
            yaml.safe_dump(summary, open(out / 'summary.yaml', 'w'), sort_keys=False)
            for j, m in summary.items():
                if m: self.get_logger().info(f'{j:20s} peak {m["peak_error"]:8.3f} {m["unit"]}  rms {m["rms_error"]:8.3f} {m["unit"]}'
                                             f'  max |I| {m["max_abs_current_A"]} A')
            self.get_logger().info(f'saved {out}')


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl+C -> KeyboardInterrupt, context stays up to cancel
    node = Runner()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
