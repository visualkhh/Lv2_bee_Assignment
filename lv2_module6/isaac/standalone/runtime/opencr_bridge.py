"""Virtual OpenCR board for the Isaac Sim Script Editor.

Isaac plays the OpenCR: it speaks the same ASCII serial protocol as firmware/beebot_opencr
(header of beebot_opencr.ino) over a pseudo-terminal, so the real ROS side
(control/BeebotSystemHardware + controllers + EKF + Nav2 + MoveIt) runs unchanged.

    SBC -> board  C,<wheelL rad/s>,<wheelR rad/s>,<j1..j5 rad>,<gripper m>   (empty field = hold; wheels -> 0)
    board -> SBC  S,<40 values> every physics step (firmware: 10 ms)
                  E,0,0,0,0,0,0,0,0 every 0.5 s (no motor errors in sim)

Usage:
  1. Open scene_sim.usda (or beebot.usda) with the beebot Physics variant "physx", press Play.
  2. Run this file once in the Script Editor. It prints the port link (default /tmp/opencr).
  3. ros2 launch ros_system ros_system.launch.py port:=/tmp/opencr
     (or bringup with the same port:= argument)
  Re-running this file closes the old port first; restart ros2_control afterwards.
  Remove the ActionGraph IMU publisher (isaac_read_imu_node, ros2_publish_imu): /imu now comes from
  imu_sensor_broadcaster like on the real robot, two publishers would fight.

Joint twist: OPENCR_TWIST_SCALE=1 (isaac_sim.sh default) makes joint2/3 twist under gravity where the encoders cannot see it,
as measured on the real arm (0 = off). The hand then sits below the TF until rl_arm_correction gravity_enabled cancels it.

Real-board behaviour copied from firmware/beebot_opencr and the Isaac Lab training env (BEE_SIM_REAL=0 turns all of it off):
  - servo zero offsets from the URDF (above), arm goals rounded to servo ticks with the firmware 0.8 tick hysteresis,
    arm positions reported in whole encoder ticks (4096 / rev)
  - command latency per joint = training LATENCY_S (isaac/rl/bee_isaac/common.py, measured, report 4)
  - gripper force limit from the C 9th field (Goal Current, default 600 mA), 72.5 N/A per finger like training
    (GRIP_FORCE 29 N at 0.4 A); gripper current reported from the finger force
  - S lines every 10 ms (STATE_PERIOD_MS), not every physics step
  - with twist on: joint2/3 drive stiffness = measured servo stiffness (SERVO_K), like training twist=1

Not emulated: T (torque on/off) and R (motor reboot) lines are ignored; temperatures are a constant.
Comments/strings stay English: the Script Editor breaks Korean text.
"""
import builtins
import collections
import math
import os
import tty

import numpy as np
import omni.physics.tensors
import omni.physx
import omni.usd
from isaacsim.sensors.experimental.physics import IMUSensor
from pxr import UsdPhysics

LINK = "/tmp/opencr"
WHEELS = ["wheel_left_joint", "wheel_right_joint"]
ARM = ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper_left_joint"]  # protocol order
FINGERS = ["gripper_left_joint", "gripper_right_joint"]  # right finger mimics left (URDF mimic, multiplier 1)
# servo value = joint value + offset (URDF <param name="offset">, real servo zero error).
# The sim has no zero error, so the board undoes the same offsets: USD angle = servo value - offset = joint value.
# Read from the URDF the ROS side loads (BEE_URDF = folder under lv2_module6/urdf, ros_system urdf:= default).
def _urdf_offsets():
    import re
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "urdf",
                        os.environ.get("BEE_URDF", "default"), "urdf.xacro")
    try:
        text = open(path).read()
    except OSError:
        print(f"[opencr_bridge] {path} not found, offsets 0")
        return {}
    ctrl = text[text.find("<ros2_control"):]
    out = {}
    for name, body in re.findall(r'<joint name="(\w+)">(.*?)</joint>', ctrl, re.S):
        m = re.search(r'<param name="offset">([-0-9.eE]+)</param>', body)
        if m:
            out[name] = float(m.group(1))
    return out


REAL = os.environ.get("BEE_SIM_REAL", "1") != "0"
OFFSET = {name: 0.0 for name in ARM}
if REAL:
    OFFSET.update({k: v for k, v in _urdf_offsets().items() if k in OFFSET})
print("[opencr_bridge] servo zero offsets (rad):", OFFSET)
RAW_PER_RAD = 4096 / (2 * math.pi)  # firmware RAW_PER_RAD
GOAL_HYSTERESIS_RAW = 0.8  # firmware config.h
LATENCY = dict(zip(ARM, (0.045, 0.062, 0.052, 0.045, 0.045, 0.020) if REAL else (0.0,) * 6))  # [s] = common.py LATENCY_S, _GRIP_S
GRIP_N_PER_A = 29.0 / 0.4  # per finger, = common.py GRIP_FORCE at the team current 0.4 A
GRIP_A_DEFAULT, GRIP_A_MIN, GRIP_A_MAX = 0.6, 0.00269, 1.2  # firmware GRIPPER_GOAL_CURRENT_MA, _CURRENT_MIN_MA, _MAX_MA
STATE_PERIOD = 0.010 if REAL else 0.0  # [s] firmware STATE_PERIOD_MS


def ticks(servo_rad):
    """Servo value rounded to whole encoder ticks (firmware raw_to_rad(rad_to_raw(x)))."""
    return round(servo_rad * RAW_PER_RAD) / RAW_PER_RAD if REAL else servo_rad
# ponytail: effort -> current with one constant (XM430-W350 stall ~4.1 N*m @ 2.3 A, 12 V); gripper current: GRIP_N_PER_A below.
# arm_safety_monitor reads these amps; replace with a measured current/torque curve if contact detection misbehaves in sim.
NM_PER_A = 1.78
TEMP_C = 35  # same as faker/fake_opencr (no thermal model)
BATTERY_V = 12.0  # same as faker/fake_opencr (no discharge model)
COMMAND_TIMEOUT = 0.5  # [s] firmware COMMAND_TIMEOUT_MS
ERROR_PERIOD = 0.5  # [s] firmware MOTOR_CHECK_MS
TWIST_SCALE = float(os.environ.get("OPENCR_TWIST_SCALE", "0"))  # 0 = off, 1 = as measured
TWIST_K = {"joint2": 12.2, "joint3": 18.8}  # [Nm/rad] stiffness fit of the 11-04 sag measurement (scripts removed, see git history)
# Servo position loop as a joint drive (Dynamixel P 800 / I 0 / D 0 do not convert to Nm/rad, so the stiffness is measured):
# with twist on, the drive is only the servo error the encoder sees, 1 / servo_c = training place_env servo_c (0.022, 0.033)
# rad/Nm, measured 11-08 (sag integral / tau). The USD 0.42 Nm/deg (24 Nm/rad) stays for joint1/4/5 and for twist off.
SERVO_K = {"joint2": 1 / 0.022, "joint3": 1 / 0.033}  # [Nm/rad]


def gravity_torques(q2, q3, q4):
    """joint2, joint3 gravity torque [Nm]: same point masses as gravity::torques in ml/rl_arm_correction.hpp (URDF values)."""
    def x_of(th, x, z):  # arm plane vector rotated by pitch th, x component
        return x * math.cos(th) + z * math.sin(th)
    t23, t234 = q2 + q3, q2 + q3 + q4
    p3 = x_of(q2, 0.024, 0.128)
    fa = p3 + x_of(t23, 0.093796, 0.0)
    p4 = p3 + x_of(t23, 0.124, 0.0)
    wr = p4 + x_of(t234, 0.043623, 0.011449)
    hd = p4 + x_of(t234, 0.0629 + 0.052686, 0.014472)
    g, m_ua, m_fa, m_wr, m_hd = 9.81, 0.128865, 0.117300, 0.090152, 0.371242
    return (g * (m_ua * x_of(q2, 0.008487, 0.104997) + m_fa * fa + m_wr * wr + m_hd * hd),
            g * (m_fa * (fa - p3) + m_wr * (wr - p3) + m_hd * (hd - p3)))


class Board:
    def __init__(self):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)  # raw bytes, no echo: like a USB CDC port
        os.set_blocking(self.master, False)
        if os.path.lexists(LINK):
            os.remove(LINK)
        self.rx = b""
        self.view = None
        self.t = 0.0
        self.last_cmd = -1e9
        self.last_err = -1e9
        self.twist = {}  # [rad] true joint angle = encoder + twist
        self.goal_raw = {}  # [tick] last arm goal written to the servo (hysteresis like the firmware)
        self.pending = {name: collections.deque() for name in ARM}  # (time, joint target) waiting for LATENCY
        self.grip_a = GRIP_A_DEFAULT
        self.grip_a_set = None  # applied to the finger drives (re-applied after a Stop / Play)
        self.last_state = -1e9
        self.sub =omni.physx.get_physx_interface().subscribe_physics_step_events(self.step)
        print(f"[opencr_bridge] virtual OpenCR ready, {LINK} appears on Play")

    def close(self):
        self.sub = None
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass
        if os.path.islink(LINK):
            os.remove(LINK)
        print("[opencr_bridge] closed")

    # ---- physics side
    def attach(self):
        """(Re)create the tensor view after Play; a Stop invalidates it."""
        stage = omni.usd.get_context().get_stage()
        root = next(p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI) and "beebot" in str(p.GetPath()))
        imu = next(p for p in stage.Traverse() if p.GetName() == "Imu_Sensor")
        # stage id must be explicit: the default -1 fails in 6.1 ("Failed to get a valid attached USD stage id")
        self.sim = omni.physics.tensors.create_simulation_view("numpy", omni.usd.get_context().get_stage_id())
        self.view = self.sim.create_articulation_view(str(root.GetPath()))
        names = self.view.shared_metatype.dof_names
        self.idx = {n: names.index(n) for n in WHEELS + ARM + FINGERS}
        self.imu = IMUSensor(str(imu.GetPath()))
        self.wheel_prev = None  # for unwrapping wheel angles (protocol: cumulative rad since boot)
        self.wheel_acc = np.zeros(2)
        self.grip_a_set = None
        # Link only once S lines can flow: BeebotSystemHardware.on_activate needs S within 3 s,
        # so isaac_sim.sh waits for this link before starting ros2_control.
        if not os.path.lexists(LINK):
            os.symlink(os.ttyname(self.slave), LINK)
            print(f"[opencr_bridge] virtual OpenCR on {LINK} -> {os.ttyname(self.slave)}")
        print(f"[opencr_bridge] attached {root.GetPath()} ({len(names)} dofs), IMU {imu.GetPath()}")
        if REAL and TWIST_SCALE:
            k = self.view.get_dof_stiffnesses()
            for name, v in SERVO_K.items():
                k[0, self.idx[name]] = v
            self.view.set_dof_stiffnesses(k, np.array([0], dtype=np.int32))
            print("[opencr_bridge] arm drive stiffness (Nm/rad):",
                  {n: round(float(k[0, self.idx[n]]), 1) for n in ARM[:5]})

    def step(self, dt):
        if self.view is None or not self.sim.is_valid:
            self.attach()
        self.t += dt
        self.update_twist()
        self.read_commands()
        self.apply_due()
        if self.t - self.last_cmd > COMMAND_TIMEOUT:  # watchdog: wheels stop, arm keeps its target
            self.set_targets(wheels=[0.0, 0.0])
        if self.t - self.last_state >= STATE_PERIOD - 1e-6:
            self.last_state = self.t
            self.send(self.state_line())
        if self.t - self.last_err >= ERROR_PERIOD:
            self.send("E" + ",0" * 8)
            self.last_err = self.t

    def update_twist(self):
        """shortcut: quasi-static, no spring motion, and a held puck or contact force does not change it; for those, spring joints in the USD."""
        if not TWIST_SCALE:
            return
        pos = self.view.get_dof_positions()[0]
        tau = gravity_torques(*(float(pos[self.idx[n]]) for n in ("joint2", "joint3", "joint4")))
        self.twist = {n: TWIST_SCALE * t / TWIST_K[n] for n, t in zip(("joint2", "joint3"), tau)}

    def set_targets(self, wheels=None, arm=None):
        all_ = np.array([0], dtype=np.int32)
        if wheels is not None:
            v = self.view.get_dof_velocity_targets()
            for name, w in zip(WHEELS, wheels):
                v[0, self.idx[name]] = w
            self.view.set_dof_velocity_targets(v, all_)
        if arm:
            p = self.view.get_dof_position_targets()
            for name, q in arm.items():
                for dof in (FINGERS if name == "gripper_left_joint" else [name]):
                    p[0, self.idx[dof]] = q + self.twist.get(dof, 0.0)
            self.view.set_dof_position_targets(p, all_)

    # ---- serial side
    def read_commands(self):
        try:
            self.rx += os.read(self.master, 4096)
        except (BlockingIOError, OSError):
            pass
        *lines, self.rx = self.rx.split(b"\n")
        cmds = [ln for ln in lines if ln.startswith(b"C")]
        if not cmds:
            return
        f = cmds[-1].decode(errors="replace").strip().split(",")[1:]  # firmware also applies only the last C
        f += [""] * (8 - len(f))
        num = lambda s: float(s) if s.strip() else None
        wheels = [num(s) or 0.0 for s in f[:2]]  # empty wheel field = stop
        for name, s in zip(ARM, f[2:8]):
            servo = num(s)
            if servo is None:  # empty = hold
                continue
            if name != "gripper_left_joint" and REAL:  # firmware: goal in ticks, rewritten only past the hysteresis
                raw = servo * RAW_PER_RAD
                if name in self.goal_raw and abs(raw - self.goal_raw[name]) < GOAL_HYSTERESIS_RAW:
                    continue
                self.goal_raw[name] = round(raw)
                servo = self.goal_raw[name] / RAW_PER_RAD
            self.pending[name].append((self.t, servo - OFFSET[name]))
        if len(f) > 8 and num(f[8]) is not None:  # Goal Current [A] (firmware set_gripper_current)
            self.grip_a = min(max(num(f[8]), GRIP_A_MIN), GRIP_A_MAX)
        self.set_targets(wheels=wheels)
        self.last_cmd = self.t

    def apply_due(self):
        """Joint targets reach the joints after LATENCY (servo bus + servo response, measured on the robot)."""
        due = {}
        for name, q in self.pending.items():
            while q and q[0][0] <= self.t - LATENCY[name] + 1e-9:
                due[name] = q.popleft()[1]
        if due:
            self.set_targets(arm=due)
        if REAL and self.grip_a != self.grip_a_set:
            try:
                m = self.view.get_dof_max_forces()
                for dof in FINGERS:
                    m[0, self.idx[dof]] = self.grip_a * GRIP_N_PER_A
                self.view.set_dof_max_forces(m, np.array([0], dtype=np.int32))
                self.grip_a_set = self.grip_a
            except Exception as e:  # tensor API without max forces: keep the USD maxForce
                print(f"[opencr_bridge] gripper force limit not set: {e}")
                self.grip_a_set = self.grip_a

    def state_line(self):
        pos = self.view.get_dof_positions()[0]
        vel = self.view.get_dof_velocities()[0]
        try:
            eff = self.view.get_dof_projected_joint_forces()[0]
        except Exception:  # not in every tensor API version
            eff = np.zeros_like(pos)
        # wheels: unwrap so the angle keeps counting like the servo's multi-turn position
        w = np.array([pos[self.idx[n]] for n in WHEELS])
        if self.wheel_prev is not None:
            self.wheel_acc += (w - self.wheel_prev + math.pi) % (2 * math.pi) - math.pi
        else:
            self.wheel_acc = w.copy()
        self.wheel_prev = w
        v = []
        for k, name in enumerate(WHEELS):
            v += [self.wheel_acc[k], vel[self.idx[name]], 0.0]  # wheel current A: no load model
        for name in ARM:
            i = self.idx[name]
            if name == "gripper_left_joint":  # closing force -> + current (servo current while squeezing)
                v += [pos[i], vel[i], -eff[i] / GRIP_N_PER_A if REAL else 0.0]
                continue
            v += [ticks(pos[i] + OFFSET[name] - self.twist.get(name, 0.0)), vel[i], eff[i] / NM_PER_A]
        r = self.imu.get_sensor_reading(read_gravity=True)
        v += [r.orientation_x, r.orientation_y, r.orientation_z, r.orientation_w,
              r.angular_velocity_x, r.angular_velocity_y, r.angular_velocity_z,
              r.linear_acceleration_x, r.linear_acceleration_y, r.linear_acceleration_z]
        return "S," + ",".join(f"{float(x):.4f}" for x in v) + "," + ",".join([str(TEMP_C)] * 8) + f",{BATTERY_V:.2f}"

    def send(self, line):
        try:
            os.write(self.master, (line + "\n").encode())
        except (BlockingIOError, OSError):
            pass  # nobody reading and the pty buffer is full: drop, like an unplugged USB


def camera_info_off(stage):
    """Isaac camera_info publishers off — every Isaac start path execs this file (isaac_startup.py).
    On ROS Lyrical (experimental in Isaac 6.1) ROS2CameraInfoHelper leaks ~1 GB/min of RAM: OOM after ~25 min (11-06,
    each graph node type switched off in turn — only this one leaks; with Isaac's bundled Jazzy it does not).
    Nothing is lost: camera_adapter.py publishes camera_info itself (Isaac's arrives corrupted on Lyrical).
    Session layer only — the scene files are unchanged."""
    from pxr import Usd
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for prim in stage.Traverse():
            t = prim.GetAttribute("node:type")
            if t and t.Get() == "isaacsim.ros2.bridge.ROS2CameraInfoHelper":
                prim.GetAttribute("inputs:enabled").Set(False)


camera_info_off(omni.usd.get_context().get_stage())
old = getattr(builtins, "_beebot_opencr", None)
if old is not None:
    old.close()
builtins._beebot_opencr = Board()
