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

Not emulated: T (torque on/off) and R (motor reboot) lines are ignored; temperatures are a constant.
Comments/strings stay English: the Script Editor breaks Korean text.
"""
import builtins
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
# servo value = joint value + offset (URDF <param name="offset">, all 0.0 in urdf/beebot_mesh)
OFFSET = {name: 0.0 for name in ARM}
# ponytail: effort -> current with one constant (XM430-W350 stall ~4.1 N*m @ 2.3 A, 12 V); gripper current is not modeled.
# arm_safety_monitor reads these amps; replace with a measured current/torque curve if contact detection misbehaves in sim.
NM_PER_A = 1.78
TEMP_C = 35  # same as faker/fake_opencr (no thermal model)
COMMAND_TIMEOUT = 0.5  # [s] firmware COMMAND_TIMEOUT_MS
ERROR_PERIOD = 0.5  # [s] firmware MOTOR_CHECK_MS


class Board:
    def __init__(self):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)  # raw bytes, no echo: like a USB CDC port
        os.set_blocking(self.master, False)
        if os.path.lexists(LINK):
            os.remove(LINK)
        os.symlink(os.ttyname(self.slave), LINK)
        self.rx = b""
        self.view = None
        self.t = 0.0
        self.last_cmd = -1e9
        self.last_err = -1e9
        self.sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self.step)
        print(f"[opencr_bridge] virtual OpenCR on {LINK} -> {os.ttyname(self.slave)}")

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
        self.sim = omni.physics.tensors.create_simulation_view("numpy")
        self.view = self.sim.create_articulation_view(str(root.GetPath()))
        names = self.view.shared_metatype.dof_names
        self.idx = {n: names.index(n) for n in WHEELS + ARM + FINGERS}
        self.imu = IMUSensor(str(imu.GetPath()))
        self.wheel_prev = None  # for unwrapping wheel angles (protocol: cumulative rad since boot)
        self.wheel_acc = np.zeros(2)
        print(f"[opencr_bridge] attached {root.GetPath()} ({len(names)} dofs), IMU {imu.GetPath()}")

    def step(self, dt):
        if self.view is None or not self.sim.is_valid:
            self.attach()
        self.t += dt
        self.read_commands()
        if self.t - self.last_cmd > COMMAND_TIMEOUT:  # watchdog: wheels stop, arm keeps its target
            self.set_targets(wheels=[0.0, 0.0])
        self.send(self.state_line())
        if self.t - self.last_err >= ERROR_PERIOD:
            self.send("E" + ",0" * 8)
            self.last_err = self.t

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
                    p[0, self.idx[dof]] = q
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
        arm = {name: num(s) - OFFSET[name] for name, s in zip(ARM, f[2:8]) if num(s) is not None}  # empty = hold
        self.set_targets(wheels=wheels, arm=arm)
        self.last_cmd = self.t

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
            v += [self.wheel_acc[k], vel[self.idx[name]]]
        for name in ARM:
            i = self.idx[name]
            amps = 0.0 if name == "gripper_left_joint" else eff[i] / NM_PER_A
            v += [pos[i] + OFFSET[name], vel[i], amps]
        r = self.imu.get_sensor_reading(read_gravity=True)
        v += [r.orientation_x, r.orientation_y, r.orientation_z, r.orientation_w,
              r.angular_velocity_x, r.angular_velocity_y, r.angular_velocity_z,
              r.linear_acceleration_x, r.linear_acceleration_y, r.linear_acceleration_z]
        return "S," + ",".join(f"{float(x):.4f}" for x in v) + "," + ",".join([str(TEMP_C)] * 8)

    def send(self, line):
        try:
            os.write(self.master, (line + "\n").encode())
        except (BlockingIOError, OSError):
            pass  # nobody reading and the pty buffer is full: drop, like an unplugged USB


old = getattr(builtins, "_beebot_opencr", None)
if old is not None:
    old.close()
builtins._beebot_opencr = Board()
