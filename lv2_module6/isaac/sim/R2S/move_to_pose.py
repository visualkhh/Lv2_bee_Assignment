# Isaac Sim Script Editor: slowly move the beebot arm to a joint pose by ramping the USD drive targets.
# Press Play first (targets only advance on physics steps). Run again to start a new move; the old one is replaced.
# Pose = real robot balance point from /joint_states (2026-10-28). Arm joints in rad, gripper in m.
import builtins
import math

import omni.physx
import omni.usd
from pxr import Usd, UsdPhysics

TARGET = {
    "joint1": -0.00307,
    "joint2": -0.40957,
    "joint3": 0.22396,
    "joint4": 0.0721,
    "joint5": -0.0046,
    "gripper_left_joint": 0.009,
}
DURATION_S = 5.0  # sim time for the whole move (cosine profile, zero speed at both ends)

stage = omni.usd.get_context().get_stage()
drives = {}  # joint name -> (target attribute, start value, goal value) in USD units (deg or m)
for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
    name = prim.GetName()
    if name not in TARGET or name in drives:
        continue
    if prim.IsA(UsdPhysics.RevoluteJoint):
        kind, goal = "angular", math.degrees(TARGET[name])  # USD angular drive target is in degrees
    elif prim.IsA(UsdPhysics.PrismaticJoint):
        kind, goal = "linear", TARGET[name]
    else:
        continue
    attr = UsdPhysics.DriveAPI.Get(prim, kind).GetTargetPositionAttr()
    if not attr:
        print(f"[move_to_pose] {prim.GetPath()} has no {kind} drive, skipped")
        continue
    drives[name] = (attr, attr.Get() or 0.0, goal)
    print(f"[move_to_pose] {prim.GetPath()}: {drives[name][1]:.3f} -> {goal:.3f}")

missing = set(TARGET) - set(drives)
if missing:
    print(f"[move_to_pose] not found: {sorted(missing)}")

old = getattr(builtins, "_beebot_move_sub", None)
if old is not None:
    old.unsubscribe()
elapsed = [0.0]


def on_step(dt):
    elapsed[0] += dt
    s = min(elapsed[0] / DURATION_S, 1.0)
    w = 0.5 - 0.5 * math.cos(math.pi * s)
    for attr, start, goal in drives.values():
        attr.Set(start + (goal - start) * w)
    if s >= 1.0:
        builtins._beebot_move_sub.unsubscribe()
        builtins._beebot_move_sub = None
        print("[move_to_pose] done")


builtins._beebot_move_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(on_step)
