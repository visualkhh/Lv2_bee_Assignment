"""Wheel drive test helpers for the Isaac Sim Script Editor.

Open beebot.usda (Physics variant physx), press Play, run this file once, then e.g.:

    drive(-3, 3); measure()     # spin in place, expect ~37.4 deg/s
    drive(6.06, 6.06); measure()
    drive(0, 0)

Tip-over tests (each prints max tilt / lift / slide of base_link):

    tip_reach()   # stow -> REACH in one step
    tip_swing()   # arm at REACH, joint1 -90 -> +90 in one step
    tip_stop()    # 0.2 m/s straight, then drive(0, 0)

Wheel/arm commands live in the session layer and survive Stop/Play; tests reset wheels first.
Run one test at a time and wait for its result line. Do not re-run the exec() line
meanwhile: it resets _sub and cancels pending measurements.

Comments/strings stay English: the Script Editor breaks Korean text.
"""
import asyncio
import math

import omni.kit.app
import omni.physx
import omni.usd
from pxr import Usd

physx = omni.physx.get_physx_interface()
JOINTS = "/beebot/Physics/"
BASE = "/beebot/Geometry/base_footprint/base_link"
_sub = {}
STOW = (0, 0, 0, 0, 0)
# horizontal reach, derived from URDF axes (zero pose = upper arm up, forearm level); check visually
REACH = (0, 90, -90, 0, 0)


def drive(left, right):
    """Wheel target velocity in rad/s (USD drive attribute is deg/s)."""
    stage = omni.usd.get_context().get_stage()  # fetch each call: a reopened stage invalidates old handles
    # session layer is never saved, so test commands cannot leak into beebot.usda via Ctrl+S
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for joint, v in (("wheel_left_joint", left), ("wheel_right_joint", right)):
            stage.GetPrimAtPath(JOINTS + joint).GetAttribute("drive:angular:physics:targetVelocity").Set(math.degrees(v))


def arm(*deg):
    """Arm joint1..5 target positions in deg (USD drive targetPosition is deg)."""
    stage = omni.usd.get_context().get_stage()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for i, v in enumerate(deg, 1):
            stage.GetPrimAtPath(f"{JOINTS}joint{i}").GetAttribute("drive:angular:physics:targetPosition").Set(float(v))


def _pose():
    t = physx.get_rigidbody_transformation(BASE)
    x, y, z, w = t["rotation"]
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return t["position"], yaw


def measure(seconds=3.0, settle=1.0):
    """Print mean yaw rate and speed over `seconds`, after `settle` s of spin-up."""
    s = {"t": 0.0, "prev": None, "yaw": 0.0, "p0": None}

    def on_step(dt):
        pos, yaw = _pose()
        if s["prev"] is not None and s["t"] >= settle:
            s["yaw"] += (yaw - s["prev"] + 180) % 360 - 180
            if s["p0"] is None:
                s["p0"] = pos
        s["prev"] = yaw
        s["t"] += dt
        if s["t"] >= settle + seconds:
            dist = math.dist(pos[:2], s["p0"][:2])
            print(f"yaw rate {s['yaw'] / seconds:.1f} deg/s, speed {dist / seconds:.3f} m/s")
            _sub.pop("m", None)

    _sub["m"] = physx.subscribe_physics_step_events(on_step)


def grip(m):
    """Gripper opening target in m (0 closed .. 0.0265 open); right finger should follow via NewtonMimicAPI."""
    stage = omni.usd.get_context().get_stage()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        stage.GetPrimAtPath(JOINTS + "gripper_left_joint").GetAttribute("drive:linear:physics:targetPosition").Set(float(m))


def fingers():
    """Print world distance of each finger from gripper_link (they should match)."""
    g = BASE + "/arm_base_link/shoulder_link/upper_arm_link/forearm_link/wrist_link/gripper_link"
    pg = physx.get_rigidbody_transformation(g)["position"]
    for side in ("left", "right"):
        p = physx.get_rigidbody_transformation(f"{g}/finger_{side}_link")["position"]
        print(f"finger_{side}: {math.dist(p, pg) * 1000:.1f} mm from gripper_link")


def _up(q):
    """Body z axis in world frame from quat (x, y, z, w)."""
    x, y, z, w = q
    return (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))


def run(steps, seconds, watch_from=0.0, label="run"):
    """Call each fn in steps [(t, fn), ...] at sim time t (s); from watch_from until seconds,
    track base_link tilt (vs its pose at watch_from), lift and horizontal slide, then print maxima."""
    s = {"t": 0.0, "ref": None, "tilt": 0.0, "end": 0.0, "lift": 0.0, "slide": 0.0}

    def on_step(dt):
        s["t"] += dt
        if s["t"] < watch_from:
            return
        tr = physx.get_rigidbody_transformation(BASE)
        pos, up = tr["position"], _up(tr["rotation"])
        if s["ref"] is None:
            s["ref"] = (pos, up)
        p0, u0 = s["ref"]
        dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(up, u0))))
        s["end"] = math.degrees(math.acos(dot))
        s["tilt"] = max(s["tilt"], s["end"])
        s["lift"] = max(s["lift"], pos[2] - p0[2])
        s["slide"] = max(s["slide"], math.dist(pos[:2], p0[:2]))

    async def sequence():
        # USD writes happen on app updates, not inside the physics step callback
        app = omni.kit.app.get_app()
        for t, fn in sorted(steps, key=lambda x: x[0]):
            while s["t"] < t:
                await app.next_update_async()
            fn()
        while s["t"] < seconds:
            await app.next_update_async()
        _sub.pop(label, None)
        print(f"{label}: max tilt {s['tilt']:.1f} deg (end {s['end']:.1f}), lift {s['lift'] * 1000:.1f} mm, slide {s['slide'] * 100:.1f} cm")

    _sub[label] = physx.subscribe_physics_step_events(on_step)
    asyncio.ensure_future(sequence())


def tip_reach(pose=REACH):
    run([(0, lambda: drive(0, 0)), (0, lambda: arm(*STOW)), (2, lambda: arm(*pose))], 5, watch_from=2, label="reach")


def tip_swing(pose=REACH, j1_from=-90, j1_to=90):
    a, b = list(pose), list(pose)
    a[0], b[0] = j1_from, j1_to
    run([(0, lambda: drive(0, 0)), (0, lambda: arm(*a)), (3, lambda: arm(*b))], 6, watch_from=3, label="swing")


def tip_stop(v=6.06):
    run([(0, lambda: arm(*STOW)), (0, lambda: drive(v, v)), (3, lambda: drive(0, 0))], 5, watch_from=3, label="stop")
