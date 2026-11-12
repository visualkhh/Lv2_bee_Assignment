"""Isaac Sim startup for the team: open usda/scene_sim.usda with the physx variant and start
opencr_bridge.py (virtual OpenCR on /tmp/opencr) so the ROS side can connect right away.
Viewport starts with colliders hidden.
While playing, puck ground truth goes to /tmp/beebot_sim_pucks.json (loops/pick_loop.py use_sim=true logs it).
Same result on every machine, no personal layout file.

    isaacsim --exec isaac_startup.py        (isaac_sim.sh does this)
"""
import asyncio
import json
import math
import os
import re
import time

import carb.settings
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, PhysxSchema, Usd

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.environ.get("ISAAC_SCENE") or os.path.join(HERE, "..", "..", "usda", "scene_sim.usda")  # isaac_sim.sh <scene>
RESET_FILE = "/tmp/beebot_isaac_reset"  # isaac_sim.sh와 같은 경로
TRUTH_FILE = "/tmp/beebot_sim_pucks.json"
TRUTH_EVERY = 15  # frames (~4 Hz at 60 fps)
BASE = "/World/beebot/Geometry/base_footprint/base_link"  # 1 cm above base_footprint, same x y (URDF base_joint)
PUCK = re.compile(r"puck_(box|cyl)_(red|green)_\d+$")
PUCK_H = 0.06
ARM = "/World/beebot/Geometry/base_footprint/base_link/arm_base_link/shoulder_link/upper_arm_link/forearm_link/wrist_link/gripper_link"
CAMS = {"gripper_cam": ARM + "/gripper_cam_link/gripper_cam_color_optical_frame"}


def write_truth(stage, pucks):
    """Puck centres in base_link (x forward, y left) [m], yaw of the long/up axis [deg], tilt from upright [deg].
    The puck prim origin is its bottom-face centre, so the centre is origin + up * PUCK_H/2 (matters when lying)."""
    inv = omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(BASE)).GetInverse()
    out = {}
    for path in pucks:
        m = omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(path)) * inv  # puck -> base_link
        up = m.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()
        c = m.Transform(Gf.Vec3d(0, 0, PUCK_H / 2))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))
        out[path.rsplit("/", 1)[1]] = {"x": c[0], "y": c[1], "z": c[2], "tilt": tilt,
                                       "yaw": math.degrees(math.atan2(up[1], up[0])) if tilt > 45 else None}
    cams = {}  # optical frames (x right, y down, z forward) in base_link — compare with TF base_footprint -> *_color_optical_frame
    for name, path in CAMS.items():
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            m = omni.usd.get_world_transform_matrix(prim) * inv
            cams[name] = {"t": list(m.ExtractTranslation()), "R": [list(m.TransformDir(Gf.Vec3d(*a))) for a in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]}
    with open(TRUTH_FILE + ".tmp", "w") as f:
        json.dump({"t": time.time(), "pucks": out, "cams": cams}, f)
    os.replace(TRUTH_FILE + ".tmp", TRUTH_FILE)


async def _setup():
    app = omni.kit.app.get_app()
    for _ in range(60):  # let the default windows come up first
        await app.next_update_async()
    await omni.usd.get_context().open_stage_async(os.path.abspath(SCENE))
    # Viewport: no collider debug draw, no guide-purpose prims (the robot's collision meshes are guides).
    # Set after opening the stage so per-user persistent settings don't win.
    # Render mode stays at the default RTX - Real-Time: /rtx/rendermode is global, so it also drives the ROS camera images.
    settings = carb.settings.get_settings()
    settings.set_int("/persistent/physics/visualizationDisplayColliders", 0)
    settings.set_bool("/persistent/app/hydra/displayPurpose/guide", False)
    stage = omni.usd.get_context().get_stage()
    # scene_sim.usda selects no Physics variant ("none" = no articulation, opencr_bridge finds nothing).
    # Pick physx in the session layer: never saved into the file.
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        stage.GetPrimAtPath("/World/beebot").GetVariantSet("Physics").SetVariantSelection("physx")
        # CPU PhysX: one robot + a few pucks is too small for GPU dynamics, whose per-step GPU<->CPU sync
        # (opencr_bridge reads joints via the numpy tensor API every step) cost more than the solve.
        # Measured 2026-10-30: 26 -> 39 fps, sim speed 0.43x -> 0.65x real time.
        scene = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/PhysicsScene"))
        scene.CreateEnableGPUDynamicsAttr().Set(False)
        scene.CreateBroadphaseTypeAttr().Set("MBP")
        # Physics rate = Isaac Lab training (SimulationCfg dt 1/200): the scene default is 60 Hz.
        # Also lets opencr_bridge send S lines every 10 ms like the firmware. BEE_PHYSICS_HZ=60 if the sim gets too slow.
        scene.CreateTimeStepsPerSecondAttr().Set(int(os.environ.get("BEE_PHYSICS_HZ", "200")))
    bridge = os.path.join(HERE, "opencr_bridge.py")  # creates the pty now; it attaches to the robot on the first physics step
    exec(compile(open(bridge).read(), bridge, "exec"), {"__name__": "__main__", "__file__": bridge})
    timeline = omni.timeline.get_timeline_interface()
    if os.environ.get("ISAAC_AUTOPLAY") == "1":  # 시연: Play를 자동으로
        timeline.play()
    pucks = [str(p.GetPath()) for p in stage.Traverse() if PUCK.match(p.GetName())]
    frame = 0
    while True:  # reset.sh: Stop(장면 처음 자세로) → Play — ROS 재시작은 isaac_sim.sh
        await app.next_update_async()
        frame += 1
        if pucks and frame % TRUTH_EVERY == 0 and timeline.is_playing():
            try:
                write_truth(stage, pucks)
            except Exception as e:  # noqa: BLE001 - never stop the reset loop over the truth file
                if frame % (TRUTH_EVERY * 240) == 0:
                    print(f"[isaac_startup] puck truth not written: {e!r}", flush=True)
        if os.path.exists(RESET_FILE):
            timeline.stop()
            for _ in range(30):
                await app.next_update_async()
            timeline.play()
            os.remove(RESET_FILE)


asyncio.ensure_future(_setup())
