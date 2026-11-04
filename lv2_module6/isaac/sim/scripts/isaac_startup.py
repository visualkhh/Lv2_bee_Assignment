"""Isaac Sim startup for the team: open usda/scene_sim.usda with the physx variant and start
opencr_bridge.py (virtual OpenCR on /tmp/opencr) so the ROS side can connect right away.
Viewport starts in RTX - Minimal with colliders hidden.
Same result on every machine, no personal layout file.

    isaacsim --exec isaac_startup.py        (isaac_sim.sh does this)
"""
import asyncio
import os

import carb.settings
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import PhysxSchema, Usd

HERE = os.path.dirname(os.path.abspath(__file__))
# BEE_SCENE=scene_lab.usda isaac_sim.sh   another scene in usda/ (default scene_sim.usda)
SCENE = os.path.join(HERE, "..", "usda", os.environ.get("BEE_SCENE", "scene_sim.usda"))


async def _setup():
    app = omni.kit.app.get_app()
    for _ in range(60):  # let the default windows come up first
        await app.next_update_async()
    print(f"[isaac_startup] scene {os.path.abspath(SCENE)}")
    await omni.usd.get_context().open_stage_async(os.path.abspath(SCENE))
    # Viewport: light RTX - Minimal renderer, no collider debug draw, no guide-purpose prims (the robot's
    # collision meshes are guides). Set after opening the stage so per-user persistent settings don't win.
    settings = carb.settings.get_settings()
    settings.set("/rtx/rendermode", "MinimalRendering")
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
    # BEE_PROBLEM=<seed>|random: a random dice-mission problem (pucks · signal · barcode) — problem_setup.py
    if os.environ.get("BEE_PROBLEM"):
        ps = {}
        path = os.path.join(HERE, "problem_setup.py")
        exec(compile(open(path).read(), path, "exec"), ps)
        p = os.environ["BEE_PROBLEM"]
        seq = ps["setup"](None if p == "random" else int(p), float(os.environ.get("BEE_LYING", "0")), os.environ.get("BEE_RUN_DIR"))
        ps["play_signal"](seq)  # blinks start on Play (timeline time)
    bridge = os.path.join(HERE, "opencr_bridge.py")  # creates the pty now; it attaches to the robot on the first physics step
    exec(compile(open(bridge).read(), bridge, "exec"), {"__name__": "__main__", "__file__": bridge})
    if os.environ.get("BEE_AUTOPLAY") == "1":  # press Play (the bridge links /tmp/opencr on the first step)
        for _ in range(30):
            await app.next_update_async()
        omni.timeline.get_timeline_interface().play()


asyncio.ensure_future(_setup())
