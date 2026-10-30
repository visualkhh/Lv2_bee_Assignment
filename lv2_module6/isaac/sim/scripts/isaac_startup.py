"""Isaac Sim startup for the team: open usda/scene_sim.usda with the physx variant and start
opencr_bridge.py (virtual OpenCR on /tmp/opencr) so the ROS side can connect right away.
Same result on every machine, no personal layout file.

    isaacsim --exec isaac_startup.py        (isaac_sim.sh does this)
"""
import asyncio
import os

import omni.kit.app
import omni.usd
from pxr import PhysxSchema, Usd

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "..", "usda", "scene_sim.usda")


async def _setup():
    app = omni.kit.app.get_app()
    for _ in range(60):  # let the default windows come up first
        await app.next_update_async()
    await omni.usd.get_context().open_stage_async(os.path.abspath(SCENE))
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
    bridge = os.path.join(HERE, "opencr_bridge.py")  # creates the pty now; it attaches to the robot on the first physics step
    exec(compile(open(bridge).read(), bridge, "exec"), {"__name__": "__main__", "__file__": bridge})


asyncio.ensure_future(_setup())
