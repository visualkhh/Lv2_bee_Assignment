"""Isaac Sim startup for the team: open usda/scene_sim.usda with the physx variant and start
opencr_bridge.py (virtual OpenCR on /tmp/opencr) so the ROS side can connect right away.
Viewport starts with colliders hidden, with a gripper_cam_rgb viewport docked to its right.
Same result on every machine, no personal layout file.

    isaacsim --exec isaac_startup.py        (isaac_sim.sh does this)
"""
import asyncio
import os

import carb.settings
import omni.kit.app
import omni.timeline
import omni.ui as ui
import omni.usd
from omni.kit.viewport.utility import create_viewport_window
from pxr import PhysxSchema, Usd

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "..", "..", "usda", "scene_sim.usda")
RESET_FILE = "/tmp/beebot_isaac_reset"  # isaac_sim.sh와 같은 경로


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
    # Second viewport docked right of the main one, looking through the gripper camera (what perception sees).
    cam = next(p.GetPath() for p in stage.Traverse() if p.GetName() == "gripper_cam_rgb")
    cam_vp = create_viewport_window("Gripper Cam", width=640, height=480, camera_path=cam)
    await app.next_update_async()
    cam_vp.dock_in(ui.Workspace.get_window("Viewport"), ui.DockPosition.RIGHT, 0.5)
    bridge = os.path.join(HERE, "opencr_bridge.py")  # creates the pty now; it attaches to the robot on the first physics step
    exec(compile(open(bridge).read(), bridge, "exec"), {"__name__": "__main__", "__file__": bridge})
    timeline = omni.timeline.get_timeline_interface()
    if os.environ.get("ISAAC_AUTOPLAY") == "1":  # 시연: Play를 자동으로
        timeline.play()
    while True:  # reset.sh: Stop(장면 처음 자세로) → Play — ROS 재시작은 isaac_sim.sh
        await app.next_update_async()
        if os.path.exists(RESET_FILE):
            timeline.stop()
            for _ in range(30):
                await app.next_update_async()
            timeline.play()
            os.remove(RESET_FILE)


asyncio.ensure_future(_setup())
