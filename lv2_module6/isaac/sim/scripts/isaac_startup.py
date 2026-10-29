"""Isaac Sim startup for the team: open usda/scene_sim.usda, put the Script Editor in a full-height
column left of the Viewport, load the Script Editor tools of this folder as tabs, and start
opencr_bridge.py (virtual OpenCR on /tmp/opencr) so the ROS side can connect right away.
Same result on every machine, no personal layout file.

    isaacsim --exec isaac_startup.py        (isaac_sim.sh does this)
"""
import asyncio
import os

import carb
import omni.kit.app
import omni.ui as ui
import omni.usd
from pxr import Usd

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "..", "usda", "scene_sim.usda")
TABS = ["opencr_bridge.py", "sim_drive.py"]  # scripts meant to run inside the Script Editor


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
    bridge = os.path.join(HERE, "opencr_bridge.py")  # creates the pty now; it attaches to the robot on the first physics step
    exec(compile(open(bridge).read(), bridge, "exec"), {"__name__": "__main__", "__file__": bridge})
    ui.Workspace.show_window("Script Editor")
    await app.next_update_async()
    editor = ui.Workspace.get_window("Script Editor")
    viewport = ui.Workspace.get_window("Viewport")
    if editor and viewport:
        editor.dock_in(viewport, ui.DockPosition.LEFT, 0.3)  # left column, Console/Content stay under the Viewport
    for _ in range(5):  # the editor widget is built on its first draw
        await app.next_update_async()
    # ponytail: private attribute of omni.kit.window.script_editor 2.0.x; if it changes, tabs are skipped (window still opens)
    tabs = getattr(getattr(editor, "_script_editor_widget", None), "_tabs", None)
    if tabs is None:
        carb.log_warn("[isaac_startup] Script Editor tabs not reachable, open the scripts with File > Open")
        return
    for name in TABS:
        tabs.load_script(os.path.join(HERE, name))


asyncio.ensure_future(_setup())
