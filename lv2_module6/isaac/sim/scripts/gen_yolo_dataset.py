"""Render a YOLO puck dataset from scene_sim.usda (headless, Isaac Sim Python only).

Classes = color x shape: puck_red_box, puck_red_cyl, puck_green_box, puck_green_cyl.
perception_node splits the name on "_" and reads the hue token, so these names keep
check_model_color working.

Each frame: the 12 scene pucks are scattered on the start boxes and L1-L3 tops (some lying),
dome light is scaled, and one camera with the D435i intrinsics of beebot.usda (640x480) is posed
like either the pole front cam (z 0.446 m, pitched about 18 deg down, 0.35-3 m away) or the
gripper cam (0.10-0.40 m above a puck, up to 30 deg off vertical). The robot is hidden, so
gripper fingers never appear in the frame. Every 12th frame has no pucks (negative sample).

    OMNI_KIT_ACCEPT_EULA=YES ~/pa-course/.venv-isaacsim/bin/python \
        isaac/sim/scripts/gen_yolo_dataset.py --output ~/data/bee_yolo_v1
    # sanity check: --train 8 --val 0 --test 0, then look at images/ and labels/
"""
import argparse
import json
import math
import os
import random
from pathlib import Path

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "..", "usda", "scene_sim.usda")
NAMES = ["puck_red_box", "puck_red_cyl", "puck_green_box", "puck_green_cyl"]
W, H = 640, 480
FOCAL, H_AP, V_AP = 15.2449, 15.855, 11.891  # beebot.usda gripper_cam_rgb / pole_front_cam_rgb
POLE_Z, POLE_PITCH = 0.446, 18.0  # pole cam height [m] and down pitch [deg] at the default pose
SURFACES = ["/World/start_boxes/start_%d" % i for i in range(1, 7)] + ["/World/shelf_L%d" % i for i in (1, 2, 3)]
P_GRIPPER, P_LYING, MAX_OCCLUSION = 0.4, 0.15, 0.85


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--train", type=int, default=1500)
    p.add_argument("--val", type=int, default=300)
    p.add_argument("--test", type=int, default=300)
    p.add_argument("--subframes", type=int, default=4, help="RTX subframes per capture (raise if ghosting)")
    a = p.parse_args()
    out = a.output.expanduser().resolve()
    if out.exists():
        raise SystemExit("Refusing to overwrite " + str(out))

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "renderer": "RaytracedLighting", "width": W, "height": H,
                         "anti_aliasing": 0, "multi_gpu": False})
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.usd
        import yaml
        from PIL import Image
        from pxr import Gf, Usd, UsdGeom, UsdSemantics

        ctx = omni.usd.get_context()
        ctx.open_stage(os.path.abspath(SCENE))
        while ctx.get_stage_loading_status()[2] > 0:
            app.update()
        stage = ctx.get_stage()
        UsdGeom.Imageable(stage.GetPrimAtPath("/World/beebot")).MakeInvisible()

        # Pucks: class from prim name puck_<shape>_<color>_<n>
        pucks = []
        for prim in stage.GetPrimAtPath("/World/pucks").GetChildren():
            _, shape, color, _ = prim.GetName().split("_")
            name = "puck_%s_%s" % (color, shape)
            UsdSemantics.LabelsAPI.Apply(prim, "class").CreateLabelsAttr([name])
            pucks.append(prim)
        if len(pucks) != 12:
            raise RuntimeError("Expected 12 pucks, found %d" % len(pucks))

        # Surface tops from world bounds, shrunk so a standing puck (3 cm) stays on top
        bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
        tops = []
        for path in SURFACES:
            r = bbc.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
            lo, hi = r.GetMin(), r.GetMax()
            tops.append((lo[0] + .03, hi[0] - .03, lo[1] + .03, hi[1] - .03, hi[2]))
        arena = bbc.ComputeWorldBound(stage.GetPrimAtPath("/World/maze")).ComputeAlignedRange()

        dome = stage.GetPrimAtPath("/World/Environment/DomeLight")
        dome_base = dome.GetAttribute("inputs:intensity").Get()

        cam = UsdGeom.Camera.Define(stage, "/World/DatasetCam")
        cam.CreateFocalLengthAttr(FOCAL)
        cam.CreateHorizontalApertureAttr(H_AP)
        cam.CreateVerticalApertureAttr(V_AP)
        cam.CreateClippingRangeAttr(Gf.Vec2f(.01, 20))
        cam_op = cam.AddTransformOp()
        rp = rep.create.render_product(str(cam.GetPath()), (W, H))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([rp])
        bbox = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
        bbox.attach([rp])

        def scatter(rng):
            placed = []  # (x, y, top_z) of every puck, for spacing and camera targets
            for prim in pucks:
                for _ in range(100):
                    x0, x1, y0, y1, z = rng.choice(tops)
                    x, y = rng.uniform(x0, x1), rng.uniform(y0, y1)
                    if all(math.hypot(x - px, y - py) > .07 for px, py, _ in placed):
                        break
                lying = rng.random() < P_LYING
                prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(x, y, z + (.015 if lying else 0)))
                prim.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3f(90 if lying else 0, 0, rng.uniform(0, 360)))
                placed.append((x, y, z + (.03 if lying else .06)))
            return placed

        def look(eye, target, up):
            cam_op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*up)).GetInverse())

        def pose_camera(rng, target):
            tx, ty, tz = target
            if rng.random() < P_GRIPPER:
                h, tilt, az = rng.uniform(.10, .40), math.radians(rng.uniform(0, 30)), rng.uniform(0, 2 * math.pi)
                eye = (tx + h * math.tan(tilt) * math.cos(az), ty + h * math.tan(tilt) * math.sin(az), tz + h)
                aim = (tx + rng.uniform(-.04, .04), ty + rng.uniform(-.04, .04), tz)
                roll = rng.uniform(0, 2 * math.pi)
                look(eye, aim, (math.cos(roll), math.sin(roll), 0))
                return "gripper"
            # pole: fixed height/pitch, puck somewhere inside the horizontal FOV
            for _ in range(50):
                d, az = rng.uniform(.35, 3.0), rng.uniform(0, 2 * math.pi)
                ex, ey = tx + d * math.cos(az), ty + d * math.sin(az)
                if arena.GetMin()[0] < ex < arena.GetMax()[0] and arena.GetMin()[1] < ey < arena.GetMax()[1]:
                    break
            yaw = math.atan2(ty - ey, tx - ex) + math.radians(rng.uniform(-22, 22))
            pitch = math.radians(POLE_PITCH + rng.uniform(-3, 3))
            ez = POLE_Z + rng.uniform(-.01, .01)
            fwd = (math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch), -math.sin(pitch))
            look((ex, ey, ez), (ex + fwd[0], ey + fwd[1], ez + fwd[2]), (0, 0, 1))
            return "pole"

        def capture():
            rep.orchestrator.step(rt_subframes=a.subframes)
            pixels = np.asarray(rgb.get_data())[:, :, :3]
            data = bbox.get_data()
            ids = data["info"]["idToLabels"]
            rows = []
            for row in data["data"]:
                label = ids.get(str(int(row["semanticId"])), {}).get("class", "")
                if label not in NAMES or float(row["occlusionRatio"]) > MAX_OCCLUSION:
                    continue
                x1, y1 = max(0.0, float(row["x_min"])), max(0.0, float(row["y_min"]))
                x2, y2 = min(float(W), float(row["x_max"])), min(float(H), float(row["y_max"]))
                if x2 - x1 < 3 or y2 - y1 < 3:
                    continue
                rows.append("%d %.6f %.6f %.6f %.6f" % (NAMES.index(label), (x1 + x2) / 2 / W,
                                                        (y1 + y2) / 2 / H, (x2 - x1) / W, (y2 - y1) / H))
            return pixels, rows

        for _ in range(20):
            app.update()
        manifest = {"scene": os.path.abspath(SCENE), "classes": NAMES, "resolution": [W, H], "samples": []}
        for split, count, base in [("train", a.train, 100000), ("val", a.val, 300000), ("test", a.test, 500000)]:
            (out / "images" / split).mkdir(parents=True)
            (out / "labels" / split).mkdir(parents=True)
            for i in range(count):
                seed = base + i
                rng = random.Random(seed)
                negative = i % 12 == 11
                placed = scatter(rng)
                for prim in pucks:
                    img = UsdGeom.Imageable(prim)
                    img.MakeInvisible() if negative else img.MakeVisible()
                dome.GetAttribute("inputs:intensity").Set(dome_base * rng.uniform(.3, 1.8))
                for _ in range(8):  # walls/posts often hide the target: re-pose until a puck is labeled
                    view = pose_camera(rng, rng.choice(placed))
                    pixels, rows = capture()
                    if rows or negative:
                        break
                Image.fromarray(pixels).save(out / "images" / split / ("%05d.png" % i))
                (out / "labels" / split / ("%05d.txt" % i)).write_text("".join(r + "\n" for r in rows))
                manifest["samples"].append(dict(split=split, index=i, seed=seed, view=view, negative=negative, labels=len(rows)))
                print("RENDER %s %d/%d %s labels=%d" % (split, i + 1, count, view, len(rows)), flush=True)

        (out / "manifest.json").write_text(json.dumps(manifest, indent=1))
        (out / "dataset.yaml").write_text(yaml.safe_dump(dict(path=str(out), train="images/train", val="images/val",
                                                              test="images/test", names=dict(enumerate(NAMES))), sort_keys=False))
        print("DATASET COMPLETE", out, flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
