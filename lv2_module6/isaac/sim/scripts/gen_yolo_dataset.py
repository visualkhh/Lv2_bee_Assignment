"""Render a YOLO-OBB puck + box dataset from scene_sim.usda (headless, Isaac Sim Python only).

Classes = color x shape x posture: puck_red_box, puck_red_cyl, puck_green_box, puck_green_cyl and the
same four with a _lying suffix, plus "box": the 20.7 cm cardboard boxes (start 1-6 and places L1-L3 look
the same apart from the L1-L3 top tag, so one class; which box it is comes from the map / the tag). perception_node splits the name on "_" and reads the hue token (index 1),
so these names keep check_model_color working. Labels are Ultralytics OBB rows
(class x1 y1 x2 y2 x3 y3 x4 y4, normalized): the min-area rectangle of the visible puck mask, so for a
lying puck its long side follows the puck axis in the image (0-180 deg, the two ends look the same).

Each frame: the 12 scene pucks are crowded onto 3-5 of the start boxes and L1-L3 tops (2-4 per top, so
mixed colors and shapes share a view; P_LYING of them lying, any yaw), each gets a random red/green body color (keeps classes balanced; the tag id stays the prim's),
dome light is scaled, and one camera with the D435i intrinsics of beebot.usda (640x480) is posed like
either the pole front cam (z 0.446 m, pitched about 18 deg down, 0.45-3 m away) or the gripper cam
(0.10-0.40 m above a puck, up to 30 deg off vertical). The robot is hidden, so gripper fingers never
appear in the frame. Every 12th frame has no pucks (negative sample). Clean : noisy frames = 3 : 2;
noisy frames get exposure / white-balance / blur / sensor-noise / JPEG artifacts in image space.

manifest.json keeps the boxes (name, center, yaw), per frame the camera pose (world, USD camera axes) and K, and per puck its tag id,
color, shape, lying flag, world position and world axis (bottom -> top), for checking the lying
direction / tag PnP against ground truth later.

    OMNI_KIT_ACCEPT_EULA=YES ~/.venv-isaacsim/bin/python \
        isaac/sim/scripts/gen_yolo_dataset.py --output ~/data/bee_yolo_obb_v1
    # sanity check: --train 10 --val 0 --test 0, then look at images/ and labels/
    # train: yolo obb train data=~/data/bee_yolo_obb_v1/dataset.yaml model=yolo11n-obb.pt imgsz=640
"""
import argparse
import json
import math
import os
import random
from pathlib import Path

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "..", "usda", "scene_sim.usda")
BASE = ["puck_red_box", "puck_red_cyl", "puck_green_box", "puck_green_cyl"]
NAMES = BASE + [n + "_lying" for n in BASE] + ["box"]
COLORS = {"red": (0.6157, 0.1333, 0.2078), "green": (0, 0.6824, 0.2588)}  # puck_*_red_1 / *_green_0 .usda
W, H = 640, 480
FOCAL, H_AP, V_AP = 15.2449, 15.855, 11.891  # beebot.usda gripper_cam_rgb / pole_front_cam_rgb
FX, FY = FOCAL / H_AP * W, FOCAL / V_AP * H
POLE_Z, POLE_PITCH = 0.446, 18.0  # pole cam height [m] and down pitch [deg] at the default pose
SURFACES = ["/World/start_boxes/start_%d" % i for i in range(1, 7)] + ["/World/shelf_L%d" % i for i in (1, 2, 3)]
P_GRIPPER, P_LYING, MAX_OCCLUSION = 0.4, 0.4, 0.85
PUCK_R, PUCK_L = 0.015, 0.06  # puck half width, height [m]; origin = bottom center


def add_noise(rng, img):
    """Image-space camera artifacts; returns (uint8 image, params). Hue shifts stay small: color is the class."""
    import cv2
    import numpy as np
    nrng = np.random.default_rng(rng.getrandbits(32))
    p = dict(gain=rng.uniform(.55, 1.45), gamma=rng.uniform(.8, 1.25),
             wb=[rng.uniform(.92, 1.08) for _ in range(3)], blur=rng.choice(["none", "gauss", "motion"]),
             sigma=rng.uniform(2, 10), jpeg=rng.randint(35, 90))
    x = img.astype(np.float32) / 255
    x = np.clip(x * p["gain"] * np.array(p["wb"], np.float32), 0, 1) ** p["gamma"]
    if p["blur"] == "gauss":
        x = cv2.GaussianBlur(x, (0, 0), rng.uniform(.6, 1.6))
    elif p["blur"] == "motion":
        k = rng.choice([5, 7, 9])
        kern = np.zeros((k, k), np.float32)
        kern[k // 2] = 1
        kern = cv2.warpAffine(kern, cv2.getRotationMatrix2D((k / 2 - .5, k / 2 - .5), rng.uniform(0, 180), 1), (k, k))
        x = cv2.filter2D(x, -1, kern / max(kern.sum(), 1e-6))
    x = x + nrng.normal(0, p["sigma"] / 255, x.shape) + nrng.normal(0, 1, x.shape) * np.sqrt(x / 400)  # read + shot
    x = (np.clip(x, 0, 1) * 255 + .5).astype(np.uint8)
    ok, buf = cv2.imencode(".jpg", x[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, p["jpeg"]])
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)[:, :, ::-1].copy(), p


def obb_row(cls, mask):
    """Ultralytics OBB label row from a boolean mask, or None if too small."""
    import cv2
    import numpy as np
    ys, xs = np.nonzero(mask)
    if len(xs) < 12:
        return None
    (cx, cy), (w, h), ang = cv2.minAreaRect(np.stack([xs, ys], 1).astype(np.float32))
    pts = cv2.boxPoints(((cx + .5, cy + .5), (w + 1, h + 1), ang))  # pixel centers -> pixel edges
    pts = pts / np.array([W, H], np.float32)
    return "%d " % cls + " ".join("%.6f" % v for v in np.clip(pts, 0, 1).ravel())


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--train", type=int, default=4000)
    p.add_argument("--val", type=int, default=500)
    p.add_argument("--test", type=int, default=500)
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
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdSemantics

        ctx = omni.usd.get_context()
        ctx.open_stage(os.path.abspath(SCENE))
        while ctx.get_stage_loading_status()[2] > 0:
            app.update()
        stage = ctx.get_stage()
        UsdGeom.Imageable(stage.GetPrimAtPath("/World/beebot")).MakeInvisible()

        # Pucks: shape and tag id from prim name puck_<shape>_<color>_<id>; the semantic label is the
        # prim name itself so bbox / segmentation rows map back to a puck whose color changes per frame.
        pucks = []
        for prim in stage.GetPrimAtPath("/World/pucks").GetChildren():
            _, shape, _, tag = prim.GetName().split("_")
            UsdSemantics.LabelsAPI.Apply(prim, "class").CreateLabelsAttr([prim.GetName()])
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)  # PhysX must not move the posed pucks
            tint = stage.GetPrimAtPath("%s/%s/Looks/mtlxmultiply2" % (prim.GetPath(), shape))
            if not tint:
                raise RuntimeError("No body color shader under " + str(prim.GetPath()))
            pucks.append(dict(prim=prim, name=prim.GetName(), shape=shape, tag=int(tag),
                              tint=tint.GetAttribute("inputs:in1") or tint.CreateAttribute("inputs:in1", Sdf.ValueTypeNames.Color3f)))
        if len(pucks) != 12:
            raise RuntimeError("Expected 12 pucks, found %d" % len(pucks))
        by_name = {pk["name"]: pk for pk in pucks}
        boxes = []  # start 1-6 + L1-L3, labeled with their prim name like the pucks
        for path in SURFACES:
            prim = stage.GetPrimAtPath(path)
            UsdSemantics.LabelsAPI.Apply(prim, "class").CreateLabelsAttr([prim.GetName().lower()])  # replicator lowercases labels
            m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            x = m.TransformDir(Gf.Vec3d(1, 0, 0))
            boxes.append(dict(name=prim.GetName(), center=list(m.ExtractTranslation()),
                              yaw=math.degrees(math.atan2(x[1], x[0])) % 90))  # square box: 90 deg symmetric
        box_names = {b["name"].lower() for b in boxes}

        # Surface tops from world bounds (all axis-aligned squares)
        bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
        tops = []
        for path in SURFACES:
            r = bbc.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
            lo, hi = r.GetMin(), r.GetMax()
            tops.append((lo[0], hi[0], lo[1], hi[1], hi[2]))
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
        seg = rep.AnnotatorRegistry.get_annotator("semantic_segmentation", init_params={"colorize": False})
        seg.attach([rp])

        def scatter(rng):
            placed = []  # (x, y, z): body center for spacing, top surface height for camera targets
            used = rng.sample(tops, rng.randint(3, 5))  # crowd the tops so mixed pucks share a view
            for pk in pucks:
                lying = rng.random() < P_LYING
                yaw = rng.uniform(0, 360)
                margin = PUCK_L / 2 + PUCK_R if lying else PUCK_R * 2  # body must stay on the top
                for n in range(160):  # crowded tops full -> spill over to any top
                    x0, x1, y0, y1, z = rng.choice(used if n < 100 else tops)
                    cx, cy = rng.uniform(x0 + margin, x1 - margin), rng.uniform(y0 + margin, y1 - margin)
                    if all(math.hypot(cx - px, cy - py) > .07 for px, py, _ in placed):
                        break
                color = rng.choice(sorted(COLORS))
                pk["tint"].Set(Gf.Vec3f(*COLORS[color]))
                if lying:  # rotateXYZ(90, 0, yaw) sends local +Z (bottom -> top) to (sin yaw, -cos yaw, 0)
                    ax, ay = math.sin(math.radians(yaw)), -math.cos(math.radians(yaw))
                    pos = (cx - ax * PUCK_L / 2, cy - ay * PUCK_L / 2, z + PUCK_R)
                    placed.append((cx, cy, z + 2 * PUCK_R))
                else:
                    pos = (cx, cy, z)
                    placed.append((cx, cy, z + PUCK_L))
                pk["prim"].GetAttribute("xformOp:translate").Set(Gf.Vec3d(*pos))
                pk["prim"].GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3f(90 if lying else 0, 0, yaw))
                pk.update(color=color, lying=lying)
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
            # pole: fixed height/pitch, puck somewhere inside the horizontal FOV. Closer than ~0.45 m the
            # puck is below the bottom edge of the 42.6 deg vertical FOV.
            for _ in range(50):
                d, az = rng.uniform(.45, 3.0), rng.uniform(0, 2 * math.pi)
                ex, ey = tx + d * math.cos(az), ty + d * math.sin(az)
                if arena.GetMin()[0] < ex < arena.GetMax()[0] and arena.GetMin()[1] < ey < arena.GetMax()[1]:
                    break
            yaw = math.atan2(ty - ey, tx - ex) + math.radians(rng.uniform(-22, 22))
            pitch = math.radians(POLE_PITCH + rng.uniform(-3, 3))
            ez = POLE_Z + rng.uniform(-.01, .01)
            fwd = (math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch), -math.sin(pitch))
            look((ex, ey, ez), (ex + fwd[0], ey + fwd[1], ez + fwd[2]), (0, 0, 1))
            return "pole"

        def project(pk):
            """Pixel of the puck's body center in the dataset camera (far away if behind it)."""
            c = UsdGeom.Xformable(pk["prim"]).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).Transform(
                Gf.Vec3d(0, 0, PUCK_L / 2))
            c = cam_op.Get().GetInverse().Transform(c)  # USD camera: -Z forward, +Y up
            return (W / 2 + FX * c[0] / -c[2], H / 2 + FY * c[1] / c[2]) if c[2] < 0 else (1e9, 1e9)

        def capture():
            rep.orchestrator.step(rt_subframes=a.subframes, delta_time=0.0)  # timeline (physics) stays put
            pixels = np.asarray(rgb.get_data())[:, :, :3]
            sdata = seg.get_data()
            sids = np.asarray(sdata["data"])
            sid_of = {lab.get("class", ""): int(k) for k, lab in sdata["info"]["idToLabels"].items()}
            data = bbox.get_data()
            ids = data["info"]["idToLabels"]
            rows, n_puck = [], 0
            for row in data["data"]:
                label = ids.get(str(int(row["semanticId"])), {}).get("class", "")
                pk = by_name.get(label)
                # occlusionRatio reads -1 (not computed) in replicator 1.13, so in practice only the 12 px mask rule drops slivers
                if (pk is None and label not in box_names) or float(row["occlusionRatio"]) > MAX_OCCLUSION:
                    continue
                if min(row["x_max"], W) - max(row["x_min"], 0) < 3 or min(row["y_max"], H) - max(row["y_min"], 0) < 3:
                    continue
                if pk is None:  # box
                    r = obb_row(NAMES.index("box"), sids == sid_of.get(label, -1))
                    if r:
                        rows.append(r)
                    continue
                cls = NAMES.index("puck_%s_%s%s" % (pk["color"], pk["shape"], "_lying" if pk["lying"] else ""))
                mask = sids == sid_of.get(pk["name"], -1)
                r = obb_row(cls, mask)
                if r:
                    ys, xs = np.nonzero(mask)
                    # label must belong to the puck whose ground truth projects onto the mask (a heavily
                    # cropped/occluded mask can still sit nearer a neighbor, so only the rate is checked)
                    near = min(pucks, key=lambda q: math.dist(project(q), (xs.mean(), ys.mean())))
                    stats["checked"] += 1
                    stats["mismatch"] += near is not pk
                    rows.append(r)
                    n_puck += 1
            return pixels, rows, n_puck

        def matrix(m):
            return [list(m.GetRow(i)) for i in range(4)]

        stats = {"checked": 0, "mismatch": 0}
        for _ in range(20):
            app.update()
        manifest = {"scene": os.path.abspath(SCENE), "classes": NAMES, "resolution": [W, H], "boxes": boxes,
                    "K": [[FX, 0, W / 2], [0, FY, H / 2], [0, 0, 1]], "samples": []}
        for split, count, base in [("train", a.train, 100000), ("val", a.val, 300000), ("test", a.test, 500000)]:
            (out / "images" / split).mkdir(parents=True)
            (out / "labels" / split).mkdir(parents=True)
            for i in range(count):
                seed = base + i
                rng = random.Random(seed)
                negative = i % 12 == 11
                noisy = i % 5 >= 3  # clean : noisy = 3 : 2
                placed = scatter(rng)
                if negative:  # sink the pucks out of view; toggling visibility scrambles replicator's semantic ids
                    for pk in pucks:
                        t = pk["prim"].GetAttribute("xformOp:translate")
                        t.Set(t.Get() - Gf.Vec3d(0, 0, 100))
                dome.GetAttribute("inputs:intensity").Set(dome_base * rng.uniform(.3, 1.8))
                for _ in range(8):  # walls/posts often hide the target: re-pose until a puck is labeled
                    view = pose_camera(rng, rng.choice(placed))
                    pixels, rows, n_puck = capture()
                    if n_puck or negative:
                        break
                noise = None
                if noisy:
                    pixels, noise = add_noise(rng, pixels)
                Image.fromarray(pixels).save(out / "images" / split / ("%05d.png" % i))
                (out / "labels" / split / ("%05d.txt" % i)).write_text("".join(r + "\n" for r in rows))
                gt = []
                for pk in pucks:
                    m = UsdGeom.Xformable(pk["prim"]).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                    axis = m.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()
                    if abs(axis[2]) > .05 if pk["lying"] else axis[2] < .99:
                        raise RuntimeError("%s pose drifted from the label: axis %s" % (pk["name"], axis))
                    gt.append(dict(name=pk["name"], tag=pk["tag"], color=pk["color"], shape=pk["shape"],
                                   lying=pk["lying"], pos=list(m.ExtractTranslation()), axis=list(axis)))
                cam_m = UsdGeom.Xformable(cam.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                manifest["samples"].append(dict(split=split, index=i, seed=seed, view=view, negative=negative,
                                                noise=noise, labels=len(rows), cam_to_world=matrix(cam_m),
                                                pucks=[] if negative else gt))
                print("RENDER %s %d/%d %s %s labels=%d mismatch=%d/%d" % (split, i + 1, count, view, "noisy" if noisy
                                                                          else "clean", len(rows), stats["mismatch"], stats["checked"]), flush=True)
                if stats["checked"] >= 50 and stats["mismatch"] > .03 * stats["checked"]:
                    raise RuntimeError("labels landing on the wrong puck: semantic ids out of sync (%s)" % stats)

        (out / "manifest.json").write_text(json.dumps(manifest, indent=1))
        (out / "dataset.yaml").write_text(yaml.safe_dump(dict(path=str(out), train="images/train", val="images/val",
                                                              test="images/test", names=dict(enumerate(NAMES))), sort_keys=False))
        print("DATASET COMPLETE", out, flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
