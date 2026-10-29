"""Synthetic YOLO dataset of pucks from scene_sim.usda, seen through beebot's own cameras.

Run headless with the Isaac Sim venv python (pip install has no python.sh):
    source ~/pa-course/.venv-isaacsim/bin/activate
    OMNI_KIT_ACCEPT_EULA=YES python sdg_pucks.py --out synthetic_datas/v1
Quick check first: --train 8 --val 4 --test 4

Output (YOLO format), file prefix = camera:
    <out>/images/{train,val,test}/pole_0000.png, <out>/labels/{train,val,test}/pole_0000.txt
    <out>/dataset.yaml

Pucks and shelves stay exactly as authored (made kinematic in memory so the robot cannot push them).
Only beebot moves. Each sample, cameras alternate:
  - pick a random puck as target; light: normal / dark / bright in turn
  - pole: base 0.5-2.5 m from the target, facing it +-25 deg, arm STOW
  - grip: random arm pose whose gripper cam looks down 0.12-0.6 m; base slid/rotated so that view hits the target
  - candidate base poses whose footprint overlaps any object are skipped (PhysX overlap query);
    after teleport, the sample is dropped if the robot got pushed/tilted or the arm got blocked
  - every 10th sample pair hides all pucks (negative, empty label file)
Non-negative frames where the camera sees no puck are re-rolled (max 50).
Class = color x shape, taken from the prim name (puck_box_red_0 -> puck_red_box).
"""
import argparse
import math
import os
import random

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--train", type=int, default=1600)
ap.add_argument("--val", type=int, default=200)
ap.add_argument("--test", type=int, default=200)
ap.add_argument("--seed", type=int, default=0)
args = ap.parse_args()
if os.path.exists(args.out):
    raise SystemExit(f"Refusing to overwrite {args.out}")

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import carb
import numpy as np
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim, SingleArticulation
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.utils.types import ArticulationAction
from omni.physx import get_physx_scene_query_interface
from PIL import Image
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdSemantics

SCENE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "usda", "scene_sim.usda")
W, H = 640, 480  # real RealSense color profile 640x480 (D435 crops the 16:9 sensor sides: ~55 x 42.6 deg)
CLASSES = ["puck_red_box", "puck_red_cyl", "puck_green_box", "puck_green_cyl"]
ROBOT = "/World/beebot"
BASE = ROBOT + "/Geometry/base_footprint/base_link"
GROUND = "/World/Environment/GroundPlane"
ARM = ["joint1", "joint2", "joint3", "joint4", "joint5"]
ARM_RANGE_DEG = [(-0, 0), (-110, -100), (85, 80), (30, 25), (-0, 0)]  # inside the USD joint limits
LIGHT = {"normal": (0.9, 1.4), "dark": (0.15, 0.3), "bright": (1.8, 2.5)}  # x authored dome intensity
MARGIN = 0.03  # [m] free space kept around the robot footprint

random.seed(args.seed)
open_stage(SCENE)
stage = omni.usd.get_context().get_stage()
# scene_sim.usda selects no Physics variant ("none" = no articulation); pick physx in memory only
stage.GetPrimAtPath(ROBOT).GetVariantSet("Physics").SetVariantSelection("physx")
for _ in range(10):  # let payloads/materials load
    app.update()
find = lambda name: next(p for p in stage.Traverse() if p.GetName() == name)

pucks = []
for prim in stage.GetPrimAtPath("/World/pucks").GetChildren():
    _, shape, color, _ = prim.GetName().split("_")  # puck_box_red_0
    UsdSemantics.LabelsAPI.Apply(prim, "class").CreateLabelsAttr([f"puck_{color}_{shape}"])
    pucks.append((UsdGeom.Imageable(prim), prim.GetAttribute("xformOp:translate").Get()))

# in memory: every rigid body except the robot (pucks, shelves) becomes kinematic, so nothing can be pushed
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.RigidBodyAPI) and not str(prim.GetPath()).startswith(ROBOT):
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)

dome = UsdLux.DomeLight(stage.GetPrimAtPath("/World/Environment/DomeLight")).GetIntensityAttr()
dome0 = dome.Get()

# robot footprint box in the base_link frame (STOW pose, as authored), floor contact left out
xc = UsdGeom.XformCache()
bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
rng = bbox.ComputeRelativeBound(stage.GetPrimAtPath(ROBOT + "/Geometry"), stage.GetPrimAtPath(BASE)).ComputeAlignedRange()
lo, hi = rng.GetMin() - Gf.Vec3d(MARGIN, MARGIN, 0), rng.GetMax() + Gf.Vec3d(MARGIN, MARGIN, MARGIN)
lo[2] += 0.02  # keep the box off the floor
foot_half = (hi - lo) / 2
foot_center = (hi + lo) / 2

world = World(stage_units_in_meters=1.0, physics_prim_path="/PhysicsScene")
robot = world.scene.add(SingleArticulation(BASE, name="beebot"))
# gripper cam pose = gripper_cam_link physics pose x fixed link->camera offset (read once from USD)
grip_link = world.scene.add(RigidPrim(str(find("gripper_cam_link").GetPath()), name="grip_link"))
grip_offset = xc.GetLocalToWorldTransform(find("gripper_cam_rgb")) * xc.GetLocalToWorldTransform(find("gripper_cam_link")).GetInverse()
world.reset()
base_z = robot.get_world_pose()[0][2]
arm_idx = [robot.get_dof_index(j) for j in ARM]
finger_idx = [robot.get_dof_index(j) for j in ("gripper_left_joint", "gripper_right_joint")]
sq = get_physx_scene_query_interface()

annot = {}
for cam, name in (("pole", "pole_front_cam_rgb"), ("grip", "gripper_cam_rgb")):
    rp = rep.create.render_product(str(find(name).GetPath()), (W, H))
    annot[cam] = [rep.AnnotatorRegistry.get_annotator(a) for a in ("rgb", "bounding_box_2d_tight")]
    for a in annot[cam]:
        a.attach(rp)


def free(xy, yaw):
    """True if the robot footprint at this base pose touches nothing but the floor and itself."""
    c, s = math.cos(yaw), math.sin(yaw)
    center = (xy[0] + c * foot_center[0] - s * foot_center[1], xy[1] + s * foot_center[0] + c * foot_center[1],
              base_z + foot_center[2])
    hits = []

    def report(hit):
        if not (hit.rigid_body.startswith(ROBOT) or hit.collision.startswith(GROUND)):
            hits.append(hit.collision)
            return False  # stop at the first real hit
        return True

    sq.overlap_box(carb.Float3(*foot_half), carb.Float3(*center), carb.Float4(0, 0, math.sin(yaw / 2), math.cos(yaw / 2)), report, False)
    return not hits


def teleport(xy, yaw, arm_deg, z=0.0, opening=0.0):
    robot.set_world_pose(np.array([xy[0], xy[1], base_z + z]), np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]))  # wxyz
    robot.set_linear_velocity(np.zeros(3))
    robot.set_angular_velocity(np.zeros(3))
    q = robot.get_joint_positions()
    q[arm_idx] = np.radians(arm_deg)
    q[finger_idx] = opening  # [m] per finger, 0-0.0265 (USD prismatic limits)
    robot.set_joint_positions(q)
    robot.set_joint_velocities(np.zeros_like(q))
    robot.apply_action(ArticulationAction(joint_positions=q))  # drives hold the pose
    for _ in range(5):  # let the teleport settle
        world.step(render=False)


def settled(xy, arm_deg):
    """False if the robot got pushed or tilted, or the arm was blocked by something."""
    p, (w, x, y, z) = robot.get_world_pose()
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))
    arm_err = np.abs(np.degrees(robot.get_joint_positions()[arm_idx]) - arm_deg).max()
    return tilt < 2 and math.hypot(p[0] - xy[0], p[1] - xy[1]) < 0.01 and arm_err < 3


def place_pole(target):
    for _ in range(100):
        az = random.uniform(0, 2 * math.pi)
        d = random.uniform(0.5, 2.5)
        xy = (target[0] + d * math.cos(az), target[1] + d * math.sin(az))
        yaw = az + math.pi + math.radians(random.uniform(-25, 25))  # face the target +-25 deg
        if free(xy, yaw):
            teleport(xy, yaw, [0] * 5)
            return settled(xy, [0] * 5)
    return False


def place_grip(target):
    target = target + Gf.Vec3d(random.uniform(-0.03, 0.03), random.uniform(-0.03, 0.03), 0)  # puck not always centered
    for _ in range(100):
        arm = [random.uniform(*r) for r in ARM_RANGE_DEG]
        opening = random.uniform(0, 0.0265)  # fingers show in the gripper cam: vary them too
        teleport((target[0], target[1]), 0.0, arm, z=5.0)  # probe in free air: nothing to collide with
        p, o = grip_link.get_world_poses()  # shapes (1, 3), (1, 4) wxyz
        link = Gf.Matrix4d().SetRotate(Gf.Quatd(*map(float, o[0])))
        cam = grip_offset * link.SetTranslateOnly(Gf.Vec3d(*map(float, p[0])))
        eye, ray = cam.ExtractTranslation(), cam.TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized()  # USD camera looks -Z
        if ray[2] > -0.5:  # want at least 30 deg down
            continue
        bp, (bw, _, _, bz) = robot.get_world_pose()
        t = (target[2] + bp[2] - base_z - eye[2]) / ray[2]  # target height, measured in the lifted probe
        if not 0.12 < t < 0.6:
            continue
        hit = eye + ray * t
        # where the camera looks, in the base frame -> put the base so that point lands on the target
        byaw = 2 * math.atan2(bz, bw)
        hx, hy = hit[0] - bp[0], hit[1] - bp[1]
        vx, vy = math.cos(byaw) * hx + math.sin(byaw) * hy, -math.sin(byaw) * hx + math.cos(byaw) * hy
        for _ in range(8):  # try a few headings around the target for a free spot
            a = random.uniform(0, 2 * math.pi)
            xy = (target[0] - (math.cos(a) * vx - math.sin(a) * vy), target[1] - (math.sin(a) * vx + math.cos(a) * vy))
            if free(xy, a):
                teleport(xy, a, arm, opening=opening)
                return settled(xy, arm)
    return False


def setup(cam, light, negative):
    for vis, _ in pucks:
        vis.GetVisibilityAttr().Set(UsdGeom.Tokens.invisible if negative else UsdGeom.Tokens.inherited)
    # ponytail: only dome intensity varies; add HDRI swap/rotation if real-robot mAP drops under other lighting
    dome.Set(dome0 * random.uniform(*LIGHT[light]))
    target = Gf.Vec3d(random.choice(pucks)[1])
    return (place_pole if cam == "pole" else place_grip)(target)


def capture(cam):
    rep.orchestrator.step(rt_subframes=8)  # extra subframes so RTX converges after big moves
    rgb, bbox = annot[cam]
    boxes = bbox.get_data()
    id2label = boxes["info"]["idToLabels"]
    lines = []
    for b in boxes["data"]:
        name = id2label.get(str(b["semanticId"]), {}).get("class")
        x0, y0, x1, y1 = (float(b[k]) for k in ("x_min", "y_min", "x_max", "y_max"))
        if name in CLASSES and b["occlusionRatio"] <= 0.7 and min(x1 - x0, y1 - y0) >= 4:
            lines.append(f"{CLASSES.index(name)} {(x0 + x1) / 2 / W:.6f} {(y0 + y1) / 2 / H:.6f} "
                         f"{(x1 - x0) / W:.6f} {(y1 - y0) / H:.6f}\n")
    return np.asarray(rgb.get_data())[:, :, :3], lines


for split, count in (("train", args.train), ("val", args.val), ("test", args.test)):
    os.makedirs(f"{args.out}/images/{split}")
    os.makedirs(f"{args.out}/labels/{split}")
    for i in range(count):
        cam, light, negative = ("pole", "grip")[i % 2], list(LIGHT)[i % 3], i // 2 % 10 == 9
        for _ in range(50):
            if not setup(cam, light, negative):
                continue
            pixels, lines = capture(cam)
            if lines or negative:
                break
        else:
            raise RuntimeError(f"{cam}: no usable sample in 50 tries (free spot / arm range / line of sight)")
        name = f"{cam}_{i:04d}"
        Image.fromarray(pixels).save(f"{args.out}/images/{split}/{name}.png")
        with open(f"{args.out}/labels/{split}/{name}.txt", "w") as f:
            f.writelines(lines)
        print(f"[sdg] {split} {i + 1}/{count} {name} {light} boxes={len(lines)}", flush=True)

with open(f"{args.out}/dataset.yaml", "w") as f:
    f.write(f"path: {os.path.abspath(args.out)}\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n")
    f.writelines(f"  {i}: {c}\n" for i, c in enumerate(CLASSES))

rep.orchestrator.wait_until_complete()
app.close()
