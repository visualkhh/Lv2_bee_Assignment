"""Offline plan for the ROS stand-up + regrasp dummy (stand_dummy.sh, removed: see git history) in scene_lab.usda, read as placed by the
user (robot facing start_1, puck_box_green_6 lying on it); nothing is moved. The dummy first drives straight ahead to
the training dock (start_1 centre 0.08 m + half a box ahead of the root, right_env BOX_CX); the plan is made for that
dock pose, so the real stopping error adds to the perception error (as on the robot). Perception stand-in: true pose + the
training noise (+-5 mm, +-10 deg). Writes the joint samples the dummy node streams to the arm_controller (JTC).

    ~/pa-course/IsaacLab-6.1/.venv/bin/python stand_plan.py [--seed 0] [--out /tmp/stand_plan.json]
(no Isaac app needed: right_env only plans with pinocchio here)
"""

import argparse
import json
import math

import numpy as np
import torch
from pxr import Gf, Usd, UsdGeom

from bee_isaac import common as bc
from bee_isaac.tasks import pick_env as pk
from bee_isaac.tasks import right_env as re_

parser = argparse.ArgumentParser()
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--out", default="/tmp/stand_plan.json")
args = parser.parse_args()
rng = np.random.default_rng(args.seed)

DT = 0.02  # sample period of the streamed trajectory (policy rate in training)
LYING = "puck_box_green_6"  # the lying box puck on start_1

stage = Usd.Stage.Open(str(bc.USD_DIR / "scene_lab.usda"))  # read only; the user places robot and pucks there
xf = lambda p: Gf.Transform(UsdGeom.Xformable(stage.GetPrimAtPath(p)).ComputeLocalToWorldTransform(0))
yaw_of = lambda r, v: math.atan2(r.TransformDir(v)[1], r.TransformDir(v)[0])
rt = xf("/World/beebot")
robot = (rt.GetTranslation()[0], rt.GetTranslation()[1], yaw_of(rt.GetRotation(), Gf.Vec3d(1, 0, 0)))
pt = xf(f"/World/pucks/{LYING}")
axis = pt.GetRotation().TransformDir(Gf.Vec3d(0, 0, 1))  # puck axis (root = near-end bottom face -> far end)
assert abs(axis[2]) < 0.2, f"{LYING} is not lying (axis z {axis[2]:.2f})"
centre_w = pt.GetTranslation() + axis * (re_.PUCK_L / 2)

# puck centre and axis yaw in the robot root frame (true), then perceived
c, s = math.cos(robot[2]), math.sin(robot[2])
dx, dy = centre_w[0] - robot[0], centre_w[1] - robot[1]
true_xy = np.array([c * dx + s * dy, -s * dx + c * dy])
true_yaw = math.remainder(math.atan2(axis[1], axis[0]) - robot[2], 2 * math.pi)
bt = xf("/World/start_boxes/start_1").GetTranslation()
drive = c * (bt[0] - robot[0]) + s * (bt[1] - robot[1]) - (0.08 + bc.BOX_HALF)  # straight ahead to the dock
true_xy[0] -= drive
cfg = re_.RightEnvCfg()
cfg.regrasp = 1
meas_xy = true_xy + rng.uniform(-cfg.pos_noise_m, cfg.pos_noise_m, 2)
meas_yaw = true_yaw + rng.uniform(-cfg.yaw_noise, cfg.yaw_noise)

ik = bc.ArmIK()
centre = np.array([meas_xy[0], meas_xy[1], bc.BOX_H]) - ik.base_in_root()
qs, _ = re_.plan_waypoints(ik, centre, meas_yaw, cfg, is_box=True)
wp, (q_grasp, q_lift) = np.array(qs[:-2]), qs[-2:]

dur = lambda qa, qb: bc.trapezoid_duration(float(np.abs(np.asarray(qb) - np.asarray(qa)).max()), cfg.vel_scale)


def move(qa, qb, t_move, hold=0.0):
    """Smooth-step samples qa -> qb (as the training reference), then hold."""
    n = max(1, int(math.ceil(t_move / DT)))
    out = [np.asarray(qa) + bc.move_s((k + 1) * DT, t_move) * (np.asarray(qb) - np.asarray(qa)) for k in range(n)]
    return out + [np.asarray(qb)] * int(round(hold / DT))


stow = pk.STOW.numpy()
approach = move(stow, wp[0], dur(stow, wp[0]), cfg.hold_s) + move(wp[0], wp[1], dur(wp[0], wp[1]))
w = torch.tensor(wp, dtype=torch.float32)[None]
groups = re_.arc_groups(cfg, len(wp) - 3)
arc_dur = re_.arc_durations(w, groups, cfg.arc_vel_scale or cfg.vel_scale, cfg.hang_s)
t_arc = float(arc_dur.sum())
arc = [re_.arc_q(w, groups, arc_dur, torch.tensor([min((k + 1) * DT, t_arc)]))[0].numpy() for k in range(int(math.ceil(t_arc / DT)))]

plan = {
    "dt": DT,
    "approach": np.array(approach).tolist(),  # stow -> above -> hold -> down to the pinch pose
    "arc": np.array(arc).tolist(),            # pinch pose -> arc -> set-down pose (ends at wp[-2])
    "q_set": wp[-2].tolist(),
    "q_grasp": np.asarray(q_grasp).tolist(),  # regrasp: same wrist, straight down to the pick grasp depth
    "q_lift": np.asarray(q_lift).tolist(),
    "t_grasp": dur(wp[-2], q_grasp), "t_lift": dur(q_grasp, q_lift),
    "hold_s": cfg.hold_s, "close_s": cfg.close_s, "settle_s": cfg.settle_s, "open_s": cfg.open_s, "regrasp_close_s": 0.5,
    "open_m": bc.gap_joint(40.0), "grip_stop_m": bc.GRIP_STOP,
    "pinch_force_n": cfg.pinch_force, "grip_force_n": bc.GRIP_FORCE,
    "robot": robot, "drive_m": drive,  # drive straight this far first (dock); pucks as placed in scene_lab.usda
    "puck_true": [*true_xy.tolist(), math.degrees(true_yaw)], "puck_meas": [*meas_xy.tolist(), math.degrees(meas_yaw)],
}
with open(args.out, "w") as f:
    json.dump(plan, f)
print(f"[stand_plan] drive {1000 * drive:.0f} mm to the dock; puck true {np.round(1000 * true_xy).tolist()} mm {math.degrees(true_yaw):.1f} deg, "
      f"perceived {np.round(1000 * meas_xy).tolist()} mm {math.degrees(meas_yaw):.1f} deg; "
      f"{len(approach)} + {len(arc)} samples -> {args.out}")
