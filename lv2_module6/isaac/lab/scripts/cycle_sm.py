"""Scripted pick -> basket cycles on Bee-Puck-Load: one puck per env (cycle timing / route comparison) or several
pucks loaded one after another (sequential loading).

    isaaclab.sh -p cycle_sm.py --num_envs 192 --latency                 (one puck per env, env i -> slot i % 12)
    isaaclab.sh -p cycle_sm.py --num_envs 64 --latency --pucks 5        (5 pucks on the shelf grid, loaded in turn)
Defaults are the setup chosen on 2026-10-28 (report logs/20261027_sim_tests, 11-13): carry pose at 90 deg (beside the
robot), no pauses at transit waypoints, let go 3 mm above the slot floor, fill order with slots 2, 8, 9 last.
    add --viz newton to watch (window stays open at the end)

Cycle = approach above the puck -> descend -> close -> lift -> [loaded leg] -> over the slot -> lower onto the slot
floor -> release -> lift out -> [empty leg] -> approach above the next puck. Moves are joint-space, timed like MoveIt
with the team joint limits (3 rad/s, 5 rad/s^2) x --vel_scale, trapezoidal and synchronized over the joints.

Routes (legs between the shelf and the basket):
    direct  lift -> over slot;          lift out -> raise -> next approach
    carry   lift -> carry -> over slot;  lift out -> carry -> next approach   (carry: fixed pose behind the arm,
            gripper down, one per basket side; carry <-> slot can be planned once at boot)
    home    lift -> home -> over slot;   lift out -> raise -> fold -> home -> next approach

--transit_hold is the pause at the transit waypoints (lift, carry, home, over slot, lift out, raise): 0 = continuous
(no settling between legs), > 0 = stop-and-go. The sag integral only runs while a joint target is held.
"""

import argparse
import math
from collections import deque

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=12)
parser.add_argument("--seed", type=int, default=None)
parser.add_argument("--route", choices=["direct", "carry", "home"], default="carry")
parser.add_argument("--pucks", type=int, default=1, help="pucks per env: 1 = single cycle, up to 5 = target + 4 grid neighbours")
parser.add_argument("--gap_mm", type=float, default=50.0, help="shelf grid gap (centre to centre) when --pucks > 1")
parser.add_argument("--order", choices=["back_first", "front_first", "good_first"], default="good_first",
                    help="basket fill order for --pucks > 1: pole-side row first, arm-side row first, or back to front with the "
                         "slots that fail in the single-puck test (2, 8, 9) last")
parser.add_argument("--carry_deg", type=float, default=90.0, help="carry pose heading from the front [deg], mirrored per basket side")
parser.add_argument("--vel_scale", type=float, default=0.3, help="x team joint limits (3 rad/s, 5 rad/s^2)")
parser.add_argument("--transit_hold", type=float, default=0.0, help="pause [s] at transit waypoints")
parser.add_argument("--settle", type=float, default=0.3, help="pause [s] after descend and lower (accuracy points)")
parser.add_argument("--hz", type=float, default=100.0)
parser.add_argument("--latency", action="store_true")
parser.add_argument("--open_gap_mm", type=float, default=40.0)
parser.add_argument("--lower_mm", type=float, default=3.0, help="puck bottom above the slot floor when letting go [mm]")
parser.add_argument("--release_gap_mm", type=float, default=None, help="finger gap when letting go (default = --open_gap_mm)")
parser.add_argument("--mirror_wrist", action="store_true", help="tool yaw heading - 90 deg on the -y side of the basket (mirror)")
parser.add_argument("--no_correct", action="store_true", help="do not aim the puck bottom (measured in the hand) at the slot")
parser.add_argument("--no_diag", action="store_true", help="cylinders: keep the face rule instead of the grid diagonal")
parser.add_argument("--out", default=None)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
simulation_app = AppLauncher(args).app

import gymnasium as gym
import numpy as np
import pinocchio as pin
import torch

import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation, RigidObject
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

import bee_env_cfg
from pick_lib import ArmIK

PUCK_H = 0.06
FINGER_TIP = 0.0206
GRIP_DEPTH = 0.020
GRASP_Z = PUCK_H - GRIP_DEPTH + FINGER_TIP  # end effector above the puck bottom when grasped
APPROACH_Z, LIFT_Z = GRASP_Z + 0.05, GRASP_Z + 0.05
DOWN = pin.Quaternion(np.array([0.0, 1.0, 0.0, 0.0])).toRotationMatrix()  # end effector z pointing down
OPEN = (0.0083 + (args.open_gap_mm / 1000 - 0.030) / 2) / 0.0265  # finger gap 30 mm + 2 x (joint - 8.3 mm)
CLOSE = 0.0
RELEASE = OPEN if args.release_gap_mm is None else (0.0083 + (args.release_gap_mm / 1000 - 0.030) / 2) / 0.0265
CLOSE_S, RELEASE_S = 0.5, 0.4  # gripper dwell
CORR_GAIN, CORR_DT, CORR_MAX = 0.3, 5 / 60.0, 0.2
POS_NOISE, YAW_NOISE = 0.005, math.radians(10.0)
HELD_DIST = 0.05
SLOTS = [(x, y, 0.09125) for x in (-0.16984, -0.10484, -0.03984) for y in (-0.0975, -0.0325, 0.0325, 0.0975)]
JOINT1_XY = (0.047985, -0.00023)
# per row (0 = pole side): over-slot height of the puck bottom [m] and gripper pitch [deg] (2026-10-27 report, 7 / 10)
OVER = {0: (0.05, -20.0), 1: (0.045, 0.0), 2: (0.06, 0.0)}
SET_TILTS = (0.0, -5.0, -10.0, -15.0)
LIFT_UPS = (0.06, 0.05, 0.045, 0.04, 0.035)
RAISE_R, RAISE_Z = 0.09, 0.38
WRIST = math.pi / 2  # tool yaw = heading + 90 deg (joint5 -90): fingers close along the arm; needed on the pole side
CARRY_HEADING = math.radians(args.carry_deg)  # carry pose heading, on the slot's side (150 = over the arm-side row)
CARRY_R = 0.08
IN_SLOT_XY, IN_SLOT_Z, IN_SLOT_TILT = 0.02, 0.015, math.radians(20.0)
LATENCY_S = (0.045, 0.062, 0.052, 0.045, 0.045, 0.020)
SLOT_ORDER = {"back_first": [1, 2, 0, 3, 5, 6, 4, 7, 9, 10, 8, 11], "front_first": [9, 10, 8, 11, 5, 6, 4, 7, 1, 2, 0, 3],
              # single-puck placement per slot (2026-10-28, 16 envs each): 2 -> 25 %, 8 -> 12 %, 9 -> 6 %, the rest 75-100 %
              "good_first": [0, 1, 3, 4, 5, 6, 7, 10, 11, 2, 8, 9]}


def rot_tool(heading, pitch_deg, yaw):
    """Gripper down, pitched about the arm's sideways axis, tool yaw about its own axis."""
    return pin.utils.rotate("z", heading) @ pin.utils.rotate("y", math.radians(pitch_deg)) @ DOWN @ pin.utils.rotate("z", -(yaw - heading))


def main():
    cfg = parse_env_cfg("Bee-Puck-Load", num_envs=args.num_envs)
    cfg.seed = args.seed
    cfg.sim.dt = 1.0 / (cfg.decimation * args.hz)
    cfg.episode_length_s = 600.0
    nb_names = []
    if args.pucks > 1:
        for k in range(args.pucks - 1):
            name = f"nb{k}"
            setattr(cfg.scene, name, cfg.scene.puck.replace(prim_path="{ENV_REGEX_NS}/" + name,
                    spawn=sim_utils.UsdFileCfg(usd_path=bee_env_cfg.PUCK_USDS[k % 4]),
                    init_state=cfg.scene.puck.init_state.replace(pos=(1.0, 0.2 * k, 0.0))))
            nb_names.append(name)
    env = gym.make("Bee-Puck-Load", cfg=cfg)
    env.reset()
    u: ManagerBasedRLEnv = env.unwrapped
    robot: Articulation = u.scene["robot"]
    pucks: list[RigidObject] = [u.scene["puck"]] + [u.scene[nm] for nm in nb_names]
    n, dev = u.num_envs, u.device
    org = u.scene.env_origins
    step_dt = cfg.sim.dt * cfg.decimation
    arm_ids = robot.find_joints("joint[1-5]", preserve_order=True)[0]
    default_arm = robot.data.default_joint_pos.torch[:, arm_ids].clone()
    ee_id = robot.find_bodies("gripper_link")[0][0]
    base_id = robot.find_bodies("base_link")[0][0]
    ee_offset = torch.tensor(bee_env_cfg.EE_OFFSET_POS, device=dev).repeat(n, 1)
    ee_rot = torch.tensor(bee_env_cfg.EE_OFFSET_ROT, device=dev).repeat(n, 1)
    up_half = torch.tensor([[0.0, 0.0, PUCK_H / 2]], device=dev).repeat(n, 1)
    z_axis = torch.tensor([[0.0, 0.0, 1.0]], device=dev).repeat(n, 1)
    kinds = []
    for p_idx in range(len(pucks)):
        prim = "puck" if p_idx == 0 else nb_names[p_idx - 1]
        kinds.append([u.sim.stage.GetPrimAtPath(f"/World/envs/env_{i}/{prim}").GetPrimStack()[1].layer.identifier.split("/")[-1][:-5]
                      for i in range(n)])

    def base_pose():
        return robot.data.body_pos_w.torch[:, base_id], robot.data.body_quat_w.torch[:, base_id]

    def to_root(pos_w):
        bp, bq = base_pose()
        return math_utils.quat_apply_inverse(bq, pos_w - bp)

    def ee_pose_b():
        g_pos, g_quat = robot.data.body_pos_w.torch[:, ee_id], robot.data.body_quat_w.torch[:, ee_id]
        _, bq = base_pose()
        return to_root(g_pos + math_utils.quat_apply(g_quat, ee_offset)), math_utils.quat_mul(math_utils.quat_inv(bq), math_utils.quat_mul(g_quat, ee_rot))

    def puck_tilt(p):
        _, bq = base_pose()
        q_b = math_utils.quat_mul(math_utils.quat_inv(bq), p.data.root_quat_w.torch)
        return torch.acos(math_utils.quat_apply(q_b, z_axis)[:, 2].clamp(-1, 1))

    # ---- actions with optional latency
    def action(q_cmd, grip):
        return torch.cat([q_cmd - default_arm, grip[:, None].repeat(1, 2)], dim=1)

    lag = [round(s / step_dt) if args.latency else 0 for s in LATENCY_S]
    lag = lag + [lag[-1]]
    hist = deque([action(default_arm, torch.full((n,), OPEN, device=dev))] * (max(lag) + 1), maxlen=max(lag) + 1)

    def step(a):
        hist.append(a)
        env.step(torch.stack([hist[-1 - d][:, j] for j, d in enumerate(lag)], dim=1))

    # ---- shelf grid of pucks (target = puck 0 at the env's random reset pose, the rest around it)
    grip_now = torch.full((n,), OPEN, device=dev)
    for k in range(int(0.5 / step_dt)):
        step(action(default_arm, grip_now))
        if k == 0 and len(pucks) > 1:
            gap = args.gap_mm / 1000
            tgt = pucks[0].data.root_pos_w.torch - org
            lo_x, hi_x = bee_env_cfg.SHELF_NEAR_X + 0.02, bee_env_cfg.SHELF_NEAR_X + 2 * bee_env_cfg.SHELF_HALF - 0.02
            hi_y = bee_env_cfg.SHELF_HALF - 0.02
            for k2, (p, (dx, dy)) in enumerate(zip(pucks[1:], [(gap, 0), (-gap, 0), (0, gap), (0, -gap)])):
                pos = tgt.clone()
                pos[:, 0] += dx
                pos[:, 1] += dy
                pos[:, 2] = bee_env_cfg.SHELF_TOP + 0.002
                off = (pos[:, 0] < lo_x) | (pos[:, 0] > hi_x) | (pos[:, 1].abs() > hi_y)
                pos[off] = torch.tensor([1.0, 0.2 * k2, 0.0], device=dev)
                yaw_nb = (torch.rand(n, device=dev) * 2 - 1) * math.pi
                q = math_utils.quat_from_euler_xyz(torch.zeros(n, device=dev), torch.zeros(n, device=dev), yaw_nb)
                p.write_root_pose_to_sim_index(root_pose=torch.cat([pos + org, q], dim=1))
                p.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev))
    on_shelf = torch.stack([((p.data.root_pos_w.torch - org)[:, 2] > bee_env_cfg.SHELF_TOP - 0.01) for p in pucks], dim=1)  # (n, P)

    # ---- motion engine: synchronized trapezoid in joint space, per-env duration, sag integral while holding
    vmax, amax = 3.0 * args.vel_scale, 5.0 * args.vel_scale
    corr_every = max(1, round(CORR_DT / step_dt))
    st = {"q_prev": default_arm.clone(), "corr": torch.zeros(n, 5, device=dev), "grip": grip_now.clone()}
    timeline = []  # (label, per-env seconds) for the cycle time breakdown

    def move(q_end, grip, hold, label, active):
        """Move active envs to q_end (inactive envs keep their pose); returns the largest joint error while moving."""
        q0 = st["q_prev"]
        q_end = torch.where(active[:, None], q_end, q0)
        dist = (q_end - q0).abs().max(dim=1).values
        t_acc = vmax / amax
        tri = dist <= vmax * t_acc
        dur = torch.where(tri, 2 * torch.sqrt(dist / amax), dist / vmax + t_acc).clamp(min=0.1)
        dur = torch.where(active, dur, torch.zeros_like(dur))
        hold_t = torch.where(active, torch.full_like(dur, hold), torch.zeros_like(dur))
        total = int(math.ceil((dur + hold_t).max().item() / step_dt)) + 1
        g = torch.where(active, torch.full_like(dur, grip), st["grip"])
        worst = torch.zeros(n, device=dev)
        for k in range(total):
            t = (k + 1) * step_dt
            # normalized trapezoid / triangle position along the move, 0..1
            a_ = torch.where(tri, 4 * dist / dur.clamp(min=1e-6) ** 2, torch.full_like(dist, amax))
            ta = torch.where(tri, dur / 2, torch.full_like(dist, t_acc))
            v_ = a_ * ta
            tt = torch.minimum(torch.full_like(dur, t), dur)
            pos = torch.where(tt < ta, 0.5 * a_ * tt ** 2,
                  torch.where(tt < dur - ta, 0.5 * a_ * ta ** 2 + v_ * (tt - ta), dist - 0.5 * a_ * (dur - tt) ** 2))
            s = torch.where(dist > 1e-6, (pos / dist.clamp(min=1e-6)).clamp(0, 1), torch.ones_like(dist))
            q_ref = q0 + s[:, None] * (q_end - q0)
            step(action(q_ref + st["corr"], g))
            q_err = q_ref - robot.data.joint_pos.torch[:, arm_ids]
            worst = torch.maximum(worst, torch.where(active, q_err.abs().max(dim=1).values, torch.zeros_like(dist)))
            holding = active & (t >= dur)
            if k % corr_every == corr_every - 1:
                st["corr"] = torch.where(holding[:, None], (st["corr"] + CORR_GAIN * q_err).clamp(-CORR_MAX, CORR_MAX), st["corr"])
        st["q_prev"] = q_end
        st["grip"] = g
        timeline.append((label, (dur + hold_t).cpu().numpy()))
        return worst

    def dwell(grip, secs, label, active):
        return move(st["q_prev"], grip, secs, label, active)

    # ---- fixed poses: carry (one per basket side), solved once like a boot-time plan
    ik = ArmIK()
    carry_q = {}
    for side in (1, -1):
        hd = side * CARRY_HEADING
        wr = -WRIST if (args.mirror_wrist and side < 0) else WRIST
        rot = rot_tool(hd, 0.0, hd + wr)
        best = None
        for z in np.arange(0.40, 0.20, -0.01):
            p = np.array([JOINT1_XY[0] + CARRY_R * math.cos(hd), JOINT1_XY[1] + CARRY_R * math.sin(hd), z])
            q, e_pos, e_ang = ik.solve(p, rot, [np.array([hd, j2, j3, 1.5, -wr]) for j2 in (-0.5, 0.0, 0.5) for j3 in (0.5, -0.5)])
            if e_pos < 2e-3 and e_ang < math.radians(2.0):
                best = (q, z)
                break
        carry_q[side] = torch.tensor(best[0] if best else q, dtype=torch.float32, device=dev)
        print(f"[carry] side {side:+d}: end effector z {best[1] if best else float('nan'):.2f} m, joints deg {np.degrees(best[0] if best else q).round(0).tolist()}")

    # ---- one cycle for puck index p_idx in every active env
    n_p = len(pucks)
    order = SLOT_ORDER[args.order]
    slot_of = torch.zeros(n, n_p, dtype=torch.long, device=dev)
    if n_p == 1:
        slot_of[:, 0] = torch.arange(n, device=dev) % len(SLOTS)
    else:
        rank = torch.cumsum(on_shelf.long(), dim=1) - 1  # k-th puck on the shelf goes to the k-th slot in the order
        slot_of = torch.tensor(order, device=dev)[rank.clamp(min=0)]
    records = {k: torch.zeros(n, n_p, device=dev) for k in ("picked", "placed", "blocked", "t_cycle", "tilt", "stuck")}
    for v in records.values():
        v.fill_(float("nan"))
    # pucks of this env already in the basket: (n, P) slot index or -1
    in_basket_slot = torch.full((n, n_p), -1, dtype=torch.long, device=dev)

    for p_idx in range(n_p):
        P = pucks[p_idx]
        active = on_shelf[:, p_idx].clone()
        if not active.any():
            continue
        t0 = len(timeline)
        # perception: puck pose with noise, yaw rule (box: face closest to heading, cylinder: grid diagonal)
        pb = to_root(P.data.root_pos_w.torch).clone()
        pb[:, :2] += (torch.rand(n, 2, device=dev) * 2 - 1) * POS_NOISE
        _, bq = base_pose()
        yaw = math_utils.euler_xyz_from_quat(math_utils.quat_mul(math_utils.quat_inv(bq), P.data.root_quat_w.torch))[2]
        yaw = yaw + (torch.rand(n, device=dev) * 2 - 1) * YAW_NOISE
        heading = torch.atan2(pb[:, 1], pb[:, 0])
        yaw = heading + torch.remainder(yaw - heading + math.pi / 4, math.pi / 2) - math.pi / 4
        if not args.no_diag:
            cyl = torch.tensor(["cyl" in kinds[p_idx][i] for i in range(n)], device=dev)
            diag = heading + torch.remainder(math.pi / 4 - heading + math.pi / 4, math.pi / 2) - math.pi / 4
            yaw = torch.where(cyl, diag, yaw)
        # pick IK
        qs = {k: st["q_prev"].clone() for k in ("approach", "descend", "lift")}
        pb_np, yw, hd = pb.cpu().numpy(), yaw.cpu().numpy(), heading.cpu().numpy()
        for i in range(n):
            if not active[i]:
                continue
            rot = pin.utils.rotate("z", float(yw[i])) @ DOWN
            seeds = [np.array([hd[i], -0.2, 0.7, 1.0, s]) for s in (0.0, yw[i] - hd[i], hd[i] - yw[i])]
            for name, dz in (("approach", APPROACH_Z), ("descend", GRASP_Z), ("lift", LIFT_Z)):
                q, *_ = ik.solve(pb_np[i] + np.array([0.0, 0.0, dz]), rot, seeds)
                qs[name][i] = torch.from_numpy(q).float()
                seeds = [q]
        z0 = P.data.root_pos_w.torch[:, 2].clone()
        b1 = move(qs["approach"], OPEN, 0.0, "approach", active)
        b2 = move(qs["descend"], OPEN, args.settle, "descend", active)
        dwell(CLOSE, CLOSE_S, "close", active)
        b3 = move(qs["lift"], CLOSE, args.transit_hold, "lift", active)
        lifted = (P.data.root_pos_w.torch[:, 2] - z0) > 0.03

        # basket targets; puck bottom in the end effector frame measured now (wrist camera on the robot)
        slot = slot_of[:, p_idx]
        e_p, e_q = ee_pose_b()
        d_ee = math_utils.quat_apply_inverse(e_q, to_root(P.data.root_pos_w.torch) - e_p)
        if args.no_correct:
            d_ee = torch.tensor([[0.0, 0.0, GRASP_Z]], device=dev).repeat(n, 1)
        d_np = d_ee.cpu().numpy()
        bk = {k: st["q_prev"].clone() for k in ("over", "lower", "lift_out", "raise")}
        for i in range(n):
            if not active[i]:
                continue
            sx, sy, sz = SLOTS[slot[i].item()]
            row = slot[i].item() // 4
            hd_s = math.atan2(sy - JOINT1_XY[1], sx - JOINT1_XY[0])
            wr = -WRIST if (args.mirror_wrist and sy < 0) else WRIST
            yaw_s = hd_s + wr
            h_over, t_over = OVER[row]
            seeds = [np.array([hd_s, j2, j3, j4, -wr]) for j2 in (-0.3, 0.3, 0.8) for j3 in (0.7, -0.5) for j4 in (1.0, 1.9)]
            rot = rot_tool(hd_s, t_over, yaw_s)
            q_over, *_ = ik.solve(np.array([sx, sy, sz + h_over]) - rot @ d_np[i], rot, seeds)
            for t_ in SET_TILTS:
                rot = rot_tool(hd_s, t_, yaw_s)
                q_low, e_pos, e_ang = ik.solve(np.array([sx, sy, sz + args.lower_mm / 1000]) - rot @ d_np[i], rot, [q_over])
                if e_pos < 2e-3 and e_ang < math.radians(2.0):
                    break
            q_lift = q_low
            for up_ in LIFT_UPS:
                q_try, e_pos, e_ang = ik.solve(np.array([sx, sy, sz + up_]) - rot @ d_np[i], rot, [q_low])
                if e_pos < 2e-3 and e_ang < math.radians(2.0):
                    q_lift = q_try
                    break
            p_r = np.array([JOINT1_XY[0] + RAISE_R * math.cos(hd_s), JOINT1_XY[1] + RAISE_R * math.sin(hd_s), RAISE_Z])
            q_raise, *_ = ik.solve(p_r, rot, [q_lift], w_ang=0.05)
            for k_, q_ in (("over", q_over), ("lower", q_low), ("lift_out", q_lift), ("raise", q_raise)):
                bk[k_][i] = torch.from_numpy(q_).float()
        side = torch.where(torch.tensor(SLOTS, device=dev)[slot][:, 1] >= 0, 1, -1)
        q_carry = torch.where((side > 0)[:, None], carry_q[1], carry_q[-1])

        blocked = torch.zeros(n, device=dev)
        # loaded leg
        if args.route == "carry":
            blocked = torch.maximum(blocked, move(q_carry, CLOSE, args.transit_hold, "carry", active))
        elif args.route == "home":
            blocked = torch.maximum(blocked, move(default_arm, CLOSE, args.transit_hold, "home", active))
        blocked = torch.maximum(blocked, move(bk["over"], CLOSE, args.transit_hold, "over_slot", active))
        held = (to_root(P.data.root_pos_w.torch + math_utils.quat_apply(P.data.root_quat_w.torch, up_half)) - ee_pose_b()[0]).norm(dim=1) < HELD_DIST
        blocked = torch.maximum(blocked, move(bk["lower"], CLOSE, args.settle, "lower", active))
        dwell(RELEASE, RELEASE_S, "release", active)
        blocked = torch.maximum(blocked, move(bk["lift_out"], RELEASE, args.transit_hold, "lift_out", active))
        # still in the hand after letting go and lifting out = stuck between the fingers (it then rides along)
        stuck = (to_root(P.data.root_pos_w.torch + math_utils.quat_apply(P.data.root_quat_w.torch, up_half)) - ee_pose_b()[0]).norm(dim=1) < HELD_DIST
        # empty leg towards the next puck (single puck: back to the same approach pose)
        if args.route == "carry":
            blocked = torch.maximum(blocked, move(q_carry, OPEN, args.transit_hold, "carry_back", active))
        elif args.route == "home":
            blocked = torch.maximum(blocked, move(bk["raise"], OPEN, args.transit_hold, "raise", active))
            q_fold = default_arm.clone()
            q_fold[:, 0] = bk["raise"][:, 0]
            blocked = torch.maximum(blocked, move(q_fold, OPEN, args.transit_hold, "fold", active))
            blocked = torch.maximum(blocked, move(default_arm, OPEN, args.transit_hold, "home_back", active))
        else:
            blocked = torch.maximum(blocked, move(bk["raise"], OPEN, args.transit_hold, "raise", active))
        nxt = qs["approach"]  # proxy for the next puck's approach pose
        blocked = torch.maximum(blocked, move(nxt, OPEN, 0.0, "to_next", active))

        # results for this puck
        sl = torch.tensor(SLOTS, device=dev)[slot]
        pbn = to_root(P.data.root_pos_w.torch)
        tilt = puck_tilt(P)
        in_slot = ((pbn[:, :2] - sl[:, :2]).abs().max(dim=1).values < IN_SLOT_XY) & ((pbn[:, 2] - sl[:, 2]).abs() < IN_SLOT_Z) & (tilt < IN_SLOT_TILT)
        # cycle time = from above the puck (approach reached) to above the next puck
        t_cyc = sum(v for lab, v in timeline[t0 + 1:])
        a = active
        records["picked"][:, p_idx] = torch.where(a, (lifted & held).float(), records["picked"][:, p_idx])
        records["placed"][:, p_idx] = torch.where(a, in_slot.float(), records["placed"][:, p_idx])
        records["blocked"][:, p_idx] = torch.where(a, blocked, records["blocked"][:, p_idx])
        records["t_cycle"][:, p_idx] = torch.where(a, torch.tensor(t_cyc, device=dev, dtype=torch.float32), records["t_cycle"][:, p_idx])
        records["tilt"][:, p_idx] = torch.where(a, tilt, records["tilt"][:, p_idx])
        records["stuck"][:, p_idx] = torch.where(a, stuck.float(), records["stuck"][:, p_idx])
        in_basket_slot[:, p_idx] = torch.where(a, slot, in_basket_slot[:, p_idx])
        print(f"[cycle {p_idx}] envs {a.sum().item()}: picked {(lifted & held & a).sum().item()}, placed {(in_slot & a).sum().item()}, "
              f"blocked>10deg {((blocked > math.radians(10)) & a).sum().item()}, stuck in hand {(stuck & a).sum().item()}, cycle s median {np.median(t_cyc[a.cpu().numpy()]):.1f}")

    # pucks still upright in their slots at the very end (later loads may knock earlier ones)
    final_ok = torch.zeros(n, n_p, device=dev)
    for p_idx, P in enumerate(pucks):
        sl = torch.tensor(SLOTS, device=dev)[in_basket_slot[:, p_idx].clamp(min=0)]
        pbn = to_root(P.data.root_pos_w.torch)
        ok_ = ((pbn[:, :2] - sl[:, :2]).abs().max(dim=1).values < IN_SLOT_XY) & ((pbn[:, 2] - sl[:, 2]).abs() < IN_SLOT_Z) & (puck_tilt(P) < IN_SLOT_TILT)
        final_ok[:, p_idx] = torch.where(in_basket_slot[:, p_idx] >= 0, ok_.float(), torch.full_like(ok_, float("nan"), dtype=torch.float32))
    placed = records["placed"]
    print(f"[result] route {args.route}, hold {args.transit_hold}s, pucks {n_p}: picked {np.nansum(records['picked'].cpu().numpy()):.0f}/"
          f"{on_shelf.sum().item()}, placed {np.nansum(placed.cpu().numpy()):.0f}, still in slot at the end {np.nansum(final_ok.cpu().numpy()):.0f}")
    t_np = records["t_cycle"].cpu().numpy()
    print(f"[result] cycle time s: median {np.nanmedian(t_np):.2f}, p90 {np.nanpercentile(t_np, 90):.2f}")
    labels = [lab for lab, _ in timeline]
    if args.out:
        out = {k: v.cpu().numpy() for k, v in records.items()}
        out |= {"final_ok": final_ok.cpu().numpy(), "on_shelf": on_shelf.cpu().numpy(), "slot_of": slot_of.cpu().numpy(),
                "kinds": np.array(kinds), "timeline_labels": np.array(labels), "timeline_s": np.stack([v for _, v in timeline]),
                "args": str({k: v for k, v in vars(args).items() if k != "out"})}
        np.savez_compressed(args.out, **out)
        print(f"[out] {args.out}")

    if getattr(args, "visualizer", None):
        print("[hold] holding last pose, close the window or press Ctrl+C to exit")
        try:
            while simulation_app.is_running():
                step(action(st["q_prev"] + st["corr"], st["grip"]))
        except KeyboardInterrupt:
            pass
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
