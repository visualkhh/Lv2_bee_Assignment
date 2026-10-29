"""Scripted pick on Bee-Puck-Load (joint position actions): home -> approach -> descend -> close -> lift -> home.

    isaaclab.sh -p pick_sm.py --num_envs 4 --viz newton   (stays open until the window is closed)
    isaaclab.sh -p pick_sm.py --num_envs 256           (success rate under randomization)
    isaaclab.sh -p pick_sm.py --num_envs 256 --latency (same, with the real robot's command latency)

Test knobs (all off by default, defaults = the plain run above):
    --seed 1 --out run.npz      reproducible run, per-env results (+ --trace: per-step puck / base / arm)
    --grip_force 38 --torsion 5 gripper squeeze per finger [N] at a 30 mm puck, torsional patch radius [mm]
    --hz 60                     control rate (sim dt = 1 / (2 hz)); default 100
    --lat_jitter_ms 20          per-env extra command delay, uniform +-, on top of --latency
    --neighbors 50              4 more pucks around the target (shelf grid, centre-to-centre gap [mm])
    --park_err_mm 20            puck window widened by +- this (robot stopping error in front of the shelf)
    --pull down|out|twist       after home: ramp an external force / torque on the puck until it slips
    --open_gap_mm 40            open the fingers only this wide (real gripper about 38-40 mm, sim limit 66 mm)
    --diag                      cylinders: fingers close along the grid diagonal (neighbours 1.41 x farther)
    --box_clear                 boxes: pick the face pair whose finger axis points at the most room (needs --neighbors)
    --basket                    after home: put the puck into a basket slot on the robot (env i -> slot i % 12,
                                wrist option (i // 12) % 4 = target yaw + 0/90/180/270 deg), then home again

Puck pose, mass and friction are randomized by the env (EventCfg). The script sees the puck pose with
perception-like noise, solves joint targets offline with pinocchio (URDF = same kinematics as the USD),
and ramps between them in joint space, like a MoveIt joint trajectory on the real arm.

Why not the Lab differential IK action: with the gripper pointing down, joint1 and joint5 both turn about
a vertical axis. The one-step DLS solver splits the error between them and wanders, so pucks off the
centre line or with a yaw were missed by 5-15 cm.
"""

import argparse
import math
from collections import deque

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--latency", action="store_true", help="delay commands by the measured real-robot lag")
parser.add_argument("--seed", type=int, default=None)
parser.add_argument("--out", default=None, help="save per-env results (.npz)")
parser.add_argument("--trace", action="store_true", help="with --out: also save per-step puck / base / arm state")
parser.add_argument("--grip_force", type=float, default=None, help="squeeze per finger [N] at a 30 mm puck")
parser.add_argument("--grip_damping", type=float, default=None, help="gripper drive damping [N s/m] (cfg 10)")
parser.add_argument("--torsion", type=float, default=None, help="torsional patch radius [mm] of the robot colliders")
parser.add_argument("--hz", type=float, default=100.0, help="control rate [Hz]")
parser.add_argument("--lat_jitter_ms", type=float, default=0.0)
parser.add_argument("--neighbors", type=float, default=None, help="centre-to-centre gap [mm] of 4 neighbour pucks")
parser.add_argument("--park_err_mm", type=float, default=0.0)
parser.add_argument("--pull", choices=["down", "out", "twist"], default=None)
parser.add_argument("--box_clear", action="store_true",
                    help="boxes: of the two face pairs, grasp the one whose finger axis has the most room to the neighbours")
parser.add_argument("--diag", action="store_true", help="cylinders: close the fingers along the shelf grid diagonal")
parser.add_argument("--basket", action="store_true", help="put the puck into the robot's basket after the pick")
parser.add_argument("--basket_skip_home", action="store_true",
                    help="with --basket: go from lift straight to the basket, without the home pose (puck stays hanging down)")
parser.add_argument("--basket_correct", action="store_true",
                    help="measure the puck in the hand once before the basket (wrist camera on the robot) and aim its bottom at the slot")
parser.add_argument("--basket_retreat", choices=["fold", "direct"], default="fold", help="how the empty hand leaves the basket")
parser.add_argument("--open_gap_mm", type=float, default=None, help="finger gap when open (default 66 mm = joint limit)")
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
FINGER_TIP = 0.0206  # fingertips reach this far past end_effector_link (finger mesh in the URDF)
GRIP_DEPTH = 0.020  # how much of the puck top the fingers cover
GRASP_Z = PUCK_H - GRIP_DEPTH + FINGER_TIP  # end_effector_link above puck origin (bottom center)
APPROACH_Z = GRASP_Z + 0.05
LIFT_Z = GRASP_Z + 0.05
DOWN_QUAT = (0.0, 1.0, 0.0, 0.0)  # xyzw, end effector z pointing down
OPEN, CLOSE = 1.0, 0.0  # gripper action x 0.0265 m (JointPositionAction scale)
# Sim finger inner gap = 30 mm + 2 x (joint - 8.3 mm): 13 mm closed, 66 mm at OPEN = 1.0. The real gripper
# opens only to about 38-40 mm (gap = 40.1 - 1.22 x value, R2S 2026-10-24), so --open_gap_mm 40 matches it.
if args.open_gap_mm is not None:
    OPEN = (0.0083 + (args.open_gap_mm / 1000 - 0.030) / 2) / 0.0265

# Low joint stiffness (0.42 Nm/deg = 24 Nm/rad, matches the real servo with the measured KT 2.2 Nm/A) leaves the arm sagging below its target.
# A joint-space integral after each ramp shifts the command until the joints sit on target,
# like a servo I term or a feedforward table on the real arm.
# ponytail: plain integral with clamp; replace with a gravity feedforward if it oscillates.
CORR_GAIN, CORR_EVERY, CORR_MAX = 0.3, 5, 0.2  # rad
RAMP = 0.7  # share of each state's duration spent ramping to the new joint target

# perception noise on the puck pose the script sees (uniform, +-)
POS_NOISE = 0.005
YAW_NOISE = math.radians(10.0)

# (name, target z above puck origin or None = home pose (env init_state), gripper, duration s)
STATES = [
    ("approach", APPROACH_Z, OPEN, 3.0),
    ("descend", GRASP_Z, OPEN, 1.5),
    ("close", GRASP_Z, CLOSE, 0.7),
    ("lift", LIFT_Z, CLOSE, 2.5),
    ("home", None, CLOSE, 3.0),
]
HELD_DIST = 0.05  # puck centre within this of the end effector = still in the hand
BOX_LAT = 0.035  # box_clear: half width of the band the open fingers + a neighbour puck occupy beside the finger axis
GRIP_STOP = 0.0083  # gripper joint position where the fingers stop on a 30 mm puck (close command = 0)
CORR_DT = CORR_EVERY / 60.0  # the integral runs at a fixed time step, whatever the control rate
PULL_S = 4.0  # pull test: linear ramp time
SLIP_POS, SLIP_ANG = 0.005, math.radians(10.0)  # puck moved this far in the hand = slipped
# basket on the robot: slot = cell floor centre in base_link (URDF basket_slot_0..11, 3 rows pole -> arm x 4 cols)
SLOTS = [(x, y, 0.09125) for x in (-0.16984, -0.10484, -0.03984) for y in (-0.0975, -0.0325, 0.0325, 0.0975)]
JOINT1_XY = (0.047985, -0.00023)  # joint1 axis in base_link (arm_mount_joint)
# Basket walls are 35 mm above the slot floor (storage_collision STLs). Offline IK (gripper straight down) reaches
# a puck bottom of 65 mm over the arm-side row, 45-60 mm over the middle row, but only 0-10 mm over the pole-side row
# (joint4 hits +113 deg). Tilting the gripper 20 deg (tip away from the arm) reaches 75-80 mm there.
# per row (0 = pole side): (name, puck bottom above the slot floor [m], gripper pitch [deg], gripper, duration s)
# release: fingers open to a 36 mm gap only (3 mm each side of the 30 mm puck), so they barely touch it on the way out
RELEASE = (0.0083 + (0.036 - 0.030) / 2) / 0.0265
if args.basket_retreat == "direct":  # first version (2026-10-27 report, section 7): put down at 5 mm, open fully,
    # back up to the over-slot height, then one joint-space move home (sweeps the arm past the pole)
    BASKET_PLAN = {
        0: [("over_slot", 0.05, -20.0, CLOSE, 3.0), ("lower", 0.005, -10.0, CLOSE, 1.5),
            ("release", 0.005, -10.0, OPEN, 0.7), ("retreat", 0.04, -20.0, OPEN, 1.0)],
        1: [("over_slot", 0.045, 0.0, CLOSE, 3.0), ("lower", 0.005, 0.0, CLOSE, 1.5),
            ("release", 0.005, 0.0, OPEN, 0.7), ("retreat", 0.045, 0.0, OPEN, 1.0)],
        2: [("over_slot", 0.06, 0.0, CLOSE, 3.0), ("lower", 0.005, 0.0, CLOSE, 1.5),
            ("release", 0.005, 0.0, OPEN, 0.7), ("retreat", 0.06, 0.0, OPEN, 1.0)],
    }
    BASKET_TAIL = [("home2", OPEN, 3.0)]
else:  # fold: set the puck down on the slot floor (the floor holds it upright; let go above it and it tilts and wedges
    # diagonally between the open fingers, then rides up with the hand), lift the empty hand straight up as far as the gripper orientation can be
    # kept (fingertips must clear the puck top: at least +35 mm), raise it high next to the joint1 axis, fold the arm
    # with joint1 still pointing at the slot, then turn joint1 home (the arm never sweeps past the pole stretched out)
    # lower / release tilt None = the smallest of SET_TILTS that is reachable (pole-side row cannot stand straight)
    BASKET_PLAN = {
        0: [("over_slot", 0.05, -20.0, CLOSE, 3.0), ("lower", 0.0, None, CLOSE, 1.5), ("release", 0.0, None, OPEN, 0.7)],
        1: [("over_slot", 0.045, 0.0, CLOSE, 3.0), ("lower", 0.0, None, CLOSE, 1.5), ("release", 0.0, None, OPEN, 0.7)],
        2: [("over_slot", 0.06, 0.0, CLOSE, 3.0), ("lower", 0.0, None, CLOSE, 1.5), ("release", 0.0, None, OPEN, 0.7)],
    }
    BASKET_TAIL = [("lift", OPEN, 1.0), ("raise", OPEN, 1.5), ("fold", OPEN, 2.0), ("home2", OPEN, 2.0)]
SET_TILTS = (0.0, -5.0, -10.0, -15.0)
LIFT_UPS = (0.06, 0.05, 0.045, 0.04, 0.035)  # tried in order, first one reachable with the release orientation
RAISE_R, RAISE_Z = 0.09, 0.38  # raise: end effector 9 cm from the joint1 axis towards the slot, 38 cm above base_link
BASKET_STATES = [(name, grip, dur) for name, _, _, grip, dur in BASKET_PLAN[0]] + BASKET_TAIL
IN_SLOT_XY, IN_SLOT_Z, IN_SLOT_TILT = 0.02, 0.015, math.radians(20.0)

# Command -> measured response lag of the real arm (R2S sine tests after the SyncRead firmware, 2026-10-26),
# order joint1..5, gripper. Applied as a pure delay on the action, rounded to whole control steps.
# ponytail: the real lag also contains servo dynamics that the sim drive already has, so this is a bit
# pessimistic; split into transport delay + drive tuning once the R2S fit gives the pure-delay part.
LATENCY_S = (0.045, 0.062, 0.052, 0.045, 0.045, 0.020)


def main():
    cfg = parse_env_cfg("Bee-Puck-Load", num_envs=args.num_envs)
    cfg.seed = args.seed
    if args.basket:
        cfg.episode_length_s = 40.0  # pick + basket is about 21 s; the default 20 s would reset mid-move
    cfg.sim.dt = 1.0 / (cfg.decimation * args.hz)
    if args.grip_force is not None:  # implicit PD: force = stiffness x (stop position - 0)
        grip = cfg.scene.robot.actuators["gripper"]
        grip.stiffness = args.grip_force / GRIP_STOP
        grip.effort_limit_sim = max(20.0, 2.0 * args.grip_force)
    if args.grip_damping is not None:
        cfg.scene.robot.actuators["gripper"].damping = args.grip_damping
    if args.torsion is not None:
        cp = cfg.scene.robot.spawn.collision_props
        cp.torsional_patch_radius = args.torsion / 1000
        cp.min_torsional_patch_radius = min(args.torsion, 5.0) / 1000
    if args.park_err_mm:
        e = args.park_err_mm / 1000
        rng = cfg.events.reset_puck.params["pose_range"]
        rng["x"] = (rng["x"][0] - e, rng["x"][1] + e)
        rng["y"] = (rng["y"][0] - e, rng["y"][1] + e)
    nb_names = []
    if args.neighbors is not None:
        for k in range(4):
            name = f"nb{k}"
            # one fixed puck type per neighbour: a second MultiUsdFileCfg asset made every env's target the same type
            setattr(cfg.scene, name, cfg.scene.puck.replace(prim_path="{ENV_REGEX_NS}/" + name,
                    spawn=sim_utils.UsdFileCfg(usd_path=bee_env_cfg.PUCK_USDS[k % 4]),
                    init_state=cfg.scene.puck.init_state.replace(pos=(1.0, 0.2 * k, 0.0))))
            nb_names.append(name)
    env = gym.make("Bee-Puck-Load", cfg=cfg)
    env.reset()
    u: ManagerBasedRLEnv = env.unwrapped
    robot: Articulation = u.scene["robot"]
    puck: RigidObject = u.scene["puck"]
    n, dev = u.num_envs, u.device
    step_dt = cfg.sim.dt * cfg.decimation
    arm_ids = robot.find_joints("joint[1-5]", preserve_order=True)[0]
    default_arm = robot.data.default_joint_pos.torch[:, arm_ids].clone()
    # end_effector_link is merged away (fixed joint): rebuild it from gripper_link + offset, for reporting
    ee_id = robot.find_bodies("gripper_link")[0][0]
    ee_offset = torch.tensor(bee_env_cfg.EE_OFFSET_POS, device=dev).repeat(n, 1)

    # IK works in base_link. Lab's root is base_footprint (10 mm lower) when the base is fixed and
    # base_link when it is free, so read base_link itself instead of the root.
    base_id = robot.find_bodies("base_link")[0][0]

    def to_root(pos_w):
        base_pos = robot.data.body_pos_w.torch[:, base_id]
        base_quat = robot.data.body_quat_w.torch[:, base_id]
        return math_utils.quat_apply_inverse(base_quat, pos_w - base_pos)

    def ee_b():
        g_pos = robot.data.body_pos_w.torch[:, ee_id]
        g_quat = robot.data.body_quat_w.torch[:, ee_id]
        return to_root(g_pos + math_utils.quat_apply(g_quat, ee_offset))

    def action(q_cmd, grip):
        return torch.cat([q_cmd - default_arm, torch.full((n, 2), grip, device=dev)], dim=1)

    # action columns: joint1..5, then both gripper joints (same lag); optional per-env jitter
    base_lag = [round(s / step_dt) if args.latency else 0 for s in LATENCY_S]
    base_lag = torch.tensor(base_lag + [base_lag[-1]], device=dev)
    jit = round(args.lat_jitter_ms / 1000 / step_dt)
    lag = (base_lag[None, :] + torch.randint(-jit, jit + 1, (n, 1), device=dev)).clamp(min=0)  # (n, 7)
    depth = int(lag.max().item()) + 1
    hist = deque([action(default_arm, OPEN)] * depth, maxlen=depth)
    env_i, col_i = torch.arange(n, device=dev)[:, None], torch.arange(7, device=dev)[None, :]
    if args.latency or jit:
        print(f"[latency] steps of {step_dt * 1000:.1f} ms per action column: {base_lag.tolist()}, jitter +-{jit} steps")

    trace = {"puck": [], "base": [], "arm": [], "puck_q": [], "fingers": []}
    fing_ids = robot.find_bodies("finger_.*_link")[0]

    def step(a):
        hist.append(a)
        h = torch.stack(list(hist))  # (depth, n, 7), newest last
        env.step(h[depth - 1 - lag, env_i, col_i])
        if args.trace:
            trace["puck"].append((puck.data.root_pos_w.torch - u.scene.env_origins).cpu().numpy())
            trace["base"].append((robot.data.body_pos_w.torch[:, base_id] - u.scene.env_origins).cpu().numpy())
            trace["arm"].append(robot.data.joint_pos.torch[:, arm_ids].cpu().numpy())
            trace["puck_q"].append(puck.data.root_quat_w.torch.cpu().numpy())
            trace["fingers"].append((robot.data.body_pos_w.torch[:, fing_ids] - u.scene.env_origins[:, None]).cpu().numpy())

    # neighbour pucks on a shelf grid around the target, those that would hang over the shelf edge are parked
    nbs = [u.scene[name] for name in nb_names]
    nb_start = []

    def place_neighbors():
        gap = args.neighbors / 1000
        org = u.scene.env_origins
        tgt = puck.data.root_pos_w.torch - org
        lo_x = bee_env_cfg.SHELF_NEAR_X + 0.02
        hi_x = bee_env_cfg.SHELF_NEAR_X + 2 * bee_env_cfg.SHELF_HALF - 0.02
        hi_y = bee_env_cfg.SHELF_HALF - 0.02
        for k, (nb, (dx, dy)) in enumerate(zip(nbs, [(gap, 0), (-gap, 0), (0, gap), (0, -gap)])):
            p = tgt.clone()
            p[:, 0] += dx
            p[:, 1] += dy
            p[:, 2] = bee_env_cfg.SHELF_TOP + 0.002
            off = (p[:, 0] < lo_x) | (p[:, 0] > hi_x) | (p[:, 1].abs() > hi_y)
            p[off] = torch.tensor([1.0, 0.2 * k, 0.0], device=dev)
            yaw_nb = (torch.rand(n, device=dev) * 2 - 1) * math.pi
            q = math_utils.quat_from_euler_xyz(torch.zeros(n, device=dev), torch.zeros(n, device=dev), yaw_nb)
            nb.write_root_pose_to_sim_index(root_pose=torch.cat([p + org, q], dim=1))
            nb.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev))
            nb_start.append((p, ~off))


    # settle at home before looking: right after reset the buffers are stale (base_link reads z 0) and
    # the puck is still dropping onto the shelf, which put every target 12 mm too high
    for k in range(int(0.5 / step_dt)):
        step(action(default_arm, OPEN))
        if k == 0 and nbs:  # after one step the reset puck pose is in the buffers
            place_neighbors()

    # puck pose is read once (it rests on a fixed shelf until grasped), with noise
    puck_b = to_root(puck.data.root_pos_w.torch).clone()
    puck_b[:, :2] += (torch.rand(n, 2, device=dev) * 2 - 1) * POS_NOISE
    puck_z0 = puck.data.root_pos_w.torch[:, 2].clone()
    puck_q_b = math_utils.quat_mul(math_utils.quat_inv(robot.data.body_quat_w.torch[:, base_id]), puck.data.root_quat_w.torch)
    yaw = math_utils.euler_xyz_from_quat(puck_q_b)[2] + (torch.rand(n, device=dev) * 2 - 1) * YAW_NOISE
    # Box faces repeat every 90 deg: pick the face closest to the arm's own heading (joint1 points the
    # arm at the puck), so joint5 turns at most 45 deg. Cylinders do not care, so the rule is harmless.
    heading = torch.atan2(puck_b[:, 1], puck_b[:, 0])
    yaw = heading + torch.remainder(yaw - heading + math.pi / 4, math.pi / 2) - math.pi / 4
    if args.diag:
        # The fingers close along the tool y axis, which is yaw + 90 deg. On a square grid of pucks the nearest
        # neighbours sit along the grid axes, the diagonal ones 1.41 x farther: for a cylinder (any yaw grasps it)
        # pick yaw = 45 deg + k x 90 deg, the one closest to the heading. Boxes keep their face rule.
        cyl = torch.tensor(["cyl" in u.sim.stage.GetPrimAtPath(f"/World/envs/env_{i}/puck").GetPrimStack()[1].layer.identifier
                            for i in range(n)], device=dev)
        diag = heading + torch.remainder(math.pi / 4 - heading + math.pi / 4, math.pi / 2) - math.pi / 4
        yaw = torch.where(cyl, diag, yaw)
    if args.box_clear and nb_start:
        # A box can only be pinched on two face pairs: fingers along yaw + 90 deg or along yaw. For each, find the
        # neighbours inside the band the opening fingers sweep (BOX_LAT to either side of the finger axis) and take
        # the nearest one along the axis; grasp the face pair whose nearest neighbour is farther. Neighbour positions
        # are what perception would report: the placed pucks with the same noise as the target.
        box = torch.tensor(["box" in u.sim.stage.GetPrimAtPath(f"/World/envs/env_{i}/puck").GetPrimStack()[1].layer.identifier
                            for i in range(n)], device=dev)
        nb_b = []
        for p0, on in nb_start:
            q_ = to_root(p0 + u.scene.env_origins)[:, :2] + (torch.rand(n, 2, device=dev) * 2 - 1) * POS_NOISE
            nb_b.append(torch.where(on[:, None], q_, torch.full_like(q_, 1e3)))  # absent neighbour: far away

        def room(y_):
            d = torch.stack([torch.cos(y_ + math.pi / 2), torch.sin(y_ + math.pi / 2)], dim=1)  # finger axis
            best = torch.full((n,), 1e3, device=dev)
            for q_ in nb_b:
                v = q_ - puck_b[:, :2]
                along = (v * d).sum(dim=1).abs()
                lat = (v[:, 0] * d[:, 1] - v[:, 1] * d[:, 0]).abs()
                best = torch.where(lat < BOX_LAT, torch.minimum(best, along), best)
            return best

        alt = yaw + math.pi / 2
        alt = torch.atan2(torch.sin(alt), torch.cos(alt))
        swap = box & (room(alt) > room(yaw) + 0.005)
        yaw = torch.where(swap, alt, yaw)
        print(f"[box_clear] boxes switched to the other face pair: {swap.sum().item()}/{box.sum().item()}")

    # joint targets per state, solved on the CPU; later states start from the previous solution
    ik = ArmIK()
    r_down = pin.Quaternion(np.array(DOWN_QUAT)).toRotationMatrix()
    q_goal = {name: torch.zeros(n, 5, device=dev) for name, *_ in STATES}
    ik_ok = torch.ones(n, dtype=torch.bool, device=dev)
    pb, yw, hd = puck_b.cpu().numpy(), yaw.cpu().numpy(), heading.cpu().numpy()
    for i in range(n):
        rot = pin.utils.rotate("z", float(yw[i])) @ r_down
        seeds = [np.array([hd[i], -0.2, 0.7, 1.0, s]) for s in (0.0, yw[i] - hd[i], hd[i] - yw[i])]
        for name, dz, *_ in STATES:
            if dz is None:
                continue
            q, e_pos, e_ang = ik.solve(pb[i] + np.array([0.0, 0.0, dz]), rot, seeds)
            q_goal[name][i] = torch.from_numpy(q).float()
            ik_ok[i] &= bool(e_pos < 2e-3 and e_ang < math.radians(2.0))
            seeds = [q]
    q_goal["home"] = default_arm.clone()
    if (~ik_ok).any():
        print(f"[ik] no exact solution for {(~ik_ok).sum().item()}/{n} envs (out of reach)")

    q_prev = robot.data.joint_pos.torch[:, arm_ids].clone()
    corr = torch.zeros(n, 5, device=dev)
    corr_every = max(1, round(CORR_DT / step_dt))
    ee_err = {}

    def move(q_ref_end, grip, dur):
        """Ramp from q_prev to q_ref_end in joint space, then hold; returns the largest joint error while moving."""
        nonlocal q_prev, corr
        steps = int(dur / step_dt)
        worst = torch.zeros(n, device=dev)
        for k in range(steps):
            t = min(1.0, (k + 1) / (RAMP * steps))
            q_ref = q_prev + t * (q_ref_end - q_prev)
            step(action(q_ref + corr, grip))
            q_err = q_ref - robot.data.joint_pos.torch[:, arm_ids]
            worst = torch.maximum(worst, q_err.abs().max(dim=1).values)
            # integrate only after the ramp, so the lag of a moving arm is not mistaken for sag (no windup)
            if t >= 1.0 and k % corr_every == corr_every - 1:
                corr = (corr + CORR_GAIN * q_err).clamp(-CORR_MAX, CORR_MAX)
        q_prev = q_ref_end
        return worst

    for name, dz, grip, dur in STATES:
        q_ref_end = q_goal[name]
        if name == "home" and args.basket and args.basket_skip_home:
            continue
        move(q_ref_end, grip, dur)
        if dz is None:
            j_err = (q_ref_end - robot.data.joint_pos.torch[:, arm_ids]).abs().max(dim=1).values
            print(f"[{name}] max joint error deg: median {math.degrees(j_err.median().item()):.1f}, max {math.degrees(j_err.max().item()):.1f}")
        else:
            target = puck_b.clone()
            target[:, 2] += dz
            err_mm = (ee_b() - target).norm(dim=1) * 1000
            ee_err[name] = err_mm
            if n <= 8:
                print(f"[{name}] ee error mm: {[round(e, 1) for e in err_mm.tolist()]}")
            else:
                print(f"[{name}] ee error mm: median {err_mm.median().item():.1f}, max {err_mm.max().item():.1f}")
        if name == "lift":
            lift_ok = (puck.data.root_pos_w.torch[:, 2] - puck_z0) > 0.03

    # success = puck lifted off the shelf AND still in the hand back at the home pose
    lifted = puck.data.root_pos_w.torch[:, 2] - puck_z0
    # origin is the bottom centre: step half the height along the puck's own axis (it may lie sideways)
    up = torch.tensor([[0.0, 0.0, PUCK_H / 2]], device=dev).repeat(n, 1)
    puck_c = to_root(puck.data.root_pos_w.torch + math_utils.quat_apply(puck.data.root_quat_w.torch, up))
    held = (puck_c - ee_b()).norm(dim=1) < HELD_DIST
    ok = (lifted > 0.03) & held
    if n <= 8:
        print(f"[result] puck lift at home mm: {[round(v, 1) for v in (lifted * 1000).tolist()]}")
    print(f"[result] lifted after lift: {lift_ok.sum().item()}/{n}")
    print(f"[result] success (held at home): {ok.sum().item()}/{n} = {ok.float().mean().item() * 100:.1f}%")
    # per puck type, read from which usda each env's puck prim was spawned from
    kinds = []
    for i in range(n):
        layers = [spec.layer.identifier for spec in u.sim.stage.GetPrimAtPath(f"/World/envs/env_{i}/puck").GetPrimStack()]
        kinds.append(next((l.split("/")[-1][:-5] for l in layers if "/puck_" in l), "?"))
    for kind in sorted(set(kinds)):
        idx = torch.tensor([k == kind for k in kinds], device=dev)
        print(f"[result]   {kind}: {ok[idx].sum().item()}/{idx.sum().item()}")
    fail = (~ok).nonzero().flatten().tolist()
    if fail:
        print("[result] failed envs (first 10) puck xy, yaw deg: " + ", ".join(
            f"{i}:({puck_b[i, 0].item():.3f},{puck_b[i, 1].item():.3f},{math.degrees(yaw[i].item()):.0f})" for i in fail[:10]))

    res = {"ok": ok, "lift_ok": lift_ok, "held": held, "lifted": lifted, "puck_b": puck_b, "yaw": yaw}
    res |= {f"ee_err_{k}": v for k, v in ee_err.items()}

    # basket: IK per env to its slot (gripper down, yaw = heading from joint1 + wrist option), joint-space moves
    if args.basket:
        slot = torch.arange(n, device=dev) % len(SLOTS)
        wrist = (torch.arange(n, device=dev) // len(SLOTS)) % 4
        bq = {name: default_arm.clone() for name, *_ in BASKET_STATES}
        b_ik = torch.ones(n, dtype=torch.bool, device=dev)
        # puck bottom centre in the end effector frame (it sits off centre by the grasp error and hangs tilted)
        g_quat_b = math_utils.quat_mul(math_utils.quat_inv(robot.data.body_quat_w.torch[:, base_id]), robot.data.body_quat_w.torch[:, ee_id])
        ee_q_b = math_utils.quat_mul(g_quat_b, torch.tensor(bee_env_cfg.EE_OFFSET_ROT, device=dev).repeat(n, 1))
        d_ee = math_utils.quat_apply_inverse(ee_q_b, to_root(puck.data.root_pos_w.torch) - ee_b())
        nominal = torch.tensor([0.0, 0.0, GRASP_Z], device=dev)  # what the plain plan assumes: straight below, GRASP_Z
        d_ee_np = (d_ee if args.basket_correct else nominal.repeat(n, 1)).cpu().numpy()
        res |= {"puck_in_hand_ee": d_ee}
        for i in range(n):
            sx, sy, sz = SLOTS[slot[i].item()]
            hd_s = math.atan2(sy - JOINT1_XY[1], sx - JOINT1_XY[0])
            yaw_s = hd_s + wrist[i].item() * math.pi / 2
            j5 = math.remainder(yaw_s - hd_s, 2 * math.pi)
            seeds = [np.array([hd_s, j2, j3, j4, j5]) for j2 in (-0.3, 0.3, 0.8) for j3 in (0.7, -0.5) for j4 in (1.0, 1.9)]
            for name, h, tilt, *_ in BASKET_PLAN[slot[i].item() // 4]:
                # pitch about the arm's sideways axis (in the heading frame), then the wrist option about the tool axis
                for t_ in ([tilt] if tilt is not None else SET_TILTS):
                    rot = pin.utils.rotate("z", hd_s) @ pin.utils.rotate("y", math.radians(t_)) @ r_down @ pin.utils.rotate("z", -(yaw_s - hd_s))
                    # end effector so that the puck bottom lands on (slot + h): ee = target - R d (d = bottom in ee frame)
                    q, e_pos, e_ang = ik.solve(np.array([sx, sy, sz + h]) - rot @ d_ee_np[i], rot, seeds)
                    if e_pos < 2e-3 and e_ang < math.radians(2.0):
                        break
                bq[name][i] = torch.from_numpy(q).float()
                b_ik[i] &= bool(e_pos < 2e-3 and e_ang < math.radians(2.0))
                seeds = [q]
            if "lift" in bq:
                q_lift = q
                for up_ in LIFT_UPS:  # straight up, same orientation, as high as reachable
                    q_try, e_pos, e_ang = ik.solve(np.array([sx, sy, sz + h + up_]) - rot @ d_ee_np[i], rot, [q])
                    if e_pos < 2e-3 and e_ang < math.radians(2.0):
                        q_lift = q_try
                        break
                bq["lift"][i] = torch.from_numpy(q_lift).float()
                # raise: high and close to the joint1 axis, orientation free (empty hand)
                p_r = np.array([JOINT1_XY[0] + RAISE_R * math.cos(hd_s), JOINT1_XY[1] + RAISE_R * math.sin(hd_s), RAISE_Z])
                q_raise, *_ = ik.solve(p_r, rot, [q_lift], w_ang=0.05)
                bq["raise"][i] = torch.from_numpy(q_raise).float()
                bq["fold"][i] = default_arm[i].clone()
                bq["fold"][i, 0] = float(q_raise[0])
        worst = {}
        z_axis = torch.tensor([[0.0, 0.0, 1.0]], device=dev).repeat(n, 1)

        def puck_tilt():
            q_b = math_utils.quat_mul(math_utils.quat_inv(robot.data.body_quat_w.torch[:, base_id]), puck.data.root_quat_w.torch)
            return torch.acos(math_utils.quat_apply(q_b, z_axis)[:, 2].clamp(-1, 1))

        stage_tilt = {}
        for name, grip, dur in BASKET_STATES:
            worst[name] = move(bq[name], grip, dur)
            stage_tilt[name] = puck_tilt()
            if name == "lower":
                p_in_hand = (to_root(puck.data.root_pos_w.torch + math_utils.quat_apply(puck.data.root_quat_w.torch, up)) - ee_b()).norm(dim=1) < HELD_DIST
        res |= {f"b_tilt_{k}": v for k, v in stage_tilt.items()}
        # in the slot: bottom centre near the slot floor centre, upright, and no longer in the hand
        sl = torch.tensor(SLOTS, device=dev)[slot]
        pb_now = to_root(puck.data.root_pos_w.torch)
        q_b = math_utils.quat_mul(math_utils.quat_inv(robot.data.body_quat_w.torch[:, base_id]), puck.data.root_quat_w.torch)
        tilt = torch.acos(math_utils.quat_apply(q_b, torch.tensor([[0.0, 0.0, 1.0]], device=dev).repeat(n, 1))[:, 2].clamp(-1, 1))
        in_slot = ((pb_now[:, :2] - sl[:, :2]).abs().max(dim=1).values < IN_SLOT_XY) & ((pb_now[:, 2] - sl[:, 2]).abs() < IN_SLOT_Z) & (tilt < IN_SLOT_TILT)
        b_ok = ok & in_slot
        print(f"[basket] IK exact {b_ik.sum().item()}/{n}; in slot {b_ok.sum().item()}/{ok.sum().item()} picked")
        for w in range(4):
            row = []
            for sidx in range(len(SLOTS)):
                m = ok & (slot == sidx) & (wrist == w)
                row.append(f"{b_ok[m].sum().item()}/{m.sum().item()}")
            print(f"[basket] wrist +{90 * w:3d} deg, slots 0..11: {' '.join(row)}")
        res |= {"slot": slot, "wrist": wrist, "b_ik": b_ik, "b_ok": b_ok, "in_slot": in_slot, "tilt": tilt,
                "puck_slot_b": pb_now, "held_at_lower": p_in_hand, "b_q_over": bq["over_slot"], "b_q_lower": bq["lower"]}
        res |= {f"b_worst_{k}": v for k, v in worst.items()}

    # neighbours: moved more than 10 mm = knocked by the fingers (only those placed on the shelf)
    if nbs:
        moved = torch.zeros(n, dtype=torch.bool, device=dev)
        placed = torch.zeros(n, dtype=torch.int64, device=dev)
        for nb, (p0, on) in zip(nbs, nb_start):
            d = (nb.data.root_pos_w.torch - u.scene.env_origins - p0)[:, :2].norm(dim=1)
            moved |= on & (d > 0.01)
            placed += on.long()
        res |= {"nb_moved": moved, "nb_placed": placed}
        print(f"[neighbors] gap {args.neighbors:.0f} mm: on shelf per env median {placed.float().median().item():.0f}, "
              f"envs with a neighbour knocked > 10 mm: {moved.sum().item()}/{n}")

    # pull-out: ramp a world force (down), a force along the fingers' approach axis (out) or a torque about the
    # finger closing axis (twist) on the held puck; record the value when the puck has moved in the hand
    if args.pull:
        f_grip = args.grip_force if args.grip_force is not None else 1000.0 * GRIP_STOP
        top = 8.0 * f_grip if args.pull != "twist" else 0.05 * f_grip  # N or N m, well above the expected slip
        fing = robot.find_bodies("finger_.*_link")[0]
        ee_rot = torch.tensor(bee_env_cfg.EE_OFFSET_ROT, device=dev).repeat(n, 1)

        def hand():
            g_pos, g_quat = robot.data.body_pos_w.torch[:, ee_id], robot.data.body_quat_w.torch[:, ee_id]
            return g_pos + math_utils.quat_apply(g_quat, ee_offset), math_utils.quat_mul(g_quat, ee_rot)

        def rel():
            e_p, e_q = hand()
            c = puck.data.root_pos_w.torch + math_utils.quat_apply(puck.data.root_quat_w.torch, up)
            return math_utils.quat_apply_inverse(e_q, c - e_p), math_utils.quat_mul(math_utils.quat_inv(e_q), puck.data.root_quat_w.torch)

        p0, q0 = rel()
        slip = torch.full((n,), float("nan"), device=dev)
        steps = int(PULL_S / step_dt)
        for k in range(steps):
            mag = top * (k + 1) / steps
            e_p, e_q = hand()
            if args.pull == "down":
                vec = torch.tensor([0.0, 0.0, -1.0], device=dev).repeat(n, 1)
            elif args.pull == "out":
                vec = math_utils.quat_apply(e_q, torch.tensor([0.0, 0.0, 1.0], device=dev).repeat(n, 1))
            else:
                fp = robot.data.body_pos_w.torch[:, fing]
                vec = torch.nn.functional.normalize(fp[:, 0] - fp[:, 1], dim=1)
            w = (vec * mag)[:, None, :]
            zero = torch.zeros_like(w)
            puck.permanent_wrench_composer.set_forces_and_torques_index(
                forces=zero if args.pull == "twist" else w, torques=w if args.pull == "twist" else zero, body_ids=[0], is_global=True)
            step(action(q_prev + corr, CLOSE))
            p1, q1 = rel()
            ang = 2 * torch.acos(math_utils.quat_mul(math_utils.quat_inv(q0), q1)[:, 3].abs().clamp(max=1.0))
            moved = ((p1 - p0).norm(dim=1) > SLIP_POS) | (ang > SLIP_ANG)
            slip = torch.where(ok & moved & slip.isnan(), torch.full_like(slip, mag), slip)
        z = torch.zeros(n, 1, 3, device=dev)
        puck.permanent_wrench_composer.set_forces_and_torques_index(forces=z, torques=z, body_ids=[0], is_global=True)
        held_all = ok & slip.isnan()
        slip = torch.where(held_all, torch.full_like(slip, top), slip)  # censored: never slipped up to `top`
        unit = "N m" if args.pull == "twist" else "N"
        sv = slip[ok]
        print(f"[pull:{args.pull}] slip {unit}: median {sv.median().item():.3f}, p10 {sv.quantile(0.1).item():.3f}, "
              f"p90 {sv.quantile(0.9).item():.3f}; never slipped up to {top:.2f}: {held_all.sum().item()}/{ok.sum().item()}")
        res |= {"slip": slip, "slip_top": torch.tensor(top), "slip_censored": held_all}

    if args.out:
        out = {k: (v.cpu().numpy() if torch.is_tensor(v) else v) for k, v in res.items()}
        out |= {"kind": np.array(kinds), "step_dt": step_dt, "args": str(vars(args))}
        if args.trace:
            out |= {f"trace_{k}": np.stack(v) for k, v in trace.items()}
        np.savez_compressed(args.out, **out)
        print(f"[out] {args.out}")

    # with a viewer, keep holding the last command until the window is closed or Ctrl+C
    # (the env still auto-resets at episode_length_s)
    if getattr(args, "visualizer", None):
        print("[hold] holding last pose, close the window or press Ctrl+C to exit")
        try:
            while simulation_app.is_running():
                step(action(q_prev + corr, CLOSE))
        except KeyboardInterrupt:
            pass

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
