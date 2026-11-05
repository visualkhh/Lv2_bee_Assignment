"""One manipulator cycle in Isaac Sim on the mission scene isaac/sim/usda/scene_lab.usda.

    from the saved start pose drive to the south face of start_1 -> pick one of its two pucks -> load it into a
    basket slot -> drive east along the bottom corridor to shelf_L1 -> take it out of the slot -> place it on the
    centre cell (dice 1) with the trained residual policies (pick and place)

Arm moves are the scripted ones of feature/isaac-pick cycle_sm.py (offline IK, joint-space moves at 0.3 x team
limits, measured command latency, sag integral while holding). Only the place descent uses the RL residual
(policies/place_residual_sag.pt if present, else place_residual.pt, trained by train_place.py on Bee-Place-Residual);
--no_policy runs it with residual 0. A policy that sees the sag integral (5 more observations) gets the integral law
it was trained on (error ref + residual - q); older policies keep ref - q.
Perception is a stand-in: ground-truth poses plus the noise used in training (puck +-5 mm / 10 deg, cell +-5 mm,
puck in hand +-2 mm, gripper camera bias 2 mm + noise 1 mm). Driving is a plain P controller on the true base pose.

    python demo_cycle.py --viz kit                 (window)
    python demo_cycle.py --runs 20                 (headless check: success rate over 20 cycles, no window)
"""

import argparse
import math
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--policy", default=None, help="TorchScript residual policy (default policies/place_residual_sag.pt if present, "
                    "else place_residual.pt; with --fast place_residual_fast.pt)")
parser.add_argument("--fast", action="store_true", help="place at 0.6 x team limits with 0.15 s settle (policy trained with speed randomization)")
parser.add_argument("--no_policy", action="store_true", help="place with residual 0 (scripted base motion only)")
parser.add_argument("--cell", type=int, default=4, help="dice cell 0..5 (rows near, middle; 4 = centre = dice 1)")
parser.add_argument("--slot", type=int, default=5, help="basket slot 0..11")
parser.add_argument("--puck", choices=["a", "b"], default="a", help="start_1 puck: a = puck_box_red_0 (west), b = puck_box_green_6 (east)")
parser.add_argument("--mixed", action="store_true",
                    help="start_1 holds a standing cylinder (puck a, puck_cyl_red_3 moved from start_4) and a lying box (puck b): "
                         "both go to L1, the box is stood up first with the residual policy of right_env.py")
parser.add_argument("--right_policy", default=None, help="default policies/right_residual_sag.pt if present, else right_residual_tilt.pt")
parser.add_argument("--stand_only", choices=["stow", "carry"], default=None,
                    help="--mixed check: only stand the lying box up, arm starting from stow or the carry pose")
parser.add_argument("--pick_policy", default=None, help="TorchScript pick residual policy (default policies/pick_residual_sag.pt if present)")
parser.add_argument("--pick_zero", action="store_true", help="pick with residual 0 (scripted motion only)")
parser.add_argument("--then_pick", action="store_true", help="--stand_only: grasp the stood-up box in place right after the stand-up (lift and stop)")
parser.add_argument("--load_only", action="store_true", help="stop once the pucks are in the basket (no drive to L1)")
parser.add_argument("--then_load", action="store_true", help="--stand_only: grasp the stood-up box in place, load it into --slot2, end at stow")
parser.add_argument("--right_zero", action="store_true", help="stand up with residual 0 (scripted hang only)")
parser.add_argument("--show_carry", action="store_true", help="pick one puck, go to the carry pose and stop there")
parser.add_argument("--no_sag", action="store_true", help="no sag integral (the real robot has none; cycle_sm.py added it)")
parser.add_argument("--slot2", type=int, default=6, help="--mixed: basket slot of the second puck")
parser.add_argument("--cell2", type=int, default=0, help="--mixed: dice cell of the second puck")
parser.add_argument("--runs", type=int, default=1, help="cycles in a row (the scene resets between them)")
parser.add_argument("--seed", type=int, default=0, help="perception noise seed (-1 = random)")
parser.add_argument("--debug", action="store_true")
parser.add_argument("--hold", action="store_true", help="keep the window open after the last cycle")
parser.add_argument("--view", default="cam", help="Kit viewport camera: a camera prim under /World in scene_lab.usda "
                    "(default cam = /World/cam/Camera, set up by hand), or free = the old eye / target view")
parser.add_argument("--render", default="MinimalRendering",
                    choices=["MinimalRendering", "RealTimePathTracing", "RaytracedLighting", "PathTracing"],
                    help="Kit viewport renderer (default the light RTX - Minimal mode)")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
pol_dir = Path(__file__).resolve().parent / "policies"
if args.policy is None:
    sag = pol_dir / "place_residual_sag.pt"
    args.policy = str(pol_dir / "place_residual_fast.pt" if args.fast else sag if sag.exists() else pol_dir / "place_residual.pt")
if args.pick_policy is None:
    sag = pol_dir / "pick_residual_sag.pt"
    args.pick_policy = str(sag) if sag.exists() else None
if args.right_policy is None:
    sag = pol_dir / "right_residual_sag.pt"
    args.right_policy = str(sag if sag.exists() else pol_dir / "right_residual_tilt.pt")
app = AppLauncher(args).app

import numpy as np  # noqa: E402
import pinocchio as pin  # noqa: E402
import torch  # noqa: E402
import warp as wp  # noqa: E402
from isaaclab_physx.physics import PhysxCfg  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
import isaaclab.utils.math as mu  # noqa: E402
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab.visualizers import VisualizerCfg  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402

import bee_common as bc  # noqa: E402
import pick_env as pk  # noqa: E402
import place_env as pe  # noqa: E402
import right_env as re_  # noqa: E402

DEMO_USD = str(bc.USD_DIR / "scene_lab.usda")
DT, DECIM = 1 / 200, 4  # physics 200 Hz, policy 50 Hz (as training)
_STAGE = Usd.Stage.Open(DEMO_USD)


def usd_pose(path):
    """World pose of a prim in scene_lab.usda -> (pos, quat x, y, z, w)."""
    tf = Gf.Transform(UsdGeom.Xformable(_STAGE.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
    q = tf.GetRotation().GetQuat()
    return tuple(tf.GetTranslation()), (*q.GetImaginary(), q.GetReal())


def usd_yaw(path):
    x, y, z, w = usd_pose(path)[1]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def lying_pose(spot, yaw):
    """Root pose of a puck lying on the box top with its centre at `spot` and its axis (near -> far end) at world yaw."""
    pos = (spot[0] - re_.PUCK_L / 2 * math.cos(yaw), spot[1] - re_.PUCK_L / 2 * math.sin(yaw), bc.BOX_H + re_.PUCK_W / 2 + 0.0005)
    c, s = math.cos(yaw / 2), math.sin(yaw / 2)
    h = math.sqrt(0.5)  # q_z(yaw) * q_y(90 deg), x y z w
    return pos, (-s * h, c * h, s * h, c * h)


LYING_YAW = math.radians(110.0)  # 20 deg off the start_1 dock heading (+y), within the trained +-90 deg
ROBOT_POSE = usd_pose("/World/beebot")
SPOTS = {"puck_a": "puck_box_red_0", "puck_b": "puck_box_green_6"}  # the two pucks on start_1
PUCKS = {**SPOTS, "puck_a": "puck_cyl_red_3"} if args.mixed else SPOTS
REACH = 0.08 + bc.BOX_HALF  # puck pair / box centre ahead of the root when docked (as training)
# start_1: dock on the south face so the two pucks (lying along x) sit side by side across the gripper, as in
# pick_env; the north face is 0.12 m from wall_d0. VIA_Y is a lane south of the box to drive around it.
_mid = [sum(usd_pose(f"/World/pucks/{n}")[0][i] for n in SPOTS.values()) / 2 for i in (0, 1)]
START_DOCK = (_mid[0], _mid[1] - REACH, math.pi / 2)
VIA_Y = START_DOCK[1] - 0.15
# dice box shelf_L1, approached from the west along the bottom corridor: the face whose normal is closest to +x
_c, _ = usd_pose("/World/shelf_L1")
_yaw = usd_yaw("/World/shelf_L1")
DICE_BOX = (_c[0], _c[1], math.remainder(_yaw, math.pi / 2))  # centre x, y and approach heading
DOCK = (DICE_BOX[0] - REACH * math.cos(DICE_BOX[2]), DICE_BOX[1] - REACH * math.sin(DICE_BOX[2]), DICE_BOX[2])
WHEEL_R, WHEEL_SEP = 0.033, 0.2878

# cycle_sm.py constants (feature/isaac-pick), gripper values in joint metres here
APPROACH_Z = bc.GRASP_Z + 0.05
OPEN, CLOSE = bc.gap_joint(40.0), 0.0
CLOSE_S, RELEASE_S, SETTLE_S = 0.5, 0.4, 0.3
POS_NOISE, YAW_NOISE = 0.005, math.radians(10.0)
SLOTS = [(x, y, 0.09125) for x in (-0.16984, -0.10484, -0.03984) for y in (-0.0975, -0.0325, 0.0325, 0.0975)]
ST = dict(np.load(Path(__file__).resolve().parent / "slot_table.npz"))  # load / unload poses per slot (slot_table.py)
LIFT_CLEARS = (0.10, 0.09, 0.08)  # held puck bottom above the start box top before the sweep to carry (pucks: 60 mm)
STOW = (0.0, -1.9024, 1.344, 0.9023, 0.0)  # beebot.usda default pose
# carry pose (cycle_sm.py route "carry", report 20261027_sim_tests 11-13), one per basket side, joint1..5 [rad]:
# fingertip 0.08 m beside joint1 at +-90 deg from the front, 0.21 m up, tool down, wrist turned 90 deg
# (+y side mirrored). Was an IK solve at start-up with fixed seeds, which always gave these values.
CARRY = {1: (math.pi / 2, -0.465027, 0.102271, 1.933412, math.pi / 2),  # slot on the left (+y)
         -1: (-math.pi / 2, -0.465027, 0.102271, 1.933412, -math.pi / 2)}  # slot on the right (-y)
VEL_SCALE = 0.3
EYE, TARGET = (-0.15, -3.9, 1.5), (-0.15, -2.45, 0.0)  # window camera over the bottom corridor


@configclass
class DemoSceneCfg(InteractiveSceneCfg):
    demo = AssetBaseCfg(prim_path="{ENV_REGEX_NS}/demo", spawn=sim_utils.UsdFileCfg(usd_path=DEMO_USD))
    # the scene's own beebot has no physics variant selected: spawn the training robot at its pose instead
    robot = pe.robot_cfg().replace(prim_path="{ENV_REGEX_NS}/robot")
    puck_a = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/demo/pucks/" + PUCKS["puck_a"], spawn=None)
    puck_b = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/demo/pucks/" + PUCKS["puck_b"], spawn=None)
    dice = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/demo/shelf_L1", spawn=None)
    start = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/demo/start_boxes/start_1", spawn=None)

    def __post_init__(self):
        super().__post_init__()
        self.robot.init_state.pos, self.robot.init_state.rot = ROBOT_POSE
        for k, name in PUCKS.items():
            pos, rot = usd_pose(f"/World/pucks/{SPOTS[k]}")
            if args.mixed and k == "puck_b":  # lying on its side, centre on the spot, root = bottom face = near end
                pos, rot = lying_pose(pos, LYING_YAW)
            getattr(self, k).init_state = RigidObjectCfg.InitialStateCfg(pos=pos, rot=rot)
        pos, rot = usd_pose("/World/shelf_L1")
        self.dice.init_state = RigidObjectCfg.InitialStateCfg(pos=pos, rot=rot)
        pos, rot = usd_pose("/World/start_boxes/start_1")
        self.start.init_state = RigidObjectCfg.InitialStateCfg(pos=pos, rot=rot)


def sees_corr(policy, base_dim):
    """True when the exported policy takes the 5 sag integral observations after its base_dim ones."""
    return policy is not None and dict(policy.named_parameters())["mlp.0.weight"].shape[1] == base_dim + 5


class Bot:
    """Joint-space motion engine of cycle_sm.py for one robot, at 200 Hz with latency and the sag integral."""

    def __init__(self, sim, scene):
        self.sim, self.scene = sim, scene
        self.robot = scene["robot"]
        self.dev = sim.device
        self.arm = self.robot.find_joints("joint[1-5]", preserve_order=True)[0]
        self.grip = self.robot.find_joints("gripper_.*_joint")[0]
        self.wheels = self.robot.find_joints(["wheel_left_joint", "wheel_right_joint"], preserve_order=True)[0]
        self.ee_id = self.robot.find_bodies("gripper_link")[0][0]
        self.base_id = self.robot.find_bodies("base_link")[0][0]
        self.ee_off = torch.tensor([bc.EE_OFFSET_POS], device=self.dev)
        self.ee_rot = torch.tensor([bc.EE_OFFSET_ROT], device=self.dev)
        lat = [*bc.LATENCY_S, bc.LATENCY_GRIP_S]
        self.lag = [round(s / DT) for s in lat]
        self.k = 0
        self.ik = bc.ArmIK()
        self.reset_state()

    def reset_state(self, q=None):
        q = self.robot.data.joint_pos.torch[0, self.arm].clone() if q is None else q
        self.q_ref, self.corr, self.delta = q, torch.zeros(5, device=self.dev), torch.zeros(5, device=self.dev)
        self.keep_delta = False
        self.g_ref = OPEN
        self.wheel_v = torch.zeros(2, device=self.dev)
        self.hist = [torch.cat([q, torch.tensor([OPEN], device=self.dev)])] * (max(self.lag) + 1)

    # ---- frames
    def base(self):
        return self.robot.data.body_pos_w.torch[:, self.base_id], self.robot.data.body_quat_w.torch[:, self.base_id]

    def to_base(self, pos_w):
        bp, bq = self.base()
        return mu.quat_apply_inverse(bq, pos_w.reshape(1, 3) - bp)[0]

    def ee(self):
        g_pos, g_quat = self.robot.data.body_pos_w.torch[:, self.ee_id], self.robot.data.body_quat_w.torch[:, self.ee_id]
        _, bq = self.base()
        pos = self.to_base((g_pos + mu.quat_apply(g_quat, self.ee_off))[0])
        return pos, mu.quat_mul(mu.quat_inv(bq), mu.quat_mul(g_quat, self.ee_rot))[0]

    def root_pose2d(self):
        p, q = self.robot.data.root_pos_w.torch[0], self.robot.data.root_quat_w.torch[0]
        yaw = mu.euler_xyz_from_quat(q[None])[2][0].item()
        return p[0].item() - self.scene.env_origins[0, 0].item(), p[1].item() - self.scene.env_origins[0, 1].item(), yaw

    # ---- one physics step
    def step(self, holding):
        if holding and self.k % round((5 / 60) / DT) == 0:
            # sag-aware policies (corr in the obs) were trained on ref + residual - q: the integral keeps the residual;
            # older ones on ref - q, which pulls the residual back out while holding (place_env._sag_step)
            err = self.q_ref + (self.delta if self.keep_delta else 0.0) - self.robot.data.joint_pos.torch[0, self.arm]
            self.corr = (self.corr + (0.0 if args.no_sag else 0.3) * err).clamp(-0.2, 0.2)
        self.hist.append(torch.cat([self.q_ref + self.corr + self.delta, torch.tensor([self.g_ref], device=self.dev)]))
        self.hist.pop(0)
        tgt = torch.stack([self.hist[-1 - d][j] for j, d in enumerate(self.lag)])
        self.robot.set_joint_position_target_index(target=tgt[None, :5], joint_ids=self.arm)
        self.robot.set_joint_position_target_index(target=tgt[None, 5:6].repeat(1, len(self.grip)), joint_ids=self.grip)
        self.robot.set_joint_velocity_target_index(target=self.wheel_v[None], joint_ids=self.wheels)
        self.scene.write_data_to_sim()
        self.k += 1
        self.sim.step(render=self.k % DECIM == 0)
        self.scene.update(DT)

    def move(self, q_end, grip=None, hold=0.0, label=""):
        q0 = self.q_ref.clone()
        q_end = torch.as_tensor(q_end, dtype=torch.float32, device=self.dev)
        dur = bc.trapezoid_duration((q_end - q0).abs().max().item(), VEL_SCALE)
        if grip is not None:
            self.g_ref = grip
        if label:
            print(f"[demo] {label}", flush=True)
        n = int(math.ceil((dur + hold) / DT))
        for i in range(n):
            t = (i + 1) * DT
            self.q_ref = q0 + bc.move_s(t, dur) * (q_end - q0)
            self.step(holding=t >= dur)

    def new_skill(self):
        """Each skill starts its sag integral from zero, as every training episode does: a value built up in another
        pose (stow while driving) pushes the next pose off (unload hands stopped 24 mm high on the slot rim)."""
        self.corr = torch.zeros(5, device=self.dev)

    def dwell(self, grip, secs, label=""):
        self.move(self.q_ref, grip, secs, label)

    def drive_to(self, x, y, yaw, reverse=False, label=""):
        """Turn to face the goal, roll to it, turn to the final yaw (P control on the true root pose)."""
        if label:
            print(f"[demo] {label}", flush=True)

        def wheels(v, w):
            self.wheel_v = torch.tensor([(v - w * WHEEL_SEP / 2) / WHEEL_R, (v + w * WHEEL_SEP / 2) / WHEEL_R], device=self.dev)

        def turn(target, tol=math.radians(0.7)):
            for _ in range(int(8 / DT)):
                e = math.remainder(target - self.root_pose2d()[2], 2 * math.pi)
                if abs(e) < tol:
                    break
                wheels(0.0, max(-1.0, min(1.0, 3.0 * e)))
                self.step(holding=True)
            wheels(0.0, 0.0)

        x0, y0, _ = self.root_pose2d()
        if math.hypot(x - x0, y - y0) > 0.005:
            hd = math.atan2(y - y0, x - x0) + (math.pi if reverse else 0.0)
            turn(hd)
            for _ in range(int((5 + math.hypot(x - x0, y - y0) / 0.1) / DT)):  # 0.12 m/s cap + margin
                cx, cy, cyaw = self.root_pose2d()
                fwd = (x - cx) * math.cos(cyaw) + (y - cy) * math.sin(cyaw)
                if abs(fwd) < 0.003:
                    break
                lat = -(x - cx) * math.sin(cyaw) + (y - cy) * math.cos(cyaw)
                v = max(-0.12, min(0.12, 1.5 * fwd))
                wheels(v, max(-0.5, min(0.5, 4.0 * lat * (1 if v >= 0 else -1))))
                self.step(holding=True)
            wheels(0.0, 0.0)
        turn(yaw)
        for _ in range(int(0.5 / DT)):  # let the base settle
            self.step(holding=True)


class Demo:
    def __init__(self, sim, scene):
        self.sim, self.scene, self.bot = sim, scene, Bot(sim, scene)
        self.dev = sim.device
        self.org = scene.env_origins[0]
        self.puck_name = "puck_a" if args.mixed or args.puck == "a" else "puck_b"  # --mixed: the standing cylinder first
        self.use(self.puck_name)
        self.policy = None
        if not args.no_policy:
            self.policy = torch.jit.load(args.policy, map_location=self.dev)
            self.policy.eval()
            print(f"[demo] place residual policy: {args.policy}")
        self.pick_policy = None
        if args.pick_policy and not args.pick_zero:
            self.pick_policy = torch.jit.load(args.pick_policy, map_location=self.dev)
            self.pick_policy.eval()
            print(f"[demo] pick residual policy: {args.pick_policy}")
        self.carry = CARRY
        if args.mixed:
            self.right_policy = torch.jit.load(args.right_policy, map_location=self.dev)
            self.right_policy.eval()
            print(f"[demo] stand-up residual policy: {args.right_policy}")

    def puck_bottom_in_ee(self):
        e_p, e_q = self.bot.ee()
        return mu.quat_apply_inverse(e_q[None], (self.bot.to_base(self.puck.data.root_pos_w.torch[0]) - e_p)[None])[0]

    def in_hand(self):
        mid = self.puck.data.root_pos_w.torch[0] + mu.quat_apply(self.puck.data.root_quat_w.torch, torch.tensor([[0, 0, bc.PUCK_H / 2]], device=self.dev))[0]
        return (self.bot.to_base(mid) - self.bot.ee()[0]).norm().item() < 0.05

    # ---- segment 1-2: pick from the start box (cycle_sm.py --layout start_y --finger_rule perp)
    def pick(self):
        b = self.bot
        b.new_skill()
        pb = self.bot.to_base(self.puck.data.root_pos_w.torch[0]).cpu().numpy()
        pb[:2] += np.random.uniform(-POS_NOISE, POS_NOISE, 2)
        _, bq = b.base()
        yaw = mu.euler_xyz_from_quat(mu.quat_mul(mu.quat_inv(bq), self.puck.data.root_quat_w.torch))[2][0].item()
        yaw_true = yaw
        yaw += np.random.uniform(-YAW_NOISE, YAW_NOISE)
        hd = math.atan2(pb[1], pb[0])
        want = math.pi / 2  # fingers close at right angles to the line through the 2 pucks (along y)
        want = hd + math.remainder(want - hd, math.pi)
        yaw = want + math.remainder(yaw - want, math.pi / 2) if self.is_box else want
        rot = pin.utils.rotate("z", yaw) @ bc.DOWN
        seeds = [np.array([hd, -0.2, 0.7, 1.0, s]) for s in (0.0, yaw - hd, hd - yaw)]
        q = {}
        for name, dz in (("approach", APPROACH_Z), ("descend", bc.GRASP_Z)):
            q[name], *_ = b.ik.solve(pb + np.array([0.0, 0.0, dz]), rot, seeds)
            seeds = [q[name]]
        z0 = self.puck.data.root_pos_w.torch[0, 2].item()
        # pick phases exactly as pick_env.PickEnv: transit from here, hold 0.3 s above the puck (the sag integral removes
        # the ~3 mm overshoot along the finger axis first: without it a finger landed on the puck top, 3/5 in a test),
        # down, settle, close, lift; the residual policy adds its delta from the hold until the fingers close
        cfg, dev = pk.PickEnvCfg(), self.dev
        q_ab, q_dn = (torch.tensor(q[k], dtype=torch.float32, device=dev) for k in ("approach", "descend"))
        q0 = b.q_ref.clone()
        t_tr = bc.trapezoid_duration((q_ab - q0).abs().max().item(), VEL_SCALE)
        t_mv = bc.trapezoid_duration((q_dn - q_ab).abs().max().item(), VEL_SCALE)
        t_ph = np.cumsum([t_tr + cfg.hold_s, t_mv, cfg.settle_s, cfg.release_s, t_mv])
        cam_bias = torch.zeros(3, device=dev)
        cam_bias[:2] = torch.empty(2, device=dev).uniform_(-cfg.cam_bias_m, cfg.cam_bias_m)
        yaw_bias = np.random.uniform(-math.radians(3.0), math.radians(3.0))
        up_h = torch.tensor([[0.0, 0.0, bc.PUCK_H / 2]], device=dev)
        b.keep_delta = sees_corr(self.pick_policy, pk.OBS_DIM)
        prev_act = torch.zeros(5, device=dev)
        print("[demo] 1 stow -> above the puck", flush=True)
        print(f"[demo] 2 pick: {'residual policy on' if self.pick_policy is not None else 'residual 0'}", flush=True)
        t = 0.0
        while t < t_ph[-1] - 1e-9:
            ph = int(np.sum(t >= t_ph))
            # observation (pick_env.PickEnv._get_observations)
            qj = b.robot.data.joint_pos.torch[0, b.arm]
            qd = b.robot.data.joint_vel.torch[0, b.arm]
            onehot = torch.nn.functional.one_hot(torch.tensor(min(ph, 5), device=dev), 6).float()
            start = 0.0 if ph == 0 else t_ph[min(ph, 5) - 1]
            prog = min(max(t - start, 0.0), 2.0)
            pq = self.puck.data.root_quat_w.torch
            cam = b.to_base(self.puck.data.root_pos_w.torch[0] + mu.quat_apply(pq, up_h)[0]) - b.ee()[0]
            cam = cam + cam_bias + cfg.cam_noise_m * torch.randn(3, device=dev)
            _, bq = b.base()
            pyaw = mu.euler_xyz_from_quat(mu.quat_mul(mu.quat_inv(bq), pq))[2][0].item()
            face = math.remainder(pyaw - yaw, math.pi / 2) if self.is_box else 0.0
            cam_yaw = (face + yaw_bias + cfg.cam_yaw_noise * np.random.randn()) * float(self.is_box)
            obs = torch.cat([qj - b.q_ref, q_dn - b.q_ref, 0.1 * qd, b.q_ref - q_ab, prev_act, onehot,
                             torch.tensor([prog], device=dev), 100.0 * cam[:2],
                             torch.tensor([cam_yaw, float(self.is_box)], device=dev)]).float()
            if b.keep_delta:
                obs = torch.cat([obs, 10.0 * b.corr])
            a = torch.zeros(5, device=dev)
            if self.pick_policy is not None:
                with torch.inference_mode():
                    a = self.pick_policy(obs[None])[0].clamp(-1, 1)
            if ph < 3 and t >= t_tr:
                b.delta = cfg.residual_rad * a
            prev_act = a.clone()
            for _ in range(DECIM):
                p = int(np.sum(t >= t_ph))
                if t < t_tr:
                    b.q_ref = q0 + bc.move_s(t, t_tr) * (q_ab - q0)
                elif p >= 3:
                    b.q_ref = q_dn + bc.move_s(t - t_ph[3], t_mv) * (q_ab - q_dn)
                else:
                    b.q_ref = q_ab + bc.move_s(t - t_ph[0], t_mv) * (q_dn - q_ab)
                b.g_ref = CLOSE if p >= 3 else OPEN
                b.step(holding=(p == 0 and t >= t_tr) or p in (2, 3))
                t += DT
        b.delta, b.keep_delta = torch.zeros(5, device=dev), False
        ok = self.puck.data.root_pos_w.torch[0, 2].item() - z0 > 0.03 and self.in_hand()
        if not ok:
            pw = b.to_base(self.puck.data.root_pos_w.torch[0])
            up = mu.quat_apply(self.puck.data.root_quat_w.torch, torch.tensor([[0.0, 0.0, 1.0]], device=self.dev))[0]
            print(f"[demo] pick FAILED: puck at {1000 * pw[0].item():.0f}, {1000 * pw[1].item():.0f}, {1000 * pw[2].item():.0f} mm, "
                  f"tilt {math.degrees(math.acos(max(-1.0, min(1.0, up[2].item())))):.0f} deg, tool yaw {math.degrees(yaw):.0f} "
                  f"vs puck yaw {math.degrees(yaw_true):.0f} deg")
        else:
            print("[demo] pick OK")
        return ok

    # ---- segment 3-4: carry -> load into the basket slot
    def load(self, slot, to_carry=False):
        """to_carry: end at the carry pose instead of stow (another puck follows: the arm is already beside the basket)."""
        b = self.bot
        b.new_skill()
        # slot table (slot_table.py): poses for the nominal puck in the hand, shifted by the measured offset
        e = self.puck_bottom_in_ee().cpu().numpy() - ST["d_nom"]
        sx, sy, sz = SLOTS[slot]
        k = "box" if self.is_box else "cyl"
        at = lambda name, rot: ST[f"{k}_q_{name}"][slot] + ST[f"{k}_P_{name}"][slot] @ (-ST[f"{k}_rot_{rot}"][slot] @ e)
        q_over, q_low, q_lift = at("over", "over"), at("low", "low"), at("lift", "low")
        carry = self.carry[1 if sy >= 0 else -1]
        dbg = lambda tag: args.debug and print(
            f"[debug] load {tag}: puck bottom in ee {[round(1000 * v, 1) for v in self.puck_bottom_in_ee().tolist()]} mm, "
            f"tilt {math.degrees(math.acos(max(-1.0, min(1.0, self.up_z())))):.1f} deg, "
            f"grip q {[round(1000 * v, 1) for v in b.robot.data.joint_pos.torch[0, b.grip].tolist()]} mm, "
            f"f {[round(v, 2) for v in b.robot.data.applied_torque.torch[0, b.grip].tolist()]} N, "
            f"puck - slot {[round(1000 * v) for v in (b.to_base(self.puck.data.root_pos_w.torch[0]) - torch.tensor([sx, sy, sz], device=self.dev)).tolist()]} mm", flush=True)
        dbg("start")
        # straight up first: the pick / regrasp lift (50 mm) leaves the held puck only 2-6 mm over a standing
        # neighbour (60 mm tall) on the sweep to carry; 100 mm above the box top clears it by ~38 mm, 80 mm by ~24 mm
        p_, r_ = b.ik.fk(b.q_ref.cpu().numpy())
        for clear in LIFT_CLEARS:  # highest reachable: the far side of start_1 is near the arm's reach at 100 mm
            z_safe = bc.BOX_H + clear + bc.GRASP_Z - b.ik.base_in_root()[2]
            if p_[2] >= z_safe - 0.005:
                break
            # 5 joints: the same tool pose is not reachable everywhere higher up; let the orientation give way a little
            q_up, e_p, e_a = b.ik.solve(np.array([p_[0], p_[1], z_safe]), r_, [b.q_ref.cpu().numpy()], w_ang=0.2)
            if e_p < 6e-3 and e_a < math.radians(8.0):
                b.move(torch.tensor(q_up, dtype=torch.float32, device=self.dev), CLOSE, 0.0, f"3 lift clear of the other pucks ({1000 * clear:.0f} mm)")
                break
        else:
            print("[demo] 3 lift clear: not reachable, straight to carry", flush=True)
        b.move(carry, CLOSE, 0.0, "3 carry pose (beside the robot)")
        dbg("carry")
        b.move(q_over, CLOSE, 0.0, f"4 load: over slot {slot}")
        dbg("over")
        b.move(q_low, CLOSE, SETTLE_S, "4 load: lower into the slot")
        dbg("low")
        b.dwell(OPEN, RELEASE_S, "4 load: release")
        b.move(q_lift, OPEN, 0.0, "4 load: lift out")
        if to_carry:
            b.move(self.carry[1 if sy >= 0 else -1], OPEN, 0.0, "4 load: back to carry (next puck)")
        else:
            b.move(STOW, OPEN, 0.0, "4 load: back to stow")
        pb = b.to_base(self.puck.data.root_pos_w.torch[0])
        ok = abs(pb[0].item() - sx) < 0.02 and abs(pb[1].item() - sy) < 0.02 and abs(pb[2].item() - sz) < 0.015
        print(f"[demo] load {'OK' if ok else 'FAILED'} (puck {1000 * (pb[0].item() - sx):+.1f}, {1000 * (pb[1].item() - sy):+.1f} mm off the slot centre)")
        return ok

    # ---- segment 6-8: take it out of the slot
    def unload(self, slot):
        b = self.bot
        b.new_skill()
        sx, sy, sz = SLOTS[slot]
        k = "box" if self.is_box else "cyl"
        q_over, q_g = ST[f"{k}_q_over"][slot], ST[f"{k}_q_grasp"][slot]  # slot table; pole-side slots grasp 8 mm off the pole
        if args.debug:
            pp = b.to_base(self.puck.data.root_pos_w.torch[0])
            _, bq_ = b.base()
            pyaw = math.degrees(mu.euler_xyz_from_quat(mu.quat_mul(mu.quat_inv(bq_), self.puck.data.root_quat_w.torch))[2][0].item())
            print(f"[debug] unload slot {slot}: puck yaw {pyaw:.0f} deg (box faces every 90)")
            print(f"[debug] unload slot {slot}: puck {[round(1000 * v) for v in pp.tolist()]} mm vs slot "
                  f"{[round(1000 * v) for v in (sx, sy, sz)]}, tilt {math.degrees(math.acos(max(-1.0, min(1.0, self.up_z())))):.0f} deg")
        carry = self.carry[1 if sy >= 0 else -1]
        b.move(carry, OPEN, 0.0, "6 unload: carry pose (stow -> slot straight sweeps the basket)")
        b.move(q_over, OPEN, 0.0, f"6 unload: over slot {slot}")
        dbg = lambda tag: args.debug and print(f"[debug] {tag}: puck {[round(1000 * v) for v in (b.to_base(self.puck.data.root_pos_w.torch[0]) - torch.tensor([sx, sy, sz], device=self.dev)).tolist()]} mm off slot, "
                                              f"ee {[round(1000 * v) for v in (b.ee()[0] - torch.tensor([sx, sy, sz], device=self.dev)).tolist()]}, "
                                              f"grip q {[round(1000 * v, 1) for v in b.robot.data.joint_pos.torch[0, b.grip].tolist()]} mm, f {[round(v, 1) for v in b.robot.data.applied_torque.torch[0, b.grip].tolist()]} N")
        dbg("over slot")
        b.move(q_g, OPEN, SETTLE_S, "7 unload: down to the puck")
        dbg("down")
        b.dwell(CLOSE, CLOSE_S, "7 unload: close")
        dbg("closed")
        b.move(q_over, CLOSE, 0.0, "7 unload: lift")
        dbg("lifted")
        b.move(carry, CLOSE, 0.5, "8 carry pose")
        ok = self.in_hand()
        print(f"[demo] unload {'OK' if ok else 'FAILED'}")
        return ok

    # ---- segment 9: place on the dice cell, residual policy on the scripted descent (mirrors place_env.PlaceEnv)
    def place(self, cell):
        b, cfg, dev = self.bot, pe.PlaceEnvCfg(), self.dev
        b.new_skill()
        if args.fast:
            cfg.vel_scale, cfg.settle_s = 0.6, 0.15
        r, c = divmod(cell, 3)
        bx, by, _ = DICE_BOX
        # grid centred on the box; docked heading -y: row r at (r - 1) pitch along the heading (0 = near row),
        # column c at (c - 1) pitch to the robot's left, as in place_env.CELLS
        hx, hy = math.cos(DOCK[2]), math.sin(DOCK[2])
        lx, ly = -hy, hx
        if args.debug:
            dp = self.scene["dice"].data.root_pos_w.torch[0] - self.org
            print(f"[debug] dice box moved {[round(1000 * (dp[i].item() - DICE_BOX[i]), 1) for i in (0, 1)]} mm, "
                  f"yaw {math.degrees(math.remainder(mu.euler_xyz_from_quat(self.scene['dice'].data.root_quat_w.torch)[2][0].item() - DICE_BOX[2], math.pi / 2)):+.2f} deg")
        cell_w = np.array([bx + bc.CELL_PITCH * ((r - 1) * hx + (c - 1) * lx), by + bc.CELL_PITCH * ((r - 1) * hy + (c - 1) * ly), bc.BOX_H])
        cell_true = b.to_base(torch.tensor(cell_w, dtype=torch.float32, device=dev) + self.org)
        aim = cell_true.cpu().numpy().copy()
        aim[:2] += np.random.uniform(-cfg.cell_noise_m, cfg.cell_noise_m, 2)
        d_meas = self.puck_bottom_in_ee().cpu().numpy()
        d_meas[:2] += np.random.uniform(-cfg.hand_noise_m, cfg.hand_noise_m, 2)
        rot, hd, yaw = bc.place_rot(aim)
        seeds = bc.place_seeds(hd, yaw)
        q_above, *_ = b.ik.solve(aim + np.array([0, 0, cfg.above_m]) - rot @ d_meas, rot, seeds)
        q_down, *_ = b.ik.solve(aim + np.array([0, 0, cfg.place_m]) - rot @ d_meas, rot, [q_above])
        q_above = torch.tensor(q_above, dtype=torch.float32, device=dev)
        q_down = torch.tensor(q_down, dtype=torch.float32, device=dev)
        b.move(q_above, CLOSE, 0.0, f"9 place: over dice cell {cell}")

        # place phases exactly as the training env: hold, down, settle, release, up, wait (2 s stillness check)
        t_move = bc.trapezoid_duration((q_down - q_above).abs().max().item(), cfg.vel_scale)
        t_ph = np.cumsum([cfg.hold_s, t_move, cfg.settle_s, cfg.release_s, t_move, 2.0])
        gap_q = bc.gap_joint(cfg.release_gap_mm)
        cam_bias = torch.zeros(3, device=dev)
        cam_bias[:2] = torch.empty(2, device=dev).uniform_(-cfg.cam_bias_m, cfg.cam_bias_m)
        prev_act = torch.zeros(5, device=dev)
        b.keep_delta = sees_corr(self.policy, pe.OBS_DIM)
        t = 0.0
        after_up = self.puck.data.root_pos_w.torch[0].clone()
        print(f"[demo] 9 place: {'residual policy on' if self.policy is not None else 'residual 0'}", flush=True)
        while t < t_ph[-1] - 1e-9:
            ph = int(np.sum(t >= t_ph))
            # observation (place_env.PlaceEnv._get_observations)
            q = b.robot.data.joint_pos.torch[0, b.arm]
            qd = b.robot.data.joint_vel.torch[0, b.arm]
            onehot = torch.nn.functional.one_hot(torch.tensor(min(ph, 5), device=dev), 6).float()
            start = 0.0 if ph == 0 else t_ph[min(ph, 6) - 1]
            prog = min(max(t - start, 0.0), 2.0)
            bottom = b.to_base(self.puck.data.root_pos_w.torch[0])
            cam = cell_true - bottom + cam_bias + cfg.cam_noise_m * torch.randn(3, device=dev)
            obs = torch.cat([q - b.q_ref, q_down - b.q_ref, 0.1 * qd, b.q_ref - q_above, prev_act, onehot,
                             torch.tensor([prog], device=dev), 100.0 * cam[:2]]).float()
            if b.keep_delta:
                obs = torch.cat([obs, 10.0 * b.corr])
            if ph < 3:
                a = torch.zeros(5, device=dev)
                if self.policy is not None:
                    with torch.inference_mode():
                        a = self.policy(obs[None])[0].clamp(-1, 1)
                b.delta = cfg.residual_rad * a
                prev_act = a.clone()
            for _ in range(DECIM):
                p = int(np.sum(t >= t_ph))
                s_down = bc.move_s(t - t_ph[0], t_ph[1] - t_ph[0])
                s_up = bc.move_s(t - t_ph[3], t_ph[4] - t_ph[3])
                b.q_ref = q_down + s_up * (q_above - q_down) if p >= 3 else q_above + s_down * (q_down - q_above)
                b.g_ref = gap_q if p >= 3 else CLOSE
                b.step(holding=p in (0, 2, 3, 5))
                t += DT
                if p == 4 and int(np.sum(t >= t_ph)) == 5:
                    after_up = self.puck.data.root_pos_w.torch[0].clone()
        b.delta, b.keep_delta = torch.zeros(5, device=dev), False
        bottom = b.to_base(self.puck.data.root_pos_w.torch[0])
        err = (bottom[:2] - cell_true[:2]).norm().item()
        drift = (self.puck.data.root_pos_w.torch[0] - after_up).norm().item()
        up = mu.quat_apply(self.puck.data.root_quat_w.torch, torch.tensor([[0.0, 0.0, 1.0]], device=dev))[0]
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, up[2].item()))))
        on_box = abs(bottom[2].item() - cell_true[2].item()) < 0.005
        ok = (not self.in_hand()) and on_box and tilt < 5.0 and drift < 0.002 and err < 0.010
        print(f"[demo] place {'OK' if ok else 'FAILED'}: centre error {1000 * err:.1f} mm, tilt {tilt:.1f} deg, "
              f"moved {1000 * drift:.1f} mm in 2 s")
        return ok, err

    def use(self, name):
        self.puck = self.scene[name]
        self.is_box = "_box_" in PUCKS[name]

    def up_z(self):
        return mu.quat_apply(self.puck.data.root_quat_w.torch, torch.tensor([[0.0, 0.0, 1.0]], device=self.dev))[0, 2].item()

    def pick_load(self, slot, to_carry=False):
        b = self.bot
        for attempt in range(3):  # missed grasp: the wrist camera sees an empty hand -> open, look again, retry
            if self.pick():
                break
            if self.up_z() < math.cos(math.radians(30.0)):
                print("[demo] puck knocked over: a top-down grasp cannot take it, cycle stops")
                return False
            b.dwell(OPEN, 0.3, "2 pick: retry")
        else:
            return False
        if not self.load(slot, to_carry):
            return False
        return True

    # ---- stand a lying puck up on the start box (right_env.RightEnv, hang mode, residual policy)
    def stand_up(self, regrasp=False):
        """regrasp: after the hand opens, grasp the standing puck in place (right_env regrasp=1: wrist kept, straight
        down to the pick grasp depth, close at full force, lift) and end holding it, instead of lifting away."""
        b, cfg, dev = self.bot, re_.RightEnvCfg(), self.dev
        cfg.regrasp = int(regrasp)
        z = torch.tensor([[0.0, 0.0, 1.0]], device=dev)
        ax_w = mu.quat_apply(self.puck.data.root_quat_w.torch, z)[0]
        centre_w = self.puck.data.root_pos_w.torch[0] + ax_w * re_.PUCK_L / 2 - self.org
        # no re-dock: stand it up from where the first puck was picked (the waypoints are solved for its real place;
        # backing out and docking again shoved the start box)
        # perception: centre +-5 mm, axis yaw +-10 deg (as training)
        _, bq = b.base()
        ax_b = mu.quat_apply_inverse(bq, ax_w[None])[0]
        yaw_meas = max(-math.pi / 2, min(math.pi / 2, math.atan2(ax_b[1].item(), ax_b[0].item()) + np.random.uniform(-cfg.yaw_noise, cfg.yaw_noise)))
        c = centre_w.clone() + self.org
        c[2] = bc.BOX_H + self.org[2]
        centre_b = b.to_base(c).cpu().numpy()
        centre_b[:2] += np.random.uniform(-cfg.pos_noise_m, cfg.pos_noise_m, 2)
        qs, _ = re_.plan_waypoints(b.ik, centre_b, yaw_meas, cfg, is_box=self.is_box)
        if regrasp:  # last 2: grasp / lift of the regrasp
            rg, qs = torch.tensor(np.array(qs[-2:]), dtype=torch.float32, device=dev), qs[:-2]
        wp = torch.tensor(np.array(qs), dtype=torch.float32, device=dev)
        b.new_skill()
        q_start = b.q_ref.clone()
        mv = lambda qa, qb: bc.trapezoid_duration((qa - qb).abs().max().item(), cfg.vel_scale)
        t_tr = mv(wp[0], q_start)
        groups = re_.arc_groups(cfg, len(qs) - 3)
        arc_dur = re_.arc_durations(wp[None], groups, cfg.arc_vel_scale or cfg.vel_scale, cfg.hang_s)
        seg = [t_tr + cfg.hold_s, mv(wp[1], wp[0]), cfg.close_s, arc_dur.sum().item(), cfg.settle_s, cfg.open_s,
               mv(wp[-1], wp[-2]), cfg.wait_s]
        if regrasp:  # 6: down to the grasp, hold (sag integral), close; 7: lift, wait (right_env._reset_idx)
            rg_d = mv(rg[0], wp[-2])
            seg[6], seg[7] = rg_d + cfg.hold_s + CLOSE_S, mv(rg[1], rg[0]) + cfg.wait_s
        t_ph = np.cumsum(seg)
        t_close = t_ph[6] - CLOSE_S
        # current-limited pinch as in training (force_grip): close fully, drive force capped at pinch_force
        pinch_q = 0.0 if cfg.force_grip else bc.GRIP_STOP * (1 - cfg.pinch_force / bc.GRIP_FORCE)
        env0 = torch.tensor([0], device=dev)
        lim0 = b.robot.data.joint_effort_limits.torch[0, b.grip].clone()
        if cfg.force_grip:
            b.robot.write_joint_effort_limit_to_sim_index(limits=torch.full((1, len(b.grip)), cfg.pinch_force, device=dev),
                                                          joint_ids=b.grip, env_ids=env0)
        # neighbour: the other start_1 puck if it is still standing on the box (observation as right_env._nb_rel)
        other = self.scene["puck_a" if self.puck is self.scene["puck_b"] else "puck_b"]
        nb_on = abs(other.data.root_pos_w.torch[0, 2].item() - self.org[2].item() - bc.BOX_H) < 0.01
        cam_bias = torch.zeros(3, device=dev)
        cam_bias[:2] = torch.empty(2, device=dev).uniform_(-cfg.cam_bias_m, cfg.cam_bias_m)
        prev_act = torch.zeros(5, device=dev)
        print(f"[demo] R stand up: lying puck yaw {math.degrees(yaw_meas):+.0f} deg (measured), pinch {cfg.pinch_force:.1f} N", flush=True)

        def nb_obs():
            if not nb_on:
                return torch.zeros(3, device=dev)
            c = self.puck.data.root_pos_w.torch[0] + mu.quat_apply(self.puck.data.root_quat_w.torch, z * re_.PUCK_L / 2)[0]
            rel = (b.to_base(other.data.root_pos_w.torch[0]) - b.to_base(c))[:2]
            return torch.cat([10.0 * rel, torch.ones(1, device=dev)])

        def ref(t):  # right_env.RightEnv._ref for one env
            ph = int(np.sum(t >= t_ph))
            s = lambda i0, i1: bc.move_s(t - t_ph[i0], max(t_ph[i1] - t_ph[i0], 1e-3))
            if ph == 0:
                q = q_start + bc.move_s(t, max(t_tr, 1e-3)) * (wp[0] - q_start) if t < t_tr else wp[0]
            elif ph == 1:
                q = wp[0] + s(0, 1) * (wp[1] - wp[0])
            elif ph == 2:
                q = wp[1]
            elif ph == 3:
                q = re_.arc_q(wp[None], groups, arc_dur, torch.tensor([float(t - t_ph[2])], device=dev))[0]
            elif ph in (4, 5):
                q = wp[-2]
            elif regrasp:  # right_env.RightEnv._ref, regrasp branch
                t_up = t_ph[7] - cfg.wait_s
                if ph == 6:
                    q = wp[-2] + bc.move_s(t - t_ph[5], max(rg_d, 1e-3)) * (rg[0] - wp[-2])
                else:
                    q = rg[0] + bc.move_s(t - t_ph[6], max(t_up - t_ph[6], 1e-3)) * (rg[1] - rg[0])
                g = CLOSE if t >= t_close else OPEN
                hold = (ph == 6 and t >= t_ph[5] + rg_d) or (ph >= 7 and t >= t_up)
                return q, g, hold, ph
            elif ph == 6:
                q = wp[-2] + s(5, 6) * (wp[-1] - wp[-2])
            else:
                q = wp[-1]
            g = pinch_q if 2 <= ph <= 4 else OPEN
            hold = (ph == 0 and t >= t_tr) or ph in (2, 4, 5, 7)
            return q, g, hold, ph

        b.keep_delta = sees_corr(self.right_policy, re_.OBS_DIM)
        t = 0.0
        box0, root0 = self.scene["start"].data.root_pos_w.torch[0].clone(), b.root_pose2d()
        trace, probed, obs_seen = [], set(), set()
        replanned = False
        while t < t_ph[-1] - 1e-9:
            ph = int(np.sum(t >= t_ph))
            if args.debug and ph in (1, 2, 3, 4) and ph not in probed:
                probed.add(ph)
                bp_, bq_ = b.base()
                g_ = mu.quat_apply_inverse(bq_, torch.tensor([[0.0, 0.0, -1.0]], device=dev))[0]
                print(f"[probe] phase {ph}: base_link z {1000 * (bp_[0, 2] - self.org[2]).item():.1f} mm, "
                      f"tilt {math.degrees(math.acos(max(-1.0, min(1.0, -g_[2].item())))):.2f} deg, ee_b {[round(1000 * v, 1) for v in b.ee()[0].tolist()]}, "
                      f"grip {[round(v, 2) for v in b.robot.data.applied_torque.torch[0, b.grip].tolist()]} N, axis z {self.up_z():.2f}, "
                      f"centre_b {[round(1000 * v, 1) for v in centre_b.tolist()]}", flush=True)
            if args.debug:
                trace.append((ph, (self.scene["start"].data.root_pos_w.torch[0] - box0)[:2].norm().item(),
                              math.hypot(b.root_pose2d()[0] - root0[0], b.root_pose2d()[1] - root0[1]), self.up_z()))
            # observation (right_env.RightEnv._get_observations)
            q = b.robot.data.joint_pos.torch[0, b.arm]
            qd = b.robot.data.joint_vel.torch[0, b.arm]
            onehot = torch.nn.functional.one_hot(torch.tensor(min(ph, 7), device=dev), 8).float()
            start = 0.0 if ph == 0 else t_ph[min(ph, 7) - 1]
            prog = min(max(t - start, 0.0), 3.0)
            rel = b.to_base(self.puck.data.root_pos_w.torch[0]) - b.ee()[0]
            cam = rel + cam_bias + cfg.cam_noise_m * torch.randn(3, device=dev)
            _, bq = b.base()
            axis = mu.quat_apply_inverse(bq, mu.quat_apply(self.puck.data.root_quat_w.torch, z))[0]
            obs = torch.cat([q - b.q_ref, wp[-2] - b.q_ref, 0.1 * qd, prev_act, onehot, torch.tensor([prog], device=dev),
                             100.0 * cam, axis + 0.03 * torch.randn(3, device=dev),
                             torch.tensor([float(self.is_box)], device=dev), b.q_ref - wp[1], nb_obs()]).float()
            if b.keep_delta:
                obs = torch.cat([obs, 10.0 * b.corr])
            if args.debug and ph not in obs_seen:
                obs_seen.add(ph)
                print(f"[obs] kind {int(self.is_box)} phase {ph} t {t:.2f}: " + " ".join(f"{v:.3f}" for v in obs.tolist()), flush=True)
            with torch.inference_mode():
                a = self.right_policy(obs[None])[0].clamp(-1, 1) * (0.0 if args.right_zero else 1.0)
            if 1 <= ph <= 4:
                b.delta = cfg.residual_rad * a
            prev_act = a.clone()
            if regrasp and ph == 6 and not replanned:
                # hand open over the standing puck: the gripper camera measures it again (training pick camera
                # bias, +-2 mm) and the grasp is re-solved there, same wrist; the stand-up plan alone was ~8 mm off
                # (its perception error), enough for the puck to swing in the fingers on the way to the basket
                replanned = True
                b.delta = torch.zeros(5, device=dev)  # the stand-up residual was frozen here (up to 0.1 rad: ~10 mm off at the grasp)
                m = b.to_base(self.puck.data.root_pos_w.torch[0]).cpu().numpy()
                m[:2] += np.random.uniform(-pk.PickEnvCfg().cam_bias_m, pk.PickEnvCfg().cam_bias_m, 2)
                _, r_rg = b.ik.fk(rg[0].cpu().numpy())
                g_ = np.array([m[0], m[1], centre_b[2] + bc.GRASP_Z])
                for i, dz in enumerate((0.0, cfg.regrasp_lift_m)):
                    q_, e_p, e_a = b.ik.solve(g_ + [0, 0, dz], r_rg, [rg[i].cpu().numpy()])
                    if e_p < 2e-3 and e_a < math.radians(2.0):
                        rg[i] = torch.tensor(q_, dtype=torch.float32, device=dev)
            if args.debug and regrasp and t_close - DT * DECIM <= t < t_close:
                print(f"[debug] regrasp before close: puck bottom in ee {[round(1000 * v, 1) for v in self.puck_bottom_in_ee().tolist()]} mm, "
                      f"ee - plan {[round(1000 * v, 1) for v in (b.ee()[0].cpu() - torch.tensor(b.ik.fk(rg[0].cpu().numpy())[0], dtype=torch.float32)).tolist()]} mm, "
                      f"corr {[round(v, 4) for v in b.corr.tolist()]}", flush=True)
            for _ in range(DECIM):
                if regrasp and cfg.force_grip and t_close <= t < t_close + DT:  # full grip force for the regrasp close
                    b.robot.write_joint_effort_limit_to_sim_index(limits=lim0[None], joint_ids=b.grip, env_ids=env0)
                b.q_ref, b.g_ref, hold, _ = ref(t)
                b.step(holding=hold)
                t += DT
        b.delta, b.keep_delta = torch.zeros(5, device=dev), False
        b.robot.write_joint_effort_limit_to_sim_index(limits=lim0[None], joint_ids=b.grip, env_ids=env0)  # full grip again
        if args.debug:  # per phase: start box moved, robot root moved [mm], puck axis z (max over the phase)
            for p_ in range(8):
                rows = [r for r in trace if r[0] == p_]
                if rows:
                    print(f"[debug] R phase {p_}: box {1000 * max(r[1] for r in rows):.1f} mm, root {1000 * max(r[2] for r in rows):.1f} mm, "
                          f"axis z max {max(r[3] for r in rows):.2f}")
        up = self.up_z()
        on_box = abs(self.puck.data.root_pos_w.torch[0, 2].item() - self.org[2].item() - bc.BOX_H) < 0.005
        if regrasp:
            lifted = self.puck.data.root_pos_w.torch[0, 2].item() - self.org[2].item() - bc.BOX_H > 0.03
            ok = lifted and self.in_hand()
            print(f"[demo] stand up + regrasp in place {'OK' if ok else 'FAILED'}: tilt {math.degrees(math.acos(max(-1.0, min(1.0, up)))):.1f} deg, "
                  f"{'lifted' if lifted else 'not lifted'}, {'in' if self.in_hand() else 'not in'} the hand")
            return ok
        ok = up > math.cos(math.radians(5.0)) and on_box and not self.in_hand()
        print(f"[demo] stand up {'OK' if ok else 'FAILED'}: tilt {math.degrees(math.acos(max(-1.0, min(1.0, up)))):.1f} deg, "
              f"{'on' if on_box else 'off'} the box top")
        return ok

    def run(self):
        b = self.bot
        x0, y0, yaw0 = b.root_pose2d()
        box0 = self.scene["start"].data.root_pos_w.torch[0].clone()
        if abs(x0 - START_DOCK[0]) < 0.02 and y0 < START_DOCK[1] and abs(math.remainder(yaw0 - START_DOCK[2], 2 * math.pi)) < math.radians(15):
            # already lined up in front of start_1 (robot placed there in scene_lab.usda): straight in, no turning
            b.drive_to(*START_DOCK, label="0 drive: straight to the dock on the south face of start_1")
        else:
            b.drive_to(x0, VIA_Y, math.pi, label="0 drive: south of start_1")
            b.drive_to(START_DOCK[0], VIA_Y, math.pi / 2, label="0 drive: under start_1")
            b.drive_to(*START_DOCK, label="0 drive: dock on the south face of start_1")
        moved = (self.scene["start"].data.root_pos_w.torch[0] - box0)[:2].norm().item()
        print(f"[demo] docked; start box moved {1000 * moved:.1f} mm", flush=True)
        self.use(self.puck_name)  # the last cycle ended on the second puck
        jobs = [(self.puck_name, args.slot, args.cell)]
        if args.show_carry:
            ok = self.pick()
            b.move(self.carry[1 if SLOTS[args.slot][1] >= 0 else -1], CLOSE, 1.0, "3 carry pose (beside the robot)")
            print(f"[demo] carry joints deg {[round(math.degrees(v), 2) for v in b.robot.data.joint_pos.torch[0, b.arm].tolist()]}")
            return ok, float("nan")
        if args.stand_only:
            self.use("puck_b")
            if args.stand_only == "carry":
                b.move(self.carry[1 if SLOTS[args.slot][1] >= 0 else -1], OPEN, 0.0)
            ok = self.stand_up(regrasp=args.then_pick or args.then_load)  # stood up, grasped in place and lifted
            if ok and args.then_load:
                ok = self.load(args.slot2)
            return ok, float("nan")
        if not self.pick_load(args.slot, to_carry=args.mixed):  # --mixed: the lying puck follows
            return False, float("nan")
        if args.mixed:
            self.use("puck_b")
            if not self.stand_up(regrasp=True):  # stood up and grasped in place
                return False, float("nan")
            if not self.load(args.slot2):
                return False, float("nan")
            jobs.append(("puck_b", args.slot2, args.cell2))
        if args.load_only:
            return True, float("nan")
        b.drive_to(START_DOCK[0], VIA_Y, math.pi / 2, reverse=True, label="5 drive: back off from the start box")
        # line up 0.15 m out on the face normal first: an angled approach catches the box corner and shoves it
        b.drive_to(DOCK[0] - 0.15 * math.cos(DOCK[2]), DOCK[1] - 0.15 * math.sin(DOCK[2]), DOCK[2], label="5 drive: in front of the dice box")
        b.drive_to(DOCK[0], DOCK[1], DOCK[2], label="5 drive: dock at the dice box")
        x, y, yaw = b.root_pose2d()
        print(f"[demo] docked {1000 * (x - DOCK[0]):+.0f}, {1000 * (y - DOCK[1]):+.0f} mm, {math.degrees(yaw - DOCK[2]):+.1f} deg off the dock pose")
        errs = []
        for name, slot, cell in jobs:
            self.use(name)
            if not self.unload(slot):
                return False, float("nan")
            ok, err = self.place(cell)
            errs.append(err)
            if not ok:
                return False, err
        return True, max(errs)


def reset_scene(scene, bot):
    """Robot, pucks and joints back to the scene_lab.usda state."""
    robot = scene["robot"]
    root = robot.data.default_root_pose.torch.clone()
    root[:, :3] += scene.env_origins
    robot.write_root_pose_to_sim_index(root_pose=root)
    robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros(1, 6, device=root.device))
    jp = robot.data.default_joint_pos.torch.clone()
    stow = torch.tensor(STOW, device=root.device)
    arm = robot.find_joints("joint[1-5]", preserve_order=True)[0]
    jp[:, arm] = stow
    robot.write_joint_state_to_sim_index(position=jp, velocity=torch.zeros_like(jp))
    for name in (*PUCKS, "dice", "start"):
        p = scene[name]
        pose = p.data.default_root_pose.torch.clone()
        pose[:, :3] += scene.env_origins
        p.write_root_pose_to_sim_index(root_pose=pose)
        p.write_root_velocity_to_sim_index(root_velocity=torch.zeros(1, 6, device=root.device))
    robot.set_joint_position_target_index(target=jp[:, arm], joint_ids=arm)
    scene.write_data_to_sim()
    bot.reset_state(stow)  # not joint_pos: its buffer still holds the last cycle's pose until the next step


def main():
    if args.seed >= 0:
        np.random.seed(args.seed)
        torch.manual_seed(args.seed)
    eye, target = ((-0.80, -3.00, 0.60), (-1.26, -2.62, 0.15)) if args.show_carry else (EYE, TARGET)
    sim_cfg = sim_utils.SimulationCfg(dt=DT, render_interval=DECIM, physics=PhysxCfg(), device=args.device)  # --device cpu: CPU PhysX
    sim_cfg.default_visualizer_cfg = VisualizerCfg(eye=eye, lookat=target)
    sim = sim_utils.SimulationContext(sim_cfg)
    import carb  # noqa: E402
    carb.settings.get_settings().set("/rtx/rendermode", args.render)  # after the context: it applies its own render settings
    scene = InteractiveScene(DemoSceneCfg(num_envs=1, env_spacing=4.0))
    sim.stage.GetPrimAtPath(scene.env_prim_paths[0] + "/demo/beebot").SetActive(False)
    # the traffic sign's meshes carry no points in this scene (the newton viewer fails on them); the cycle never uses it
    sim.stage.GetPrimAtPath(scene.env_prim_paths[0] + "/demo/sign").SetActive(False)
    if args.mixed:  # its spot on start_1 now holds the cylinder
        sim.stage.GetPrimAtPath(scene.env_prim_paths[0] + "/demo/pucks/" + SPOTS["puck_a"]).SetActive(False)
    sim.reset()
    sim.set_camera_view(eye=eye, target=target)
    cam_path = f"{scene.env_prim_paths[0]}/demo/{args.view}/Camera"
    if args.view != "free" and not args.show_carry and sim.stage.GetPrimAtPath(cam_path).IsValid():
        try:  # Kit window only (no viewport when headless or with the newton viewers)
            from omni.kit.viewport.utility import get_active_viewport
            vp = get_active_viewport()
            if vp is not None:
                vp.camera_path = cam_path
        except ImportError:
            pass
    # write the shape materials back through the tensor API, as the training envs' material events do: with the
    # same values PhysX then handles the puck / finger contacts like in training (stand-up Δ=0 at one pose: USD-bound
    # materials 64 %, rewritten 92 %; 2026-11-01)
    for name in ("robot", *PUCKS):
        v = scene[name].root_physx_view
        v.set_material_properties(v.get_material_properties(), wp.from_torch(torch.arange(scene.num_envs, dtype=torch.int32)))
    demo = Demo(sim, scene)
    results = []
    for run in range(args.runs):
        reset_scene(scene, demo.bot)
        for _ in range(int(0.5 / DT)):  # settle on the floor
            demo.bot.step(holding=True)
        print(f"[demo] ===== cycle {run + 1}/{args.runs} =====", flush=True)
        if args.debug:
            for nm in ("puck_a", "puck_b"):
                pp = scene[nm].data.root_pos_w.torch[0] - scene.env_origins[0]
                print(f"[debug] {nm} at {[round(1000 * v) for v in pp.tolist()]} mm")
            print(f"[debug] root {demo.bot.root_pose2d()}, arm deg {[round(math.degrees(v)) for v in scene['robot'].data.joint_pos.torch[0, demo.bot.arm].tolist()]}")
        k0 = demo.bot.k
        results.append(demo.run())
        print(f"[demo] cycle time {(demo.bot.k - k0) * DT:.1f} s (sim time)", flush=True)
    ok = [r[0] for r in results]
    errs = [1000 * r[1] for r in results if r[0] or not math.isnan(r[1])]
    print(f"[demo] cycles OK {sum(ok)}/{len(ok)}" + (f", place centre error mm median {np.median(errs):.1f} max {np.max(errs):.1f}" if errs else ""))
    if args.hold:
        print("[demo] holding the last pose, close the window to exit")
        while app.is_running():
            demo.bot.step(holding=True)


if __name__ == "__main__":
    main()
    app.close()
