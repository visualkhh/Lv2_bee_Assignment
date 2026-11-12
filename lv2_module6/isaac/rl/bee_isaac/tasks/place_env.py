"""Bee-Place-Residual: residual RL for the dice cell place (segment 9 of the RL task split, skill place_pip).

Episode = the puck is already in the hand above a dice cell (end of the transit segment) ->
    hold (sag settles) -> down to the cell -> settle -> open to 36 mm -> up -> wait, then judge.
The base motion is the scripted one of cycle_sm.py --place: joint targets from offline IK aimed at a cell position
with perception error (cell +-5 mm, puck in hand +-2 mm), plus the sag integral of cycle_sm.py while holding.
The policy adds a joint residual (|delta| <= 0.05 rad) until the fingers open; after that the residual is frozen.
The policy also sees a gripper-camera style measurement of the cell relative to the puck (noisy, biased per episode),
which the base plan did not use. Critic gets the true state (asymmetric actor-critic).

Only the near and middle rows: the far row tips the robot over (report 4, mechanical, not a control problem).
"""

from __future__ import annotations

import math
from pathlib import Path

import gymnasium as gym
import numpy as np
import torch
import warp as wp
from isaaclab_physx.physics import PhysxCfg

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg, AssetBaseCfg, RigidObject, RigidObjectCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from bee_isaac import common as bc
from bee_isaac.assets.beebot import robot_cfg
from bee_isaac.constants import CUMOTION_DIR

BOX_NEAR_X = 0.08  # box near edge in the robot root frame (robot front at 0.069)
BOX_CX = BOX_NEAR_X + bc.BOX_HALF
CELLS = [(BOX_CX + bc.CELL_PITCH * (r - 1), bc.CELL_PITCH * (c - 1)) for r in (0, 1) for c in range(3)]  # near, mid rows
OBS_DIM, CRITIC_EXTRA = 34, 14


def randomize_mass_synced(env, env_ids, asset_cfg: SceneEntityCfg, scale_range: tuple[float, float]):
    """Scale body mass and inertia per env, written synchronously.

    Lab 6.1 (PhysX) set_masses_index hands PhysX CPU buffers before the GPU -> CPU copy is done: env 0 sometimes got
    mass 0 and fell through the floor (2026-10-27 report 7). Same workaround as feature/isaac-pick bee_env_cfg.py.
    """
    asset = env.scene[asset_cfg.name]
    n = env.scene.num_envs
    m0 = asset.data.body_mass.torch.clone()
    i0 = asset.data.body_inertia.torch.clone()
    lo, hi = scale_range
    scale = lo + (hi - lo) * torch.rand(n, 1, device=m0.device)
    mass, inertia = (m0 * scale).float(), (i0 * scale[..., None]).float()
    ids = wp.from_torch(torch.arange(n, dtype=torch.int32))
    wp.synchronize()
    asset.root_view.set_masses(wp.from_torch(mass.cpu().contiguous()), indices=ids)
    asset.root_view.set_inertias(wp.from_torch(inertia.cpu().reshape(-1).contiguous()), indices=ids)
    wp.copy(asset.data._body_mass, wp.from_torch(mass.contiguous()))
    wp.copy(asset.data._body_inertia, wp.from_torch(inertia.contiguous()))


# gravity::torques of ros2_ws/src/ml/include/ml/rl_arm_correction.hpp (planar arm, masses and centres of mass from the URDF)
_G = 9.81
_M_UA, _UA_X, _UA_Z = 0.128865, 0.008487, 0.104997
_M_FA, _FA_X = 0.117300, 0.093796
_M_WR, _WR_X, _WR_Z = 0.090152, 0.043623, 0.011449
_M_HD, _HD_X, _HD_Z = 0.371242, 0.052686, 0.014472
_J3_X, _J3_Z, _J4_X, _HAND_X = 0.024, 0.128, 0.124, 0.0629


def _rotx(th, x, z):  # x of the vector (x, z) turned by pitch th
    return x * torch.cos(th) + z * torch.sin(th)


def _gravity_torques(q2, q3, q4):
    """Gravity torque [Nm] about joints 2 and 3 (forward sag +), shape [n, 2]."""
    t23, t234 = q2 + q3, q2 + q3 + q4
    ua, p3 = _rotx(q2, _UA_X, _UA_Z), _rotx(q2, _J3_X, _J3_Z)
    p4x = p3 + _rotx(t23, _J4_X, 0.0)
    fax = p3 + _rotx(t23, _FA_X, 0.0)
    wr, hd = p4x + _rotx(t234, _WR_X, _WR_Z), p4x + _rotx(t234, _HAND_X + _HD_X, _HD_Z)
    return torch.stack([_G * (_M_UA * ua + _M_FA * fax + _M_WR * wr + _M_HD * hd),
                        _G * (_M_FA * (fax - p3) + _M_WR * (wr - p3) + _M_HD * (hd - p3))], dim=1)


@configclass
class PlaceSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
    light = AssetBaseCfg(prim_path="/World/DomeLight", spawn=sim_utils.DomeLightCfg(intensity=1000.0, exposure=1.25))
    robot: ArticulationCfg = robot_cfg().replace(prim_path="{ENV_REGEX_NS}/beebot")
    box = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/box",
        spawn=sim_utils.UsdFileCfg(usd_path=str(bc.USD_DIR / "shelf" / "shelf_L3.usda"), scale=(0.205 / 0.207, 0.205 / 0.207, 1.0),
                                   rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True)),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(BOX_CX, 0.0, 0.0)),
    )
    puck = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/puck",
        spawn=sim_utils.MultiUsdFileCfg(usd_path=bc.PUCK_USDS),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(BOX_CX, 0.0, bc.BOX_H + 0.002)),
    )


@configclass
class PlaceEventCfg:
    puck_mass = EventTerm(func=randomize_mass_synced, mode="startup",
                          params={"asset_cfg": SceneEntityCfg("puck"), "scale_range": (0.7, 1.3)})
    puck_material = EventTerm(func=mdp.randomize_rigid_body_material, mode="startup", params={
        "asset_cfg": SceneEntityCfg("puck"), "static_friction_range": (0.3, 0.6), "dynamic_friction_range": (0.25, 0.55),
        "restitution_range": (0.0, 0.4), "num_buckets": 64, "make_consistent": True})
    finger_material = EventTerm(func=mdp.randomize_rigid_body_material, mode="startup", params={
        "asset_cfg": SceneEntityCfg("robot", body_names="finger_.*_link"), "static_friction_range": (0.6, 1.0),
        "dynamic_friction_range": (0.5, 0.9), "restitution_range": (0.0, 0.0), "num_buckets": 64, "make_consistent": True})


@configclass
class PlaceEnvCfg(DirectRLEnvCfg):
    sim: SimulationCfg = SimulationCfg(dt=1 / 200, physics=PhysxCfg())
    decimation = 4  # policy 50 Hz, base motion and latency at 200 Hz
    def __post_init__(self):
        self.sim.render_interval = self.decimation

    episode_length_s = 6.0  # safety cap; episodes end when the wait phase is over
    scene: PlaceSceneCfg = PlaceSceneCfg(num_envs=256, env_spacing=1.0)
    events: PlaceEventCfg = PlaceEventCfg()
    observation_space = OBS_DIM + 5
    state_space = 0
    action_space = 5

    # task
    vel_scale = 0.3  # x team joint limits, as cycle_sm.py
    planner = "ik"  # ik: pinocchio IK + joint interpolation; cumotion: replay cumotion/cache_<task>.npz (B0)
    vel_scale_range: tuple | None = None  # (lo, hi): sampled per episode instead of vel_scale (speed DR)
    settle_range: tuple | None = None  # (lo, hi) [s]: sampled per episode instead of settle_s
    hold_s, settle_s, release_s, wait_s = 0.3, 0.3, 0.4, 0.5
    above_m, place_m, release_gap_mm = 0.05, 0.0, 36.0
    cell_noise_m, hand_noise_m = 0.005, 0.002  # base plan aims with these errors (cycle_sm.py --place defaults)
    hand_offset_m = (0.003, 0.002)  # true puck offset in the hand: along the fingers, vertical
    cam_bias_m, cam_noise_m = 0.002, 0.001  # gripper-camera cell measurement: per-episode bias, per-step noise
    residual_rad = 0.05
    latency = True
    latency_scale_range: tuple | None = None  # (lo, hi): per episode and joint, measured latency x U(lo, hi)
    corr_gain, corr_dt, corr_max = 0.3, 5 / 60.0, 0.2  # sag integral of cycle_sm.py while a target is held
    # (error = ref + residual - q since 2026-11-01, so the integral removes the sag but keeps the residual;
    # the real rl_arm_correction does the same, it was ref - q before, which undid the residual while holding)
    corr_gain_range: tuple | None = None  # (lo, hi): per episode, corr_gain x U(lo, hi)
    sag_scale_range: tuple | None = None  # (lo, hi): per episode, sag x U(lo, hi) by arm joint stiffness / U
    obs_corr = True  # policy also sees the sag integral (last 5 observations; older policies: --obs_dim)
    ff_gain, ff_max = 1.0, 0.2  # gravity feed-forward as rl_arm_correction (gravity_enabled): delta_j = -c_j x tau_j, joints 2-3 (0: off)

    # reward
    w_track, w_tilt, w_rate, w_act = 0.1, 0.2, 0.05, 0.002
    w_success, w_final, w_fail = 10.0, 5.0, 5.0


class PlaceEnv(DirectRLEnv):
    cfg: PlaceEnvCfg
    TASK = "place"  # cumotion cache name

    def __init__(self, cfg: PlaceEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, dev = self.num_envs, self.device
        self.robot: Articulation = self.scene["robot"]
        self.puck: RigidObject = self.scene["puck"]
        self.arm_ids = self.robot.find_joints("joint[1-5]", preserve_order=True)[0]
        self.grip_ids = self.robot.find_joints("gripper_.*_joint")[0]
        self.ee_id = self.robot.find_bodies("gripper_link")[0][0]
        self.base_id = self.robot.find_bodies("base_link")[0][0]
        self.ee_off = torch.tensor(bc.EE_OFFSET_POS, device=dev).repeat(n, 1)
        self.ee_rot = torch.tensor(bc.EE_OFFSET_ROT, device=dev).repeat(n, 1)

        # ---- per-cell nominal plan (offline IK) and its linearization: q = q_nom + P (p - p_nom)
        ik = bc.ArmIK()
        self.base_off = torch.tensor(ik.base_in_root(), device=dev, dtype=torch.float32)  # base_link in the root frame
        d_nom = np.array([0.0, 0.0, bc.GRASP_Z])
        rows = {k: [] for k in ("q_above", "q_down", "P_above", "P_down", "p_above", "p_down", "rot", "cell_b")}
        for cx, cy in CELLS:
            cell_b = np.array([cx, cy, bc.BOX_H]) - ik.base_in_root()
            rot, hd, yaw = bc.place_rot(cell_b)
            seeds = bc.place_seeds(hd, yaw)
            for k, h in (("above", cfg.above_m), ("down", cfg.place_m)):
                p = cell_b + np.array([0.0, 0.0, h]) - rot @ d_nom
                q, e_pos, e_ang = ik.solve(p, rot, seeds)
                assert e_pos < 2e-3 and e_ang < math.radians(2.0), f"IK failed for cell {(cx, cy)} {k}: {e_pos}, {e_ang}"
                seeds = [q]
                rows["q_" + k].append(q)
                rows["P_" + k].append(ik.jac_pos(q))
                rows["p_" + k].append(p)
            rows["rot"].append(rot)
            rows["cell_b"].append(cell_b)
        self.plan = {k: torch.tensor(np.array(v), device=dev, dtype=torch.float32) for k, v in rows.items()}
        self.dn_tab = self.tr_tab = None
        self.tab_idx = torch.zeros(n, dtype=torch.long, device=dev)
        if cfg.planner == "cumotion" and self.TASK == "place":
            c = self._load_tabs("place")
            self.plan["q_above"], self.plan["q_down"] = c["dn_q"][:, 0], c["dn_q"][:, -1]
        # finger closing axis (gripper_link y) in the end effector frame; the puck sits centred along it
        _, r_g = ik.frame_in_ee("gripper_link")
        self.close_axis_ee = torch.tensor(r_g[:, 1], device=dev, dtype=torch.float32)
        self.d_nom = torch.tensor(d_nom, device=dev, dtype=torch.float32)

        # ---- latency: per-joint delay of the joint targets in physics steps
        lat = [*bc.LATENCY_S, bc.LATENCY_GRIP_S] if cfg.latency else [0.0] * 6
        self.lat_s = torch.tensor(lat, device=dev)
        self.lag = torch.round(self.lat_s / self.physics_dt).long().repeat(n, 1)  # (n, 6), resampled by _sample_lag
        hi = cfg.latency_scale_range[1] if cfg.latency_scale_range else 1.0
        self.hist = torch.zeros(int(math.ceil(max(lat) * hi / self.physics_dt)) + 2, n, 6, device=dev)
        self.hist_i = 0

        # ---- per-env episode state
        z = lambda *s: torch.zeros(n, *s, device=dev)
        self.cell = torch.zeros(n, dtype=torch.long, device=dev)
        self.q_above, self.q_down, self.corr, self.delta = z(5), z(5), z(5), z(5)
        self.corr_k = torch.full((n,), cfg.corr_gain, device=dev)
        self.k_arm0 = self.robot.data.joint_stiffness.torch[:, self.arm_ids].clone()
        self.t_ph = z(6)  # phase end times: hold, down, settle, release, up, wait
        self.t = z()
        self.cam_bias = z(3)
        self.aim_err, self.hand_err, self.hand_true = z(3), z(3), z(3)
        self.prev_act = z(5)
        self.q_ref = z(5)
        self.q_start, self.t_tr = z(5), z()  # optional transit q_start -> q_above in phase 0 (residual off, no sag integral)
        self.grip_ref = z()
        self.held0 = torch.ones(n, dtype=torch.bool, device=dev)
        self.in_hand, self.dropped, self.tipped = (torch.zeros(n, dtype=torch.bool, device=dev) for _ in range(3))
        self.act_rate, self.act_mag = z(), z()
        self.puck_after_up = z(3)
        self.success = torch.zeros(n, dtype=torch.bool, device=dev)
        self.final_err = z()
        self.gap_q = bc.gap_joint(cfg.release_gap_mm)
        self.grip_before, self.grip_after = 0.0, self.gap_q  # finger target before / from phase 3 (closed -> release)
        self.is_box = None
        self.stats = {"success": [], "err": [], "dropped": [], "tipped": []}

    # ------------------------------------------------------------------ helpers
    def _kinds(self):
        if self.is_box is None:
            st = self.sim.stage
            self.is_box = torch.tensor(["box" in st.GetPrimAtPath(f"/World/envs/env_{i}/puck").GetPrimStack()[1].layer.identifier
                                        for i in range(self.num_envs)], device=self.device)
        return self.is_box

    def _base(self):
        return self.robot.data.body_pos_w.torch[:, self.base_id], self.robot.data.body_quat_w.torch[:, self.base_id]

    def _to_base(self, pos_w):
        bp, bq = self._base()
        return math_utils.quat_apply_inverse(bq, pos_w - bp)

    def _ee_b(self):
        g_pos, g_quat = self.robot.data.body_pos_w.torch[:, self.ee_id], self.robot.data.body_quat_w.torch[:, self.ee_id]
        _, bq = self._base()
        pos = self._to_base(g_pos + math_utils.quat_apply(g_quat, self.ee_off))
        return pos, math_utils.quat_mul(math_utils.quat_inv(bq), math_utils.quat_mul(g_quat, self.ee_rot))

    def _puck_bottom_b(self):
        return self._to_base(self.puck.data.root_pos_w.torch)

    def _puck_tilt(self):
        _, bq = self._base()
        q_b = math_utils.quat_mul(math_utils.quat_inv(bq), self.puck.data.root_quat_w.torch)
        up = torch.zeros(self.num_envs, 3, device=self.device)
        up[:, 2] = 1.0
        return torch.acos(math_utils.quat_apply(q_b, up)[:, 2].clamp(-1, 1))

    def _in_hand(self):
        up = torch.zeros(self.num_envs, 3, device=self.device)
        up[:, 2] = bc.PUCK_H / 2
        mid = self._to_base(self.puck.data.root_pos_w.torch + math_utils.quat_apply(self.puck.data.root_quat_w.torch, up))
        return (mid - self._ee_b()[0]).norm(dim=1) < 0.05

    def _base_tilt(self):
        g = self.robot.data.projected_gravity_b.torch
        return torch.acos((-g[:, 2]).clamp(-1, 1))

    def _phase(self):
        """0 hold, 1 down, 2 settle, 3 release, 4 up, 5 wait, 6 done."""
        return (self.t[:, None] >= self.t_ph).sum(dim=1)

    def _gravity_ff(self):
        """Feed-forward joint offsets [n, 5]: -tau_j / k_j (nominal stiffness) for joints 2 and 3, as the real controller
        (joint4 compliance is 0 there). tau from gravity::torques of rl_arm_correction.hpp, torch port."""
        if self.cfg.ff_gain == 0.0:
            return torch.zeros_like(self.delta)
        q = self.robot.data.joint_pos.torch[:, self.arm_ids]
        tau = _gravity_torques(q[:, 1], q[:, 2], q[:, 3])
        ff = torch.zeros_like(self.delta)
        ff[:, 1:3] = (-self.cfg.ff_gain * tau[:, :2] / self.k_arm0[:, 1:3]).clamp(-self.cfg.ff_max, self.cfg.ff_max)
        return ff

    def _sag_step(self, q_ref, holding):
        """Arm target q_ref + corr + delta + gravity ff. While holding, every corr_dt the integral moves corr by corr_k x
        (q_ref + delta - q): it takes out the sag that the feed-forward left, not the residual. (The real controller puts the
        feed-forward into the error too: its twist hides behind the encoder. Here joint_pos is the drooped joint itself.)"""
        dt = self.physics_dt
        tick = (torch.round(self.t / dt).long() % max(1, round(self.cfg.corr_dt / dt))) == 0
        q_err = q_ref + self.delta - self.robot.data.joint_pos.torch[:, self.arm_ids]
        new = (self.corr + self.corr_k[:, None] * q_err).clamp(-self.cfg.corr_max, self.cfg.corr_max)
        self.corr = torch.where((holding & tick)[:, None], new, self.corr)
        return q_ref + self.corr + self.delta + self._gravity_ff()

    def _sample_sag(self, env_ids):
        """Per episode: corr = 0, integral gain and sag size (arm stiffness k0 / s: sag under gravity ~ 1 / k)."""
        cfg, m, dev = self.cfg, len(env_ids), self.device
        self.corr[env_ids] = 0.0
        g = 1.0 if cfg.corr_gain_range is None else torch.empty(m, device=dev).uniform_(*cfg.corr_gain_range)
        self.corr_k[env_ids] = cfg.corr_gain * g
        if cfg.sag_scale_range is not None:
            s = torch.empty(m, 1, device=dev).uniform_(*cfg.sag_scale_range)
            self.robot.write_joint_stiffness_to_sim_index(stiffness=self.k_arm0[env_ids] / s, joint_ids=self.arm_ids, env_ids=env_ids)

    def _corr_obs(self, obs):
        return torch.cat([obs, 10.0 * self.corr], dim=1) if self.cfg.obs_corr else obs

    # ------------------------------------------------------------------ stepping
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        a = actions.clamp(-1.0, 1.0)
        live = (self._phase() < 3) & (self.t >= self.t_tr)  # residual active until the fingers open, frozen after
        self.delta = torch.where(live[:, None], self.cfg.residual_rad * a, self.delta)
        self.act_rate = ((a - self.prev_act) ** 2).sum(dim=1) * live
        self.act_mag = (a ** 2).sum(dim=1) * live
        self.prev_act = a.clone()

    def _apply_action(self) -> None:
        dt = self.physics_dt
        ph = self._phase()
        t = self.t
        # base reference: hold above -> down -> hold -> hold -> up -> hold
        if self.dn_tab is None:
            s_down = bc.move_s(t - self.t_ph[:, 0], self.t_ph[:, 1] - self.t_ph[:, 0])
            s_up = bc.move_s(t - self.t_ph[:, 3], self.t_ph[:, 4] - self.t_ph[:, 3])
            q_dn = self.q_above + s_down[:, None] * (self.q_down - self.q_above)
            q_up = self.q_down + s_up[:, None] * (self.q_above - self.q_down)
        else:  # cuMotion path and velocity profile (sampled uniform in its time), shifted to this episode's targets
            lin = lambda a, b: ((t - self.t_ph[:, a]) / (self.t_ph[:, b] - self.t_ph[:, a]).clamp(min=1e-3)).clamp(0, 1)
            tab = self.dn_tab[self.tab_idx]
            q_dn = self._shaped(self.q_above, self.q_down, tab, lin(0, 1))
            q_up = self._shaped(self.q_down, self.q_above, tab.flip(1), lin(3, 4))
        if self.tr_tab is None:
            q_tr = self.q_start + bc.move_s(t, self.t_tr.clamp(min=1e-3))[:, None] * (self.q_above - self.q_start)
        else:
            q_tr = self._shaped(self.q_start, self.q_above, self.tr_tab[self.tab_idx], (t / self.t_tr.clamp(min=1e-3)).clamp(0, 1))
        q_ref = torch.where((t < self.t_tr)[:, None], q_tr, q_dn)
        q_ref = torch.where((ph >= 3)[:, None], q_up, q_ref)
        self.q_ref = q_ref
        self.grip_ref = torch.where(ph >= 3, torch.full_like(t, self.grip_after), torch.full_like(t, self.grip_before))
        # sag integral while a target is held (not during moves), as cycle_sm.py
        holding = ((ph == 0) & (t >= self.t_tr)) | (ph == 2) | (ph == 3) | (ph == 5)
        self._send(torch.cat([self._sag_step(q_ref, holding), self.grip_ref[:, None]], dim=1))
        self.t = self.t + dt
        # puck position right after the hand is up (start of the 2 s style stillness check)
        just_up = (self._phase() == 5) & (self.t - self.t_ph[:, 4] < dt * 1.5)
        if just_up.any():
            self.puck_after_up = torch.where(just_up[:, None], self.puck.data.root_pos_w.torch, self.puck_after_up)

    def _send(self, tgt):
        """Joint targets (n, 6: arm 5 + gripper) through the per-env, per-joint latency ring buffer."""
        self.hist_i = (self.hist_i + 1) % self.hist.shape[0]
        self.hist[self.hist_i] = tgt
        idx = (self.hist_i - self.lag) % self.hist.shape[0]
        delayed = self.hist[idx, torch.arange(self.num_envs, device=self.device)[:, None], torch.arange(6, device=self.device)]
        self.robot.set_joint_position_target_index(target=delayed[:, :5], joint_ids=self.arm_ids)
        self.robot.set_joint_position_target_index(target=delayed[:, 5:6].repeat(1, len(self.grip_ids)), joint_ids=self.grip_ids)

    def _load_tabs(self, task):
        """cumotion/cache_<task>.npz -> self.dn_tab / self.tr_tab (K, N, 5) and their timing (K,)."""
        d = np.load(CUMOTION_DIR / f"cache_{task}.npz")
        c = {k: torch.tensor(d[k], device=self.device, dtype=torch.float32) for k in d.files}
        c = {k: v.reshape(-1, *v.shape[-2:]) if k.endswith("_q") else v.reshape(-1) if v.dim() <= 2 and not k.startswith("q_") else v
             for k, v in c.items()}
        self.dn_tab, self.dn_time = c["dn_q"], (c["dn_dur"], c["dn_v"], c["dn_a"])
        if "tr_q" in c:
            self.tr_tab, self.tr_time = c["tr_q"], (c["tr_dur"], c["tr_v"], c["tr_a"])
        return c

    @staticmethod
    def _tab_dur(timing, idx, vel_scale):
        """cuMotion duration stretched so its peak speed / acceleration fit vel_scale x team limits."""
        dur, v, a = (x[idx] for x in timing)
        vs = torch.as_tensor(vel_scale, device=dur.device, dtype=dur.dtype)
        return dur * torch.maximum(v / (vs * bc.VMAX), (a / (vs * bc.AMAX)).sqrt()).clamp(min=1.0)

    @staticmethod
    def _shaped(qa, qb, tab, s):
        """Point at fraction s of the tabulated path, shifted so it runs from qa to qb (offsets blended linearly)."""
        n = tab.shape[1] - 1
        x = s * n
        k = x.floor().long().clamp(0, n - 1)
        fr = (x - k)[:, None]
        i = torch.arange(tab.shape[0], device=tab.device)
        q = tab[i, k] + fr * (tab[i, k + 1] - tab[i, k])
        return qa + (q - tab[:, 0]) + s[:, None] * ((qb - qa) - (tab[:, -1] - tab[:, 0]))

    def _sample_lag(self, env_ids):
        r = self.cfg.latency_scale_range
        if r:
            scale = torch.empty(len(env_ids), 6, device=self.device).uniform_(*r)
            self.lag[env_ids] = torch.round(self.lat_s * scale / self.physics_dt).long()

    # ------------------------------------------------------------------ MDP
    def _cam(self):
        """Gripper-camera style measurement of the cell relative to the puck bottom (base frame)."""
        rel = self.plan["cell_b"][self.cell] - self._puck_bottom_b()
        return rel + self.cam_bias + self.cfg.cam_noise_m * torch.randn_like(rel)

    def _get_observations(self) -> dict:
        ph = self._phase()
        q = self.robot.data.joint_pos.torch[:, self.arm_ids]
        qd = self.robot.data.joint_vel.torch[:, self.arm_ids]
        onehot = torch.nn.functional.one_hot(ph.clamp(max=5), 6).float()
        prog = ((self.t - torch.cat([torch.zeros_like(self.t[:, None]), self.t_ph[:, :-1]], 1).gather(1, ph.clamp(max=5)[:, None])[:, 0])
                / 1.0).clamp(0, 2)
        obs = torch.cat([
            q - self.q_ref,                  # 5 tracking error (what the joints see)
            self.q_down - self.q_ref,        # 5 remaining planned motion
            0.1 * qd,                        # 5
            self.q_ref - self.q_above,       # 5
            self.prev_act,                   # 5
            onehot,                          # 6
            prog[:, None],                   # 1
            100.0 * self._cam()[:, :2],      # 2 cell - puck [cm], from the gripper camera
        ], dim=1)
        obs = self._corr_obs(obs)            # (+5 sag integral x 10)
        bottom = self._puck_bottom_b()
        priv = torch.cat([
            100.0 * (self.plan["cell_b"][self.cell] - bottom),  # 3 true cell - puck
            self._puck_tilt()[:, None],                         # 1
            self.puck.data.root_lin_vel_w.torch,                # 3
            100.0 * self.aim_err,                               # 3
            100.0 * self.hand_err,                              # 3
            self._base_tilt()[:, None],                         # 1
        ], dim=1)
        return {"policy": obs, "critic": torch.cat([obs, priv], dim=1)}

    def _get_rewards(self) -> torch.Tensor:
        ph = self._phase()
        bottom = self._puck_bottom_b()
        cell = self.plan["cell_b"][self.cell]
        e = (bottom[:, :2] - cell[:, :2]).norm(dim=1)
        w = torch.tensor([0.2, 0.5, 1.0, 1.0, 0.0, 0.0, 0.0], device=self.device)[ph]
        tilt = self._puck_tilt()
        r = self.cfg.w_track * w * torch.exp(-(e / 0.004) ** 2) - self.cfg.w_tilt * w * tilt
        r = r - self.cfg.w_rate * self.act_rate - self.cfg.w_act * self.act_mag
        # end of episode: judge like cycle_sm.py place (centre 10 mm, tilt 5 deg, still, out of the hand, on the box)
        done = ph >= 6
        in_hand = self.in_hand
        drift = (self.puck.data.root_pos_w.torch - self.puck_after_up).norm(dim=1)
        on_box = (bottom[:, 2] - (cell[:, 2])).abs() < 0.005
        ok = self.held0 & ~in_hand & on_box & (tilt < math.radians(5.0)) & (drift < 0.002) & (e < 0.010)
        r = r + done * (self.cfg.w_success * ok + self.cfg.w_final * torch.exp(-(e / 0.004) ** 2))
        self.success = torch.where(done, ok, self.success)
        self.final_err = torch.where(done, e, self.final_err)
        r = r - self.cfg.w_fail * (self.dropped | self.tipped)
        return r

    def _get_dones(self):
        ph = self._phase()
        self.in_hand = self._in_hand()
        self.dropped = (ph < 3) & ~self.in_hand  # failures that end the episode early
        self.tipped = self._base_tilt() > math.radians(10.0)
        return (ph >= 6) | self.dropped | self.tipped, self.episode_length_buf >= self.max_episode_length

    def _reset_idx(self, env_ids):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        if self.common_step_counter > 0 and len(env_ids) > 0:
            done = self._phase()[env_ids] >= 6
            if done.any():
                ids = env_ids[done]
                log = self.extras.setdefault("log", {})
                log["Metrics/success"] = self.success[ids].float().mean().item()
                log["Metrics/err_mm"] = 1000 * self.final_err[ids].mean().item()
                self.stats["success"] += self.success[ids].tolist()
                self.stats["err"] += self.final_err[ids].tolist()
            self.stats["dropped"] += self.dropped[env_ids].tolist()
            self.stats["tipped"] += self.tipped[env_ids].tolist()
        super()._reset_idx(env_ids)
        m, dev, cfg = len(env_ids), self.device, self.cfg
        u = lambda *s: torch.rand(m, *s, device=dev) * 2 - 1

        # robot at its spawn pose, at rest
        root = self.robot.data.default_root_pose.torch[env_ids].clone()
        root[:, :3] += self.scene.env_origins[env_ids]
        self.robot.write_root_pose_to_sim_index(root_pose=root, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)

        # task sample: cell, plan errors, true puck offset in the hand
        cell = torch.randint(0, len(CELLS), (m,), device=dev)
        rot = self.plan["rot"][cell]  # (m, 3, 3)
        aim = torch.zeros(m, 3, device=dev)
        aim[:, :2] = cfg.cell_noise_m * u(2)
        along = torch.cross(self.close_axis_ee.expand(m, 3), torch.tensor([0.0, 0.0, 1.0], device=dev).expand(m, 3), dim=1)
        d_true = self.d_nom + along * cfg.hand_offset_m[0] * u(1)
        d_true[:, 2] += cfg.hand_offset_m[1] * u()
        hand_err = torch.zeros(m, 3, device=dev)
        hand_err[:, :2] = cfg.hand_noise_m * u(2)
        d_meas = d_true + hand_err
        # targets (base frame) and joint plan by linearization around the per-cell nominal solution
        off = -(rot @ (d_meas - self.d_nom)[..., None])[..., 0] + aim
        q_above = self.plan["q_above"][cell] + (self.plan["P_above"][cell] @ off[..., None])[..., 0]
        q_down = self.plan["q_down"][cell] + (self.plan["P_down"][cell] @ off[..., None])[..., 0]
        dist = (q_down - q_above).abs().max(dim=1).values.cpu().numpy()
        vs = np.full(m, cfg.vel_scale) if cfg.vel_scale_range is None else np.random.uniform(*cfg.vel_scale_range, m)
        t_move = torch.tensor([bc.trapezoid_duration(float(x), float(v)) for x, v in zip(dist, vs)], device=dev)
        if self.dn_tab is not None:
            self.tab_idx[env_ids] = cell
            t_move = self._tab_dur(self.dn_time, cell, torch.tensor(vs, device=dev, dtype=torch.float32))
        settle = torch.full_like(t_move, cfg.settle_s) if cfg.settle_range is None else \
            torch.empty(m, device=dev).uniform_(*cfg.settle_range)
        seg = torch.stack([torch.full_like(t_move, cfg.hold_s), t_move, settle,
                           torch.full_like(t_move, cfg.release_s), t_move, torch.full_like(t_move, cfg.wait_s)], dim=1)
        self.t_ph[env_ids] = torch.cumsum(seg, dim=1)
        self.t[env_ids] = 0.0
        self.cell[env_ids], self.q_above[env_ids], self.q_down[env_ids] = cell, q_above, q_down
        self.q_ref[env_ids] = q_above
        self.aim_err[env_ids], self.hand_err[env_ids], self.hand_true[env_ids] = aim, hand_err, d_true - self.d_nom
        self.cam_bias[env_ids] = 0.0
        self.cam_bias[env_ids, :2] = cfg.cam_bias_m * u(2)
        self._sample_sag(env_ids)
        self.delta[env_ids] = 0.0
        self.prev_act[env_ids] = 0.0
        self.success[env_ids] = False
        self.held0[env_ids] = True
        self.dropped[env_ids] = False
        self.tipped[env_ids] = False

        # arm at the above pose, fingers closed on the puck
        jp = self.robot.data.default_joint_pos.torch[env_ids].clone()
        jp[:, self.arm_ids] = q_above
        for gi in self.grip_ids:
            jp[:, gi] = bc.GRIP_STOP
        self.robot.write_joint_state_to_sim_index(position=jp, velocity=torch.zeros_like(jp), env_ids=env_ids)
        tgt = torch.cat([q_above, torch.zeros(m, 1, device=dev)], dim=1)
        self.hist[:, env_ids] = tgt
        self.robot.set_joint_position_target_index(target=q_above, joint_ids=self.arm_ids, env_ids=env_ids)

        # puck in the hand: bottom at ee + R d_true, upright; box faces square to the fingers
        p_ee = self.plan["p_above"][cell] + off
        bottom_b = p_ee + (rot @ d_true[..., None])[..., 0]
        pos_w = bottom_b + self.base_off + self.scene.env_origins[env_ids]
        ax = (rot @ self.close_axis_ee)  # closing axis in base frame
        yaw = torch.atan2(ax[:, 1], ax[:, 0]) + math.radians(4.0) * u()
        yaw = torch.where(self._kinds()[env_ids], yaw, math.pi * u())
        quat = math_utils.quat_from_euler_xyz(torch.zeros(m, device=dev), torch.zeros(m, device=dev), yaw)
        self.puck.write_root_pose_to_sim_index(root_pose=torch.cat([pos_w, quat], dim=1), env_ids=env_ids)
        self.puck.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)
        self.puck_after_up[env_ids] = pos_w


gym.register(
    id="Bee-Place-Residual",
    entry_point=f"{__name__}:PlaceEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": f"{__name__}:PlaceEnvCfg"},
)
