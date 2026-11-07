"""Bee-Pick-Residual: residual RL for the start box pick (segment 2 of the RL task split, skill pick_box).

Episode = the open hand is above the target puck (end of the transit segment) ->
    hold (sag settles) -> down to the grasp height -> settle -> close -> lift -> wait, then judge.
Two pucks stand on the start box at the box centre +-40 mm along y (cycle_sm.py --layout start_y); the target is one
of them, the other is a neighbour that must not be knocked (absent in 30 % of episodes = the second pick).
The base motion is the scripted one of demo_cycle.py pick(): offline IK aimed at the perceived puck (+-5 mm, +-10 deg),
fingers at right angles to the line through the 2 pucks (box puck: nearest face). The policy adds a joint residual
(|delta| <= 0.05 rad) until the fingers close and sees a gripper-camera style "puck - hand" measurement.

Same phase timeline, latency and sag integral as place_env.PlaceEnv, which this class reuses.
"""

from __future__ import annotations

import math

import gymnasium as gym
import numpy as np
import pinocchio as pin
import torch

import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.envs import DirectRLEnv
from isaaclab.utils import configclass

from bee_isaac import common as bc
from bee_isaac.tasks import place_env as pe

START_GAP = 0.080  # 2 pucks at the box centre +- 40 mm along y
YAW_BINS = np.radians(np.arange(-45, 46, 1))  # tool yaw offsets from the finger rule, planned offline
OBS_DIM, CRITIC_EXTRA = 36, 13
STOW = torch.tensor([0.0, -1.9024, 1.344, 0.9023, 0.0])  # demo_cycle.py reset_scene


@configclass
class PickSceneCfg(pe.PlaceSceneCfg):
    box = pe.PlaceSceneCfg().box.replace(
        spawn=sim_utils.UsdFileCfg(usd_path=str(bc.USD_DIR / "shelf" / "shelf.usda"), scale=(0.205 / 0.207, 0.205 / 0.207, 1.0),
                                   rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True)))
    nb = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/nb",
        spawn=sim_utils.MultiUsdFileCfg(usd_path=bc.PUCK_USDS),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(pe.BOX_CX, START_GAP / 2, bc.BOX_H + 0.002)),
    )


@configclass
class PickEnvCfg(pe.PlaceEnvCfg):
    scene: PickSceneCfg = PickSceneCfg(num_envs=256, env_spacing=1.0)
    observation_space = OBS_DIM + 5
    episode_length_s = 8.0

    hold_s, settle_s, release_s, wait_s = 0.3, 0.3, 0.5, 0.3  # release_s = close time here (demo CLOSE_S)
    above_m, open_gap_mm = 0.05, 40.0
    pos_noise_m, yaw_noise = 0.005, math.radians(10.0)  # perception (demo POS_NOISE, YAW_NOISE)
    park_m = 0.010  # base stopping error: both pucks shifted together
    transit = True  # start at stow and move to the above pose inside the episode (real transit end state)
    start_jitter_rad = 0.01  # transit = False: jitter around the above pose instead
    nb_prob = 0.7
    cam_bias_m, cam_noise_m = 0.002, 0.001
    cam_yaw_noise = math.radians(2.0)

    # reward
    w_track, w_tilt, w_nb, w_rate, w_act = 0.1, 0.5, 50.0, 0.05, 0.002
    w_success, w_final, w_fail = 10.0, 5.0, 5.0


class PickEnv(pe.PlaceEnv):
    cfg: PickEnvCfg
    TASK = "pick"

    def __init__(self, cfg: PickEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)  # ponytail: also plans the 6 place cells (unused, ~1 s)
        n, dev = self.num_envs, self.device
        self.nb = self.scene["nb"]
        self.grip_before, self.grip_after = bc.gap_joint(cfg.open_gap_mm), 0.0  # open -> close from phase 3

        # ---- offline plan per side (target at -y / +y) and tool yaw bin, linearized in position like PlaceEnv
        ik = bc.ArmIK()
        base = ik.base_in_root()
        rows = {k: [] for k in ("q_above", "q_down", "P_above", "P_down", "want")}
        for side in (-1.0, 1.0):
            pb = np.array([pe.BOX_CX, side * START_GAP / 2, bc.BOX_H]) - base
            hd = math.atan2(pb[1], pb[0])
            want = hd + math.remainder(math.pi / 2 - hd, math.pi)
            rows["want"].append(want)
            seeds = [np.array([hd, -0.2, 0.7, 1.0, s]) for s in (0.0, want - hd, hd - want)]
            r_side = {k: [] for k in ("q_above", "q_down", "P_above", "P_down")}
            for dy in YAW_BINS:
                rot = pin.utils.rotate("z", want + dy) @ bc.DOWN
                for k, dz in (("above", bc.GRASP_Z + cfg.above_m), ("down", bc.GRASP_Z)):
                    q, e_pos, e_ang = ik.solve(pb + np.array([0.0, 0.0, dz]), rot, seeds)
                    assert e_pos < 2e-3 and e_ang < math.radians(2.0), f"IK failed: side {side} yaw {math.degrees(dy):.0f} {k}"
                    seeds = [q]
                    r_side["q_" + k].append(q)
                    r_side["P_" + k].append(ik.jac_pos(q))
                seeds = [r_side["q_above"][-1]]
            for k, v in r_side.items():
                rows[k].append(v)
        self.pplan = {k: torch.tensor(np.array(v), device=dev, dtype=torch.float32) for k, v in rows.items()}
        if cfg.planner == "cumotion":
            c = self._load_tabs("pick")  # (side x yaw bin) flattened
            K = len(YAW_BINS)
            self.pplan["q_above"] = c["dn_q"][:, 0].reshape(2, K, 5)
            self.pplan["q_down"] = c["dn_q"][:, -1].reshape(2, K, 5)
        self.p_nom = torch.tensor([[pe.BOX_CX, s * START_GAP / 2, bc.BOX_H] for s in (-1.0, 1.0)], device=dev) \
            - torch.tensor(base, device=dev, dtype=torch.float32)

        z = lambda *s: torch.zeros(n, *s, device=dev)
        self.side = torch.zeros(n, dtype=torch.long, device=dev)
        self.tool_yaw, self.yaw_err, self.cam_yaw_bias = z(), z(), z()
        self.nb_on = torch.zeros(n, dtype=torch.bool, device=dev)
        self.nb0 = z(3)
        self.nb_hit = torch.zeros(n, dtype=torch.bool, device=dev)
        self.knocked = torch.zeros(n, dtype=torch.bool, device=dev)

    # ------------------------------------------------------------------ helpers
    def _mid_b(self):
        """Target puck centre (half height) in the base frame."""
        up = torch.zeros(self.num_envs, 3, device=self.device)
        up[:, 2] = bc.PUCK_H / 2
        return self._to_base(self.puck.data.root_pos_w.torch + math_utils.quat_apply(self.puck.data.root_quat_w.torch, up))

    def _puck_yaw_b(self):
        _, bq = self._base()
        return math_utils.euler_xyz_from_quat(math_utils.quat_mul(math_utils.quat_inv(bq), self.puck.data.root_quat_w.torch))[2]

    def _face_err(self):
        """Box puck: puck face yaw - tool yaw, wrapped to +-45 deg (0 for a cylinder)."""
        e = torch.remainder(self._puck_yaw_b() - self.tool_yaw + math.pi / 4, math.pi / 2) - math.pi / 4
        return torch.where(self._kinds(), e, torch.zeros_like(e))

    def _nb_state(self):
        """Neighbour moved > 3 mm or tilted > 5 deg (only when it is on the box)."""
        d = (self.nb.data.root_pos_w.torch - self.nb0)[:, :2].norm(dim=1)
        up = torch.zeros(self.num_envs, 3, device=self.device)
        up[:, 2] = 1.0
        tilt = torch.acos(math_utils.quat_apply(self.nb.data.root_quat_w.torch, up)[:, 2].clamp(-1, 1))
        return d * self.nb_on, self.nb_on & ((d > 0.003) | (tilt > math.radians(5.0)))

    # ------------------------------------------------------------------ MDP
    def _get_observations(self) -> dict:
        ph = self._phase()
        q = self.robot.data.joint_pos.torch[:, self.arm_ids]
        qd = self.robot.data.joint_vel.torch[:, self.arm_ids]
        onehot = torch.nn.functional.one_hot(ph.clamp(max=5), 6).float()
        prog = ((self.t - torch.cat([torch.zeros_like(self.t[:, None]), self.t_ph[:, :-1]], 1).gather(1, ph.clamp(max=5)[:, None])[:, 0])
                / 1.0).clamp(0, 2)
        rel = self._mid_b() - self._ee_b()[0]  # puck - hand
        cam = rel + self.cam_bias + self.cfg.cam_noise_m * torch.randn_like(rel)
        face = self._face_err()
        cam_yaw = face + self.cam_yaw_bias + self.cfg.cam_yaw_noise * torch.randn_like(face)
        is_box = self._kinds().float()
        obs = torch.cat([
            q - self.q_ref, self.q_down - self.q_ref, 0.1 * qd, self.q_ref - self.q_above, self.prev_act,  # 25
            onehot, prog[:, None],                                                                         # 7
            100.0 * cam[:, :2], (cam_yaw * is_box)[:, None], is_box[:, None],                              # 4
        ], dim=1)
        obs = self._corr_obs(obs)                                                                          # (+5)
        nb_d, _ = self._nb_state()
        priv = torch.cat([
            100.0 * rel,                                 # 3
            self._puck_tilt()[:, None],                  # 1
            self.puck.data.root_lin_vel_w.torch,         # 3
            100.0 * self.aim_err,                        # 3
            face[:, None],                               # 1
            self._base_tilt()[:, None],                  # 1
            100.0 * nb_d[:, None],                       # 1
        ], dim=1)
        return {"policy": obs, "critic": torch.cat([obs, priv], dim=1)}

    def _get_rewards(self) -> torch.Tensor:
        ph = self._phase()
        e = (self._mid_b() - self._ee_b()[0])[:, :2].norm(dim=1)
        w = torch.tensor([0.2, 0.5, 1.0, 0.0, 0.0, 0.0, 0.0], device=self.device)[ph]
        tilt = self._puck_tilt()
        nb_d, nb_hit = self._nb_state()
        self.nb_hit |= nb_hit
        r = self.cfg.w_track * w * torch.exp(-(e / 0.004) ** 2) - self.cfg.w_tilt * (ph < 4) * tilt - self.cfg.w_nb * nb_d
        r = r - self.cfg.w_rate * self.act_rate - self.cfg.w_act * self.act_mag
        self.final_err = torch.where(ph < 3, e, self.final_err)  # aim error when the fingers start closing
        # end of episode: lifted > 30 mm and in the hand, neighbour untouched (cycle_sm.py pick)
        done = ph >= 6
        lifted = (self.puck.data.root_pos_w.torch[:, 2] - self.scene.env_origins[:, 2]) - bc.BOX_H > 0.03
        ok = lifted & self.in_hand & ~self.nb_hit
        r = r + done * (self.cfg.w_success * ok + self.cfg.w_final * torch.exp(-(self.final_err / 0.004) ** 2))
        self.success = torch.where(done, ok, self.success)
        return r - self.cfg.w_fail * (self.knocked | self.tipped)

    def _get_dones(self):
        ph = self._phase()
        self.in_hand = self._in_hand()
        self.knocked = self._puck_tilt() > math.radians(30.0)  # a top-down grasp cannot take a puck lying down
        self.dropped = self.knocked  # stats key shared with train.report
        self.tipped = self._base_tilt() > math.radians(10.0)
        return (ph >= 6) | self.knocked | self.tipped, self.episode_length_buf >= self.max_episode_length

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
                log["Metrics/nb_hit"] = self.nb_hit[ids].float().mean().item()
                self.stats["success"] += self.success[ids].tolist()
                self.stats["err"] += self.final_err[ids].tolist()
            self.stats["dropped"] += self.knocked[env_ids].tolist()
            self.stats["tipped"] += self.tipped[env_ids].tolist()
        DirectRLEnv._reset_idx(self, env_ids)
        m, dev, cfg = len(env_ids), self.device, self.cfg
        u = lambda *s: torch.rand(m, *s, device=dev) * 2 - 1
        org = self.scene.env_origins[env_ids]

        root = self.robot.data.default_root_pose.torch[env_ids].clone()
        root[:, :3] += org
        self.robot.write_root_pose_to_sim_index(root_pose=root, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)

        # pucks: target on one side, neighbour on the other (or away), both shifted by the park error, random yaw
        side = torch.randint(0, 2, (m,), device=dev)
        sgn = side.float() * 2 - 1
        park = cfg.park_m * u(2)
        yaw_t, yaw_n = math.pi * u(), math.pi * u()
        zero = torch.zeros(m, device=dev)
        pos_t = torch.stack([pe.BOX_CX + park[:, 0], sgn * START_GAP / 2 + park[:, 1], zero + bc.BOX_H + 0.001], 1) + org
        pos_n = torch.stack([pe.BOX_CX + park[:, 0], -sgn * START_GAP / 2 + park[:, 1], zero + bc.BOX_H + 0.001], 1) + org
        nb_on = torch.rand(m, device=dev) < cfg.nb_prob
        pos_n = torch.where(nb_on[:, None], pos_n, org + torch.tensor([0.6, 0.3, 0.001], device=dev))
        for obj, pos, yw in ((self.puck, pos_t, yaw_t), (self.nb, pos_n, yaw_n)):
            quat = math_utils.quat_from_euler_xyz(zero, zero, yw)
            obj.write_root_pose_to_sim_index(root_pose=torch.cat([pos, quat], dim=1), env_ids=env_ids)
            obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)
        self.nb0[env_ids], self.nb_on[env_ids] = pos_n, nb_on

        # perception -> tool yaw (finger rule perp, demo pick()) -> plan by yaw bin + position linearization
        aim = torch.zeros(m, 3, device=dev)
        aim[:, :2] = park + cfg.pos_noise_m * u(2)  # perceived puck - nominal
        want = self.pplan["want"][side]
        yaw_meas = yaw_t + cfg.yaw_noise * u()  # robot root frame = world yaw here (base not rotated at spawn)
        is_box = self._kinds()[env_ids]
        dy = torch.where(is_box, torch.remainder(yaw_meas - want + math.pi / 4, math.pi / 2) - math.pi / 4, zero)
        b = torch.round((dy - YAW_BINS[0]) / math.radians(1.0)).long().clamp(0, len(YAW_BINS) - 1)
        q_above = self.pplan["q_above"][side, b] + (self.pplan["P_above"][side, b] @ aim[..., None])[..., 0]
        q_down = self.pplan["q_down"][side, b] + (self.pplan["P_down"][side, b] @ aim[..., None])[..., 0]
        self.tool_yaw[env_ids] = want + torch.tensor(YAW_BINS, device=dev, dtype=torch.float32)[b]
        self.side[env_ids] = side
        self.aim_err[env_ids] = aim - torch.cat([park, zero[:, None]], 1)  # perceived - true

        dist = (q_down - q_above).abs().max(dim=1).values.cpu().numpy()
        t_move = torch.tensor([bc.trapezoid_duration(float(x), cfg.vel_scale) for x in dist], device=dev)
        f = lambda v: torch.full_like(t_move, v)
        if cfg.transit:
            q0 = STOW.to(dev).expand(m, 5)
            t_tr = torch.tensor([bc.trapezoid_duration(float(x), cfg.vel_scale)
                                 for x in (q_above - q0).abs().max(dim=1).values.cpu().numpy()], device=dev)
        else:
            q0 = q_above + cfg.start_jitter_rad * u(5)
            t_tr = f(0.0)
        if self.dn_tab is not None:
            idx = side * len(YAW_BINS) + b
            self.tab_idx[env_ids] = idx
            t_move = self._tab_dur(self.dn_time, idx, cfg.vel_scale)
            if cfg.transit:
                t_tr = self._tab_dur(self.tr_time, idx, cfg.vel_scale)
        self.q_start[env_ids], self.t_tr[env_ids] = q0, t_tr
        self.t_ph[env_ids] = torch.cumsum(torch.stack([t_tr + cfg.hold_s, t_move, f(cfg.settle_s), f(cfg.release_s), t_move, f(cfg.wait_s)], 1), 1)
        self.t[env_ids] = 0.0
        self.q_above[env_ids], self.q_down[env_ids], self.q_ref[env_ids] = q_above, q_down, q0
        self.cam_bias[env_ids] = 0.0
        self.cam_bias[env_ids, :2] = cfg.cam_bias_m * u(2)
        self.cam_yaw_bias[env_ids] = math.radians(3.0) * u()
        for buf in (self.delta, self.prev_act):
            buf[env_ids] = 0.0
        self._sample_sag(env_ids)
        for buf in (self.success, self.nb_hit, self.knocked, self.dropped, self.tipped):
            buf[env_ids] = False
        self.final_err[env_ids] = 0.0

        # arm at stow (or near the above pose), fingers open
        jp = self.robot.data.default_joint_pos.torch[env_ids].clone()
        jp[:, self.arm_ids] = q0
        for gi in self.grip_ids:
            jp[:, gi] = self.grip_before
        self.robot.write_joint_state_to_sim_index(position=jp, velocity=torch.zeros_like(jp), env_ids=env_ids)
        self.hist[:, env_ids] = torch.cat([q0, torch.full((m, 1), self.grip_before, device=dev)], dim=1)
        self.robot.set_joint_position_target_index(target=q0, joint_ids=self.arm_ids, env_ids=env_ids)


gym.register(
    id="Bee-Pick-Residual",
    entry_point=f"{__name__}:PickEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": f"{__name__}:PickEnvCfg"},
)
