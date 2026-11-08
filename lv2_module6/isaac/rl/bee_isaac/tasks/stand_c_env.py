"""Bee-Stand_c-Residual: stand a lying puck up with a learned hand trajectory (design method C, 2026-11-08).

The episode starts with the open gripper above the far end of a lying puck (IK put it there). The policy outputs a hand
velocity (3 values: along the puck axis, across it, up) every 20 ms; a damped least squares step turns the hand target
into joint targets (tool down + yaw held), rate limited. The gripper is rule driven: it closes when the hand is at the
pinch point (force-capped, as RightEnv), opens once the puck stands (tilt < 5 deg) for settle_s, then the hand lifts
away and the episode is judged like RightEnv (puck up, on the box, out of the hand, box and neighbour unmoved).

Reuses RightEnv for the scene, the per-episode randomization and the stats. Curriculum: start_phi_deg >= 0 starts the
episode already pinching the puck tilted by that angle (arc waypoints every 7.5 deg).
"""

from __future__ import annotations

import math

import gymnasium as gym
import torch

import isaaclab.utils.math as math_utils
from isaaclab.utils import configclass

from bee_isaac import common as bc
from bee_isaac.stand_ik import TorchArm
from bee_isaac.tasks import place_env as pe
from bee_isaac.tasks import right_env as re

OBS_C = 28
EE_UP = bc.GRASP_Z - bc.PUCK_H + (bc.GRIP_DEPTH / 2)


@configclass
class StandCEnvCfg(re.RightEnvCfg):
    observation_space = OBS_C
    action_space = 3
    episode_length_s = 16.0
    arc_end_deg = 90.0  # arc waypoints only used for the curriculum start poses and the scripted teacher
    lift_away_m = 0.05
    step_m = 0.012  # hand step per policy step at action 1 (0.6 m/s)
    rate_vmax = 3.0  # joint reference rate limit [rad/s] (team limit)
    w_ang = 0.5  # orientation weight of the IK step
    t_approach_s, t_lift_s = 4.0, 6.0  # time out: still approaching / still not standing
    close_tol_z, close_tol_lat, close_tol_ax = 0.004, 0.004, 0.008  # hand within these of the pinch point: the gripper closes
    z_floor_m = 0.012  # the hand never goes lower than this below the pinch height
    release_deg, relapse_deg = 5.0, 5.0  # stands for settle_s -> open; tilted again during settle -> back to pinching
    w_stand_max = 0.5  # puck angular speed [rad/s] below which 'standing' counts
    lift_dur_s = 0.5
    start_phi_deg = -1.0  # >= 0: start pinching the puck tilted by this angle (curriculum); < 0: open gripper above the puck
    start_phi_max_deg = -1.0  # >= 0: part of the envs (1 - above_prob) start at a random angle in min..max instead
    start_phi_min_deg = 0.0
    above_prob = 1.0
    approach_only = False  # episode ends (success) when the gripper closes: trains the approach alone
    # reward
    w_prog, w_time, w_grasp = 5.0, 0.002, 1.0
    w_tilt_prog, w_cap, cap_m = 3.0, 0.1, 0.06
    w_slide = 0.2
    w_box = 30.0  # per step x start box shift [m] (the box is a free body; > 3 mm ends the episode)


class StandCEnv(re.RightEnv):
    cfg: StandCEnvCfg
    TASK = "stand_c"

    def __init__(self, cfg: StandCEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, dev = self.num_envs, self.device
        self.arm = TorchArm(dev)
        z = lambda *s: torch.zeros(n, *s, device=dev)
        self.prev_act = z(3)
        self.ph = torch.zeros(n, dtype=torch.long, device=dev)
        self.t_in = z()
        self.x_t, self.q_goal, self.p_pinch = z(3), z(5), z(3)
        self.R_tgt = torch.eye(3, device=dev).repeat(n, 1, 1)
        self.ur, self.lat = z(3), z(3)
        self.d_prev, self.up_prev, self.ee_obs_prev = z(), z(), z(3)
        self.stuck = torch.zeros(n, dtype=torch.bool, device=dev)
        self.zhat = torch.tensor([0.0, 0.0, 1.0], device=dev)

    # ------------------------------------------------------------------ helpers
    def _phase(self):
        return self.ph

    def _frame(self, v):
        """base frame vectors -> (along the puck axis, across it, up)."""
        return torch.stack([(v * self.ur).sum(1), (v * self.lat).sum(1), v[:, 2]], 1)

    # ------------------------------------------------------------------ stepping
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        cfg, dt = self.cfg, self.step_dt
        a = actions.clamp(-1.0, 1.0)
        ph = self.ph
        self.t_in = self.t_in + dt
        up_z = self._up_z()
        ee = self._ee_b()[0]
        e = self.p_pinch - ee
        near = ((ee[:, 2] - self.p_pinch[:, 2]) < cfg.close_tol_z) & ((e * self.lat).sum(1).abs() < cfg.close_tol_lat) \
            & ((e * self.ur).sum(1).abs() < cfg.close_tol_ax)
        wn = self.puck.data.root_ang_vel_w.torch.norm(dim=1)
        stood = (up_z > math.cos(math.radians(cfg.release_deg))) & (wn < cfg.w_stand_max)
        relapse = (ph == 4) & (up_z < math.cos(math.radians(cfg.relapse_deg)))
        adv = ((ph == 0) & (self.t_in >= cfg.hold_s)) | ((ph == 1) & near) | ((ph == 2) & (self.t_in >= cfg.close_s)) \
            | ((ph == 3) & stood) | ((ph == 4) & (self.t_in >= cfg.settle_s) & ~relapse) | ((ph == 5) & (self.t_in >= cfg.open_s)) \
            | ((ph == 6) & (self.t_in >= cfg.lift_dur_s)) | ((ph == 7) & (self.t_in >= cfg.wait_s))
        new = torch.where(relapse, torch.full_like(ph, 3), ph + adv.long())
        if cfg.approach_only:
            new = torch.where(new == 2, torch.full_like(new, 8), new)
        self.t_in = torch.where(new != ph, torch.zeros_like(self.t_in), self.t_in)
        self.stuck = ((new == 1) & (self.t_in > cfg.t_approach_s)) | ((new == 3) & (self.t_in > cfg.t_lift_s))
        self.ph = ph = new
        live = (ph == 1) | (ph == 3)
        self.act_rate = ((a - self.prev_act) ** 2).sum(dim=1) * live
        self.act_mag = (a ** 2).sum(dim=1) * live
        self.prev_act = a.clone()
        s = cfg.step_m * a * live[:, None]
        x = self.x_t + s[:, 0:1] * self.ur + s[:, 1:2] * self.lat + s[:, 2:3] * self.zhat
        x[:, 2] = x[:, 2] + (ph == 6) * cfg.lift_away_m / cfg.lift_dur_s * dt
        d = x - self.p_pinch  # keep the hand target in a box around the pinch point
        dr = (d * self.ur).sum(1).clamp(-0.08, 0.04)
        dl = (d * self.lat).sum(1).clamp(-0.03, 0.03)
        dz = d[:, 2].clamp(-cfg.z_floor_m, 0.12 if cfg.lift_away_m <= 0.06 else 0.2)
        self.x_t = self.p_pinch + dr[:, None] * self.ur + dl[:, None] * self.lat + dz[:, None] * self.zhat
        self.q_goal = self.arm.step(self.q_goal, self.x_t, self.R_tgt, iters=3, w_ang=cfg.w_ang)

    def _apply_action(self) -> None:
        dt = self.physics_dt
        ph = self.ph
        lim = self.cfg.rate_vmax * dt
        self.q_ref = self.q_ref + (self.q_goal - self.q_ref).clamp(-lim, lim)
        g = torch.where((ph >= 2) & (ph <= 4), self.pinch_q, torch.full_like(self.t, self.open_q))
        hold = ~((ph == 1) | (ph == 3) | (ph == 6))
        self.grip_ref = g
        self._send(torch.cat([self._sag_step(self.q_ref, hold), g[:, None]], dim=1))
        self.t = self.t + dt

    # ------------------------------------------------------------------ MDP
    def _get_observations(self) -> dict:
        ph = self.ph
        ee = self._ee_b()[0]
        n = self.num_envs
        _, bq = self._base()
        pin_m = self.p_pinch + self.cam_bias + self.cfg.cam_noise_m * torch.randn_like(self.p_pinch)
        piv = self._to_base(self.pivot0)
        w = math_utils.quat_apply_inverse(bq, self.puck.data.root_ang_vel_w.torch)
        hv = (ee - self.ee_obs_prev) / self.step_dt
        self.ee_obs_prev = ee
        grip = self.robot.data.joint_pos.torch[:, self.grip_ids[0]]
        onehot = torch.nn.functional.one_hot(ph.clamp(max=7), 8).float()
        axis = self._axis() + 0.03 * torch.randn(n, 3, device=self.device)
        obs = torch.cat([
            100.0 * self._frame(pin_m - ee),        # 3 pinch point - hand [cm]
            10.0 * self._frame(ee - piv),           # 3 hand - near bottom edge [dm]
            self._frame(axis),                      # 3 puck axis
            0.2 * self._frame(w),                   # 3 puck angular velocity
            50.0 * grip[:, None],                   # 1
            onehot,                                 # 8
            self.prev_act,                          # 3
            self.t_in.clamp(0, 3)[:, None],         # 1
            5.0 * self._frame(hv),                  # 3
        ], dim=1)
        up_z = self._up_z()
        priv = torch.cat([up_z[:, None], self.puck.data.root_lin_vel_w.torch,
                          100.0 * (self.puck.data.root_pos_w.torch - self.pivot0)[:, :2],
                          100.0 * self._box_shift()[:, None], self._nb_tilt()[:, None]], dim=1)  # 8
        return {"policy": obs, "critic": torch.cat([obs, priv], dim=1)}

    def _get_rewards(self) -> torch.Tensor:
        cfg = self.cfg
        ph = self.ph
        up_z = self._up_z()
        ee = self._ee_b()[0]
        d = (self.p_pinch - ee).norm(dim=1)
        slide = (self.puck.data.root_pos_w.torch - self.pivot0)[:, :2].norm(dim=1)
        appr = ph == 1
        piv = (ph == 3) | (ph == 4)
        opened = (ph >= 5) & (ph <= 7)
        live = (ph >= 2) & (ph <= 4)
        self.up_max = torch.where(live, torch.maximum(self.up_max, up_z), self.up_max)
        f = self.robot.data.applied_torque.torch[:, self.grip_ids].abs().mean(dim=1)
        self.grip_sum += f * live
        self.grip_n += live
        self.grip_max = torch.where(live, torch.maximum(self.grip_max, f), self.grip_max)
        self.up_open = torch.where(ph == 4, up_z, self.up_open)
        r = cfg.w_prog * (self.d_prev - d) * appr - cfg.w_time * appr
        r = r + cfg.w_grasp * ((ph == 2) & (self.t_in < 1.5 * self.step_dt))
        r = r + cfg.w_tilt_prog * (up_z - self.up_prev) * piv + cfg.w_up * piv * up_z - cfg.w_slide * piv * slide
        r = r - cfg.w_box * self._box_shift() * (ph >= 1) * (ph <= 5)
        r = r - cfg.w_cap * (ee[:, 2] - self.p_pinch[:, 2] - cfg.cap_m).clamp(min=0) * piv
        r = r + cfg.w_open_up * opened * up_z - cfg.w_rate * self.act_rate - cfg.w_act * self.act_mag
        self.d_prev, self.up_prev = d, up_z
        done = ph >= 8
        on_box = (self.puck.data.root_pos_w.torch[:, 2] - self.scene.env_origins[:, 2] - bc.BOX_H).abs() < 0.005
        ok = (up_z > math.cos(math.radians(5.0))) & on_box & ~self.in_hand & ~self.box_moved & ~self.nb_fell
        if cfg.approach_only:
            ok = torch.ones_like(ok)
        self.stood = torch.where(ph == 5, up_z > math.cos(math.radians(5.0)), self.stood)
        self.success = torch.where(done, ok, self.success)
        self.final_err = torch.where(done, slide, self.final_err)
        r = r + done * cfg.w_success * ok
        return r - cfg.w_fail * (self.fell | self.box_moved | self.nb_fell | self.fell_over)

    def _get_dones(self):
        cfg = self.cfg
        ph = self.ph
        self.in_hand = self._in_hand()
        self.fell = (self.puck.data.root_pos_w.torch[:, 2] - self.scene.env_origins[:, 2]) < bc.BOX_H - 0.02
        self.dropped = self.fell
        self.tipped = self._base_tilt() > math.radians(10.0)
        self.box_moved = self._box_shift() > cfg.box_move_m
        self.nb_fell = self._nb_tilt() > math.radians(cfg.fall_deg)
        self.fell_over = ((ph >= 6) & (self._up_z() < math.cos(math.radians(cfg.fall_deg)))) | self.stuck
        if self.stuck.any():
            self.stats.setdefault("stuck_ph", []).extend(self.ph[self.stuck].tolist())
            self.stats.setdefault("stuck_up", []).extend(self._up_z()[self.stuck].tolist())
            self.stats.setdefault("stuck_wn", []).extend(self.puck.data.root_ang_vel_w.torch.norm(dim=1)[self.stuck].tolist())
            self.stats.setdefault("stuck_slide", []).extend((self.puck.data.root_pos_w.torch - self.pivot0)[:, :2].norm(dim=1)[self.stuck].tolist())
        fail = self.fell | self.box_moved | self.nb_fell | self.fell_over
        return (ph >= 8) | fail, self.episode_length_buf >= self.max_episode_length

    def _reset_idx(self, env_ids):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        super()._reset_idx(env_ids)
        if len(env_ids) == 0:
            return
        m, dev, cfg = len(env_ids), self.device, self.cfg
        # true pinch point (base frame, plan convention: puck centre at box top height)
        org = self.scene.env_origins[env_ids]
        u = self.u_true[env_ids]
        piv = self.pivot0[env_ids] - org - self.base_off
        piv[:, 2] = bc.BOX_H - self.base_off[2]
        self.p_pinch[env_ids] = piv + u * re.PUCK_L + torch.tensor([0.0, 0.0, cfg.contact_h_m + EE_UP], device=dev)
        self.ur[env_ids] = u
        self.lat[env_ids] = torch.stack([-u[:, 1], u[:, 0], torch.zeros(m, device=dev)], 1)
        # start state: above the puck (open) or pinching it tilted by phi (curriculum)
        start_above = torch.ones(m, dtype=torch.bool, device=dev)
        k = torch.zeros(m, dtype=torch.long, device=dev)  # waypoint index 1 + phi / 7.5
        if cfg.start_phi_deg >= 0:
            start_above[:] = False
            k[:] = int(round(cfg.start_phi_deg / 7.5))
        elif cfg.start_phi_max_deg >= 0:
            start_above = torch.rand(m, device=dev) < cfg.above_prob
            k = torch.randint(int(round(cfg.start_phi_min_deg / 7.5)), int(round(cfg.start_phi_max_deg / 7.5)) + 1, (m,), device=dev)
        wi = torch.where(start_above, torch.zeros_like(k), 1 + k)
        q0 = self.wp[env_ids, wi]
        p0, R0, *_ = self.arm.fk(q0)
        self.R_tgt[env_ids], self.x_t[env_ids], self.q_goal[env_ids], self.q_ref[env_ids] = R0, p0, q0, q0
        self.ph[env_ids] = torch.where(start_above, torch.zeros_like(wi), torch.full_like(wi, 3))
        self.t_in[env_ids] = 0.0
        self.stuck[env_ids] = False
        self.d_prev[env_ids] = (self.p_pinch[env_ids] - p0).norm(dim=1)
        self.up_prev[env_ids] = 0.0
        self.ee_obs_prev[env_ids] = p0
        pinch = ~start_above
        phi = k.float() * math.radians(7.5)
        jp = self.robot.data.default_joint_pos.torch[env_ids].clone()
        jp[:, self.arm_ids] = q0
        for gi in self.grip_ids:
            jp[:, gi] = torch.where(pinch, torch.full((m,), bc.GRIP_STOP, device=dev), torch.full((m,), self.open_q, device=dev))
        self.robot.write_joint_state_to_sim_index(position=jp, velocity=torch.zeros_like(jp), env_ids=env_ids)
        self.hist[:, env_ids] = torch.cat([q0, jp[:, self.grip_ids[0]:self.grip_ids[0] + 1]], dim=1)
        self.robot.set_joint_position_target_index(target=q0, joint_ids=self.arm_ids, env_ids=env_ids)
        if pinch.any():  # puck rotated about its near bottom edge by phi (axis cos phi u + sin phi z), still
            edge = torch.cat([self.pivot0[env_ids, :2], (org[:, 2] + bc.BOX_H)[:, None]], 1)
            off = 0.015 * torch.stack([-phi.sin() * u[:, 0], -phi.sin() * u[:, 1], phi.cos()], 1)
            root = torch.where(pinch[:, None], edge + off + torch.tensor([0.0, 0.0, 0.001], device=dev), self.puck.data.root_pos_w.torch[env_ids])
            zero = torch.zeros(m, device=dev)
            yw = torch.atan2(u[:, 1], u[:, 0])
            quat = math_utils.quat_mul(math_utils.quat_from_euler_xyz(zero, zero, yw),
                                       math_utils.quat_from_euler_xyz(zero, math.pi / 2 - phi, zero))
            cur_q = self.puck.data.root_quat_w.torch[env_ids]
            quat = torch.where(pinch[:, None], quat, cur_q)
            self.puck.write_root_pose_to_sim_index(root_pose=torch.cat([root, quat], dim=1), env_ids=env_ids)
            self.puck.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)


gym.register(
    id="Bee-Stand_c-Residual",
    entry_point=f"{__name__}:StandCEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": f"{__name__}:StandCEnvCfg"},
)
