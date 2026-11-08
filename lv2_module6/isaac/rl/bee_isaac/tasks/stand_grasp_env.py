"""Bee-Stand_grasp-Residual: pinch the far end of a lying box puck exactly, then stand it up with the real arc.

2026-11-08 user decision: what makes the real stand fail is the grasp, not the arc. With the arc of the real robot
(skills.yaml skill_stand: arc 0 -> 100 deg about the near bottom edge, rising 30 mm, no way back, open at the end)
the puck stands when the fingers close on the far end; a few mm off and a finger lands on the puck (the open gap is
44 mm on a 30 mm puck: 7 mm per side) or the pinch slips. So, as the pick policy: the policy only corrects a small
joint residual (|delta| <= 0.05 rad) while the hand waits above the puck and goes down, from a gripper-camera
measurement; from the close on the residual is frozen and the scripted arc runs.

Episode = (transit) -> hold above the far end (0.5 s) -> down -> pinch (0.4 N force cap) -> arc -> pause -> open ->
lift away -> wait; judged as RightEnv (standing < 5 deg, on the box, out of the hand, box not shoved).

Observation = the real `pick` layout (ml/arm_obs.hpp, 41): q - q_ref, q_down - q_ref, 0.1 qd, q_ref - q_above,
previous action, phase one-hot (real contact phases 0 hold, 1 down, 3 close, 4 arc, 5 after), progress,
100 x camera (grasp point - hand, x y), face yaw error, is_box, 10 x sag integral.
The camera value is what the real robot can build: hand_relative "puck" rel (perceived puck centre - hand) plus the
planned (grasp point - puck centre) = u_plan x L/2, which the stand skill sends as HandTarget point - box_point.
It is captured every 0.25 s while the camera is far enough from the puck and held (re-subtracted from the moving hand)
after that; as on the real robot (rl_arm_correction) the policy only acts on a measurement younger than 0.45 s.

The puck may lie either way round (yaw -180..180): the stand skill picks the end to pinch from where the puck stood
up would land, so both ends are trained (yaw bins the IK cannot reach are left out).
"""

from __future__ import annotations

import math

import gymnasium as gym
import numpy as np
import torch

import isaaclab.utils.math as math_utils
from isaaclab.utils import configclass

from bee_isaac import common as bc
from bee_isaac.tasks import place_env as pe
from bee_isaac.tasks import right_env as rt

PUCK_L, PUCK_W = rt.PUCK_L, rt.PUCK_W
YAW_STEP = math.radians(5.0)
YAWS = np.radians(np.arange(-180, 180, 5))
OBS = 41  # = ml::arm_obs_size(ArmLayout::Pick)
SIM2REAL_PH = (0, 1, 3, 4, 5, 5, 5, 5, 5)  # sim phase -> real contact phase one-hot index


def plan_waypoints(ik, centre, yw, cfg, is_box=True):
    """rt.plan_waypoints with the real arc: arc_return False = no way back, the hand opens at the arc end."""
    if cfg.arc_return:
        return rt.plan_waypoints(ik, centre, yw, cfg, is_box)
    import pinocchio as pin
    _, r_g = ik.frame_in_ee("gripper_link")
    c0 = bc.DOWN @ r_g[:, 1]
    a, b = PUCK_L - cfg.pinch_from_end_m - cfg.press_m, rt.contact_h(cfg, is_box)
    ee_up = bc.GRASP_Z - bc.PUCK_H + (bc.GRIP_DEPTH / 2)
    u = np.array([math.cos(yw), math.sin(yw), 0.0])
    pivot = centre - u * PUCK_L / 2
    perp = np.array([-u[1], u[0], 0.0])
    yaw_t = math.atan2(perp[1], perp[0]) - math.atan2(c0[1], c0[0])
    rot = pin.utils.rotate("z", yaw_t) @ bc.DOWN
    fs = np.radians(np.arange(0.0, cfg.arc_end_deg + 1e-6, 7.5))  # = stand_plan.cpp (100 deg -> last point 97.5)
    on_arc = lambda f, h: pivot + u * (a * math.cos(f) - b * math.sin(f)) + np.array([0, 0, a * math.sin(f) + b * math.cos(f) + ee_up + h])
    arc = [on_arc(f, cfg.tilt_over_m * f / fs[-1]) for f in fs]
    pts = [arc[0] + [0, 0, cfg.above_m]] + arc + [arc[-1], arc[-1] + [0, 0, cfg.lift_away_m]]
    # as stand_plan.cpp plan_ik: above / lift-away heights come down when the arm does not reach them
    lower = {0: [arc[0] + [0, 0, h] for h in (0.03, 0.02, 0.01)], len(pts) - 1: [arc[-1] + [0, 0, h] for h in (0.06, 0.04, 0.03)]}
    hd = math.atan2(pts[1][1], pts[1][0])
    seeds = [np.array([hd, -0.2, 0.7, 1.0, s]) for s in (yaw_t - hd, hd - yaw_t, 0.0, math.pi / 2, -math.pi / 2)]
    qs, Ps = [], []
    for i, p in enumerate(pts):
        for p in [p] + lower.get(i, []):
            q, e_pos, e_ang = ik.solve(p, rot, seeds)
            if e_pos <= 2e-3 and e_ang <= math.radians(2.0):
                break
        else:
            raise ValueError(f"IK failed: yaw {math.degrees(yw):.0f} at {p}")
        if qs and np.abs(q - qs[-1]).max() > 0.8:  # wrist flip between neighbours: the real plan_ik refuses it (MAX_JUMP)
            raise ValueError(f"wrist flip: yaw {math.degrees(yw):.0f}")
        seeds = [q]
        qs.append(q)
        Ps.append(ik.jac_pos(q))
    return qs, Ps


@configclass
class StandGraspEnvCfg(rt.RightEnvCfg):
    observation_space = OBS
    episode_length_s = 14.0

    # real skill_stand (skills.yaml 11-08)
    arc_end_deg, tilt_over_m, arc_return, set_dz_m = 100.0, 0.030, False, -0.020
    above_m, lift_away_m = 0.04, 0.08
    arc_vel_scale = 0.7
    vel_scale = 0.3  # down = real contact move (arm_io CONTACT_VS)
    hold_s = 0.5  # above the puck before going down: time to measure and correct
    open_gap_mm = 44.0  # stand_open_m 0.012
    contact_h_m = 0.030
    residual_rad = 0.05
    delta_step_rad = 0.004  # = rl_arm_correction max_delta_step

    # perception left after the stand_puck_dx_m offset (11-08 real: inspect vs hand camera 1-6 mm, once 16 mm in y)
    pos_noise_m, yaw_noise = 0.008, math.radians(6.0)
    park_m, park_y_m = 0.012, 0.045
    nb_prob = 0.0  # one lying puck (11-08 user decision)
    yaw_min_deg, yaw_max_deg = -180.0, 180.0
    cam_bias_m, cam_noise_m, cam_yaw_noise = 0.002, 0.001, math.radians(3.0)
    cam_period_s, cam_age_max = 0.25, 0.45
    cam_min_up_m = 0.015  # no new capture when the hand is less than this above the pinch height (puck too close)

    # reward
    w_align_dense, w_align_close = 0.02, 5.0  # dense while going down / once at the close (main goal: the exact corner)
    w_success, w_fail = 4.0, 2.0  # standing is judged too, but the sim arc alone stands 78-82 % with a perfect grasp (11-08):
    # weighted lower so the policy does not learn a sim-only grasp offset
    al_lat, al_ax, al_z = 0.003, 0.004, 0.003  # alignment scales [m]: closing axis, puck axis, height
    w_push = 2.0  # x puck shift before the close [cm] (a finger on the puck)
    w_up = 0.01


class StandGraspEnv(rt.RightEnv):
    """Phases as RightEnv (mode tilt, no way back): 0 transit + hold, 1 down, 2 pinch, 3 arc + pause, 4 settle, 5 open,
    6 lift away, 7 wait, 8 done."""
    cfg: StandGraspEnvCfg
    TASK = "stand_grasp"

    def __init__(self, cfg: StandGraspEnvCfg, render_mode: str | None = None, **kwargs):
        cfg.scene.puck.spawn.usd_path = [u for u in cfg.scene.puck.spawn.usd_path if "/box/" in u]
        cfg.observation_space = OBS
        cfg.regrasp = 0
        # RightEnv.__init__ plans its waypoint tables as [kind][lateral][yaw] with rt.plan_waypoints over rt.YAW_BINS:
        # swap in the real arc and all yaws for the call, remember which (kind, lateral, yaw) planned
        ik0 = bc.ArmIK()
        fallback = plan_waypoints(ik0, np.array([pe.BOX_CX, 0.0, bc.BOX_H]) - ik0.base_in_root(), 0.0, cfg, True)
        valid = []

        def plan(ik, centre, yw, c, is_box=True):
            try:
                out = plan_waypoints(ik, centre, yw, c, is_box)
                valid.append(True)
                return out
            except (ValueError, AssertionError):
                valid.append(False)
                return fallback  # right shape, never drawn

        saved = rt.YAW_BINS, rt.plan_waypoints, rt.arc_groups
        rt.YAW_BINS, rt.plan_waypoints = YAWS, plan
        rt.arc_groups = lambda c, n_seg: [(0, n_seg - 1), (n_seg - 1, n_seg)]  # arc up, pause (no way back)
        try:
            super().__init__(cfg, render_mode, **kwargs)
        finally:
            rt.YAW_BINS, rt.plan_waypoints, rt.arc_groups = saved
        n, dev = self.num_envs, self.device
        ok = torch.tensor(valid, device=dev).reshape(2, len(rt.YCS), len(YAWS))[1]  # box rows
        self.yaw_ok = ok.all(dim=0)  # yaw bin planned at every lateral table position
        bad = [int(round(math.degrees(y))) for y, v in zip(YAWS, self.yaw_ok.tolist()) if not v]
        print(f"[stand_grasp] yaw bins planned {int(self.yaw_ok.sum())}/{len(YAWS)}, left out: {bad}", flush=True)
        self.open_q = bc.gap_joint(cfg.open_gap_mm)
        z = lambda *s: torch.zeros(n, *s, device=dev)
        self.yaw_meas, self.yaw_true, self.u_plan = z(), z(), z(3)
        self.cap_c, self.cap_t, self.next_cap = z(2), z() - 9.0, z()
        self.face_bias, self.pushed = z(), z()
        self.puck0 = z(3)
        self.closed_once = torch.zeros(n, dtype=torch.bool, device=dev)
        self.e_close = z(3)
        self.stats.update({"e_lat": [], "e_ax": [], "e_z": [], "pushed": [], "yaw": []})

    # ------------------------------------------------------------------ geometry
    def _centre_b(self):
        """Lying puck centre in the base frame (root = near-end face centre, puck z = its axis)."""
        z = torch.zeros(self.num_envs, 3, device=self.device)
        z[:, 2] = PUCK_L / 2
        return self._to_base(self.puck.data.root_pos_w.torch + math_utils.quat_apply(self.puck.data.root_quat_w.torch, z))

    def _grasp_target_b(self):
        """Where the hand (ee) should be at the close: above the far-end face centre at the plan's pinch height."""
        c = self._centre_b()
        g = c + self.u_true * PUCK_L / 2
        g[:, 2] = c[:, 2] - PUCK_W / 2 + self.cfg.contact_h_m + bc.GRASP_Z - bc.PUCK_H + bc.GRIP_DEPTH / 2
        return g

    def _align(self):
        """Hand - target along the closing axis (lat), the puck axis (ax) and up (z) [m]."""
        d = self._ee_b()[0] - self._grasp_target_b()
        u = self.u_true
        perp = torch.stack([-u[:, 1], u[:, 0], torch.zeros_like(u[:, 0])], 1)
        return (d * perp).sum(1), (d * u).sum(1), d[:, 2]

    def _tool_yaw(self):
        """hand_geom.tool_yaw: heading of the ee x axis (tool pointing down)."""
        _, q = self._ee_b()
        x = math_utils.quat_apply(q, torch.tensor([1.0, 0.0, 0.0], device=self.device).expand(self.num_envs, 3))
        return torch.atan2(x[:, 1], x[:, 0])

    # ------------------------------------------------------------------ camera
    def _update_cam(self):
        """Capture the perceived puck centre every cam_period_s while the hand is cam_min_up_m above the pinch."""
        up = self._align()[2]
        can = (self._phase() <= 1) & (self.t >= self.next_cap) & (up > self.cfg.cam_min_up_m) & (self.t >= self.t_tr)
        c = self._centre_b()[:, :2] + self.cam_bias[:, :2] + self.cfg.cam_noise_m * torch.randn(self.num_envs, 2, device=self.device)
        self.cap_c = torch.where(can[:, None], c, self.cap_c)
        self.cap_t = torch.where(can, self.t, self.cap_t)
        self.next_cap = torch.where(can, self.t + self.cfg.cam_period_s, self.next_cap)

    def _cam_rel(self):
        """hand_relative rel (held capture - hand now) + planned (grasp point - centre)."""
        return self.cap_c + self.u_plan[:, :2] * PUCK_L / 2 - self._ee_b()[0][:, :2]

    # ------------------------------------------------------------------ stepping
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        a = actions.clamp(-1.0, 1.0)
        self._update_cam()
        fresh = (self.t - self.cap_t) <= self.cfg.cam_age_max
        live = (self._phase() <= 1) & (self.t >= self.t_tr) & fresh
        self.delta = torch.where(live[:, None], pe.step_delta(self.delta, self.cfg.residual_rad * a, self.cfg.delta_step_rad), self.delta)
        self.act_rate = ((a - self.prev_act) ** 2).sum(dim=1) * live
        self.act_mag = (a ** 2).sum(dim=1) * live
        self.prev_act = torch.where(live[:, None], a, self.prev_act)  # real: only an acted step enters the history

    # ------------------------------------------------------------------ MDP
    def _get_observations(self) -> dict:
        ph = self._phase()
        q = self.robot.data.joint_pos.torch[:, self.arm_ids]
        qd = self.robot.data.joint_vel.torch[:, self.arm_ids]
        real_ph = torch.tensor(SIM2REAL_PH, device=self.device)[ph.clamp(max=8)]
        onehot = torch.nn.functional.one_hot(real_ph, 6).float()
        start = torch.cat([torch.zeros_like(self.t[:, None]), self.t_ph[:, :-1]], 1).gather(1, ph.clamp(max=7)[:, None])[:, 0]
        start = torch.where(ph == 0, self.t_tr, start)
        prog = (self.t - start).clamp(0, 2)
        face = torch.remainder(self.yaw_meas + self.face_bias - self._tool_yaw() + math.pi / 4, math.pi / 2) - math.pi / 4
        obs = torch.cat([
            q - self.q_ref, self.wp[:, 1] - self.q_ref, 0.1 * qd, self.q_ref - self.wp[:, 0], self.prev_act,  # 25
            onehot, prog[:, None],                                                                             # 7
            100.0 * self._cam_rel(), face[:, None], torch.ones_like(face)[:, None],                           # 4
        ], dim=1)
        obs = self._corr_obs(obs)                                                                              # +5 = 41
        lat, ax, dz = self._align()
        priv = torch.cat([100.0 * torch.stack([lat, ax, dz], 1), self._up_z()[:, None], self.puck.data.root_lin_vel_w.torch,
                          100.0 * self.aim_err, 100.0 * (self.puck.data.root_pos_w.torch - self.puck0)[:, :2],
                          100.0 * self._box_shift()[:, None], ((self.t - self.cap_t) <= self.cfg.cam_age_max).float()[:, None]], dim=1)
        return {"policy": obs, "critic": torch.cat([obs, priv], dim=1)}

    def _get_rewards(self) -> torch.Tensor:
        cfg = self.cfg
        r = super()._get_rewards()
        ph = self._phase()
        lat, ax, dz = self._align()
        score = torch.exp(-(lat / cfg.al_lat) ** 2) + 0.6 * torch.exp(-(ax / cfg.al_ax) ** 2) + 0.4 * torch.exp(-(dz / cfg.al_z) ** 2)
        r = r + cfg.w_align_dense * (ph == 1) * score
        at_close = (ph >= 2) & ~self.closed_once
        self.e_close = torch.where(at_close[:, None], torch.stack([lat, ax, dz], 1), self.e_close)
        r = r + cfg.w_align_close * at_close * score
        self.closed_once = self.closed_once | at_close
        push = (self.puck.data.root_pos_w.torch - self.puck0)[:, :2].norm(dim=1)
        self.pushed = torch.where(ph <= 1, push, self.pushed)
        return r - cfg.w_push * (ph <= 1) * 100.0 * (push - 0.002).clamp(min=0) * self.step_dt

    def _reset_idx(self, env_ids):
        """RightEnv._reset_idx without the neighbour / regrasp, the yaw drawn from the planned bins over -180..180."""
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        s = self.stats
        if self.common_step_counter > 0 and len(env_ids) > 0:
            done = self._phase()[env_ids] >= 8
            early = (self.fell | self.box_moved | self.fell_over)[env_ids] & ~done
            ended = done | early
            ids = env_ids[ended]
            ok = self.success[ids] & done[ended]
            s["success"] += ok.tolist()
            s["err"] += self.final_err[ids].tolist()
            s["dropped"] += [False] * len(ids)  # train.py n_ended counts success + dropped + tipped: failures are in success here
            s["tipped"] += [False] * len(ids)
            s["stood"] = s.get("stood", []) + self.stood[ids].tolist()
            s["box_moved"] += self.box_moved[ids].tolist()
            s["fell_over"] += (self.fell_over | self.fell)[ids].tolist()
            s["nb_fell"] += [False] * len(ids)
            seen = self.closed_once[ids]
            s["e_lat"] += self.e_close[ids][seen, 0].tolist()
            s["e_ax"] += self.e_close[ids][seen, 1].tolist()
            s["e_z"] += self.e_close[ids][seen, 2].tolist()
            s["pushed"] += self.pushed[ids].tolist()
            s["yaw"] += self.yaw_true[ids].tolist()
            if len(ids):
                self.extras.setdefault("log", {})["Metrics/success"] = ok.float().mean().item()
        rt.DirectRLEnv._reset_idx(self, env_ids)
        m, dev, cfg = len(env_ids), self.device, self.cfg
        u_ = lambda *sh: torch.rand(m, *sh, device=dev) * 2 - 1
        org = self.scene.env_origins[env_ids]
        zero = torch.zeros(m, device=dev)
        root = self.robot.data.default_root_pose.torch[env_ids].clone()
        root[:, :3] += org
        self.robot.write_root_pose_to_sim_index(root_pose=root, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)

        # lying puck: axis (near end -> far end = the end the hand pinches) from a planned yaw bin
        valid = self.yaw_ok.nonzero()[:, 0]
        bins = torch.tensor(YAWS, device=dev, dtype=torch.float32)
        yw = bins[valid[torch.randint(len(valid), (m,), device=dev)]] + 0.5 * YAW_STEP * u_()
        u = torch.stack([yw.cos(), yw.sin(), zero], 1)
        park = torch.stack([cfg.park_m * u_(), cfg.park_y_c + cfg.park_y_m * u_()], 1)
        centre = torch.stack([pe.BOX_CX + park[:, 0], park[:, 1], zero + bc.BOX_H + PUCK_W / 2 + 0.001], 1)
        root_p = centre - u * PUCK_L / 2 + org
        quat = math_utils.quat_mul(math_utils.quat_from_euler_xyz(zero, zero, yw), math_utils.quat_from_euler_xyz(zero, zero + math.pi / 2, zero))
        self.puck.write_root_pose_to_sim_index(root_pose=torch.cat([root_p, quat], dim=1), env_ids=env_ids)
        self.puck.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)
        box = self.box.data.default_root_pose.torch[env_ids].clone()
        box[:, :3] += org
        self.box.write_root_pose_to_sim_index(root_pose=box, env_ids=env_ids)
        self.box.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)
        self.box0[env_ids] = box[:, :3]
        nb_p = torch.tensor([0.6, 0.3, 0.001], device=dev).expand(m, 3) + org  # no neighbour: parked off the box
        self.nb.write_root_pose_to_sim_index(root_pose=torch.cat([nb_p, math_utils.quat_from_euler_xyz(zero, zero, zero)], 1), env_ids=env_ids)
        self.nb.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)
        self.nb_on[env_ids] = False
        self.pivot0[env_ids] = root_p
        self.u_true[env_ids] = u
        self.yaw_true[env_ids] = yw
        self.puck0[env_ids] = root_p

        # perception (after the real x offset): aim + yaw error -> yaw bin + position linearization
        aim = torch.zeros(m, 3, device=dev)
        aim[:, :2] = park + cfg.pos_noise_m * u_(2)
        y_m = yw + cfg.yaw_noise * u_()
        b = torch.remainder(torch.round((y_m - float(YAWS[0])) / YAW_STEP).long(), len(YAWS))
        b = torch.where(self.yaw_ok[b], b, valid[(valid[None] - b[:, None]).abs().argmin(dim=1)])  # unplanned bin: nearest planned
        yi = (aim[:, 1:2] - self.ycs[None]).abs().argmin(dim=1)
        rel = aim.clone()
        rel[:, 1] -= self.ycs[yi]
        self.wp[env_ids] = self.wp_nom[1, yi, b] + (self.wp_P[1, yi, b] @ rel[:, None, :, None])[..., 0]
        self.aim_err[env_ids] = aim - torch.cat([park, zero[:, None]], 1)
        self.yaw_meas[env_ids] = bins[b]
        self.u_plan[env_ids] = torch.stack([bins[b].cos(), bins[b].sin(), zero], 1)

        q0 = rt.pk.STOW.to(dev).expand(m, 5)
        dist = lambda qa, qb: torch.tensor([bc.trapezoid_duration(float(x), cfg.vel_scale) for x in (qa - qb).abs().max(dim=1).values.cpu().numpy()], device=dev)
        wp = self.wp[env_ids]
        t_tr = dist(wp[:, 0], q0)
        f = lambda v: torch.full_like(t_tr, v)
        self.arc_dur[env_ids] = rt.arc_durations(wp, self.groups, cfg.arc_vel_scale or cfg.vel_scale, cfg.hang_s)
        seg = [t_tr + cfg.hold_s, dist(wp[:, 1], wp[:, 0]), f(cfg.close_s), self.arc_dur[env_ids].sum(dim=1), f(cfg.settle_s),
               f(cfg.open_s), dist(wp[:, -1], wp[:, -2]), f(cfg.wait_s)]
        self.t_ph[env_ids] = torch.cumsum(torch.stack(seg, 1), 1)
        self.t_tr[env_ids], self.q_start[env_ids], self.q_ref[env_ids] = t_tr, q0, q0
        self.t[env_ids] = 0.0
        self._sample_lag(env_ids)
        f_pinch = cfg.pinch_force * torch.empty(m, device=dev).uniform_(*cfg.pinch_scale_range)
        self.f_pinch[env_ids] = f_pinch
        self.pinch_q[env_ids] = 0.0  # force_grip: close fully, drive force capped at the pinch force
        self.lim_cur[env_ids] = f_pinch
        self.robot.write_joint_effort_limit_to_sim_index(limits=f_pinch[:, None].expand(m, len(self.grip_ids)).contiguous(),
                                                         joint_ids=self.grip_ids, env_ids=env_ids)
        self.cam_bias[env_ids] = 0.0
        self.cam_bias[env_ids, :2] = cfg.cam_bias_m * u_(2)
        for buf in (self.delta, self.prev_act):
            buf[env_ids] = 0.0
        self._sample_sag(env_ids)
        self.ee_prev[env_ids], self.v_prev[env_ids] = 0.0, 0.0
        self.up_max[env_ids], self.up_open[env_ids] = 0.0, 0.0
        self.grip_sum[env_ids], self.grip_n[env_ids], self.grip_max[env_ids] = 0.0, 0.0, 0.0
        for buf in (self.success, self.fell, self.dropped, self.tipped, self.box_moved, self.nb_fell, self.fell_over, self.stood, self.closed_once):
            buf[env_ids] = False
        self.final_err[env_ids], self.e_close[env_ids], self.pushed[env_ids] = 0.0, 0.0, 0.0
        self.cap_t[env_ids], self.next_cap[env_ids] = -9.0, 0.0
        self.face_bias[env_ids] = cfg.cam_yaw_noise * u_()

        jp = self.robot.data.default_joint_pos.torch[env_ids].clone()
        jp[:, self.arm_ids] = q0
        for gi in self.grip_ids:
            jp[:, gi] = self.open_q
        self.robot.write_joint_state_to_sim_index(position=jp, velocity=torch.zeros_like(jp), env_ids=env_ids)
        self.hist[:, env_ids] = torch.cat([q0, torch.full((m, 1), self.open_q, device=dev)], dim=1)
        self.robot.set_joint_position_target_index(target=q0, joint_ids=self.arm_ids, env_ids=env_ids)


gym.register(
    id="Bee-Stand_grasp-Residual",
    entry_point=f"{__name__}:StandGraspEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": f"{__name__}:StandGraspEnvCfg"},
)
