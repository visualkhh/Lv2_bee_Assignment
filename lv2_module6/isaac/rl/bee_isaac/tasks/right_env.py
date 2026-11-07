"""Bee-Right-Residual: stand a lying puck up on the start box (RL task split, optional nudge skill, plan R2/R3).

A 5-axis arm cannot pitch the tool level close to the box top (R1, 2026-10-31: the wrist would sit inside the body),
so the puck is pivoted instead of flipped in the hand. The tool keeps pointing down the whole time:
    transit stow -> above the far end of the puck -> down -> pinch the far end lightly -> lift the pinch point on an
    arc about the bottom edge of the near end (the puck turns between the fingers like a hinge) -> settle ->
    open -> lift away -> wait, then judge: the puck stands (tilt < 5 deg) on the box, out of the hand.
The base motion is offline IK on the arc waypoints (per puck yaw bin, linearized in position like the other envs).
The policy adds a joint residual (|delta| <= 0.1 rad) from the descent until the fingers open.

mode "tilt" (default since 2026-10-31, user design): pinch the far end lightly, lift it a little while pulling it
toward the near end (one move, the puck swings to standing between the fingers on its near end), pause once, set it
down. The start box is a free rigid body and a standing neighbour puck may sit beside the lying one: shoving the box
(> 3 mm) or knocking the neighbour over fails the episode; the robot base may tilt.
"""

from __future__ import annotations

import math

import gymnasium as gym
import numpy as np
import pinocchio as pin
import torch

import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
from isaaclab.envs import DirectRLEnv
from isaaclab.utils import configclass

from bee_isaac import common as bc
from bee_isaac.tasks import pick_env as pk
from bee_isaac.tasks import place_env as pe

YAW_BINS = np.radians(np.arange(-90, 91, 5))  # direction near end -> far end of the lying puck (away from the robot)
PHIS = np.radians(np.arange(0, 91, 15))  # pivot angle of the puck along the arc
PUCK_L, PUCK_W = 0.060, 0.030
YCS = (-0.04, 0.0, 0.04)  # lateral puck positions the waypoint tables are solved at (the rest is linearized)
OBS_DIM = 44


def contact_h(cfg, is_box):
    return cfg.contact_h_m if is_box else cfg.contact_h_cyl_m


def plan_waypoints(ik, centre, yw, cfg, is_box=True):
    """Joint waypoints [above, start, (arc | hang) ..., lift away] for a lying puck with its centre on the box top at
    `centre` (base frame) and its axis (near end -> far end) at yaw `yw`; also their position Jacobians."""
    _, r_g = ik.frame_in_ee("gripper_link")
    c0 = bc.DOWN @ r_g[:, 1]  # closing axis in the base frame at tool yaw 0
    a, b = PUCK_L - cfg.pinch_from_end_m - cfg.press_m, contact_h(cfg, is_box)
    ee_up = bc.GRASP_Z - bc.PUCK_H + (bc.GRIP_DEPTH / 2)  # ee above the finger contact centre (tool down)
    u = np.array([math.cos(yw), math.sin(yw), 0.0])
    pivot = centre - u * PUCK_L / 2  # bottom edge of the near end
    perp = np.array([-u[1], u[0], 0.0])
    yaw_t = math.atan2(perp[1], perp[0]) - math.atan2(c0[1], c0[0])  # closing axis across the puck
    rot = pin.utils.rotate("z", yaw_t) @ bc.DOWN
    pts = [pivot + u * (a * math.cos(f) - b * math.sin(f)) + np.array([0, 0, a * math.sin(f) + b * math.cos(f) + ee_up])
           for f in PHIS]
    if cfg.mode == "hang":  # lift the pinched end: gravity swings the puck down between the fingers, then set it down
        lo = pts[0].copy()
        lo[2] = pts[0][2] - b + a + 0.002  # same, from the pinch height (b above the box top)
        hi = lo + [0, 0, 0.03]
        pts = [pts[0], pts[0] + [0, 0, 0.03], hi, hi, hi, lo, lo]
    elif cfg.mode == "tilt":  # one move along the arc about the near bottom edge past upright (to arc_end_deg),
        # rising tilt_over_m on the way (the hand has to lead: the light pinch lets the puck lag), pause, then back
        # along the same arc to upright (phi 90) and set it down there. Waypoints every 7.5 deg (joint-linear chords
        # stay within 0.5 mm of the circle)
        fs = np.radians(np.arange(0.0, cfg.arc_end_deg + 1e-6, 7.5))
        rise = lambda f: cfg.tilt_over_m * f / fs[-1]
        on_arc = lambda f, h: pivot + u * (a * math.cos(f) - b * math.sin(f)) + np.array([0, 0, a * math.sin(f) + b * math.cos(f) + ee_up + h])
        arc = [on_arc(f, rise(f)) for f in fs]
        back = [on_arc(f, rise(fs[-1])) for f in fs[::-1] if f < fs[-1] - 1e-9 and f >= math.pi / 2 - 1e-9]
        pts = arc + [arc[-1]] + back + [on_arc(math.pi / 2, cfg.set_dz_m)]
    pts = [pts[0] + [0, 0, cfg.above_m]] + pts + [pts[-1] + [0, 0, cfg.lift_away_m]]
    if cfg.regrasp:  # 2 more (split off in RightEnv): grasp on the standing puck's centre line, then lifted
        g = pivot - u * PUCK_W / 2 + np.array([0.0, 0.0, bc.GRASP_Z])
        pts += [g, g + [0, 0, cfg.regrasp_lift_m]]
    hd = math.atan2(pts[1][1], pts[1][0])
    seeds = [np.array([hd, -0.2, 0.7, 1.0, s]) for s in (yaw_t - hd, hd - yaw_t, 0.0, math.pi / 2, -math.pi / 2)]
    qs, Ps = [], []
    for p in pts:
        q, e_pos, e_ang = ik.solve(p, rot, seeds)
        assert e_pos < 2e-3 and e_ang < math.radians(2.0), f"IK failed: yaw {math.degrees(yw):.0f} at {p}"
        seeds = [q]
        qs.append(q)
        Ps.append(ik.jac_pos(q))
    return qs, Ps


def arc_groups(cfg, n_seg):
    """Arc segments (between waypoints 1 .. W-2) grouped into moves that start and stop at rest: tilt = [arc, pause,
    set down]; the other modes stop at every waypoint."""
    if cfg.mode == "tilt":  # arc up, pause, arc back + set down (see plan_waypoints)
        n = int(round(cfg.arc_end_deg / 7.5))
        return [(0, n), (n, n + 1), (n + 1, n_seg)]
    return [(k, k + 1) for k in range(n_seg)]


def _seg_len(wp):
    return (wp[:, 2:-1] - wp[:, 1:-2]).abs().max(dim=2).values  # (m, W - 3) largest joint step per segment


def arc_durations(wp, groups, vel_scale, hold_s):
    """(m, W, 5) waypoints -> (m, G) duration of each move: a trapezoid at vel_scale over its whole joint path, at
    least hold_s (a zero-length move is a pause)."""
    d = _seg_len(wp)
    L = torch.stack([d[:, i0:i1].sum(dim=1) for i0, i1 in groups], 1)
    vmax, amax = bc.VMAX * vel_scale, bc.AMAX * vel_scale
    dur = torch.where(L <= vmax * vmax / amax, 2 * (L / amax).sqrt(), L / vmax + vmax / amax)
    return dur.clamp(min=hold_s)


def arc_q(wp, groups, dur, tr):
    """Arc reference at time tr into the arc: each move one smooth step, piecewise linear through its waypoints."""
    d = _seg_len(wp)
    idx = torch.arange(wp.shape[0], device=wp.device)
    cum_t = dur.cumsum(dim=1)
    g = (tr[:, None] >= cum_t).sum(dim=1).clamp(max=len(groups) - 1)
    q = wp[:, 1].clone()
    for gi, (i0, i1) in enumerate(groups):
        sel = g == gi
        if not sel.any():
            continue
        dg = d[:, i0:i1]
        c = dg.cumsum(dim=1)
        s = bc.move_s(tr - (cum_t[:, gi] - dur[:, gi]), dur[:, gi]).clamp(0, 1)
        target = s * c[:, -1]
        k = (target[:, None] > c).sum(dim=1).clamp(max=i1 - i0 - 1)
        fr = ((target - (c[idx, k] - dg[idx, k])) / dg[idx, k].clamp(min=1e-9)).clamp(0, 1)[:, None]
        qg = wp[idx, 1 + i0 + k] + fr * (wp[idx, 2 + i0 + k] - wp[idx, 1 + i0 + k])
        q = torch.where(sel[:, None], qg, q)
    return q


@configclass
class RightSceneCfg(pk.PickSceneCfg):
    # the start box is a free cardboard box (0.4 kg) here: shoving it is a failure
    box = pk.PickSceneCfg().box.replace(spawn=pk.PickSceneCfg().box.spawn.replace(
        rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=False)))


@configclass
class RightEnvCfg(pe.PlaceEnvCfg):
    scene: RightSceneCfg = RightSceneCfg(num_envs=256, env_spacing=1.0)
    observation_space = OBS_DIM + 5
    episode_length_s = 12.0

    hold_s, close_s, settle_s, open_s, wait_s = 0.3, 0.4, 0.3, 0.3, 0.5
    hang_s = 0.5  # each pause in the arc (zero-length segment), and the shortest segment
    # (arc: every segment its own smooth step at vel_scale; was one smooth step over all segments in 2 s, which
    # stopped the joints dead at the hang pauses: a visible jerk)
    above_m = 0.04
    lift_away_m = 0.08  # after letting go: clear the standing puck before the next move
    pinch_force = 0.4  # N per finger while pivoting (full grip = bc.GRIP_FORCE). Force-capped sweep 2026-11-01 (Δ=0,
    # puck fixed, no DR): 0.4 N box 100 % cyl 88 %, 0.6 N 0-7 %, >= 1 N 0 %: the puck must turn between the fingers
    pinch_scale_range = (0.85, 1.15)  # per episode: real force control error
    latency_scale_range = (0.8, 1.2)
    residual_rad = 0.1  # the set-down has to push the top over the bottom edge (~16 mm)
    pinch_from_end_m = 0.0  # gripper centre line on the far end face (the top face once standing)
    mode = "tilt"  # arc: pivot about the near bottom edge; hang: lift the pinched end, let gravity turn it, set down;
    # tilt: arc past upright about the near bottom edge in one move, pause, set down
    tilt_over_m = 0.010  # tilt: extra rise over the arc (hand leads the puck)
    arc_end_deg = 120.0  # tilt: arc past upright, the lightly pinched puck lags the hand
    set_dz_m = -0.010  # tilt: set-down point above the standing pinch point (< 0 presses the hanging puck flat)
    arc_vel_scale = 0.5  # arc moves at this x team limits (0 = vel_scale); 0.3: box 63 %, 0.5: 100 % (fixed, no DR)
    force_grip = 1  # 1: pinch by a drive force cap (closed target) instead of a position offset (0.1 mm off the stop
    # was ~0.5 N there, so perception noise swung the real force 0.7 -> 1.2 N and the puck stopped turning)
    set_force = 0.0  # force_grip: force cap [N] from the move back to upright until the hand opens (0 = keep pinch)
    fix_nominal = 0  # eval: pin one randomized quantity to its USD value (see __init__)
    dr = 1  # 0: no domain randomization (masses, friction, latency, pinch force); only the puck shape varies
    press_m = 0.0  # arc radius shortened by this: the hand pushes the puck into its pivot edge (more friction there)
    contact_h_m = 0.030  # box: finger centre at the lying puck's top face, the fingers hold its upper half only
    # (Δ=0, box only, 1024 ep: 20 mm 27.7 %, 30 mm 43.0 %; 30 mm with the centre line 12 mm in from the end 32.7 %)
    contact_h_cyl_m = 0.015  # cylinder: on its axis, so it hangs upright (off-axis it hangs tilted, 20 mm ~10 deg)
    pos_noise_m, yaw_noise = 0.005, math.radians(10.0)
    regrasp = 0  # eval: after the hand opens, grasp the standing puck in place (wrist kept, straight down to the
    # pick grasp depth, close at full force, lift) instead of lifting away. Plan from the stand-up plan only (no
    # second perception); the frozen residual stays on. Judged like pick: lifted > 30 mm and in the hand
    regrasp_lift_m = 0.05
    yaw_min_deg, yaw_max_deg = -90.0, 90.0  # lying puck axis yaw range (eval: fix a direction)
    park_m = 0.010  # base stopping error along the heading
    park_y_c = 0.0  # eval: centre of the lateral range
    park_y_m = 0.045  # lying puck across the box (two pucks sit at +-40 mm; the demo does not re-dock for it)
    cam_bias_m, cam_noise_m = 0.002, 0.001

    nb_prob, nb_gap_m = 0.7, 0.070  # standing neighbour beside the lying puck (across its axis), either side
    box_move_m = 0.003  # start box shoved further than this: failure
    fall_deg = 30.0  # puck down again after the hand opened / neighbour knocked over: failure

    # reward
    w_up, w_slide, w_rate, w_act = 0.01, 0.0, 0.05, 0.002  # w_up: while pinched (phases 2-4); w_slide: arc mode only
    w_open_up = 0.1  # puck axis z after the hand opened (phases 5-7)
    w_jerk, jerk_deadband = 0.1, 2.0  # x (gripper velocity change per policy step [cm/s] - deadband), phases 2-4
    w_success, w_fail = 10.0, 5.0


class RightEnv(pe.PlaceEnv):
    """Phases: 0 transit + hold, 1 down, 2 pinch, 3 arc, 4 settle, 5 open, 6 lift away, 7 wait, 8 done."""

    cfg: RightEnvCfg
    TASK = "right"  # no cumotion cache yet

    def __init__(self, cfg: RightEnvCfg, render_mode: str | None = None, **kwargs):
        if not cfg.dr:
            cfg.events, cfg.latency_scale_range, cfg.pinch_scale_range = None, None, (1.0, 1.0)
            cfg.corr_gain_range = cfg.sag_scale_range = None
        elif cfg.fix_nominal:  # eval: one randomized quantity pinned to the USD value (1 puck restitution, 2 puck
            # friction, 3 finger friction, 4 puck mass, 5 latency, 6 pinch force)
            ev, k = cfg.events, int(cfg.fix_nominal)
            if k in (8, 9):  # all nominal, only the material (8) or only the mass (9) event applied
                cfg.latency_scale_range, cfg.pinch_scale_range = None, (1.0, 1.0)
                if k == 8:
                    ev.puck_mass = None
                    ev.puck_material.params.update(restitution_range=(0.4, 0.4), static_friction_range=(0.35, 0.35), dynamic_friction_range=(0.3, 0.3))
                    ev.finger_material.params.update(static_friction_range=(0.9, 0.9), dynamic_friction_range=(0.8, 0.8))
                else:
                    ev.puck_material = ev.finger_material = None
                    ev.puck_mass.params["scale_range"] = (1.0, 1.0)
            if k == 7:  # all of them (events still applied, at the USD values)
                ev.puck_material.params.update(restitution_range=(0.4, 0.4), static_friction_range=(0.35, 0.35), dynamic_friction_range=(0.3, 0.3))
                ev.finger_material.params.update(static_friction_range=(0.9, 0.9), dynamic_friction_range=(0.8, 0.8))
                ev.puck_mass.params["scale_range"] = (1.0, 1.0)
                cfg.latency_scale_range, cfg.pinch_scale_range = None, (1.0, 1.0)
            if k == 1:
                ev.puck_material.params["restitution_range"] = (0.4, 0.4)
            elif k == 2:
                ev.puck_material.params["static_friction_range"], ev.puck_material.params["dynamic_friction_range"] = (0.35, 0.35), (0.3, 0.3)
            elif k == 3:
                ev.finger_material.params["static_friction_range"], ev.finger_material.params["dynamic_friction_range"] = (0.9, 0.9), (0.8, 0.8)
            elif k == 4:
                ev.puck_mass.params["scale_range"] = (1.0, 1.0)
            elif k == 5:
                cfg.latency_scale_range = None
            elif k == 6:
                cfg.pinch_scale_range = (1.0, 1.0)
        if cfg.regrasp:
            cfg.episode_length_s += 4.0
        super().__init__(cfg, render_mode, **kwargs)  # ponytail: also plans the 6 place cells (unused, ~1 s)
        n, dev = self.num_envs, self.device
        self.nb = self.scene["nb"]
        self.open_q = bc.gap_joint(40.0)
        self.pinch_q = torch.full((n,), bc.GRIP_STOP * (1 - cfg.pinch_force / bc.GRIP_FORCE), device=dev)  # force = stiffness x (stop - target)

        # ---- offline plan: per yaw bin, joint waypoints above, arc phi 0..90, lift away; position Jacobians
        ik = bc.ArmIK()
        tabs = [[list(zip(*(plan_waypoints(ik, np.array([pe.BOX_CX, yc, bc.BOX_H]) - ik.base_in_root(), yw, cfg, is_box=k == 1)
                            for yw in YAW_BINS))) for yc in YCS] for k in (0, 1)]
        self.wp_nom = torch.tensor(np.array([[t[0] for t in r] for r in tabs]), device=dev, dtype=torch.float32)  # (cyl/box, ycs, bins, W, 5)
        self.wp_P = torch.tensor(np.array([[t[1] for t in r] for r in tabs]), device=dev, dtype=torch.float32)  # (.., W, 5, 3)
        self.ycs = torch.tensor(YCS, device=dev)
        if cfg.regrasp:  # last 2 waypoints: regrasp grasp / lift
            self.rg_nom, self.rg_P = self.wp_nom[..., -2:, :], self.wp_P[..., -2:, :, :]
            self.wp_nom, self.wp_P = self.wp_nom[..., :-2, :], self.wp_P[..., :-2, :, :]
        W = self.wp_nom.shape[3]

        z = lambda *s: torch.zeros(n, *s, device=dev)
        self.wp = z(W, 5)
        self.groups = arc_groups(cfg, W - 3)
        self.arc_dur = z(len(self.groups))
        self.t_ph = z(8)
        self.rg, self.rg_d, self.t_close = z(2, 5), z(), z()  # regrasp: grasp / lift q, descent time, close start
        self.stood = torch.zeros(n, dtype=torch.bool, device=dev)
        self.u_true = z(3)
        self.slid = torch.zeros(n, dtype=torch.bool, device=dev)
        self.fell = torch.zeros(n, dtype=torch.bool, device=dev)
        self.pivot0 = z(3)
        self.stats = {"success": [], "err": [], "dropped": [], "tipped": [], "box_moved": [], "nb_fell": [], "fell_over": []}
        self.box = self.scene["box"]
        self.box0 = z(3)
        self.nb_on = torch.zeros(n, dtype=torch.bool, device=dev)
        self.ee_prev, self.v_prev = z(3), z(3)
        self.up_max, self.up_open = z(), z()
        self.grip_sum, self.grip_n, self.grip_max = z(), z(), z()
        self.f_pinch, self.lim_cur = z(), z()  # diagnostics: best axis z while pinched, axis z when the hand opens
        self.box_moved, self.nb_fell, self.fell_over = (torch.zeros(n, dtype=torch.bool, device=dev) for _ in range(3))

    # ------------------------------------------------------------------ helpers
    def _up_z(self):
        """z of the puck axis (1 = standing, 0 = lying)."""
        up = torch.zeros(self.num_envs, 3, device=self.device)
        up[:, 2] = 1.0
        return math_utils.quat_apply(self.puck.data.root_quat_w.torch, up)[:, 2]

    def _axis(self):
        """Puck axis (near end -> far end) in the base frame: which way it leans, not only how much."""
        z = torch.zeros(self.num_envs, 3, device=self.device)
        z[:, 2] = 1.0
        _, bq = self._base()
        return math_utils.quat_apply_inverse(bq, math_utils.quat_apply(self.puck.data.root_quat_w.torch, z))

    def _phase(self):
        return (self.t[:, None] >= self.t_ph).sum(dim=1)

    def _ref(self, t):
        ph = (t[:, None] >= self.t_ph).sum(dim=1)
        s = lambda i0, i1: bc.move_s(t - self.t_ph[:, i0], (self.t_ph[:, i1] - self.t_ph[:, i0]).clamp(min=1e-3))
        q = torch.where((t < self.t_tr)[:, None], self.q_start + bc.move_s(t, self.t_tr.clamp(min=1e-3))[:, None] * (self.wp[:, 0] - self.q_start),
                        self.wp[:, 0])
        q = torch.where((ph == 1)[:, None], self.wp[:, 0] + s(0, 1)[:, None] * (self.wp[:, 1] - self.wp[:, 0]), q)
        q = torch.where((ph == 2)[:, None], self.wp[:, 1], q)
        q = torch.where((ph == 3)[:, None], arc_q(self.wp, self.groups, self.arc_dur, t - self.t_ph[:, 2]), q)
        q = torch.where(((ph == 4) | (ph == 5))[:, None], self.wp[:, -2], q)
        if self.cfg.regrasp:  # 6: down to the grasp, hold, close; 7: lift, wait
            t_up = self.t_ph[:, 7] - self.cfg.wait_s
            dn = bc.move_s(t - self.t_ph[:, 5], self.rg_d.clamp(min=1e-3))[:, None]
            up = bc.move_s(t - self.t_ph[:, 6], (t_up - self.t_ph[:, 6]).clamp(min=1e-3))[:, None]
            q = torch.where((ph == 6)[:, None], self.wp[:, -2] + dn * (self.rg[:, 0] - self.wp[:, -2]), q)
            q = torch.where((ph >= 7)[:, None], self.rg[:, 0] + up * (self.rg[:, 1] - self.rg[:, 0]), q)
            closed = (ph >= 6) & (t >= self.t_close)
            g = torch.where(((ph >= 2) & (ph <= 4)) | closed, self.pinch_q, torch.full_like(t, self.open_q))
            g = torch.where(closed, torch.zeros_like(g), g)
            hold = ((ph == 0) & (t >= self.t_tr)) | (ph == 2) | (ph == 4) | (ph == 5) | \
                ((ph == 6) & (t >= self.t_ph[:, 5] + self.rg_d)) | ((ph == 7) & (t >= t_up))
            return q, g, hold, ph
        q = torch.where((ph == 6)[:, None], self.wp[:, -2] + s(5, 6)[:, None] * (self.wp[:, -1] - self.wp[:, -2]), q)
        q = torch.where((ph >= 7)[:, None], self.wp[:, -1], q)
        g = torch.where((ph >= 2) & (ph <= 4), self.pinch_q, torch.full_like(t, self.open_q))
        hold = ((ph == 0) & (t >= self.t_tr)) | (ph == 2) | (ph == 4) | (ph == 5) | (ph == 7)
        return q, g, hold, ph

    # ------------------------------------------------------------------ stepping
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        a = actions.clamp(-1.0, 1.0)
        ph = self._phase()
        live = (ph >= 1) & (ph <= 4)
        self.delta = torch.where(live[:, None], self.cfg.residual_rad * a, self.delta)
        self.act_rate = ((a - self.prev_act) ** 2).sum(dim=1) * live
        self.act_mag = (a ** 2).sum(dim=1) * live
        self.prev_act = a.clone()

    def _apply_action(self) -> None:
        dt = self.physics_dt
        q_ref, g, hold, _ = self._ref(self.t)
        self.q_ref, self.grip_ref = q_ref, g
        self._send(torch.cat([self._sag_step(q_ref, hold), g[:, None]], dim=1))
        if self.cfg.force_grip and (self.cfg.set_force > 0 or self.cfg.regrasp):
            lim = self.f_pinch
            if self.cfg.set_force > 0:
                # grip firmly once the puck hangs upright (from the move back to upright on): it is set down square
                firm = self.t >= self.t_ph[:, 2] + self.arc_dur[:, :2].sum(dim=1)
                lim = torch.where(firm, torch.full_like(self.f_pinch, self.cfg.set_force), lim)
            if self.cfg.regrasp:  # full grip force for the regrasp close
                lim = torch.where((self._phase() >= 6) & (self.t >= self.t_close), torch.full_like(lim, bc.GRIP_FORCE), lim)
            ch = (lim != self.lim_cur).nonzero()[:, 0]
            if len(ch):
                self.lim_cur[ch] = lim[ch]
                self.robot.write_joint_effort_limit_to_sim_index(limits=lim[ch, None].expand(len(ch), len(self.grip_ids)).contiguous(),
                                                                 joint_ids=self.grip_ids, env_ids=ch)
        self.t = self.t + dt

    # ------------------------------------------------------------------ MDP
    def _get_observations(self) -> dict:
        ph = self._phase()
        q = self.robot.data.joint_pos.torch[:, self.arm_ids]
        qd = self.robot.data.joint_vel.torch[:, self.arm_ids]
        onehot = torch.nn.functional.one_hot(ph.clamp(max=7), 8).float()
        prog = (self.t - torch.cat([torch.zeros_like(self.t[:, None]), self.t_ph[:, :-1]], 1).gather(1, ph.clamp(max=7)[:, None])[:, 0]).clamp(0, 3)
        up_z = self._up_z()
        rel = self._to_base(self.puck.data.root_pos_w.torch) - self._ee_b()[0]  # near-end bottom face centre - hand
        cam = rel + self.cam_bias + self.cfg.cam_noise_m * torch.randn_like(rel)
        obs = torch.cat([
            q - self.q_ref, self.wp[:, -2] - self.q_ref, 0.1 * qd, self.prev_act,  # 20
            onehot, prog[:, None],                                              # 9
            100.0 * cam, self._axis() + 0.03 * torch.randn(self.num_envs, 3, device=self.device),  # 6
            self._kinds().float()[:, None],                                     # 1
            self.q_ref - self.wp[:, 1],                                         # 5
            10.0 * self._nb_rel() * self.nb_on[:, None], self.nb_on.float()[:, None],  # 3: neighbour - puck centre
        ], dim=1)
        obs = self._corr_obs(obs)                                               # (+5)
        priv = torch.cat([100.0 * rel, up_z[:, None], self.puck.data.root_lin_vel_w.torch, 100.0 * self.aim_err,
                          100.0 * (self.puck.data.root_pos_w.torch - self.pivot0)[:, :2],
                          100.0 * self._box_shift()[:, None], self._nb_tilt()[:, None]], dim=1)  # 14
        return {"policy": obs, "critic": torch.cat([obs, priv], dim=1)}

    def _nb_rel(self):
        """Neighbour - lying puck centre, base frame xy [m]."""
        z = torch.zeros(self.num_envs, 3, device=self.device)
        z[:, 2] = PUCK_L / 2
        c = self.puck.data.root_pos_w.torch + math_utils.quat_apply(self.puck.data.root_quat_w.torch, z)
        return (self._to_base(self.nb.data.root_pos_w.torch) - self._to_base(c))[:, :2]

    def _nb_tilt(self):
        z = torch.zeros(self.num_envs, 3, device=self.device)
        z[:, 2] = 1.0
        return torch.acos(math_utils.quat_apply(self.nb.data.root_quat_w.torch, z)[:, 2].clamp(-1, 1)) * self.nb_on

    def _box_shift(self):
        return (self.box.data.root_pos_w.torch - self.box0)[:, :2].norm(dim=1)

    def _get_rewards(self) -> torch.Tensor:
        cfg = self.cfg
        ph = self._phase()
        up_z = self._up_z()
        slide = (self.puck.data.root_pos_w.torch - self.pivot0)[:, :2].norm(dim=1)  # the near end should stay put
        live = (ph >= 2) & (ph <= 4)
        opened = (ph >= 5) & (ph <= 7)
        self.up_max = torch.where(live, torch.maximum(self.up_max, up_z), self.up_max)
        # squeeze force per finger [N] (implicit drive: stiffness x position error), while pinched
        f = self.robot.data.applied_torque.torch[:, self.grip_ids].abs().mean(dim=1)
        self.grip_sum += f * live
        self.grip_n += live
        self.grip_max = torch.where(live, torch.maximum(self.grip_max, f), self.grip_max)
        self.up_open = torch.where(ph == 4, up_z, self.up_open)
        # gripper velocity change per policy step beyond what the smooth base motion needs (jerks)
        ee = self._ee_b()[0]
        v = (ee - self.ee_prev) / self.step_dt
        dv = 100.0 * (v - self.v_prev).norm(dim=1)  # cm/s
        self.ee_prev, self.v_prev = ee, v
        jerk = (dv - cfg.jerk_deadband).clamp(min=0) * live * (self.episode_length_buf > 2)
        r = cfg.w_up * live * up_z + cfg.w_open_up * opened * up_z - cfg.w_slide * live * slide - cfg.w_jerk * jerk
        r = r - cfg.w_rate * self.act_rate - cfg.w_act * self.act_mag
        done = ph >= 8
        on_box = (self.puck.data.root_pos_w.torch[:, 2] - self.scene.env_origins[:, 2] - bc.BOX_H).abs() < 0.005
        ok = (up_z > math.cos(math.radians(5.0))) & on_box & ~self.in_hand & ~self.box_moved & ~self.nb_fell
        self.stood = torch.where(ph == 5, up_z > math.cos(math.radians(5.0)), self.stood)  # upright with the hand open
        if cfg.regrasp:
            lifted = (self.puck.data.root_pos_w.torch[:, 2] - self.scene.env_origins[:, 2]) - bc.BOX_H > 0.03
            ok = lifted & self.in_hand & ~self.box_moved & ~self.nb_fell
        self.success = torch.where(done, ok, self.success)
        self.final_err = torch.where(done, slide, self.final_err)
        r = r + done * cfg.w_success * ok
        return r - cfg.w_fail * (self.fell | self.box_moved | self.nb_fell | self.fell_over)

    def _get_dones(self):
        cfg = self.cfg
        ph = self._phase()
        self.in_hand = self._in_hand()
        # off the box top (fell down the side) or dropped from the hand above the box
        self.fell = (self.puck.data.root_pos_w.torch[:, 2] - self.scene.env_origins[:, 2]) < bc.BOX_H - 0.02
        self.dropped = self.fell
        self.tipped = self._base_tilt() > math.radians(10.0)  # logged only: the base may tilt
        self.box_moved = self._box_shift() > cfg.box_move_m
        self.nb_fell = self._nb_tilt() > math.radians(cfg.fall_deg)
        # down again after the hand opened and lifted away (or never stood up)
        self.fell_over = (ph >= 6) & (self._up_z() < math.cos(math.radians(cfg.fall_deg)))
        fail = self.fell | self.box_moved | self.nb_fell | self.fell_over
        return (ph >= 8) | fail, self.episode_length_buf >= self.max_episode_length

    def _reset_idx(self, env_ids):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        if self.common_step_counter > 0 and len(env_ids) > 0:
            done = self._phase()[env_ids] >= 8
            if done.any():
                ids = env_ids[done]
                log = self.extras.setdefault("log", {})
                log["Metrics/success"] = self.success[ids].float().mean().item()
                self.stats["success"] += self.success[ids].tolist()
                self.stats["err"] += self.final_err[ids].tolist()
            self.stats["dropped"] += self.fell[env_ids].tolist()
            # early failures count as finished, unsuccessful episodes (tipped is not a failure here: not counted)
            early = (self.box_moved | self.nb_fell | self.fell_over)[env_ids] & ~self.fell[env_ids] & ~done
            self.stats["success"] += [False] * int(early.sum().item())
            # per finished episode: puck kind (1 box), success, mean / max squeeze force while pinched
            fin = done | early | self.fell[env_ids]
            ids = env_ids[fin]
            ok = torch.where(done[fin], self.success[ids], torch.zeros_like(self.success[ids]))
            self.stats.setdefault("by_kind", []).extend(zip(self._kinds()[ids].tolist(), ok.tolist(),
                                                           (self.grip_sum[ids] / self.grip_n[ids].clamp(min=1)).tolist(),
                                                           self.grip_max[ids].tolist()))
            self.stats.setdefault("stood", []).extend(self.stood[ids].tolist())
            self.stats["box_moved"] += self.box_moved[env_ids].tolist()
            self.stats.setdefault("up_max", []).extend(self.up_max[env_ids].tolist())
            self.stats.setdefault("up_open", []).extend(self.up_open[env_ids].tolist())
            self.stats["nb_fell"] += self.nb_fell[env_ids].tolist()
            self.stats["fell_over"] += self.fell_over[env_ids].tolist()
        DirectRLEnv._reset_idx(self, env_ids)
        m, dev, cfg = len(env_ids), self.device, self.cfg
        u_ = lambda *s: torch.rand(m, *s, device=dev) * 2 - 1
        org = self.scene.env_origins[env_ids]
        zero = torch.zeros(m, device=dev)

        root = self.robot.data.default_root_pose.torch[env_ids].clone()
        root[:, :3] += org
        self.robot.write_root_pose_to_sim_index(root_pose=root, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)

        # lying puck: axis along yaw (near end -> far end), random within +-90 deg; root = bottom face = near end
        lo, hi = math.radians(cfg.yaw_min_deg), math.radians(cfg.yaw_max_deg)
        yw = lo + (hi - lo) * (u_() + 1) / 2
        u = torch.stack([yw.cos(), yw.sin(), zero], 1)
        park = torch.stack([cfg.park_m * u_(), cfg.park_y_c + cfg.park_y_m * u_()], 1)  # the lying puck anywhere across the box
        centre = torch.stack([pe.BOX_CX + park[:, 0], park[:, 1], zero + bc.BOX_H + PUCK_W / 2 + 0.001], 1)
        root_p = centre - u * PUCK_L / 2 + org
        quat = math_utils.quat_mul(math_utils.quat_from_euler_xyz(zero, zero, yw),
                                   math_utils.quat_from_euler_xyz(zero, zero + math.pi / 2, zero))
        self.puck.write_root_pose_to_sim_index(root_pose=torch.cat([root_p, quat], dim=1), env_ids=env_ids)
        self.puck.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)
        # start box back in place (free body: it may have been shoved)
        box = self.box.data.default_root_pose.torch[env_ids].clone()
        box[:, :3] += org
        self.box.write_root_pose_to_sim_index(root_pose=box, env_ids=env_ids)
        self.box.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)
        self.box0[env_ids] = box[:, :3]
        # standing neighbour across the lying puck's axis, either side (else parked off the box)
        on = torch.rand(m, device=dev) < cfg.nb_prob
        perp = torch.stack([-u[:, 1], u[:, 0], zero], 1)
        off = (park * perp[:, :2]).sum(dim=1)  # lying puck off the box centre across its axis: neighbour on the inner side
        side = torch.where(off.abs() > 0.01, -off.sign(), torch.where(torch.rand(m, device=dev) < 0.5, -1.0, 1.0))
        nb_p = centre + side[:, None] * cfg.nb_gap_m * perp + torch.cat([0.003 * u_(2), zero[:, None]], 1)
        nb_p[:, 2] = bc.BOX_H + 0.001
        on = on & ((nb_p[:, 0] - pe.BOX_CX).abs() < 0.08) & (nb_p[:, 1].abs() < 0.08)  # stays on the box top        nb_p = torch.where(on[:, None], nb_p, torch.tensor([0.6, 0.3, 0.001], device=dev).expand(m, 3)) + org
        nb_q = math_utils.quat_from_euler_xyz(zero, zero, math.pi * u_())
        self.nb.write_root_pose_to_sim_index(root_pose=torch.cat([nb_p, nb_q], 1), env_ids=env_ids)
        self.nb.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)
        self.nb_on[env_ids] = on
        self.pivot0[env_ids] = root_p
        self.u_true[env_ids] = u

        # perception -> yaw bin + position linearization of every waypoint
        aim = torch.zeros(m, 3, device=dev)
        aim[:, :2] = park + cfg.pos_noise_m * u_(2)
        yw_meas = (yw + cfg.yaw_noise * u_()).clamp(-math.pi / 2, math.pi / 2)
        b = torch.round((yw_meas - YAW_BINS[0]) / math.radians(5.0)).long().clamp(0, len(YAW_BINS) - 1)
        kd = self._kinds()[env_ids].long()
        yi = (aim[:, 1:2] - self.ycs[None]).abs().argmin(dim=1)
        rel = aim.clone()
        rel[:, 1] -= self.ycs[yi]
        self.wp[env_ids] = self.wp_nom[kd, yi, b] + (self.wp_P[kd, yi, b] @ rel[:, None, :, None])[..., 0]
        self.aim_err[env_ids] = aim - torch.cat([park, zero[:, None]], 1)

        q0 = pk.STOW.to(dev).expand(m, 5)
        dist = lambda qa, qb: torch.tensor([bc.trapezoid_duration(float(x), cfg.vel_scale) for x in (qa - qb).abs().max(dim=1).values.cpu().numpy()], device=dev)
        wp = self.wp[env_ids]
        t_tr = dist(wp[:, 0], q0)
        f = lambda v: torch.full_like(t_tr, v)
        self.arc_dur[env_ids] = arc_durations(wp, self.groups, cfg.arc_vel_scale or cfg.vel_scale, cfg.hang_s)
        seg = [t_tr + cfg.hold_s, dist(wp[:, 1], wp[:, 0]), f(cfg.close_s), self.arc_dur[env_ids].sum(dim=1), f(cfg.settle_s), f(cfg.open_s),
               dist(wp[:, -1], wp[:, -2]), f(cfg.wait_s)]
        if cfg.regrasp:
            rg = self.rg_nom[kd, yi, b] + (self.rg_P[kd, yi, b] @ rel[:, None, :, None])[..., 0]
            self.rg[env_ids], self.rg_d[env_ids] = rg, dist(rg[:, 0], wp[:, -2])
            seg[6] = self.rg_d[env_ids] + cfg.hold_s + 0.5  # down, hold (sag integral), close (demo CLOSE_S)
            seg[7] = dist(rg[:, 1], rg[:, 0]) + cfg.wait_s
        self.t_ph[env_ids] = torch.cumsum(torch.stack(seg, 1), 1)
        if cfg.regrasp:
            self.t_close[env_ids] = self.t_ph[env_ids, 6] - 0.5
        self.t_tr[env_ids], self.q_start[env_ids], self.q_ref[env_ids] = t_tr, q0, q0
        self.t[env_ids] = 0.0
        self._sample_lag(env_ids)
        f_pinch = cfg.pinch_force * torch.empty(m, device=dev).uniform_(*cfg.pinch_scale_range)
        self.f_pinch[env_ids] = f_pinch
        if cfg.force_grip:  # current-limited gripper (Dynamixel): close fully, drive force capped at the pinch force
            self.pinch_q[env_ids] = 0.0
            self.lim_cur[env_ids] = f_pinch
            lim = f_pinch[:, None].expand(m, len(self.grip_ids)).contiguous()
            self.robot.write_joint_effort_limit_to_sim_index(limits=lim, joint_ids=self.grip_ids, env_ids=env_ids)
        else:  # position offset: force = stiffness x overlap, 0.1 mm off the stop is ~0.5 N
            self.pinch_q[env_ids] = bc.GRIP_STOP * (1 - f_pinch / bc.GRIP_FORCE)
        self.cam_bias[env_ids] = 0.0
        self.cam_bias[env_ids, :2] = cfg.cam_bias_m * u_(2)
        for buf in (self.delta, self.prev_act):
            buf[env_ids] = 0.0
        self._sample_sag(env_ids)
        self.ee_prev[env_ids], self.v_prev[env_ids] = 0.0, 0.0
        self.up_max[env_ids], self.up_open[env_ids] = 0.0, 0.0
        self.grip_sum[env_ids], self.grip_n[env_ids], self.grip_max[env_ids] = 0.0, 0.0, 0.0
        for buf in (self.success, self.fell, self.dropped, self.tipped, self.box_moved, self.nb_fell, self.fell_over, self.stood):
            buf[env_ids] = False
        self.final_err[env_ids] = 0.0

        jp = self.robot.data.default_joint_pos.torch[env_ids].clone()
        jp[:, self.arm_ids] = q0
        for gi in self.grip_ids:
            jp[:, gi] = self.open_q
        self.robot.write_joint_state_to_sim_index(position=jp, velocity=torch.zeros_like(jp), env_ids=env_ids)
        self.hist[:, env_ids] = torch.cat([q0, torch.full((m, 1), self.open_q, device=dev)], dim=1)
        self.robot.set_joint_position_target_index(target=q0, joint_ids=self.arm_ids, env_ids=env_ids)


gym.register(
    id="Bee-Right-Residual",
    entry_point=f"{__name__}:RightEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": f"{__name__}:RightEnvCfg"},
)
