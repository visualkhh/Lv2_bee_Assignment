"""Basket skills (skills.yaml load_slot / unload_slot): residual RL on the slot table moves of demo_cycle.py.

Bee-Load-Residual   = place_env.PlaceEnv with the 12 basket slots as targets: puck in the hand over a slot ->
    hold -> down to q_low -> settle -> open -> up -> wait; judged like a place (in the slot, upright, out of the hand).
Bee-Unload-Residual = pick_env.PickEnv on a puck sitting in a slot: start above the slot at q_over (start_over; old: transit from carry) -> down to q_grasp ->
    close -> lift to q_over -> wait; judged lifted > 30 mm over the slot floor and in the hand.
Base motion = slot_table.npz (offline IK per slot and puck kind, P = joint step per position step) with the same
perception / hand errors, latency and sag integral as the parent envs; the policy adds a joint residual.
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import gymnasium as gym
import numpy as np
import torch

import isaaclab.utils.math as math_utils
from isaaclab.envs import DirectRLEnv
from isaaclab.utils import configclass

from bee_isaac import common as bc
from bee_isaac.constants import DATA_DIR
from bee_isaac.tasks import pick_env as pk
from bee_isaac.tasks import place_env as pe

SLOTS = [(x, y, 0.09125) for x in (-0.16984, -0.10484, -0.03984) for y in (-0.0975, -0.0325, 0.0325, 0.0975)]  # base_link
ST = dict(np.load(DATA_DIR / "slot_table.npz"))
KINDS = ("box", "cyl")  # plan index = slot + 12 * kind


def slot_plan(ik, dev):
    """(24, ...) tables: q / P / end effector position / tool rotation over and at the slot, slot position."""
    rows = {k: [] for k in ("q_above", "q_down", "q_grasp", "P_above", "P_down", "P_grasp", "p_above", "rot", "rot_down", "rot_low", "cell_b")}
    for kind in KINDS:
        for s in range(12):
            q_over, q_low = ST[f"{kind}_q_over"][s], ST[f"{kind}_q_low"][s]
            p_over, r_over = ik.fk(q_over)
            rows["q_above"].append(q_over)
            rows["q_down"].append(q_low)
            rows["q_grasp"].append(ST[f"{kind}_q_grasp"][s])
            rows["P_above"].append(ST[f"{kind}_P_over"][s])
            rows["P_down"].append(ST[f"{kind}_P_low"][s])
            rows["P_grasp"].append(ST[f"{kind}_P_grasp"][s])
            rows["p_above"].append(p_over)
            rows["rot"].append(r_over)
            rows["rot_down"].append(ik.fk(ST[f"{kind}_q_grasp"][s])[1])  # unload grasp
            rows["rot_low"].append(ik.fk(q_low)[1])  # load release
            rows["cell_b"].append(SLOTS[s])
    return {k: torch.tensor(np.array(v), device=dev, dtype=torch.float32) for k, v in rows.items()}


# ---------------------------------------------------------------------------------------------------------- load
@configclass
class LoadEnvCfg(pe.PlaceEnvCfg):
    episode_length_s = 7.0
    cell_noise_m = 0.001  # the basket is fixed on the robot: slot position is known (calibration error only)
    # 1st run (11-03): success 94.8 -> 98.3 % but drops before the release 4 -> 10 / 1024 (0.4 -> 1.0 %): the policy traded
    # drops (-5) for centring (+10). A drop cannot be undone in the mission -> drop penalty 5 -> 20
    w_fail = 20.0
    # open wider than the place env (36 mm) so the fingers clear the puck by ~10 mm per side before the hand lifts (fewer jams).
    # 47.6 mm in this sim's gap_joint convention = finger joint 0.015 m, the value ROS load_skill sends (real gap ~50 mm)
    release_gap_mm = 47.6

    # ---- design-note scenario ("BEE, load residual learning design"). Every field below defaults to the old behaviour.
    # Flags are int 0/1 and "off" values are 0 (train.py --cfg casts with type(default)(str); bool("0") would be True and
    # a None default cannot be set from the command line except as a range a:b).
    # curriculum_stage: -1 = off (fields apply as set); else it overwrites the scenario fields (_apply_load_stage) cumulatively:
    #   0 none, 1 hidden offset + hidden tilt, 2 arrival error, 3 speed DR, 4 latency DR (= everything). No planner stall: the
    #   real load sends a precomputed table trajectory to the JTC in one go (no online planner), so the reference never freezes.
    curriculum_stage: int = -1
    arrive_err_m: float = 0.0  # +- [m] hand-to-slot error at the arrival over the slot (aim error), replaces cell_noise_m; scenario 0.005. 0: off
    # hidden puck offset in the hand (x along the finger faces, y along the closing axis), +- [m]. The base plan does NOT
    # compensate it (the old hand_offset_m is compensated). The actor cannot see it, the critic can (critic_hidden).
    hidden_offset_m: tuple = (0.0, 0.0)  # scenario (0.005, 0.001)
    hidden_tilt_deg: float = 0.0  # puck tilt in the hand U(0, max), random sign, about the closing axis; scenario 9
    hidden_tilt_far_deg: float = -1.0  # same for slots 0..3 (far row, hand already pitched 5 deg); < 0: use hidden_tilt_deg
    latency_ms_range: tuple | None = None  # (lo, hi) [ms]: arm command latency per episode and joint (scenario 45:62); None: measured values
    stall_prob: float = 0.0  # probability per episode of one short planner stall (reference time freezes) during hold/down
    stall_min_s, stall_max_s = 0.1, 0.5
    # The puck is teleported into the closed hand at the reset and shifts 2..5 mm while the fingers take hold (spawn artefact,
    # 11-07: +4.4 mm along the pads at slots 1, 2). 1: at the end of the hold phase the base plan re-aims q_down by that shift
    # (actual minus intended puck-in-hand pose), so the hidden offset stays the only puck-in-hand error the plan does not know.
    settle_replan: int = 1
    obs_hand_slot: int = 0  # 1: the 2 camera observations become hand-to-slot from FK (nominal puck pose in the hand), no puck truth
    obs_plan: int = 0  # 1: actor also gets plan_valid, plan_age / stall_max_s (last 2 policy observations)
    critic_hidden: int = 0  # 1: critic also gets true hand offset (3, cm) and hidden tilt (1, rad) (last 4 critic observations)

    # reward / termination. reward_v2 = 1: reward as in the note (no tilt term, no final-centring term), no dt factor
    reward_v2: int = 0
    w_align = 0.1  # exp(-(d / 5 mm)^2) per step, phases hold/down/settle only (until the fingers open)
    w_pull = 0.05  # per step after the release: puck displacement from its release pose (up and sideways), saturating at pull_scale_m
    pull_scale_m = 0.01
    w_stuck = 5.0  # at the end: puck came along with the hand (in hand, or rose > pull_rise_m after the release)
    pull_rise_m = 0.005
    w_short = 3.0  # once at the release: depth in the slot < d_rel_m
    d_rel_m = 0.0  # minimum release depth [m]; 0: off (value comes from the d_rel test, still open)
    slot_depth_m = 0.035  # slot depth, not re-checked in CAD (note)
    w_drop = 5.0  # puck dropped before the release, tipped, exited the slot, or torque collision (all end the episode)
    # immediate-termination detectors, 0 = off
    exit_rise_m = 0.0  # after the release the puck rose more than this over its release height (carried out), e.g. 0.02
    slot_exit_m = 0.0  # after the release the puck bottom is farther than this from the slot centre, e.g. 0.03
    # torque collision: any arm joint |applied torque| > collide_tau_nm while all arm joints move slower than collide_qd for
    # collide_s. Gravity holding torque is NOT subtracted and the real rule is in amperes (3.0 A, unconverted): calibrate the threshold.
    collide_tau_nm = 0.0
    collide_qd, collide_s = 0.1, 0.3


LOAD_STAGES = ("hidden", "arrive", "speed", "latency")  # curriculum order of the design note (planner stall dropped, 11-07)


def _apply_load_stage(cfg):
    """curriculum_stage k >= 0 overwrites the scenario fields: the first k of LOAD_STAGES are on (4 = everything).
    Observation / reward flags are on from stage 0 so the observation size never changes between --resume stages.
    Actor observation = the real 'place' layout (ml/arm_obs.hpp, 39): no plan_valid / plan_age."""
    s = cfg.curriculum_stage
    if s < 0:
        return
    on = lambda k: s > LOAD_STAGES.index(k)
    cfg.obs_hand_slot = cfg.critic_hidden = cfg.reward_v2 = 1
    cfg.obs_plan = 0
    cfg.hidden_offset_m = (0.005, 0.001) if on("hidden") else (0.0, 0.0)
    # the parent's plan noises too, so stage 0 is error-free like unload stage 0: puck-in-hand knowledge (with the hidden
    # offset), slot calibration (with the arrival error)
    cfg.hand_offset_m = (0.003, 0.002) if on("hidden") else (0.0, 0.0)
    cfg.hand_noise_m = 0.002 if on("hidden") else 0.0
    cfg.cell_noise_m = 0.001 if on("arrive") else 0.0
    cfg.hidden_tilt_deg = 7.5 if on("hidden") else 0.0  # pick success 5 deg x margin 1.5 (pick -> load contract)
    cfg.arrive_err_m = 0.005 if on("arrive") else 0.0  # real arrival 2-4 mm over the slot (11-08); 10 mm taught a full-limit push from step 1
    cfg.vel_scale_range = (0.15, 0.3) if on("speed") else None
    cfg.latency_ms_range = (45.0, 62.0) if on("latency") else None
    cfg.stall_prob = 0.0
    cfg.d_rel_m = 0.018  # planned release depth 20..23 mm (drop 12..15 mm in the 35 mm ring); 2..5 mm short is the margin
    cfg.exit_rise_m, cfg.slot_exit_m = 0.02, 0.03  # carried out 20 mm / bottom outside the ring (hole half width 17 mm)


def _rodrigues(v, k, th):
    """Rotate rows of v (n, 3) by th (n,) about the unit axes k (n, 3)."""
    c, s = torch.cos(th)[:, None], torch.sin(th)[:, None]
    return v * c + torch.cross(k, v, dim=1) * s + k * (k * v).sum(1, keepdim=True) * (1 - c)


class LoadEnv(pe.PlaceEnv):
    cfg: LoadEnvCfg
    TASK = "load"

    def __init__(self, cfg: LoadEnvCfg, render_mode: str | None = None, **kwargs):
        _apply_load_stage(cfg)
        cfg.observation_space = pe.OBS_DIM + 5 + 2 * int(cfg.obs_plan)
        super().__init__(cfg, render_mode, **kwargs)  # ponytail: also plans the 6 dice cells (unused, ~1 s)
        self._ik = bc.ArmIK()
        self.plan = slot_plan(self._ik, self.device)
        n, dev = self.num_envs, self.device
        z = lambda: torch.zeros(n, device=dev)
        b = lambda: torch.zeros(n, dtype=torch.bool, device=dev)
        self.hidden_tilt, self.stall_dur, self.stall_age, self.rise_max, self.coll_t = z(), z(), z(), z(), z()
        self.stall_t0 = torch.full((n,), float("inf"), device=dev)
        self.stalling, self.open_seen, self.shallow_new, self.exited, self.collided, self.replanned = b(), b(), b(), b(), b(), b()
        self.puck_open = torch.zeros(n, 3, device=dev)  # puck bottom (base frame) at the first step of the release phase
        self.ee_open, self.inhand_open, self.inhand_start = (torch.zeros(n, 3, device=dev) for _ in range(3))  # BEE_DEBUG only

    def _on(self, stage):
        """Curriculum gate: feature of this stage is active."""
        return self.cfg.curriculum_stage < 0 or self.cfg.curriculum_stage >= stage

    def _apply_action(self):
        """PlaceEnv._apply_action; a planner stall freezes the reference time (phases, base motion) for stall_dur."""
        if self.cfg.settle_replan:  # end of the hold: cancel the spawn shift of the puck in the hand (see settle_replan)
            due = (self.t >= self.t_ph[:, 0]) & ~self.replanned
            if due.any():
                pos, quat = self._ee_b()
                art = math_utils.quat_apply_inverse(quat, self._puck_bottom_b() - pos) - self.d_nom - self.hand_true
                fix = -(self.plan["rot_low"][self.cell] @ art[..., None])  # move the hand against the puck's shift
                q_new = self.q_down + (self.plan["P_down"][self.cell] @ fix)[..., 0]
                self.q_down = torch.where(due[:, None], q_new, self.q_down)
                self.replanned = self.replanned | due
        super()._apply_action()
        if self.cfg.stall_prob > 0:
            s = (self.t >= self.stall_t0) & (self.stall_age < self.stall_dur)
            self.t = self.t - self.physics_dt * s
            self.stall_age = self.stall_age + self.physics_dt * s
            self.stalling = s

    def _cam(self):
        """obs_hand_slot: slot minus the NOMINAL puck bottom in the hand (FK, exact), instead of the biased puck-truth camera."""
        if not self.cfg.obs_hand_slot:
            return super()._cam()
        pos, quat = self._ee_b()
        nominal = pos + math_utils.quat_apply(quat, self.d_nom.expand(self.num_envs, 3))
        return self.plan["cell_b"][self.cell] - nominal

    def _get_observations(self) -> dict:
        out = super()._get_observations()
        cfg = self.cfg
        if not (cfg.obs_plan or cfg.critic_hidden):
            return out
        pol = out["policy"]
        priv = out["critic"][:, pol.shape[1]:]
        if cfg.obs_plan:
            age = torch.where(self.stalling, self.stall_age, torch.zeros_like(self.stall_age))  # 0 again once the stall is over
            pol = torch.cat([pol, torch.stack([(~self.stalling).float(), age / cfg.stall_max_s], 1)], 1)
        extra = [100.0 * self.hand_true, self.hidden_tilt[:, None]] if cfg.critic_hidden else []
        return {"policy": pol, "critic": torch.cat([pol, priv, *extra], 1)}

    def _get_dones(self):
        """PlaceEnv._get_dones + release bookkeeping + optional exit / torque-collision termination (counted as dropped)."""
        term, trunc = super()._get_dones()
        cfg = self.cfg
        ph = self._phase()
        bottom, cell = self._puck_bottom_b(), self.plan["cell_b"][self.cell]
        first = (ph >= 3) & ~self.open_seen  # first step of the release phase: fingers start to open
        self.puck_open = torch.where(first[:, None], bottom, self.puck_open)
        if os.environ.get("BEE_DEBUG"):  # puck-in-hand offset right after the spawn (debug only)
            early = self.episode_length_buf <= 2
            if early.any():
                pos, quat = self._ee_b()
                inh0 = math_utils.quat_apply_inverse(quat, bottom - pos) - self.d_nom - self.hand_true
                self.inhand_start = torch.where(early[:, None], inh0, self.inhand_start)
        if os.environ.get("BEE_DEBUG") and first.any():  # hand position and puck-in-hand offset at the release (debug only)
            pos, quat = self._ee_b()
            self.ee_open = torch.where(first[:, None], pos, self.ee_open)
            inh = math_utils.quat_apply_inverse(quat, bottom - pos) - self.d_nom - self.hand_true
            self.inhand_open = torch.where(first[:, None], inh, self.inhand_open)
        depth = cfg.slot_depth_m - (bottom[:, 2] - cell[:, 2])
        self.shallow_new = first & (depth < cfg.d_rel_m) if cfg.d_rel_m > 0 else torch.zeros_like(first)
        self.open_seen = self.open_seen | first
        self.rise_max = torch.where(self.open_seen, torch.maximum(self.rise_max, bottom[:, 2] - self.puck_open[:, 2]), self.rise_max)
        ex = torch.zeros_like(term)
        if cfg.exit_rise_m > 0:
            ex = ex | (self.rise_max > cfg.exit_rise_m)
        if cfg.slot_exit_m > 0:
            ex = ex | ((bottom[:, :2] - cell[:, :2]).norm(dim=1) > cfg.slot_exit_m)
        self.exited = ex & self.open_seen & (ph < 6)
        if cfg.collide_tau_nm > 0:
            tau = self.robot.data.applied_torque.torch[:, self.arm_ids].abs().max(dim=1).values
            qd = self.robot.data.joint_vel.torch[:, self.arm_ids].abs().max(dim=1).values
            hit = (tau > cfg.collide_tau_nm) & (qd < cfg.collide_qd)
            self.coll_t = torch.where(hit, self.coll_t + self.step_dt, torch.zeros_like(self.coll_t))
            self.collided = (self.coll_t >= cfg.collide_s) & (ph < 6)
        self.dropped = self.dropped | self.exited | self.collided  # same bucket for the stats / episode counting of train.py
        return term | self.exited | self.collided, trunc

    def _get_rewards(self) -> torch.Tensor:
        """reward_v2: note's reward (returned as is, no dt factor). Else PlaceEnv's."""
        cfg = self.cfg
        if not cfg.reward_v2:
            return super()._get_rewards()
        ph = self._phase()
        bottom, cell = self._puck_bottom_b(), self.plan["cell_b"][self.cell]
        e = (bottom[:, :2] - cell[:, :2]).norm(dim=1)
        w = torch.tensor([0.2, 0.5, 1.0, 0.0, 0.0, 0.0, 0.0], device=self.device)[ph]  # alignment only until the fingers open
        r = cfg.w_align * w * torch.exp(-(e / 0.005) ** 2)
        r = r - cfg.w_rate * self.act_rate - cfg.w_act * self.act_mag
        mv = bottom - self.puck_open  # sideways drag + rise only: the puck falls 12..15 mm into the slot by design after the release
        disp = mv[:, :2].norm(dim=1) + mv[:, 2].clamp(min=0.0)
        r = r - cfg.w_pull * self.open_seen * (disp / cfg.pull_scale_m).clamp(max=1.0)  # pull-out while the hand leaves
        r = r - cfg.w_short * self.shallow_new
        done = ph >= 6
        tilt = self._puck_tilt()
        drift = (self.puck.data.root_pos_w.torch - self.puck_after_up).norm(dim=1)
        on_box = (bottom[:, 2] - cell[:, 2]).abs() < 0.005
        ok = self.held0 & ~self.in_hand & on_box & (tilt < math.radians(5.0)) & (drift < 0.002) & (e < 0.010)
        stuck = done & self.open_seen & (self.in_hand | (self.rise_max > cfg.pull_rise_m))
        r = r + done * cfg.w_success * ok - cfg.w_stuck * stuck
        self.success = torch.where(done, ok, self.success)
        self.final_err = torch.where(done, e, self.final_err)
        return r - cfg.w_drop * (self.dropped | self.tipped)

    def _reset_idx(self, env_ids):
        """PlaceEnv._reset_idx with the slot table: target = slot of this env's puck kind."""
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
            if os.environ.get("BEE_DEBUG") and done.any():  # finished but not a success: which check failed
                bottom = self._puck_bottom_b()
                cellb = self.plan["cell_b"][self.cell]
                e = (bottom[:, :2] - cellb[:, :2]).norm(dim=1) * 1000
                dz = (bottom[:, 2] - cellb[:, 2]) * 1000
                drift = (self.puck.data.root_pos_w.torch - self.puck_after_up).norm(dim=1) * 1000
                for i in env_ids[done].tolist():
                    print(f"[inh] slot {self.cell[i].item() % 12} {KINDS[self.cell[i].item() // 12]} "
                          + " ".join(f"{v * 1000:.1f}" for v in self.inhand_start[i].tolist()), flush=True)
                for i in env_ids[done & ~self.success[env_ids]].tolist():
                    print(f"[miss] slot {self.cell[i].item() % 12} {KINDS[self.cell[i].item() // 12]} centre {e[i].item():.1f} mm dz {dz[i].item():.1f} mm "
                          f"tilt {math.degrees(self._puck_tilt()[i].item()):.1f} in_hand {bool(self.in_hand[i])} drift {drift[i].item():.1f} mm "
                          f"at release xy mm {[round(v, 1) for v in ((self.puck_open[i, :2] - cellb[i, :2]) * 1000).tolist()]} dz {(self.puck_open[i, 2] - cellb[i, 2]).item() * 1000:.1f} "
                          f"hand err mm {[round(v, 1) for v in ((self.ee_open[i].cpu().numpy() - self._ik.fk(self.q_down[i].cpu().numpy())[0]) * 1000).tolist()]} "
                          f"slip in hand mm {[round(v, 1) for v in (self.inhand_open[i] * 1000).tolist()]} "
                          f"(after spawn {[round(v, 1) for v in (self.inhand_start[i] * 1000).tolist()]})", flush=True)
            if os.environ.get("BEE_DEBUG"):  # where and when the puck left the hand before the release
                for i in env_ids[self.dropped[env_ids]].tolist():
                    rel = (self._puck_bottom_b()[i] - self.plan["cell_b"][self.cell[i]]) * 1000
                    print(f"[drop] slot {self.cell[i].item() % 12} {KINDS[self.cell[i].item() // 12]} phase {self._phase()[i].item()} "
                          f"t {self.t[i].item():.2f}/{self.t_ph[i, 2].item():.2f} s puck-slot mm {[round(v) for v in rel.tolist()]}", flush=True)
        DirectRLEnv._reset_idx(self, env_ids)
        m, dev, cfg = len(env_ids), self.device, self.cfg
        u = lambda *s: torch.rand(m, *s, device=dev) * 2 - 1

        root = self.robot.data.default_root_pose.torch[env_ids].clone()
        root[:, :3] += self.scene.env_origins[env_ids]
        self.robot.write_root_pose_to_sim_index(root_pose=root, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)

        cell = torch.randint(0, 12, (m,), device=dev) + 12 * (~self._kinds()[env_ids]).long()
        rot, rot_dn = self.plan["rot"][cell], self.plan["rot_low"][cell]  # q_low tool rotation (was the unload grasp's)
        aim = torch.zeros(m, 3, device=dev)
        aim[:, :2] = (cfg.arrive_err_m if cfg.arrive_err_m > 0 and self._on(2) else cfg.cell_noise_m) * u(2)
        along = torch.cross(self.close_axis_ee.expand(m, 3), torch.tensor([0.0, 0.0, 1.0], device=dev).expand(m, 3), dim=1)
        d_true = self.d_nom + along * cfg.hand_offset_m[0] * u(1)
        d_true[:, 2] += cfg.hand_offset_m[1] * u()
        hid = torch.zeros(m, 3, device=dev)  # hidden offset: in the hand, not in the plan
        if self._on(1):
            hid = along * cfg.hidden_offset_m[0] * u(1) + self.close_axis_ee.expand(m, 3) * cfg.hidden_offset_m[1] * u(1)
        d_true = d_true + hid
        hand_err = torch.zeros(m, 3, device=dev)
        hand_err[:, :2] = cfg.hand_noise_m * u(2)
        e = d_true - hid + hand_err - self.d_nom  # measured puck offset in the hand (slot_table: q = q_nom + P @ (-R e))
        off = -(rot @ e[..., None])[..., 0] + aim
        off_dn = -(rot_dn @ e[..., None])[..., 0] + aim
        q_above = self.plan["q_above"][cell] + (self.plan["P_above"][cell] @ off[..., None])[..., 0]
        q_down = self.plan["q_down"][cell] + (self.plan["P_down"][cell] @ off_dn[..., None])[..., 0]
        dist = (q_down - q_above).abs().max(dim=1).values.cpu().numpy()
        vr = cfg.vel_scale_range if self._on(3) else None  # descent speed DR (the note: 0.15..0.3); old LoadEnv ignored the range
        vs = np.full(m, cfg.vel_scale) if vr is None else np.random.uniform(*vr, m)
        t_move = torch.tensor([bc.trapezoid_duration(float(x), float(v)) for x, v in zip(dist, vs)], device=dev)
        f = lambda v: torch.full_like(t_move, v)
        self.t_ph[env_ids] = torch.cumsum(torch.stack([f(cfg.hold_s), t_move, f(cfg.settle_s), f(cfg.release_s), t_move, f(cfg.wait_s)], 1), 1)
        # planner stall: reference time freezes for stall_dur from stall_t0 (hold / down only); the policy sees plan_valid = 0
        has = (torch.rand(m, device=dev) < cfg.stall_prob) & self._on(5)
        self.stall_dur[env_ids] = torch.where(has, torch.empty(m, device=dev).uniform_(cfg.stall_min_s, cfg.stall_max_s), torch.zeros(m, device=dev))
        self.stall_t0[env_ids] = torch.rand(m, device=dev) * self.t_ph[env_ids, 1]
        self.stall_age[env_ids] = 0.0
        self.stalling[env_ids] = False
        if cfg.latency and cfg.latency_ms_range is not None and self._on(4):  # per episode and arm joint, gripper keeps its own
            lag = torch.round(torch.empty(m, 5, device=dev).uniform_(*cfg.latency_ms_range) / 1000 / self.physics_dt).long()
            self.lag[env_ids, :5] = lag.clamp(max=self.hist.shape[0] - 1)
        self.t[env_ids] = 0.0
        self.t_tr[env_ids] = 0.0
        self.cell[env_ids], self.q_above[env_ids], self.q_down[env_ids] = cell, q_above, q_down
        self.q_ref[env_ids] = q_above
        self.aim_err[env_ids], self.hand_err[env_ids], self.hand_true[env_ids] = aim, hand_err, d_true - self.d_nom
        self.cam_bias[env_ids] = 0.0
        self.cam_bias[env_ids, :2] = cfg.cam_bias_m * u(2)
        self._sample_sag(env_ids)
        for buf in (self.delta, self.prev_act):
            buf[env_ids] = 0.0
        for buf in (self.success, self.dropped, self.tipped):
            buf[env_ids] = False
        self.held0[env_ids] = True
        for buf in (self.open_seen, self.shallow_new, self.exited, self.collided, self.replanned):
            buf[env_ids] = False
        self.rise_max[env_ids] = 0.0
        self.coll_t[env_ids] = 0.0

        jp = self.robot.data.default_joint_pos.torch[env_ids].clone()
        jp[:, self.arm_ids] = q_above
        for gi in self.grip_ids:
            jp[:, gi] = bc.GRIP_STOP
        self.robot.write_joint_state_to_sim_index(position=jp, velocity=torch.zeros_like(jp), env_ids=env_ids)
        self.hist[:, env_ids] = torch.cat([q_above, torch.zeros(m, 1, device=dev)], dim=1)
        self.robot.set_joint_position_target_index(target=q_above, joint_ids=self.arm_ids, env_ids=env_ids)

        p_ee = self.plan["p_above"][cell] + off
        # hidden tilt: the puck pivots about the closing axis through the grasp point (the fingers pinch it), random sign
        th = torch.zeros(m, device=dev)
        if self._on(1):
            tmax = torch.where(cell % 12 < 4, cfg.hidden_tilt_far_deg if cfg.hidden_tilt_far_deg >= 0 else cfg.hidden_tilt_deg, cfg.hidden_tilt_deg)
            th = math.radians(1.0) * tmax * torch.rand(m, device=dev) * torch.sign(u())
        self.hidden_tilt[env_ids] = th  # signed: the side it leans to matters
        k_ax = rot @ self.close_axis_ee
        bottom_b = p_ee + _rodrigues((rot @ d_true[..., None])[..., 0], k_ax, th)
        pos_w = bottom_b + self.base_off + self.scene.env_origins[env_ids]
        # puck axis along the tool (the far-row over poses pitch the hand 20 deg: an upright puck there sits inside the
        # fingers and is thrown out at the first step), box faces square to the fingers, cylinder any turn about its axis
        z_p = -rot[:, :, 2]
        x_p = rot @ self.close_axis_ee
        x_p = x_p - (x_p * z_p).sum(1, keepdim=True) * z_p
        x_p = x_p / x_p.norm(dim=1, keepdim=True)
        a = torch.where(self._kinds()[env_ids], math.radians(4.0) * u(), math.pi * u())[:, None]
        y_p = torch.cross(z_p, x_p, dim=1)
        x_p, y_p = torch.cos(a) * x_p + torch.sin(a) * y_p, -torch.sin(a) * x_p + torch.cos(a) * y_p
        x_p, y_p, z_p = (_rodrigues(v, k_ax, th) for v in (x_p, y_p, z_p))  # hidden tilt (identity when th = 0)
        quat = math_utils.quat_from_matrix(torch.stack([x_p, y_p, z_p], dim=2))
        self.puck.write_root_pose_to_sim_index(root_pose=torch.cat([pos_w, quat], dim=1), env_ids=env_ids)
        self.puck.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)
        self.puck_after_up[env_ids] = pos_w
        self.puck_open[env_ids] = bottom_b


# -------------------------------------------------------------------------------------------------------- unload
UNLOAD_OBS_BASE, UNLOAD_OBS_EXTRA = pk.OBS_DIM + 5, 17  # 41 + (2 older actions 10, gripper pos + load 2, tau ratio 5) = ml/arm_obs.hpp unload_slot
# curriculum (UnloadEnvCfg.curriculum_stage = k >= 0): the first k items are on, one more per stage (design note step 4)
# no tilt stage: the storage_support ring (34 mm hole, 2 mm per side for the 30 mm puck) keeps the puck upright (10-30 test: 0 deg)
UNLOAD_STAGES = ("slot", "arrive", "speed", "gap", "latency")  # no planner stall (table trajectory, as load)


def _apply_unload_stage(cfg):
    """curriculum_stage >= 0 overwrites the DR fields below with 'off' / full-scenario values (stage 5 = everything on)."""
    s = cfg.curriculum_stage
    if s < 0:
        return
    on = lambda k: s > UNLOAD_STAGES.index(k)
    cfg.slot_noise_m = 0.001 if on("slot") else 0.0  # ring: 2 mm per side, a box puck turned 3 deg has 1.2 mm
    cfg.tilt_max_deg = 0.0
    cfg.arrive_err_m = 0.005 if on("arrive") else 0.0  # as load: real arrival 2-4 mm (11-08)
    cfg.arrive_qd_rad_s = 0.05 if on("arrive") else 0.0  # leftover shaking after the arrival, as pick
    cfg.vel_scale_range = (0.15, 0.3) if on("speed") else None
    cfg.open_gap_range = (45.0, 50.0) if on("gap") else None
    cfg.latency_scale_range = (0.8, 1.2) if on("latency") else None  # assumed range, as right_env
    cfg.pause_prob = 0.0


@configclass
class UnloadEnvCfg(pk.PickEnvCfg):
    nb_prob = 0.0  # one puck per slot
    open_gap_mm = 47.5  # = skills unload_skill UNLOAD_OPEN_GAP_M (curriculum gap stage: 45..50)
    slot_noise_m = 0.001  # puck position in the slot ring: 34 mm hole, 2 mm per side (box turned 3 deg: 1.2 mm); more spawns it inside the wall
    start_over = True  # the hand has already arrived above the slot (carry -> q_over transit is outside this env); False: old start at carry
    arrive_err_m: float = -1.0  # +- [m] hand-to-slot error at the arrival above the slot (< 0: slot_noise_m); scenario: 0.005

    # ---- design-note scenario ("BEE, unload residual learning design"). Every field below defaults to the old behaviour.
    # Flags are int 0/1 (train.py --cfg casts with type(default)(str): bool("0") would be True; a None default can only be
    # set as a range a:b). curriculum_stage k >= 0 overwrites slot_noise_m, tilt_max_deg, arrive_err_m, vel_scale_range,
    # open_gap_range, latency_scale_range, pause_prob: the first k of (slot, tilt, arrive, speed, gap, latency, pause) are on.
    curriculum_stage: int = -1
    scenario_v2: int = 0  # 1: split_align = lift_score = fail_terminate = obs_extra = 1
    tilt_max_deg: float = 0.0  # puck tilt in the slot U(0, max) about a random horizontal axis; scenario 5
    open_gap_range: tuple | None = None  # (lo, hi) [mm] start opening gap per episode instead of open_gap_mm; scenario (45, 50)
    lift_vel_scale: float = 0.0  # lift speed scale; 0: same as the descent (vel_scale / vel_scale_range sample)
    arrive_qd_rad_s: float = 0.0  # +- joint speed [rad/s] of the arm right after the arrival (U per joint); 0: at rest
    pause_prob: float = 0.0  # probability per episode of one planner stall (reference time freezes) during the descent
    pause_max_s: float = 0.5  # stall length U(0, max) [s]
    obs_extra: int = 0  # 1: observation 41 -> 58 (older actions, gripper, tau ratio); critic + true slot offset, gap, stall timer
    # 1: the "camera" observation is the slot's nominal puck centre - hand from FK, face / is_box 0 — the real unload uses
    # hand_relative kind slot_puck (TF only): the wrist camera cannot be used there (perception drops pucks inside the body box)
    slot_tf: int = 0

    # reward / termination
    split_align: int = 0  # 1: step alignment = exp(-(e_y/4mm)^2) + 0.6 exp(-((e_x - s_pole)/3.5mm)^2) in the tool frame
    lift_score: int = 0  # 1: end-of-episode o_x / theta scoring and success (needs lifted + in hand)
    w_ox, w_theta, w_lift_bad = 2.5, 2.5, 3.0  # exp(-(o_x/3.5mm)^2), exp(-(theta/0.1)^2); lifted + in hand but not aligned
    ox_ok_m, theta_ok_deg, ox_sigma_m, theta_sigma_rad = 0.004, 5.0, 0.0035, 0.1
    fail_terminate: int = 0  # 1: puck dropped after the close or torque stall ends the episode with -w_fail (like knocked / tipped)
    w_tau: float = 0.0  # step penalty w_tau * sum(max(0, |tau|/tau_limit_nm - collide_ratio)); note: "small", 0 = off
    # torque collision: |tau| / tau_limit_nm >= collide_ratio and |qd| < collide_vel_rad_s for collide_hold_s, per joint.
    # Sim torque -> real current (3.0 A) conversion is undecided: collide_ratio is the stand-in (note: 0.8, same as the torque penalty)
    tau_limit_nm, collide_ratio, collide_vel_rad_s, collide_hold_s = 4.1, 0.8, 0.10, 0.3
    collide = 0  # no torque-collision failure, as pick_env (the real 3.0 A check never fires below the 2.1 A stall current)


class UnloadEnv(pk.PickEnv):
    cfg: UnloadEnvCfg
    TASK = "unload"

    def __init__(self, cfg: UnloadEnvCfg, render_mode: str | None = None, **kwargs):
        _apply_unload_stage(cfg)  # before super(): latency_scale_range sizes the latency buffer there
        if cfg.scenario_v2:
            cfg.split_align = cfg.lift_score = cfg.fail_terminate = cfg.obs_extra = 1
            cfg.slot_tf = 1
        if cfg.obs_extra:
            cfg.observation_space = UNLOAD_OBS_BASE + UNLOAD_OBS_EXTRA
        super().__init__(cfg, render_mode, **kwargs)
        n, dev = self.num_envs, self.device
        ik = bc.ArmIK()
        self.splan = slot_plan(ik, dev)
        self.slot_z = torch.zeros(n, device=dev)
        # Nominal grasp geometry per plan index, from FK at q_grasp with the puck upright at the slot centre (mid height):
        # s_pole   = puck centre - hand along the tool x (finger direction), hand frame: about 8 mm box / 4.9 mm cylinder at the
        #            pole-side slots 1, 2 (off-centre grasp on purpose), +-2.6 mm at slots 0, 3 (hand pitched 5 deg, puck upright)
        # nom_h    = the same offset in the horizontal plane along the horizontal tool x / y (pre-close alignment target;
        #            slots 0, 3 roll the hand 5 deg about the finger axis: about 5 mm along the closing axis is the plan there)
        s_pole, nom_h = [], []
        hz = lambda v: v * np.array([1.0, 1.0, 0.0]) / np.linalg.norm(v[:2])
        for kind in KINDS:
            for s in range(12):
                p, r = ik.fk(ST[f"{kind}_q_grasp"][s])
                mid = np.array(SLOTS[s]) + np.array([0.0, 0.0, bc.PUCK_H / 2]) - p
                s_pole.append(float(mid @ r[:, 0]))
                nom_h.append([float(mid @ hz(r[:, 0])), float(mid @ hz(r[:, 1]))])
        f32 = lambda v: torch.tensor(np.array(v), device=dev, dtype=torch.float32)
        self.s_pole, self.nom_h = f32(s_pole), f32(nom_h)
        self.slot_mid = f32([np.array(SLOTS[s]) + np.array([0.0, 0.0, bc.PUCK_H / 2]) for _ in KINDS for s in range(12)])  # = hand_relative slot_puck
        self.tilt0 = torch.zeros(n, device=dev)  # sampled puck tilt in the slot (not the policy's doing)
        self.grip_open = torch.full((n,), self.grip_before, device=dev)  # per-env opening before the close
        self.act2, self.act3 = torch.zeros(n, 5, device=dev), torch.zeros(n, 5, device=dev)  # actions 2 and 3 steps ago
        self.pause_t0, self.pause_left = torch.full((n,), 1e9, device=dev), torch.zeros(n, device=dev)
        self.plan_valid, self.plan_age = torch.ones(n, device=dev), torch.zeros(n, device=dev)
        self.stall_t = torch.zeros(n, 5, device=dev)  # per joint time with high torque and no motion
        self.missed = torch.zeros(n, dtype=torch.bool, device=dev)
        self.collided = torch.zeros(n, dtype=torch.bool, device=dev)
        self.ox_end, self.th_end = torch.full((n,), float("nan"), device=dev), torch.full((n,), float("nan"), device=dev)

    # ------------------------------------------------------------------ helpers
    def _legacy_cam(self, rel, face, is_box):
        if not self.cfg.slot_tf:
            return super()._legacy_cam(rel, face, is_box)
        z = torch.zeros_like(face)
        return self.slot_mid[self.cell] - self._ee_b()[0], z, z

    def _tool_axes(self):
        """Tool x (finger direction) and y (closing axis) in the base frame, (n, 3) each."""
        r = math_utils.matrix_from_quat(self._ee_b()[1])
        return r[:, :, 0], r[:, :, 1]

    def _tau_ratio(self):
        return self.robot.data.applied_torque.torch[:, self.arm_ids].abs() / self.cfg.tau_limit_nm

    def _theta_hand(self):
        """Angle between the puck axis and the tool axis (hand frame, not world vertical: the far-row q_over pitches the hand
        20 deg). load starts with the puck along the tool axis, so this is the unload -> place / load contract angle."""
        _, bq = self._base()
        up = torch.zeros(self.num_envs, 3, device=self.device)
        up[:, 2] = 1.0
        ax = math_utils.quat_apply(math_utils.quat_mul(math_utils.quat_inv(bq), self.puck.data.root_quat_w.torch), up)
        tool_z = math_utils.matrix_from_quat(self._ee_b()[1])[:, :, 2]
        return torch.acos((ax * tool_z).sum(1).abs().clamp(max=1.0))

    # ------------------------------------------------------------------ actuation hooks
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.act3, self.act2 = self.act2, self.prev_act.clone()
        super()._pre_physics_step(actions)

    def _apply_action(self) -> None:
        super()._apply_action()
        if self.cfg.pause_prob > 0:  # planner stall: undo the clock tick, the reference stays where it is (policy and sag integral run on)
            dt = self.physics_dt
            t0 = self.t - dt
            stall = (t0 >= self.pause_t0) & (self.pause_left > 0)
            self.t = torch.where(stall, t0, self.t)
            self.pause_left = self.pause_left - dt * stall
            self.plan_valid = (~stall).float()
            self.plan_age = torch.where(stall, self.plan_age + dt, torch.zeros_like(self.plan_age))

    def _send(self, tgt):
        """Latency buffer as the parent, with this episode's opening gap as the gripper target before the close."""
        tgt = tgt.clone()
        tgt[:, 5] = torch.where(self._phase() < 3, self.grip_open, tgt[:, 5])
        super()._send(tgt)

    # ------------------------------------------------------------------ MDP
    def _get_observations(self) -> dict:
        out = super()._get_observations()
        if not self.cfg.obs_extra:
            return out
        pol, priv = out["policy"], out["critic"][:, UNLOAD_OBS_BASE:]
        gp = self.robot.data.joint_pos.torch[:, self.grip_ids].mean(dim=1)
        gf = self.robot.data.applied_torque.torch[:, self.grip_ids].abs().mean(dim=1) / bc.GRIP_FORCE  # gripper load ("current")
        extra = torch.cat([self.act2, self.act3, 50.0 * gp[:, None], gf[:, None], self._tau_ratio().clamp(max=3.0)], dim=1)  # 17
        pol = torch.cat([pol, extra], dim=1)
        off = 100.0 * (self._puck_bottom_b()[:, :2] - self.splan["cell_b"][self.cell][:, :2])  # true puck offset in the slot [cm]
        critic = torch.cat([pol, priv, off, 50.0 * self.grip_open[:, None], self.stall_t.max(dim=1, keepdim=True).values], dim=1)
        return {"policy": pol, "critic": critic}

    def _get_dones(self):
        term, trunc = super()._get_dones()
        cfg, ph = self.cfg, self._phase()
        if cfg.fail_terminate:
            qd = self.robot.data.joint_vel.torch[:, self.arm_ids]
            hit = (self._tau_ratio() >= cfg.collide_ratio) & (qd.abs() < cfg.collide_vel_rad_s)
            self.stall_t = (self.stall_t + self.step_dt) * hit
            self.collided = (self.stall_t >= cfg.collide_hold_s).any(dim=1) & (ph < 6) & bool(cfg.collide)
            self.missed = (ph >= 3) & ~self.in_hand & (ph < 6)  # the puck left the hand after the close
            term = term | self.collided | self.missed
        return term, trunc

    def _get_rewards(self) -> torch.Tensor:
        """PickEnv._get_rewards with 'lifted' measured over the slot floor (no neighbour); flags add the note's scoring."""
        cfg = self.cfg
        ph = self._phase()
        rel = self._mid_b() - self._ee_b()[0]  # puck - hand
        e = rel[:, :2].norm(dim=1)
        w = torch.tensor([0.2, 0.5, 1.0, 0.0, 0.0, 0.0, 0.0], device=self.device)[ph]
        tilt = self._puck_tilt()
        ax_x, ax_y = self._tool_axes()
        o_x = (rel * ax_x).sum(dim=1) - self.s_pole[self.cell]  # hand frame: finger-direction offset minus the planned shift
        if cfg.split_align:  # before the close: horizontal plane only (a pitched hand would mix the height gap into e_x)
            flat = torch.tensor([1.0, 1.0, 0.0], device=self.device)
            hx, hy, rh = ax_x * flat, ax_y * flat, rel * flat
            hx, hy = hx / hx.norm(dim=1, keepdim=True).clamp(min=1e-6), hy / hy.norm(dim=1, keepdim=True).clamp(min=1e-6)
            nom = self.nom_h[self.cell]
            e_x, e_y = (rh * hx).sum(dim=1) - nom[:, 0], (rh * hy).sum(dim=1) - nom[:, 1]
            align = torch.exp(-(e_y / 0.004) ** 2) + 0.6 * torch.exp(-(e_x / 0.0035) ** 2)
        else:
            align = torch.exp(-(e / 0.004) ** 2)
        # tilt penalty only for tilt the hand added (the sampled tilt in the slot is not the policy's doing)
        r = cfg.w_track * w * align - cfg.w_tilt * (ph < 4) * (tilt - self.tilt0).clamp(min=0.0)
        r = r - cfg.w_rate * self.act_rate - cfg.w_act * self.act_mag
        if cfg.w_tau > 0:
            r = r - cfg.w_tau * (self._tau_ratio() - cfg.collide_ratio).clamp(min=0).sum(dim=1)
        self.final_err = torch.where(ph < 3, e, self.final_err)
        done = ph >= 6
        lifted = self._puck_bottom_b()[:, 2] - self.slot_z > 0.03
        fail = self.knocked | self.tipped
        if cfg.fail_terminate:
            fail = fail | self.missed | self.collided
        grasped = lifted & self.in_hand
        ok = grasped
        final = cfg.w_final * torch.exp(-(self.final_err / 0.004) ** 2)
        if cfg.lift_score:
            th = self._theta_hand()
            self.ox_end = torch.where(done & grasped, o_x, torch.full_like(o_x, float("nan")))
            self.th_end = torch.where(done & grasped, th, torch.full_like(th, float("nan")))
            aligned = grasped & (o_x.abs() < cfg.ox_ok_m) & (th < math.radians(cfg.theta_ok_deg))
            ok = aligned
            score = cfg.w_ox * torch.exp(-(o_x / cfg.ox_sigma_m) ** 2) + cfg.w_theta * torch.exp(-(th / cfg.theta_sigma_rad) ** 2)
            final = final + grasped * score + cfg.w_lift_bad * (grasped & ~aligned)
        if cfg.fail_terminate:
            ok = ok & ~fail  # success only without knocked / tipped / missed / collided
        r = r + done * (cfg.w_success * ok + final)
        self.success = torch.where(done, ok, self.success)
        return r - cfg.w_fail * fail

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
                if self.cfg.lift_score:
                    self.stats.setdefault("ox", []).extend(self.ox_end[ids].tolist())
                    self.stats.setdefault("theta", []).extend(self.th_end[ids].tolist())
            # "dropped" is the failure bucket train.py counts episodes with (knocked + missed + collided); causes kept apart
            self.stats["dropped"] += (self.knocked | self.missed | self.collided)[env_ids].tolist()
            for k, v in (("knocked", self.knocked), ("missed", self.missed), ("collided", self.collided)):
                self.stats.setdefault(k, []).extend(v[env_ids].tolist())
            self.stats["tipped"] += self.tipped[env_ids].tolist()
            if os.environ.get("BEE_DEBUG"):  # which slot / kind / phase the early failures come from
                bad = (self.knocked | self.missed | self.collided)[env_ids]
                rel = (self._mid_b() - self._ee_b()[0]) * 1000
                for i in env_ids[bad].tolist():
                    why = "knocked" if self.knocked[i] else "missed" if self.missed[i] else "collided"
                    print(f"[fail] {why} slot {self.cell[i].item() % 12} {KINDS[self.cell[i].item() // 12]} phase {self._phase()[i].item()} "
                          f"t {self.t[i].item():.2f} puck-hand mm {[round(v) for v in rel[i].tolist()]} "
                          f"tilt {math.degrees(self._puck_tilt()[i].item()):.0f} grip {self.robot.data.joint_pos.torch[i, self.grip_ids].mean().item() * 1000:.1f} mm "
                          f"q-qref deg {[round(v, 1) for v in torch.rad2deg(self.robot.data.joint_pos.torch[i, self.arm_ids] - self.q_ref[i]).tolist()]} "
                          f"tau {[round(v, 2) for v in self.robot.data.applied_torque.torch[i, self.arm_ids].tolist()]}", flush=True)
        DirectRLEnv._reset_idx(self, env_ids)
        m, dev, cfg = len(env_ids), self.device, self.cfg
        u = lambda *s: torch.rand(m, *s, device=dev) * 2 - 1
        org = self.scene.env_origins[env_ids]
        zero = torch.zeros(m, device=dev)

        root = self.robot.data.default_root_pose.torch[env_ids].clone()
        root[:, :3] += org
        self.robot.write_root_pose_to_sim_index(root_pose=root, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)

        # puck in a slot (box faces on the slot walls = base axes, slot_table wall_yaw), neighbour parked away
        is_box = self._kinds()[env_ids]
        slot = torch.randint(0, 12, (m,), device=dev)
        cell = slot + 12 * (~is_box).long()
        slot_b = self.splan["cell_b"][cell].clone()
        slot_b[:, :2] += cfg.slot_noise_m * u(2)
        yaw = torch.where(is_box, math.radians(3.0) * u(), math.pi * u())
        pos_w = slot_b + self.base_off + org
        quat = math_utils.quat_from_euler_xyz(zero, zero, yaw)
        tilt = zero.clone()
        # hidden puck tilt: random horizontal axis, 0..max. A standing 30 x 60 mm puck rights itself below ~26 deg
        # (tan = 15 / 30), so with a flat slot floor this is a start transient (rocking), not a lasting lean; a lasting lean
        # needs the ring geometry (puck propped on its lip), which is not in the scene yet
        if cfg.tilt_max_deg > 0:
            tilt = math.radians(cfg.tilt_max_deg) * torch.rand(m, device=dev)
            az = math.pi * u()
            quat = math_utils.quat_mul(math_utils.quat_from_angle_axis(tilt, torch.stack([torch.cos(az), torch.sin(az), zero], 1)), quat)
            pos_w[:, 2] += 0.015 * torch.sin(tilt)  # lowest rim of the 30 mm puck stays at the slot floor
        self.puck.write_root_pose_to_sim_index(root_pose=torch.cat([pos_w, quat], 1), env_ids=env_ids)
        self.puck.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)
        park = org + torch.tensor([0.6, 0.3, 0.001], device=dev)
        self.nb.write_root_pose_to_sim_index(root_pose=torch.cat([park, math_utils.quat_from_euler_xyz(zero, zero, zero)], 1), env_ids=env_ids)
        self.nb.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)
        self.nb0[env_ids], self.nb_on[env_ids] = park, False
        self.slot_z[env_ids] = slot_b[:, 2]
        self.tilt0[env_ids] = tilt

        # plan: carry -> q_over -> q_grasp, aimed with the slot error unknown (ring centres the puck)
        aim = torch.zeros(m, 3, device=dev)
        aim[:, :2] = (cfg.slot_noise_m if cfg.arrive_err_m < 0 else cfg.arrive_err_m) * u(2)  # where the base plan aims (arrival error)
        q_above = self.splan["q_above"][cell] + (self.splan["P_above"][cell] @ aim[..., None])[..., 0]
        q_down = self.splan["q_grasp"][cell] + (self.splan["P_grasp"][cell] @ aim[..., None])[..., 0]
        rd = self.splan["rot_down"][cell]
        self.tool_yaw[env_ids] = torch.atan2(rd[:, 1, 0], rd[:, 0, 0])
        self.aim_err[env_ids] = aim - torch.cat([slot_b[:, :2] - self.splan["cell_b"][cell][:, :2], zero[:, None]], 1)
        q0 = torch.tensor(np.array([ST["box_carry"][s] for s in slot.tolist()]), device=dev, dtype=torch.float32)
        mv = lambda a, b: torch.tensor([bc.trapezoid_duration(float(x), cfg.vel_scale) for x in (b - a).abs().max(dim=1).values.cpu().numpy()], device=dev)
        if cfg.start_over:  # scenario: start already above the slot, no transit
            q0 = q_above.clone()
        t_tr = torch.zeros(m, device=dev) if cfg.start_over else mv(q0, q_above)
        # descent / lift durations: speed DR (vel_scale_range) per episode, lift speed optionally separate
        vs = torch.empty(m).uniform_(*cfg.vel_scale_range).double().numpy() if cfg.vel_scale_range else np.full(m, cfg.vel_scale)
        vs_up = np.full(m, cfg.lift_vel_scale) if cfg.lift_vel_scale > 0 else vs
        dist = (q_down - q_above).abs().max(dim=1).values.cpu().numpy()
        dur = lambda v: torch.tensor([bc.trapezoid_duration(float(d), float(s)) for d, s in zip(dist, v)], device=dev)
        t_move, t_up = dur(vs), dur(vs_up)
        f = lambda v: torch.full_like(t_move, v)
        self.q_start[env_ids], self.t_tr[env_ids] = q0, t_tr
        self.t_ph[env_ids] = torch.cumsum(torch.stack([t_tr + cfg.hold_s, t_move, f(cfg.settle_s), f(cfg.release_s), t_up, f(cfg.wait_s)], 1), 1)
        self.t[env_ids] = 0.0
        self.q_above[env_ids], self.q_down[env_ids], self.q_ref[env_ids] = q_above, q_down, q0
        self.cam_bias[env_ids] = 0.0
        self.cam_bias[env_ids, :2] = cfg.cam_bias_m * u(2)
        self.cam_yaw_bias[env_ids] = math.radians(3.0) * u()
        for buf in (self.delta, self.prev_act, self.act2, self.act3, self.stall_t):
            buf[env_ids] = 0.0
        self._sample_sag(env_ids)
        self._sample_lag(env_ids)  # no-op unless latency_scale_range is set
        for buf in (self.success, self.nb_hit, self.knocked, self.dropped, self.tipped, self.missed, self.collided):
            buf[env_ids] = False
        self.final_err[env_ids] = 0.0
        self.cell[env_ids] = cell
        # opening gap before the close (same formula as bc.gap_joint, per env)
        gap = torch.empty(m, device=dev).uniform_(*cfg.open_gap_range) if cfg.open_gap_range else torch.full((m,), cfg.open_gap_mm, device=dev)
        self.grip_open[env_ids] = bc.GRIP_STOP + (gap / 1000 - 0.030) / 2
        # planner stall: the reference clock freezes for U(0, pause_max_s) at a random point of the descent
        has = torch.rand(m, device=dev) < cfg.pause_prob
        start = self.t_ph[env_ids, 0] + (0.1 + 0.8 * torch.rand(m, device=dev)) * t_move
        self.pause_t0[env_ids] = torch.where(has, start, torch.full_like(start, 1e9))
        self.pause_left[env_ids] = has.float() * cfg.pause_max_s * torch.rand(m, device=dev)
        self.plan_valid[env_ids], self.plan_age[env_ids] = 1.0, 0.0

        jp = self.robot.data.default_joint_pos.torch[env_ids].clone()
        jp[:, self.arm_ids] = q0
        for gi in self.grip_ids:
            jp[:, gi] = self.grip_open[env_ids]
        jv = torch.zeros_like(jp)
        jv[:, self.arm_ids] = cfg.arrive_qd_rad_s * u(5)  # arm still moving a little right after the arrival
        self.robot.write_joint_state_to_sim_index(position=jp, velocity=jv, env_ids=env_ids)
        self.hist[:, env_ids] = torch.cat([q0, self.grip_open[env_ids][:, None]], dim=1)
        self.robot.set_joint_position_target_index(target=q0, joint_ids=self.arm_ids, env_ids=env_ids)


for _id, _cls, _cfg in (("Bee-Load-Residual", "LoadEnv", "LoadEnvCfg"), ("Bee-Unload-Residual", "UnloadEnv", "UnloadEnvCfg")):
    gym.register(id=_id, entry_point=f"{__name__}:{_cls}", disable_env_checker=True,
                 kwargs={"env_cfg_entry_point": f"{__name__}:{_cfg}"})
