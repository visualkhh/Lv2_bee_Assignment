"""Basket skills (skills.yaml load_slot / unload_slot): residual RL on the slot table moves of demo_cycle.py.

Bee-Load-Residual   = place_env.PlaceEnv with the 12 basket slots as targets: puck in the hand over a slot ->
    hold -> down to q_low -> settle -> open -> up -> wait; judged like a place (in the slot, upright, out of the hand).
Bee-Unload-Residual = pick_env.PickEnv on a puck sitting in a slot: transit carry -> q_over -> down to q_grasp ->
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

import bee_common as bc
import pick_env as pk
import place_env as pe

SLOTS = [(x, y, 0.09125) for x in (-0.16984, -0.10484, -0.03984) for y in (-0.0975, -0.0325, 0.0325, 0.0975)]  # base_link
ST = dict(np.load(Path(__file__).resolve().parent / "slot_table.npz"))
KINDS = ("box", "cyl")  # plan index = slot + 12 * kind


def slot_plan(ik, dev):
    """(24, ...) tables: q / P / end effector position / tool rotation over and at the slot, slot position."""
    rows = {k: [] for k in ("q_above", "q_down", "q_grasp", "P_above", "P_down", "p_above", "rot", "rot_down", "cell_b")}
    for kind in KINDS:
        for s in range(12):
            q_over, q_low = ST[f"{kind}_q_over"][s], ST[f"{kind}_q_low"][s]
            p_over, r_over = ik.fk(q_over)
            rows["q_above"].append(q_over)
            rows["q_down"].append(q_low)
            rows["q_grasp"].append(ST[f"{kind}_q_grasp"][s])
            rows["P_above"].append(ST[f"{kind}_P_over"][s])
            rows["P_down"].append(ST[f"{kind}_P_low"][s])
            rows["p_above"].append(p_over)
            rows["rot"].append(r_over)
            rows["rot_down"].append(ik.fk(ST[f"{kind}_q_grasp"][s])[1])
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


class LoadEnv(pe.PlaceEnv):
    cfg: LoadEnvCfg
    TASK = "load"

    def __init__(self, cfg: LoadEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)  # ponytail: also plans the 6 dice cells (unused, ~1 s)
        self.plan = slot_plan(bc.ArmIK(), self.device)

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
        rot, rot_dn = self.plan["rot"][cell], self.plan["rot_down"][cell]
        aim = torch.zeros(m, 3, device=dev)
        aim[:, :2] = cfg.cell_noise_m * u(2)
        along = torch.cross(self.close_axis_ee.expand(m, 3), torch.tensor([0.0, 0.0, 1.0], device=dev).expand(m, 3), dim=1)
        d_true = self.d_nom + along * cfg.hand_offset_m[0] * u(1)
        d_true[:, 2] += cfg.hand_offset_m[1] * u()
        hand_err = torch.zeros(m, 3, device=dev)
        hand_err[:, :2] = cfg.hand_noise_m * u(2)
        e = d_true + hand_err - self.d_nom  # measured puck offset in the hand (slot_table: q = q_nom + P @ (-R e))
        off = -(rot @ e[..., None])[..., 0] + aim
        off_dn = -(rot_dn @ e[..., None])[..., 0] + aim
        q_above = self.plan["q_above"][cell] + (self.plan["P_above"][cell] @ off[..., None])[..., 0]
        q_down = self.plan["q_down"][cell] + (self.plan["P_down"][cell] @ off_dn[..., None])[..., 0]
        dist = (q_down - q_above).abs().max(dim=1).values.cpu().numpy()
        t_move = torch.tensor([bc.trapezoid_duration(float(x), cfg.vel_scale) for x in dist], device=dev)
        f = lambda v: torch.full_like(t_move, v)
        self.t_ph[env_ids] = torch.cumsum(torch.stack([f(cfg.hold_s), t_move, f(cfg.settle_s), f(cfg.release_s), t_move, f(cfg.wait_s)], 1), 1)
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

        jp = self.robot.data.default_joint_pos.torch[env_ids].clone()
        jp[:, self.arm_ids] = q_above
        for gi in self.grip_ids:
            jp[:, gi] = bc.GRIP_STOP
        self.robot.write_joint_state_to_sim_index(position=jp, velocity=torch.zeros_like(jp), env_ids=env_ids)
        self.hist[:, env_ids] = torch.cat([q_above, torch.zeros(m, 1, device=dev)], dim=1)
        self.robot.set_joint_position_target_index(target=q_above, joint_ids=self.arm_ids, env_ids=env_ids)

        p_ee = self.plan["p_above"][cell] + off
        bottom_b = p_ee + (rot @ d_true[..., None])[..., 0]
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
        quat = math_utils.quat_from_matrix(torch.stack([x_p, y_p, z_p], dim=2))
        self.puck.write_root_pose_to_sim_index(root_pose=torch.cat([pos_w, quat], dim=1), env_ids=env_ids)
        self.puck.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)
        self.puck_after_up[env_ids] = pos_w


# -------------------------------------------------------------------------------------------------------- unload
@configclass
class UnloadEnvCfg(pk.PickEnvCfg):
    nb_prob = 0.0  # one puck per slot
    slot_noise_m = 0.003  # puck position in the slot ring (perception of the ledger slot: none, the ring centres it)


class UnloadEnv(pk.PickEnv):
    cfg: UnloadEnvCfg
    TASK = "unload"

    def __init__(self, cfg: UnloadEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self.splan = slot_plan(bc.ArmIK(), self.device)
        self.slot_z = torch.zeros(self.num_envs, device=self.device)

    def _get_rewards(self) -> torch.Tensor:
        """PickEnv._get_rewards with 'lifted' measured over the slot floor (no neighbour)."""
        ph = self._phase()
        e = (self._mid_b() - self._ee_b()[0])[:, :2].norm(dim=1)
        w = torch.tensor([0.2, 0.5, 1.0, 0.0, 0.0, 0.0, 0.0], device=self.device)[ph]
        tilt = self._puck_tilt()
        r = self.cfg.w_track * w * torch.exp(-(e / 0.004) ** 2) - self.cfg.w_tilt * (ph < 4) * tilt
        r = r - self.cfg.w_rate * self.act_rate - self.cfg.w_act * self.act_mag
        self.final_err = torch.where(ph < 3, e, self.final_err)
        done = ph >= 6
        lifted = self._puck_bottom_b()[:, 2] - self.slot_z > 0.03
        ok = lifted & self.in_hand
        r = r + done * (self.cfg.w_success * ok + self.cfg.w_final * torch.exp(-(self.final_err / 0.004) ** 2))
        self.success = torch.where(done, ok, self.success)
        return r - self.cfg.w_fail * (self.knocked | self.tipped)

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
            self.stats["dropped"] += self.knocked[env_ids].tolist()
            self.stats["tipped"] += self.tipped[env_ids].tolist()
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
        self.puck.write_root_pose_to_sim_index(root_pose=torch.cat([pos_w, math_utils.quat_from_euler_xyz(zero, zero, yaw)], 1), env_ids=env_ids)
        self.puck.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)
        park = org + torch.tensor([0.6, 0.3, 0.001], device=dev)
        self.nb.write_root_pose_to_sim_index(root_pose=torch.cat([park, math_utils.quat_from_euler_xyz(zero, zero, zero)], 1), env_ids=env_ids)
        self.nb.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)
        self.nb0[env_ids], self.nb_on[env_ids] = park, False
        self.slot_z[env_ids] = slot_b[:, 2]

        # plan: carry -> q_over -> q_grasp, aimed with the slot error unknown (ring centres the puck)
        aim = torch.zeros(m, 3, device=dev)
        aim[:, :2] = cfg.slot_noise_m * u(2)  # where the base plan thinks the puck is - nominal
        q_above = self.splan["q_above"][cell] + (self.splan["P_above"][cell] @ aim[..., None])[..., 0]
        q_down = self.splan["q_grasp"][cell] + (self.splan["P_down"][cell] @ aim[..., None])[..., 0]
        rd = self.splan["rot_down"][cell]
        self.tool_yaw[env_ids] = torch.atan2(rd[:, 1, 0], rd[:, 0, 0])
        self.aim_err[env_ids] = aim - torch.cat([slot_b[:, :2] - self.splan["cell_b"][cell][:, :2], zero[:, None]], 1)
        q0 = torch.tensor(np.array([ST["box_carry"][s] for s in slot.tolist()]), device=dev, dtype=torch.float32)
        mv = lambda a, b: torch.tensor([bc.trapezoid_duration(float(x), cfg.vel_scale) for x in (b - a).abs().max(dim=1).values.cpu().numpy()], device=dev)
        t_tr, t_move = mv(q0, q_above), mv(q_above, q_down)
        f = lambda v: torch.full_like(t_move, v)
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

        jp = self.robot.data.default_joint_pos.torch[env_ids].clone()
        jp[:, self.arm_ids] = q0
        for gi in self.grip_ids:
            jp[:, gi] = self.grip_before
        self.robot.write_joint_state_to_sim_index(position=jp, velocity=torch.zeros_like(jp), env_ids=env_ids)
        self.hist[:, env_ids] = torch.cat([q0, torch.full((m, 1), self.grip_before, device=dev)], dim=1)
        self.robot.set_joint_position_target_index(target=q0, joint_ids=self.arm_ids, env_ids=env_ids)


for _id, _cls, _cfg in (("Bee-Load-Residual", "LoadEnv", "LoadEnvCfg"), ("Bee-Unload-Residual", "UnloadEnv", "UnloadEnvCfg")):
    gym.register(id=_id, entry_point=f"{__name__}:{_cls}", disable_env_checker=True,
                 kwargs={"env_cfg_entry_point": f"{__name__}:{_cfg}"})
