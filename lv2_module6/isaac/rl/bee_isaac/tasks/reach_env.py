"""Free arm moves (skills.yaml move_pose / pregrasp / camera_look_down / camera_clear_view): residual RL on the end pose.

Episode = arm at a start pose (stow / carry, jittered) -> joint-space move (same smooth step as the other envs) to the
goal -> hold (sag integral as rl_arm_correction) -> judged on the end effector error once the hold is over.
The policy adds a joint residual (|delta| <= 0.05 rad) from the end of the move until the judge: the sag pulls the hand
down a few mm at reach (report 4), which the integral alone removes slowly.
cfg.target:
  pose  : named poses (stow, carry left / right)                      skill move_pose
  clear : stow (pole camera view free; ponytail: until the scan pose is fixed in plan.yaml)  skill camera_clear_view
  above : tool down 50 mm over a puck top on the start box, yaw bins   skill pregrasp
  look  : tool down inspect_height_m (0.08; plan.yaml 0.15 is out of reach) over a puck top            skill camera_look_down
Reuses place_env.PlaceEnv (latency ring buffer, sag integral, phase clock); the puck is parked away, fingers open.
"""

from __future__ import annotations

import math
import os

import gymnasium as gym
import numpy as np
import pinocchio as pin
import torch

import isaaclab.utils.math as math_utils
from isaaclab.envs import DirectRLEnv
from isaaclab.utils import configclass

from bee_isaac import common as bc
from bee_isaac.tasks import pick_env as pk
from bee_isaac.tasks import place_env as pe

STOW = pk.STOW.numpy()
CARRY_L = np.array([math.pi / 2, -0.465027, 0.102271, 1.933412, math.pi / 2])
CARRY_R = np.array([-math.pi / 2, -0.465027, 0.102271, 1.933412, -math.pi / 2])
STARTS = [STOW, CARRY_L, CARRY_R]
OBS_DIM = 30


@configclass
class ReachEnvCfg(pe.PlaceEnvCfg):
    observation_space = OBS_DIM + 5
    episode_length_s = 8.0
    target = "above"
    # settle 0.6 -> 1.2 s (11-03): far-row goals (20 · 22 · 25) were on target (0.2 mm) but still moving at the judge
    hold_s, settle_s, wait_s = 0.3, 1.2, 0.4  # after the move: hold (residual off), settle (residual on), wait (judge)
    # above: (x, y, yaw) grid indices to leave out — near row · centre · +45 deg (goal 5) misses by 17.8 mm in 65 of 76
    # baseline tries (11-03, 2048 episodes): the hand cannot get there, nothing a residual can fix
    skip_above = ((0, 1, 2),)
    start_jitter_rad = 0.05
    box_span_m = 0.06  # above / look: puck xy on the start box, +- this around the box centre
    # plan.yaml coordinator inspect_height_m is 0.15, but tool-down 0.15 m over a puck on a 70 mm box is out of reach
    # (pinocchio IK 2026-11-03: <= 0.10 m near the box centre, 0.08 m over the whole box). Train at 0.08 until decided.
    inspect_height_m = 0.08
    look_tilt_deg = 25.0  # look: camera tilt allowed off straight down
    above_m = 0.05
    tol_m = 0.005  # success: end effector within this of the goal at the end, joints still
    w_track, w_rate, w_act, w_success, w_final, w_fail = 0.1, 0.05, 0.002, 10.0, 5.0, 5.0


class ReachEnv(pe.PlaceEnv):
    cfg: ReachEnvCfg
    TASK = "reach"

    def __init__(self, cfg: ReachEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, dev = self.num_envs, self.device
        ik = bc.ArmIK()
        base = ik.base_in_root()
        goals = []
        if cfg.target in ("pose", "clear"):
            goals = [STOW] if cfg.target == "clear" else [STOW, CARRY_L, CARRY_R]
        else:
            h = cfg.above_m if cfg.target == "above" else cfg.inspect_height_m
            # look: tool straight down is out of reach 0.15 m over the box; the camera may tilt (orientation gives way)
            w_ang, max_ang = (1.0, math.radians(2.0)) if cfg.target == "above" else (0.3, math.radians(cfg.look_tilt_deg))
            s = cfg.box_span_m
            for xi, x in enumerate((pe.BOX_CX - s, pe.BOX_CX, pe.BOX_CX + s)):
                for yi, y in enumerate((-s, 0.0, s)):
                    pb = np.array([x, y, bc.BOX_H + bc.PUCK_H + h]) - base  # tool tip over the puck top
                    hd = math.atan2(pb[1], pb[0])
                    for di, dy in enumerate(np.radians((-45, 0, 45))):
                        if cfg.target == "above" and (xi, yi, di) in cfg.skip_above:
                            continue
                        rot = pin.utils.rotate("z", hd + math.pi / 2 + dy) @ bc.DOWN
                        q, e_p, e_a = ik.solve(pb, rot, [np.array([hd, -0.2, 0.7, 1.0, sd]) for sd in (0.0, dy, -dy, math.pi / 2)],
                                               w_ang=w_ang)
                        if e_p < 2e-3 and e_a < max_ang:
                            goals.append(q)
        assert goals, f"no reachable goal for target {cfg.target}"
        self.goal_q = torch.tensor(np.array(goals), device=dev, dtype=torch.float32)
        self.goal_p = torch.tensor(np.array([ik.fk(q)[0] for q in goals]), device=dev, dtype=torch.float32)
        self.starts = torch.tensor(np.array(STARTS), device=dev, dtype=torch.float32)
        self.goal = torch.zeros(n, dtype=torch.long, device=dev)
        self.grip_before = self.grip_after = bc.gap_joint(40.0)
        print(f"[reach] target {cfg.target}: {len(goals)} goal poses")
        if os.environ.get("BEE_DEBUG"):
            for k, (q, p) in enumerate(zip(goals, self.goal_p.tolist())):
                print(f"[goal] {k} q {np.round(q, 2).tolist()} p mm {[round(1000 * v) for v in p]}")

    def _err(self):
        return self.goal_p[self.goal] - self._ee_b()[0]

    def _get_observations(self) -> dict:
        ph = self._phase()
        q = self.robot.data.joint_pos.torch[:, self.arm_ids]
        qd = self.robot.data.joint_vel.torch[:, self.arm_ids]
        onehot = torch.nn.functional.one_hot(ph.clamp(max=5), 6).float()
        prog = (self.t / self.t_ph[:, 5].clamp(min=1e-3)).clamp(0, 1)
        e = self._err()
        obs = torch.cat([q - self.q_ref, self.q_down - self.q_ref, 0.1 * qd, self.prev_act, onehot, prog[:, None], 100.0 * e], 1)
        obs = self._corr_obs(obs)
        return {"policy": obs, "critic": torch.cat([obs, self._base_tilt()[:, None]], 1)}

    def _get_rewards(self) -> torch.Tensor:
        ph = self._phase()
        e = self._err().norm(dim=1)
        w = (self.t >= self.t_tr).float()  # tracking counts once the move is over
        r = self.cfg.w_track * w * torch.exp(-(e / 0.004) ** 2) - self.cfg.w_rate * self.act_rate - self.cfg.w_act * self.act_mag
        done = ph >= 6
        still = self.robot.data.joint_vel.torch[:, self.arm_ids].abs().max(dim=1).values < 0.05
        ok = (e < self.cfg.tol_m) & still
        r = r + done * (self.cfg.w_success * ok + self.cfg.w_final * torch.exp(-(e / 0.004) ** 2))
        self.success = torch.where(done, ok, self.success)
        self.final_err = torch.where(done, e, self.final_err)
        return r - self.cfg.w_fail * self.tipped

    def _get_dones(self):
        self.in_hand = torch.zeros_like(self.success)
        self.dropped = torch.zeros_like(self.success)
        self.tipped = self._base_tilt() > math.radians(10.0)
        return (self._phase() >= 6) | self.tipped, self.episode_length_buf >= self.max_episode_length

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
                if os.environ.get("BEE_DEBUG"):
                    for i in ids[~self.success[ids]].tolist():
                        print(f"[fail] goal {self.goal[i].item()} err {1000 * self.final_err[i].item():.1f} mm "
                              f"err xyz {[round(1000 * v) for v in self._err()[i].tolist()]}", flush=True)
            self.stats["dropped"] += [False] * len(env_ids)
            self.stats["tipped"] += self.tipped[env_ids].tolist()
        DirectRLEnv._reset_idx(self, env_ids)
        m, dev, cfg = len(env_ids), self.device, self.cfg
        org = self.scene.env_origins[env_ids]
        zero = torch.zeros(m, device=dev)

        root = self.robot.data.default_root_pose.torch[env_ids].clone()
        root[:, :3] += org
        self.robot.write_root_pose_to_sim_index(root_pose=root, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)
        park = org + torch.tensor([0.6, 0.3, 0.001], device=dev)
        self.puck.write_root_pose_to_sim_index(root_pose=torch.cat([park, math_utils.quat_from_euler_xyz(zero, zero, zero)], 1), env_ids=env_ids)
        self.puck.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)

        goal = torch.randint(0, len(self.goal_q), (m,), device=dev)
        q_goal = self.goal_q[goal]
        q0 = self.starts[torch.randint(0, len(self.starts), (m,), device=dev)] + cfg.start_jitter_rad * (torch.rand(m, 5, device=dev) * 2 - 1)
        same = (q0 - q_goal).abs().max(dim=1).values < 0.3  # start next to the goal: start from stow instead
        q0 = torch.where(same[:, None], self.starts[0].expand(m, 5), q0)
        t_tr = torch.tensor([bc.trapezoid_duration(float(x), cfg.vel_scale) for x in (q_goal - q0).abs().max(dim=1).values.cpu().numpy()], device=dev)
        f = lambda v: torch.full_like(t_tr, v)
        eps = f(0.02)
        self.t_ph[env_ids] = torch.cumsum(torch.stack([t_tr + cfg.hold_s, eps, f(cfg.settle_s), eps, eps, f(cfg.wait_s)], 1), 1)
        self.t[env_ids] = 0.0
        self.q_start[env_ids], self.t_tr[env_ids] = q0, t_tr
        self.goal[env_ids] = goal
        self.q_above[env_ids], self.q_down[env_ids], self.q_ref[env_ids] = q_goal, q_goal, q0
        self._sample_sag(env_ids)
        for buf in (self.delta, self.prev_act):
            buf[env_ids] = 0.0
        for buf in (self.success, self.dropped, self.tipped):
            buf[env_ids] = False
        self.final_err[env_ids] = 0.0

        jp = self.robot.data.default_joint_pos.torch[env_ids].clone()
        jp[:, self.arm_ids] = q0
        for gi in self.grip_ids:
            jp[:, gi] = self.grip_before
        self.robot.write_joint_state_to_sim_index(position=jp, velocity=torch.zeros_like(jp), env_ids=env_ids)
        self.hist[:, env_ids] = torch.cat([q0, torch.full((m, 1), self.grip_before, device=dev)], dim=1)
        self.robot.set_joint_position_target_index(target=q0, joint_ids=self.arm_ids, env_ids=env_ids)


gym.register(id="Bee-Reach-Residual", entry_point=f"{__name__}:ReachEnv", disable_env_checker=True,
             kwargs={"env_cfg_entry_point": f"{__name__}:ReachEnvCfg"})
