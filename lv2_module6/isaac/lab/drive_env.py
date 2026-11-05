"""Base driving skills (skills.yaml drive_straight / drive_turn / drive_arc / dock_box / park_home): residual RL on (v, w).

Episode = robot at rest at the env origin, arm at stow; a goal pose (x, y, yaw) in the start frame per cfg.task ->
baseline go-to-pose controller (turn to the goal, roll with heading correction, turn to the final yaw; the P control of
demo_cycle.py drive_to, vectorized) + policy residual (dv, dw) -> wheel speed targets through a latency buffer and a
per-wheel speed scale error (wheel radius / calibration). Success = position and yaw within tolerance and the base
still for hold_s. The policy sees the goal with SLAM-like pose noise; the critic sees the true goal.
  straight: goal ahead 0.2-1.5 m            turn: in place +-180 deg          arc: 0.3-1.2 m ahead, +-0.6 m aside, +-90 deg
  dock: a start box 0.6-1.2 m away, goal = the docked pose (box centre REACH ahead), touching the box fails
  park: goal 0.3-1.5 m away, 5 cm / 10 deg, still for 3 s (start zone)
"""

from __future__ import annotations

import math

import gymnasium as gym
import torch
from isaaclab_physx.physics import PhysxCfg

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

import bee_common as bc
import pick_env as pk
import place_env as pe

WHEEL_R, WHEEL_SEP = 0.033, 0.2878  # demo_cycle.py
REACH = 0.08 + bc.BOX_HALF  # box centre ahead of the root when docked (demo_cycle.py)
OBS_DIM = 13
TASKS = {  # goal sampler ranges, tolerance [m, deg], hold [s], time limit [s]
    "straight": dict(tol=(0.01, 1.0), hold=0.5, limit=20.0),
    "turn": dict(tol=(0.01, 0.7), hold=0.5, limit=10.0),
    "arc": dict(tol=(0.02, 2.0), hold=0.5, limit=25.0),
    "dock": dict(tol=(0.01, 1.5), hold=0.5, limit=25.0),
    "park": dict(tol=(0.05, 10.0), hold=3.0, limit=25.0),
}


@configclass
class DriveSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
    light = AssetBaseCfg(prim_path="/World/DomeLight", spawn=sim_utils.DomeLightCfg(intensity=1000.0, exposure=1.25))
    robot = pe.robot_cfg().replace(prim_path="{ENV_REGEX_NS}/beebot")
    box = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/box",
        spawn=sim_utils.UsdFileCfg(usd_path=str(bc.USD_DIR / "shelf" / "shelf.usda"), scale=(0.205 / 0.207, 0.205 / 0.207, 1.0),
                                   rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True)),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(3.0, 0.0, 0.0)),
    )


@configclass
class DriveEventCfg:
    wheel_material = EventTerm(func=mdp.randomize_rigid_body_material, mode="startup", params={
        "asset_cfg": SceneEntityCfg("robot", body_names="wheel_.*"), "static_friction_range": (0.6, 1.1),
        "dynamic_friction_range": (0.5, 1.0), "restitution_range": (0.0, 0.0), "num_buckets": 64, "make_consistent": True})


@configclass
class DriveEnvCfg(DirectRLEnvCfg):
    sim: SimulationCfg = SimulationCfg(dt=1 / 200, physics=PhysxCfg())
    decimation = 8  # policy 25 Hz

    def __post_init__(self):
        self.sim.render_interval = self.decimation

    episode_length_s = 25.0
    scene: DriveSceneCfg = DriveSceneCfg(num_envs=256, env_spacing=4.0)
    events: DriveEventCfg = DriveEventCfg()
    observation_space = OBS_DIM
    state_space = 0
    action_space = 2

    task = "straight"
    v_max, w_max = 0.12, 1.0  # baseline caps (demo drive_to); the residual may go to the robot limits below
    v_lim, w_lim = 0.26, 2.0
    res_v, res_w = 0.02, 0.1  # residual scale for action +-1 (0.05 / 0.3 pulled the dock base controller apart)
    wheel_scale = (0.97, 1.03)  # per wheel speed error
    latency_s = (0.02, 0.06)  # command latency range
    pose_noise_m, yaw_noise_deg = 0.005, 0.5  # policy's goal from a SLAM-like pose
    w_prog, w_yaw, w_rate, w_act, w_tilt = 2.0, 0.5, 0.02, 0.005, 1.0
    w_success, w_fail = 10.0, 5.0


class DriveEnv(DirectRLEnv):
    cfg: DriveEnvCfg

    def __init__(self, cfg: DriveEnvCfg, render_mode: str | None = None, **kwargs):
        cfg.episode_length_s = TASKS[cfg.task]["limit"]  # time limit per task
        super().__init__(cfg, render_mode, **kwargs)
        n, dev = self.num_envs, self.device
        self.robot: Articulation = self.scene["robot"]
        self.box = self.scene["box"]
        self.arm_ids = self.robot.find_joints("joint[1-5]", preserve_order=True)[0]
        self.grip_ids = self.robot.find_joints("gripper_.*_joint")[0]
        self.wheel_ids = self.robot.find_joints(["wheel_left_joint", "wheel_right_joint"], preserve_order=True)[0]
        self.task_spec = TASKS[cfg.task]
        z = lambda *s: torch.zeros(n, *s, device=dev)
        self.goal = z(3)  # x, y, yaw in the env frame (start pose = origin, yaw 0)
        self.box_c = z(2)
        self.scale = torch.ones(n, 2, device=dev)
        self.lag = torch.zeros(n, dtype=torch.long, device=dev)
        lmax = int(math.ceil(cfg.latency_s[1] / self.physics_dt)) + 2
        self.hist = torch.zeros(lmax, n, 2, device=dev)
        self.hist_i = 0
        self.act, self.prev_act, self.cmd = z(2), z(2), z(2)
        self.prev_d, self.prev_y = z(), z()
        self.held, self.success, self.final_err = z(), torch.zeros(n, dtype=torch.bool, device=dev), z()
        self.tipped = torch.zeros(n, dtype=torch.bool, device=dev)
        self.hit = torch.zeros(n, dtype=torch.bool, device=dev)
        self.noise = z(3)
        self.stats = {"success": [], "err": [], "dropped": [], "tipped": []}
        self.stow = pk.STOW.to(dev).expand(n, 5)
        self.grip_open = bc.gap_joint(40.0)

    # ------------------------------------------------------------------ helpers
    def _pose(self):
        p = self.robot.data.root_pos_w.torch[:, :2] - self.scene.env_origins[:, :2]
        yaw = math_utils.euler_xyz_from_quat(self.robot.data.root_quat_w.torch)[2]
        return p, yaw

    def _goal_rel(self, noisy=False):
        """Goal in the robot frame: (dx, dy, dyaw)."""
        p, yaw = self._pose()
        if noisy:
            p = p + self.noise[:, :2]
            yaw = yaw + self.noise[:, 2]
        d = self.goal[:, :2] - p
        c, s = torch.cos(yaw), torch.sin(yaw)
        return torch.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], math_utils.wrap_to_pi(self.goal[:, 2] - yaw)], 1)

    def _baseline(self, g):
        """demo_cycle.py drive_to as one vectorized step: turn to the goal, roll, then turn to the final yaw."""
        cfg = self.cfg
        dist = g[:, :2].norm(dim=1)
        head = torch.atan2(g[:, 1], g[:, 0])
        far = dist > 0.01
        turn_first = far & (head.abs() > math.radians(10.0))
        v = torch.where(far & ~turn_first, (1.5 * g[:, 0]).clamp(-cfg.v_max, cfg.v_max), torch.zeros_like(dist))
        w_roll = (4.0 * g[:, 1]).clamp(-0.5, 0.5)
        w = torch.where(turn_first, (3.0 * head).clamp(-cfg.w_max, cfg.w_max), torch.where(far, w_roll, (3.0 * g[:, 2]).clamp(-cfg.w_max, cfg.w_max)))
        return torch.stack([v, w], 1)

    def _tilt(self):
        g = self.robot.data.projected_gravity_b.torch
        return torch.acos((-g[:, 2]).clamp(-1, 1))

    # ------------------------------------------------------------------ stepping
    def _pre_physics_step(self, actions):
        a = actions.clamp(-1.0, 1.0)
        self.act_rate = ((a - self.prev_act) ** 2).sum(1)
        self.prev_act, self.act = self.act, a
        base = self._baseline(self._goal_rel(noisy=True))
        v = (base[:, 0] + self.cfg.res_v * a[:, 0]).clamp(-self.cfg.v_lim, self.cfg.v_lim)
        w = (base[:, 1] + self.cfg.res_w * a[:, 1]).clamp(-self.cfg.w_lim, self.cfg.w_lim)
        self.cmd = torch.stack([v, w], 1)

    def _apply_action(self):
        v, w = self.cmd[:, 0], self.cmd[:, 1]
        wheels = torch.stack([(v - w * WHEEL_SEP / 2) / WHEEL_R, (v + w * WHEEL_SEP / 2) / WHEEL_R], 1) * self.scale
        self.hist_i = (self.hist_i + 1) % self.hist.shape[0]
        self.hist[self.hist_i] = wheels
        delayed = self.hist[(self.hist_i - self.lag) % self.hist.shape[0], torch.arange(self.num_envs, device=self.device)]
        self.robot.set_joint_velocity_target_index(target=delayed, joint_ids=self.wheel_ids)
        self.robot.set_joint_position_target_index(target=self.stow, joint_ids=self.arm_ids)
        self.robot.set_joint_position_target_index(target=torch.full((self.num_envs, len(self.grip_ids)), self.grip_open, device=self.device),
                                                   joint_ids=self.grip_ids)

    # ------------------------------------------------------------------ MDP
    def _get_observations(self):
        g = self._goal_rel(noisy=True)
        _, yaw = self._pose()
        lv = math_utils.quat_apply_inverse(self.robot.data.root_quat_w.torch, self.robot.data.root_lin_vel_w.torch)
        wz = self.robot.data.root_ang_vel_w.torch[:, 2:3]
        frac = (self.episode_length_buf.float() / self.max_episode_length)[:, None]
        obs = torch.cat([g[:, :2], torch.cos(g[:, 2:3]), torch.sin(g[:, 2:3]), lv[:, :2], wz, self.cmd, self.act, frac, self._tilt()[:, None]], 1)
        gt = self._goal_rel()
        return {"policy": obs, "critic": torch.cat([obs, gt, self.scale, self.lag[:, None].float() * self.physics_dt], 1)}

    def _get_rewards(self):
        cfg = self.cfg
        g = self._goal_rel()
        d, ye = g[:, :2].norm(dim=1), g[:, 2].abs()
        r = cfg.w_prog * (self.prev_d - d) + cfg.w_yaw * (self.prev_y - ye)
        self.prev_d, self.prev_y = d, ye
        r = r - cfg.w_rate * self.act_rate - cfg.w_act * (self.act ** 2).sum(1) - cfg.w_tilt * self._tilt()
        lv = self.robot.data.root_lin_vel_w.torch[:, :2].norm(dim=1)
        wz = self.robot.data.root_ang_vel_w.torch[:, 2].abs()
        tol_p, tol_y = self.task_spec["tol"]
        inside = (d < tol_p) & (ye < math.radians(tol_y)) & (lv < 0.01) & (wz < 0.03)
        self.held = torch.where(inside, self.held + self.step_dt, torch.zeros_like(self.held))
        done = self.held >= self.task_spec["hold"]
        self.success = self.success | done
        self.final_err = d
        return r + cfg.w_success * done - cfg.w_fail * (self.tipped | self.hit)

    def _get_dones(self):
        self.tipped = self._tilt() > math.radians(10.0)
        if self.cfg.task == "dock":  # the root came closer to the box centre than the docked distance - 2 cm: touched
            p, _ = self._pose()
            self.hit = (p - self.box_c).norm(dim=1) < REACH - 0.02
        return self.success | self.tipped | self.hit, self.episode_length_buf >= self.max_episode_length

    def _reset_idx(self, env_ids):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        if self.common_step_counter > 0 and len(env_ids) > 0:
            log = self.extras.setdefault("log", {})
            log["Metrics/success"] = self.success[env_ids].float().mean().item()
            log["Metrics/err_mm"] = 1000 * self.final_err[env_ids].mean().item()
            self.stats["success"] += self.success[env_ids].tolist()
            self.stats["err"] += self.final_err[env_ids].tolist()
            self.stats["dropped"] += self.hit[env_ids].tolist()
            self.stats["tipped"] += self.tipped[env_ids].tolist()
        super()._reset_idx(env_ids)
        m, dev, cfg = len(env_ids), self.device, self.cfg
        u = lambda *s: torch.rand(m, *s, device=dev) * 2 - 1
        rng = lambda lo, hi: lo + (hi - lo) * torch.rand(m, device=dev)
        org = self.scene.env_origins[env_ids]
        zero = torch.zeros(m, device=dev)

        root = self.robot.data.default_root_pose.torch[env_ids].clone()
        root[:, :3] += org
        self.robot.write_root_pose_to_sim_index(root_pose=root, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)
        jp = self.robot.data.default_joint_pos.torch[env_ids].clone()
        jp[:, self.arm_ids] = self.stow[: m]
        self.robot.write_joint_state_to_sim_index(position=jp, velocity=torch.zeros_like(jp), env_ids=env_ids)

        t = cfg.task
        box = torch.stack([zero + 3.0, zero, zero], 1)  # parked far away
        if t == "straight":
            goal = torch.stack([rng(0.2, 1.5), zero, zero], 1)
        elif t == "turn":
            goal = torch.stack([zero, zero, math.pi * u()], 1)
        elif t == "arc":
            goal = torch.stack([rng(0.3, 1.2), 0.6 * u(), 0.5 * math.pi * u()], 1)
        elif t == "dock":
            r_, a_, by = rng(0.6, 1.2), math.radians(30.0) * u(), math.radians(20.0) * u()
            box = torch.stack([r_ * torch.cos(a_), r_ * torch.sin(a_), by], 1)
            goal = torch.stack([box[:, 0] - REACH * torch.cos(by), box[:, 1] - REACH * torch.sin(by), by], 1)
        else:  # park
            r_, a_ = rng(0.3, 1.5), math.pi * u()
            goal = torch.stack([r_ * torch.cos(a_), r_ * torch.sin(a_), math.pi * u()], 1)
        self.goal[env_ids], self.box_c[env_ids] = goal, box[:, :2]
        bpos = torch.cat([box[:, :2] + org[:, :2], zero[:, None]], 1)
        self.box.write_root_pose_to_sim_index(root_pose=torch.cat([bpos, math_utils.quat_from_euler_xyz(zero, zero, box[:, 2])], 1), env_ids=env_ids)
        self.box.write_root_velocity_to_sim_index(root_velocity=torch.zeros(m, 6, device=dev), env_ids=env_ids)

        lo, hi = cfg.wheel_scale
        self.scale[env_ids] = lo + (hi - lo) * torch.rand(m, 2, device=dev)
        self.lag[env_ids] = torch.round(rng(*cfg.latency_s) / self.physics_dt).long()
        self.hist[:, env_ids] = 0.0
        self.noise[env_ids] = torch.cat([cfg.pose_noise_m * u(2), math.radians(cfg.yaw_noise_deg) * u(1)], 1)
        g0 = goal.clone()
        self.prev_d[env_ids], self.prev_y[env_ids] = g0[:, :2].norm(dim=1), g0[:, 2].abs()
        for buf in (self.act, self.prev_act, self.cmd):
            buf[env_ids] = 0.0
        self.held[env_ids] = 0.0
        for buf in (self.success, self.tipped, self.hit):
            buf[env_ids] = False


gym.register(id="Bee-Drive-Residual", entry_point=f"{__name__}:DriveEnv", disable_env_checker=True,
             kwargs={"env_cfg_entry_point": f"{__name__}:DriveEnvCfg"})
