"""Bee-Pick-Residual: residual RL for the start box pick (segment 2 of the RL task split, skill pick_box).

Episode = the open hand is above the target puck (end of the transit segment) ->
    hold (sag settles) -> down to the grasp height -> settle -> close -> lift -> wait, then judge.
Two pucks stand on the start box at the box centre +-40 mm along y (cycle_sm.py --layout start_y); the target is one
of them, the other is a neighbour that must not be knocked (absent in 30 % of episodes = the second pick).
The base motion is the scripted one of demo_cycle.py pick(): offline IK aimed at the perceived puck (+-5 mm, +-10 deg),
fingers at right angles to the line through the 2 pucks (box puck: nearest face). The policy adds a joint residual
(|delta| <= 0.05 rad) until the fingers close and sees a gripper-camera style "puck - hand" measurement.

Same phase timeline, latency and sag integral as place_env.PlaceEnv, which this class reuses.

cfg.design = True switches on the perception-fault design of "BEE, pick 잔차 학습 설계" (inspect pose error, 5 perception
situations on a curriculum stage, asymmetric observations, end-of-episode alignment score, failure terminations).
With design = False (default) nothing below the "design" markers runs and the legacy behaviour is unchanged.
Bee-Pick-Residual registers PickDesignEnvCfg: design = True and the design timings (20 s, lifted judged at 45 mm of the 50 mm lift, wait 0.5 s).
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
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass

from bee_isaac import common as bc
from bee_isaac.tasks import place_env as pe

START_GAP = 0.080  # 2 pucks at the box centre +- 40 mm along y
YAW_BINS = np.radians(np.arange(-45, 46, 1))  # tool yaw offsets from the finger rule, planned offline
OBS_DIM, CRITIC_EXTRA = 36, 13
DESIGN_OBS, DESIGN_PRIV = 26, 15  # extra actor / critic-only observations of the design
STOW = torch.tensor([0.0, -1.9024, 1.344, 0.9023, 0.0])  # demo_cycle.py reset_scene
# curriculum stage -> perception situations switched on (1 far bias, 2 near fault, 3 video delay, 5 dropout; 4 = output DR at stage 5)
STAGE_SITUATIONS = {0: (), 1: (1,), 2: (1, 3), 3: (1, 3, 5), 4: (1, 3, 5, 2), 5: (1, 3, 5, 2)}


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
class PickDesignSceneCfg(PickSceneCfg):
    """PickSceneCfg + a contact sensor on the target puck filtered to the neighbour puck (lift-contact penalty)."""
    puck = pe.PlaceSceneCfg().puck.replace(spawn=pe.PlaceSceneCfg().puck.spawn.replace(activate_contact_sensors=True))
    puck_contact = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/puck", filter_prim_paths_expr=["{ENV_REGEX_NS}/nb"], history_length=4)


@configclass
class PickEnvCfg(pe.PlaceEnvCfg):
    scene: PickSceneCfg = PickSceneCfg(num_envs=256, env_spacing=1.0)
    observation_space = OBS_DIM + 5
    episode_length_s = 8.0

    hold_s, settle_s, release_s, wait_s = 0.3, 0.3, 0.5, 0.3  # release_s = close time here (demo CLOSE_S)
    above_m, open_gap_mm = 0.05, 60.0  # real pick_open_gap_m 0.060 (40 mm missed the puck at 10-13 mm perception error, 11-08)
    pos_noise_m, yaw_noise = 0.005, math.radians(10.0)  # perception (demo POS_NOISE, YAW_NOISE)
    park_m = 0.010  # base stopping error: both pucks shifted together
    transit = True  # start at stow and move to the above pose inside the episode (real transit end state)
    start_jitter_rad = 0.01  # transit = False: jitter around the above pose instead
    nb_prob = 0.7
    cam_bias_m, cam_noise_m = 0.002, 0.001
    cam_yaw_noise = math.radians(2.0)
    lift_m = 0.03  # puck bottom above the box top that counts as lifted (design note: 0.05)

    # reward
    w_track, w_tilt, w_nb, w_rate, w_act = 0.1, 0.5, 50.0, 0.05, 0.002
    w_success, w_final, w_fail = 10.0, 5.0, 5.0

    # ---- design (BEE pick design note); everything below is inert unless design = True
    design = False
    stage = 0  # curriculum 0..5, see STAGE_SITUATIONS (1: + retarget, inspect error; 5: + output DR and common DR)
    situations: tuple | None = None  # explicit situation list instead of the stage's
    skip_situations: tuple = ()  # situations dropped from the draw (Delta = 0 baseline already > 97 %)
    force_situation: int | None = None  # evaluation: every episode in this situation (0 = normal, 1..5)
    force_insp: bool | None = None  # evaluation: True / False = inspect pose always wrong / always right
    retarget: bool | None = None  # re-aim the base plan at the latest measurement (None: stage >= 1)
    common_dr: bool | None = None  # sag / integral gain / base stop error / start joint velocity / output DR (None: stage >= 5)

    # inspect pose error, measured 11-07 (old inspect_test.py --modes skillbox, removed, see git history; 30 seeds / 60 pucks, YOLO perception, final reading vs truth):
    # 32 % of the pucks are off by 5-20 mm (xy, rms 11.9 mm, mostly -y, both pucks of a box share the offset), the rest within 3 mm.
    # The measurement is the sum of tilt / height / position error, so it all goes into xy: uniform +-a per axis has rms hypot 0.82 a.
    insp_err_prob = 5.0 / 16.0  # right : wrong = 11 : 5 (measured 41 : 19)
    insp_xy_m, insp_tilt_rad, insp_z_m = 0.010, 0.0, 0.0  # capped at 10 mm (measured rms 11.9 mm ~ +-15; the policy corrects up to ~8-9 mm per axis)
    insp_lever_m, insp_z_gain = 0.15, 0.5  # camera-to-puck lever for the tilt, apparent xy shift per metre of height error
    start_vel_rad_s = 0.05  # residual joint velocity kicked in once at the arrival above the puck (end of transit), +- per joint (common DR)
    common_park_m = 0.015  # base stopping error with common DR (note: +-10..20 mm)

    # perception errors (situation 1, 2, 3, 5)
    cam_period_s = 0.25  # measurement interval: wrist YOLO detections (perception_node runs it every 0.2 s per camera, Jetson <= 4 Hz)
    # wrist D435 vs the 16 mm top tag (perception_node: depth beyond Min-Z, tag solvePnP closer, no update without the tag)
    cam_minz_m = 0.14  # depth Min-Z at 640x360 (scales with the x resolution: about 0.28 m at 1280x720)
    cam_tan_h, cam_tan_v = math.tan(math.radians(34.5)), math.tan(math.radians(21.0))  # colour HFOV 69 deg, VFOV 42 deg (16:9)
    tag_half_m, tag_view_max_deg = 0.008, 60.0  # whole tag in the image; seen at most this far off its normal
    pnp_noise_m, tag_bias_m = 0.0003, 0.001  # solvePnP noise at cam_d0_m (grows with the distance); tag stuck off the top centre +-
    cam_d0_m, cam_ratio_max = 0.08, 3.0  # camera-puck distance at the above pose; error ratio d / d0 is capped
    s1_bias_m, s1_insp_ratio = 0.008, 1.0  # far bias b (xy, +-); share of b present in the inspect-time reading
    d_min_m, s2_bias_m, s2_jitter_m = 0.04, (0.005, 0.010), 0.003  # situation 2: below d_min bias magnitude range, jitter
    vid_delay_range = (0.03, 0.10)  # situation 3: extra video delay [s] on top of det_delay_range
    det_delay_range = (0.05, 0.15)  # every episode: capture -> detection delay of the wrist YOLO [s] (placeholder until a rosbag measures it)
    drop_len_range, drop_gap_range, drop_t_max, drop_perm_prob = (0.1, 1.0), (0.3, 1.5), 2.5, 0.5  # situation 5
    meas_age_max, age_rule = 0.45, True  # interval 0.25 + delay 0.15 + 0.05; = hand_relative max_age_s = rl_arm_correction max_cam_age_s
    # stale measurement (capture time basis) -> Delta is held or zeroed (stale_zero)
    # The URDF wrist camera loses the top tag about 30 mm above the grasp (common.py __main__), so every pick ends on a stale
    # measurement: zeroing Delta there would undo the correction while the fingers close. 1 = the old Delta = 0 rule.
    stale_zero = 0
    # Real robot (rl_arm_correction): the policy is only run with a fresh measurement (age <= meas_age_max); then its action
    # enters the action history, and Delta changes only in phases 0..2 (skills arm_io contact_step: residual_live false from the close).
    retarget_alpha, retarget_stop_d_m = 0.1, 0.04  # low-pass gain per policy step; no retarget below this distance (= d_min_m: a situation 2
    # fault must not drag the plan in the last approach; 0: off)

    # output DR (situation 4)
    # grip force per finger: ROS grip_current_a 0.1 A x 73.5 N/A = 7.4 N (real robot 11-08, with grip_close_gap_m 0.026);
    # lower end covers the case where the 3 mm position error, not the current cap, sets the force (not measured yet)
    tau_lim_range, k_dr, d_dr, grip_force_range = (3.7, 4.1), 0.2, 0.3, (5.0, 7.4)
    col_tau_ratio, col_vel, col_s = 0.8, 0.1, 0.3  # collision: tau / limit > ratio while |qd| < col_vel for col_s seconds
    # 0 = no torque-collision failure: the real check (control.yaml effort_threshold 3.0 A) is above the XM430-W350 stall current
    # (2.1 A at 11.1 V), so the real robot never stops on it (11-07)
    collide = 0

    # design reward
    w_align, w_fail_d = 0.02, 10.0
    w_lift, w_ox, w_th = 5.0, 3.0, 2.0  # hold base score + alignment scores (max 5 + 3 + 2)
    ox_scale, th_scale = 0.0035, 0.1
    w_contact, contact_n = 0.06, 1.0
    w_tau = 0.0  # torque closeness penalty (not specified in the note)
    succ_ox, succ_theta_deg = 0.004, 5.0  # success criterion (pick -> load contract: 5 deg, as load / unload)
    fail_ox, fail_theta_deg = 0.008, 15.0  # lifted but too misaligned = failure
    empty_gap_mm = 24.0  # finger gap below this after the close (puck 30 mm) = closed on nothing -> failure (counted as lost)


@configclass
class PickDesignEnvCfg(PickEnvCfg):
    """Bee-Pick-Residual: design timings (20 s episode, lifted judged at 45 mm, wait 0.3 s), design logic on, stage 0, no re-aim."""
    scene: PickDesignSceneCfg = PickDesignSceneCfg(num_envs=256, env_spacing=1.0)
    design = True
    episode_length_s = 20.0
    wait_s = 0.3  # real contact_step waits 0.3 s after the lift (arm_io.cpp)
    retarget = False  # the real pick_skill solves q_down once (IK of the goal) and never re-aims it
    lift_m = 0.045  # the plan lifts to above_m = 50 mm; sag / slip leave a few mm short, so judge at 45 mm


class PickEnv(pe.PlaceEnv):
    cfg: PickEnvCfg
    TASK = "pick"

    def __init__(self, cfg: PickEnvCfg, render_mode: str | None = None, **kwargs):
        design = cfg.design
        stage = cfg.stage
        common = (stage >= 5) if cfg.common_dr is None else cfg.common_dr
        if design:  # spaces and latency-independent cfg ranges must be fixed before the base class builds the env
            cfg.observation_space = OBS_DIM + (5 if cfg.obs_corr else 0) + DESIGN_OBS
            if common:
                cfg.sag_scale_range = cfg.sag_scale_range or (0.7, 1.5)
                cfg.corr_gain_range = cfg.corr_gain_range or (0.5, 1.5)
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

        # ---- design state (allocated always, used only when design)
        self.design, self.common_dr = design, bool(design and common)
        self.retarget_on = design and ((stage >= 1) if cfg.retarget is None else cfg.retarget)
        self.insp_on = design and stage >= 1
        sits = cfg.situations if cfg.situations is not None else STAGE_SITUATIONS[min(stage, 5)]
        skip = tuple(int(s) for s in cfg.skip_situations)  # --cfg a:b gives floats
        self.situs = tuple(int(s) for s in sits if int(s) not in skip)
        self.out_dr = self.common_dr
        b = lambda: torch.zeros(n, dtype=torch.bool, device=dev)
        self.situ, self.ybin, self.dsteps = (torch.zeros(n, dtype=torch.long, device=dev) for _ in range(3))
        self.insp, self.in_drop, self.drop_perm, self.fresh = b(), b(), b(), b()
        self.lifted_once, self.hold_done, self.f_fail, self.f_lost, self.f_col, self.f_hold, self.kicked = (b() for _ in range(7))
        self.park, self.b1, self.b2 = z(2), z(2), z(2)
        self.plan_aim, self.tgt_insp_b = z(3), z(3)
        self.drop_next, self.drop_until = torch.full((n,), math.inf, device=dev), z()
        self.next_frame, self.last_cap, self.meas_age, self.col_t, self.shaped_sum = z(), z(), z(), z(), z()
        self.last_m, self.ox, self.th = z(4), z(), z()
        self.meas_valid = torch.ones(n, device=dev)
        self.cam_off, self.cam_z, self.cam_x, self.cam_y = (torch.tensor(v, device=dev).repeat(n, 1) for v in (bc.CAM_OFF_G, *bc.CAM_AXES_G))
        self.z_up, self.tag_bias = torch.tensor([0.0, 0.0, 1.0], device=dev).repeat(n, 1), z(3)
        self.a_old1, self.a_old2 = z(5), z(5)
        self.mh_n = int(math.ceil((cfg.det_delay_range[1] + cfg.vid_delay_range[1]) / self.step_dt)) + 2  # measurement ring buffer (policy steps)
        self.mh, self.mh_i = torch.zeros(self.mh_n, n, 6, device=dev), 0
        if design:  # nominal actuator values (the legacy path never reads them)
            lim = self.robot.data.joint_effort_limits.torch
            fin = lambda x: torch.where(torch.isfinite(x) & (x > 0), x, torch.full_like(x, 4.0))
            self.tau_lim = fin(lim[:, self.arm_ids]).clone()
            self.grip_lim = fin(lim[:, self.grip_ids]).clone()
            self.d_arm0 = self.robot.data.joint_damping.torch[:, self.arm_ids].clone()
        try:
            self.nb_sensor = self.scene["puck_contact"]
        except KeyError:
            self.nb_sensor = None  # scene without the sensor: the lift-contact term is skipped
        self.stats.update({"ox": [], "theta": [], "lost": [], "collided": [], "situ": [], "insp": []})

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

    def _nb_tilt(self):
        up = torch.zeros(self.num_envs, 3, device=self.device)
        up[:, 2] = 1.0
        return torch.acos(math_utils.quat_apply(self.nb.data.root_quat_w.torch, up)[:, 2].clamp(-1, 1))

    def _nb_state(self):
        """Neighbour moved > 3 mm or tilted > 5 deg (only when it is on the box)."""
        d = (self.nb.data.root_pos_w.torch - self.nb0)[:, :2].norm(dim=1)
        tilt = self._nb_tilt()
        return d * self.nb_on, self.nb_on & ((d > 0.003) | (tilt > math.radians(5.0)))

    # ---- design helpers
    def _hand_frame(self):
        """End effector position and axes in the base frame: x along the finger pad, y = closing axis, z = approach."""
        n = self.num_envs
        pos, q = self._ee_b()
        y = math_utils.quat_apply(q, self.close_axis_ee.expand(n, 3))
        z = math_utils.quat_apply(q, torch.tensor([0.0, 0.0, 1.0], device=self.device).expand(n, 3))
        x = torch.cross(y, z, dim=1)
        return pos, x / x.norm(dim=1, keepdim=True).clamp(min=1e-6), y, z

    def _hand_metrics(self):
        """True puck centre offset along the closing axis (e_y) and the finger axis (e_x) from the gripper centre line,
        and the angle theta between the puck axis and the approach axis."""
        pos, x, y, z = self._hand_frame()
        rel = self._mid_b() - pos
        _, bq = self._base()
        up = torch.zeros(self.num_envs, 3, device=self.device)
        up[:, 2] = 1.0
        ax = math_utils.quat_apply(math_utils.quat_mul(math_utils.quat_inv(bq), self.puck.data.root_quat_w.torch), up)
        th = torch.acos((ax * z).sum(1).abs().clamp(max=1.0))
        return (rel * y).sum(1), (rel * x).sum(1), th

    def _tau_ratio(self):
        return self.robot.data.applied_torque.torch[:, self.arm_ids].abs() / self.tau_lim

    def _nb_force(self):
        """Largest target-puck / neighbour contact force over the last physics steps [N]."""
        if self.nb_sensor is None or self.nb_sensor.data.normal_force_matrix_w_history is None:
            return torch.zeros(self.num_envs, device=self.device)
        f = self.nb_sensor.data.normal_force_matrix_w_history.torch
        return f.reshape(self.num_envs, -1, 3).norm(dim=-1).amax(dim=1)

    def _sample_sag(self, env_ids):
        if not self.out_dr:
            return super()._sample_sag(env_ids)
        cfg, m, dev = self.cfg, len(env_ids), self.device
        u = lambda lo, hi, *s: lo + (hi - lo) * torch.rand(*s, device=dev)
        ac = self.arm_corr
        k0 = ac.k_arm0[env_ids].clone()
        ks = u(1 - cfg.k_dr, 1 + cfg.k_dr, m, 5)
        ac.k_arm0[env_ids] = k0 * ks  # ArmCorrection.reset writes k_arm0 / sag scale; the feed-forward keeps the nominal k0
        super()._sample_sag(env_ids)
        ac.k_arm0[env_ids] = k0
        if cfg.sag_scale_range is None:
            self.robot.write_joint_stiffness_to_sim_index(stiffness=k0 * ks, joint_ids=self.arm_ids, env_ids=env_ids)
        self.robot.write_joint_damping_to_sim_index(damping=self.d_arm0[env_ids] * u(1 - cfg.d_dr, 1 + cfg.d_dr, m, 5),
                                                    joint_ids=self.arm_ids, env_ids=env_ids)
        lim = u(*cfg.tau_lim_range, m, 1).expand(m, 5).contiguous()
        self.tau_lim[env_ids] = lim
        self.robot.write_joint_effort_limit_to_sim_index(limits=lim, joint_ids=self.arm_ids, env_ids=env_ids)
        force = u(*cfg.grip_force_range, m, 1)
        self.robot.write_joint_stiffness_to_sim_index(stiffness=(force / bc.GRIP_STOP).expand(m, len(self.grip_ids)).contiguous(),
                                                      joint_ids=self.grip_ids, env_ids=env_ids)

    def _legacy_cam(self, rel, face, is_box):
        """Non-design camera observation: puck - hand with a fixed bias and noise, face error. (UnloadEnv overrides it.)"""
        cam = rel + self.cam_bias + self.cfg.cam_noise_m * torch.randn_like(rel)
        return cam, face + self.cam_yaw_bias + self.cfg.cam_yaw_noise * torch.randn_like(face), is_box

    def _cam_view(self):
        """Wrist camera -> target puck top tag: distance [m] and whether the tag can be read (bc.tag_view)."""
        cfg = self.cfg
        g_pos, g_quat = self.robot.data.body_pos_w.torch[:, self.ee_id], self.robot.data.body_quat_w.torch[:, self.ee_id]
        rot = lambda v: math_utils.quat_apply(g_quat, v)
        n_t = math_utils.quat_apply(self.puck.data.root_quat_w.torch, self.z_up)
        v = self.puck.data.root_pos_w.torch + bc.PUCK_H * n_t - (g_pos + rot(self.cam_off))
        return bc.tag_view(v, rot(self.cam_z), rot(self.cam_x), rot(self.cam_y), n_t, cfg.cam_tan_h, cfg.cam_tan_v, cfg.tag_half_m,
                           math.cos(math.radians(cfg.tag_view_max_deg)))

    def _perceive(self):
        """One policy step of the camera stream (frames, dropout, delay). Returns the measurement the policy sees:
        (x, y, z of the puck centre in the base frame, face yaw), its age [s] and a validity flag."""
        cfg, n, dev, t = self.cfg, self.num_envs, self.device, self.t
        pm = self._mid_b()
        d = (pm - self._ee_b()[0]).norm(dim=1)
        ratio = (d / cfg.cam_d0_m).clamp(max=cfg.cam_ratio_max)[:, None]
        noise = cfg.cam_noise_m * torch.randn(n, 3, device=dev)
        pad = lambda v: torch.cat([v, torch.zeros_like(v[:, :1])], 1)
        s = self.situ[:, None]
        near = (d < cfg.d_min_m)[:, None]
        e1 = (pad(self.b1) + noise) * ratio  # far bias shrinks with the distance
        e2 = noise + near * (pad(self.b2) + cfg.s2_jitter_m * torch.randn(n, 3, device=dev))  # true until d < d_min
        err = torch.where(s == 1, e1, torch.where(s == 2, e2, self.cam_bias + noise))
        # closer than the depth Min-Z the reading is the tag solvePnP: calibration bias + tag placement + small noise (the far
        # bias of situation 1 is gone, the switch is a jump); situation 2 keeps its wrong near readings
        dc, vis = self._cam_view()
        pnp = (dc < cfg.cam_minz_m)[:, None] & ~((s == 2) & near)
        e_pnp = self.cam_bias + self.tag_bias + cfg.pnp_noise_m * (dc / cfg.cam_d0_m)[:, None] * torch.randn(n, 3, device=dev)
        err = torch.where(pnp, e_pnp, err)
        yaw = self._puck_yaw_b() + self.cam_yaw_bias + cfg.cam_yaw_noise * torch.randn(n, device=dev)  # face error: _get_observations
        meas = torch.cat([pm + err, yaw[:, None]], 1)
        # situation 5: dropout windows (repeated, or permanent from some time on)
        begin = ~self.in_drop & (t >= self.drop_next)
        dur = cfg.drop_len_range[0] + (cfg.drop_len_range[1] - cfg.drop_len_range[0]) * torch.rand(n, device=dev)
        self.drop_until = torch.where(begin, torch.where(self.drop_perm, torch.full_like(t, math.inf), t + dur), self.drop_until)
        self.in_drop = self.in_drop | begin
        end = self.in_drop & (t >= self.drop_until)
        self.in_drop = self.in_drop & ~end
        gap = cfg.drop_gap_range[0] + (cfg.drop_gap_range[1] - cfg.drop_gap_range[0]) * torch.rand(n, device=dev)
        self.drop_next = torch.where(end, t + gap, self.drop_next)
        # frames arrive at the camera rate; while dropped the last value is held (never zero-filled)
        cap = t >= self.next_frame
        new = cap & ~self.in_drop & vis  # tag out of view: no update, the last value is held
        self.last_m = torch.where(new[:, None], meas, self.last_m)
        self.last_cap = torch.where(new, t, self.last_cap)
        self.next_frame = torch.where(cap, self.next_frame + cfg.cam_period_s, self.next_frame)
        entry = torch.cat([self.last_m, self.last_cap[:, None], (~self.in_drop & vis).float()[:, None]], 1)
        self.mh_i = (self.mh_i + 1) % self.mh_n
        self.mh[self.mh_i] = entry
        if self.fresh.any():  # new episode: no stale history behind the delay
            self.mh[:, self.fresh] = entry[self.fresh][None]
            self.fresh[:] = False
        e = self.mh[(self.mh_i - self.dsteps) % self.mh_n, torch.arange(n, device=dev)]
        self.meas_age, self.meas_valid = t - e[:, 4], e[:, 5]
        return e[:, :4], self.meas_age, self.meas_valid

    def _retarget(self, m, ok):
        """Re-aim the base plan (q_above, q_down) at the latest measurement, low-passed, in the envs flagged ok."""
        cfg, s, b = self.cfg, self.side, self.ybin
        new = m[:, :2] - self.p_nom[s][:, :2]
        self.plan_aim[:, :2] = torch.where(ok[:, None], self.plan_aim[:, :2] + cfg.retarget_alpha * (new - self.plan_aim[:, :2]),
                                           self.plan_aim[:, :2])
        a = self.plan_aim[..., None]
        self.q_above = torch.where(ok[:, None], self.pplan["q_above"][s, b] + (self.pplan["P_above"][s, b] @ a)[..., 0], self.q_above)
        self.q_down = torch.where(ok[:, None], self.pplan["q_down"][s, b] + (self.pplan["P_down"][s, b] @ a)[..., 0], self.q_down)
        self.aim_err[:, :2] = self.plan_aim[:, :2] - self.park

    # ------------------------------------------------------------------ MDP
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        if not self.design:
            return super()._pre_physics_step(actions)
        if self.common_dr:  # leftover shaking right after the arrival above the puck: one velocity kick at the end of the transit
            kick = (self.t >= self.t_tr) & ~self.kicked
            if kick.any():
                ids = kick.nonzero()[:, 0]
                jp = self.robot.data.joint_pos.torch[ids].clone()
                jv = self.robot.data.joint_vel.torch[ids].clone()
                jv[:, self.arm_ids] += self.cfg.start_vel_rad_s * (torch.rand(len(ids), 5, device=self.device) * 2 - 1)
                self.robot.write_joint_state_to_sim_index(position=jp, velocity=jv, env_ids=ids)
                self.kicked |= kick
        a = actions.clamp(-1.0, 1.0)
        ph = self._phase()
        run = (ph < 6) & (self.t >= self.t_tr) & ((self.meas_age <= self.cfg.meas_age_max) | (not self.cfg.age_rule))  # policy runs
        live = (ph < 3) & (self.t >= self.t_tr)  # Delta may change (fixed from the close on)
        on = run & live
        stale = torch.zeros_like(self.delta) if self.cfg.stale_zero else self.delta
        self.delta = torch.where(on[:, None], pe.step_delta(self.delta, self.cfg.residual_rad * a, self.cfg.delta_step_rad),
                                 torch.where(live[:, None], stale, self.delta))
        self.act_rate = ((a - self.prev_act) ** 2).sum(dim=1) * on
        self.act_mag = (a ** 2).sum(dim=1) * on
        r = run[:, None]  # the action history only moves when the policy ran (transit and stale steps leave it as it was)
        self.a_old2, self.a_old1, self.prev_act = (torch.where(r, self.a_old1, self.a_old2), torch.where(r, self.prev_act, self.a_old1),
                                                   torch.where(r, a, self.prev_act))

    def _get_observations(self) -> dict:
        ph = self._phase()
        q = self.arm_corr.q_seen()
        qd = self.robot.data.joint_vel.torch[:, self.arm_ids]
        onehot = torch.nn.functional.one_hot(ph.clamp(max=5), 6).float()
        # phase 0 starts at the arrival above the puck in the design (real: contact_step starts there, the transit is MoveIt)
        t0 = self.t_tr[:, None] if self.design else torch.zeros_like(self.t[:, None])
        prog = ((self.t - torch.cat([t0, self.t_ph[:, :-1]], 1).gather(1, ph.clamp(max=5)[:, None])[:, 0])
                / 1.0).clamp(0, 2)
        rel = self._mid_b() - self._ee_b()[0]  # puck - hand
        face = self._face_err()
        is_box = self._kinds().float()
        if self.design:
            m, age, valid = self._perceive()
            ee = self._ee_b()[0]
            rel_m = m[:, :3] - ee  # latest measurement, hand relative
            if self.retarget_on:
                ok = (valid > 0.5) & (age <= self.cfg.meas_age_max) & (ph < 3) & (self.t >= self.t_tr) & (rel_m.norm(dim=1) > self.cfg.retarget_stop_d_m)
                self._retarget(m, ok)
            q_ee = self._ee_b()[1]
            x_ee = math_utils.quat_apply(q_ee, torch.tensor([1.0, 0.0, 0.0], device=self.device).expand(self.num_envs, 3))
            tool_now = torch.atan2(x_ee[:, 1], x_ee[:, 0])  # hand_geom.tool_yaw of the hand now
            fe = torch.remainder(m[:, 3] - tool_now + math.pi / 4, math.pi / 2) - math.pi / 4
            cam, cam_yaw = rel_m, torch.where(self._kinds(), fe, torch.zeros_like(fe))
            valid = (age <= self.cfg.meas_age_max).float()  # hand_relative: valid while it holds a detection younger than max_age_s
        else:
            cam, cam_yaw, is_box = self._legacy_cam(rel, face, is_box)
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
        if self.design:
            grip_q = self.robot.data.joint_pos.torch[:, self.grip_ids].mean(dim=1, keepdim=True)
            grip_i = (self.robot.data.applied_torque.torch[:, self.grip_ids].abs() / self.grip_lim).mean(dim=1, keepdim=True)
            obs = torch.cat([
                obs,
                valid[:, None],                                          # 1 meas_valid
                (age / self.cfg.meas_age_max).clamp(0, 4)[:, None],      # 1 meas_age / threshold
                10.0 * rel_m.norm(dim=1, keepdim=True),                  # 1 camera-puck distance [dm], from the measurement
                100.0 * (self.tgt_insp_b - ee),                          # 3 inspect-time target, hand relative [cm]
                100.0 * rel_m,                                           # 3 latest measured target, hand relative [cm]
                self.a_old1, self.a_old2,                                # 10 actions 2 and 3 steps back
                grip_q / 0.02, grip_i,                                   # 2 gripper position, current (torque) ratio
                self._tau_ratio().clamp(max=3.0),                        # 5 tau / limit
            ], dim=1)
            ey, ex, th = self._hand_metrics()
            up = math_utils.quat_apply(self.puck.data.root_quat_w.torch, torch.tensor([0.0, 0.0, 1.0], device=self.device).expand(self.num_envs, 3))
            priv = torch.cat([
                priv,
                100.0 * ey[:, None], 100.0 * ex[:, None], th[:, None],   # 3 true alignment (o_y-like e_y, e_x, theta)
                100.0 * (self.plan_aim - torch.cat([self.park, torch.zeros_like(self.park[:, :1])], 1)),  # 3 current plan error
                torch.nn.functional.one_hot(self.situ, 6).float(),       # 6 error mode
                self.insp.float()[:, None],                              # 1 inspect pose wrong
                up[:, :2],                                               # 2 puck axis
            ], dim=1)
        return {"policy": obs, "critic": torch.cat([obs, priv], dim=1)}

    def _get_rewards(self) -> torch.Tensor:
        ph = self._phase()
        e = (self._mid_b() - self._ee_b()[0])[:, :2].norm(dim=1)
        if self.design:
            return self._rewards_design(ph, e)
        w = torch.tensor([0.2, 0.5, 1.0, 0.0, 0.0, 0.0, 0.0], device=self.device)[ph]
        tilt = self._puck_tilt()
        nb_d, nb_hit = self._nb_state()
        self.nb_hit |= nb_hit
        r = self.cfg.w_track * w * torch.exp(-(e / 0.004) ** 2) - self.cfg.w_tilt * (ph < 4) * tilt - self.cfg.w_nb * nb_d
        r = r - self.cfg.w_rate * self.act_rate - self.cfg.w_act * self.act_mag
        self.final_err = torch.where(ph < 3, e, self.final_err)  # aim error when the fingers start closing
        # end of episode: lifted > 30 mm and in the hand, neighbour untouched (cycle_sm.py pick)
        done = ph >= 6
        lifted = (self.puck.data.root_pos_w.torch[:, 2] - self.scene.env_origins[:, 2]) - bc.BOX_H > self.cfg.lift_m
        ok = lifted & self.in_hand & ~self.nb_hit
        r = r + done * (self.cfg.w_success * ok + self.cfg.w_final * torch.exp(-(self.final_err / 0.004) ** 2))
        self.success = torch.where(done, ok, self.success)
        return r - self.cfg.w_fail * (self.knocked | self.tipped)

    def _rewards_design(self, ph, e):
        """Per step: pre-close alignment (stage 3 of the note), neighbour push, action / torque penalties, lift contact.
        At the end: hold score (5 + o_x + theta) or failure (-w_fail - alignment collected so far). No dt factor (DirectRLEnv)."""
        cfg = self.cfg
        nb_d, _ = self._nb_state()
        ey, ex, _ = self._hand_metrics()
        on = (ph < 3) & (self.t >= self.t_tr)
        r1 = cfg.w_align * (torch.exp(-(ey / 0.004) ** 2) + 0.6 * torch.exp(-(ex / 0.005) ** 2)) * on
        self.shaped_sum = self.shaped_sum + r1
        self.final_err = torch.where(ph < 3, e, self.final_err)
        r = r1 - cfg.w_nb * nb_d - cfg.w_rate * self.act_rate - cfg.w_act * self.act_mag
        r = r - cfg.w_tau * (self._tau_ratio().amax(dim=1) - 0.8).clamp(min=0.0)
        r = r - cfg.w_contact * (self._nb_force() / cfg.contact_n).clamp(max=1.0) * (ph == 4)
        hold = self.f_hold & ~self.f_fail
        r = r + hold * (cfg.w_lift + cfg.w_ox * torch.exp(-(self.ox / cfg.ox_scale) ** 2) + cfg.w_th * torch.exp(-(self.th / cfg.th_scale) ** 2))
        return r + self.f_fail * (-cfg.w_fail_d - self.shaped_sum)

    def _get_dones(self):
        ph = self._phase()
        self.in_hand = self._in_hand()
        self.tipped = self._base_tilt() > math.radians(10.0)
        if not self.design:
            self.knocked = self._puck_tilt() > math.radians(30.0)  # a top-down grasp cannot take a puck lying down
            self.dropped = self.knocked  # stats key shared with train.report
            return (ph >= 6) | self.knocked | self.tipped, self.episode_length_buf >= self.max_episode_length
        cfg = self.cfg
        # failures: puck or neighbour knocked over, lost after the lift, base tipped, torque collision
        self.knocked = (self._puck_tilt() > math.radians(30.0)) | (self.nb_on & (self._nb_tilt() > math.radians(30.0)))
        _, nb_hit = self._nb_state()
        self.nb_hit |= nb_hit
        lift = (self.puck.data.root_pos_w.torch[:, 2] - self.scene.env_origins[:, 2]) - bc.BOX_H
        self.lifted_once |= (ph >= 4) & self.in_hand & (lift > 0.01)
        # empty close: after the close (phase 4 on) the fingers went well below the puck width = nothing between them
        empty = (ph >= 4) & (self.robot.data.joint_pos.torch[:, self.grip_ids].mean(dim=1) < bc.gap_joint(cfg.empty_gap_mm))
        self.f_lost = (self.lifted_once & ~self.in_hand) | empty
        self.dropped = self.knocked | self.f_lost
        stuck = ((self._tau_ratio() > cfg.col_tau_ratio) & (self.robot.data.joint_vel.torch[:, self.arm_ids].abs() < cfg.col_vel)).any(dim=1)
        self.col_t = torch.where(stuck & (self.t >= self.t_tr), self.col_t + self.step_dt, torch.zeros_like(self.col_t))
        self.f_col = (self.col_t >= cfg.col_s) & bool(cfg.collide)
        # end of the wait: o_x and theta of the lifted puck in the hand
        done = ph >= 6
        _, ex, self.th = self._hand_metrics()
        self.ox = ex
        self.f_hold = done & (lift > cfg.lift_m) & self.in_hand & ~self.nb_hit
        severe = self.f_hold & ((ex.abs() > cfg.fail_ox) | (self.th > math.radians(cfg.fail_theta_deg)))
        self.f_fail = self.knocked | self.f_lost | self.tipped | self.f_col | severe
        ok = self.f_hold & ~self.f_fail & (ex.abs() < cfg.succ_ox) & (self.th < math.radians(cfg.succ_theta_deg))
        self.success = torch.where(done, ok, self.success & ~self.f_fail)
        self.hold_done = self.f_hold & ~self.f_fail
        return done | self.f_fail, self.episode_length_buf >= self.max_episode_length

    def _log_design(self, ids):
        """Every ended episode counts (a failure is a non-success); lists are aligned for per-situation evaluation."""
        log = self.extras.setdefault("log", {})
        f = lambda x: x[ids].float().mean().item()
        log["Metrics/success"] = f(self.success)
        log["Metrics/err_mm"] = 1000 * f(self.final_err)
        log["Metrics/nb_hit"] = f(self.nb_hit)
        log["Metrics/lost"], log["Metrics/collided"], log["Metrics/knocked"] = f(self.f_lost), f(self.f_col), f(self.knocked)
        hd = self.hold_done[ids]
        if hd.any():
            log["Metrics/ox_mm"] = 1000 * self.ox[ids][hd].abs().mean().item()
            log["Metrics/theta_deg"] = math.degrees(self.th[ids][hd].mean().item())
        self.stats["success"] += self.success[ids].tolist()
        self.stats["err"] += self.final_err[ids].tolist()
        self.stats["dropped"] += self.dropped[ids].tolist()
        self.stats["tipped"] += self.tipped[ids].tolist()
        self.stats["lost"] += self.f_lost[ids].tolist()
        self.stats["collided"] += self.f_col[ids].tolist()
        self.stats["ox"] += [x if h else float("nan") for x, h in zip(self.ox[ids].abs().tolist(), hd.tolist())]
        self.stats["theta"] += [x if h else float("nan") for x, h in zip(self.th[ids].tolist(), hd.tolist())]
        self.stats["situ"] += self.situ[ids].tolist()
        self.stats["insp"] += self.insp[ids].tolist()

    def _reset_idx(self, env_ids):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        if self.common_step_counter > 0 and len(env_ids) > 0:
            if self.design:
                self._log_design(env_ids)
            else:
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
        park = (cfg.common_park_m if self.common_dr else cfg.park_m) * u(2)
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
        if self.design:
            cand = torch.tensor([0, *self.situs], device=dev)
            situ = cand[torch.randint(len(cand), (m,), device=dev)]  # drawn independently of the inspect pose draw
            if cfg.force_situation is not None:
                situ = torch.full_like(situ, cfg.force_situation)
            insp = (torch.rand(m, device=dev) < cfg.insp_err_prob) & self.insp_on
            if cfg.force_insp is not None:
                insp = torch.full_like(insp, cfg.force_insp)
            b1 = cfg.s1_bias_m * u(2) * (situ == 1)[:, None]
            aim[:, :2] += insp[:, None] * (cfg.insp_xy_m * u(2) + cfg.insp_lever_m * cfg.insp_tilt_rad * u(2)
                                           + cfg.insp_z_gain * cfg.insp_z_m * u(2)) + cfg.s1_insp_ratio * b1
            ang = 2 * math.pi * torch.rand(m, device=dev)
            mag = cfg.s2_bias_m[0] + (cfg.s2_bias_m[1] - cfg.s2_bias_m[0]) * torch.rand(m, device=dev)
            self.situ[env_ids], self.insp[env_ids], self.b1[env_ids] = situ, insp, b1
            self.b2[env_ids] = torch.stack([mag * torch.cos(ang), mag * torch.sin(ang)], 1)
            self.park[env_ids], self.plan_aim[env_ids] = park, aim
            self.tgt_insp_b[env_ids] = self.p_nom[side] + aim + torch.tensor([0.0, 0.0, bc.PUCK_H / 2], device=dev)
        want = self.pplan["want"][side]
        yaw_meas = yaw_t + cfg.yaw_noise * u()  # robot root frame = world yaw here (base not rotated at spawn)
        is_box = self._kinds()[env_ids]
        dy = torch.where(is_box, torch.remainder(yaw_meas - want + math.pi / 4, math.pi / 2) - math.pi / 4, zero)
        b = torch.round((dy - YAW_BINS[0]) / math.radians(1.0)).long().clamp(0, len(YAW_BINS) - 1)
        q_above = self.pplan["q_above"][side, b] + (self.pplan["P_above"][side, b] @ aim[..., None])[..., 0]
        q_down = self.pplan["q_down"][side, b] + (self.pplan["P_down"][side, b] @ aim[..., None])[..., 0]
        self.tool_yaw[env_ids] = want + torch.tensor(YAW_BINS, device=dev, dtype=torch.float32)[b]
        self.side[env_ids], self.ybin[env_ids] = side, b
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
        self.tag_bias[env_ids, :2] = cfg.tag_bias_m * u(2)
        for buf in (self.delta, self.prev_act):
            buf[env_ids] = 0.0
        self._sample_sag(env_ids)
        for buf in (self.success, self.nb_hit, self.knocked, self.dropped, self.tipped):
            buf[env_ids] = False
        self.final_err[env_ids] = 0.0

        if self.design:  # camera stream, dropout schedule, termination bookkeeping
            lo, hi = cfg.vid_delay_range
            dlo, dhi = cfg.det_delay_range
            delay = dlo + (dhi - dlo) * torch.rand(m, device=dev) + torch.where(situ == 3, lo + (hi - lo) * torch.rand(m, device=dev), zero)
            self.dsteps[env_ids] = torch.round(delay / self.step_dt).long().clamp(max=self.mh_n - 1)
            s5 = situ == 5
            self.drop_perm[env_ids] = torch.rand(m, device=dev) < cfg.drop_perm_prob
            self.drop_next[env_ids] = torch.where(s5, t_tr + 0.1 + cfg.drop_t_max * torch.rand(m, device=dev), torch.full_like(zero, math.inf))
            self.in_drop[env_ids], self.drop_until[env_ids] = False, 0.0
            self.next_frame[env_ids] = self.last_cap[env_ids] = self.meas_age[env_ids] = 0.0
            self.last_m[env_ids], self.meas_valid[env_ids], self.fresh[env_ids] = 0.0, 1.0, True
            self.shaped_sum[env_ids] = self.col_t[env_ids] = 0.0
            self.a_old1[env_ids] = self.a_old2[env_ids] = 0.0
            for buf in (self.lifted_once, self.hold_done, self.f_fail, self.f_lost, self.f_col, self.f_hold, self.kicked):
                buf[env_ids] = False

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
    kwargs={"env_cfg_entry_point": f"{__name__}:PickDesignEnvCfg"},
)
