"""Arm correction of the real rl_arm_correction (ros2_ws/src/ml), one place for every skill env.

command = q_ref + corr (sag integral) + delta (policy residual) + gravity feed-forward.
The integral takes out the sag the feed-forward left: error = q_ref + delta - q (the real controller with twist_state
uses the same error, its state is already the drooped angle). Per episode: corr = 0, integral gain and sag size sampled.
Gravity torques port gravity::torques of rl_arm_correction.hpp (planar arm, masses and centres of mass from the URDF).

cfg.twist (real pair): the physics joint is the link. The servo error the encoder sees is the joint drive sag
(stiffness 1 / servo_c, x sag_scale per episode); the twist the encoder does not see is a target offset
+c_true * tau (c_true = twist_c x U(1 -+ twist_dr) per episode); the feed-forward cancels it with the nominal twist_c,
as rl_arm_correction does. The robot reports twist_state = encoder + twist_c * tau, so the state the integral and the
policy see is link - (c_true - twist_c) * tau (q_seen). c_true = twist_c: identical to no twist and no feed-forward.
"""

from __future__ import annotations

import torch

_G = 9.81
_M_UA, _UA_X, _UA_Z = 0.128865, 0.008487, 0.104997
_M_FA, _FA_X = 0.117300, 0.093796
_M_WR, _WR_X, _WR_Z = 0.090152, 0.043623, 0.011449
_M_HD, _HD_X, _HD_Z = 0.371242, 0.052686, 0.014472
_J3_X, _J3_Z, _J4_X, _HAND_X = 0.024, 0.128, 0.124, 0.0629


def _rotx(th, x, z):  # x of the vector (x, z) turned by pitch th
    return x * torch.cos(th) + z * torch.sin(th)


def gravity_torques(q2, q3, q4):
    """Gravity torque [Nm] about joints 2 and 3 (forward sag +), shape [n, 2]."""
    t23, t234 = q2 + q3, q2 + q3 + q4
    ua, p3 = _rotx(q2, _UA_X, _UA_Z), _rotx(q2, _J3_X, _J3_Z)
    p4x = p3 + _rotx(t23, _J4_X, 0.0)
    fax = p3 + _rotx(t23, _FA_X, 0.0)
    wr, hd = p4x + _rotx(t234, _WR_X, _WR_Z), p4x + _rotx(t234, _HAND_X + _HD_X, _HD_Z)
    return torch.stack([_G * (_M_UA * ua + _M_FA * fax + _M_WR * wr + _M_HD * hd),
                        _G * (_M_FA * (fax - p3) + _M_WR * (wr - p3) + _M_HD * (hd - p3))], dim=1)


class ArmCorrection:
    def __init__(self, cfg, n, device, robot, arm_ids):
        self.cfg, self.dev, self.robot, self.arm_ids = cfg, device, robot, arm_ids
        self.corr = torch.zeros(n, 5, device=device)
        self.corr_k = torch.full((n,), cfg.corr_gain, device=device)
        self.k_arm0 = robot.data.joint_stiffness.torch[:, arm_ids].clone()  # nominal stiffness: the feed-forward divides by it
        self.c_twist = torch.zeros(n, 5, device=device)  # per-episode true twist compliance [rad/Nm] (cfg.twist)
        if cfg.twist:
            self.k_arm0[:, 1] = 1.0 / cfg.servo_c[0]
            self.k_arm0[:, 2] = 1.0 / cfg.servo_c[1]
            robot.write_joint_stiffness_to_sim_index(stiffness=self.k_arm0.clone(), joint_ids=arm_ids)
            self.c_twist[:, 1:3] = torch.tensor(cfg.twist_c, device=device)

    def _tau(self, q):
        """Gravity torque about joints 2 and 3 [n, 5] (zeros elsewhere)."""
        tau = torch.zeros_like(q)
        tau[:, 1:3] = gravity_torques(q[:, 1], q[:, 2], q[:, 3])
        return tau

    def q_seen(self):
        """Arm joint state as the robot reports it: link, or link - (c_true - c_nominal) x tau with cfg.twist."""
        q = self.robot.data.joint_pos.torch[:, self.arm_ids]
        if not self.cfg.twist:
            return q
        c0 = torch.zeros_like(q)
        c0[:, 1:3] = torch.tensor(self.cfg.twist_c, device=self.dev)
        return q - (self.c_twist - c0) * self._tau(q)

    def gravity_ff(self):
        """Feed-forward joint offsets [n, 5]: -tau_j / k_j (nominal stiffness) for joints 2 and 3 (joint4 compliance is 0 on the robot)."""
        ff = torch.zeros_like(self.corr)
        if self.cfg.ff_gain == 0.0:
            return ff
        q = self.robot.data.joint_pos.torch[:, self.arm_ids]
        if self.cfg.twist:  # rl_arm_correction: -c_nominal x tau (on the robot from the reported state; here the link, ~same)
            ff[:, 1:3] = (-self.cfg.ff_gain * torch.tensor(self.cfg.twist_c, device=self.dev) * self._tau(q)[:, 1:3]).clamp(-self.cfg.ff_max, self.cfg.ff_max)
            return ff
        tau = gravity_torques(q[:, 1], q[:, 2], q[:, 3])
        ff[:, 1:3] = (-self.cfg.ff_gain * tau[:, :2] / self.k_arm0[:, 1:3]).clamp(-self.cfg.ff_max, self.cfg.ff_max)
        return ff

    def step(self, q_ref, delta, holding, t, dt):
        """Joint target. While holding, every corr_dt the integral moves corr by corr_k x (q_ref + delta - q)."""
        tick = (torch.round(t / dt).long() % max(1, round(self.cfg.corr_dt / dt))) == 0
        q_err = q_ref + delta - self.q_seen()
        new = (self.corr + self.corr_k[:, None] * q_err).clamp(-self.cfg.corr_max, self.cfg.corr_max)
        self.corr = torch.where((holding & tick)[:, None], new, self.corr)
        cmd = q_ref + self.corr + delta + self.gravity_ff()
        if self.cfg.twist:  # the twist drops the link below the servo shaft, out of the encoder's sight
            cmd = cmd + self.c_twist * self._tau(self.robot.data.joint_pos.torch[:, self.arm_ids])
        return cmd

    def reset(self, env_ids):
        """Per episode: corr = 0, integral gain and sag size (arm stiffness k0 / s: sag under gravity ~ 1 / k)."""
        cfg, m = self.cfg, len(env_ids)
        self.corr[env_ids] = 0.0
        g = 1.0 if cfg.corr_gain_range is None else torch.empty(m, device=self.dev).uniform_(*cfg.corr_gain_range)
        self.corr_k[env_ids] = cfg.corr_gain * g
        if cfg.twist:
            d = torch.empty(m, 2, device=self.dev).uniform_(1 - cfg.twist_dr, 1 + cfg.twist_dr)
            self.c_twist[env_ids, 1:3] = torch.tensor(cfg.twist_c, device=self.dev) * d
        if cfg.sag_scale_range is not None:
            s = torch.empty(m, 1, device=self.dev).uniform_(*cfg.sag_scale_range)
            self.robot.write_joint_stiffness_to_sim_index(stiffness=self.k_arm0[env_ids] / s, joint_ids=self.arm_ids, env_ids=env_ids)

    def obs(self, obs):
        return torch.cat([obs, 10.0 * self.corr], dim=1) if self.cfg.obs_corr else obs
