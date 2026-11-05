"""Shared pieces of the BEE Isaac Lab scripts: paths, robot config, offline arm IK, puck / box geometry.

Ported from feature/isaac-pick (isaac/lab/scripts/bee_env_cfg.py, pick_lib.py, cycle_sm.py) so this branch
runs on its own. Values are the final model of the 2026-10-30 sim report (grip 29 N, 100 Hz, measured latency).
"""

import math
from pathlib import Path

import numpy as np
import pinocchio as pin

ROOT = Path(__file__).resolve().parents[2]  # lv2_module6
USD_DIR = ROOT / "isaac" / "sim" / "usda"
URDF_PATH = ROOT / "urdf" / "beebot_mesh" / "beebot.urdf"  # same kinematics as the USD
PUCK_USDS = [
    str(USD_DIR / "puck" / "box" / "puck_box_green_0.usda"),
    str(USD_DIR / "puck" / "box" / "puck_box_red_1.usda"),
    str(USD_DIR / "puck" / "cyl" / "puck_cyl_green_2.usda"),
    str(USD_DIR / "puck" / "cyl" / "puck_cyl_red_3.usda"),
]

# end_effector_link relative to gripper_link (merged away in Lab, fixed joint)
EE_OFFSET_POS = (0.109055, 0.0, 0.0)
EE_OFFSET_ROT = (0.0, 0.7071068, 0.0, 0.7071068)  # xyzw

# puck and grasp
PUCK_H = 0.06
FINGER_TIP = 0.0206
GRIP_DEPTH = 0.020
GRASP_Z = PUCK_H - GRIP_DEPTH + FINGER_TIP  # end effector above the puck bottom when grasped
GRIP_STOP = 0.0062  # finger joint where the fingers stop on a 30 mm puck (convex decomposition colliders)
GRIP_FORCE = 29.0  # N per finger, team current 0.4 A (report 6)


def gap_joint(gap_mm):
    """Finger joint value for a finger gap [mm] (30 mm puck at GRIP_STOP)."""
    return GRIP_STOP + (gap_mm / 1000 - 0.030) / 2


# mission box 205 x 205 x 70, 3 x 3 dice grid at 60 mm
BOX_HALF = 0.205 / 2
BOX_H = 0.07
CELL_PITCH = 0.060

# joint motion like MoveIt with the team limits (3 rad/s, 5 rad/s^2) x vel_scale
VMAX, AMAX = 3.0, 5.0
LATENCY_S = (0.045, 0.062, 0.052, 0.045, 0.045)  # measured command latency per arm joint (report 4)
LATENCY_GRIP_S = 0.020

DOWN = pin.Quaternion(np.array([0.0, 1.0, 0.0, 0.0])).toRotationMatrix()  # end effector z pointing down
HOME_DEG = (0.0, -110.0, 80.0, 19.0, 0.0)


def trapezoid_duration(dist, vel_scale):
    """Synchronized trapezoid (or triangle) duration for the largest joint distance [rad]."""
    vmax, amax = VMAX * vel_scale, AMAX * vel_scale
    t_acc = vmax / amax
    if dist <= vmax * t_acc:
        return max(2 * math.sqrt(dist / amax), 0.1)
    return dist / vmax + t_acc


def move_s(t, dur):
    """Normalized position 0..1 at time t of a move lasting dur (smooth step; floats or torch tensors).

    ponytail: smooth step instead of the exact trapezoid of cycle_sm.py; same duration, the training env and the demo
    share this one.
    """
    x = t / dur
    x = x.clamp(0, 1) if hasattr(x, "clamp") else min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


class ArmIK:
    """Damped least squares on the full 6D pose (position + down + yaw), solved to convergence offline."""

    def __init__(self):
        self.model = pin.buildModelFromUrdf(str(URDF_PATH))
        self.data = self.model.createData()
        self.ee = self.model.getFrameId("end_effector_link")
        self.base = self.model.getFrameId("base_link")
        ids = [self.model.getJointId(f"joint{i}") for i in range(1, 6)]
        self.iq = [self.model.joints[j].idx_q for j in ids]
        self.iv = [self.model.joints[j].idx_v for j in ids]
        self.lo = self.model.lowerPositionLimit[self.iq]
        self.hi = self.model.upperPositionLimit[self.iq]

    def _q(self, q5):
        q = pin.neutral(self.model)
        q[self.iq] = q5
        return q

    def fk(self, q5):
        """End effector pose in base_link: (position, rotation)."""
        pin.framesForwardKinematics(self.model, self.data, self._q(q5))
        cur = self.data.oMf[self.base].actInv(self.data.oMf[self.ee])
        return cur.translation.copy(), cur.rotation.copy()

    def frame_in_ee(self, name):
        """Pose of a frame in the end effector frame at the neutral configuration."""
        pin.framesForwardKinematics(self.model, self.data, self._q(np.zeros(5)))
        m = self.data.oMf[self.ee].actInv(self.data.oMf[self.model.getFrameId(name)])
        return m.translation.copy(), m.rotation.copy()

    def base_in_root(self):
        pin.framesForwardKinematics(self.model, self.data, self._q(np.zeros(5)))
        return self.data.oMf[self.base].translation.copy()

    def jac_pos(self, q5):
        """Least-squares joint step per end effector position step with the orientation held: (5, 3)."""
        q = self._q(q5)
        pin.computeJointJacobians(self.model, self.data, q)
        pin.framesForwardKinematics(self.model, self.data, q)
        J = pin.getFrameJacobian(self.model, self.data, self.ee, pin.LOCAL_WORLD_ALIGNED)[:, self.iv]
        return np.linalg.pinv(J)[:, :3]

    def solve(self, pos, rot, seeds, iters=200, w_ang=1.0):
        """Return (joint1..5, position error m, angle error rad) of the best seed. w_ang < 1 lets the orientation give
        way to the position (e.g. lifting an empty hand, where only the height matters)."""
        target = pin.SE3(rot, pos)
        best = None
        for q5 in seeds:
            q = self._q(q5)
            for _ in range(iters):
                pin.framesForwardKinematics(self.model, self.data, q)
                cur = self.data.oMf[self.base].actInv(self.data.oMf[self.ee])
                err = pin.log6(cur.actInv(target)).vector
                if np.linalg.norm(err[:3]) < 2e-4 and np.linalg.norm(err[3:]) < 2e-3:
                    break
                err[3:] *= w_ang
                J = pin.computeFrameJacobian(self.model, self.data, q, self.ee, pin.LOCAL)[:, self.iv]
                J[3:] *= w_ang
                dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), err)
                q[self.iq] = np.clip(q[self.iq] + 0.5 * dq, self.lo, self.hi)
            pin.framesForwardKinematics(self.model, self.data, q)
            cur = self.data.oMf[self.base].actInv(self.data.oMf[self.ee])
            err = pin.log6(cur.actInv(target)).vector
            res = (q[self.iq].copy(), np.linalg.norm(err[:3]), np.linalg.norm(err[3:]))
            if best is None or res[1] + 0.1 * res[2] < best[1] + 0.1 * best[2]:
                best = res
        return best


def place_rot(target_xy):
    """Tool rotation over a dice cell: gripper down, yaw on the grid axis nearest the heading (box faces on the grid)."""
    hd = math.atan2(target_xy[1], target_xy[0])
    yaw = hd + math.remainder(-hd, math.pi / 2)
    return pin.utils.rotate("z", yaw) @ DOWN, hd, yaw


def place_seeds(hd, yaw):
    return [np.array([hd, -0.2, 0.7, 1.0, s]) for s in (0.0, yaw - hd, hd - yaw, math.pi / 2, -math.pi / 2)]
