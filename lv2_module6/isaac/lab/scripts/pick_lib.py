"""Shared pieces of the scripted pick / place scripts (pick_sm.py, cycle_sm.py): offline arm IK on the URDF."""

import numpy as np
import pinocchio as pin

import bee_env_cfg


class ArmIK:
    """Damped least squares on the full 6D pose (position + down + yaw), solved to convergence offline."""

    def __init__(self):
        self.model = pin.buildModelFromUrdf(str(bee_env_cfg.URDF_PATH))
        self.data = self.model.createData()
        self.ee = self.model.getFrameId("end_effector_link")
        self.base = self.model.getFrameId("base_link")
        ids = [self.model.getJointId(f"joint{i}") for i in range(1, 6)]
        self.iq = [self.model.joints[j].idx_q for j in ids]
        self.iv = [self.model.joints[j].idx_v for j in ids]
        self.lo = self.model.lowerPositionLimit[self.iq]
        self.hi = self.model.upperPositionLimit[self.iq]

    def solve(self, pos, rot, seeds, iters=200, w_ang=1.0):
        """Return (joint1..5, position error m, angle error rad) of the best seed. w_ang < 1 lets the orientation give
        way to the position (e.g. lifting an empty hand, where only the height matters)."""
        target = pin.SE3(rot, pos)
        best = None
        for q5 in seeds:
            q = pin.neutral(self.model)
            q[self.iq] = q5
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
