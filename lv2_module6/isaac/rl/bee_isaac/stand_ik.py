"""Batched torch forward kinematics + damped least squares step for the 5-axis arm (tool down + yaw held).

Kinematics come from the URDF through pinocchio (bc.ArmIK) so they stay the same as the offline IK: chain joint1 (RZ),
joint2-4 (RY), joint5 (RX), end_effector_link after joint5, all in the base_link frame.
"""

import numpy as np
import torch

from bee_isaac import common as bc

AXES = (2, 1, 1, 1, 0)  # joint1..5 rotation axis in the joint frame (RZ, RY, RY, RY, RX)


def _rot(axis, q):
    c, s = torch.cos(q), torch.sin(q)
    o, z = torch.ones_like(q), torch.zeros_like(q)
    if axis == 0:
        r = [o, z, z, z, c, -s, z, s, c]
    elif axis == 1:
        r = [c, z, s, z, o, z, -s, z, c]
    else:
        r = [c, -s, z, s, c, z, z, z, o]
    return torch.stack(r, -1).reshape(*q.shape, 3, 3)


class TorchArm:
    def __init__(self, device):
        ik = bc.ArmIK()
        m = ik.model
        f = lambda a: torch.tensor(np.asarray(a), dtype=torch.float32, device=device)
        ids = [m.getJointId(f"joint{i}") for i in range(1, 6)]
        self.R_pl = f(np.stack([m.jointPlacements[j].rotation for j in ids]))  # (5, 3, 3) parent -> joint frame
        self.p_pl = f(np.stack([m.jointPlacements[j].translation for j in ids]))  # (5, 3)
        ee = m.frames[ik.ee].placement
        self.R_ee, self.p_ee = f(ee.rotation), f(ee.translation)
        base = ik.data.oMf[ik.base] if False else None
        import pinocchio as pin
        pin.framesForwardKinematics(m, ik.data, pin.neutral(m))
        b = ik.data.oMf[ik.base]
        self.R_b, self.p_b = f(b.rotation), f(b.translation)  # base_link in the universe frame
        self.lo, self.hi = f(ik.lo), f(ik.hi)

    def fk(self, q):
        """q (n, 5) -> position (n, 3), rotation (n, 3, 3) of end_effector_link in base_link; plus joint axes / origins."""
        n = q.shape[0]
        R = torch.eye(3, device=q.device).expand(n, 3, 3)
        p = torch.zeros(n, 3, device=q.device)
        axes, origins = [], []
        for i in range(5):
            p = p + (R @ self.p_pl[i][:, None])[..., 0]
            R = R @ self.R_pl[i]
            e = torch.zeros(3, device=q.device)
            e[AXES[i]] = 1.0
            axes.append((R @ e[:, None])[..., 0])
            origins.append(p)
            R = R @ _rot(AXES[i], q[:, i])
        pos = p + (R @ self.p_ee[:, None])[..., 0]
        rot = R @ self.R_ee
        Rb_t = self.R_b.T
        pos_b = ((pos - self.p_b) @ self.R_b)  # universe -> base: R_b^T (x - p_b)
        rot_b = Rb_t @ rot
        axes_b = [a @ self.R_b for a in axes]
        orig_b = [(o - self.p_b) @ self.R_b for o in origins]
        return pos_b, rot_b, axes_b, orig_b

    def step(self, q, p_tgt, R_tgt, iters=3, lam=0.02, gain=0.8, w_ang=1.0):
        """A few damped least squares iterations toward the pose; returns the new q (n, 5)."""
        eye6 = torch.eye(6, device=q.device)
        for _ in range(iters):
            p, R, axes, orig = self.fk(q)
            e_pos = p_tgt - p
            e_rot = 0.5 * sum(torch.cross(R[:, :, k], R_tgt[:, :, k], dim=1) for k in range(3))
            err = torch.cat([e_pos, w_ang * e_rot], 1)
            J = torch.stack([torch.cat([torch.cross(a, p - o, dim=1), w_ang * a], 1) for a, o in zip(axes, orig)], 2)  # (n, 6, 5)
            dq = (J.transpose(1, 2) @ torch.linalg.solve(J @ J.transpose(1, 2) + lam * lam * eye6, err[..., None]))[..., 0]
            q = torch.maximum(torch.minimum(q + gain * dq, self.hi), self.lo)
        return q


if __name__ == "__main__":  # check against pinocchio, then a small hand move
    ik = bc.ArmIK()
    arm = TorchArm("cpu")
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(20):
        q5 = ik.lo + (ik.hi - ik.lo) * rng.random(5)
        p0, r0 = ik.fk(q5)
        p1, r1, *_ = arm.fk(torch.tensor(q5, dtype=torch.float32)[None])
        worst = max(worst, np.abs(p0 - p1[0].numpy()).max(), np.abs(r0 - r1[0].numpy()).max())
    print("fk max abs diff vs pinocchio:", worst)
    assert worst < 1e-4
    q = torch.tensor([[0.2, -0.3, 0.8, 1.0, 0.4]])
    p, R, *_ = arm.fk(q)
    for dz in (0.005, 0.02):
        q2 = arm.step(q, p + torch.tensor([[0.01, -0.01, dz]]), R, iters=5)
        p2, R2, *_ = arm.fk(q2)
        print("target step dz", dz, "pos err mm", ((p2 - p - torch.tensor([[0.01, -0.01, dz]])).norm() * 1000).item(),
              "rot err", (R2 - R).abs().max().item())
    print("ok")
