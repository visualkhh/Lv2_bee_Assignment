#!/usr/bin/env python3
"""Whole-robot centre of mass (x in base_link, wheel axis = 0) at the dice place 'down' pose of each row (section 6).

  <IsaacLab>/.venv/bin/python com_check.py     # URDF masses (beebot_mesh/beebot.urdf), arm IK as cycle_sm.py, 20 g puck in the hand
"""
import math, sys
from pathlib import Path
import numpy as np, pinocchio as pin
sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "lab" / "scripts"))
import bee_env_cfg
from pick_lib import ArmIK

DOWN = pin.Quaternion(np.array([0.0, 1.0, 0.0, 0.0])).toRotationMatrix()
D_EE = np.array([0.0, 0.0, 0.0606])  # puck bottom in the end effector frame
PUCK_M = 0.02
ik = ArmIK(); m, d = ik.model, ik.data
cx = bee_env_cfg.SHELF_NEAR_X + bee_env_cfg.SHELF_HALF


def com(q):
    qf = pin.neutral(m); qf[ik.iq] = q
    pin.framesForwardKinematics(m, d, qf)
    B = d.oMf[ik.base]; T = B.actInv(d.oMf[ik.ee])
    M = sum(i.mass for i in m.inertias)
    c = B.actInv(sum(m.inertias[j].mass * d.oMi[j].act(m.inertias[j].lever) for j in range(len(m.inertias))) / M)
    puck = T.translation + T.rotation @ (D_EE - [0, 0, 0.03])
    return (c * M + puck * PUCK_M) / (M + PUCK_M), M + PUCK_M


print(f"home pose: centre of mass x {1000 * com(np.radians([0.0, -110.0, 80.0, 19.0, 0.0]))[0][0]:+.1f} mm")
for r, name in ((0, "near"), (1, "mid"), (2, "far")):
    t = np.array([cx + 0.06 * (r - 1), 0.0, bee_env_cfg.SHELF_TOP - 0.01])  # base_link is 10 mm above the floor
    hd = math.atan2(t[1], t[0]); rot = pin.utils.rotate("z", hd) @ DOWN
    q, *_ = ik.solve(t - rot @ D_EE, rot, [np.array([hd, -0.2, 0.7, 1.0, 0.0])])
    c, M = com(q)
    print(f"{name} row (cell x {t[0]:.4f} m): robot + puck {M:.2f} kg, centre of mass x {1000 * c[0]:+.1f} mm (+ = ahead of the wheel axis)")
