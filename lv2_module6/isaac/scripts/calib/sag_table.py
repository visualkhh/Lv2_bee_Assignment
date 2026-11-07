#!/usr/bin/env python3
"""Gravity-sag table from the fitted joint stiffness (sag_fit.py): per hand-down pose, how far the hand sits below what the
encoders/FK say, and the joint angle offsets that would cancel it.

  python3 sag_table.py            # prints the table, writes sag_table.csv next to this file

Model: joint i twists by d_i = tau_i / k_i (gravity torque of everything beyond it). The tip z moves by sum_i dz_i * d_i, plus the
constant offset B found by the fit (tape datum / wheel compression - not a joint property).
Fit: logs/sag_measure/20261104_164324 (35 points, row 3 excluded, --bias --joints 2 3).
To cancel the sag at a pose: command joint_i = wanted_i - d_i (the twists d_i are positive-about-the-axis angles: add -d_i).
"""
import csv
from pathlib import Path

import numpy as np

from arm_model import Arm

K = {2: 12.2, 3: 18.8}  # [Nm/rad]
B = -4.87               # [mm] constant offset of the fit
XS = [0.13, 0.15, 0.17, 0.19, 0.21, 0.23, 0.25, 0.27, 0.29]  # [m] end_effector_link ahead of base_footprint
ZS = [0.10, 0.13, 0.16, 0.19]                                # [m] end_effector_link height (finger tips 20.6 mm lower)

arm = Arm()
rows = []
for x in XS:
    for z in ZS:
        v = arm.hand_down_ik(x, z)
        if v is None: continue
        q = [0.0, v[0], v[1], v[2], 0.0]
        tau, dz = arm.gravity_and_lever(q)
        d = {j: tau[j - 1] / K[j] for j in K}                    # twist [rad]
        part = {j: dz[j - 1] * d[j] * 1000 for j in K}            # [mm]
        rows.append(dict(x_mm=round(x * 1000), z_mm=round(z * 1000),
                         q2_deg=np.degrees(v[0]), q3_deg=np.degrees(v[1]), q4_deg=np.degrees(v[2]),
                         sag_mm=sum(part.values()) + B, sag_compliance_mm=sum(part.values()),
                         d2_deg=np.degrees(d[2]), d3_deg=np.degrees(d[3]),
                         comp_q2_deg=-np.degrees(d[2]), comp_q3_deg=-np.degrees(d[3])))

out = Path(__file__).resolve().parent / 'sag_table.csv'
with open(out, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=rows[0].keys())
    w.writeheader()
    w.writerows({k: (f'{v:.2f}' if isinstance(v, float) else v) for k, v in r.items()} for r in rows)

print(f'sag [mm] (negative = hand below FK), k2 {K[2]} k3 {K[3]} Nm/rad, offset {B} mm; rows = EE x, columns = EE z')
print('x\\z    ' + ''.join(f'{int(z * 1000):>8d}' for z in ZS))
for x in XS:
    cells = {r['z_mm']: r for r in rows if r['x_mm'] == round(x * 1000)}
    print(f'{int(x * 1000):>4d}   ' + ''.join(f'{cells[int(z * 1000)]["sag_mm"]:8.1f}' if int(z * 1000) in cells else f'{"-":>8s}' for z in ZS))
print('\njoint offsets to ADD to the commanded angle [deg] (comp_q2 / comp_q3):')
print('x\\z    ' + ''.join(f'{int(z * 1000):>14d}' for z in ZS))
for x in XS:
    cells = {r['z_mm']: r for r in rows if r['x_mm'] == round(x * 1000)}
    print(f'{int(x * 1000):>4d}   ' + ''.join(f'{cells[int(z * 1000)]["comp_q2_deg"]:6.2f}/{cells[int(z * 1000)]["comp_q3_deg"]:5.2f}' + '  ' if int(z * 1000) in cells else f'{"-":>14s}' for z in ZS))
print(f'\nwritten: {out}')
