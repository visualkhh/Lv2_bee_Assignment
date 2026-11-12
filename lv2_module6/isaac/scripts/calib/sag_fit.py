#!/usr/bin/env python3
"""Fit the joint torsional stiffness from sag_measure.py logs (tape height vs TF height at many poses).

  python3 sag_fit.py [log.csv ...] [--bias] [--joints 2 3 4] [--sigma-mm 2] [--tilt]   # default: the newest logs/sag_measure/*/log.csv

Model (arm_model.py, URDF masses): joint i twists by tau_i / k_i under the gravity torque tau_i of everything beyond it, so the
tip z moves by sum_i dz_i * tau_i / k_i (dz_i = tip z per rad of joint i). With sag = tape - TF (mm) as measured,
  sag_m = sum_i (dz_i * tau_i)_m * c_i       c_i = 1 / k_i [rad/Nm]     -> linear least squares for c_i.
The poses use the encoder angles of the log (q_joint*_deg), so the joint angle error the servo leaves is already in them.
Joint 4 does not move the tip height when the hand points down (dz_4 = 0), joint 1 / 5 carry no gravity torque: from heights
only joints 2 and 3 can be fitted. --bias adds a constant (tape datum error) - it is nearly collinear with joint 3
(tau_3 hardly changes with the pose), so leave it off unless the residuals show an offset.

With the front/rear tape pair (sag_measure.py --span > 0) every pose also gives the hand pitch: the encoders say
eps_enc = q2 + q3 + q4 - 90 deg (hand down = 0), the tape says tilt (positive = nose down), and the difference is the sum of
the twists:  tilt - eps_enc = sum_i tau_i * c_i   (joints 2, 3, 4)  -> this is what makes joint 4 fittable.
Rows are weighted by their noise: height rows sigma = --sigma-mm, tilt rows sigma = sigma * sqrt(2) / span.
"""
import argparse
import csv
import glob
from pathlib import Path

import numpy as np

from arm_model import Arm

parser = argparse.ArgumentParser()
parser.add_argument('logs', nargs='*')
parser.add_argument('--bias', action='store_true')
parser.add_argument('--exclude', nargs='+', type=int, default=[], help='log row numbers (idx column) to leave out, e.g. a misread')
parser.add_argument('--tilt', action='store_true', help='use the front/rear tilt rows too (off: the finger face has a fixed tilt, not gravity)')
parser.add_argument('--tilt-bias', action='store_true', help='constant hand pitch offset in the tilt rows (encoder zero / tape points not level)')
parser.add_argument('--joints', nargs='+', type=int, choices=[2, 3, 4], help='default: 2 3 4 with tilt data, else 2 3')
parser.add_argument('--sigma-mm', type=float, default=2.0, help='tape reading noise [mm] (weights height rows against tilt rows)')
args = parser.parse_args()
paths = args.logs or sorted(glob.glob(str(Path(__file__).resolve().parent / 'logs' / 'sag_measure' / '*' / 'log.csv')))[-1:]

arm = Arm()
rows = []  # pose, from, sag [mm], dz*tau [mm per rad/Nm], tau [Nm], tf_x, tf_z, tilt - eps_enc [rad] or nan, span [mm]
for p in paths:
    for r in csv.DictReader(open(p)):
        if r['sag_mm'] in ('', 'nan') or int(r['idx']) in args.exclude: continue
        q = np.radians([float(r[f'q_joint{i}_deg']) for i in range(1, 6)])
        tau, dz = arm.gravity_and_lever(q, float(r['gripper_pos']))
        tilt = float(r['tilt_deg']) if r.get('tilt_deg') not in (None, '', 'nan') else np.nan
        dtilt = np.radians(tilt) - (q[1] + q[2] + q[3] - np.pi / 2) if not np.isnan(tilt) else np.nan
        rows.append((r['pose'], r['from_pose'], float(r['sag_mm']), dz * tau * 1000.0, tau, float(r['tf_x_mm']), float(r['tf_z_mm']),
                     dtilt, float(r['span_mm']) if r.get('span_mm') else np.nan))
have_tilt = [r for r in rows if not np.isnan(r[7])] if args.tilt else []
joints = args.joints or ([2, 3, 4] if np.ptp([r[3][3] for r in rows]) > 15 or have_tilt else [2, 3])  # joint 4 only if the poses move the tip with it
if 4 in joints and not have_tilt and np.ptp([r[3][3] for r in rows]) < 15: raise SystemExit('joint 4 needs poses with a pitched hand (sag_measure.py --grid --pitches -30 0 30) or --tilt data')
if len(rows) + len(have_tilt) < len(joints) + args.bias + 1:
    raise SystemExit(f'only {len(rows)} usable rows (need more than {len(joints) + args.bias})')

def col(r, with_bias): return [r[3][j - 1] for j in joints] + ([1.0] if with_bias else [])
# height rows [mm], tilt rows [rad]; both scaled by 1 / sigma
Ah = np.array([col(r, args.bias) + ([0.0] if args.tilt_bias and have_tilt else []) for r in rows]) / args.sigma_mm
yh = np.array([r[2] for r in rows]) / args.sigma_mm
At = np.array([[r[4][j - 1] for j in joints] + ([0.0] if args.bias else []) + ([1.0] if args.tilt_bias else []) for r in have_tilt]).reshape(len(have_tilt), -1) if have_tilt else None
yt = np.array([r[7] for r in have_tilt])
sig_t = np.array([args.sigma_mm * np.sqrt(2) / r[8] for r in have_tilt])
A = np.vstack([Ah, At / sig_t[:, None]]) if len(have_tilt) else Ah
y = np.concatenate([yh, yt / sig_t]) if len(have_tilt) else yh
c, *_ = np.linalg.lstsq(A, y, rcond=None)
pred = np.array([np.dot(col(r, args.bias), c[:len(col(r, args.bias))]) for r in rows])
res = np.array([r[2] for r in rows]) - pred
cond = np.linalg.cond(A)
joints_names = [f'joint{j}' for j in joints]
print(f'{len(rows)} height points + {len(have_tilt)} tilt points from {len(paths)} log(s), joints {joints}{" + bias" if args.bias else ""}, cond(A) = {cond:.0f}')
for n, ci in zip(joints_names + (['bias'] if args.bias else []) + (['tilt_bias'] if args.tilt_bias and have_tilt else []), c):
    if n == 'bias': print(f'  bias   {ci:+.2f} mm')
    elif n == 'tilt_bias': print(f'  tilt_bias {np.degrees(ci):+.2f} deg (constant hand pitch offset)')
    else: print(f'  {n}: c = {ci:.5f} rad/Nm -> k = {1 / ci if ci > 0 else float("inf"):.1f} Nm/rad  ({np.degrees(ci):.3f} deg per Nm)')
print(f'height residual rms {np.sqrt(np.mean(res ** 2)):.2f} mm (data rms {np.sqrt(np.mean([r[2] ** 2 for r in rows])):.2f} mm)')
if have_tilt:
    ptilt = np.array([np.dot([r[4][j - 1] for j in joints], c[:len(joints)]) + (c[-1] if args.tilt_bias else 0.0) for r in have_tilt])
    rt = np.degrees(np.array([r[7] for r in have_tilt]) - ptilt)
    print(f'tilt residual rms {np.sqrt(np.mean(rt ** 2)):.3f} deg (data rms {np.degrees(np.sqrt(np.mean([r[7] ** 2 for r in have_tilt]))):.3f} deg)')
print(f'{"pose":10s}{"from":10s}{"x":>7s}{"tf_z":>7s}{"measured":>10s}{"model":>8s}{"resid":>7s}{"tilt_d":>8s}{"tilt_m":>8s}')
for r, pr, e in zip(rows, pred, res):
    tm = np.degrees(np.dot([r[4][j - 1] for j in joints], c[:len(joints)]) + (c[-1] if args.tilt_bias and have_tilt else 0.0)) if have_tilt and not np.isnan(r[7]) else np.nan
    print(f'{r[0]:10s}{r[1]:10s}{r[5]:7.0f}{r[6]:7.1f}{r[2]:10.1f}{pr:8.1f}{e:7.1f}{np.degrees(r[7]):8.2f}{tm:8.2f}')
if cond > 100: print('WARNING: poses are nearly collinear for these unknowns - use more varied x / z (sag_measure.py --grid)')
if any(ci <= 0 for ci in c[:len(joints)]): print('WARNING: a stiffness came out <= 0 - noise or backlash dominates; repeat poses (--rounds 2) and use --via')
