#!/usr/bin/env python3
"""Summarise approach_loop.py result.json files (argv = run folders): docking error vs the TRUE box, success <= 10 mm."""
import json
import math
import os
import sys

import numpy as np

rows = []
for d in sys.argv[1:]:
    try:
        r = json.load(open(os.path.join(d, 'result.json')))
    except (OSError, ValueError):
        continue
    e = r.get('error') or {}
    truth = r.get('true_end_pose') is not None
    pos = math.hypot(e['along_mm'], e['lateral_mm']) if e else math.nan
    fail = next((s['reason'] for s in r['steps'] if not s['ok']), '')
    rows.append((os.path.basename(d), r['ok'], fail, e.get('along_mm', math.nan), e.get('lateral_mm', math.nan),
                 e.get('yaw_deg', math.nan), e.get('gap_mm', math.nan), pos, truth))
if not rows:
    sys.exit('no result.json')
print(f"{'run':22s} {'skill':6s} {'along':>7s} {'lat':>7s} {'pos':>7s} {'yaw':>6s} {'gap':>6s}  truth  reason")
for n, ok, why, al, la, yw, gp, pos, tr in rows:
    print(f"{n:22s} {'ok' if ok else 'FAIL':6s} {al:7.1f} {la:7.1f} {pos:7.1f} {yw:6.2f} {gp:6.1f}  {'yes' if tr else 'NO':5s}  {why}")
pos = np.array([r[7] for r in rows]); yaw = np.abs([r[5] for r in rows]); n = len(rows)
fin = np.isfinite(pos)
print(f"\nrounds {n} · ground truth {sum(r[8] for r in rows)}/{n}")
print(f"position error <= 10 mm: {int((pos[fin] <= 10).sum())}/{n} = {(pos[fin] <= 10).sum() / n * 100:.0f} %"
      f"   (<= 10 mm and |yaw| <= 1.5 deg: {int(((pos <= 10) & (yaw <= 1.5)).sum())}/{n})")
print(f"skill reported success: {sum(r[1] for r in rows)}/{n}")
if fin.any():
    print(f"position error mm: median {np.median(pos[fin]):.1f} · p90 {np.percentile(pos[fin], 90):.1f} · max {pos[fin].max():.1f}")
    print(f"|yaw| deg: median {np.nanmedian(yaw):.2f} · max {np.nanmax(yaw):.2f}")
