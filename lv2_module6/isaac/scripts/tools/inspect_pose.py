#!/usr/bin/env python3
"""상자 윗면 중심을 gripper_cam으로 내려다보는 팔 자세 — 손은 수직일 필요 없음 (카메라가 손목 앞·위에 18° 기울어 달려 있음).
  FK는 skills/src/stand_plan.cpp fk와 같고 카메라 외부 파라미터는 urdf gripper_cam_joint. 조건: 광축이 목표를 3° 안으로 향함 ·
  거리 0.16~0.22 m (16 mm 태그 ≥ 34 px) · 광축 수직 기울기 최소 · joint2~4가 한계에서 margin 이상 떨어짐.
  python3 inspect_pose.py 0.24            # 상자 중심 x [m]
"""
import math
import sys

import numpy as np

ARM = np.array([0.047985, -0.00023, 0.140175])
D1, D2, L1X, L1Z, L2, L3 = 0.0409, 0.0416, 0.024, 0.128, 0.124, 0.0629
LO = np.array([-3.14159, -1.919862, -1.570796, -1.7, -3.141593])
HI = np.array([3.14159, 1.919862, 1.396263, 1.97, 3.141593])
CAM_T = np.array([0.081681, -0.000273, 0.040325])
STOW = (0.0, -1.9024, 1.344, 0.9023, 0.0)
FOCAL_PX = 465.0  # 640 px, 가로 시야 69.4°


def _rz(a): c, s = math.cos(a), math.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
def _ry(a): c, s = math.cos(a), math.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def _rx(a): c, s = math.cos(a), math.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


R_CAM = _rz(-0.0010995) @ _ry(0.3490659) @ _rx(-0.00014948)


def camera(q):
    """(카메라 위치, 광축 단위벡터) — base_footprint 좌표."""
    p2 = ARM + np.array([0, 0, D1 + D2])
    r2 = _rz(q[0]) @ _ry(q[1])
    p3 = p2 + r2 @ np.array([L1X, 0, L1Z])
    r3 = r2 @ _ry(q[2])
    p4 = p3 + r3 @ np.array([L2, 0, 0])
    r4 = r3 @ _ry(q[3])
    p5 = p4 + r4 @ np.array([L3, 0, 0])
    rg = r4 @ _rx(q[4])
    return p5 + rg @ CAM_T, rg @ R_CAM @ np.array([1.0, 0, 0])


def look_at(target, margin=0.10, d_ranges=((0.16, 0.22), (0.16, 0.26)), tol_deg=3.0, n=150000, seed=0):
    """target = 상자 윗면 중심 (x, y, z) → (q[5], 정보 dict) 또는 (None, 이유). 가까운 거리 범위부터 시도해 기울기가 가장 작은 자세."""
    for d_range in d_ranges:
        q, info = _look_at(target, margin, d_range, tol_deg, n, seed)
        if q is not None:
            return q, info
    return None, 'no pose'


def _look_at(target, margin, d_range, tol_deg, n, seed):
    rng = np.random.default_rng(seed)
    t = np.asarray(target, float)
    q = np.c_[rng.uniform(-0.4, 0.4, n), rng.uniform(LO[1] + margin, HI[1] - margin, n), rng.uniform(LO[2] + margin, HI[2] - margin, n),
              rng.uniform(LO[3] + margin, HI[3] - margin, n), np.zeros(n)]
    best = None
    for k in range(n):
        c, ax = camera(q[k])
        v = t - c
        d = np.linalg.norm(v)
        if not (d_range[0] <= d <= d_range[1]) or c[2] < 0.2:
            continue
        if math.degrees(math.acos(np.clip(ax @ v / d, -1, 1))) > tol_deg:
            continue
        tilt = math.degrees(math.acos(np.clip(-ax[2], -1, 1)))
        if best is None or tilt < best[0]:
            best = (tilt, d, q[k].copy(), c)
    if best is None:
        return None, 'no pose'
    tilt, d, qb, c = best
    return qb, {'tilt_deg': tilt, 'dist_m': d, 'cam': c.round(3).tolist(), 'tag_px': FOCAL_PX * 0.016 / d * math.cos(math.radians(tilt))}


if __name__ == '__main__':
    xc = float(sys.argv[1]) if len(sys.argv) > 1 else 0.24
    q, info = look_at((xc, 0.0, 0.13), n=60000)
    print(None if q is None else q.round(3).tolist(), info)
