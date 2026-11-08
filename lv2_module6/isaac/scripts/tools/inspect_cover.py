#!/usr/bin/env python3
"""gripper_cam 한 장에 시작 상자 위 퍽 자리가 전부 들어오는 팔 자세 — 그중 가장 작게 찍히는 태그가 가장 큰 것.
  FK는 URDF(xacro) 그대로 (robot_state_publisher · perception TF와 같음). 카메라는 Isaac gripper_cam_rgb 핀홀 (fx 615.4, 640×480).
  inspect_pose.py look_at은 카메라 위치가 y로 3 cm 어긋나 있었다 (11/07: 화면에서 70 px 옆).
  조건: 퍽 윗면 태그(z TAG_Z)와 퍽 몸통 가운데(z MID_Z)가 화면 가장자리 EDGE_PX 안 · 손가락이 가리는 화면 아래 가운데(FINGER) 밖 ·
        손끝이 상자 위 HAND_CLEAR 이상 · joint2~4가 한계에서 0.1 rad 이상.

  python3 inspect_cover.py              # 간격 2~12 cm 전부를 덮는 고정 자세 하나
  python3 inspect_cover.py --gap 0.05   # 그 간격 상자만 덮는 자세
  python3 inspect_cover.py --table      # 간격 2~12 cm 1 cm마다 → inspect_cover_table.json (manipulator_env.py --arm-q cover)
  덮을 수 없으면 덮는 비율이 가장 큰 것 (상자 앞쪽 가까운 간격은 팔이 카메라를 충분히 못 올려 한 장에 다 못 담음)
"""
import argparse
import json
import math
import os
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "..", "..", "..", "urdf", "default", "urdf.xacro")
F, W, H = 615.37, 640, 480       # Isaac gripper_cam_rgb: focalLength 15.2449 / horizontalAperture 15.855 × 640
CHASSIS_FRONT, BOX_HALF, MARGIN, BOX_Y_MAX = 0.0688, 0.1025, 0.035, 0.015  # manipulator_layout.py
TAG_Z, MID_Z, TAG_M = 0.13, 0.10, 0.016  # 서 있는 퍽 윗면 · 몸통 가운데 높이 [m], 태그 한 변
EDGE_PX = 30
FINGER = (290, 510, 360)          # u0, u1, v0 — 손가락이 찍히는 화면 아래 가운데 (11/07 Isaac 영상: u 330~470, v 385~)
HAND_CLEAR = 0.04
DEPTH_MIN = 0.17  # [m] 카메라에서 이보다 가까운 점은 안 본 것으로 (실물 D435 640×480 depth 하한 근처 — Isaac depth는 하한 없음)
TAG_CAP = 40.0    # [px] 태그가 이만큼 크면 더 키워도 이득 없음 → 그다음은 화면 가운데(AIM)에 가까운 자세 (판독 기준 약 34 px + 여유)
AIM = (320.0, 220.0)  # 담은 점들 평균이 갈 화면 위치 — 가운데보다 조금 위 (아래 가운데는 손가락)
V_SHIFT = -33.0  # Isaac 영상의 태그가 URDF FK 예측보다 위에 찍힘 [px] (11/07 원기둥 윗면 태그 12개: 사용자 자세 −26 · cover −36 — 팔 처짐으로 봄)
TABLE = os.path.join(HERE, "inspect_cover_table.json")
LO = np.array([-0.25, -1.919862 + 0.1, -1.570796 + 0.1, -1.7 + 0.1])   # joint1 (카메라가 y로 3 cm 옆이라 조금 돌림) · joint2~4 (SRDF 한계)
HI = np.array([0.25, 1.919862 - 0.1, 1.396263 - 0.1, 1.97 - 0.1])


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def _axang(ax, t):
    ax = np.asarray(ax) / np.linalg.norm(ax)
    k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + math.sin(t) * k + (1 - math.cos(t)) * k @ k


class Robot:
    def __init__(self):
        root = ET.fromstring(subprocess.run(["xacro", URDF, "mock_hardware:=true"], check=True, capture_output=True, text=True).stdout)
        self.joints = {j.find("child").get("link"): j for j in root.findall("joint")}

    def fk(self, link, q):
        """base_footprint → link 4×4. q = {joint 이름: rad}"""
        chain = []
        while link in self.joints:
            chain.append(self.joints[link])
            link = chain[-1].find("parent").get("link")
        t = np.eye(4)
        for j in reversed(chain):
            o = j.find("origin")
            a = np.eye(4)
            a[:3, :3] = _rpy(*map(float, o.get("rpy", "0 0 0").split()))
            a[:3, 3] = list(map(float, o.get("xyz", "0 0 0").split()))
            t = t @ a
            if j.get("type") in ("revolute", "continuous"):
                b = np.eye(4)
                b[:3, :3] = _axang(list(map(float, j.find("axis").get("xyz").split())), q.get(j.get("name"), 0.0))
                t = t @ b
        return t


def region(gaps):
    """덮어야 할 퍽 자리 (x, y) 격자 — 그 간격들의 상자 위 (가장자리 MARGIN 안 · 상자 y 흔들림 포함)"""
    x0 = CHASSIS_FRONT + min(gaps) + MARGIN
    x1 = CHASSIS_FRONT + max(gaps) + 2 * BOX_HALF - MARGIN
    y1 = BOX_HALF - MARGIN + BOX_Y_MAX
    return [(x, y) for x in np.linspace(x0, x1, 7) for y in np.linspace(-y1, y1, 5)]


def score(robot, q4, pts):
    """joint1~4 → ((덮은 비율, 덮은 자리 중 가장 작은 태그 [px]), 정보) · 손이 낮거나 카메라 뒤면 ((-1, 0), 이유)
    덮음 = 퍽 윗면 태그와 몸통 가운데가 둘 다 화면 가장자리 EDGE_PX 안 · 손가락 영역 밖"""
    q = {"joint1": q4[0], "joint2": q4[1], "joint3": q4[2], "joint4": q4[3], "joint5": 0.0}
    if robot.fk("end_effector_link", q)[2, 3] < TAG_Z + HAND_CLEAR:
        return (-1, 0, 0), "hand low"
    t = robot.fk("gripper_cam_color_optical_frame", q)
    seen = np.ones(len(pts), bool)
    uv = None
    for z in (TAG_Z, MID_Z):
        p = np.array([[x, y, z] for x, y in pts]) - t[:3, 3]
        c = p @ t[:3, :3]       # 광학 좌표: z 앞 · x 오른쪽 · y 아래
        if np.any(c[:, 2] < 0.05):
            return (-1, 0, 0), "behind"
        u, v = W / 2 + F * c[:, 0] / c[:, 2], H / 2 + F * c[:, 1] / c[:, 2] + V_SHIFT
        seen &= np.linalg.norm(p, axis=1) >= DEPTH_MIN
        seen &= (u > EDGE_PX) & (u < W - EDGE_PX) & (v > EDGE_PX) & (v < H - EDGE_PX)
        seen &= ~((u > FINGER[0]) & (u < FINGER[1]) & (v > FINGER[2]))
        if z == TAG_Z:  # 태그 크기: 거리 · 윗면을 비스듬히 보는 만큼 (두 변 중 한 변만 줄어듦 → √cos)
            d = np.linalg.norm(p, axis=1)
            px = F * TAG_M / d * np.sqrt(np.abs(p[:, 2]) / d)
            uv = np.stack([u, v], 1)
    cover = round(float(seen.mean()), 2)
    if not seen.any():
        return (cover, 0.0, 0.0), {}
    off = -float(np.linalg.norm(uv[seen].mean(0) - AIM))
    return (cover, round(min(float(px[seen].min()), TAG_CAP), 0), off), \
        {"cam": t[:3, 3].round(3).tolist(), "tilt_deg": round(math.degrees(math.acos(-t[2, 2])), 1)}


def search(robot, gaps, n=40000, seed=0, pts=None):
    """gaps의 상자 위 자리 (pts를 주면 그 (x, y)들 — 예: 본 퍽 위치)를 가장 많이 덮고 그중 가장 작은 태그가 가장 큰 joint1~4"""
    rng = np.random.default_rng(seed)
    pts = region(gaps) if pts is None else pts
    best = ((-1, 0, 0), None, None)
    for q4 in rng.uniform(LO, HI, (n, 4)):
        s, info = score(robot, q4, pts)
        if s > best[0]:
            best = (s, q4, info)
    if best[1] is None:
        return best
    for scale in (0.05, 0.02, 0.005):  # 주변 다듬기
        for q4 in best[1] + rng.normal(0, scale, (2000, 4)):
            if np.all(q4 > LO) and np.all(q4 < HI):
                s, info = score(robot, q4, pts)
                if s > best[0]:
                    best = (s, q4, info)
    return best


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--gap", type=float, help="상자 간격 [m] (없으면 0.02~0.12 전부)")
    ap.add_argument("-n", type=int, default=20000)
    ap.add_argument("--table", action="store_true", help=f"간격 2~12 cm 1 cm마다 자세 → {os.path.basename('inspect_cover_table.json')}")
    a = ap.parse_args()
    robot = Robot()
    if a.table:  # 간격별 자세 표 → TABLE (manipulator_env.py --arm-q cover가 가장 가까운 간격 것을 씀)
        rows = {}
        for g in np.arange(0.02, 0.1201, 0.01):
            s, q4, info = search(robot, [g], a.n)
            rows[f"{g:.2f}"] = {"q_deg": [round(math.degrees(v), 2) for v in q4] + [0.0], "cover": s[0], "min_tag_px": round(s[1], 1), **info}
            print(f"{g:.2f}", rows[f"{g:.2f}"], flush=True)
        with open(TABLE, "w") as f:
            json.dump(rows, f, indent=1)
    else:
        s, q4, info = search(robot, [a.gap] if a.gap is not None else [0.02, 0.12], a.n)
        print(f"덮음 {s[0]:.0%} · 최소 태그 {s[1]:.1f} px · q [deg] {','.join(f'{math.degrees(v):.1f}' for v in q4)},0 · {info}")
