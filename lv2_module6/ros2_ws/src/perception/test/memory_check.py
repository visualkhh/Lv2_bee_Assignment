#!/usr/bin/env python3
"""물체 기억 · 두 카메라 합치기 확인 (track_memory.py) — 카메라·로봇 없이 가짜 카메라 자세(TF)와 가짜 검출로 시나리오를 돌림

  실행: python3 memory_check.py      (ROS 필요 없음 · numpy만)
  결과: 시나리오마다 '통과' / '실패' 와 그때 카드 상태
  팀 자동 테스트(test_node.py)에는 안 들어감 — perception_node에 옮긴 뒤 test_node.py에 같은 시나리오를 넣을 예정

  좌표: odom (x 앞 · y 왼쪽 · z 위) · 카메라 광학 좌표 (x 오른쪽 · y 아래 · z 앞)
  카메라 자세 rot = 카메라 → odom 회전 (열 = 광학 x·y·z 축이 odom에서 가리키는 방향)
"""
import math
import sys

import numpy as np

from track_memory import TrackMemory

FX = FY = 606.0
CX, CY, W, H = 323.0, 241.0, 640, 480
CAM_Z = 0.43  # 폴캠 높이 [m]


def cam_rot(yaw_deg, pitch_deg=18.0):
    """yaw만큼 돌고(0° = odom +x를 봄) pitch만큼 아래를 보는 카메라의 카메라 → odom 회전
    pitch 18° = 팀 URDF pole_front_cam (수평으로 두면 바닥 1m 앞 퍽이 화면 아래로 벗어남 — 첫 시험에서 그래서 실패)"""
    a, p = math.radians(yaw_deg), math.radians(pitch_deg)
    fwd = np.array([math.cos(p) * math.cos(a), math.cos(p) * math.sin(a), -math.sin(p)])  # 광학 z (앞, 아래로 기움)
    right = np.array([math.sin(a), -math.cos(a), 0.0])                                    # 광학 x (오른쪽, 수평)
    down = np.cross(fwd, right)                                                           # 광학 y (아래) = z × x
    return np.stack([right, down, fwd], axis=1)


def view(yaw_deg, cam_pos=(0.0, 0.0, CAM_Z), depth_fn=None):
    """그 순간 카메라 view = (rot, trans, fx, fy, cx, cy, depth_m) — depth_fn(rot, trans) → 깊이 영상, 없으면 '뒤 배경 5m'"""
    rot, trans = cam_rot(yaw_deg), np.array(cam_pos, float)
    depth = depth_fn(rot, trans) if depth_fn else np.full((H, W), 5.0, np.float32)
    return rot, trans, FX, FY, CX, CY, depth


def seen_by(p, v):
    """점 p가 그 카메라 화면 안에 들어오나 (가짜 검출을 만들지 판단)"""
    rot, trans = v[0], v[1]
    pc = rot.T @ (np.asarray(p, float) - trans)
    if pc[2] < 0.1:
        return False
    u, vv = FX * pc[0] / pc[2] + CX, FY * pc[1] / pc[2] + CY
    return 0 <= u < W and 0 <= vv < H


def det(p, cls="puck", color="red", posture="standing", sigma=0.01):
    """가짜 검출 — 위치 p(odom) + 관측 공분산"""
    return {"class_name": cls, "color": color, "posture": posture, "z": np.asarray(p, float),
            "R": np.eye(3) * sigma ** 2, "conf": 0.9}


def step(mem, cam, t, v, objects):
    """한 프레임: 화면 안 물체만 검출로 넣고 합치기 → 지우기"""
    dets = [det(**o) for o in objects if seen_by(o["p"], v)]
    ids = mem.fuse(cam, t, dets)
    mem.prune(t, set(ids), v)
    return ids


results = []


def check(name, ok, mem):
    results.append(ok)
    print(f"{'통과' if ok else '실패'} | {name}")
    for tid, cls, col, p in mem.published():
        print(f"       id {tid} {cls} {col} ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})")


PUCK = {"p": (1.0, 0.0, 0.03), "cls": "puck", "color": "red", "posture": "standing"}
PERSON = {"p": (1.5, 0.3, 0.9), "cls": "person", "color": "", "posture": ""}

# 1) 퍽·사람을 봄 → 로봇이 왼쪽으로 90° 돌아 둘 다 화면 밖 → 5초 지남: 퍽은 남고 사람은 지워짐
mem = TrackMemory()
step(mem, "pole_front_cam", 0.0, view(0), [PUCK, PERSON])
first_id = mem.published()[0][0]
for k in range(1, 26):  # 0.2초마다 5초
    step(mem, "pole_front_cam", 0.2 * k, view(90), [PUCK, PERSON])
names = [c for _, c, _, _ in mem.published()]
check("① 돌아서 화면 밖 5초 — 퍽은 기억 · 사람은 2초 뒤 삭제", names == ["puck"], mem)

# 2) 다시 돌아보면 같은 id에 이어 붙음 (새 카드 안 생김)
ids = step(mem, "pole_front_cam", 5.2, view(0), [PUCK])
check("② 다시 돌아봄 — 같은 id로 이어짐", ids == [first_id] and len(mem.tracks) == 1, mem)

# 3) 누가 집어 감 — 그 자리가 잘 보이는데 3번 연속 없음 → 삭제 (1·2번째엔 아직 남음)
alive = []
for k in range(3):
    step(mem, "pole_front_cam", 5.4 + 0.2 * k, view(0), [])
    alive.append(len(mem.tracks))
check("③ 집어 감 — 잘 보이는 자리에서 3번 연속 없으면 삭제", alive == [1, 1, 0], mem)

# 4) 앞에 다른 물체가 가림 — 그 자리 깊이가 가까움 → 못 봤다고 세지 않음 → 10번 지나도 유지
mem = TrackMemory()
step(mem, "pole_front_cam", 0.0, view(0), [PUCK])


def blocked(rot, trans):
    d = np.full((H, W), 5.0, np.float32)
    d[:, :] = 0.5  # 퍽(1m)보다 앞 0.5m에 무언가가 화면을 가림
    return d


for k in range(1, 11):
    step(mem, "pole_front_cam", 0.2 * k, view(0, depth_fn=blocked), [])
check("④ 앞이 가려짐 — 못 봤다고 세지 않아 계속 기억", len(mem.tracks) == 1, mem)

# 5) 너무 멀어 못 찾음 (3m 넘음) — 세지 않음
mem = TrackMemory()
far = dict(PUCK, p=(3.5, 0.0, 0.03))
step(mem, "pole_front_cam", 0.0, view(0), [far])
for k in range(1, 11):
    step(mem, "pole_front_cam", 0.2 * k, view(0), [])
check("⑤ 3m 넘게 멂 — 못 찾아도 세지 않아 계속 기억", len(mem.tracks) == 1, mem)

# 6) 두 카메라: pole은 옆에서 '서있음', gripper는 위에서 '자세 모름' → id 하나 · 발행 이름 puck
mem = TrackMemory()
a = mem.fuse("pole_front_cam", 0.0, [det(PUCK["p"], posture="standing")])
b = mem.fuse("gripper_cam", 0.0, [det((1.01, 0.01, 0.03), posture="")])
check("⑥ 두 카메라 (서있음 + 자세 모름) — id 하나 · puck", a == b and len(mem.tracks) == 1
      and mem.published()[0][1] == "puck", mem)

# 7) pole이 '누움', gripper는 YOLO 이름 puck_lying·자세 모름 → id 하나 · 발행 이름 puck_lying
mem = TrackMemory()
a = mem.fuse("pole_front_cam", 0.0, [det(PUCK["p"], cls="puck", posture="lying")])
b = mem.fuse("gripper_cam", 0.0, [det((1.01, 0.0, 0.03), cls="puck_lying", posture="")])
check("⑦ 두 카메라 (누움 + 이름만 puck_lying) — id 하나 · puck_lying", a == b and len(mem.tracks) == 1
      and mem.published()[0][1] == "puck_lying", mem)

# 8) 같은 자리라도 색이 다르면 다른 물체 (빨강 퍽 옆에 파랑 퍽 5cm)
mem = TrackMemory()
ids = mem.fuse("pole_front_cam", 0.0, [det(PUCK["p"], color="red"), det((1.0, 0.05, 0.03), color="blue")])
check("⑧ 붙은 다른 색 퍽 두 개 — id 둘", len(set(ids)) == 2, mem)

# 9) 퍽을 0.45m 옮김 (match_dist 0.4m 넘음) → 새 자리는 새 카드 · 옛 자리는 잘 보이는데 없어서 3번 뒤 삭제
#    (0.6m 옆은 31°라 폴캠 가로 반화각 28° 밖 — 첫 시험에서 새 자리가 안 보여 실패)
mem = TrackMemory()
step(mem, "pole_front_cam", 0.0, view(0), [PUCK])
moved = dict(PUCK, p=(1.0, 0.45, 0.03))
for k in range(1, 4):
    step(mem, "pole_front_cam", 0.2 * k, view(0), [moved])
pub = mem.published()
check("⑨ 0.45m 옮김 — 새 자리만 남음 (옛 자리는 3번 뒤 삭제)", len(pub) == 1 and abs(pub[0][3][1] - 0.45) < 0.05, mem)

print(f"\n{sum(results)}/{len(results)} 통과")
sys.exit(0 if all(results) else 1)
