"""수동 조작 팔 3D 손끝 끌기 — 수평 끌기는 목표 높이 그대로 커서 아래로, Shift 끌기는 높이만 · 닿는 범위 밖으로 못 나감.
실행: .venv/bin/python test_manual_goal.py"""
import types

import numpy as np

import manual_window as mw
import robot_view

P = mw.ManualPanel
mw.shift_held = lambda e: getattr(e, "shift", False)  # 테스트: 이벤트에 shift 표시
p = types.SimpleNamespace(arm_cam=robot_view.OrbitView(target=(0.15, 0, 0.2), dist=0.8, yaw=-2.0, pitch=0.35), goal_drag='xy')
p.view_ray = lambda u, v: P.view_ray(p, u, v)


def px(pt):
    return p.arm_cam.project([pt], mw.VIEW)[0][0]


p.reach = None  # 범위 없음 = 안 막음
p.clamp_reach = lambda q: P.clamp_reach(p, q)
p.active_reach = lambda: p.reach  # '손끝 방향 자유' 꺼짐 = 집기 자세 범위
p.in_reach = lambda q, reach=None: P.in_reach(p, q, reach)
p.goal, p.drag_from = np.array([0.20, 0.0, 0.15]), (0, 0)
u, v = px([0.28, -0.05, 0.15])
P.drag_goal(p, types.SimpleNamespace(x=u, y=v))
assert np.allclose(p.goal, [0.28, -0.05, 0.15], atol=0.005)  # 수평 끌기 — 높이 그대로, 커서 아래로
p.goal_drag, p.drag_from = 'z', (0, 100)
P.drag_goal(p, types.SimpleNamespace(x=0, y=90, shift=True))
assert abs(p.goal[2] - 0.17) < 1e-9  # Shift 끌기 10px 위로 = 2cm 올라감
# 닿는 범위 (회전축 (0,0), r 0.08~0.25 · z 0~0.20 칸) — 멀면 끝까지, 방향은 그대로(joint1 360°), 몸체 지도 없으면 몸체 검사 없음
p.body, p.BODY_HAND_H, p.BODY_CLEAR_Z = None, P.BODY_HAND_H, P.BODY_CLEAR_Z
p.body_floor = lambda x, y, z: P.body_floor(p, x, y, z)
p.reach = (np.zeros(2), np.array([[r, z] for r in np.arange(0.08, 0.2501, 0.01) for z in np.arange(0.0, 0.2001, 0.01)]))
assert np.allclose(P.clamp_reach(p, np.array([0.15, 0.0, 0.10])), [0.15, 0.0, 0.10])            # 안쪽 — 그대로
assert np.allclose(P.clamp_reach(p, np.array([0.60, 0.0, 0.10])), [0.25, 0.0, 0.10], atol=1e-6)  # 멀면 바깥 끝
assert np.allclose(P.clamp_reach(p, np.array([-0.40, 0.0, 0.10])), [-0.25, 0.0, 0.10], atol=1e-6)  # 뒤도 방향 그대로
# 몸체 칸: (0.15, 0) 칸 기둥 높이 0.12까지 → 그 위(0.12 + 여유) 아니면 위로 비킴
cell = 0.01
top = np.full((60, 60, 40), -1)
top[45, 30, 12:] = 11  # x 0.15 · y 0 칸, 높이 0.11~0.12에 몸체 (z 이하 가장 높은 몸체 칸 = 11)
p.body = (-0.3, -0.3, cell, top)
c = P.clamp_reach(p, np.array([0.155, 0.005, 0.05]))
assert c[2] >= float(P.body_floor(p, c[0], c[1], c[2])) - 1e-9 and abs(np.arctan2(c[1], c[0])) < 0.1  # 몸체 위로, 같은 방향
# 동작 순서: 퍽 집기 = 벌리기 → 위 접근 → 내려가기 → 닫기 → 들어올리기
got = []
q = types.SimpleNamespace(goal=np.array([0.25, 0.0, 0.03]), APPROACH=P.APPROACH, GRIP_OPEN=P.GRIP_OPEN, GRIP_CLOSED=P.GRIP_CLOSED,
                          run_seq=lambda name, steps: got.extend(steps))
q.ee_target = lambda: q.goal
P.seq_pick(q)
assert [k for k, _ in got] == ['grip', 'move', 'move', 'grip', 'move'] and np.allclose(got[1][1], [0.25, 0.0, 0.08])
assert got[0][1] == P.GRIP_OPEN and got[3][1] == P.GRIP_CLOSED
print('ok')
