"""hand_geom 계산 확인 (ROS 없음) — 시뮬 정책 입력 정의와 같은지: python3 -m pytest test/test_hand_relative.py"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
import hand_geom as hg  # noqa: E402

DOWN = np.diag([1.0, -1.0, -1.0])  # 손끝 z 아래 (isaac/rl/bee_isaac/tasks bee_common.DOWN = x축 180°)


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def test_bottom_in_hand_below_tool():
    ee_t = np.array([0.2, 0.0, 0.25])
    b = hg.bottom_in_hand(ee_t, DOWN)
    assert np.allclose(b, [0.2, 0.0, 0.25 - hg.GRASP_Z])


def test_tool_yaw_and_face():
    R = rz(0.3) @ DOWN
    assert abs(hg.tool_yaw(R) - 0.3) < 1e-9
    assert abs(hg.face_yaw(0.3 + math.radians(10), 0.3, True) - math.radians(10)) < 1e-9
    assert abs(hg.face_yaw(0.3 + math.radians(100), 0.3, True) - math.radians(10)) < 1e-9  # 사각 퍽은 90° 대칭
    assert hg.face_yaw(1.0, 0.3, False) == 0.0


def test_to_base_and_cell_shift():
    base_t, base_R = np.array([1.0, 2.0, 0.0]), rz(math.pi / 2)
    assert np.allclose(hg.to_base([1.0, 3.0, 0.0], base_t, base_R), [1.0, 0.0, 0.0])  # 로봇 앞 1 m
    c = hg.cell_now([0.5, 0.5, 0.07], [0.5, 0.5, 0.07], [0.51, 0.49, 0.07])
    assert np.allclose(c, [0.51, 0.49, 0.07])  # 상자가 밀린 만큼 칸도
    assert np.allclose(hg.cell_now([0.5, 0.5, 0.07], [0.5, 0.5, 0.07], [0.7, 0.5, 0.07]), [0.5, 0.5, 0.07])  # 너무 크면 무시


def test_puck_pick_and_neighbour():
    pucks = [(3, np.array([0.18, 0.04, 0.10]), True, None, True), (7, np.array([0.18, -0.04, 0.10]), True, None, False)]
    assert hg.pick_target(pucks, np.array([0.18, -0.03, 0.2])) == 1  # 손에 가까운 것
    assert hg.pick_target(pucks, np.array([0.18, -0.03, 0.2]), tag=3) == 0
    assert hg.pick_target(pucks, np.zeros(3), tag=5) is None
    assert np.allclose(hg.neighbour(pucks, 0), [0.0, -0.08, 0.0])
    assert np.allclose(hg.puck_centre([0, 0, 0.13], True), [0, 0, 0.10])


def test_lying_axis_points_away():
    assert np.allclose(hg.puck_axis(False, math.pi, [0.2, 0.0, 0.08]), [1.0, 0.0, 0.0], atol=1e-9)
    assert np.allclose(hg.puck_axis(True, 0.3, [0.2, 0.0, 0.08]), [0.0, 0.0, 1.0])


if __name__ == "__main__":
    for name, f in list(globals().items()):
        if name.startswith("test_"):
            f()
            print("ok", name)


def test_held_relative_follows_the_hand():
    """마지막 검출(map)은 그 자리 · 손이 내려가면 rel이 그만큼 줄어듦 (시뮬 pick_env: last_m − 지금 손)"""
    base_t, base_R = np.array([1.0, 2.0, 0.0]), rz(math.pi / 2)
    c_map = base_t + base_R @ np.array([0.20, 0.01, 0.10])  # base_link (0.20, 0.01, 0.10)의 퍽 가운데
    held = {"kind": "puck", "c_map": c_map, "standing": True, "yaw_map": math.pi / 2 + 0.1, "is_box": True}
    ee_R = DOWN
    rel0, fy, ax = hg.held_relative(held, base_t, base_R, np.array([0.20, 0.0, 0.20]), ee_R)
    rel1, _, _ = hg.held_relative(held, base_t, base_R, np.array([0.20, 0.0, 0.15]), ee_R)
    assert np.allclose(rel0, [0.0, 0.01, -0.10]) and np.allclose(rel1, [0.0, 0.01, -0.05])
    assert abs(fy - 0.1) < 1e-9 and np.allclose(ax, [0, 0, 1])
    cell = hg.held_relative({"kind": "cell", "c_map": c_map}, base_t, base_R, np.array([0.20, 0.0, 0.20]), ee_R)[0]
    assert np.allclose(cell, np.array([0.20, 0.01, 0.10]) - hg.bottom_in_hand(np.array([0.20, 0.0, 0.20]), ee_R))


def test_aimed_centre_and_slot_puck():
    """pick 손끝 목표 → 겨냥한 퍽 가운데 (시뮬 tgt_insp_b) · unload 칸 안 퍽 − 손 (시뮬 UnloadEnv slot_tf)"""
    bottom = np.array([0.3, 0.1, 0.07])
    assert np.allclose(hg.aimed_centre(bottom + [0, 0, hg.GRASP_Z]), bottom + [0, 0, hg.PUCK_H / 2])
    floor = np.array([-0.10484, 0.0325, 0.09125])
    assert np.allclose(hg.slot_puck_rel(floor, np.array([-0.1, 0.03, 0.2])), [-0.00484, 0.0025, 0.09125 + 0.03 - 0.2])
