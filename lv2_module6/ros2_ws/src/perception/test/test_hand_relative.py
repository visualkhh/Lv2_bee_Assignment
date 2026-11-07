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
