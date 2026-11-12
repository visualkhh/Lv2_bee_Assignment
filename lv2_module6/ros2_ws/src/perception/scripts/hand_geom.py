"""hand_relative 계산 (ROS 없음, test/test_hand_relative.py) — 시뮬 학습 정책이 보던 손 카메라 상대값을 로봇에서 만든다.

시뮬 정책 입력 (isaac/lab, 브랜치 feature/skill-envs) 과 같은 정의 · 모두 base_link 좌표 [m]:
  puck : 퍽 가운데 − 손끝(end_effector_link)          pick_env · slot_env.UnloadEnv · right_env (cam = mid − ee)
  cell : 칸 가운데(상자 윗면) − 손 안 퍽 바닥           place_env · pip_env (cam = cell − puck bottom)
  slot : 적재함 칸 가운데 − 손 안 퍽 바닥               slot_env.LoadEnv
  face_yaw : 사각 퍽 면 방향 − 손 방향, ±45°로 감음     pick_env (_face_err)
손 안 퍽 바닥 = 손끝 + R_손 · (0, 0, GRASP_Z) — 손 카메라는 손 안 퍽 밑면을 못 봄 → 정상 파지 위치로 둠
  (ponytail: 시뮬 정책은 손 안 퍽 위치 오차 ±2~3 mm까지 본 값으로 학습, 여기선 0 — 손 안 오차는 보정 못 함)
"""
import math

import numpy as np

GRASP_Z = 0.06 - 0.020 + 0.0206  # 퍽 바닥이 손끝 아래 이만큼 (isaac/rl/bee_isaac/common.py GRASP_Z: PUCK_H − GRIP_DEPTH + FINGER_TIP)
PUCK_H = 0.06


def quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def wrap(a, period):
    return (a + period / 2) % period - period / 2


def to_base(p_map, base_t, base_R):
    """map 점 → base_link (base_t, base_R = map에서 본 base_link 위치·회전)."""
    return base_R.T @ (np.asarray(p_map, float) - base_t)


def tool_yaw(ee_R):
    """손 방향 yaw (손끝 z가 아래일 때 손 x축의 바닥 위 방향) — 시뮬 rot = Rz(yaw) · DOWN 의 yaw."""
    return math.atan2(ee_R[1, 0], ee_R[0, 0])


def bottom_in_hand(ee_t, ee_R):
    """손 안 퍽 바닥 (base_link) — 정상 파지."""
    return ee_t + ee_R @ np.array([0.0, 0.0, GRASP_Z])


def puck_centre(p, standing, seen_from_top=True):
    """검출 위치 → 퍽 가운데 (반 높이). 손 카메라가 위에서 본 서 있는 퍽은 윗면 점이라 반 높이 내림."""
    p = np.asarray(p, float).copy()
    if standing and seen_from_top:
        p[2] -= PUCK_H / 2
    return p


def face_yaw(puck_yaw_b, tool_yaw_b, is_box):
    """사각 퍽 면 − 손 방향 (±45°) · 원기둥·방향 모름은 0."""
    if not is_box or puck_yaw_b is None:
        return 0.0
    return wrap(puck_yaw_b - tool_yaw_b, math.pi / 2)


def puck_axis(standing, yaw_b, centre_b):
    """퍽 축 (아래 → 위, 누운 퍽은 가까운 끝 → 먼 끝) — right_env _axis와 같은 뜻. 누운 퍽 yaw는 180° 대칭 → 로봇에서 먼 쪽."""
    if standing or yaw_b is None:
        return np.array([0.0, 0.0, 1.0])
    a = np.array([math.cos(yaw_b), math.sin(yaw_b), 0.0])
    return a if a[:2] @ np.asarray(centre_b, float)[:2] >= 0 else -a


def cell_now(point_map, box_plan_map, box_seen_map, max_shift=0.05):
    """계획 때 칸 가운데 + (지금 본 상자 − 계획 때 상자) — 상자가 밀렸으면 따라감 (mission_solver tag_correction과 같은 한도)."""
    p = np.asarray(point_map, float)
    if box_seen_map is None or box_plan_map is None:
        return p
    d = np.asarray(box_seen_map, float)[:2] - np.asarray(box_plan_map, float)[:2]
    if np.hypot(*d) > max_shift:
        return p
    return p + np.array([d[0], d[1], 0.0])


def pick_target(pucks, ee_t, tag=-1):
    """pucks: [(tag, centre_b, ...)] → 목표 퍽 번호 (tag가 있으면 그 번호, 없으면 손에 가장 가까운 것) · 없으면 None."""
    if not pucks:
        return None
    if tag >= 0:
        for k, pk in enumerate(pucks):
            if pk[0] == tag:
                return k
        return None
    return min(range(len(pucks)), key=lambda k: np.hypot(*(np.asarray(pucks[k][1])[:2] - ee_t[:2])))


def neighbour(pucks, k, radius=0.12):
    """목표 퍽 k 옆(같은 상자, radius 안) 가장 가까운 다른 퍽 − 목표 [m] · 없으면 None."""
    best = None
    for j, pk in enumerate(pucks):
        if j == k:
            continue
        d = np.asarray(pk[1], float) - np.asarray(pucks[k][1], float)
        if np.hypot(*d[:2]) < radius and (best is None or np.hypot(*d[:2]) < np.hypot(*best[:2])):
            best = d
    return best


def held_relative(held, base_t, base_R, ee_t, ee_R):
    """Last detection (map frame) seen from the hand NOW → (rel, face_yaw, axis) in base_link — the sim policy input:
    the measured puck / cell stays where it was seen, the hand keeps moving (isaac pick_env: last_m − current hand).
    held: kind 'puck' (c_map = puck centre, standing, yaw_map or None, is_box) or 'cell' (c_map = cell point)."""
    c_b = to_base(held["c_map"], base_t, base_R)
    if held["kind"] == "cell":
        return c_b - bottom_in_hand(ee_t, ee_R), 0.0, np.array([0.0, 0.0, 1.0])
    base_yaw = math.atan2(base_R[1, 0], base_R[0, 0])
    yaw_b = None if held["yaw_map"] is None else wrap(held["yaw_map"] - base_yaw, 2 * math.pi)
    return (c_b - ee_t, float(face_yaw(yaw_b, tool_yaw(ee_R), held["is_box"])),
            puck_axis(held["standing"], yaw_b, c_b))


def aimed_centre(grasp_ee_map):
    """pick_skill 손끝 목표(서 있는 퍽을 위에서 집는 자세) → 계획이 겨냥한 퍽 가운데 (map) — 시뮬 pick_env tgt_insp_b."""
    return np.asarray(grasp_ee_map, float) - np.array([0.0, 0.0, GRASP_Z - PUCK_H / 2])


def slot_puck_rel(slot_floor_b, ee_t):
    """unload: 칸 안 퍽 공칭 가운데 (칸 바닥 + 반 높이) − 손끝 (base_link) — 시뮬 UnloadEnv slot_tf."""
    return np.asarray(slot_floor_b, float) + np.array([0.0, 0.0, PUCK_H / 2]) - ee_t
