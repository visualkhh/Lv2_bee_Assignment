"""가상 미션 — plan_next(다음 skill 고르기) 학습용. 물리 없이 skill 결과를 확률로 흉내.

범위: 계산기(할당)가 끝난 뒤부터 — 수집 → 배치 → 복귀. 관측·조사는 정답이 정해져 있어 계산기 몫.
퍽마다 목표(장소·칸)는 계산기 결과로 주어진다고 보고 무작위 유효 문제로 만든다.

행동 = skill 하나 + 대상 퍽. 시작 조건이 안 맞는 행동은 action_masks()로 막는다 (sb3-contrib MaskablePPO).
보상 = 채점표: 장소별 주사위 눈 +10 (그 장소 퍽 전부 칸 안) · 복귀 +10 − 걸린 시간 조금.
확률(PROBS)·시간(SKILL_S)은 지금 로봇 기준으로 맞춤 (11-13) — 값마다 출처 주석. "임시"는 잰 값이 없는 것.
로봇에 없는 skill(밀어 맞추기 NUDGE · 놓인 퍽 세우기 STAND_PLACED)은 막음 (C++ plan_next.hpp mask와 같게).
"""

import gymnasium as gym
import numpy as np

# 경기장 좌표 [m] (documents/mission/README.md)
BOXES = [(1.468, 0.950), (3.670, 3.152), (2.936, 5.354), (0.734, 5.354), (2.202, 5.354), (3.670, 3.886)]
LOCS = [(3.670, 0.950), (2.202, 3.700), (3.670, 5.354)]
HOME = (0.734, 0.950)
PLACES = BOXES + LOCS + [HOME]  # 로봇이 있을 수 있는 자리: 0~5 시작 박스 · 6~8 장소 · 9 출발 구역
HOME_I = 9

# 주사위 눈 → 칸 (3×3, 4 = 가운데)
PIPS = {1: [4], 2: [0, 8], 3: [0, 4, 8], 4: [0, 2, 6, 8], 5: [0, 2, 4, 6, 8], 6: [0, 3, 6, 2, 5, 8]}
CENTER = 4

# 퍽 상태
BOX_UP, BOX_LYING, HAND, BASKET, OK, OFF, PLACED_LYING, LOST = range(8)
N_STATUS = 8

# 행동 (퍽마다 8개 + 공통 1개)
PICK, STAND, LOAD, UNLOAD, PLACE, NUDGE, REPICK, STAND_PLACED = range(8)
N_PER_PUCK = 8
N_PUCK = 12
RETURN = N_PUCK * N_PER_PUCK  # 마지막 행동: 출발 구역으로 복귀하고 끝

# skill 성공 확률 — 출처: Isaac = models/skills/*/meta.yaml sim_policy (1024판) · 실기 = coordinator 로그 (11월, 단일 skill 시험)
#   둘 다 있으면 낮은 쪽. 실기 미션은 아직 없어서 전부 단일 skill 기준 (연속 실행에서 더 떨어질 수 있음)
PROBS = {
    "lying_at_start": 0.15,  # 임시 — 시작 때 누운 퍽 비율 (잰 값 없음)
    # pick: Isaac pick_standing_puck 100 % · 실기 30/35 중 쥐기 실패(grasp_failed_all 2 · arm_contact 1)만 보면 30/33 ≈ 0.91
    #   (nav_failed는 주행 몫 · pick_unavailable(팔 안 닿음)은 #192부터 pick_far로 다시 하므로 뺌) → 낮은 쪽 0.90
    "pick": 0.90, "pick_fail_knock": 0.30, "pick_fail_lost": 0.05,  # 실패 중 넘어뜨림·잃음 비율은 임시
    # stand: Isaac regrasp_after_stand 78.2 % (로봇은 아직 안 씀) · 실기 52/271 (튜닝 중 기록이라 대표값 아님) → 가운데 0.5
    #   불확실해서 train_plan.py eval --stand 0.3 / 0.8로 민감도 봄
    "stand": 0.5, "stand_lost": 0.016,  # stand_lost: Isaac regrasp_after_stand irreversible 1.6 %
    "load": 0.99,    # Isaac load_slot 99.0 % (실기 없음)
    "unload": 0.981,  # Isaac unload_slot 98.1 % (실기 없음)
    "place_ok": 0.99,  # Isaac place_cell 99.0 % (실기 없음)
    "place_ok_center_after_neighbours": 0.82,  # 이웃 칸 뒤 가운데 81~83 % (documents/project/ros/03-plan.md, isaac-pick)
    "place_off": 0.007, "place_lying": 0.002,  # 임시 — 실패 1 %를 벗어남·누움·밖으로 나눔 (나머지 = 상자 밖으로, LOST)
    "nudge": 0.70, "nudge_knock": 0.10,  # 로봇에 없음 (막힘) — 값은 쓰이지 않음
    "repick": 0.90,  # 놓인 퍽 다시 집기 = pick과 같게
    "stand_placed": 0.60,  # 로봇에 없음 (막힘)
}
ROBOT_SKILLS_ONLY = True  # NUDGE · STAND_PLACED 막기 (plan_next.hpp mask와 같음)
# skill 한 번 걸리는 시간 [s] — 임시 (실기 로그 시간은 시험 중 멈춤·재시도가 섞여 못 씀 · 실기 미션에서 재야 함)
SKILL_S = {PICK: 15, STAND: 25, LOAD: 12, UNLOAD: 12, PLACE: 15, NUDGE: 10, REPICK: 15, STAND_PLACED: 25}
# 주행: 벽을 피한 경로 길이 [m] — tools/plan/route_order.py 격자 A* (벽·박스를 로봇 반경 0.16 + 여유 0.05로 넓힘,
#   다가설 면 = 출발에서 가장 가까운 면). PLACES 번호 순 (0~5 박스 · 6~8 장소 · 9 출발). 다시 만들려면 route_order.py의 E를
#   [1..6, 7..9, 0] 순으로 뽑음. 직선 × 1.3 대신 (벽 돌아가기 반영)
PATH_M = [
    [0, 3.61, 5.14, 5.30, 5.45, 4.18, 1.99, 3.47, 5.43, 1.16],
    [3.61, 0, 5.51, 5.93, 6.07, 4.56, 3.57, 4.10, 5.80, 4.77],
    [5.14, 5.51, 0, 2.41, 2.07, 1.24, 5.10, 2.31, 0.91, 5.20],
    [5.30, 5.93, 2.41, 0, 1.45, 3.01, 5.52, 1.83, 3.11, 4.55],
    [5.45, 6.07, 2.07, 1.45, 0, 2.67, 5.66, 1.97, 2.77, 4.86],
    [4.18, 4.56, 1.24, 3.01, 2.67, 0, 4.15, 2.42, 1.45, 5.21],
    [1.99, 3.57, 5.10, 5.52, 5.66, 4.15, 0, 3.68, 5.39, 3.15],
    [3.47, 4.10, 2.31, 1.83, 1.97, 2.42, 3.68, 0, 3.01, 2.91],
    [5.43, 5.80, 0.91, 3.11, 2.77, 1.45, 5.39, 3.01, 0, 5.90],
    [1.16, 4.77, 5.20, 4.55, 4.86, 5.21, 3.15, 2.91, 5.90, 0],
]
# 평균 주행 속도 [m/s] — 추정: Nav2 상한 0.26 (nav2_params.yaml MPPI vx_max · DWB max_vel_x) · RPP desired_linear_vel 0.20,
#   출발·도착 제자리 회전과 감속을 넣어 0.15 (실기 미션 주행 시간을 재면 바꿈)
SPEED = 0.15
DOCK_S = 10.0    # 다가가기·물러나기 — 임시 유지 (approach 0.165 m를 ≤ 0.06~0.12 m/s + 돌기 · 물러나기 → 대략 10 s)
TIME_LIMIT = 3600.0  # 배터리 — 넘으면 복귀 점수 없음
TIME_COST = 0.001    # 초당 벌점 (시간 제한은 없지만 짧을수록 조금 낫게)
# 진척 보상 (potential-based shaping, Φ 차이) — 점수는 끝에만 나와서 학습이 "바로 복귀"에 갇히는 것을 막음.
# 퍽 상태마다 값 Φ — 끝 점수와 같은 방향(칸에 맞게 놓일수록 큼). 평가는 info["score"]만 봄
PHI = {BOX_UP: 0.0, BOX_LYING: 0.0, HAND: 0.5, BASKET: 1.0, OK: 2.5, OFF: 1.5, PLACED_LYING: 1.0, LOST: -1.0}


def travel_s(a, b):
    return 0.0 if a == b else PATH_M[a][b] / SPEED + DOCK_S


def random_problem(rng):
    """R·Y·G 횟수(각 1~6, 합 12)를 장소에 배정하고 퍽 12개에 (장소, 칸) 목표를 준다 — 계산기 결과를 흉내."""
    while True:
        c = rng.integers(1, 7, size=2)
        if 1 <= 12 - c.sum() <= 6:
            counts = [int(c[0]), int(c[1]), int(12 - c.sum())]
            break
    rng.shuffle(counts)
    targets = [(loc, cell) for loc, n in enumerate(counts) for cell in PIPS[n]]
    rng.shuffle(targets)
    return targets  # targets[puck] = (장소, 칸); 퍽 i는 시작 박스 i // 2


class MissionSim(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, probs=None, seed=None, shaping=True):
        self.p = dict(PROBS, **(probs or {}))
        self.shaping = shaping
        self.observation_space = gym.spaces.Box(0.0, 1.0, (N_PUCK * (N_STATUS + 3 + 1 + 1) + len(PLACES) + 1,), np.float32)
        self.action_space = gym.spaces.Discrete(RETURN + 1)
        self._rng = np.random.default_rng(seed)

    # ---------- gym
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        r = self._rng
        self.targets = random_problem(r)
        self.status = [BOX_LYING if r.random() < self.p["lying_at_start"] else BOX_UP for _ in range(N_PUCK)]
        self.tries = [0] * N_PUCK
        self.at = HOME_I
        self.t = 0.0
        self.returned = False
        return self._obs(), {}

    def step(self, action):
        t0 = self.t
        if not self.action_masks()[action]:  # 막힌 행동 — 아무 일 없이 1초 (마스크를 쓰면 일어나지 않음)
            self.t += 1.0
            return self._obs(), -TIME_COST, False, self.t > TIME_LIMIT, {"score": self.score(), "time": self.t}
        if action == RETURN:
            self.t += travel_s(self.at, HOME_I)
            self.at = HOME_I
            self.returned = self.t <= TIME_LIMIT
            score = self.score()
            return self._obs(), score - TIME_COST * (self.t - t0), True, False, {"score": score, "time": self.t}
        i, skill = divmod(action, N_PER_PUCK)
        phi0 = self._phi()
        self._do(i, skill)
        truncated = self.t > TIME_LIMIT
        reward = -TIME_COST * (self.t - t0) + self._phi() - phi0
        info = {}
        if truncated:  # 배터리 끝 — 그 자리에서 멈춤, 복귀 점수 없음
            info = {"score": self.score(), "time": self.t}
            reward += info["score"]
        return self._obs(), reward, False, truncated, info

    def action_masks(self):
        m = np.zeros(RETURN + 1, bool)
        hand_empty = HAND not in self.status
        in_basket = self.status.count(BASKET)
        for i, s in enumerate(self.status):
            b = i * N_PER_PUCK
            m[b + PICK] = s == BOX_UP and hand_empty
            m[b + STAND] = s == BOX_LYING and hand_empty
            m[b + LOAD] = s == HAND and in_basket < 12
            m[b + UNLOAD] = s == BASKET and hand_empty
            m[b + PLACE] = s == HAND
            m[b + NUDGE] = s == OFF and hand_empty and not ROBOT_SKILLS_ONLY
            m[b + REPICK] = s == OFF and hand_empty
            m[b + STAND_PLACED] = s == PLACED_LYING and hand_empty and not ROBOT_SKILLS_ONLY
        m[RETURN] = True
        return m

    def _phi(self):
        return sum(PHI[s] for s in self.status) if self.shaping else 0.0

    # ---------- 점수 (채점표의 주사위 눈 · 복귀 부분)
    def score(self):
        s = 0.0
        for loc in range(3):
            pucks = [i for i, (l, _) in enumerate(self.targets) if l == loc]
            if all(self.status[i] == OK for i in pucks):
                s += 10.0
        return s + (10.0 if self.returned else 0.0)

    # ---------- skill 결과 (확률)
    def _go(self, place):
        self.t += travel_s(self.at, place)
        self.at = place

    def _do(self, i, skill):
        r, p = self._rng.random(), self.p
        self.tries[i] += 1
        loc, cell = self.targets[i]
        if skill in (PICK, STAND):
            self._go(i // 2)
        elif skill in (PLACE, NUDGE, REPICK, STAND_PLACED):
            self._go(6 + loc)
        self.t += SKILL_S[skill]
        st = self.status
        if skill == PICK:
            fail = 1 - p["pick"]
            if r < p["pick"]:
                st[i] = HAND
            elif r < p["pick"] + fail * p["pick_fail_knock"]:
                st[i] = BOX_LYING
            elif r < p["pick"] + fail * (p["pick_fail_knock"] + p["pick_fail_lost"]):
                st[i] = LOST
        elif skill == STAND:
            if r < p["stand"]:
                st[i] = BOX_UP
            elif r < p["stand"] + p["stand_lost"]:
                st[i] = LOST
        elif skill == LOAD:
            st[i] = BASKET if r < p["load"] else HAND
        elif skill == UNLOAD:
            st[i] = HAND if r < p["unload"] else BASKET
        elif skill == PLACE:
            ok = p["place_ok"]
            if cell == CENTER and self._neighbours_placed(loc):
                ok = p["place_ok_center_after_neighbours"]
            if r < ok:
                st[i] = OK
            elif r < ok + p["place_off"]:
                st[i] = OFF
            elif r < ok + p["place_off"] + p["place_lying"]:
                st[i] = PLACED_LYING
            else:
                st[i] = LOST
        elif skill == NUDGE:
            if r < p["nudge"]:
                st[i] = OK
            elif r < p["nudge"] + p["nudge_knock"]:
                st[i] = PLACED_LYING
        elif skill == REPICK:
            if r < p["repick"]:
                st[i] = HAND
        elif skill == STAND_PLACED:
            if r < p["stand_placed"]:
                st[i] = OFF  # 섰지만 칸 중심은 다시 맞춰야 함

    def _neighbours_placed(self, loc):
        return any(self.status[j] in (OK, OFF, PLACED_LYING) and l == loc and c != CENTER
                   for j, (l, c) in enumerate(self.targets))

    def _obs(self):
        o = []
        for i in range(N_PUCK):
            oh = [0.0] * N_STATUS
            oh[self.status[i]] = 1.0
            loc, cell = self.targets[i]
            o += oh + [float(loc == k) for k in range(3)] + [float(cell == CENTER), min(self.tries[i], 5) / 5]
        o += [float(self.at == k) for k in range(len(PLACES))]
        o.append(min(self.t / TIME_LIMIT, 1.0))
        return np.asarray(o, np.float32)
