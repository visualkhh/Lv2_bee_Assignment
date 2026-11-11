"""가상 미션 — plan_next(다음 skill 고르기) 학습용. 물리 없이 skill 결과를 확률로 흉내.

범위: 계산기(할당)가 끝난 뒤부터 — 수집 → 배치 → 복귀. 관측·조사는 정답이 정해져 있어 계산기 몫.
퍽마다 목표(장소·칸)는 계산기 결과로 주어진다고 보고 무작위 유효 문제로 만든다.

행동 = skill 하나 + 대상 퍽. 시작 조건이 안 맞는 행동은 action_masks()로 막는다 (sb3-contrib MaskablePPO).
보상 = 채점표: 장소별 주사위 눈 +10 (그 장소 퍽 전부 칸 안) · 복귀 +10 − 걸린 시간 조금.
확률(PROBS)·시간(SKILL_S)은 임시값 — skill별 Isaac 평가 결과로 바꿔 넣는다.
"""
import math

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

# 임시 확률 — skill 평가 결과로 바꿈
PROBS = {
    "lying_at_start": 0.15,
    "pick": 0.95, "pick_fail_knock": 0.30, "pick_fail_lost": 0.05,
    "stand": 0.80, "stand_lost": 0.02,
    "load": 0.98, "unload": 0.97,
    "place_ok": 0.93, "place_ok_center_after_neighbours": 0.82,
    "place_off": 0.05, "place_lying": 0.015,  # 나머지 = 상자 밖으로 (LOST)
    "nudge": 0.70, "nudge_knock": 0.10,
    "repick": 0.95,
    "stand_placed": 0.60,
}
# skill 한 번 걸리는 시간 [s] (임시)
SKILL_S = {PICK: 15, STAND: 25, LOAD: 12, UNLOAD: 12, PLACE: 15, NUDGE: 10, REPICK: 15, STAND_PLACED: 25}
SPEED = 0.2      # [m/s] 평균 주행 (도킹 포함 전)
DETOUR = 1.3     # 직선 거리 대비 실제 경로
DOCK_S = 10.0    # 다가가기·물러나기
TIME_LIMIT = 3600.0  # 배터리 — 넘으면 복귀 점수 없음
TIME_COST = 0.001    # 초당 벌점 (시간 제한은 없지만 짧을수록 조금 낫게)
# 진척 보상 (potential-based shaping, Φ 차이) — 점수는 끝에만 나와서 학습이 "바로 복귀"에 갇히는 것을 막음.
# 퍽 상태마다 값 Φ — 끝 점수와 같은 방향(칸에 맞게 놓일수록 큼). 평가는 info["score"]만 봄
PHI = {BOX_UP: 0.0, BOX_LYING: 0.0, HAND: 0.5, BASKET: 1.0, OK: 2.5, OFF: 1.5, PLACED_LYING: 1.0, LOST: -1.0}


def travel_s(a, b):
    if a == b:
        return 0.0
    (x0, y0), (x1, y1) = PLACES[a], PLACES[b]
    return DETOUR * math.hypot(x1 - x0, y1 - y0) / SPEED + DOCK_S


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
            m[b + NUDGE] = s == OFF and hand_empty
            m[b + REPICK] = s == OFF and hand_empty
            m[b + STAND_PLACED] = s == PLACED_LYING and hand_empty
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
