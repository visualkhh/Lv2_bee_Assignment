"""계산기 플래너 — 학습 없이 최단 순서를 계산해서 다음 skill을 고름 (plan_next와 바꿔 끼울 수 있음).

매 걸음 지금 상태에서 다시 계산 (실패가 나면 남은 일로 다시 풂):
- 들를 곳 = 남은 퍽이 있는 시작 박스 + 아직 다 못 놓은 장소. 장소는 그 장소 퍽을 다 실은 뒤에만 들름
- 남은 곳을 모두 돌고 출발 구역으로 돌아오는 경로 중 가장 짧은 것을 정확히 구함 (부분집합 DP, 들를 곳 최대 9개)
- 그 경로의 첫 곳에서 할 skill을 냄. 퍽 하나라도 잃은 장소는 점수가 안 나오므로 건너뜀
skill 성공률은 보지 않음 (모두 성공한다고 보고 거리만 최소) — 확률까지 따지는 것은 plan_next 몫.
"""
from functools import lru_cache

from baseline import DICE_ORDER, PUCK_TRIES
from mission_sim import (BASKET, BOX_LYING, BOX_UP, HAND, HOME_I, LOAD, LOST, N_PER_PUCK, OFF, OK, PICK, PLACE,
                         PLACED_LYING, REPICK, RETURN, STAND, STAND_PLACED, UNLOAD, travel_s)

N_STOP = 9  # 0~5 시작 박스 · 6~8 장소 (mission_sim PLACES 번호와 같음)


@lru_cache(maxsize=500_000)
def _cost(left, node, need):
    """left(들를 곳 비트)를 다 돌고 복귀하는 최소 이동 시간. need[L] = 장소 L 전에 끝나야 할 박스 비트"""
    if not left:
        return travel_s(node, HOME_I)
    best = float("inf")
    for s in range(N_STOP):
        if left >> s & 1 and not (s >= 6 and left & need[s - 6]):
            best = min(best, travel_s(node, s) + _cost(left & ~(1 << s), s, need))
    return best


def _doomed(env, loc):
    for i, (l, _) in enumerate(env.targets):
        if l != loc:
            continue
        s, t = env.status[i], env.tries[i]
        if s == LOST or (s != OK and t >= PUCK_TRIES):  # 기준선과 같은 한도 (기준선도 배치 때 박스 퍽을 다시 집음)
            return True
    return False


def solver_action(env):
    st, tries, m = env.status, env.tries, env.action_masks()
    a = lambda i, s: i * N_PER_PUCK + s
    live = [loc for loc in range(3) if not _doomed(env, loc)]
    left, need = 0, [0, 0, 0]
    for i, (loc, _) in enumerate(env.targets):
        if loc not in live:
            continue
        if st[i] in (BOX_UP, BOX_LYING):
            left |= 1 << (i // 2)
            need[loc] |= 1 << (i // 2)
        if st[i] != OK:
            left |= 1 << (6 + loc)
    need = tuple(need)

    def score(s):  # 지금 자리 → s → 나머지 최단. 지금 자리를 같은 값이면 먼저
        return travel_s(env.at, s) + _cost(left & ~(1 << s), s, need), s != env.at

    nxt = min((s for s in range(N_STOP) if left >> s & 1 and not (s >= 6 and left & need[s - 6])),
              key=score, default=None)

    if HAND in st:  # 손에 든 퍽: 다음 들를 곳이 그 퍽 장소면 놓고, 아니면 적재함에
        i = st.index(HAND)
        if nxt == 6 + env.targets[i][0] or not m[a(i, LOAD)]:
            return a(i, PLACE)
        return a(i, LOAD)
    if nxt is None:
        return RETURN
    if nxt < 6:
        for i in (2 * nxt, 2 * nxt + 1):
            if st[i] in (BOX_UP, BOX_LYING) and env.targets[i][0] in live:
                return a(i, STAND if st[i] == BOX_LYING else PICK)
    loc = nxt - 6
    pucks = [i for i, (l, _) in enumerate(env.targets) if l == loc]
    order = DICE_ORDER[len(pucks)]
    for i in sorted(pucks, key=lambda i: order.index(env.targets[i][1])):
        for s, skill in ((BASKET, UNLOAD), (OFF, REPICK), (PLACED_LYING, STAND_PLACED)):
            if st[i] == s and m[a(i, skill)]:
                return a(i, skill)
    return RETURN  # ponytail: 여기 오면 남은 일을 못 하는 상태 — 지금 가상 미션에선 안 생김 (테스트로 확인)
