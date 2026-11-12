"""기준선 — 지금 mission_manager 규칙을 가상 미션에서 그대로 흉내 (plan_next가 이것보다 좋아야 채택).

- 수집: 시작 박스 순서대로 퍽마다 (누웠으면 세우고) 집어 적재함에 — 퍽마다 retries(2)번까지 다시
- 배치: L1 → L2 → L3, 장소 안에서는 dice_cells 순서(가운데 먼저) — 적재함에서 꺼내 놓고,
  벗어났으면 다시 집어 놓기, 누웠으면 세우기 (밀어 맞추기는 지금 규칙에 없음)
- 그 뒤 복귀
"""
from mission_sim import (BASKET, BOX_LYING, BOX_UP, HAND, LOAD, N_PER_PUCK, OFF, PICK, PLACE, PLACED_LYING, REPICK,
                         RETURN, STAND, STAND_PLACED, UNLOAD)

DICE_ORDER = {1: [4], 2: [0, 8], 3: [4, 0, 8], 4: [0, 2, 6, 8], 5: [4, 0, 2, 6, 8], 6: [0, 3, 6, 2, 5, 8]}  # mission_solver dice_cells
COLLECT_TRIES = 3   # 1번 + retries 2번
PUCK_TRIES = 10     # 퍽 하나에 쓰는 skill 횟수 한도 (무한 반복 방지)


def baseline_action(env):
    st, tries, m = env.status, env.tries, env.action_masks()
    a = lambda i, s: i * N_PER_PUCK + s
    collecting = any(s in (BOX_UP, BOX_LYING) and tries[i] < COLLECT_TRIES for i, s in enumerate(st))
    if HAND in st:
        i = st.index(HAND)
        if collecting and m[a(i, LOAD)]:
            return a(i, LOAD)
        return a(i, PLACE)
    if collecting:
        for i, s in enumerate(st):
            if s in (BOX_UP, BOX_LYING) and tries[i] < COLLECT_TRIES:
                return a(i, STAND if s == BOX_LYING else PICK)
    for loc in range(3):
        pucks = [i for i, (l, _) in enumerate(env.targets) if l == loc]
        order = DICE_ORDER[len(pucks)]
        for i in sorted(pucks, key=lambda i: order.index(env.targets[i][1])):
            if tries[i] >= PUCK_TRIES:
                continue
            for s, skill in ((BASKET, UNLOAD), (BOX_UP, PICK), (BOX_LYING, STAND), (OFF, REPICK), (PLACED_LYING, STAND_PLACED)):
                if st[i] == s and m[a(i, skill)]:
                    return a(i, skill)
    return RETURN
