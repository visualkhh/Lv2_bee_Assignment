# Plan 시나리오 — 신호등 암호 미션

> 설계 전제: [mission/README.md](README.md) · [project/ros/00-overview.md](../project/ros/00-overview.md)
> 담당 패키지: `plan` (vlm_task_planner · coordinator · signal_decoder · explorer)

---

## 전제 조건

- 퍽 위치, 신호등 위치, 목적지(L1~L3) 위치는 **모두 랜덤** — 시작 전에 알 수 없음
- 로봇은 경기장 안 임의 위치에서 시작
- 지도(map)는 SLAM(RTAB-Map)으로 실시간 생성
- 수납함(tray): 차체 위 4칸 슬롯 (뒷줄 2 → 앞줄 2 순서로 적재)

---

## Phase 0 — 시스템 부팅

```
bringup 실행
  └→ 모든 노드 기동 (Nav2 · MoveIt2 · RTAB-Map · EKF · controller_manager · perception …)
  └→ coordinator: 팔을 initial_pose로 이동 (MoveIt 없이 JTC 토픽, 3초)
  └→ state_manager: operation_mode = AUTO, map_mode = MAPPING
  └→ perception: 카메라 두 대(pole_front_cam · gripper_cam) 인식 시작
```

---

## Phase 1 — 탐색 (EXPLORE)

```
operation_mode → EXPLORE
  └→ explorer 활성화
       맵 프론티어(미탐색 경계)를 찾아 Nav2로 순서대로 이동
       가는 도중 perception이 물체를 발견하면 /tracked_objects에 실시간 등록:
         traffic_light  → 신호등 위치 등록
         puck (red/green/blue) → 퍽 위치 등록 (번호는 아직 모름)
         location (tag 12~14) → 목적지 위치 등록

  └→ coordinator: EXPLORE 중 팔을 scan_pose로 (2초마다 확인)
       gripper_cam이 뒤쪽 고정 카메라 역할 → 지나가며 SLAM 보조

  └→ 탐색 완료 조건:
       프론티어 없음(DONE) → operation_mode = AUTO 자동 복귀
       또는 신호등·퍽·목적지를 충분히 발견 → 수동으로 AUTO 전환 가능
```

> ⚠️ 탐색 중 신호등을 발견하면 signal_decoder가 자동으로 깜빡임 감시를 시작한다.
> 탐색이 끝나기 전에 신호를 읽어도 무방 — vlm_task_planner가 정보가 부족하면 wait 상태로 기다린다.

---

## Phase 2 — 신호 읽기

```
perception: pole_front_cam → traffic_light 색 감지 → /tracked_objects 갱신
signal_decoder: /tracked_objects에서 traffic_light 상태 구독

  깜빡임 3단계 해석:
    [1단계] 빨강(또는 초록) N번 깜빡임 → color=RED, count=N
    [2단계] 세 색 순서대로 깜빡임 → 각 색 점수 합산 → sum=S
    [3단계] 노랑 N번 깜빡임 → dest_tag = 11+N (1→12, 2→13, 3→14)
    단계 구분 = 세 색 동시(all) 2초 / 꺼짐 3초

  → /mission/signal 발행:
      { color: "red", count: 2, sum: 5, dest_tag: 13, raw: "R R | R Y G G R | Y Y" }
```

---

## Phase 3 — vlm_task_planner 판단

`/mission/signal` 수신 시 vlm_task_planner가 `/tracked_objects` 상태를 보고 판단.

### 케이스 A: 조합 즉시 계산 가능

```
조건: 해당 색 퍽이 보이고 + 그 중 태그 번호를 아는 것들로 합이 맞는 조합 존재
      + 목적지 태그도 이미 발견됨

판단 결과 (kind: carry):
  /task/sub_goals 발행:
    step 1: navigate(퍽 R3 위치)
    step 2: pick(R3)
    step 3: place tray          ← 수납함 칸1에 싣기
    step 4: navigate(퍽 R4 위치)
    step 5: pick(R4)
    step 6: place tray          ← 수납함 칸2에 싣기
    step 7: navigate(목적지 tag=13 위치)
    step 8: pick tray           ← 수납함에서 꺼내기 (마지막 것부터)
    step 9: place(목적지 id)
    step 10: pick tray
    step 11: place(목적지 id)
```

### 케이스 B: 퍽 번호를 모름 (태그 미확인)

```
조건: 해당 색 퍽이 보이지만 태그 번호 미확인 상태

판단 결과 (kind: inspect):
  /task/sub_goals 발행:
    step 1: navigate(퍽 A 위치)
    step 2: inspect(퍽 A)       ← 팔을 퍽 윗면 0.15m 위로 → gripper_cam이 16mm 태그 읽기
    step 3: navigate(퍽 B 위치)
    step 4: inspect(퍽 B)
    ...

  inspector 완료 → /task/status SUCCEEDED
  → vlm_task_planner 재판단 (케이스 A 또는 C)
```

### 케이스 C: 목적지 태그 미발견

```
조건: mission_signal은 받았으나 dest_tag(예: 13번)를 아직 못 봄

판단 결과 (kind: wait):
  /task/sub_goals 발행 안 함
  explorer가 계속 탐색
  → location tag=13 발견 시 vlm_task_planner 자동 재판단
```

### 케이스 D: 조합 불가능 (합이 맞는 퍽 없음)

```
조건: 번호 아는 퍽 전부 확인했는데 합이 sum이 되는 count개 조합 없음

판단 결과 (kind: fail):
  say("조합을 찾을 수 없어요") → 사람 개입 대기
```

---

## Phase 4 — coordinator 실행

`/task/sub_goals`를 받아 단계별 순서대로 실행.

```
[navigate 단계]
  대상이 reach_m(0.30m) 밖이면 approach_m(0.24m) 앞까지 먼저 이동
  Nav2 → /navigate_to_pose 액션

[pick 단계]
  grasp_scorer → 파지 후보 점수 정렬
  MoveIt2 + MTC → 접근 → 그리퍼 열기 → 파지 → attach → 들어올리기
  실패 시 max_grasp_tries(2)까지 재시도

[place tray 단계]
  수납함 다음 빈 칸(tray_slots 순서)의 tray_release_z(0.22m)에서 그리퍼 열기
  tray_count++ (최대 4칸)

[pick tray 단계]
  수납함 마지막 칸의 tray_grasp_z(0.17m)에서 파지
  tray_count--

[place 목적지 단계]
  같은 목적지에 여러 개 놓을 때 place_spacing(0.08m) 간격으로 비켜서 배치

[inspect 단계]
  navigate로 퍽 앞 접근
  팔을 inspect_height_m(0.15m)으로 내림
  inspect_wait_s(1.0초) 대기 → gripper_cam이 태그 읽기
  perception이 /tracked_objects의 details.tag 갱신

[실패 처리]
  같은 단계 max_step_failures(2)회 연속 실패
  → /robot/say 알림 + 정지
  → vlm_task_planner가 FAILED 수신 → max_replans(1)회 재계획
```

---

## Phase 5 — 수납 확인

> 여기서 "수납 확인"은 **목적지(L1~L3)에 퍽이 제대로 놓였는지** 확인하는 것 (수납함 tray와 다름).

```
carry 계획 SUCCEEDED 직후:
  vlm_task_planner → 목적지로 navigate
  gripper_cam + pole_front_cam으로 근처 퍽 번호 확인
  verify_radius_m(0.20m) 안에 옮긴 번호가 다 있으면 → 완료 ✅

  없으면:
    다른 곳에서 발견 → recarry (다시 옮기기 계획)
    아예 안 보임     → say("퍽을 찾을 수 없어요")

이후 verify_period_s(60초)마다 반복 확인
  (AUTO 모드 + 다른 계획 없을 때만)
  새 mission_signal 또는 user_command가 오면 확인 중단
```

---

## 전체 타임라인 예시

> 예시: 빨간 퍽 2개, 번호 합 5, 목적지 L2(tag=13)

```
T+0s    부팅 · 팔 초기 자세
T+5s    EXPLORE 시작 → 맵 탐색
T+30s   신호등 발견 → signal_decoder 감시 시작
T+45s   퍽 6개 발견 (번호 미확인) · 목적지 L2 발견
T+60s   신호 읽기 완료 → mission_signal { RED, 2, 5, 13 }
T+61s   vlm_task_planner: 번호 모름 → inspect 계획 발행
T+62s   coordinator: R1 inspect → R2 inspect → R3 inspect (번호 0,1,2 확인)
T+80s   vlm_task_planner 재판단: R3(2)+R4(3)=5 조합 발견 → carry 계획 발행
T+81s   coordinator: navigate R3 → pick → tray
T+95s   coordinator: navigate R4 → pick → tray
T+110s  coordinator: navigate L2 → pick tray → place → pick tray → place
T+125s  수납 확인: L2 근처 tag 2,3 확인 → 완료 ✅
T+185s  60초 후 재확인 → 이상 없음 ✅
```

---

## 비상 시나리오

| 상황 | 처리 |
|------|------|
| 신호 못 읽음 (깜빡임 놓침) | signal_decoder가 raw 그대로 `/mission/signal` — vlm_task_planner가 모호하면 say + 대기 |
| 퍽 집기 2회 실패 | coordinator → say("퍽을 잡을 수 없어요") → vlm_task_planner 재계획 1회 (다른 퍽으로 교체) |
| 수납함 꽉 참 | tray_full 실패 → vlm_task_planner가 목적지 먼저 가서 내려놓는 계획으로 재수립 |
| 목적지 끝내 못 찾음 | kind: wait → explorer 재탐색 → 없으면 say("목적지를 찾을 수 없어요") |
| 비상정지 `/cmd/e_stop` | coordinator 즉시 모든 goal 취소 · 팔 정지 · 재개 대기 |
| 수동 전환 `/cmd/operation_mode MANUAL` | coordinator 작업 일시 중단 (manual_override) · AUTO 복귀 시 새 명령 대기 |
