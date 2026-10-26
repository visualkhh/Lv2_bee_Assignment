# Phase 4 — coordinator 실행

> 목적: `/task/sub_goals`를 받아 navigate → pick → tray → place 순서로 물리적으로 실행.

## 단계별 실행 흐름

```mermaid
flowchart TD
    IN["/task/sub_goals 수신"]
    VALIDATE{"계획 검증\n허용 skill?\ntarget id 존재?"}
    REJECT["FAILED\ninvalid_plan"]

    IN --> VALIDATE
    VALIDATE -->|"실패"| REJECT
    VALIDATE -->|"통과"| STEP

    subgraph STEP["단계 순차 실행 (idx 0 → N)"]
        direction TB
        S_NAV["navigate\nNav2 이동"]
        S_PICK["pick\ngrasp_scorer → MoveIt2"]
        S_TRAY_IN["place tray\n수납함에 싣기"]
        S_TRAY_OUT["pick tray\n수납함에서 꺼내기"]
        S_PLACE["place 목적지\n내려놓기"]
        S_INSPECT["inspect\n팔 내려 태그 읽기"]
        S_SAY["say\n음성 알림"]
    end

    NEXT{"다음 단계\n있음?"}
    DONE["SUCCEEDED"]

    STEP --> NEXT
    NEXT -->|"YES"| STEP
    NEXT -->|"NO"| DONE
```

---

## navigate 상세

```mermaid
flowchart LR
    G["navigate(target_id)"]
    FAR{"target이\nreach_m(0.30m)\n밖?"}
    APP["approach_m(0.24m)\n앞까지 먼저 이동\n(물체 바라보는 방향)"]
    NAV["Nav2\n/navigate_to_pose"]
    OK["도착"]

    G --> FAR
    FAR -->|"YES"| APP --> NAV --> OK
    FAR -->|"NO"| NAV --> OK
```

---

## pick 상세

```mermaid
flowchart TD
    P["pick(puck_id)"]
    GS["grasp_scorer\n파지 후보 점수 정렬"]
    TRY["MoveIt2 + MTC\n접근 → 그리퍼 열기\n→ 파지 → attach\n→ 들어올리기"]
    OK_P["pick 성공\n→ 다음 단계"]
    RETRY{"재시도\nmax_grasp_tries\n(2회) 초과?"}
    FAIL_P["FAILED\n→ vlm 재계획"]

    P --> GS --> TRY
    TRY -->|"성공"| OK_P
    TRY -->|"실패"| RETRY
    RETRY -->|"NO → 다음 후보"| TRY
    RETRY -->|"YES"| FAIL_P
```

---

## 수납함(tray) 상세

```
수납함 슬롯 (base_footprint 기준):
  칸1: x=-0.135, y=+0.05  ← 뒷줄 왼쪽 (첫 번째 적재)
  칸2: x=-0.135, y=-0.05  ← 뒷줄 오른쪽
  칸3: x=-0.095, y=+0.05  ← 앞줄 왼쪽
  칸4: x=-0.095, y=-0.05  ← 앞줄 오른쪽 (네 번째 적재)

싣기 높이:  tray_release_z = 0.22m (손끝 기준)
꺼내기 높이: tray_grasp_z  = 0.17m
```

```mermaid
flowchart LR
    subgraph LOAD["place tray (싣기)"]
        L1["다음 빈 칸 확인\ntray_count < 4?"]
        L2["해당 칸 위\n0.22m에서 그리퍼 열기"]
        L3["tray_count++"]
        L1 -->|"YES"| L2 --> L3
        L1 -->|"NO (꽉 참)"| FULL["FAILED: tray_full"]
    end

    subgraph UNLOAD["pick tray (꺼내기)"]
        U1["마지막 칸 확인\ntray_count > 0?"]
        U2["해당 칸 0.17m에서 파지"]
        U3["tray_count--"]
        U1 -->|"YES"| U2 --> U3
        U1 -->|"NO (비어 있음)"| EMPTY["FAILED: tray_empty"]
    end
```

---

## 실패 처리

```mermaid
flowchart TD
    FAIL_STEP["단계 실패"]
    COUNT{"같은 단계\nmax_step_failures(2)회\n연속 실패?"}
    RETRY_S["재시도\n(실패 카운트++)"]
    REPORT["/task/status FAILED\n/robot/say 알림\n정지"]

    FAIL_STEP --> COUNT
    COUNT -->|"NO"| RETRY_S -->|"/task/status FAILED"| VLM_RE["vlm 재계획\n(max_replans 1회)"]
    COUNT -->|"YES"| REPORT
```

---

## 모드 전환 시 동작

| 전환 | 동작 |
|------|------|
| AUTO → MANUAL | 진행 중 goal 전부 취소 · `manual_override` 실패 |
| AUTO → EXPLORE | 진행 중 goal 전부 취소 · `explore_override` 실패 |
| MANUAL → AUTO | 새 명령(`/cmd/user_command`) 대기 |
| 비상정지 ON | 즉시 모든 goal 취소 · 재개 대기 |
