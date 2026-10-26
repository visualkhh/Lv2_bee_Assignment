# 전체 흐름 다이어그램 (v2 — 사전 지도 + 버튼 신호)

> **전제 변경 (v2)**
> - 지도(map)는 **경기 전 미리 SLAM 완료** → 경기 시작 시 LOCALIZATION 모드로 시작
> - 퍽·신호등·목적지 위치는 **경기마다 랜덤 배치** → 탐색으로 발견해야 함
> - 신호등은 **사람이 버튼을 눌러야 점멸 시작** → 버튼 누르기 전엔 감시할 필요 없음

---

## 5단계 전체 흐름

```mermaid
flowchart TD
    START(["🤖 로봇 시작\n임의 위치에서 부팅"])

    subgraph P0["Phase 0 — 부팅"]
        BOOT["모든 노드 기동"]
        MAP_L["RTAB-Map: LOCALIZATION 모드\n(미리 만든 지도 로드)\n위치 추정 시작"]
        ARM_INIT["팔 → initial_pose"]
        POSE_OK["초기 위치 확정\n(map → odom TF 안정)"]
    end

    subgraph P1["Phase 1 — 요소 탐색"]
        direction TB
        NOTE1["⚠️ 지도는 있지만\n퍽·신호등·목적지 위치는 모름\n→ 돌아다니며 카메라로 발견"]
        EXPLORE["operation_mode = EXPLORE\nexplorer: 프론티어 탐색\n(지도 안에서 미탐색 경계)"]
        DETECT["perception 실시간 감지"]
        DET_TL["🚦 traffic_light 위치 등록"]
        DET_PK["🟥 puck 위치 등록\n(번호 미확인)"]
        DET_LC["📍 location tag 12~14 등록"]
        EXPLORE_END{"필요한 요소\n모두 발견?"}
    end

    subgraph P2["Phase 2 — 신호 대기 & 읽기"]
        BTN["🧑 심판/운영자\n신호등 버튼 누름"]
        TL_START["신호등 점멸 시작"]
        SD["signal_decoder\n깜빡임 3단계 해석"]
        SIG_OUT["/mission/signal\n{ color, count, sum, dest_tag }"]
    end

    subgraph P3["Phase 3 — 계획 수립"]
        VLM["vlm_task_planner 판단\n(4가지 케이스)"]
        SUBGOALS["/task/sub_goals 발행"]
    end

    subgraph P4["Phase 4 — 실행"]
        COORD["coordinator\nnavigate → pick → tray → place"]
    end

    subgraph P5["Phase 5 — 수납 확인"]
        VERIFY["목적지 퍽 확인\n(60초 주기)"]
    end

    START --> BOOT --> MAP_L --> ARM_INIT --> POSE_OK
    POSE_OK --> NOTE1 --> EXPLORE
    EXPLORE --> DETECT
    DETECT --> DET_TL & DET_PK & DET_LC
    DET_TL & DET_PK & DET_LC --> EXPLORE_END

    EXPLORE_END -->|"아직 모름"| EXPLORE
    EXPLORE_END -->|"신호등 위치 파악됨\n(퍽·목적지 계속 탐색 병행)"| BTN

    BTN --> TL_START --> SD --> SIG_OUT --> VLM --> SUBGOALS --> COORD --> VERIFY
```

---

## 핵심 차이: v1 vs v2

```mermaid
flowchart LR
    subgraph V1["❌ v1 (SLAM 포함)"]
        V1A["부팅"] --> V1B["SLAM + 탐색 (동시)"]
        V1B --> V1C["지도 완성 + 요소 발견"]
        V1C --> V1D["신호 읽기"]
    end

    subgraph V2["✅ v2 (사전 지도)"]
        V2A["부팅"] --> V2B["LOCALIZATION\n(미리 만든 지도 로드)"]
        V2B --> V2C["탐색 (요소만 찾기)\n지도 안에서 프론티어"]
        V2C --> V2D["버튼 누름 대기"]
        V2D --> V2E["신호 읽기"]
    end
```

**달라지는 점:**
- SLAM 없음 → `map_mode = LOCALIZATION`으로 시작, `RTAB-Map`은 위치추정만
- explorer는 여전히 필요 — **지도는 있어도 퍽/목적지 위치는 카메라로 찾아야 함**
- 신호등 점멸은 **버튼 트리거** → signal_decoder는 항상 감시 상태이지만, 버튼 누르기 전엔 아무것도 깜빡이지 않음

---

## 신호 대기 상세

```mermaid
sequenceDiagram
    participant OP as 🧑 운영자
    participant TL as 신호등
    participant P as perception
    participant SD as signal_decoder
    participant VLM as vlm_task_planner

    Note over SD: 항상 감시 중 (traffic_light 상태 구독)
    Note over TL: 꺼져 있음 (버튼 누르기 전)

    OP->>TL: 버튼 누름
    TL->>P: 점멸 시작 (빨강·노랑·초록)
    P->>SD: /tracked_objects (traffic_light 색 변화)

    Note over SD: 1단계 감지 시작
    SD->>SD: 깜빡임 카운트
    SD->>SD: 단계 구분 (all 2초)
    SD->>SD: 3단계 완료
    SD->>VLM: /mission/signal { RED, 2, 5, 13 }
    VLM->>VLM: 케이스 판단 → 계획 수립
```

---

## 탐색 중 신호등 발견 → 대기 진입

```mermaid
flowchart TD
    EX["explorer 탐색 중"]
    FOUND_TL["🚦 신호등 위치 발견\n/tracked_objects 등록"]
    WAIT_BTN["신호등 앞 대기\n(버튼 누름 감지 준비)"]
    FOUND_OTHERS{"퍽·목적지도\n탐색 계속"}
    BTN_PRESS["버튼 눌림\n점멸 시작"]
    READ["signal_decoder\n3단계 해석"]

    EX --> FOUND_TL
    FOUND_TL --> WAIT_BTN
    FOUND_TL --> FOUND_OTHERS
    FOUND_OTHERS -->|"탐색 병행"| EX
    WAIT_BTN --> BTN_PRESS --> READ
```

> **구현 포인트:**
> signal_decoder는 `traffic_light`가 `off` 상태에서 갑자기 점멸 시작되는 것을 감지.
> 버튼 누르는 이벤트를 별도로 구독할 필요 없이, 점멸 자체가 트리거.
