# Phase 3 — vlm_task_planner 판단

> 목적: `/mission/signal`을 받아 `/tracked_objects` 상태를 보고 실행 계획(`/task/sub_goals`)을 수립.

## 전체 판단 트리

```mermaid
flowchart TD
    IN["/mission/signal 수신\n{ RED, count:2, sum:5, dest_tag:13 }"]

    Q_PUCK{"해당 색(RED) 퍽이\n/tracked_objects에 있음?"}
    Q_TAG{"번호 아는 퍽으로\n합=5인 2개 조합 존재?"}
    Q_DEST{"목적지 tag=13\n발견됨?"}
    Q_INSPECT{"번호 모르는\n퍽이 남아 있음?"}

    CARRY["✅ 케이스 A\ncarry 계획 수립\n→ /task/sub_goals 발행"]
    WAIT_DEST["⏳ 케이스 C\nwait\n목적지 발견 대기\n(explorer 계속)"]
    INSPECT["🔍 케이스 B\ninspect 계획 수립\n태그 읽기 후 재판단"]
    FAIL["❌ 케이스 D\nsay\n조합 불가 — 사람 개입"]
    WAIT_PUCK["⏳ 퍽 없음\nwait\nexplorer 계속"]

    IN --> Q_PUCK
    Q_PUCK -->|"NO"| WAIT_PUCK -->|"퍽 발견 시 재판단"| Q_PUCK
    Q_PUCK -->|"YES"| Q_TAG
    Q_TAG -->|"YES"| Q_DEST
    Q_DEST -->|"YES"| CARRY
    Q_DEST -->|"NO"| WAIT_DEST -->|"발견 시 재판단"| Q_DEST
    Q_TAG -->|"NO"| Q_INSPECT
    Q_INSPECT -->|"YES"| INSPECT -->|"inspect 완료 후 재판단"| Q_TAG
    Q_INSPECT -->|"NO (전부 확인했는데 합 없음)"| FAIL
```

---

## 케이스 A — carry 계획

조합을 즉시 찾은 경우. 예: R3(tag=2) + R4(tag=3) = 5

```
/task/sub_goals:
  step 1:  navigate  R3_id          ← 퍽 R3 앞으로 이동
  step 2:  pick      R3_id          ← 집기
  step 3:  place     tray           ← 수납함 칸1에 싣기
  step 4:  navigate  R4_id          ← 퍽 R4 앞으로 이동
  step 5:  pick      R4_id          ← 집기
  step 6:  place     tray           ← 수납함 칸2에 싣기
  step 7:  navigate  dest_id        ← 목적지(tag=13) 앞으로 이동
  step 8:  pick      tray           ← 수납함에서 꺼내기 (마지막 것부터)
  step 9:  place     dest_id        ← 목적지에 내려놓기
  step 10: pick      tray           ← 다음 것 꺼내기
  step 11: place     dest_id        ← 목적지에 내려놓기
```

```mermaid
sequenceDiagram
    participant VLM as vlm_task_planner
    participant CO as coordinator

    VLM->>CO: /task/sub_goals [11 steps]
    Note over CO: step 1~3: 퍽 R3 → 수납함
    Note over CO: step 4~6: 퍽 R4 → 수납함
    Note over CO: step 7~11: 목적지로 이동 → 수납함에서 꺼내 내려놓기 ×2
    CO->>VLM: /task/status SUCCEEDED
    VLM->>VLM: 수납 확인 시작 (Phase 5)
```

---

## 케이스 B — inspect 계획

퍽은 보이지만 tag 번호를 아직 모르는 경우.

```
/task/sub_goals:
  step 1:  navigate  puck_A_id
  step 2:  inspect   puck_A_id   ← 팔을 퍽 윗면 0.15m 위 → gripper_cam이 16mm 태그 읽기
  step 3:  navigate  puck_B_id
  step 4:  inspect   puck_B_id
  ...
```

```mermaid
sequenceDiagram
    participant VLM as vlm_task_planner
    participant CO as coordinator
    participant P as perception

    VLM->>CO: /task/sub_goals [inspect × N]
    CO->>P: (팔을 퍽 윗면 위로) gripper_cam 촬영
    P->>VLM: /tracked_objects (details.tag 갱신)
    CO->>VLM: /task/status SUCCEEDED
    VLM->>VLM: 재판단 → 케이스 A 또는 D
```

> **왜 inspect가 필요한가?**
> 퍽 윗면 AprilTag는 16mm — pole_front_cam(640×360)으로는 한 칸 1~2px라 읽을 수 없음.
> gripper_cam을 퍽 바로 위 0.15m에 위치시키면 한 칸 약 6px → 읽힘.

---

## 케이스 C — wait (목적지 미발견)

```mermaid
flowchart LR
    W["목적지 tag=13\n아직 못 봄"]
    E["explorer\n계속 탐색"]
    F["location tag=13\n발견"]
    R["재판단\n→ 케이스 A"]

    W --> E --> F --> R
```

---

## 케이스 D — 조합 불가

```
번호 아는 RED 퍽: tag = [0, 1, 2]  (R1, R2, R3만 확인됨)
필요: count=2, sum=5
가능한 조합: 0+1=1, 0+2=2, 1+2=3 → 합 5 없음

→ say("조합을 찾을 수 없어요") → 사람 확인
```

---

## 재계획 (FAILED 시)

```mermaid
flowchart LR
    FAIL["/task/status FAILED"]
    RE{"max_replans\n초과?"}
    REPLAN["재계획\n(다른 퍽으로 교체 등)"]
    STOP["정지\nsay 알림"]

    FAIL --> RE
    RE -->|"NO (1회 남음)"| REPLAN --> RE
    RE -->|"YES"| STOP
```
