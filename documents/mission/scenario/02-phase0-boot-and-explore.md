# Phase 0 — 부팅 & Phase 1 — 요소 탐색 (v2)

> v2 핵심: 지도는 미리 있음. SLAM 없이 LOCALIZATION 모드로 시작.
> 퍽·신호등·목적지 위치만 카메라로 찾으면 됨.

---

## Phase 0 — 부팅

```mermaid
flowchart TD
    START(["bringup 실행"])

    NODES["모든 노드 기동\nNav2 · MoveIt2 · RTAB-Map\nperception · controller_manager\nstate_manager · signal_decoder"]

    MAP_MODE{"map_mode\n파라미터?"}
    LOC["RTAB-Map: LOCALIZATION\n미리 만든 rtabmap.db 로드\n위치 추정 시작"]
    SLAM_M["RTAB-Map: MAPPING\n(SLAM 모드 — 사전 지도 없을 때)"]

    TF_WAIT["map → odom → base_footprint\nTF 안정화 대기"]
    ARM["팔 → initial_pose\n(JTC, 3초 궤적)"]
    READY(["✅ 준비 완료\noperation_mode = AUTO"])

    START --> NODES --> MAP_MODE
    MAP_MODE -->|"LOCALIZATION (기본·경기용)"| LOC --> TF_WAIT
    MAP_MODE -->|"MAPPING (사전 지도 제작 시)"| SLAM_M --> TF_WAIT
    TF_WAIT --> ARM --> READY
```

### 경기 전 사전 작업 (한 번만)

```mermaid
flowchart LR
    PRE1["경기장 세팅 완료\n(벽·마커 고정)"]
    PRE2["map_mode = MAPPING으로 실행"]
    PRE3["수동 조종으로 경기장 전체 주행\n(RTAB-Map이 지도 생성)"]
    PRE4["/map_saver/save_map\n→ rtabmap.db 저장"]
    PRE5["이후 경기는\nmap_mode = LOCALIZATION\n같은 db 재사용"]

    PRE1 --> PRE2 --> PRE3 --> PRE4 --> PRE5
```

> ⚠️ 퍽·신호등·목적지는 **경기마다 위치가 바뀌므로 지도에 포함시키지 않음**.
> 지도에는 벽·구조물만 담기고, 요소들은 매 경기 perception이 새로 발견.

---

## Phase 1 — 요소 탐색

```mermaid
flowchart TD
    READY(["operation_mode = EXPLORE\n(지도 안에서 탐색 시작)"])

    subgraph EXPLORER["explorer 동작"]
        FRT["지도에서 프론티어 찾기\n(로봇이 아직 카메라로 못 본 영역)"]
        NOTE_MAP["⚠️ 지도(벽)는 이미 알고 있음\n프론티어 = 카메라가 미도달 구역\n(벽 근처 어두운 곳 등)"]
        PICK_F{"갈 수 있는\n프론티어 있음?"}
        MOVE["Nav2로 이동"]
        BL["블랙리스트 처리\n(도착·실패·시간초과)"]
        END_E["DONE\n→ operation_mode = AUTO"]
    end

    subgraph PERCEPTION["perception 감지 (이동 중)"]
        CAM1["pole_front_cam\n전방 하향 감시"]
        CAM2["gripper_cam\n(scan_pose 중 후방)"]
        TL_F["🚦 신호등 발견\ntracked_objects 등록"]
        PK_F["🟥🟩🟦 퍽 발견\n위치 등록 (번호 미확인)"]
        LC_F["📍 목적지 발견\ntag 12~14 등록"]
    end

    READY --> FRT --> NOTE_MAP --> PICK_F
    PICK_F -->|"있음"| MOVE
    MOVE --> CAM1 & CAM2
    CAM1 & CAM2 --> TL_F & PK_F & LC_F
    TL_F & PK_F & LC_F --> BL --> PICK_F
    PICK_F -->|"없음"| END_E
    MOVE -->|"실패·시간초과"| BL
```

### 탐색 우선순위 전략

```mermaid
flowchart LR
    subgraph IDEAL["이상적 탐색 순서"]
        I1["① 신호등 위치 파악\n(신호 읽기 준비)"]
        I2["② 목적지 태그 발견\n(L1·L2·L3)"]
        I3["③ 퍽 위치 파악\n(번호 확인은 나중에)"]
        I1 --> I2 --> I3
    end

    subgraph REAL["실제 (explorer는 최단거리 우선)"]
        R1["프론티어 중\n로봇에서 가장 가까운 것"]
        R2["발견 순서 랜덤\n→ vlm이 정보 부족 시 wait"]
        R1 --> R2
    end
```

### 신호등 발견 후 동작

신호등 위치가 `tracked_objects`에 등록되면:
- **signal_decoder가 자동으로 감시 시작** (별도 명령 불필요)
- 버튼이 눌리기 전이므로 신호등은 꺼져 있음 → signal_decoder는 대기 상태
- 탐색은 계속 진행 (퍽·목적지 찾기)

### 목적지 미발견 상태에서 신호가 먼저 오면

```mermaid
flowchart LR
    SIGNAL["/mission/signal 수신\ndest_tag=13"]
    NO_DEST["location tag=13\n아직 없음"]
    WAIT["vlm: kind=wait\n/task/sub_goals 발행 안 함"]
    EXPLORE["explorer 계속 탐색"]
    FOUND["tag=13 발견"]
    PLAN["vlm 재판단\n→ carry 계획 수립"]

    SIGNAL --> NO_DEST --> WAIT --> EXPLORE --> FOUND --> PLAN
```

---

## Phase 0~1 타임라인 (예상)

```
T+0s   bringup 실행
T+3s   RTAB-Map 지도 로드 완료 · TF 안정
T+6s   팔 initial_pose 완료
T+7s   EXPLORE 시작
T+10s  신호등 발견 → signal_decoder 대기 진입
T+30s  퍽 4개 발견, 목적지 L2 발견
T+50s  나머지 퍽 발견 · 프론티어 소진 → AUTO 복귀
         (또는 충분히 발견 시 수동으로 AUTO 전환)
T+51s  버튼 누름 대기 중...
```
