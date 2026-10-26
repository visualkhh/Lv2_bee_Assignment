# 노드별 역할 명세 — 단계별 어느 노드가 무엇을 하는가

> 각 단계에서 어떤 노드가 입력을 받아 어떤 출력을 내는지 정리.
> 패키지·담당자·실행 명령 포함.

---

## 노드 전체 목록

```
패키지         노드                     담당     언어
────────────────────────────────────────────────────────
bringup        (launch only)            권형중   Python
manager        state_manager            김현하   C++
plan           signal_decoder           권형중   C++
plan           user_input               권형중   C++
plan           vlm_task_planner         권형중   C++
plan           coordinator              권형중   C++
plan           explorer                 김현하   C++
perception     perception               전승혜   Python
perception     body_filter              전승혜   C++
ml             grasp_scorer             송정혁   C++
ml             rl_nav_tuner             송정혁   C++
ml             rl_nav_correction        송정혁   C++
ml             rl_arm_correction        송정혁   C++ (controller plugin)
control        manual_control           김현하   C++
control        arm_safety_monitor       김현하   C++
control        BeebotSystemHardware     김현하   C++ (hardware plugin)
ros_system     Nav2                     📦 제공
ros_system     MoveIt2 + MTC            📦 제공
ros_system     RTAB-Map                 📦 제공
ros_system     EKF (robot_localization) 📦 제공
ros_system     controller_manager       📦 제공
ros_system     twist_mux                📦 제공
ros_system     velocity_smoother        📦 제공
ros_system     collision_monitor        📦 제공
```

---

## 실행 방법

```bash
# 경기용 (사전 SLAM 완료 후)
ros2 launch bringup bringup.launch.py map:=saved

# SLAM 지도 제작용
ros2 launch bringup bringup.launch.py map:=slam

# 특정 지도 지정
ros2 launch bringup bringup.launch.py map:=saved map_name:=20261025_competition

# 학습 노드 없이 (비교 기준)
ros2 launch bringup bringup.launch.py learning_enabled:=false

# 노드 하나만 재시작
ros2 topic pub --once /cmd/restart std_msgs/msg/String "{data: 'node perception'}"

# 전체 재시작
ros2 topic pub --once /cmd/restart std_msgs/msg/String "{data: 'all'}"
```

---

## 사전 준비 (SLAM) — 노드 역할

```mermaid
flowchart TD
    subgraph SLAM_NODES["map:=slam 으로 기동 시 활성 노드"]
        HW["🛠 BeebotSystemHardware\ncontrol · 김현하\nOpenCR 통신 100Hz\n바퀴·팔·IMU read/write"]
        CM["📦 controller_manager\ndiff_drive · JTC · gripper\n/joint_states 발행"]
        EKF["📦 EKF\nIMU + 오도메트리 융합\n/odom 발행"]
        RTAB["📦 RTAB-Map\nMAPPING 모드\n포인트클라우드 → 지도 생성\n/map 토픽 발행"]
        CAM["📦 RealSense 드라이버\npole_front_cam · gripper_cam\n컬러·depth·포인트클라우드"]
        SM["🛠 state_manager\nmanager · 김현하\nmap_mode=MAPPING 상태 관리"]
        MC["🛠 manual_control\ncontrol · 김현하\n운영자 조종 명령 수신"]
        MUX["📦 twist_mux\n수동 명령 우선순위 처리"]
    end

    OP["🧑 운영자\n수동 조종"]
    SAVE["서비스 호출\n/map_saver/save_map\n→ rtabmap.db 저장"]

    OP -->|"/cmd/manual/cmd_vel"| MC --> MUX --> CM
    CAM --> EKF --> RTAB
    HW --> CM
    SM -->|"MAPPING 상태 발행"| RTAB
    RTAB -->|"지도 완성"| SAVE
```

| 노드 | 패키지 | 담당 | 역할 | 입력 | 출력 |
|------|--------|------|------|------|------|
| `BeebotSystemHardware` | control | **김현하** | OpenCR 통신 | serial | `/joint_states` `/imu` `/motor_status` |
| `controller_manager` | ros_system | 📦 | 바퀴/팔/그리퍼 컨트롤러 | `/cmd_vel` | 바퀴 속도 명령 |
| `EKF` | ros_system | 📦 | IMU + 오도메트리 융합 | `/imu` `/joint_states` | `/odom` |
| `RTAB-Map` | ros_system | 📦 | 지도 제작 (MAPPING) | 카메라, `/odom` | `/map`, map→odom TF |
| `manual_control` | control | **김현하** | 운영자 조종 명령 처리 | `/cmd/manual/cmd_vel` | `/cmd_vel_manual` |
| `state_manager` | manager | **김현하** | map_mode 상태 관리 | `/cmd/map_mode` | `/state/map_mode` |

---

## Phase 0 — 부팅 (경기용) — 노드 역할

```mermaid
flowchart TD
    subgraph BOOT["map:=saved 로 기동 시 동작"]
        HW2["🛠 BeebotSystemHardware\nOpenCR 연결·초기화"]
        CM2["📦 controller_manager\n컨트롤러 활성화"]
        EKF2["📦 EKF\n위치 추정 시작"]
        RTAB2["📦 RTAB-Map\nLOCALIZATION 모드\nrtabmap.db 로드\n위치 추정만 수행"]
        CAM2["📦 RealSense 드라이버\n카메라 스트림 시작"]
        PER["🛠 perception\nperception · 전승혜\nYOLO + AprilTag + 바코드\n감지 시작"]
        SM2["🛠 state_manager\nmap_mode=LOCALIZATION\noperation_mode=AUTO"]
        CO["🛠 coordinator\nplan · 권형중\n부팅 직후 팔 → initial_pose\n(JTC 토픽, 3초 궤적)"]
        SD["🛠 signal_decoder\nplan · 권형중\n/tracked_objects 구독 대기"]
        VLM["🛠 vlm_task_planner\nplan · 권형중\n/mission/signal 대기"]
    end

    READY(["✅ 준비 완료\nmap→odom TF 안정\noperation_mode=AUTO"])
    BOOT --> READY
```

| 노드 | 패키지 | 담당 | 역할 | 입력 | 출력 |
|------|--------|------|------|------|------|
| `BeebotSystemHardware` | control | **김현하** | OpenCR 초기화·통신 | serial | `/joint_states` `/imu` |
| `RTAB-Map` | ros_system | 📦 | LOCALIZATION — 지도 매칭 | 카메라, `/odom` | map→odom TF |
| `EKF` | ros_system | 📦 | 오도메트리 융합 | `/imu` `/joint_states` | `/odom` |
| `perception` | perception | **전승혜** | 카메라 처리 시작 | 카메라 2대 | `/tracked_objects` |
| `state_manager` | manager | **김현하** | 시스템 상태 초기화 | 파라미터 | `/state/map_mode` `/state/operation_mode` |
| `coordinator` | plan | **권형중** | **팔 initial_pose 이동** | `/joint_states` | `/arm_controller/joint_trajectory` |
| `signal_decoder` | plan | **권형중** | 감시 준비 (대기) | `/tracked_objects` | — |
| `vlm_task_planner` | plan | **권형중** | 신호 대기 | `/mission/signal` | — |

---

## Phase 1 — 요소 탐색 — 노드 역할

```mermaid
flowchart LR
    subgraph EXPLORE["EXPLORE 모드"]
        EXP["🛠 explorer\nplan · 김현하\n프론티어 선택\nNav2에 목표 전송"]
        NAV["📦 Nav2\n경로 계획·주행·장애물 회피"]
        RLN["🛠 rl_nav_correction\nml · 송정혁\n주행 명령 Δ 보정"]
        RLT["🛠 rl_nav_tuner\nml · 송정혁\nNav2 파라미터 동적 조정"]
        PER2["🛠 perception\n전승혜\n이동 중 물체 감지 4Hz"]
        CO2["🛠 coordinator\n권형중\n팔 scan_pose 유지"]
        SD2["🛠 signal_decoder\n권형중\n신호등 발견 시 감시 시작"]
        SM3["🛠 state_manager\n김현하\nEXPLORE 허용 판단"]
        COL["📦 collision_monitor\n사람·장애물 감지 감속"]
        BF["🛠 body_filter\n전승혜\n자체 포인트클라우드 제거"]
    end

    EXP -->|"/navigate_to_pose"| NAV
    NAV --> RLN --> COL
    PER2 -->|"/tracked_objects"| SD2
    EXP -->|"완료"| SM3
```

| 노드 | 패키지 | 담당 | 역할 | 입력 | 출력 |
|------|--------|------|------|------|------|
| `explorer` | plan | **김현하** | 프론티어 탐색, 이동 목표 전송 | `/map` `/global_costmap/costmap` TF | `/navigate_to_pose` 액션 |
| `Nav2` | ros_system | 📦 | 경로 계획·추종 | `/navigate_to_pose` `/map` 센서 | `/cmd_vel_nav` |
| `rl_nav_correction` | ml | **송정혁** | 주행 명령 Δ 보정 (±0.05m/s) | `/cmd_vel_nav` | `/cmd_vel_corrected` |
| `rl_nav_tuner` | ml | **송정혁** | Nav2 speed_limit 동적 조정 | 센서 `/tracked_objects` | `/speed_limit` Nav2 파라미터 |
| `twist_mux` | ros_system | 📦 | 자동/수동/비상정지 우선순위 | `/cmd_vel_corrected` 등 | `/cmd_vel_mux` |
| `velocity_smoother` | ros_system | 📦 | 급가속 완화 | `/cmd_vel_mux` | `/cmd_vel` |
| `collision_monitor` | ros_system | 📦 | 사람·장애물 감지 감속·정지 | 센서 `/cmd_vel` | `/cmd_vel` (제한) |
| `body_filter` | perception | **전승혜** | 로봇 자체 점군 제거 | 포인트클라우드 | 필터링된 포인트클라우드 |
| `perception` | perception | **전승혜** | YOLO·AprilTag 감지·추적 | 카메라 2대 | `/tracked_objects` |
| `coordinator` | plan | **권형중** | **팔 scan_pose 유지** (SLAM 보조) | `/joint_states` `/state/map_mode` | `/arm_controller/joint_trajectory` |
| `signal_decoder` | plan | **권형중** | traffic_light 발견 시 감시 시작 | `/tracked_objects` | (신호 시 `/mission/signal`) |
| `state_manager` | manager | **김현하** | EXPLORE 허용·AUTO 복귀 | `/cmd/operation_mode` `/explore/result` | `/state/operation_mode` |

---

## Phase 2 — 신호 읽기 — 노드 역할

```mermaid
sequenceDiagram
    participant TL as 🚦 신호등
    participant CAM as 📦 RealSense
    participant PER as 🛠 perception<br/>(전승혜)
    participant SD as 🛠 signal_decoder<br/>(권형중)
    participant VLM as 🛠 vlm_task_planner<br/>(권형중)

    Note over TL: 버튼 눌림 → 점멸 시작
    TL->>CAM: 빛 변화
    CAM->>PER: 컬러 프레임
    PER->>SD: /tracked_objects<br/>(traffic_light.color = red|yellow|green|all|off)
    Note over SD: 깜빡임 카운트<br/>단계 구분(all 2초)<br/>잡음 필터(min_on_s 0.15s)
    SD->>VLM: /mission/signal<br/>{color, count, sum, dest_tag, raw}
```

| 노드 | 패키지 | 담당 | 역할 | 입력 | 출력 |
|------|--------|------|------|------|------|
| `perception` | perception | **전승혜** | 신호등 색 분류 (`classify_color`) | pole_front_cam | `/tracked_objects` (traffic_light) |
| `signal_decoder` | plan | **권형중** | 3단계 깜빡임 해석, 잡음 필터링 | `/tracked_objects` | `/mission/signal` |

---

## Phase 3 — 계획 수립 — 노드 역할

```mermaid
flowchart TD
    subgraph PLAN["계획 수립"]
        VLM2["🛠 vlm_task_planner\nplan · 권형중\n규칙 기반 판단\npick_combo() 조합 계산"]
    end

    SIG["/mission/signal"]
    OBJ["/tracked_objects"]
    STATUS["/task/status (FAILED → 재계획)"]
    SUBG["/task/sub_goals → coordinator"]

    SIG & OBJ & STATUS --> VLM2 --> SUBG
```

| 노드 | 패키지 | 담당 | 역할 | 입력 | 출력 |
|------|--------|------|------|------|------|
| `vlm_task_planner` | plan | **권형중** | 미션 신호 해석, 조합 계산, 계획 수립 | `/mission/signal` `/tracked_objects` `/task/status` | `/task/sub_goals` |

> `vlm_endpoint`가 설정되면 ai-service(외부 LLM)에 POST /plan → SubGoalArray 수신.
> 비어 있으면(기본·경기용) 규칙 템플릿만 사용.

---

## Phase 4 — 실행 — 노드 역할

```mermaid
flowchart TD
    subgraph EXEC["실행 계층"]
        CO3["🛠 coordinator\nplan · 권형중\n단계별 순서 실행\ngoal 전송·결과 수신"]

        subgraph NAV_LAYER["이동"]
            NAV3["📦 Nav2"]
            RLN3["🛠 rl_nav_correction\n송정혁\nΔ 보정"]
        end

        subgraph ARM_LAYER["팔·그리퍼"]
            MV3["📦 MoveIt2 + MTC\n역기구학·궤적 계획"]
            RLA3["🛠 rl_arm_correction\n송정혁\nchainable controller"]
            GR3["📦 gripper_controller"]
            GS3["🛠 grasp_scorer\n송정혁\n파지 후보 점수"]
        end

        subgraph SAFE_LAYER["안전"]
            ASM3["🛠 arm_safety_monitor\n김현하\n팔 접촉·과부하 감지"]
            COL3["📦 collision_monitor\n감속·정지"]
            MUX3["📦 twist_mux\n비상정지 잠금"]
        end

        subgraph HW_LAYER["하드웨어"]
            CM3["📦 controller_manager"]
            HW3["🛠 BeebotSystemHardware\n김현하\nOpenCR 통신"]
            OCR["OpenCR\n바퀴2·팔5·그리퍼·IMU"]
        end
    end

    SUBG3["/task/sub_goals"] --> CO3
    CO3 -->|navigate| NAV3 --> RLN3 --> MUX3 --> COL3 --> CM3
    CO3 -->|pick/place| GS3 --> MV3 --> RLA3 --> CM3
    CO3 -->|그리퍼| GR3 --> CM3
    CM3 --> HW3 <-->|serial| OCR
    ASM3 -->|접촉 이벤트| CO3
    CO3 --> STATUS3["/task/status"]
```

### 단계별 담당 노드 상세

| 단계 | 노드 순서 | 패키지 | 담당 |
|------|-----------|--------|------|
| **navigate** | coordinator → Nav2 → rl_nav_correction → twist_mux → velocity_smoother → collision_monitor → controller_manager → BeebotSystemHardware | plan/ros_system/ml/control | 권형중/📦/송정혁/김현하 |
| **pick** | coordinator → grasp_scorer → MoveIt2 → rl_arm_correction → controller_manager → BeebotSystemHardware | plan/ml/ros_system/control | 권형중/송정혁/📦/김현하 |
| **place tray** | coordinator → MoveIt2 → gripper_controller → BeebotSystemHardware | plan/ros_system/control | 권형중/📦/김현하 |
| **inspect** | coordinator → MoveIt2 → (팔 이동) → perception(gripper_cam 태그 읽기) | plan/ros_system/perception | 권형중/📦/전승혜 |
| **안전 감시** | arm_safety_monitor → coordinator (접촉 시 중단) | control | **김현하** |
| **자체 필터** | body_filter → Nav2·collision_monitor (자기 몸 오감지 방지) | perception | **전승혜** |

---

## Phase 5 — 수납 확인 — 노드 역할

| 노드 | 패키지 | 담당 | 역할 | 입력 | 출력 |
|------|--------|------|------|------|------|
| `vlm_task_planner` | plan | **권형중** | 60초마다 목적지로 look/recarry 계획 | `/tracked_objects` `/task/status` | `/task/sub_goals` |
| `coordinator` | plan | **권형중** | 목적지 이동 후 inspect 실행 | `/task/sub_goals` | `/navigate_to_pose` `/task/status` |
| `perception` | perception | **전승혜** | gripper_cam으로 퍽 tag 읽기 | gripper_cam | `/tracked_objects` (details.tag) |

---

## 김현하(제어) 담당 노드 요약

```mermaid
flowchart TB
    subgraph KHH["김현하 담당"]
        HW4["BeebotSystemHardware\ncontrol 패키지\n\nOpenCR ↔ ROS 2 브릿지\n100Hz read/write\n바퀴2 · 팔5 · 그리퍼 · IMU"]
        MC3["manual_control\ncontrol 패키지\n\n/cmd/manual/* 수신\n→ /cmd_vel_manual 발행\n수동 조종·팔 직접 조작"]
        ASM4["arm_safety_monitor\ncontrol 패키지\n\n팔 관절 전류 감시\n접촉 감지 → /arm_safety_event\n→ coordinator 에 전달"]
        BF3["body_filter\nperception 패키지\n\n로봇 자체 몸체를\n포인트클라우드에서 제거\n(자기 팔이 장애물로 오인 방지)"]
        SM4["state_manager\nmanager 패키지\n\n시스템 상태 총괄\noperation_mode · map_mode\nemergency · devices · restart"]
        EXP3["explorer\nplan 패키지\n\n프론티어 기반 자동 탐색\n퍽·신호등·목적지 위치 발견"]
    end

    OCR2["OpenCR\n(펌웨어: beebot_opencr)"]
    HW4 <-->|"/dev/opencr serial"| OCR2
```

| 노드 | 패키지 | 언제 활성 | 핵심 역할 |
|------|--------|----------|-----------|
| `BeebotSystemHardware` | control | 항상 | 하드웨어 최하단 — 없으면 로봇 안 움직임 |
| `manual_control` | control | 항상 | 수동 전환 시 운영자 명령 전달 |
| `arm_safety_monitor` | control | 항상 | 팔 접촉·과부하 감지·중단 |
| `body_filter` | perception | 항상 | Nav2·collision_monitor 오감지 방지 |
| `state_manager` | manager | 항상 | 전체 상태 관리 (SLAM→경기 모드 전환 포함) |
| `explorer` | plan | Phase 1 (EXPLORE) | 랜덤 배치된 요소 위치 탐색 |
