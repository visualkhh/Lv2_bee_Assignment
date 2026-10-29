[목차](README.md) · [전체 아키텍처 ▶](01-arch.md)

## 퍽 이송 로봇(beebot) — ROS 2 실전 노드 설계

"빨간 퍽을 목표 구역으로 옮겨" 같은 명령을 받아 미로 코스 안에서 **이동 + 집기 + 옮겨 놓기**를 스스로 하고 장애물을 피하는 모바일 매니퓰레이터입니다 (자율주행 기반 물류 이송 및 장애물 회피). 확정적 알고리즘(Nav2·MoveIt 2·ros2_control)이 실행을 책임지고, 학습 모델은 **앞에서 판단을 돕거나 뒤에서 실행을 보정**만 합니다.

### 가정 (하드웨어가 다르면 이 값만 바꾼다)

| 항목 | 가정 |
| --- | --- |
| 배포판 | ROS 2 Lyrical (Ubuntu 26.04, Jetson은 24.04 + NVIDIA Isaac ROS) — 속도 명령은 체인 전체 **TwistStamped** |
| 베이스 | TurtleBot3 Waffle Pi, 차동 구동, 바퀴 반경 0.033m · 간격 0.287m, 최대 0.26m/s, **LiDAR 미사용** (거리 센서는 RealSense 2대뿐) |
| 팔 | OpenMANIPULATOR-X **RM-X52-TNM** (4축 + 그리퍼, XM430-W350-T) **+ 손목 roll 1축** — 차체 앞쪽 절반 상판 위, 도달 거리 약 0.38m+, 가반하중 약 0.5kg |
| 제어보드 | OpenCR 1개가 바퀴 2 · 팔 5축 · 그리퍼 · IMU를 모두 연결 — 직접 작성한 펌웨어(beebot_opencr) ↔ 하드웨어 플러그인(BeebotSystemHardware) |
| gripper_cam | 그리퍼 위에 얹은 **Intel RealSense 깊이 카메라**(RGB-D), 팔과 함께 움직임 — 근접 파지용 |
| pole_front_cam | 뒤쪽 전원선 가이드 폴 위 **Intel RealSense 깊이 카메라**(RGB-D), 전방 하향 — **인지·SLAM·장애물·octomap 전부 담당** |
| 연산 | 로봇 탑재 PC 1대 + 학습용 GPU PC 1대 (Isaac Sim / Isaac Lab) |
| 환경 | 미로 코스 4.46 × 6.36m (벽 높이 0.39m, 통로 약 0.73m) — lv2_module6/course, 미끄러운 바닥 구간 가정 · 발제 경기장 A·B 각 4464×3212mm ([mission](../../mission/README.md)) |
| 대표 작업 | **신호등 암호 미션** ([mission/README.md](../../mission/README.md)) — 신호등 깜빡임(색·개수 · 번호 합 · 목적지)을 읽고, 그 색 **퍽**(30×30×60mm, 윗면 AprilTag 번호) 중 번호 합이 맞는 조합을 적재함에 실어 **목적지 장소**(장소 태그 L1~L3)에 내려놓기 → 그대로 있는지 주기적으로 확인. 가는 길의 장애물·사람 회피 |
| 표식 | AprilTag tag36h11 — 퍽 윗면 16mm(0~11, 번호 = 색·점수) · 장소 48mm(12~14) / 바코드 Code128(P2-xx, 장소의 작업 지시) |

> **도달 범위 주의:** 팔이 차체 앞쪽 상판(바닥 위 약 0.2m) 위에 있으므로 바닥의 퍽은 차체 앞 가까운 범위에서만 집힌다. 손목 roll을 더한 5축으로도 바닥 파지 자세는 제한적이니, 상판 높이·퍽 위치를 정하면 MoveIt 도달성부터 확인합니다.

### 계층과 원칙

| 계층 | 역할 | 대표 노드 | 결정 주기 | 학습 |
| --- | --- | --- | --- | --- |
| ① 작업 계획 | 무엇을, 어떤 순서로 | vlm_task_planner (신호는 perception signal_decoder가 해석해 `/mission/signal`로) | 명령·신호·단계 완료·최종 실패마다 (수 초) + 수납 확인 주기 | 신호등 미션은 규칙 · 일반 명령·바코드 지시는 사전학습 VLM(또는 사람 — ai-service human), 추가 학습 없음 |
| ② 임무 관리 | 검증·실행·재시도·안전 규칙 | coordinator | 이벤트마다 | 없음 |
| ③ 앞단 판단 보조 | 어떤 방식으로 할지 고르기 | grasp_scorer, rl_nav_tuner | pick 직전 1회 / 1~2초에 1회 | 지도학습 / 강화학습(또는 규칙) |
| ④ 확정 실행 | 궤적·속도 계산, 충돌 회피 | Nav2, MoveIt 2 (+MTC) | 20~100Hz | 없음 |
| ⑤ 뒷단 실행 보정 | 명령에 작은 Δ를 더해 현실 오차 보정 | rl_nav_correction, rl_arm_correction | 20Hz / 100Hz | 강화학습 (residual) |
| ⑥ 안전 | 최종 명령 제한·감속·차단 | velocity_smoother, collision_monitor, JTC 허용오차, arm_safety_monitor, 전류 한계 | 매 주기 | 없음 |
| ⑦ 하드웨어 | read → update → write | controller_manager + OpenCR | 100Hz | 없음 |

**지켜야 할 다섯 규칙**

- **보정은 범위를 제한한다.** 선속도 Δ ±0.05m/s, 각속도 Δ ±0.10rad/s, 관절 Δ ±0.05rad. 그리고 Δ를 더한 결과는 **현재 프리셋 속도 상한으로 다시 클리핑**한다.
- **안전장치는 학습 노드보다 뒤에 둔다.** RL 출력까지 포함한 최종 명령을 마지막에 거른다.
- **학습 노드는 끌 수 있어야 한다.** 노드를 내리지 않고 `enabled:=false`(Δ=0, 기본 프리셋)로 바꾸면 원래 시스템과 같아야 한다.
- **위는 느리고 추상적, 아래는 빠르고 구체적.** VLM은 제어 루프에 넣지 않는다.
- **판단 권한은 한 층에 하나.** 파지점 2순위 선택 같은 실행 세부는 coordinator가 하고, VLM은 작업 순서만 다시 짠다.

**토픽 이름 규칙 — 명령은 /cmd, 상태는 /state**

- **외부에서 들어오는 명령은 전부 `/cmd/...`.** 보내고 끝 (cmd_vel 관례 — 응답 없음). `/cmd/user_command` · `/cmd/manual/cmd_vel` · `/cmd/manual/arm` · `/cmd/manual/reboot_motors` · `/cmd/e_stop` · `/cmd/operation_mode` · `/cmd/map_mode` · `/cmd/fake_opencr/set_pose`(테스트베드).
- **상태는 `/state/<이름>`, 발행은 state_manager 하나만.** 다른 노드는 구독만 한다. 상태를 바꾸고 싶으면 `/cmd/<이름>`을 보내고 결과는 `/state/<이름>`으로 본다 (`/cmd/operation_mode` → `/state/operation_mode`, `/cmd/map_mode` → `/state/map_mode`, `/cmd/e_stop` → `/state/emergency`). ROS에는 발행자 제한 기능이 없으므로 규칙으로 지키고, 테스트베드 input_manager(호스트 파일 → 토픽)는 `/state/*` 를 거부한다.
- **연결 장치 상태도 같은 규칙.** `/state/devices`는 장치 이름만 쉼표로 내고, 장치별 상세(`active`/`disconnected`/`error`(모터 Hardware Error) + 세부 토픽)는 `/state/devices/<이름>`, 전체 그래프는 `/state/topics`로 낸다. 장치가 없어도 state_manager는 계속 뜬다.
- **응답이 꼭 필요한 요청은 토픽이 아니라 서비스** (예: `/map_saver/save_map`, `/beebotsystem/reboot_motors`).
- **원본 사실 · 상태 · 진단은 셋 다 다르다.**
  - 원본 사실 = 장치·노드가 직접 아는 값을 그대로 내는 토픽 (`/joint_states` · `/imu` · `/tracked_objects` · `/motor_status` · `/arm_safety_event`). 이름에 `/state`를 붙이지 않는다.
  - 상태 = 원본 사실을 모아 state_manager가 판단해 낸 로봇 상태 (`/state/devices/arm = error … joint2:과부하`).
  - 진단 = `/diagnostics` — **사람이 보는** 건강 상태(OK·WARN·ERROR + 값). 노드는 진단 문자열에 기대지 않는다. 노드가 알아야 하는 사실이면 전용 토픽을 만든다.

---

[목차](README.md) · [전체 아키텍처 ▶](01-arch.md) · [원본 페이지](https://dooboostore.github.io/ros2?concept=design&sec=overview)
