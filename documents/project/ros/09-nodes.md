[목차](README.md) · [◀ ros2_control 연결](08-ctrl.md) · [노드 사전 ▶](10-catalog.md)

## 직접 작성 노드와 제공 노드

직접 작성은 노드 14개 + 컨트롤러 플러그인(rl_arm_correction) + 하드웨어 플러그인(BeebotSystemHardware) + OpenCR 펌웨어 + 호스트 ai-service입니다. 나머지는 생태계 패키지를 가져와 설정하고 연결하는 일입니다.
구현 위치: `lv2_module6/ros2_ws/src/<패키지>` — 패키지는 전부 ament_cmake, 노드는 C++ (perception · scan_cam_gate만 Python 스크립트).

> 인터페이스 명세: [커스텀 인터페이스](../API.md#api-custom)

| 노드 | 구분 | 패키지 · 형태 | 핵심 작업 |
| --- | --- | --- | --- |
| perception | 직접 | perception · ROS 2 노드 (Python) | YOLO 퍽·목표 구역·장애물·신호등 검출, 3D 위치, 두 카메라 융합 추적(ID), 퍽·장소는 안 보여도 기억 · AprilTag(OpenCV aruco — 퍽 윗면 번호 `tag`, 장소 `location`) · 바코드(zxing-cpp — `barcode` + /mission/qr) · 신호등 상태(`all` 포함) → /tracked_objects |
| user_input | 직접 | plan · ROS 2 노드 | 운영자 텍스트 명령 → /cmd/user_command, /robot/say 표시 |
| vlm_task_planner | 직접 | plan · ROS 2 노드 | VLM 호출, 프롬프트, JSON 출력 파싱 → /task/sub_goals · 신호등 미션은 규칙으로 (조합 → 옮기기 → 수납 확인: 옮긴 직후 + `verify_period_s`마다 목적지에서 옮긴 번호 확인, 빠지면 다시 옮김·say) |
| signal_decoder | 직접 | perception · ROS 2 노드 (C++) | 신호등 암호 미션 — /tracked_objects의 신호등 상태를 시간축으로 쌓아 1·2·3단계(색·개수 · 번호 합 · 목적지 태그)를 읽어 /mission/signal (단계 구분 = 세 색 동시 · 꺼짐 — 파라미터) |
| explorer | 직접 | plan · ROS 2 노드 (C++) | 자동 탐사 — 조작 모드 EXPLORE일 때 /map 프론티어(빈 곳·모르는 곳 경계)를 골라 Nav2 /navigate_to_pose로 차례로 보냄, 끝나면 /explore/result |
| coordinator | 직접 | plan · ROS 2 노드 | 계획 검증(navigate·pick·place·inspect·say), 액션 호출, 실행 규칙, 재시도, 적재함(`tray`) 싣기·꺼내기, `inspect`(퍽 윗면 위로 손끝 → 번호 읽기), 시작 자세, AUTO가 아니면 멈춤 → /task/status |
| grasp_scorer | 직접 | ml · ROS 2 서비스 노드 | 모델 로드, IK 가능 후보만 채점, /score_grasps 응답 |
| rl_nav_tuner | 직접 | ml · ROS 2 노드 | 상황 요약 → 프리셋 선택 → /speed_limit + set_parameters |
| rl_nav_correction | 직접 | ml · ROS 2 노드 | 관측 구성, 정책 추론, Δ 제한, 상한 클리핑, 끊김 시 Δ=0 |
| rl_arm_correction | 직접 | ml · ros2_control chainable controller | reference에 Δ 더하기, enabled 파라미터 |
| arm_safety_monitor | 직접 | control · ROS 2 노드 | 전류로 접촉 감지, 궤적 취소, coordinator 보고 |
| manual_control | 직접 | control · ROS 2 노드 | 수동 모드(/state/operation_mode), 원격 조종 → twist_mux, 팔·그리퍼 직접 명령, `/cmd/manual/reboot_motors` → 📞 /beebotsystem/reboot_motors |
| BeebotSystemHardware | 직접 | control · ros2_control SystemInterface 플러그인 | OpenCR와 USB 시리얼(`C` 명령 · `R` 재부팅 / `S` 상태 · `E` 모터 에러), 관절 영점 offset(URDF `<param name="offset">`), 첫 `S` 전엔 활성화 안 함, /motor_status(바뀔 때만), 📞 ~/reboot_motors, mock 모드 |
| scan_cam_gate | 직접 | ros_system · ROS 2 노드 (Python) | 팔이 스캔 자세일 때만 gripper_cam depth·점구름을 RTAB-Map 쪽으로 넘김 (지도 만들 때 두 번째 카메라) |
| body_filter | 직접 | perception · ROS 2 노드 (C++) | pole_front_cam 점구름에서 자기 몸(URDF collision) 점 제거 + `stride: 2`로 솎아 Nav2 |
| state_manager | 직접 | manager · ROS 2 노드 | 로봇 상태의 주인 — /state/<이름>만 발행 (조작·지도 모드, RTAB-Map 전환) · `/state/devices`(장치 이름 목록) + `/state/devices/<이름>`(active/disconnected/error 상세 — 팔·바퀴·그리퍼 모터 에러 포함) + `/state/topics`(전체 그래프) |
| (OpenCR 펌웨어) beebot_opencr | 직접 | lv2_module6/firmware · Arduino | 다이나믹셀 8개 + IMU, 단위 변환, watchdog · 한 패킷 BulkWrite(바퀴 속도 + 팔·그리퍼 위치) · SyncRead(전류·속도·위치) · `S` 10ms · Hardware Error `E` 0.5초 · `R` 재부팅 · 12V 끊겼다 오면 토크 천천히 복구 |
| (호스트) ai-service | 직접 | lv2_module6/ai-service · Python HTTP 서버 (ROS 아님) | vlm_task_planner의 POST /plan → backend(VLM CLI · **human**: 사람이 웹 페이지에서 계획) → SubGoal 목록 |
| realsense2_camera ×2 (pole_front_cam, gripper_cam) | 제공 | Intel RealSense ROS 2 래퍼 | 장치를 USB 3로 열고 토픽 발행 — serial_no, camera_name, 해상도·fps, align_depth, pointcloud 켜기 |
| robot_state_publisher | 제공 | ROS 2 기본 | URDF(urdf/<이름>/urdf.xacro — launch `urdf:=`, 기본 beebot_mesh) |
| robot_localization (ekf) | 제공 | ROS 2 패키지 | /odom + /imu 융합, odom TF 발행 |
| twist_mux | 제공 | ROS 2 패키지 | 자동 /cmd_vel_corrected · 수동 /cmd_vel_manual 고르기, /state/emergency 잠금(/cmd/e_stop → state_manager) → /cmd_vel_mux |
| bt_navigator, planner_server, controller_server, behavior_server, velocity_smoother, collision_monitor, lifecycle_manager | 제공 | Nav2 | nav2_params.yaml (voxel layer에 점구름, velocity_smoother 입력 /cmd_vel_mux), 리매핑 |
| rtabmap | 제공 | rtabmap_ros | 매핑/위치추정 모드, RGB-D 토픽·오도메트리 입력, Reg/Force3DoF |
| move_group + MTC | 제공 | MoveIt 2 | Setup Assistant 설정 패키지, pick_ik, octomap 센서, 속도 스케일 |
| controller_manager, JTC, gripper_controller(parallel_gripper), diff_drive_controller, imu_sensor_broadcaster, joint_state_broadcaster | 제공 | ros2_control | 컨트롤러 YAML, URDF ros2_control 태그 |
| diagnostic_aggregator | 제공 | diagnostics | 노드별 /diagnostics를 계층별로 묶어 /diagnostics_agg |

### 노드 외에 직접 만드는 것

- **커스텀 인터페이스:** 별도 interfaces 패키지 없이 메시지를 내보내는 패키지가 가진다 — perception(TrackedObject(Array), MissionSignal) · plan(SubGoal(Array), TaskStatus) · control(ArmSafetyEvent, MotorStatus) · ml(ScoreGrasps.srv). 필드는 [API.md](../API.md#api-custom)
- **launch 파일:** 전체 기동(bringup), 📦 기동(ros_system), 리매핑 (diff_drive_controller 토픽 → /cmd_vel·/odom, twist_mux 출력 → /cmd_vel_mux), 학습 노드 on/off 인자
- **설정 파일:** 패키지별 config/*.yaml, nav2_params.yaml, ekf.yaml, controllers.yaml, twist_mux.yaml, diagnostics.yaml, collision_monitor 다각형, MoveIt 설정 패키지
- **상태 진단:** 모든 🛠 노드 · 하드웨어 플러그인이 diagnostic_updater로 /diagnostics
- **OpenCR 펌웨어 · 대회 코스 · 운영 CLI:** lv2_module6/firmware · course · tool (run-cli — 상태·진단 상세·수동 조작·모터 재부팅). simulator는 더 쓰지 않음
- **Isaac Lab 환경 코드:** 관측·행동·보상·랜덤화 정의 (ROS 노드 아님)

### 패키지 구성

```text
lv2_module6/
├── urdf/<이름>/urdf.xacro         # 모델 폴더마다 (기본 beebot_mesh, launch urdf:=) — TB3 + OM-X(RM-X52-TNM) + 손목 roll + 카메라 2대 · ros2_control 태그
├── firmware/beebot_opencr/       # OpenCR 펌웨어
├── course/                       # 대회 코스 (미로)
├── ai-service/                   # 호스트 계획 서버 (VLM · 사람) — ROS 아님
├── tool/                         # run-cli 운영 도구
├── simulator/                    # SIL 시뮬레이터 (더 쓰지 않음)
└── ros2_ws/src/
    ├── perception/   # perception · body_filter · signal_decoder + msg
    ├── plan/         # user_input · vlm_task_planner · explorer · coordinator + msg
    ├── ml/           # grasp_scorer · rl_nav_tuner · rl_nav_correction · rl_arm_correction(플러그인) + srv
    ├── control/      # arm_safety_monitor · manual_control · BeebotSystemHardware(플러그인) + msg
    ├── manager/      # state_manager (상태 /state/*)
    ├── ros_system/   # 📦 기동 · config (controllers · ekf · twist_mux) · scan_cam_gate
    ├── bringup/      # 전체 실행 · diagnostics.yaml
    └── faker/        # 테스트베드 대역 · 호스트 파일 연동
```
---

[목차](README.md) · [◀ ros2_control 연결](08-ctrl.md) · [노드 사전 ▶](10-catalog.md) · [원본 페이지](https://dooboostore.github.io/ros2?concept=design&sec=nodes)
