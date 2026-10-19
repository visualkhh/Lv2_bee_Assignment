[목차](README.md) · [◀ ros2_control 연결](08-ctrl.md) · [노드 사전 ▶](10-catalog.md)

## 직접 작성 노드와 제공 노드

직접 작성은 노드 10개 + 하드웨어 플러그인(BeebotSystemHardware) + OpenCR 펌웨어입니다. 나머지는 생태계 패키지를 가져와 설정하고 연결하는 일입니다.
구현 위치: `lv2_module6/ros2_ws/src/<패키지>` — 전부 C++ (ament_cmake).

> 인터페이스 명세: [커스텀 인터페이스](../API.md#api-custom)

| 노드 | 구분 | 패키지 · 형태 | 핵심 작업 |
| --- | --- | --- | --- |
| perception | 직접 | perception · ROS 2 노드 | 퍽·목표 구역·장애물 검출, 3D 위치, ID 추적, /tracked_objects |
| user_input | 직접 | plan · ROS 2 노드 | 운영자 텍스트 명령 → /cmd/user_command, /robot_say 표시 |
| vlm_task_planner | 직접 | plan · ROS 2 노드 | VLM 호출, 프롬프트, JSON 출력 파싱 → /sub_goals |
| explorer | 직접 | plan · ROS 2 노드 (C++) | 자동 탐사 — 조작 모드 EXPLORE일 때 /map 프론티어(빈 곳·모르는 곳 경계)를 골라 Nav2 /navigate_to_pose로 차례로 보냄, 끝나면 /explore/result |
| coordinator | 직접 | plan · ROS 2 노드 (BT 권장: BehaviorTree.CPP) | 계획 검증, 액션 호출, 실행 규칙, 재시도, MTC 태스크 구성, 수동 모드면 멈춤 |
| grasp_scorer | 직접 | ml · ROS 2 서비스 노드 | 모델 로드, IK 가능 후보만 채점, /score_grasps 응답 |
| rl_nav_tuner | 직접 | ml · ROS 2 노드 | 상황 요약 → 프리셋 선택 → /speed_limit + set_parameters |
| rl_nav_correction | 직접 | ml · ROS 2 노드 | 관측 구성, 정책 추론, Δ 제한, 상한 클리핑, 끊김 시 Δ=0 |
| rl_arm_correction | 직접 | ml · ros2_control chainable controller | reference에 Δ 더하기, enabled 파라미터 |
| arm_safety_monitor | 직접 | control · ROS 2 노드 | 전류로 접촉 감지, 궤적 취소, coordinator 보고 |
| manual_control | 직접 | control · ROS 2 노드 | 수동 모드(/state/operation_mode), 원격 조종 → twist_mux, 팔·그리퍼 직접 명령 |
| BeebotSystemHardware | 직접 | control · ros2_control SystemInterface 플러그인 | OpenCR와 USB 시리얼(C 명령 · S 상태), mock 모드 |
| body_filter | 직접 | control · ROS 2 노드 | pole_cam 점구름에서 자기 몸(URDF collision) 점 제거 → Nav2 |
| state_manager | 직접 | manager · ROS 2 노드 | 로봇 상태의 주인 — 판단해서 /state/<이름>만 발행 (조작 모드 · 지도 모드, RTAB-Map 모드 전환) |
| (OpenCR 펌웨어) beebot_opencr | 직접 | lv2_module6/firmware · Arduino | 다이나믹셀 8개 + IMU, 단위 변환, watchdog |
| realsense2_camera ×2 (pole_cam, gripper_cam) | 제공 | Intel RealSense ROS 2 래퍼 | 장치를 USB 3로 열고 토픽 발행 — serial_no, camera_name, 해상도·fps, align_depth, pointcloud 켜기 |
| robot_state_publisher | 제공 | ROS 2 기본 | URDF(urdf/<이름>/urdf.xacro — launch `urdf:=`, 기본 beebot_mesh) |
| robot_localization (ekf) | 제공 | ROS 2 패키지 | /odom + /imu 융합, odom TF 발행 |
| twist_mux | 제공 | ROS 2 패키지 | 자동 /cmd_vel_corrected · 수동 /cmd_vel_manual 고르기, /state/emergency 잠금(/cmd/e_stop → state_manager) → /cmd_vel_mux |
| bt_navigator, planner_server, controller_server, behavior_server, velocity_smoother, collision_monitor, lifecycle_manager | 제공 | Nav2 | nav2_params.yaml (voxel layer에 점구름, velocity_smoother 입력 /cmd_vel_mux), 리매핑 |
| rtabmap | 제공 | rtabmap_ros | 매핑/위치추정 모드, RGB-D 토픽·오도메트리 입력, Reg/Force3DoF |
| move_group + MTC | 제공 | MoveIt 2 | Setup Assistant 설정 패키지, pick_ik, octomap 센서, 속도 스케일 |
| controller_manager, JTC, gripper_controller(parallel_gripper), diff_drive_controller, imu_sensor_broadcaster, joint_state_broadcaster | 제공 | ros2_control | 컨트롤러 YAML, URDF ros2_control 태그 |
| diagnostic_aggregator | 제공 | diagnostics | 노드별 /diagnostics를 계층별로 묶어 /diagnostics_agg |

### 노드 외에 직접 만드는 것

- **커스텀 인터페이스:** 별도 interfaces 패키지 없이 메시지를 내보내는 패키지가 가진다 — perception(TrackedObject(Array)) · plan(SubGoal(Array), TaskStatus) · control(ArmSafetyEvent) · ml(ScoreGrasps.srv)
- **launch 파일:** 전체 기동(bringup), 📦 기동(ros_system), 리매핑 (diff_drive_controller 토픽 → /cmd_vel·/odom, twist_mux 출력 → /cmd_vel_mux), 학습 노드 on/off 인자
- **설정 파일:** 패키지별 config/*.yaml, nav2_params.yaml, ekf.yaml, controllers.yaml, twist_mux.yaml, diagnostics.yaml, collision_monitor 다각형, MoveIt 설정 패키지
- **상태 진단:** 모든 🛠 노드 · 하드웨어 플러그인이 diagnostic_updater로 /diagnostics
- **OpenCR 펌웨어 · 대회 코스 · SIL 시뮬레이터:** lv2_module6/firmware · course · simulator
- **Isaac Lab 환경 코드:** 관측·행동·보상·랜덤화 정의 (ROS 노드 아님)

### 패키지 구성

```text
lv2_module6/
├── urdf/<이름>/urdf.xacro         # 모델 폴더마다 (기본 beebot_mesh, launch urdf:=) — TB3 + OM-X(RM-X52-TNM) + 손목 roll + 카메라 2대 · ros2_control 태그
├── firmware/beebot_opencr/       # OpenCR 펌웨어
├── course/                       # 대회 코스 (미로)
├── simulator/                    # SIL 시뮬레이터
└── ros2_ws/src/
    ├── perception/   # perception + msg
    ├── plan/         # user_input · vlm_task_planner · coordinator + msg
    ├── ml/           # grasp_scorer · rl_nav_tuner · rl_nav_correction · rl_arm_correction(플러그인) + srv
    ├── control/      # arm_safety_monitor · manual_control · body_filter · BeebotSystemHardware(플러그인) + msg
    ├── manager/      # state_manager (상태 /state/*)
    ├── ros_system/   # 📦 기동 · config (controllers · ekf · twist_mux)
    ├── bringup/      # 전체 실행 · diagnostics.yaml
    └── faker/        # 테스트베드 대역 · 호스트 파일 연동
```
---

[목차](README.md) · [◀ ros2_control 연결](08-ctrl.md) · [노드 사전 ▶](10-catalog.md) · [원본 페이지](https://dooboostore.github.io/ros2?concept=design&sec=nodes)
