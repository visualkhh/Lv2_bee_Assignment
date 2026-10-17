[목차](README.md) · [◀ 직접작성·제공](09-nodes.md) · [학습 설계 ▶](11-learn.md)

## 노드 사전

노드마다 제공/직접 작성, 패키지, 형태, **받는 것**(구독·서비스 서버·액션 서버·인터페이스)과 **내보내는 것**(발행·호출·요청)을 메시지 타입과 함께 정리했습니다.

> 인터페이스 명세: [커스텀 인터페이스](../API.md#api-custom)

방식: ✉️ 구독/발행 = 토픽 · 📞 서버/호출/응답 = 서비스 · 🎬 서버/요청/결과 = 액션 · 🧩 읽기/쓰기 = ros2_control 인터페이스 · 🔌 장치 = USB·시리얼 · ⌨️ 운영자 → 로봇 · 💬 로봇 → 운영자 · 💾 파일 · ⚙️ 파라미터 · 🌐 외부 API

- **센서·인지**: [📦 realsense2_camera (pole_cam)](#realsense_pole) · [📦 realsense2_camera (gripper_cam)](#realsense_gripper) · [🛠️ perception](#perception) · [📦 robot_state_publisher](#robot_state_publisher)
- **계획·임무**: [🛠️ user_input](#user_input) · [🛠️ vlm_task_planner](#vlm_task_planner) · [🛠️ coordinator](#coordinator) · [🛠️ grasp_scorer](#grasp_scorer) · [🛠️ rl_nav_tuner](#rl_nav_tuner)
- **Nav2·위치추정**: [📦 rtabmap](#rtabmap) · [📦 bt_navigator](#bt_navigator) · [📦 planner_server](#planner_server) · [📦 controller_server](#controller_server) · [📦 behavior_server](#behavior_server) · [📦 costmap_2d (voxel layer)](#costmap) · [📦 lifecycle_manager_navigation](#lifecycle_manager)
- **베이스 명령 체인**: [🛠️ rl_nav_correction](#rl_nav_correction) · [📦 velocity_smoother](#velocity_smoother) · [📦 collision_monitor](#collision_monitor) · [📦 ekf_node (robot_localization)](#ekf) · [📦 teleop_twist_keyboard](#teleop)
- **팔·MoveIt**: [📦 move_group (+ MTC 실행)](#move_group) · [📦 arm_controller (JointTrajectoryController)](#arm_controller) · [🛠️ rl_arm_correction](#rl_arm_correction) · [📦 gripper_controller](#gripper_controller) · [🛠️ arm_safety_monitor](#arm_safety_monitor)
- **ros2_control**: [📦 controller_manager](#controller_manager) · [🛠️ BeebotSystemHardware](#hw_interface) · [🛠️ manual_control](#manual_control) · [📦 twist_mux](#twist_mux) · [📦 diff_drive_controller](#diff_drive_controller) · [📦 imu_sensor_broadcaster](#imu_sensor_broadcaster) · [📦 joint_state_broadcaster](#joint_state_broadcaster)

## 센서·인지

<a id="realsense_pole"></a>
### 📦 제공 · realsense2_camera (pole_cam)

패키지 `realsense2_camera` · 형태 ROS 2 노드

폴대에 고정된 RealSense를 USB 3로 열어 컬러 영상, 컬러에 정렬된 depth, 점구름을 발행한다. 로봇의 "눈" 전체 — 인지·SLAM·장애물·octomap이 모두 여기서 나온다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🔌 장치 | `pole_cam RealSense` | `librealsense2 (USB 3)` | 카메라 하드웨어 | serial_no로 어느 카메라인지 지정 |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 발행 | `/pole_cam/color/image_raw` | `sensor_msgs/Image (rgb8)` | perception, rtabmap | 30Hz · best_effort |
| ✉️ 발행 | `/pole_cam/color/camera_info` | `sensor_msgs/CameraInfo` | perception, rtabmap | 내부 파라미터 (fx, fy, cx, cy) |
| ✉️ 발행 | `/pole_cam/aligned_depth_to_color/image_raw` | `sensor_msgs/Image (16UC1, mm)` | perception, rtabmap | 컬러 픽셀과 1:1 정렬된 depth |
| ✉️ 발행 | `/pole_cam/depth/color/points` | `sensor_msgs/PointCloud2` | costmap_2d, collision_monitor, move_group | 10~15Hz로 다운샘플 권장 |
| ✉️ 발행 | `/tf_static` | `tf2_msgs/TFMessage` | 전체 | pole_cam_link → color/depth optical 프레임 |

**⚙️ 주요 파라미터**

- `serial_no`
- `camera_namespace: ''`
- `camera_name: pole_cam`
- `rgb_camera.color_profile: 640x480x30`
- `depth_module.depth_profile: 640x480x30`
- `align_depth.enable: true`
- `pointcloud.enable: true`
- `decimation_filter.enable: true`

**📝 주의**

- URDF에 카메라 내부 프레임까지 넣었다면 드라이버의 publish_tf를 끄고 하나만 남긴다.
- depth 단위는 16UC1 = 밀리미터. 32FC1(미터)로 바꾸는 노드와 섞지 않는다.

<a id="realsense_gripper"></a>
### 📦 제공 · realsense2_camera (gripper_cam)

패키지 `realsense2_camera` · 형태 ROS 2 노드

그리퍼에 달린 RealSense. 팔과 함께 움직이며 파지 직전의 근접 영상·점구름을 준다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🔌 장치 | `gripper_cam RealSense` | `librealsense2 (USB 3)` | 카메라 하드웨어 | pole_cam과 다른 serial_no |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 발행 | `/gripper_cam/color/image_raw` | `sensor_msgs/Image (rgb8)` | perception | 15Hz · best_effort |
| ✉️ 발행 | `/gripper_cam/color/camera_info` | `sensor_msgs/CameraInfo` | perception |  |
| ✉️ 발행 | `/gripper_cam/aligned_depth_to_color/image_raw` | `sensor_msgs/Image (16UC1, mm)` | perception |  |
| ✉️ 발행 | `/gripper_cam/depth/color/points` | `sensor_msgs/PointCloud2` | grasp_scorer | 근접 파지 점구름 |

**⚙️ 주요 파라미터**

- `serial_no`
- `camera_namespace: ''`
- `camera_name: gripper_cam`
- `rgb_camera.color_profile: 640x480x15`
- `align_depth.enable: true`
- `pointcloud.enable: true`

**📝 주의**

- D435 계열은 최소 측정 거리 약 0.2~0.3m — 그리퍼가퍽에 바짝 붙으면 depth가 빈다. 접근 시작점에서 미리 찍어 두거나 D405 같은 근거리 모델을 쓴다.
- 움직이는 카메라라 점구름을 쓸 땐 영상 시각(header.stamp)의 TF로 변환해야 위치가 맞는다.

<a id="perception"></a>
### 🛠️ 직접 작성 · perception

패키지 `perception` · 형태 ROS 2 노드 (C++)

두 카메라 영상에서 물체를 검출하고, depth로 3D 위치를 구해 map 좌표로 바꾼 뒤 id를 붙여 추적한다. VLM·coordinator·grasp_scorer가 같은 물체를 id로 가리킬 수 있게 하는 게 핵심.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 구독 | `/pole_cam/color/image_raw` | `sensor_msgs/Image` | realsense2_camera (pole_cam) | message_filters로 depth와 시각 동기화 |
| ✉️ 구독 | `/pole_cam/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | realsense2_camera (pole_cam) |  |
| ✉️ 구독 | `/pole_cam/color/camera_info` | `sensor_msgs/CameraInfo` | realsense2_camera (pole_cam) | 픽셀 → 3D 역투영 |
| ✉️ 구독 | `/gripper_cam/color/image_raw` | `sensor_msgs/Image` | realsense2_camera (gripper_cam) | pick 단계에서만 켬 |
| ✉️ 구독 | `/gripper_cam/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | realsense2_camera (gripper_cam) |  |
| ✉️ 구독 | `/tf, /tf_static` | `tf2_msgs/TFMessage` | robot_state_publisher, rtabmap, ekf | 카메라 → map 변환 (영상 시각 기준) |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 발행 | `/tracked_objects` | `perception/TrackedObjectArray` | vlm_task_planner, coordinator, grasp_scorer | 10Hz · reliable |

**⚙️ 주요 파라미터**

- `model_path (검출 모델 ONNX)`
- `conf_threshold: 0.5`
- `track_timeout_s: 3.0`
- `target_frame: map`

**📝 주의**

- 검출 모델은 Isaac Sim Replicator 합성 데이터 + 실사진으로 파인튜닝 (학습 설계 탭).
- 같은 물체가 잠깐 가려져도 id가 유지되도록 track_timeout 동안 기억한다.

<a id="robot_state_publisher"></a>
### 📦 제공 · robot_state_publisher

패키지 `robot_state_publisher` · 형태 ROS 2 노드

URDF와 관절 각도로 로봇 몸의 모든 링크 좌표(TF)를 계산해 낸다. 카메라·팔·그리퍼 위치가 전부 여기서 이어진다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 구독 | `/joint_states` | `sensor_msgs/JointState` | joint_state_broadcaster | 팔·바퀴·그리퍼 관절각 |
| ⚙️ 파라미터 | `robot_description` | `URDF 문자열 (xacro 결과)` | launch | 카메라 2대·ros2_control 태그 포함 |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 발행 | `/tf` | `tf2_msgs/TFMessage` | 전체 | 움직이는 관절 (팔 링크) |
| ✉️ 발행 | `/tf_static` | `tf2_msgs/TFMessage` | 전체 | 고정 링크 (카메라 마운트 등) · transient_local |
| ✉️ 발행 | `/robot_description` | `std_msgs/String` | controller_manager, move_group | transient_local — Jazzy 이후 controller_manager는 이 토픽에서 URDF를 받는다 |

## 계획·임무

<a id="user_input"></a>
### 🛠️ 직접 작성 · user_input

패키지 `plan` · 형태 ROS 2 노드 (C++)

운영자와 로봇 사이의 텍스트 입출력 (외부 입력 노드). 텍스트 명령을 /cmd/user_command로 내보내고, 로봇이 알릴 말(/robot_say)과 진행 상황(/task_status)을 보여 준다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ⌨️ 입력 | `텍스트 명령` | `command 파라미터 · 터미널 표준입력` | 운영자 | 예: "빨간 퍽을 목표 구역으로 옮겨" (시뮬레이터는 /cmd/user_command를 직접 넣기도 함) |
| ✉️ 구독 | `/robot_say` | `std_msgs/String` | coordinator | 알릴 문장 |
| ✉️ 구독 | `/task_status` | `plan/TaskStatus` | coordinator | 진행 상황 표시 |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 발행 | `/cmd/user_command` | `std_msgs/String` | vlm_task_planner | vlm_task_planner가 붙을 때까지 들고 있다가 보냄 |
| ✉️ 발행 | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | diagnostic_aggregator | 못 보낸 명령 수 · 마지막 명령 |
| 💬 알림 | `알림` | `로그 · 화면` | 운영자 | 예: "퍽을 옮겼어요", "퍽을 잡을 수 없어요" |

**⚙️ 주요 파라미터**

- `command` (시작 후 한 번 보낼 명령)

<a id="vlm_task_planner"></a>
### 🛠️ 직접 작성 · vlm_task_planner

패키지 `plan` · 형태 ROS 2 노드 (C++)

운영자 명령과 인식 결과를 VLM에 보내 "무엇을 어떤 순서로" 할지만 정한다. 실행 세부(파지점, 속도)는 결정하지 않는다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 구독 | `/cmd/user_command` | `std_msgs/String` | user_input | 명령마다 |
| ✉️ 구독 | `/tracked_objects` | `perception/TrackedObjectArray` | perception | 최신 목록을 프롬프트에 넣음 |
| ✉️ 구독 | `/pole_cam/color/image_raw` | `sensor_msgs/Image` | realsense2_camera (pole_cam) | 계획할 때 한 장만 |
| ✉️ 구독 | `/task_status` | `plan/TaskStatus` | coordinator | FAILED면 재계획 |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 발행 | `/sub_goals` | `plan/SubGoalArray` | coordinator | reliable + transient_local |
| 🌐 외부 | `VLM API` | `HTTPS (JSON 응답 강제)` | VLM 서비스 | 수 초 걸림 — 제어 루프 밖 |

**⚙️ 주요 파라미터**

- `model`
- `allowed_skills: [navigate, pick, place, say]`
- `max_replans: 2`
- `timeout_s: 20`

**📝 주의**

- 출력은 JSON 스키마로 강제하고, 파싱 실패하면 재요청 1회 후 운영자에게 알린다.

<a id="coordinator"></a>
### 🛠️ 직접 작성 · coordinator

패키지 `plan` · 형태 ROS 2 노드 (C++, BehaviorTree.CPP 권장)

VLM 계획을 검증하고 Nav2·MoveIt에 액션으로 실행을 맡긴다. 이동 전 팔 접기, 정지 후에만 팔 동작, 파지 재시도, 실패 보고 같은 실행 규칙을 지키는 유일한 실행권자.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 구독 | `/sub_goals` | `plan/SubGoalArray` | vlm_task_planner |  |
| ✉️ 구독 | `/tracked_objects` | `perception/TrackedObjectArray` | perception | 대상 id 존재 확인, 목표 좌표 계산 |
| ✉️ 구독 | `/arm_safety_event` | `control/ArmSafetyEvent` | arm_safety_monitor | 접촉 → 단계 실패 처리 |
| ✉️ 구독 | `/tf` | `tf2_msgs/TFMessage` | rtabmap, ekf | map→base_footprint가 나와야 이동 시작 |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🎬 요청 | `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | bt_navigator | feedback: 남은 거리 · result: 성공/실패 |
| 🎬 요청 | `/execute_task_solution` | `moveit_task_constructor_msgs/action/ExecuteTaskSolution` | move_group | MTC 태스크는 이 노드 안에서 계획 |
| 📞 호출 | `/score_grasps` | `ml/srv/ScoreGrasps` | grasp_scorer | pick 직전 1회 |
| 📞 호출 | `/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` | move_group | 퍽·장애물 CollisionObject 등록 |
| 📞 호출 | `/clear_octomap` | `std_srvs/srv/Empty` | move_group | pick 직전 octomap 초기화 |
| 📞 호출 | `/local_costmap/local_costmap/set_parameters` | `rcl_interfaces/srv/SetParameters` | controller_server | 퍽을 들면 footprint 확장 |
| ✉️ 발행 | `/task_status` | `plan/TaskStatus` | vlm_task_planner, user_input | 단계 시작·완료·실패 |
| ✉️ 발행 | `/robot_say` | `std_msgs/String` | user_input | 운영자에게 할 말 — "퍽을 옮겼어요", "퍽을 잡을 수 없어요" |

**⚙️ 주요 파라미터**

- `max_retries_per_step: 2`
- `settle_time_s: 0.5`
- `stow_pose`
- `carry_footprint`

**📝 주의**

- MTC(MoveIt Task Constructor)는 라이브러리라 태스크 계획은 coordinator 프로세스에서, 실행은 move_group의 ExecuteTaskSolution 기능으로 한다.
- 파지 2순위 재시도는 여기서 처리하고, 모두 실패했을 때만 VLM에 재계획을 요청한다.

<a id="grasp_scorer"></a>
### 🛠️ 직접 작성 · grasp_scorer

패키지 `ml` · 형태 ROS 2 서비스 노드 (C++ + ONNX Runtime)

퍽 주변에 파지 후보를 만들고, OM-X가 IK로 닿는 것만 남긴 뒤 학습 모델로 성공 확률을 매겨 높은 순으로 돌려준다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 📞 서버 | `/score_grasps` | `ml/srv/ScoreGrasps` | coordinator | 요청: object_id, max_candidates |
| ✉️ 구독 | `/tracked_objects` | `perception/TrackedObjectArray` | perception | 대상 퍽의 위치·크기 |
| ✉️ 구독 | `/gripper_cam/depth/color/points` | `sensor_msgs/PointCloud2` | realsense2_camera (gripper_cam) | 최신 1장 캐시 |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 📞 응답 | `/score_grasps` | `grasps: PoseStamped[] · scores: float32[]` | coordinator | 점수 높은 순 |
| 📞 호출 | `/compute_ik` | `moveit_msgs/srv/GetPositionIK` | move_group | 후보마다 도달 가능 여부 확인 |

**⚙️ 주요 파라미터**

- `model_path (ONNX)`
- `num_samples: 48`
- `min_score: 0.3`
- `enabled (false면 규칙 기반: 옆에서 수평)`

<a id="rl_nav_tuner"></a>
### 🛠️ 직접 작성 · rl_nav_tuner

패키지 `ml` · 형태 ROS 2 노드 (C++, 규칙 → 필요 시 RL)

주변 상황(통로 폭, 장애물 거리, 목표 거리)을 보고 주행 프리셋 A/B/C 중 하나를 골라 속도 상한과 DWB 운동 파라미터를 바꾼다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 구독 | `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | controller_server | 통로 폭·최근접 장애물 계산 |
| ✉️ 구독 | `/odometry/filtered` | `nav_msgs/Odometry` | ekf | 현재 속도 |
| ✉️ 구독 | `/plan` | `nav_msgs/Path` | planner_server | 목표까지 남은 거리 |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 발행 | `/speed_limit` | `nav2_msgs/SpeedLimit` | controller_server, rl_nav_correction | percentage: true, speed_limit: 100/60/40 |
| 📞 호출 | `/controller_server/set_parameters` | `rcl_interfaces/srv/SetParameters` | controller_server | FollowPath.max_vel_theta, acc_lim_x, decel_lim_x |

**⚙️ 주요 파라미터**

- `rate_hz: 1.0`
- `hysteresis_s: 2.0 (프리셋이 출렁이지 않게)`
- `enabled (false면 프리셋 A 고정)`

**📝 주의**

- DWB critic 가중치는 실행 중 바꿔도 반영되지 않을 수 있어 건드리지 않는다.

## Nav2·위치추정

<a id="rtabmap"></a>
### 📦 제공 · rtabmap

패키지 `rtabmap_slam (rtabmap_ros)` · 형태 ROS 2 노드

RGB-D SLAM. 매핑 모드에서는 지도를 만들어 rtabmap.db에 저장하고, 위치추정 모드에서는 같은 DB로 로봇 위치를 찾아 map→odom을 낸다. Nav2용 2D 지도(/map)도 depth로 직접 만든다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 구독 | `/pole_cam/color/image_raw` | `sensor_msgs/Image` | realsense2_camera (pole_cam) | approx_sync |
| ✉️ 구독 | `/pole_cam/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | realsense2_camera (pole_cam) |  |
| ✉️ 구독 | `/pole_cam/color/camera_info` | `sensor_msgs/CameraInfo` | realsense2_camera (pole_cam) |  |
| ✉️ 구독 | `/odometry/filtered` | `nav_msgs/Odometry` | ekf | 시각 오도메트리 대신 휠+IMU |
| 💾 파일 | `~/.ros/rtabmap.db` | `SQLite DB` | 디스크 | 위치추정 모드에서 읽음 |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 발행 | `/map` | `nav_msgs/OccupancyGrid` | planner_server (global_costmap static layer) | transient_local |
| ✉️ 발행 | `/tf` | `tf2_msgs/TFMessage` | 전체 | map → odom |
| 💾 파일 | `~/.ros/rtabmap.db` | `SQLite DB` | 디스크 | 매핑 모드에서 저장 |
| ✉️ 발행 | `/mapData` | `rtabmap_msgs/MapData` | RViz (rtabmap_viz) | 그래프 시각화용 |

**⚙️ 주요 파라미터**

- `frame_id: base_footprint`
- `subscribe_depth: true`
- `approx_sync: true`
- `Reg/Force3DoF: true`
- `Grid/Sensor: 0 (scan_cloud = points_filtered, 자기 몸 뺀 점구름)`
- `Mem/IncrementalMemory: true(매핑) / false(위치추정)`
- `Mem/InitWMWithAllNodes: true (위치추정)`

**📝 주의**

- Nav2 lifecycle 관리 대상이 아니다. localization용 lifecycle_manager가 필요 없다.
- 위치추정 모드는 시작 직후 장면을 맞출 때까지 map→odom이 없다 — coordinator가 TF를 기다린다.

<a id="bt_navigator"></a>
### 📦 제공 · bt_navigator

패키지 `nav2_bt_navigator` · 형태 ROS 2 lifecycle 노드

NavigateToPose 목표를 받아 행동 트리로 "경로 계산 → 따라가기 → 막히면 복구"를 지휘한다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🎬 서버 | `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | coordinator | goal: PoseStamped (map) |
| ✉️ 구독 | `/odometry/filtered` | `nav_msgs/Odometry` | ekf | odom_topic 파라미터 |
| ✉️ 구독 | `/tf` | `tf2_msgs/TFMessage` | rtabmap, ekf | map→base_footprint |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🎬 요청 | `/compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | planner_server | 약 1Hz 재계획 |
| 🎬 요청 | `/follow_path` | `nav2_msgs/action/FollowPath` | controller_server |  |
| 🎬 요청 | `/spin, /backup, /wait` | `nav2_msgs/action/Spin · BackUp · Wait` | behavior_server | 복구 |

**⚙️ 주요 파라미터**

- `robot_base_frame: base_footprint`
- `default_nav_to_pose_bt_xml (BackUp 제한 버전)`
- `odom_topic: /odometry/filtered`

**📝 주의**

- 카메라가 앞만 보므로 BackUp 복구는 빼거나 0.1m 이하로 제한한 BT XML을 쓴다.

<a id="planner_server"></a>
### 📦 제공 · planner_server

패키지 `nav2_planner` · 형태 ROS 2 lifecycle 노드 (+ global_costmap)

지도와 장애물을 반영한 전역 경로를 만든다. 안에 global_costmap이 들어 있다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🎬 서버 | `/compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | bt_navigator |  |
| ✉️ 구독 | `/map` | `nav_msgs/OccupancyGrid` | rtabmap | static layer |
| ✉️ 구독 | `/pole_cam/depth/color/points_filtered` | `sensor_msgs/PointCloud2` | body_filter (자기 몸 점 제거) | voxel layer |
| ✉️ 구독 | `/tf` | `tf2_msgs/TFMessage` | rtabmap, ekf |  |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 발행 | `/plan` | `nav_msgs/Path` | rl_nav_correction, rl_nav_tuner, RViz | 계획마다 |
| ✉️ 발행 | `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | RViz |  |

**⚙️ 주요 파라미터**

- `planner_plugins: [GridBased] (NavFn 또는 SmacPlanner2D)`
- `global_costmap.plugins: [static_layer, voxel_layer, inflation_layer]`
- `robot_base_frame: base_footprint`

<a id="controller_server"></a>
### 📦 제공 · controller_server

패키지 `nav2_controller` · 형태 ROS 2 lifecycle 노드 (+ local_costmap · DWB)

전역 경로를 따라가는 속도 명령을 20Hz로 계산한다. 안에 local_costmap과 DWB 컨트롤러가 들어 있다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🎬 서버 | `/follow_path` | `nav2_msgs/action/FollowPath` | bt_navigator |  |
| ✉️ 구독 | `/odometry/filtered` | `nav_msgs/Odometry` | ekf |  |
| ✉️ 구독 | `/speed_limit` | `nav2_msgs/SpeedLimit` | rl_nav_tuner | 속도 상한 (백분율) |
| ✉️ 구독 | `/pole_cam/depth/color/points_filtered` | `sensor_msgs/PointCloud2` | body_filter (자기 몸 점 제거) | local voxel layer |
| 📞 서버 | `/controller_server/set_parameters` | `rcl_interfaces/srv/SetParameters` | rl_nav_tuner | kinematic 파라미터 런타임 변경 |
| 📞 서버 | `/local_costmap/local_costmap/set_parameters` | `rcl_interfaces/srv/SetParameters` | coordinator | footprint 변경 |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 발행 | `/cmd_vel_nav` | `geometry_msgs/TwistStamped` | rl_nav_correction | 20Hz |
| ✉️ 발행 | `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | rl_nav_tuner, RViz |  |
| ✉️ 발행 | `/local_costmap/costmap_raw` | `nav2_msgs/Costmap` | behavior_server |  |

**⚙️ 주요 파라미터**

- `controller_frequency: 20.0`
- `FollowPath.plugin: dwb_core::DWBLocalPlanner`
- `FollowPath.max_vel_x: 0.26`
- `enable_stamped_cmd_vel: true`

<a id="behavior_server"></a>
### 📦 제공 · behavior_server

패키지 `nav2_behaviors` · 형태 ROS 2 lifecycle 노드

막혔을 때의 복구 동작(제자리 회전, 후진, 대기)을 실행한다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🎬 서버 | `/spin, /backup, /wait` | `nav2_msgs/action/Spin · BackUp · Wait` | bt_navigator |  |
| ✉️ 구독 | `/local_costmap/costmap_raw` | `nav2_msgs/Costmap` | controller_server | 충돌 검사 |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 발행 | `/cmd_vel_nav` | `geometry_msgs/TwistStamped` | rl_nav_correction | 리매핑해서 안전 체인을 똑같이 지나게 함 |

**⚙️ 주요 파라미터**

- `behavior_plugins: [spin, backup, wait]`
- `backup 최대 거리 0.1m (BT에서 제한)`

<a id="costmap"></a>
### 📦 제공 · costmap_2d (voxel layer)

패키지 `nav2_costmap_2d` · 형태 플러그인 (planner_server·controller_server 안, 노드 아님)

점구름을 3D 격자(voxel)에 쌓아 2D 비용 지도로 내린다. LiDAR가 없어서 장애물은 전부 이 층으로 들어온다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 구독 | `/pole_cam/depth/color/points_filtered` | `sensor_msgs/PointCloud2` | body_filter (자기 몸 점 제거) | observation_sources: pointcloud |
| ✉️ 구독 | `/map` | `nav_msgs/OccupancyGrid` | rtabmap | global만 (static layer) |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🧩 내부 | `비용 격자` | `Costmap2D (메모리)` | planner_server, controller_server | 같은 프로세스 |
| ✉️ 발행 | `/global_costmap/costmap, /local_costmap/costmap` | `nav_msgs/OccupancyGrid` | RViz, rl_nav_tuner |  |

**⚙️ 주요 파라미터**

- `voxel_layer.pointcloud.data_type: PointCloud2`
- `min_obstacle_height: 0.03`
- `max_obstacle_height: 0.40`
- `marking: true, clearing: true`
- `inflation_layer.inflation_radius: 0.25`

**📝 주의**

- 화각 밖 장애물이 바로 지워지지 않게 clearing을 보수적으로 둔다 (STVL로 바꾸면 시간 감쇠 가능).

<a id="lifecycle_manager"></a>
### 📦 제공 · lifecycle_manager_navigation

패키지 `nav2_lifecycle_manager` · 형태 ROS 2 노드

Nav2 주행 계획 서버(planner · controller · behavior · bt_navigator)를 정해진 순서로 configure → activate 하고, 하나가 죽으면 이것들을 내린다. map(→odom)이 있어야 뜬다.

바퀴 명령 체인(velocity_smoother · collision_monitor)은 별도 관리자 **lifecycle_manager_base**가 올린다 — odom만 있으면 뜨므로 RTAB-Map이 아직 map→odom을 못 내도(매핑 시작 직후) 수동 조작은 바퀴까지 간다. 하나로 묶었을 때 global_costmap 실패에 수동 조작까지 막혔음 (실측). 서비스 `/lifecycle_manager_base/manage_nodes` 동일.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 📞 서버 | `/lifecycle_manager_navigation/manage_nodes` | `nav2_msgs/srv/ManageLifecycleNodes` | 운영자·launch | 시작·정지·리셋 |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 📞 호출 | `*/change_state` | `lifecycle_msgs/srv/ChangeState` | bt_navigator, planner_server, controller_server, behavior_server, velocity_smoother, collision_monitor |  |

**⚙️ 주요 파라미터**

- `autostart: true`
- `node_names: [...]`
- `bond_timeout: 4.0`

## 베이스 명령 체인

<a id="rl_nav_correction"></a>
### 🛠️ 직접 작성 · rl_nav_correction

패키지 `ml` · 형태 ROS 2 노드 (C++ + ONNX Runtime)

Nav2 속도 명령에 작은 Δ를 더해 미끄러운 바닥 미끄러짐 같은 현실 오차를 보정한다. 결과는 다시 속도 상한으로 자른다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 구독 | `/cmd_vel_nav` | `geometry_msgs/TwistStamped` | controller_server, behavior_server | 이 메시지가 올 때마다 1회 추론 |
| ✉️ 구독 | `/plan` | `nav_msgs/Path` | planner_server | 경로 대비 횡·방향 오차 |
| ✉️ 구독 | `/odom` | `nav_msgs/Odometry` | diff_drive_controller | 바퀴 속도 |
| ✉️ 구독 | `/imu` | `sensor_msgs/Imu` | imu_sensor_broadcaster | 실제 각속도 |
| ✉️ 구독 | `/speed_limit` | `nav2_msgs/SpeedLimit` | rl_nav_tuner | 재클리핑 상한 |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 발행 | `/cmd_vel_corrected` | `geometry_msgs/TwistStamped` | velocity_smoother | 20Hz |

**⚙️ 주요 파라미터**

- `enabled (false면 Δ=0 통과)`
- `model_path (ONNX)`
- `delta_v_max: 0.05`
- `delta_w_max: 0.10`
- `input_timeout_s: 0.2`

**📝 주의**

- 관측 벡터 순서·단위·정규화가 Isaac Lab 학습 환경과 완전히 같아야 한다.

<a id="velocity_smoother"></a>
### 📦 제공 · velocity_smoother

패키지 `nav2_velocity_smoother` · 형태 ROS 2 lifecycle 노드

속도 명령의 가속·감속을 제한해 덜컹임을 없앤다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 구독 | `/cmd_vel_corrected` | `geometry_msgs/TwistStamped` | rl_nav_correction | 기본 입력 cmd_vel을 리매핑 |
| ✉️ 구독 | `/odometry/filtered` | `nav_msgs/Odometry` | ekf | feedback: CLOSED_LOOP일 때만 |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 발행 | `/cmd_vel_smoothed` | `geometry_msgs/TwistStamped` | collision_monitor | 20Hz |

**⚙️ 주요 파라미터**

- `smoothing_frequency: 20.0`
- `feedback: OPEN_LOOP`
- `max_velocity: [0.26, 0.0, 1.82]`
- `max_accel: [2.5, 0.0, 3.2]`
- `enable_stamped_cmd_vel: true`

<a id="collision_monitor"></a>
### 📦 제공 · collision_monitor

패키지 `nav2_collision_monitor` · 형태 ROS 2 lifecycle 노드

바퀴로 가기 직전의 마지막 안전 필터. 점구름이 다각형 안에 들어오면 감속하거나 멈춘다. RL 출력까지 포함한 최종 명령을 거른다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 구독 | `/cmd_vel_smoothed` | `geometry_msgs/TwistStamped` | velocity_smoother |  |
| ✉️ 구독 | `/pole_cam/depth/color/points` | `sensor_msgs/PointCloud2` | realsense2_camera (pole_cam) | min_height 0.03 · max_height 0.40 |
| ✉️ 구독 | `/tf` | `tf2_msgs/TFMessage` | robot_state_publisher | 카메라 → base_footprint |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 발행 | `/cmd_vel` | `geometry_msgs/TwistStamped` | diff_drive_controller | 20Hz |
| ✉️ 발행 | `/collision_monitor_state` | `nav2_msgs/CollisionMonitorState` | RViz, coordinator | 어떤 다각형이 걸렸는지 |

**⚙️ 주요 파라미터**

- `polygons: [FootprintApproach, SlowDown, Stop]`
- `SlowDown: 0.3m · 40%`
- `Stop: 0.15m`
- `observation_sources: [pointcloud]`

**📝 주의**

- 카메라 화각(약 87°) 밖은 지켜 주지 못한다 — 후진·회전은 짧게.

<a id="ekf"></a>
### 📦 제공 · ekf_node (robot_localization)

패키지 `robot_localization` · 형태 ROS 2 노드

바퀴 오도메트리와 IMU를 융합해 매끄러운 odom→base_footprint를 낸다. 미끄러짐에 대한 확정적 1차 보정.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 구독 | `/odom` | `nav_msgs/Odometry` | diff_drive_controller | odom0: vx, vyaw |
| ✉️ 구독 | `/imu` | `sensor_msgs/Imu` | imu_sensor_broadcaster | imu0: vyaw |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 발행 | `/odometry/filtered` | `nav_msgs/Odometry` | rtabmap, bt_navigator, controller_server, rl_nav_tuner | 50Hz |
| ✉️ 발행 | `/tf` | `tf2_msgs/TFMessage` | 전체 | odom → base_footprint |

**⚙️ 주요 파라미터**

- `two_d_mode: true`
- `frequency: 50.0`
- `world_frame: odom`
- `base_link_frame: base_footprint`
- `publish_tf: true`

**📝 주의**

- diff_drive_controller의 enable_odom_tf는 false — odom TF 발행자는 ekf 하나.

<a id="teleop"></a>
### 📦 제공 · teleop_twist_keyboard

패키지 `teleop_twist_keyboard` · 형태 ROS 2 노드 (터미널)

매핑 단계에서 키보드로 로봇을 몬다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ⌨️ 입력 | `키보드` | `터미널` | 운영자 |  |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 발행 | `/cmd_vel` | `geometry_msgs/TwistStamped` | diff_drive_controller | stamped:=true 필수 (Jazzy 이후) |

**⚙️ 주요 파라미터**

- `stamped: true`
- `frame_id: base_footprint`

**📝 주의**

- 안전을 위해 매핑 때도 collision_monitor 입력(/cmd_vel_smoothed)으로 리매핑해 쓰는 편이 낫다.

## 팔·MoveIt

<a id="move_group"></a>
### 📦 제공 · move_group (+ MTC 실행)

패키지 `moveit_ros_move_group` · 형태 ROS 2 노드

팔 궤적 계획·충돌 검사·IK를 맡고, MTC 태스크 결과를 실행해 컨트롤러에 넘긴다. pole_cam 점구름으로 octomap을 쌓는다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🎬 서버 | `/execute_task_solution` | `moveit_task_constructor_msgs/action/ExecuteTaskSolution` | coordinator | ExecuteTaskSolutionCapability 필요 |
| 📞 서버 | `/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` | coordinator |  |
| 📞 서버 | `/clear_octomap` | `std_srvs/srv/Empty` | coordinator |  |
| 📞 서버 | `/compute_ik` | `moveit_msgs/srv/GetPositionIK` | grasp_scorer | pick_ik |
| ✉️ 구독 | `/joint_states` | `sensor_msgs/JointState` | joint_state_broadcaster | 현재 자세 |
| ✉️ 구독 | `/pole_cam/depth/color/points` | `sensor_msgs/PointCloud2` | realsense2_camera (pole_cam) | octomap (자기 몸 필터링) |
| ✉️ 구독 | `/robot_description` | `std_msgs/String` | robot_state_publisher |  |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🎬 요청 | `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | arm_controller | 궤적마다 |
| 🎬 요청 | `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | gripper_controller | 열기·닫기 |
| ✉️ 발행 | `/monitored_planning_scene` | `moveit_msgs/PlanningScene` | coordinator(MTC), RViz |  |

**⚙️ 주요 파라미터**

- `kinematics_solver: pick_ik (위치 위주)`
- `octomap_resolution: 0.02`
- `sensors: PointCloudOctomapUpdater`
- `max_range: 1.5`
- `default_velocity_scaling: 0.4`

<a id="arm_controller"></a>
### 📦 제공 · arm_controller (JointTrajectoryController)

패키지 `joint_trajectory_controller` · 형태 ros2_control 컨트롤러 플러그인

궤적을 시간에 맞춰 보간해 매 주기 관절 목표를 낸다. 하드웨어가 아니라 rl_arm_correction의 reference에 쓴다(체인). 경로 허용오차를 넘으면 스스로 abort.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🎬 서버 | `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | move_group |  |
| 📞 서버 | `/arm_controller/follow_joint_trajectory/_action/cancel_goal` | `action_msgs/srv/CancelGoal` | arm_safety_monitor | 취소 → 현재 자세 유지 |
| 🧩 읽기 | `joint1~4/position, velocity` | `state interface (double)` | hardware interface | 100Hz |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🧩 쓰기 | `rl_arm_correction/joint1~4/position` | `reference interface (double)` | rl_arm_correction | command_joints로 체인 |
| ✉️ 발행 | `/arm_controller/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` | 모니터링·RViz |  |

**⚙️ 주요 파라미터**

- `joints: [joint1..joint4]`
- `command_joints: [rl_arm_correction/joint1..4]`
- `constraints.joint2.trajectory: 0.10`
- `goal_time: 0.5`

<a id="rl_arm_correction"></a>
### 🛠️ 직접 작성 · rl_arm_correction

패키지 `ml` · 형태 ros2_control chainable 컨트롤러 플러그인 (C++, ml/RlArmCorrection)

JTC가 낸 관절 목표에 작은 Δ를 더해 중력 처짐 같은 오차를 보정하고 하드웨어에 쓴다. ROS 노드가 아니라 controller_manager 안에서 같은 주기에 돈다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🧩 읽기 | `rl_arm_correction/joint1~4/position` | `reference interface (double)` | arm_controller | 자기가 내놓은 입력 자리 |
| 🧩 읽기 | `joint1~4/position, velocity` | `state interface (double)` | hardware interface | 현재 관절각·속도 → 추종 오차 |
| 🧩 읽기 | `gripper/position, gripper/effort` | `state interface (double)` | hardware interface | 파지 여부: 덜 닫혔는데 힘이 걸려 있으면 "들고 있음" (state는 여러 컨트롤러가 함께 읽을 수 있음) |
| ⚙️ 파라미터 | `enabled` | `bool` | ros2 param set | false면 Δ=0 (보정 끔) |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🧩 쓰기 | `joint1~4/position` | `command interface (double)` | hardware interface | reference + Δ, ±0.05rad 클리핑 |
| ✉️ 발행 | `~/delta` | `std_msgs/Float64MultiArray` | 디버그 | realtime_tools::RealtimePublisher |

**⚙️ 주요 파라미터**

- `enabled (false면 Δ=0)`
- `model_path (ONNX)`
- `delta_max: 0.05`
- `joints: [joint1..joint4]`
- `gripper_joint: gripper_left_joint`
- `grasp_effort_threshold (이 힘 이상이면 들고 있음)`
- `gripper_closed_position (이보다 더 닫혔으면 빈손)`

**🔧 직접 연결·설정할 것**

1. ml 패키지에 controller_interface::ChainableControllerInterface를 상속한 RlArmCorrection 클래스를 만든다.
2. on_export_reference_interfaces()로 rl_arm_correction/joint1~4/position 입력 자리를 내놓고, command_interface_configuration()에 joint1~4/position, state_interface_configuration()에 joint1~4/position·velocity·gripper_left_joint/position·effort를 요청한다.
3. update_and_write_commands()에서 reference + Δ(ONNX 추론, ±0.05rad 클리핑)를 command에 쓴다. enabled=false면 Δ=0.
4. pluginlib 등록(플러그인 xml + CMake pluginlib_export_plugin_description_file) → controllers.yaml에 type: ml/RlArmCorrection.
5. arm_controller의 command_joints를 rl_arm_correction/joint1~4로 바꾸고, spawner에서 rl_arm_correction을 arm_controller보다 먼저 켠다.

**📝 주의**

- 관측값은 모두 controller_manager 안의 인터페이스에서 읽는다 — 토픽을 구독하지 않으므로 실시간 루프가 끊기지 않는다.
- 파지 여부 판정 규칙(임계값)은 Isaac Lab 학습 환경의 관측 계산과 똑같이 맞춘다.
- update() 안에서 메모리 할당 금지 — ONNX 세션·텐서는 on_configure에서 미리 만든다.
- 체인 때문에 이것만 deactivate할 수 없다. 끌 땐 enabled 파라미터를 쓴다.

<a id="gripper_controller"></a>
### 📦 제공 · gripper_controller

패키지 `gripper_controllers (GripperActionController)` · 형태 ros2_control 컨트롤러 플러그인

그리퍼를 열고 닫는다. 결과의 위치·멈춤 여부로 "빈손" 판단을 한다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🎬 서버 | `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | move_group (MTC) | goal: position, max_effort |
| 🧩 읽기 | `gripper/position, effort` | `state interface (double)` | hardware interface |  |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🧩 쓰기 | `gripper/position` | `command interface (double)` | hardware interface |  |
| 🎬 결과 | `result` | `position · effort · stalled · reached_goal` | move_group → coordinator | 완전히 닫힘에 가까우면 빈손 |

**📝 주의**

- Jazzy 이후로는 parallel_gripper_action_controller로 옮겨 가는 추세.

<a id="manual_control"></a>
### 🛠️ 직접 작성 · manual_control

패키지 `control` · 형태 ROS 2 노드 (C++)

수동 조작. /state/operation_mode(state_manager 발행)가 MANUAL이면 운영자의 바퀴·팔 명령을 실행 계층에 넘긴다 (모드 결정은 manager/state_manager). 수동 바퀴 명령도 안전 체인(twist_mux → velocity_smoother → collision_monitor)을 지난다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 구독 | `/state/operation_mode` | `std_msgs/String (transient_local)` | state_manager | MANUAL일 때만 수동 명령 통과 |
| ✉️ 구독 | `/cmd/manual/cmd_vel` | `geometry_msgs/TwistStamped` | teleop_twist_keyboard · 시뮬레이터 | 수동일 때만 넘김, 0.5초 끊기면 0 |
| ✉️ 구독 | `/cmd/manual/arm` | `sensor_msgs/JointState` | 시뮬레이터 · 운영자 | joint1~5 rad · gripper_left_joint m |
| ✉️ 구독 | `/joint_states` | `sensor_msgs/JointState` | joint_state_broadcaster | 목표에 없는 관절은 지금 각으로 채움 |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 발행 | `/cmd_vel_manual` | `geometry_msgs/TwistStamped` | twist_mux | 자동보다 우선 |
| ✉️ 발행 | `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | arm_controller (JTC) | 5축 전체 한 점 |
| 🎬 요청 | `/gripper_controller/gripper_cmd` | `control_msgs/ParallelGripperCommand` | gripper_controller |  |
| ✉️ 발행 | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | diagnostic_aggregator | 모드 · 명령 수 |

<a id="twist_mux"></a>
### 📦 제공 · twist_mux

패키지 `twist_mux` · 형태 ROS 2 노드

바퀴 속도 명령 고르기. 우선순위가 높은 입력이 이기고, 잠금 토픽이 켜지면 모두 막는다 (비상정지). 출력은 velocity_smoother 입력.

| 방식 | 이름 | 메시지 타입 / 형식 | 상대 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 구독 | `/cmd_vel_corrected` | `geometry_msgs/TwistStamped` | rl_nav_correction | 자동, priority 10 |
| ✉️ 구독 | `/cmd_vel_manual` | `geometry_msgs/TwistStamped` | manual_control | 수동, priority 100 |
| ✉️ 구독 | `/state/emergency` | `std_msgs/Bool` | state_manager (`/cmd/e_stop` 그대로) | 잠금 priority 255 — true면 전부 차단 |
| ✉️ 발행 | `/cmd_vel_mux` | `geometry_msgs/TwistStamped` | velocity_smoother |  |

<a id="arm_safety_monitor"></a>
### 🛠️ 직접 작성 · arm_safety_monitor

패키지 `control` · 형태 ROS 2 노드 (C++)

모터 전류로 사람 손 같은 예상 못 한 접촉을 감지해 궤적을 취소하고 coordinator에 알린다. 추종 오차 감지는 JTC 내장 기능이 맡는다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 구독 | `/joint_states` | `sensor_msgs/JointState` | joint_state_broadcaster | effort 필드 = 전류 |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 📞 호출 | `/arm_controller/follow_joint_trajectory/_action/cancel_goal` | `action_msgs/srv/CancelGoal` | arm_controller | 빈 goal_id = 전체 취소 |
| ✉️ 발행 | `/arm_safety_event` | `control/ArmSafetyEvent` | coordinator | 접촉 시 |

**⚙️ 주요 파라미터**

- `current_threshold_a (관절별, 퍽 든 상태 포함 정상값보다 충분히 높게)`
- `hold_time_s: 0.1`
- `enabled`

**📝 주의**

- effort(전류)는 OpenCR 펌웨어가 S 상태로 보내고 BeebotSystemHardware가 effort state로 낸다.

## ros2_control

<a id="controller_manager"></a>
### 📦 제공 · controller_manager

패키지 `controller_manager (ros2_control_node)` · 형태 ROS 2 노드 (실시간 루프)

하드웨어 인터페이스와 모든 컨트롤러를 한 프로세스에 올려 100Hz로 read → update → write를 돈다. 팔·바퀴·그리퍼·IMU가 같은 OpenCR이라 하나만 둔다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 구독 | `/robot_description` | `std_msgs/String` | robot_state_publisher | URDF 속 <ros2_control> 태그 → 어떤 플러그인·관절·인터페이스를 쓸지 |
| ⚙️ 파라미터 | `controllers.yaml` | `YAML 파일 (--params-file)` | launch | update_rate, 컨트롤러 이름·타입, 컨트롤러별 파라미터 |
| 📞 서버 | `/controller_manager/load_controller` | `controller_manager_msgs/srv/LoadController` | spawner | 컨트롤러 플러그인 로드 |
| 📞 서버 | `/controller_manager/configure_controller` | `controller_manager_msgs/srv/ConfigureController` | spawner | on_configure (파라미터 읽기) |
| 📞 서버 | `/controller_manager/switch_controller` | `controller_manager_msgs/srv/SwitchController` | spawner·운영자 | 활성화/비활성화 (체인은 뒤에서부터) |
| 📞 서버 | `/controller_manager/list_controllers` | `controller_manager_msgs/srv/ListControllers` | ros2 control CLI | 상태 확인 |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🧩 호출 | `read() / write()` | `hardware interface 플러그인` | hardware interface | 100Hz — 장치 통신은 플러그인이 함 |
| 🧩 호출 | `update()` | `컨트롤러 플러그인` | arm_controller, rl_arm_correction, gripper_controller, diff_drive_controller, imu_sensor_broadcaster, joint_state_broadcaster | 체인 순서대로 |
| ✉️ 발행 | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 모니터링 (rqt_robot_monitor) | 루프 주기·실행 시간 경고 |

**⚙️ 주요 파라미터**

- `update_rate: 100`
- `<컨트롤러이름>.type (6개, 아래 단계 2)`

**🔧 직접 연결·설정할 것**

1. URDF(lv2_module6/urdf/beebot.urdf.xacro)에 <ros2_control name="BeebotSystem" type="system"> 블록 — <plugin>control/BeebotSystemHardware</plugin>, <param> mock · mock_effort · port, 그리고 관절(wheel_left/right_joint, joint1~5, gripper_left_joint)·센서(imu)별 command/state 인터페이스 (팔·그리퍼에 effort).
2. controllers.yaml의 controller_manager.ros__parameters에 update_rate: 100과 컨트롤러 6개의 type: joint_state_broadcaster/JointStateBroadcaster · imu_sensor_broadcaster/IMUSensorBroadcaster · diff_drive_controller/DiffDriveController · ml/RlArmCorrection · joint_trajectory_controller/JointTrajectoryController · parallel_gripper_action_controller/GripperActionController. 컨트롤러별 파라미터는 같은 파일의 <이름>.ros__parameters 아래에.
3. launch(ros_system): robot_state_publisher(URDF 발행) → ros2_control_node(parameters=[controllers.yaml]) → spawner(--param-file controllers.yaml, --controller-ros-args로 ~/cmd_vel·~/odom·~/imu 리매핑 — ros2_control_node에 준 리매핑은 컨트롤러에 안 먹음)로 joint_state_broadcaster, imu_sensor_broadcaster, diff_drive_controller, rl_arm_correction, arm_controller, gripper_controller 순서 (체인은 rl_arm_correction이 arm_controller보다 먼저).
4. 확인: ros2 control list_hardware_components → list_hardware_interfaces([claimed] 표시, joint2/effort가 보이는지) → list_controllers(전부 active) → view_controller_chains.
5. 장치를 바꾸거나 추가할 때는 controller_manager가 아니라 URDF의 <plugin>과 <param>을 바꾼다 (장치마다 <ros2_control> 블록 하나).

**📝 주의**

- controller_manager는 장치를 모른다 — 장치별 연결·전송은 hardware interface 플러그인이 맡고, controller_manager는 read/update/write만 돌린다.
- URDF는 /robot_description 토픽(robot_state_publisher)에서 받는다 (Jazzy 이후).

<a id="hw_interface"></a>
### 🛠️ 직접 작성 · BeebotSystemHardware (+ OpenCR 펌웨어 beebot_opencr)

패키지 `control` · 형태 ros2_control SystemInterface 플러그인 (C++) — 짝: `lv2_module6/firmware/beebot_opencr` (Arduino)

controller_manager의 변수(double)를 OpenCR과 주고받는다. OpenCR 펌웨어가 다이나믹셀 8개(바퀴 2 · 팔 5 · 그리퍼)와 내장 IMU를 맡고 단위 변환(rad ↔ 서보 각도·rpm)도 한다.
호스트 ↔ OpenCR은 USB 시리얼 ASCII 줄 프로토콜: `C,<바퀴L>,<바퀴R>,<j1..j5>,<그리퍼>` (명령 8개) · `S,…` (상태 32개 — 바퀴 pos·vel, 관절 pos·vel·전류, 그리퍼, IMU).

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🧩 읽기 | `wheel_left/right_joint/velocity` | `command interface (double, rad/s)` | diff_drive_controller |  |
| 🧩 읽기 | `joint1~5/position` | `command interface (double, rad)` | rl_arm_correction | 체인 끝 — JTC 목표 + Δ (joint5 = 손목 roll) |
| 🧩 읽기 | `gripper_left_joint/position` | `command interface (double, m)` | gripper_controller | gripper_right_joint는 URDF mimic |
| 🔌 장치 | `OpenCR S 상태` | `USB 시리얼 ASCII (/dev/ttyACM0)` | OpenCR 펌웨어 | 25ms마다 |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🧩 쓰기 | `wheel_*_joint/position·velocity` | `state interface (double)` | diff_drive_controller, joint_state_broadcaster |  |
| 🧩 쓰기 | `joint1~5 · gripper_left_joint/position·velocity·effort` | `state interface (double)` | arm_controller, rl_arm_correction, gripper_controller, joint_state_broadcaster | effort = 서보 전류 [A] |
| 🧩 쓰기 | `imu/orientation·angular_velocity·linear_acceleration` | `state interface (double)` | imu_sensor_broadcaster |  |
| 🔌 장치 | `OpenCR C 명령` | `USB 시리얼 ASCII` | OpenCR 펌웨어 | 매 주기 (100Hz) |
| ✉️ 발행 | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | diagnostic_aggregator | "OpenCR 연결": 마지막 S 수신 · 송수신 줄 수 · 해석 실패 |

**⚙️ 주요 파라미터 (URDF `<ros2_control><hardware><param>`)**

- `mock` (true면 명령을 상태로 되돌림 — 테스트베드)
- `mock_effort`
- `port: /dev/ttyACM0`

**📝 주의**

- 프로토콜을 바꾸면 펌웨어(beebot_opencr.ino)와 플러그인(beebot_system_hardware.cpp)을 같이 바꾼다 — 테스트베드 test-hardware · test-firmware가 확인.
- 모터 ID: 바퀴 1·2, 팔 11~14, 손목 roll 15, 그리퍼 16 (OM-X 기본 그리퍼 15에서 옮김).
- ROBOTIS turtlebot3_manipulation 공식 플러그인·펌웨어는 손목 축이 없어 쓰지 않는다.

<a id="diff_drive_controller"></a>
### 📦 제공 · diff_drive_controller

패키지 `diff_drive_controller` · 형태 ros2_control 컨트롤러 플러그인

(선속도, 각속도)를 좌·우 바퀴 속도로 바꾸고, 바퀴 회전량으로 오도메트리를 계산한다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 구독 | `/cmd_vel` | `geometry_msgs/TwistStamped` | collision_monitor (매핑 땐 teleop) | 기본 ~/cmd_vel을 리매핑 |
| 🧩 읽기 | `wheel_left/right position, velocity` | `state interface (double)` | hardware interface |  |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🧩 쓰기 | `wheel_left/right/velocity` | `command interface (double, rad/s)` | hardware interface |  |
| ✉️ 발행 | `/odom` | `nav_msgs/Odometry` | ekf, rl_nav_correction | 50Hz · 기본 ~/odom을 리매핑 |

**⚙️ 주요 파라미터**

- `wheel_separation: 0.287`
- `wheel_radius: 0.033`
- `base_frame_id: base_footprint`
- `enable_odom_tf: false`
- `cmd_vel_timeout: 0.5`
- `publish_rate: 50.0`

<a id="imu_sensor_broadcaster"></a>
### 📦 제공 · imu_sensor_broadcaster

패키지 `imu_sensor_broadcaster` · 형태 ros2_control 브로드캐스터 플러그인

OpenCR IMU의 state interface를 Imu 메시지로 발행한다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🧩 읽기 | `imu/orientation·angular_velocity·linear_acceleration` | `state interface (double)` | hardware interface |  |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 발행 | `/imu` | `sensor_msgs/Imu` | ekf, rl_nav_correction | 100Hz · best_effort · 기본 ~/imu를 리매핑 |

**⚙️ 주요 파라미터**

- `sensor_name: imu`
- `frame_id: imu_link`

<a id="joint_state_broadcaster"></a>
### 📦 제공 · joint_state_broadcaster

패키지 `joint_state_broadcaster` · 형태 ros2_control 브로드캐스터 플러그인

모든 관절의 state interface를 JointState로 발행한다. TF·MoveIt·안전 감시가 이걸 본다.

**⬅ 받는 것 (입력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 보내는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| 🧩 읽기 | `모든 관절 position·velocity·effort` | `state interface (double)` | hardware interface |  |

**➡ 내보내는 것 (출력)**

| 방식 | 이름 | 메시지 타입 / 형식 | 받는 쪽 | 비고 |
| --- | --- | --- | --- | --- |
| ✉️ 발행 | `/joint_states` | `sensor_msgs/JointState` | robot_state_publisher, move_group, arm_safety_monitor | 100Hz |
| ✉️ 발행 | `/dynamic_joint_states` | `control_msgs/msg/DynamicJointState` | 디버그 |  |

## 커스텀 인터페이스 정의 (메시지를 내보내는 패키지가 가짐)

> 정본: [API.md 커스텀 인터페이스](../API.md#api-custom) — 아래는 발췌. 파일과 다르면 파일이 맞습니다.

**perception/msg/TrackedObject.msg**

```text
uint32 id
string class_name
float32 confidence
geometry_msgs/PoseWithCovariance pose   # map 기준
geometry_msgs/Vector3 size              # m
builtin_interfaces/Time last_seen
```

**perception/msg/TrackedObjectArray.msg**

```text
std_msgs/Header header                   # frame_id: map
TrackedObject[] objects
```

**plan/msg/SubGoal.msg**

```text
uint32 step
string skill        # navigate | pick | place | say
string target       # 물체 id 또는 "5"
string relation     # front | on | ...
float32 offset_m
```

**plan/msg/SubGoalArray.msg**

```text
std_msgs/Header header
uint32 plan_id
SubGoal[] goals
```

**plan/msg/TaskStatus.msg**

```text
uint8 RUNNING=0
uint8 SUCCEEDED=1
uint8 FAILED=2
uint32 plan_id
uint32 step
uint8 state
string reason       # grasp_failed_all | nav_blocked | arm_contact ...
```

**control/msg/ArmSafetyEvent.msg**

```text
std_msgs/Header header
string joint_name
float32 current_a
float32 threshold_a
string reason       # overcurrent
```

**ml/srv/ScoreGrasps.srv**

```text
uint32 object_id
uint8 max_candidates
---
geometry_msgs/PoseStamped[] grasps   # IK 통과한 것만, 점수 높은 순
float32[] scores                     # 0~1, grasps와 같은 순서
bool success
string message
```


---

[목차](README.md) · [◀ 직접작성·제공](09-nodes.md) · [학습 설계 ▶](11-learn.md) · [원본 페이지](https://dooboostore.github.io/ros2?concept=design&sec=catalog)
