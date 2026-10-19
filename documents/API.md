# API 명세서 — 토픽·서비스·액션

협업용 인터페이스 명세입니다. 설계 근거는 [ros2-design](ros2-design/README.md) (특히 [노드 사전](ros2-design/10-catalog.md)), 구현은 [lv2_module6](../lv2_module6/README.md)입니다.
커스텀 타입 원문은 `.msg`·`.srv` 파일이 정본입니다 — 아래 정의와 파일이 다르면 **파일이 맞습니다**.

표기: ✉️ 토픽 · 🎬 액션 · 📞 서비스 · 🧩 ros2_control 내부 인터페이스 · 🔌 장치. QoS 규칙은 [§7](ros2-design/07-tf.md) 요약.

<a id="api-toc"></a>
## 목차

1. [커스텀 인터페이스 정의](#api-custom)
2. [토픽 목록](#api-topics)
3. [액션 목록](#api-actions)
4. [서비스 목록](#api-services)
5. [TF](#api-tf)
6. [QoS 규칙](#api-qos)
7. [바꿀 때 절차](#api-change)

<a id="api-custom"></a>
## 1. 커스텀 인터페이스 정의

별도 interfaces 패키지 없이 **그 메시지를 내보내는 패키지**가 가집니다 ([DEVELOP.md](../lv2_module6/develop/DEVELOP.md#4-커스텀-메시지)).
C++ 헤더는 타입명을 snake_case로: `TrackedObjectArray` → `perception/msg/tracked_object_array.hpp`.

<a id="api-msg-trackedobject"></a>
### perception/msg/TrackedObject ([파일](../lv2_module6/ros2_ws/src/perception/msg/TrackedObject.msg))

```text
uint32 id
string class_name
float32 confidence
geometry_msgs/PoseWithCovariance pose   # map 기준 — 두 카메라 융합 위치 + 위치 공분산 (필드 뜻: documents/API.md)
geometry_msgs/Vector3 size              # m — x·y = 카메라 영상 박스 가로·세로 폭 환산 (map 축 아님), z = 클래스 기본값
builtin_interfaces/Time last_seen       # 마지막 관측 영상 시각
string camera                           # 마지막으로 본 카메라 (pole_cam | gripper_cam) — VLM 이미지 마킹용
int32[2] px                             # 그 카메라 영상 속 박스 중심 픽셀 [u, v]
string color                            # 퍽: red | orange | yellow | green | blue | purple | white | black | unknown · 신호등 상태: red | yellow | green | off · 그 외 빈 값 (박스 가운데 HSV)
```

물체 하나 = 추적 트랙 하나. 카메라마다 YOLO 박스 → 박스 중심 픽셀 + 박스 안 정렬 depth 중앙값으로 3D 점 1개 → 영상 시각의 TF로 map 변환 →
**자기 몸이면 버림** — 3D: 팔 링크를 잇는 선분 6cm 안 · 차체 반경 0.2m 안 바닥 위 / 영상: 팔(TF 링크를 화면에 투영, 두께 4cm)과 겹치는 박스(퍽 제외 — 박스 일부만 팔이면 depth가 팔 뒤 바닥이라 3D로는 못 거름) → 같은 클래스·0.4m 안의 트랙에 1:1로 붙여 **칼만 갱신(두 카메라 융합)**. 포인트 클라우드는 쓰지 않는다 (객체 단위 융합). 구현: [perception_node.py](../lv2_module6/ros2_ws/src/perception/scripts/perception_node.py), 파라미터: [perception.yaml](../lv2_module6/ros2_ws/src/perception/config/perception.yaml).

| 필드 | 뜻 | 주의 |
| --- | --- | --- |
| `id` | 추적 번호 (1부터 증가) — VLM·coordinator·grasp_scorer가 같은 물체를 이 번호로 가리킴 (`SubGoal.target`) | 어느 카메라에도 **2초** 안 보이면 트랙 삭제 → 다시 보이면 **새 번호**. 번호는 재사용 안 함 |
| `class_name` | `puck` · `puck_lying` · `goal_zone` · `station_zone` · `qr_board` · `obstacle` · `person` · `traffic_light` | YOLO 모델의 `names`를 따름 (학습 클래스가 바뀌면 같이 바뀜). 다른 클래스끼리는 가까워도 합치지 않음 |
| `confidence` | 마지막 관측의 YOLO 신뢰도 (0~1, `confidence` 파라미터 이상만) | 평균 아님 |
| `pose.pose.position` | map 좌표 [m] — 원점 = map 원점(시뮬레이터: 출발 칸 `spawn`), x 앞 · y 왼쪽 · z 위(바닥 ≈ 0). **추적은 odom 기준, 발행 순간의 map ← odom(RTAB-Map 최신 보정)으로 바꿔 냄** — 보정이 바뀌면(맞든 틀리든) 로봇 위치와 함께 옮겨져 로봇과의 상대 위치는 유지. 위치추정 자체가 틀리면 map 좌표는 그만큼 통째로 어긋남 | 박스 중심 방향의 **카메라 쪽 앞면 점** — 실제 중심보다 카메라 쪽으로 물체 반지름쯤 당겨짐 (퍽 2~3cm). z도 앞면 점 높이. 두 카메라가 다른 방향에서 보면 일부 상쇄 |
| `pose.pose.orientation` | 항상 `(0, 0, 0, 1)` | 방향은 추정하지 않음 |
| `pose.covariance` | 6×6 행 우선. **위치 3×3 블록**(인덱스 0~2·6~8·12~14) = 융합 위치 불확실성 [m²], 회전 대각(21·28·35) = 1e3(모름) | 관측 잡음만 반영 — 위의 앞면 쏠림(바이어스)은 안 들어감. 관측 σ: depth 축 `depth_sigma0 + depth_sigma_k·z²`, 가로·세로 `pixel_sigma·z/f` |
| `size` | [m] x = 박스 가로 폭 · y = 박스 세로 높이 (픽셀 × 거리 / 초점거리), z = 클래스 기본값(퍽 0.04 등) | **map 축 크기가 아님** (카메라 영상 기준). 관측마다 칼만 이득 비율로 갱신 |
| `last_seen` | 마지막으로 관측된 **영상 시각** (카메라 header.stamp) | `TrackedObjectArray.header.stamp`(발행 시각)과 다름 |
| `camera` | **마지막으로** 본 카메라 `pole_cam` · `gripper_cam` | 두 카메라가 융합한 물체여도 하나만 — 어느 카메라들이 봤는지는 `/perception/<cam>/detections`의 같은 `id`로 |
| `px` | `camera` 영상(640×480) 속 박스 중심 픽셀 `[u, v]` — VLM이 그 영상에 표시할 때 씀 | 다른 카메라 영상의 좌표가 아님 |
| `color` | 퍽: `red` · `orange` · `yellow` · `green` · `blue` · `purple` · `white` · `black` · `unknown` / 신호등: `red` · `yellow` · `green` · `off` / 그 외 `""` | 박스 가운데(가장자리 20%씩 뺌 — 바닥 안 섞이게) HSV 다수결, 채도 없는 화소가 대부분이면 흰색·검정. 마지막 관측 값 |

<a id="api-msg-trackedobjectarray"></a>
### perception/msg/TrackedObjectArray ([파일](../lv2_module6/ros2_ws/src/perception/msg/TrackedObjectArray.msg))

```text
std_msgs/Header header                   # frame_id: map
TrackedObject[] objects
```

- 냄새: perception → `/tracked_objects` (10Hz · reliable). 받는 쪽: vlm_task_planner · coordinator · grasp_scorer.
- `header.stamp` = **발행 시각**(10Hz 타이머), `header.frame_id` = `map`. `objects` = 지금 살아 있는 트랙 전부 (id 오름차순, 없으면 빈 배열).
- 같은 타입을 `/perception/<cam>/detections`로도 낸다 — **융합 전** 그 카메라 한 프레임 검출. `id` = 그 검출이 붙은 트랙 id, `pose.covariance` = 그 관측의 공분산, `header.stamp` = 영상 시각. 디버그·시뮬레이터 표시용 (받는 노드 없음).

<a id="api-msg-subgoal"></a>
### plan/msg/SubGoal ([파일](../lv2_module6/ros2_ws/src/plan/msg/SubGoal.msg))

```text
uint32 step
string skill        # navigate | pick | place | say
string target       # 물체 id (/tracked_objects의 id)
string relation     # front | on | ...
float32 offset_m
```

<a id="api-msg-subgoalarray"></a>
### plan/msg/SubGoalArray ([파일](../lv2_module6/ros2_ws/src/plan/msg/SubGoalArray.msg))

```text
std_msgs/Header header
uint32 plan_id
SubGoal[] goals
```

- 냄새: vlm_task_planner → `/sub_goals` (reliable + transient_local). 받는 쪽: coordinator.

<a id="api-msg-taskstatus"></a>
### plan/msg/TaskStatus ([파일](../lv2_module6/ros2_ws/src/plan/msg/TaskStatus.msg))

```text
uint8 RUNNING=0
uint8 SUCCEEDED=1
uint8 FAILED=2
uint32 plan_id
uint32 step
uint8 state
string reason       # grasp_failed_all | nav_failed | arm_contact ...
```

- 냄새: coordinator → `/task_status` (reliable + transient_local). 받는 쪽: vlm_task_planner(FAILED면 재계획) · user_input(표시).
- 주의: [노드 사전](ros2-design/10-catalog.md)의 reason 예시(`nav_blocked`)와 파일(`nav_failed`)이 다릅니다 — 파일이 정본.

<a id="api-msg-armsafetyevent"></a>
### control/msg/ArmSafetyEvent ([파일](../lv2_module6/ros2_ws/src/control/msg/ArmSafetyEvent.msg))

```text
std_msgs/Header header
string joint_name
float32 current_a
float32 threshold_a
string reason       # overcurrent
```

- 냄새: arm_safety_monitor → `/arm_safety_event`. 받는 쪽: coordinator (단계 실패 처리).

<a id="api-srv-scoregrasps"></a>
### ml/srv/ScoreGrasps ([파일](../lv2_module6/ros2_ws/src/ml/srv/ScoreGrasps.srv))

```text
uint32 object_id
uint8 max_candidates
---
geometry_msgs/PoseStamped[] grasps   # IK 통과한 것만, 점수 높은 순
float32[] scores                     # 0~1, grasps와 같은 순서
bool success
string message
```

- 서버: grasp_scorer ← coordinator (pick 직전 1회 호출).

<a id="api-topics"></a>
## 2. 토픽 목록

### 센서·인지 ([02](ros2-design/02-sense.md) · [노드 사전](ros2-design/10-catalog.md#perception))

| 이름 | 타입 | 보내는 쪽 → 받는 쪽 | 주기 · QoS |
| --- | --- | --- | --- |
| `/pole_cam/color/image_raw` | sensor_msgs/Image (rgb8) | realsense2_camera (pole_cam) → perception, rtabmap | 30Hz · best_effort |
| `/pole_cam/color/camera_info` | sensor_msgs/CameraInfo | realsense2_camera (pole_cam) → perception, rtabmap | 내부 파라미터 |
| `/pole_cam/depth/camera_info` | sensor_msgs/CameraInfo | realsense2_camera (pole_cam) → (예비) | 정렬 depth와 동일 해상도·프레임 |
| `/pole_cam/aligned_depth_to_color/image_raw` | sensor_msgs/Image (16UC1, mm) | realsense2_camera (pole_cam) → perception, rtabmap | 컬러와 1:1 정렬 |
| `/pole_cam/depth/color/points` | sensor_msgs/PointCloud2 | realsense2_camera (pole_cam) → body_filter, move_group | 10~15Hz 다운샘플 권장 |
| `/pole_cam/depth/color/points_filtered` | sensor_msgs/PointCloud2 | body_filter → costmap_2d, collision_monitor | 자기 몸(URDF collision + padding) 점을 뺀 것 |
| `/gripper_cam/color/image_raw` | sensor_msgs/Image (rgb8) | realsense2_camera (gripper_cam) → perception | 15Hz · best_effort |
| `/gripper_cam/aligned_depth_to_color/image_raw` | sensor_msgs/Image (16UC1, mm) | realsense2_camera (gripper_cam) → perception | pick 단계 위주 |
| `/gripper_cam/aligned_depth_to_color/image_scan` | sensor_msgs/Image (16UC1, mm) | scan_cam_gate → rtabmap_sync(gripper) | gripper_cam depth — 팔이 스캔 자세(`scan_pose.yaml`)에 멈춰 있을 때만 그대로, 아니면 0 (지도 만들 때 두 번째 카메라) |
| `/gripper_cam/depth/color/points_scan` | sensor_msgs/PointCloud2 | scan_cam_gate → point_cloud_aggregator | gripper_cam 점구름 — 스캔 자세일 때만(카메라 0.5m 안 = 자기 몸은 뺌), 아니면 빈 점구름 |
| `/rtabmap/scan_cloud` | sensor_msgs/PointCloud2 (base_footprint) | point_cloud_aggregator → rtabmap | 몸 뺀 pole_cam 점구름 + 위 gripper_cam 점구름 — RTAB-Map 격자(`/map`)·ICP |
| `/rtabmap/rgbd_image0` · `/rtabmap/rgbd_image1` · `/rtabmap/rgbd_images` | rtabmap_msgs/RGBDImage · RGBDImages | rgbd_sync ×2 → rgbdx_sync → rtabmap | pole_cam · gripper_cam(게이트 거친 depth)을 묶어 RTAB-Map에 |
| `/gripper_cam/depth/color/points` | sensor_msgs/PointCloud2 | realsense2_camera (gripper_cam) → grasp_scorer | 근접 파지용 |
| `/tracked_objects` | perception/TrackedObjectArray | perception → vlm_task_planner, coordinator, grasp_scorer | 10Hz · reliable — 두 카메라 융합 결과 ([필드 뜻](#api-msg-trackedobject)) |
| `/perception/pole_cam/detections` · `/perception/gripper_cam/detections` | perception/TrackedObjectArray | perception → (시뮬레이터·디버그) | 추론마다(카메라당 ≤4Hz) · reliable — 융합 전 카메라별 검출, `id` = 붙은 트랙 |

- depth 단위는 16UC1 = 밀리미터. D435 최소 측정 거리 약 0.2~0.3m — 그리퍼가 퍽에 바짝 붙으면 빈다.

### 계획·임무 ([03](ros2-design/03-plan.md))

| 이름 | 타입 | 보내는 쪽 → 받는 쪽 | 주기 · QoS |
| --- | --- | --- | --- |
| `/cmd/user_command` | std_msgs/String | user_input → vlm_task_planner | 명령마다 · reliable |
| `/mission_qr` | std_msgs/String | perception → vlm_task_planner | A 지역 QR 문제 (`MISSION:SORT_COLOR` · `STAND_UP` · `MOVE_TO_B`), 읽은 내용이 바뀔 때만 · reliable + transient_local |
| `/robot_say` | std_msgs/String | coordinator → user_input | 알릴 때 · reliable |
| `/sub_goals` | plan/SubGoalArray | vlm_task_planner → coordinator | 계획마다 · reliable + transient_local. target = 물체 id · `"x,y"`(navigate) · **`"tray"`(pick·place — 로봇 뒤 적재함: place tray = 다음 칸에 싣기, pick tray = 나중에 실은 것부터 꺼내기, 최대 4칸 — MISSION ③)** · 말할 내용(say) |
| `/task_status` | plan/TaskStatus | coordinator → vlm_task_planner, user_input | 단계 완료·실패 시 · reliable + transient_local |
| `/speed_limit` | nav2_msgs/SpeedLimit | rl_nav_tuner → controller_server, rl_nav_correction | 프리셋 변경 시 · reliable |
| `/arm_safety_event` | control/ArmSafetyEvent | arm_safety_monitor → coordinator | 접촉 시 |

### Nav2·위치추정 ([04](ros2-design/04-nav.md))

| 이름 | 타입 | 보내는 쪽 → 받는 쪽 | 주기 · QoS |
| --- | --- | --- | --- |
| `/map` | nav_msgs/OccupancyGrid | rtabmap → planner_server (global_costmap static layer) | 갱신 시 · transient_local |
| `/plan` | nav_msgs/Path | planner_server → rl_nav_correction, rl_nav_tuner, RViz | 계획마다 |
| `/cmd_vel_nav` | geometry_msgs/TwistStamped | controller_server, behavior_server → rl_nav_correction | 20Hz |
| `/local_costmap/costmap` | nav_msgs/OccupancyGrid | controller_server → rl_nav_tuner, RViz | 통로 폭·최근접 장애물 계산용 |
| `/local_costmap/costmap_raw` | nav2_msgs/Costmap | controller_server → behavior_server | 충돌 검사 |
| `/global_costmap/costmap` | nav_msgs/OccupancyGrid | planner_server → RViz | 표시용 |
| `/odometry/filtered` | nav_msgs/Odometry | ekf → rtabmap, bt_navigator, controller_server, rl_nav_tuner | 50Hz |
| `/tf` (map→odom) | tf2_msgs/TFMessage | rtabmap → 전체 | 위치 찾을 때만 (없으면 coordinator가 대기) |
| `/tf` (odom→base_footprint) | tf2_msgs/TFMessage | ekf → 전체 | 연속 |
| `/collision_monitor_state` | nav2_msgs/CollisionMonitorState | collision_monitor → RViz, coordinator | 걸린 다각형 표시 |

### 베이스 명령 체인 ([06](ros2-design/06-base.md))

`/cmd_vel_nav` → rl_nav_correction → `/cmd_vel_corrected` → twist_mux → `/cmd_vel_mux` → velocity_smoother → `/cmd_vel_smoothed` → collision_monitor → `/cmd_vel` → diff_drive_controller. 전부 `geometry_msgs/TwistStamped`, 체인 전체 통일 (Jazzy 이후 Nav2 `enable_stamped_cmd_vel: true`).

| 이름 | 타입 | 보내는 쪽 → 받는 쪽 | 주기 · QoS |
| --- | --- | --- | --- |
| `/cmd_vel_corrected` | geometry_msgs/TwistStamped | rl_nav_correction → twist_mux (자동, priority 10) | 20Hz |
| `/cmd_vel_manual` | geometry_msgs/TwistStamped | manual_control → twist_mux (수동, priority 100) | 수동 조작 시 |
| `/cmd/e_stop` | std_msgs/Bool | 운영자 → state_manager → `/state/emergency` → twist_mux (잠금 priority 255) | true면 전부 차단 |
| `/cmd_vel_mux` | geometry_msgs/TwistStamped | twist_mux → velocity_smoother | 20Hz |
| `/cmd_vel_smoothed` | geometry_msgs/TwistStamped | velocity_smoother → collision_monitor | 20Hz |
| `/cmd_vel` | geometry_msgs/TwistStamped | collision_monitor → diff_drive_controller | 20Hz |
| `/odom` | nav_msgs/Odometry | diff_drive_controller → ekf, rl_nav_correction | 50Hz |
| `/imu` | sensor_msgs/Imu | imu_sensor_broadcaster → ekf, rl_nav_correction | 100Hz · best_effort |
| `/odometry/filtered` | nav_msgs/Odometry | ekf → rtabmap, bt_navigator, controller_server, rl_nav_tuner | 50Hz |

- diff_drive_controller 토픽은 기본 `/diff_drive_controller/cmd_vel`·`/odom` — launch에서 `/cmd_vel`·`/odom`으로 리매핑.
- odom TF는 ekf만 (`diff_drive_controller enable_odom_tf: false`, ekf `publish_tf: true`).

### 수동 조작·운영 모드 ([노드 사전](ros2-design/10-catalog.md#manual_control))

| 이름 | 타입 | 보내는 쪽 → 받는 쪽 | 주기 · QoS |
| --- | --- | --- | --- |
| `/cmd/operation_mode` | std_msgs/String | 시뮬레이터 · 운영자 → state_manager | MANUAL · AUTO · EXPLORE(자동 탐사 — 지도 모드 MAPPING일 때만 허락) — 조작 모드 명령 (외부 명령은 `/cmd/<이름>`(보내고 끝 — cmd_vel 관례), 결과는 `/state/<이름>` — 둘 다 state_manager만 구독·발행) |
| `/cmd/manual/cmd_vel` | geometry_msgs/TwistStamped | teleop_twist_keyboard · 시뮬레이터 → manual_control | 수동일 때만 넘김, 0.5초 끊기면 0 |
| `/cmd/manual/arm` | sensor_msgs/JointState | 시뮬레이터 · 운영자 → manual_control | joint1~5 rad · gripper_left_joint m |
| `/state/operation_mode` | std_msgs/String (transient_local) | state_manager → manual_control · coordinator · explorer | AUTO · MANUAL · EXPLORE (AUTO가 아니면 coordinator는 작업 멈춤, EXPLORE면 explorer가 탐사) |
| `/explore/result` | std_msgs/String | explorer → state_manager | 탐사 끝: `DONE …`(프론티어 없음) · `FAILED …` → state_manager가 AUTO로 되돌림 |
| `/state/map_list` | std_msgs/String (transient_local) | state_manager → 시뮬레이터 · 누구나 | 가진 지도 폴더 이름들 (줄바꿈 구분, `<map_dir>/<이름>/rtabmap.db` 있는 것) — 바뀔 때만 |
| `/cmd/restart` | std_msgs/String | 시뮬레이터 [ROS 재시작] · `ros-faker-restart` · 운영자(`ros2 topic pub --once`) → state_manager | **`all [이름:=값 …]`** = 이 launch 전체를 정상 종료 → 같은 명령줄로 다시 (인자를 주면 그 인자만 바꿈, `map:=`을 바꾸면 `map_name:=` 버림 · 테스트베드는 그 사이 `build-ws`) · **`node <이름>`** = 그 노드 프로세스만 같은 명령줄·환경(파라미터 파일·리매핑 그대로)으로 다시 (Nav2 수명주기 노드는 다시 활성화 필요). 실기 bringup · 테스트베드 같은 길. 같은 요청이 두 번 오지 않게 하는 건 보내는 쪽 — 테스트베드 input_manager는 읽은 파일을 지움. 도는 중엔 새 요청 거절 |
| `/state/restartable` | std_msgs/String (transient_local) | state_manager → 시뮬레이터 진단 칸 · 누구나 | `node`로 재시작할 수 있는 노드 `"이름 pid"` 줄들 (이 launch 아래 ROS 노드 프로세스 — 컨트롤러·costmap 하위·내부 노드는 프로세스가 아니라 없음). 2초마다 확인 · 바뀔 때만 |
| `/state/restart` | std_msgs/String (transient_local) | state_manager → 시뮬레이터 상태판 · 누구나 | 마지막 재시작 요청 결과 — `all: 재시작 중 …` · `node perception: 재시작 (pid …)` · `거절: …` · 새로 뜨면 `시작 (pid …)` |
| `/cmd/map_name` | std_msgs/String | 시뮬레이터 지도 목록 · 운영자 → state_manager | 지도 폴더 이름 — `<map_dir>/<이름>/rtabmap.db`를 RTAB-Map `/rtabmap/rtabmap/load_database`로 열고 위치추정 모드로 (결과 `/state/map_name` · `/state/map_mode`) |
| `/cmd/map_mode` | std_msgs/String | 시뮬레이터 · 운영자 → state_manager | MAPPING(스캔) · LOCALIZATION(위치추정) — RTAB-Map 모드 전환 요청 |
| `/state/map_mode` | std_msgs/String (transient_local) | state_manager → 시뮬레이터 · 누구나 | MAPPING · LOCALIZATION · COURSE(정지 지도, 바꿀 수 없음) — RTAB-Map 응답 뒤에만 바뀜 |
| `/state/map_name` | std_msgs/String (transient_local) | state_manager → 시뮬레이터 · 누구나 | 지금 쓰는 지도 폴더 이름 (`<map_dir>/<이름>/` rtabmap.db · map.pgm · map.yaml, 빈 값 = 폴더 없음) — 시뮬레이터 [지도 저장]이 같은 폴더에 씀 |
| `/cmd_vel_manual` | geometry_msgs/TwistStamped | manual_control → twist_mux | 위 체인 참조 |
| `/arm_controller/joint_trajectory` | trajectory_msgs/JointTrajectory | manual_control → arm_controller (JTC) | 5축 전체 한 점 |

### 팔·MoveIt ([05](ros2-design/05-arm.md))

| 이름 | 타입 | 보내는 쪽 → 받는 쪽 | 주기 |
| --- | --- | --- | --- |
| `/arm_controller/follow_joint_trajectory` | 액션이지만 토픽 아님 — [§3](#3-액션-목록) 참조 | move_group → arm_controller | 궤적마다 |
| `/joint_states` | sensor_msgs/JointState | joint_state_broadcaster → robot_state_publisher, move_group, arm_safety_monitor | 100Hz |
| `/arm_safety_event` | control/ArmSafetyEvent | arm_safety_monitor → coordinator | 접촉 시 |
| `/monitored_planning_scene` | moveit_msgs/PlanningScene | move_group → coordinator(MTC), RViz | 갱신 시 |
| `/arm_controller/controller_state` | control_msgs/JointTrajectoryControllerState | arm_controller → 모니터링·RViz | 상태 표시 |

### ros2_control·진단 ([08](ros2-design/08-ctrl.md))

| 이름 | 타입 | 보내는 쪽 → 받는 쪽 | 비고 |
| --- | --- | --- | --- |
| `/robot_description` | std_msgs/String | robot_state_publisher → controller_manager, move_group | transient_local — URDF |
| `/tf`, `/tf_static` | tf2_msgs/TFMessage | robot_state_publisher → 전체 | 관절·고정 링크 |
| `/joint_states` | sensor_msgs/JointState | joint_state_broadcaster → 전체 | 100Hz (위 표 중복 — 양쪽에서 씀) |
| `/dynamic_joint_states` | control_msgs/DynamicJointState | joint_state_broadcaster → 디버그 | 표시용 |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | 모든 🛠 노드·하드웨어 플러그인 → diagnostic_aggregator | 1Hz · OK·WARN·ERROR + 값 |
| `/diagnostics_agg` | diagnostic_msgs/DiagnosticArray | diagnostic_aggregator → 표시·시뮬레이터 | 계층별 묶음 |

<a id="api-actions"></a>
## 3. 액션 목록

| 이름 | 타입 | 요청 → 서버 | 용도 |
| --- | --- | --- | --- |
| `/navigate_to_pose` | nav2_msgs/action/NavigateToPose | coordinator → bt_navigator | 이동 단계마다. feedback: 남은 거리 |
| `/compute_path_to_pose` | nav2_msgs/action/ComputePathToPose | bt_navigator → planner_server | 약 1Hz 재계획 |
| `/follow_path` | nav2_msgs/action/FollowPath | bt_navigator → controller_server | 경로 변경 시 |
| `/spin`, `/backup`, `/wait` | nav2_msgs/action/Spin · BackUp · Wait | bt_navigator → behavior_server | 복구 시. BackUp은 0.1m 이하로 제한 (뒤를 못 봄) |
| `/execute_task_solution` | moveit_task_constructor_msgs/action/ExecuteTaskSolution | coordinator(MTC 태스크) → move_group | pick·place마다 |
| `/arm_controller/follow_joint_trajectory` | control_msgs/action/FollowJointTrajectory | move_group → arm_controller | MTC 단계마다 |
| `/gripper_controller/gripper_cmd` | control_msgs/action/GripperCommand | move_group(MTC) → gripper_controller | 열기·닫기. 결과 stalled로 빈손 판단 |

<a id="api-services"></a>
## 4. 서비스 목록

| 이름 | 타입 | 호출 → 서버 | 용도 |
| --- | --- | --- | --- |
| `/score_grasps` | ml/srv/ScoreGrasps | coordinator → grasp_scorer | pick 직전 1회 |
| `/compute_ik` | moveit_msgs/srv/GetPositionIK | grasp_scorer → move_group | 후보마다 도달 가능 여부 |
| `/apply_planning_scene` | moveit_msgs/srv/ApplyPlanningScene | coordinator → move_group | pick 전 퍽·장애물 등록 |
| `/clear_octomap` | std_srvs/srv/Empty | coordinator → move_group | pick 직전 octomap 초기화 |
| `/controller_server/set_parameters` | rcl_interfaces/srv/SetParameters | rl_nav_tuner → controller_server | FollowPath.max_vel_theta 등 (실행 중 반영되는 것만) |
| `/local_costmap/local_costmap/set_parameters` | rcl_interfaces/srv/SetParameters | coordinator → controller_server | 퍽을 들면 footprint 확장 |
| `/arm_controller/follow_joint_trajectory/_action/cancel_goal` | action_msgs/srv/CancelGoal | arm_safety_monitor → arm_controller | 빈 goal_id = 전체 취소 → JTC 현재 자세 유지 |
| `/controller_manager/load_controller` 등 | controller_manager_msgs/srv/* | spawner → controller_manager | load · configure · switch · list (기동 시) |
| `*/change_state` | lifecycle_msgs/srv/ChangeState | lifecycle_manager → Nav2 서버들 | configure → activate |
| `/lifecycle_manager_navigation/manage_nodes` | nav2_msgs/srv/ManageLifecycleNodes | 운영자·launch → lifecycle_manager | 주행 계획 서버 (planner · controller · behavior · bt_navigator) 시작·정지·리셋 |
| `/lifecycle_manager_base/manage_nodes` | nav2_msgs/srv/ManageLifecycleNodes | 운영자·launch → lifecycle_manager | 바퀴 명령 체인 (velocity_smoother · collision_monitor) 시작·정지·리셋 — 지도 없이도 뜸 |

<a id="api-tf"></a>
## 5. TF

[07](ros2-design/07-tf.md). 발행자는 딱 둘 + robot_state_publisher.

| 변환 | 발행자 | 성격 |
| --- | --- | --- |
| map → odom | rtabmap | 가끔 점프 (루프 클로저·재위치 추정) |
| odom → base_footprint | robot_localization ekf | 연속, 드리프트 있음 |
| base_footprint → base_link → 센서·팔 | robot_state_publisher (URDF) | 고정 또는 /joint_states로 변함 |
| *_link → *_optical_frame | robot_state_publisher | 고정 (z 앞 · x 오른쪽 · y 아래) |

- Nav2 `robot_base_frame`, ekf `base_link_frame`, MoveIt 부모를 **모두 base_footprint**로 통일.

<a id="api-qos"></a>
## 6. QoS 규칙

[07 QoS 정리](ros2-design/07-tf.md#qoS-정리).

| 데이터 | Reliability | Durability | Depth | 이유 |
| --- | --- | --- | --- | --- |
| 카메라·점구름·IMU | best_effort | volatile | 1~5 | 최신 값만 의미 있음 |
| /cmd_vel 체인 | reliable | volatile | 1 | 명령 유실 방지, 오래된 명령은 버림 |
| /map, /tf_static | reliable | transient_local | 1 | 늦게 붙어도 받음 |
| /sub_goals, /task_status | reliable | transient_local | 10 | 재시작해도 마지막 계획 받음 |
| /tracked_objects | reliable | volatile | 5 | 누락 시 다음 값으로 충분 |

> best_effort 발행자를 reliable 구독자가 받으면 **연결 자체가 안 된다.** 센서 구독은 `rclcpp::SensorDataQoS()`.

<a id="api-change"></a>
## 7. 바꿀 때 절차

1. `.msg`/`.srv`를 고치거나 만든다 (파일명 CamelCase, 필드 snake_case).
2. 새 파일이면 그 패키지 `CMakeLists.txt`의 `rosidl_generate_interfaces(...)`에 추가. 다른 패키지 타입을 쓰면 `DEPENDENCIES`와 `package.xml`에도 추가.
3. `colcon build --symlink-install --packages-above <패키지>` — 쓰는 쪽까지 다시 빌드.
4. 받는 쪽·보내는 쪽 담당자에게 알린다 (PR 설명에 바뀐 필드). 담당표는 [lv2_module6/README.md](../lv2_module6/README.md#담당자).
5. 이 문서(API.md)의 해당 표도 같이 고친다. `ros2 interface show <타입>`으로 확인.
