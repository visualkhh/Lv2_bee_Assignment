# API 명세서 — 토픽·서비스·액션

협업용 인터페이스 명세입니다. 설계 근거는 [documents/project/ros](project/ros/README.md) (특히 [노드 사전](project/ros/10-catalog.md)), 구현은 [lv2_module6](../lv2_module6/README.md)입니다.
커스텀 타입 원문은 `.msg`·`.srv` 파일이 정본입니다 — 아래 정의와 파일이 다르면 **파일이 맞습니다**.

표기: ✉️ 토픽 · 🎬 액션 · 📞 서비스 · 🧩 ros2_control 내부 인터페이스 · 🔌 장치. QoS 규칙은 [§7](project/ros/07-tf.md) 요약.

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

별도 interfaces 패키지 없이 **그 메시지를 내보내는 패키지**가 가집니다 ([DEVELOP.md](../lv2_module6/simulator/gazebo/DEVELOP.md#4-커스텀-메시지)).
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
string[] cameras                        # 본 카메라 전부 (pole_front_cam | gripper_cam) — 오래 전에 본 것부터, 마지막 = 가장 최근
                                        # 빈 배열([]) = 지도에 위치는 기억하지만 현재 어느 카메라에도 안 잡힘 (벽 뒤·시야 밖·집어 든 직후 등)
                                        #   → pose(마지막 위치)·last_seen(마지막 시각)은 유효 — 그 위치로 navigate 가능
                                        #   → 물체 삭제 정책: 퍽·location·barcode는 miss_count번 연속 미검출 시 삭제 / 사람·장애물·신호등은 TRACK_TIMEOUT_S(2초) 후 삭제
int32[2] px                             # 마지막 카메라(cameras 끝) 영상 속 박스 중심 픽셀 [u, v] — VLM 이미지 마킹용
string color                            # 퍽: red | orange | yellow | green | blue | purple | white | black | unknown (윗면 태그를 읽었으면 태그 번호대 색) · 신호등 상태: red | yellow | green | off | all(두 색 이상 동시) · 그 외 빈 값 (박스 가운데 HSV)
diagnostic_msgs/KeyValue[] details      # 클래스별 속성 — puck: posture · tag(윗면 AprilTag 0~11) / location: tag(12~14) / barcode: content(작업 지시) / traffic_light: state · 그 외 빈 배열
```

표식 (발제 「AprilTag와 바코드 실제 배포 표식」, tag36h11 — OpenCV aruco로 읽음): 퍽 윗면 16mm R1~R4 = 0~3 · G1~G4 = 4~7 · B1~B4 = 8~11 → 그 퍽의 `tag`(한 번 읽으면 기억 · gripper_cam이 위에서 봐야 읽힘 — coordinator `inspect`) ·
장소 48mm L1~L3 = 12~14 → class `location` 물체 · 줄무늬 바코드(P2-01 …) = 장소 작업 지시 → class `barcode` 물체(`content`) + `/mission/qr`. 셋 다 안 보여도 기억.

물체 하나 = 추적 트랙 하나. 카메라마다 YOLO 박스 → 박스 중심 픽셀 + 박스 안 정렬 depth 중앙값으로 3D 점 1개 → 영상 시각의 TF로 map 변환 →
**자기 몸이면 버림** — 3D: 팔 링크를 잇는 선분 6cm 안 · 차체 반경 0.2m 안 바닥 위 / 영상: 팔(TF 링크를 화면에 투영, 두께 4cm)과 겹치는 박스(퍽 제외 — 박스 일부만 팔이면 depth가 팔 뒤 바닥이라 3D로는 못 거름) → 같은 클래스·0.4m 안의 트랙에 1:1로 붙여 **칼만 갱신(두 카메라 융합)**. 포인트 클라우드는 쓰지 않는다 (객체 단위 융합). 구현: [perception_node.py](../lv2_module6/ros2_ws/src/perception/scripts/perception_node.py), 파라미터: [perception.yaml](../lv2_module6/ros2_ws/src/perception/config/perception.yaml).

| 필드 | 뜻 | 주의 |
| --- | --- | --- |
| `id` | 추적 번호 (1부터 증가) — VLM·coordinator·grasp_scorer가 같은 물체를 이 번호로 가리킴 (`SubGoal.target`) | **기억하는 물체**(`puck*` · `goal_zone` · `station_zone` · `qr_board` · `location` · `barcode`, `remember_min_hits`번 이상 잡힌 것)는 안 보여도 남음 — 그 자리가 잘 보이는데 `miss_count`번 연속 못 찾을 때만 삭제(옮겨짐·집어 감). 그 밖(사람·장애물·신호등)은 어느 카메라에도 **2초** 안 보이면 삭제 → 다시 보이면 **새 번호**. 번호는 재사용 안 함 |
| `class_name` | YOLO: `puck` · `puck_lying` · `goal_zone` · `station_zone` · `qr_board` · `obstacle` · `person` · `traffic_light` / 표식 디코드: `location`(장소 AprilTag 12~14) · `barcode`(바코드 P2-xx) | YOLO 클래스는 모델의 `names`를 따름 (`puck_red`처럼 색이 붙은 이름은 `puck` + color로 나눔). 다른 클래스끼리는 가까워도 합치지 않음 |
| `confidence` | 마지막 관측의 YOLO 신뢰도 (0~1, `confidence` 파라미터 이상만) | 평균 아님 |
| `pose.pose.position` | map 좌표 [m] — 원점 = map 원점(시뮬레이터: 출발 칸 `spawn`), x 앞 · y 왼쪽 · z 위(바닥 ≈ 0). **추적은 odom 기준, 발행 순간의 map ← odom(RTAB-Map 최신 보정)으로 바꿔 냄** — 보정이 바뀌면(맞든 틀리든) 로봇 위치와 함께 옮겨져 로봇과의 상대 위치는 유지. 위치추정 자체가 틀리면 map 좌표는 그만큼 통째로 어긋남 | 박스 중심 방향의 **카메라 쪽 앞면 점** — 실제 중심보다 카메라 쪽으로 물체 반지름쯤 당겨짐 (퍽 2~3cm). z도 앞면 점 높이. 두 카메라가 다른 방향에서 보면 일부 상쇄 |
| `pose.pose.orientation` | 항상 `(0, 0, 0, 1)` | 방향은 추정하지 않음 |
| `pose.covariance` | 6×6 행 우선. **위치 3×3 블록**(인덱스 0~2·6~8·12~14) = 융합 위치 불확실성 [m²], 회전 대각(21·28·35) = 1e3(모름) | 관측 잡음만 반영 — 위의 앞면 쏠림(바이어스)은 안 들어감. 관측 σ: depth 축 `depth_sigma0 + depth_sigma_k·z²`, 가로·세로 `pixel_sigma·z/f` |
| `size` | [m] x = 박스 가로 폭 · y = 박스 세로 높이 (픽셀 × 거리 / 초점거리), z = 클래스 기본값(퍽 0.06 — 발제 30×30×60mm, 누운 퍽 0.03 등). 크기를 못 재면 셋 다 클래스 기본값 | **map 축 크기가 아님** (카메라 영상 기준). 관측마다 칼만 이득 비율로 갱신 |
| `last_seen` | 마지막으로 관측된 **영상 시각** (카메라 header.stamp) | `TrackedObjectArray.header.stamp`(발행 시각)과 다름 |
| `cameras` | 이 트랙을 본 카메라 전부 `pole_front_cam` · `gripper_cam` — 마지막으로 본 시각 순, **끝 = 가장 최근에 본 카메라** | **빈 배열(`[]`) = 지도에 위치는 기억하고 있지만 현재 어느 카메라에도 잡히지 않음** (벽 뒤·시야 밖·집어 든 직후 등). 비어 있어도 `pose`(마지막 관측 위치)·`last_seen`(마지막 관측 시각)은 유효 — 그 위치로 navigate 가능. 따로 `camera` 필드는 없음 — `cameras[-1]`. **물체 종류별 삭제 정책**: 퍽·목표구역·QR판은 안 보여도 유지(옮겨졌다고 판단되면 `miss_count`번 연속 미검출 시 삭제) / 사람·장애물·신호등은 `TRACK_TIMEOUT_S`(2초) 안 보이면 삭제. 수신 측 판별 패턴: `cameras` 비어 있으면 "알고 있지만 현재 미관측" / `last_seen` 기준 경과 시간으로 얼마나 전에 봤는지 구분 |
| `px` | `cameras` 끝 카메라 영상(640×480) 속 박스 중심 픽셀 `[u, v]` — VLM이 그 영상에 표시할 때 씀 (planner가 ai-service에 `camera` + `px {카메라: [u, v]}`로 보냄) | 다른 카메라 영상의 좌표가 아님 |
| `color` | 퍽: `red` · `orange` · `yellow` · `green` · `blue` · `purple` · `white` · `black` · `unknown` / 신호등: `red` · `yellow` · `green` · `off` · `all`(두 색 이상 동시 — 신호등 미션 단계 구분) / 그 외 `""` | 박스 가운데(가장자리 20%씩 뺌 — 바닥 안 섞이게) HSV 다수결, 채도 없는 화소가 대부분이면 흰색·검정. 마지막 관측 값. 퍽 윗면 태그를 읽었으면 태그 번호대 색(R 0~3 · G 4~7 · B 8~11)이 우선 |
| `details` | `puck`: `posture` · `tag` / `location`: `tag` / `barcode`: `content` / `traffic_light`: `state` | `tag`·`content`는 한 번 읽으면 기억 (멀어져 못 읽어도 덮지 않음) |

<a id="api-msg-trackedobjectarray"></a>
### perception/msg/TrackedObjectArray ([파일](../lv2_module6/ros2_ws/src/perception/msg/TrackedObjectArray.msg))

```text
std_msgs/Header header                   # frame_id: map
TrackedObject[] objects
```

- 내는 쪽: perception → `/tracked_objects` (10Hz · reliable). 받는 쪽: vlm_task_planner · coordinator · grasp_scorer.
- `header.stamp` = **발행 시각**(10Hz 타이머), `header.frame_id` = `map`. `objects` = 지금 살아 있는 트랙 전부 (id 오름차순, 없으면 빈 배열).
- 같은 타입을 `/perception/<cam>/detections`로도 낸다 — **융합 전** 그 카메라 한 프레임 검출. `id` = 그 검출이 붙은 트랙 id, `pose.covariance` = 그 관측의 공분산, `header.stamp` = 영상 시각. 디버그·시뮬레이터 표시용 (받는 노드 없음).

#### `cameras` 필드 활용 패턴

`cameras` 빈 배열 여부로 **"현재 보이는지 / 지도에만 기억된 상태인지"** 를 구분할 수 있다.

```
cameras = ["pole_front_cam"]              → 지금 pole_front_cam에 잡힘
cameras = ["pole_front_cam","gripper_cam"]→ 두 카메라 모두 잡힘 (끝이 가장 최근)
cameras = []                              → 지도에 위치는 있지만 현재 어느 카메라에도 안 잡힘
```

수신 측 판별 예시 (Python):

```python
from rclpy.time import Time

for obj in tracked_objects.objects:
    # ① 지금 카메라에 잡히는 것만
    if obj.cameras:
        latest_cam = obj.cameras[-1]   # 가장 최근에 본 카메라

    # ② 지도엔 알지만 지금 안 보이는 것 (벽 뒤·시야 밖·집어 든 직후)
    if not obj.cameras:
        # pose(마지막 위치)·last_seen(마지막 시각)은 여전히 유효
        # → 그 위치로 navigate하면 가까이 갔을 때 cameras가 다시 채워짐
        pass

    # ③ 최근 N초 이내에 본 것 (now는 node.get_clock().now())
    last = Time.from_msg(obj.last_seen)
    age_s = (now - last).nanoseconds / 1e9
    if age_s < 2.0:
        pass  # 2초 이내 관측
```

물체 종류별 삭제 정책:

| 클래스 | 안 보일 때 | 삭제 조건 |
|--------|-----------|-----------|
| `puck` · `location` · `barcode` · `goal_zone` | `cameras=[]` 유지 | `miss_count`번 연속 미검출 (그 자리가 잘 보이는데 없는 경우 — 옮겨지거나 집어 감) |
| `person` · `obstacle` · `traffic_light` | `cameras=[]` 유지 | `TRACK_TIMEOUT_S`(2.0초) 안 보이면 트랙 삭제 |

<a id="api-msg-subgoal"></a>
### plan/msg/SubGoal ([파일](../lv2_module6/ros2_ws/src/plan/msg/SubGoal.msg))

```text
uint32 step
string skill        # navigate | pick | place | inspect | say
string target       # 물체 id (/tracked_objects의 id) · "x,y"(navigate) · "tray"(pick·place 적재함) · 말할 내용(say)
string relation     # front | on | ...
float32 offset_m
```

| skill | target | 하는 일 (coordinator) |
| --- | --- | --- |
| `navigate` | 물체 id · `"x,y"`(map) | 그 물체 앞 `offset_m`까지 (relation `front`) 또는 그 좌표로 Nav2 이동 |
| `pick` | 물체 id · `"tray"` | 파지 후보(`/score_grasps`) 순서로 집기 · tray = 적재함에서 나중에 실은 것부터 꺼내기 |
| `place` | 물체 id · `"tray"` | 그 물체 위에(relation `on`, 여러 개면 `place_spacing`씩 비킴) · tray = 적재함 다음 칸에 싣기 (최대 4칸) |
| `inspect` | 퍽 id | 손끝을 퍽 윗면 위 `inspect_height_m`로 → `inspect_wait_s` 기다림 — gripper_cam이 윗면 16mm AprilTag(번호)를 읽게. 못 읽어도 성공 (읽었는지는 `details.tag`) |
| `say` | 말할 내용 | `/robot/say`로 알림 |

<a id="api-msg-subgoalarray"></a>
### plan/msg/SubGoalArray ([파일](../lv2_module6/ros2_ws/src/plan/msg/SubGoalArray.msg))

```text
std_msgs/Header header
uint32 plan_id
SubGoal[] goals
```

- 내는 쪽: vlm_task_planner → `/task/sub_goals` (reliable + transient_local). 받는 쪽: coordinator.

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

- 내는 쪽: coordinator → `/task/status` (reliable + transient_local). 받는 쪽: vlm_task_planner(FAILED면 재계획 · SUCCEEDED면 다음 판단 — inspect 뒤 다시 조합, 옮기기 뒤 수납 확인) · user_input(표시).
- reason (coordinator.cpp): 이동 `nav_failed` · `approach_failed` / 집기 `no_grasp` · `grasp_failed_all` · `arm_contact` / 놓기 `place_failed` · `place_no_target` / 적재함 `tray_full` · `tray_empty` · `tray_place_failed` · `tray_pick_failed` / 보기 `inspect_no_target` · `inspect_failed` / 멈춤 `manual_override`·`manual_mode` · `explore_override`·`explore_mode` · `estop`·`estop_blocked` · `cancelled` — `manual`·`explore`·`estop`·`cancelled`로 시작하면 planner는 재계획하지 않음.
- 주의: [노드 사전](project/ros/10-catalog.md)의 reason 예시(`nav_blocked`)와 파일(`nav_failed`)이 다릅니다 — 파일이 정본.

<a id="api-msg-missionsignal"></a>
### perception/msg/MissionSignal ([파일](../lv2_module6/ros2_ws/src/perception/msg/MissionSignal.msg))

```text
string color      # 1단계: 옮길 퍽 색 (red | green | yellow — 깜빡인 색)
int32 count       # 1단계: 그 색 퍽 개수 (깜빡인 횟수)
int32 sum         # 2단계: 고를 퍽들의 태그 번호 합 (깜빡인 색 점수의 합)
int32 dest_tag    # 3단계: 목적지 장소 태그 번호 (노랑 1·2·3번 → 12·13·14)
string raw        # 읽은 깜빡임 그대로 (예: "R R | R Y G G R | Y Y") — 확인용
```

- 내는 쪽: signal_decoder (perception — 인지가 신호를 해석해 미션 토픽으로 냄. `/mission/qr`와 같은 자리) → `/mission/signal` (reliable + transient_local, 확정될 때마다). 받는 쪽: vlm_task_planner.
- 신호등 미션 규칙은 [mission/README.md](mission/README.md). 단계 구분(세 색 동시 `separator_s` · 꺼짐 `gap_s`)·색 점수·목적지 태그는 [perception.yaml](../lv2_module6/ros2_ws/src/perception/config/perception.yaml) `signal_decoder` 파라미터 (발제 미정 — 제안값).

<a id="api-msg-armsafetyevent"></a>
### control/msg/ArmSafetyEvent ([파일](../lv2_module6/ros2_ws/src/control/msg/ArmSafetyEvent.msg))

```text
std_msgs/Header header
string joint_name
float32 current_a
float32 threshold_a
string reason       # overcurrent
```

- 내는 쪽: arm_safety_monitor → `/arm_safety_event`. 받는 쪽: coordinator (단계 실패 처리).

<a id="api-msg-motorstatus"></a>
### control/msg/MotorStatus ([파일](../lv2_module6/ros2_ws/src/control/msg/MotorStatus.msg))

```text
std_msgs/Header header
string[] joints   # 프로토콜 순서: wheel_left_joint, wheel_right_joint, joint1~5, gripper_left_joint
int32[] error     # 0 정상 · -1 응답 없음 · 비트 1 입력전압 · 4 과열 · 8 엔코더 · 16 전기충격 · 32 과부하 (Reboot로만 풀림)
string[] text     # 사람이 읽는 이름 — "정상" · "과부하" · "과열·과부하" · "응답 없음"
int32[] temperature  # Present Temperature °C · -1 모름 (응답 없음 · 온도 없는 예전 펌웨어 · mock). 80 °C(Temperature Limit)면 과열 에러로 멈춤
```

- 내는 쪽: BeebotSystemHardware(controller_manager 안) → `/motor_status` (reliable + transient_local, 0.5초마다 보고 바뀔 때만). 받는 쪽: state_manager → `/state/devices/<wheels|arm|gripper>`의 `error`.
- 원본은 펌웨어 `E` 줄(DYNAMIXEL Hardware Error Status, addr 70) · `S` 줄 끝 8칸(Present Temperature, addr 146 — 0.5초마다 읽은 값). 에러나 온도가 바뀔 때 낸다. 온도 70 °C 이상이면 진단 "OpenCR 연결"이 WARN — [firmware/README.md](../lv2_module6/firmware/README.md). 로봇 상태가 아니라 **하드웨어 원본 사실**이라 `/state/...`가 아님 (`/state/<이름>`은 state_manager만 냄). 사람용 설명은 `/diagnostics`에도 따로 있지만 노드는 진단 문자열을 읽지 않고 이 토픽을 읽는다.

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

### 센서·인지 ([02](project/ros/02-sense.md) · [노드 사전](project/ros/10-catalog.md#perception))

| 이름 | 타입 | 보내는 쪽 → 받는 쪽 | 주기 · QoS |
| --- | --- | --- | --- |
| `/pole_front_cam/color/image_raw` | sensor_msgs/Image (rgb8) | realsense2_camera (pole_front_cam) → perception, rtabmap | 30Hz · best_effort |
| `/pole_front_cam/color/camera_info` | sensor_msgs/CameraInfo | realsense2_camera (pole_front_cam) → perception, rtabmap | 내부 파라미터 |
| `/pole_front_cam/depth/camera_info` | sensor_msgs/CameraInfo | realsense2_camera (pole_front_cam) → (예비) | 정렬 depth와 동일 해상도·프레임 |
| `/pole_front_cam/aligned_depth_to_color/image_raw` | sensor_msgs/Image (16UC1, mm) | realsense2_camera (pole_front_cam) → perception, rtabmap | 컬러와 1:1 정렬 |
| `/pole_front_cam/depth/color/points` | sensor_msgs/PointCloud2 | realsense2_camera (pole_front_cam) → body_filter, move_group | 10~15Hz 다운샘플 권장 |
| `/pole_front_cam/depth/color/points_filtered` | sensor_msgs/PointCloud2 | body_filter → costmap_2d, collision_monitor | 자기 몸(URDF collision + padding) 점을 뺀 것 |
| `/gripper_cam/color/image_raw` | sensor_msgs/Image (rgb8) | realsense2_camera (gripper_cam) → perception | 15Hz · best_effort |
| `/gripper_cam/aligned_depth_to_color/image_raw` | sensor_msgs/Image (16UC1, mm) | realsense2_camera (gripper_cam) → perception | pick 단계 위주 |
| `/gripper_cam/aligned_depth_to_color/image_scan` | sensor_msgs/Image (16UC1, mm) | scan_cam_gate → rtabmap_sync(gripper) | gripper_cam depth — 팔이 스캔 자세(`scan_pose.yaml`)에 멈춰 있을 때만 그대로, 아니면 0 (지도 만들 때 두 번째 카메라) |
| `/gripper_cam/depth/color/points_scan` | sensor_msgs/PointCloud2 | scan_cam_gate → point_cloud_aggregator | gripper_cam 점구름 — 스캔 자세일 때만(카메라 0.5m 안 = 자기 몸은 뺌), 아니면 빈 점구름 |
| `/rtabmap/scan_cloud` | sensor_msgs/PointCloud2 (base_footprint) | point_cloud_aggregator → rtabmap | 몸 뺀 pole_front_cam 점구름 + 위 gripper_cam 점구름 — RTAB-Map 격자(`/map`)·ICP |
| `/rtabmap/rgbd_image0` · `/rtabmap/rgbd_image1` · `/rtabmap/rgbd_images` | rtabmap_msgs/RGBDImage · RGBDImages | rgbd_sync ×2 → rgbdx_sync → rtabmap | pole_front_cam · gripper_cam(게이트 거친 depth)을 묶어 RTAB-Map에 |
| `/gripper_cam/depth/color/points` | sensor_msgs/PointCloud2 | realsense2_camera (gripper_cam) → grasp_scorer | 근접 파지용 |
| `/tracked_objects` | perception/TrackedObjectArray | perception → vlm_task_planner, coordinator, grasp_scorer | 10Hz · reliable — 두 카메라 융합 결과 ([필드 뜻](#api-msg-trackedobject)) |
| `/perception/pole_front_cam/detections` · `/perception/gripper_cam/detections` | perception/TrackedObjectArray | perception → (시뮬레이터·디버그) | 추론마다(카메라당 ≤4Hz) · reliable — 융합 전 카메라별 검출, `id` = 붙은 트랙 |

- depth 단위는 16UC1 = 밀리미터. D435 최소 측정 거리 약 0.2~0.3m — 그리퍼가 퍽에 바짝 붙으면 빈다.

### 계획·임무 ([03](project/ros/03-plan.md))

| 이름 | 타입 | 보내는 쪽 → 받는 쪽 | 주기 · QoS |
| --- | --- | --- | --- |
| `/cmd/user_command` | std_msgs/String | user_input → vlm_task_planner | 명령마다 · reliable |
| `/mission/qr` | std_msgs/String | perception → vlm_task_planner | 장소 바코드(작업 지시 문자열, 예 `P2-01`) · QR — 읽은 내용이 바뀔 때만 · reliable + transient_local. VLM이 해석 |
| `/mission/signal` | perception/MissionSignal | signal_decoder (perception) → vlm_task_planner | 신호등 암호 미션 답 {color, count, sum, dest_tag, raw} — 확정될 때마다 · reliable + transient_local. planner가 규칙으로 계획 (번호 모르면 inspect → 조합 → 적재함 → 목적지 장소 → **수납 확인**: 옮긴 직후와 `verify_period_s`마다 목적지로 가서 옮긴 번호가 `verify_radius_m` 안에 있는지 보고, 다른 곳에 있으면 다시 옮김 · 없으면 say) |
| `/robot/say` | std_msgs/String | coordinator → user_input | 알릴 때 · reliable |
| `/task/sub_goals` | plan/SubGoalArray | vlm_task_planner → coordinator | 계획마다 · reliable + transient_local. target = 물체 id · `"x,y"`(navigate) · **`"tray"`(pick·place — 로봇 뒤 적재함: place tray = 다음 칸에 싣기, pick tray = 나중에 실은 것부터 꺼내기, 최대 4칸)** · 퍽 id(inspect) · 말할 내용(say) |
| `/task/status` | plan/TaskStatus | coordinator → vlm_task_planner, user_input | 단계 완료·실패 시 · reliable + transient_local |
| `/speed_limit` | nav2_msgs/SpeedLimit | rl_nav_tuner → controller_server, rl_nav_correction | 프리셋 변경 시 · reliable |
| `/arm_safety_event` | control/ArmSafetyEvent | arm_safety_monitor → coordinator | 접촉 시 |
| `/motor_status` | control/MotorStatus | BeebotSystemHardware → state_manager | 모터별 Hardware Error (0 정상 · −1 응답 없음 · 비트 1 입력전압 4 과열 8 엔코더 16 전기충격 32 과부하) + 사람이 읽는 이름 + 온도 °C — 바뀔 때만 · reliable + transient_local (펌웨어 `E` 줄 · `S` 끝 온도) |

### Nav2·위치추정 ([04](project/ros/04-nav.md))

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
| `/collision_monitor_state` | nav2_msgs/CollisionMonitorState | collision_monitor → RViz | 걸린 다각형 표시 |

### 베이스 명령 체인 ([06](project/ros/06-base.md))

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

### 수동 조작·운영 모드 ([노드 사전](project/ros/10-catalog.md#manual_control))

| 이름 | 타입 | 보내는 쪽 → 받는 쪽 | 주기 · QoS |
| --- | --- | --- | --- |
| `/cmd/operation_mode` | std_msgs/String | 시뮬레이터 · 운영자 → state_manager | MANUAL · AUTO · EXPLORE(자동 탐사 — 지도 모드 MAPPING일 때만 허락) · TEACH(팔 토크 끄고 손으로 자세 — MAPPING이 아닐 때만) — 조작 모드 명령 (외부 명령은 `/cmd/<이름>`(보내고 끝 — cmd_vel 관례), 결과는 `/state/<이름>` — 둘 다 state_manager만 구독·발행) |
| `/cmd/manual/cmd_vel` | geometry_msgs/TwistStamped | teleop_twist_keyboard · 시뮬레이터 → manual_control | 수동일 때만 넘김, 0.5초 끊기면 0 |
| `/cmd/manual/arm` | sensor_msgs/JointState | 시뮬레이터 · 운영자 → manual_control | joint1~5 rad · gripper_left_joint m |
| `/cmd/manual/reboot_motors` | std_msgs/Empty | 시뮬레이터 · 운영자 → manual_control | 수동 모드에서만. Hardware Error(과부하·과열 등) 난 DYNAMIXEL만 재부팅 → 📞 `/beebotsystem/reboot_motors` (std_srvs/Trigger) → 펌웨어 `R`. 결과는 manual_control 진단 |
| `/state/operation_mode` | std_msgs/String (transient_local) | state_manager → manual_control · coordinator · explorer · vlm_task_planner | AUTO · MANUAL · EXPLORE · TEACH (TEACH면 manual_control이 팔 토크를 끄고 나갈 때 켬, AUTO가 아니면 coordinator는 작업 멈춤, EXPLORE면 explorer가 탐사, planner의 주기 수납 확인은 AUTO일 때만) |
| `/state/devices` | std_msgs/String (transient_local) | state_manager → 시뮬레이터 · 누구나 | **장치 이름만 쉼표 구분** — `pole_front_cam,gripper_cam,opencr,imu,wheels,arm,gripper` · 상태가 아니라 목록 (2초마다 확인, 바뀔 때만) |
| `/state/devices/<이름>` | std_msgs/String (transient_local) | state_manager → 시뮬레이터 · 누구나 | 장치별 상세 — `종류\|active · disconnected · error\|세부 토픽·관절` (예: `camera\|disconnected\|/pole_front_cam/...`) · 장치마다 동적 토픽. `wheels`·`arm`·`gripper`는 그 관절 DYNAMIXEL에 Hardware Error가 있으면 `error` + 세부 끝에 `joint2:과부하` (원본은 `/motor_status`, 재부팅 `/cmd/manual/reboot_motors`) |
| `/state/topics` | std_msgs/String (transient_local) | state_manager → 시뮬레이터 · 누구나 | 전체 ROS 그래프 — `토픽\|타입\|publisher 수` 줄들. `/state/devices`에 없는 새 센서도 여기서 찾음 |
| `/explore/result` | std_msgs/String | explorer → state_manager | 탐사 끝: `DONE …`(프론티어 없음) · `FAILED …` → state_manager가 AUTO로 되돌림 |
| `/state/map_list` | std_msgs/String (transient_local) | state_manager → 시뮬레이터 · 누구나 | 가진 지도 폴더 이름들 (줄바꿈 구분, `<map_dir>/<이름>/rtabmap.db` 있는 것) — 바뀔 때만 |
| `/cmd/restart` | std_msgs/String | 시뮬레이터 [ROS 재시작] · `ros-faker-restart` · 운영자(`ros2 topic pub --once`) → state_manager | **`all [이름:=값 …]`** = 이 launch 전체를 정상 종료 → 같은 명령줄로 다시 (인자를 주면 그 인자만 바꿈, `map:=`을 바꾸면 `map_name:=` 버림 · 테스트베드는 그 사이 `build-ws`) · **`node <이름>`** = 그 노드 프로세스만 같은 명령줄·환경(파라미터 파일·리매핑 그대로)으로 다시 (Nav2 수명주기 노드는 다시 활성화 필요). 실기 bringup · 테스트베드 같은 길. 같은 요청이 두 번 오지 않게 하는 건 보내는 쪽 — 테스트베드 input_manager는 읽은 파일을 지움. 도는 중엔 새 요청 거절 |
| `/state/restartable` | std_msgs/String (transient_local) | state_manager → 시뮬레이터 진단 칸 · 누구나 | `node`로 재시작할 수 있는 노드 `"이름 pid"` 줄들 (이 launch 아래 ROS 노드 프로세스 — 컨트롤러·costmap 하위·내부 노드는 프로세스가 아니라 없음). 2초마다 확인 · 바뀔 때만 |
| `/state/restart` | std_msgs/String (transient_local) | state_manager → 시뮬레이터 상태판 · 누구나 | 마지막 재시작 요청 결과 — `all: 재시작 중 …` · `node perception: 재시작 (pid …)` · `거절: …` · 새로 뜨면 `시작 (pid …)` |
| `/cmd/map_name` | std_msgs/String | 시뮬레이터 지도 목록 · 운영자 → state_manager | 지도 폴더 이름 — `<map_dir>/<이름>/rtabmap.db`를 RTAB-Map `/rtabmap/rtabmap/load_database`로 열고 위치추정 모드로 (결과 `/state/map_name` · `/state/map_mode`) |
| `/cmd/map_mode` | std_msgs/String | 시뮬레이터 · 운영자 → state_manager | MAPPING(스캔) · LOCALIZATION(위치추정) — RTAB-Map 모드 전환 요청 |
| `/state/map_mode` | std_msgs/String (transient_local) | state_manager → 시뮬레이터 · 누구나 | MAPPING · LOCALIZATION · COURSE(정지 지도, 바꿀 수 없음) — RTAB-Map 응답 뒤에만 바뀜 |
| `/state/map_name` | std_msgs/String (transient_local) | state_manager → 시뮬레이터 · 누구나 | 지금 쓰는 지도 폴더 이름 (`<map_dir>/<이름>/` rtabmap.db · map.pgm · map.yaml, 빈 값 = 폴더 없음) — 시뮬레이터 [지도 저장]이 같은 폴더에 씀 |

- `/state/devices`는 **이름 목록만** 낸다. 장치별 `active/disconnected`와 세부 토픽은 `/state/devices/<이름>`에서 본다.
  - `active`: 카메라 `camera_info`, 로봇 `/joint_states`, IMU `/imu`가 **최근 1초 안에 실제로 왔는지**
  - `disconnected`: 해당 데이터가 1초 넘게 없거나 장치가 연결되지 않은 상태
- `/state/topics`는 `get_topic_names_and_types()` + publisher 수를 그대로 내는 **전체 그래프 덤프**다.
  장치 목록에 없는 새 센서/토픽은 여기서 먼저 찾고, 필요하면 `state_manager`의 장치 카탈로그에 추가한다.
- manual_control이 내는 쪽: `/cmd_vel_manual` (geometry_msgs/TwistStamped → twist_mux, 위 체인 참조) · `/arm_controller/joint_trajectory` (trajectory_msgs/JointTrajectory → arm_controller(JTC), 5축 전체 한 점).

### 팔·MoveIt ([05](project/ros/05-arm.md))

| 이름 | 타입 | 보내는 쪽 → 받는 쪽 | 주기 |
| --- | --- | --- | --- |
| `/arm_controller/follow_joint_trajectory` | 액션이지만 토픽 아님 — [§3](#3-액션-목록) 참조 | move_group → arm_controller | 궤적마다 |
| `/joint_states` | sensor_msgs/JointState | joint_state_broadcaster → robot_state_publisher, move_group, arm_safety_monitor(전류 감시), coordinator(현재 팔 각), manual_control(수동 팔 기준 각), state_manager(OpenCR 살아 있나), scan_cam_gate(스캔 자세인가) | 100Hz |
| `/arm_safety_event` | control/ArmSafetyEvent | arm_safety_monitor → coordinator | 접촉 시 |
| `/monitored_planning_scene` | moveit_msgs/PlanningScene | move_group → RViz | 갱신 시 |
| `/arm_controller/controller_state` | control_msgs/JointTrajectoryControllerState | arm_controller → 모니터링·RViz | 상태 표시 |

### ros2_control·진단 ([08](project/ros/08-ctrl.md))

| 이름 | 타입 | 보내는 쪽 → 받는 쪽 | 비고 |
| --- | --- | --- | --- |
| `/robot_description` | std_msgs/String | robot_state_publisher → controller_manager, move_group | transient_local — URDF |
| `/tf`, `/tf_static` | tf2_msgs/TFMessage | robot_state_publisher → 전체 | 관절·고정 링크 |
| `/joint_states` | sensor_msgs/JointState | joint_state_broadcaster → 전체 | 100Hz (위 표 중복 — 양쪽에서 씀) |
| `/dynamic_joint_states` | control_msgs/DynamicJointState | joint_state_broadcaster → 디버그 | 표시용 |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | 모든 🛠 노드·하드웨어 플러그인 → diagnostic_aggregator | 1Hz · OK·WARN·ERROR + 값 |
| `/diagnostics_agg` | diagnostic_msgs/DiagnosticArray | diagnostic_aggregator → 표시·CLI(`run-cli status --diagnostics`) | 계층별 묶음 ([bringup/config/diagnostics.yaml](../lv2_module6/ros2_ws/src/bringup/config/diagnostics.yaml) 그룹) |
| `/diagnostics_toplevel_state` | diagnostic_msgs/DiagnosticStatus | diagnostic_aggregator → 표시 | 전체 한 줄 (가장 나쁜 수준) |
| `/motor_status` | control/MotorStatus | BeebotSystemHardware → state_manager | 위 계획·임무 표 참조 ([정의](#api-msg-motorstatus)) |

- **사람용(진단) ↔ 노드용(토픽) 구분**: `/diagnostics`는 사람이 보는 건강 상태 — 노드는 진단 문자열에 기대지 않는다. 노드가 알아야 하는 사실은 전용 토픽(`/motor_status` 등)으로, 로봇 상태는 state_manager의 `/state/...`로.

<a id="api-actions"></a>
## 3. 액션 목록

| 이름 | 타입 | 요청 → 서버 | 용도 |
| --- | --- | --- | --- |
| `/navigate_to_pose` | nav2_msgs/action/NavigateToPose | coordinator · explorer → bt_navigator | 이동 단계마다 (explorer = 탐색 모드 목표). feedback: 남은 거리 |
| `/compute_path_to_pose` | nav2_msgs/action/ComputePathToPose | bt_navigator → planner_server | 약 1Hz 재계획 |
| `/follow_path` | nav2_msgs/action/FollowPath | bt_navigator → controller_server | 경로 변경 시 |
| `/spin`, `/backup`, `/wait` | nav2_msgs/action/Spin · BackUp · Wait | bt_navigator → behavior_server | 복구 시. BackUp은 0.1m 이하로 제한 (뒤를 못 봄) |
| `/execute_trajectory` | moveit_msgs/action/ExecuteTrajectory | coordinator → move_group | pick·place·inspect 단계마다 — `/plan_kinematic_path`로 받은 궤적 실행 (MTC 안 씀) |
| `/arm_controller/follow_joint_trajectory` | control_msgs/action/FollowJointTrajectory | move_group → arm_controller | `/execute_trajectory` 실행 때 (moveit_controllers.yaml엔 arm_controller만) |
| `/gripper_controller/gripper_cmd` | control_msgs/action/ParallelGripperCommand | coordinator · manual_control → gripper_controller | 열기·닫기 (position = 손가락 하나 이동량 m). MoveIt을 거치지 않음 — parallel_gripper_action_controller. 결과 stalled로 빈손 판단 |

<a id="api-services"></a>
## 4. 서비스 목록

| 이름 | 타입 | 호출 → 서버 | 용도 |
| --- | --- | --- | --- |
| `/score_grasps` | ml/srv/ScoreGrasps | coordinator → grasp_scorer | pick 직전 1회 |
| `/collision_monitor/toggle` | nav2_msgs/srv/Toggle | manual_control → collision_monitor | `/state/operation_mode`가 MANUAL이면 끔(속도 명령 그대로 통과 — 근접 정지 없음) · 나가면 켬. 비상정지 잠금(twist_mux)·가속 제한(velocity_smoother)은 수동도 그대로. collision_monitor가 다시 뜨면 다시 보냄 |
| `/beebotsystem/set_arm_torque` | std_srvs/srv/SetBool | manual_control → BeebotSystemHardware | 운영 모드 TEACH에 들어가면 팔(joint1~5)·그리퍼 토크 끔 · 나가면 JTC·그리퍼 목표를 지금 값으로 맞춘 뒤 켬 (펌웨어 `T`, 바퀴 그대로). 직접 부르면 켤 때 팔이 예전 목표로 튈 수 있음 — `/cmd/operation_mode TEACH`로 |
| `/beebotsystem/reboot_motors` | std_srvs/srv/Trigger | manual_control(`/cmd/manual/reboot_motors`) → BeebotSystemHardware | Hardware Error 난 DYNAMIXEL만 재부팅 (펌웨어 `R` 줄) — 운영자는 토픽 `/cmd/manual/reboot_motors`로 (수동 모드에서만) |
| `/plan_kinematic_path` | moveit_msgs/srv/GetMotionPlan | coordinator → move_group | pick·place·inspect 단계마다 팔 궤적 계획 → `/execute_trajectory` |
| `/apply_planning_scene` | moveit_msgs/srv/ApplyPlanningScene | coordinator → move_group | pick 전 퍽·장애물 등록 |
| `/clear_octomap` | std_srvs/srv/Empty | coordinator → move_group | pick 직전 octomap 초기화 |
| `/controller_server/set_parameters` | rcl_interfaces/srv/SetParameters | rl_nav_tuner → controller_server | FollowPath.max_vel_theta 등 (실행 중 반영되는 것만) |
| `/rtabmap/rtabmap/set_mode_mapping` · `/rtabmap/rtabmap/set_mode_localization` | std_srvs/srv/Empty | state_manager → rtabmap | `/cmd/map_mode`에 따라 매핑 ↔ 위치추정 전환 |
| `/rtabmap/rtabmap/load_database` | rtabmap_msgs/srv/LoadDatabase | state_manager → rtabmap | 저장 지도(`<map_dir>/<이름>/rtabmap.db`) 불러오기 |
| `/arm_controller/follow_joint_trajectory/_action/cancel_goal` | action_msgs/srv/CancelGoal | arm_safety_monitor → arm_controller | 빈 goal_id = 전체 취소 → JTC 현재 자세 유지 |
| `/controller_manager/load_controller` 등 | controller_manager_msgs/srv/* | spawner → controller_manager | load · configure · switch · list (기동 시) |
| `*/change_state` | lifecycle_msgs/srv/ChangeState | lifecycle_manager → Nav2 서버들 | configure → activate |
| `/lifecycle_manager_navigation/manage_nodes` | nav2_msgs/srv/ManageLifecycleNodes | 운영자·launch → lifecycle_manager | 주행 계획 서버 (planner · controller · behavior · bt_navigator) 시작·정지·리셋 |
| `/lifecycle_manager_base/manage_nodes` | nav2_msgs/srv/ManageLifecycleNodes | 운영자·launch → lifecycle_manager | 바퀴 명령 체인 (velocity_smoother · collision_monitor) 시작·정지·리셋 — 지도 없이도 뜸 |

- 아직 코드에 없는 것 (설계만): grasp_scorer → `/compute_ik`(후보 도달 검사 — 지금은 고정 후보), coordinator → `/local_costmap/local_costmap/set_parameters`(퍽을 들면 footprint 확장 — coordinator.hpp TODO).

<a id="api-tf"></a>
## 5. TF

[07](project/ros/07-tf.md). 발행자는 딱 둘 + robot_state_publisher.

| 변환 | 발행자 | 성격 |
| --- | --- | --- |
| map → odom | rtabmap | 가끔 점프 (루프 클로저·재위치 추정) |
| odom → base_footprint | robot_localization ekf | 연속, 드리프트 있음 |
| base_footprint → base_link → 센서·팔 | robot_state_publisher (URDF) | 고정 또는 /joint_states로 변함 |
| *_link → *_optical_frame | robot_state_publisher | 고정 (z 앞 · x 오른쪽 · y 아래) |

- Nav2 `robot_base_frame`, ekf `base_link_frame`, MoveIt 부모를 **모두 base_footprint**로 통일.

<a id="api-qos"></a>
## 6. QoS 규칙

[07 QoS 정리](project/ros/07-tf.md#qoS-정리).

| 데이터 | Reliability | Durability | Depth | 이유 |
| --- | --- | --- | --- | --- |
| 카메라·점구름·IMU | best_effort | volatile | 1~5 | 최신 값만 의미 있음 |
| /cmd_vel 체인 | reliable | volatile | 1 | 명령 유실 방지, 오래된 명령은 버림 |
| /map, /tf_static | reliable | transient_local | 1 | 늦게 붙어도 받음 |
| /task/sub_goals, /task/status | reliable | transient_local | 10 | 재시작해도 마지막 계획 받음 |
| /mission/signal, /mission/qr, /motor_status | reliable | transient_local | 1 | 바뀔 때만 냄 — 늦게 붙어도 마지막 값 |
| /state/... | reliable | transient_local | 1 | state_manager가 바뀔 때만 냄 |
| /tracked_objects | reliable | volatile | 5 | 누락 시 다음 값으로 충분 |

> best_effort 발행자를 reliable 구독자가 받으면 **연결 자체가 안 된다.** 센서 구독은 `rclcpp::SensorDataQoS()`.

<a id="api-change"></a>
## 7. 바꿀 때 절차

1. `.msg`/`.srv`를 고치거나 만든다 (파일명 CamelCase, 필드 snake_case).
2. 새 파일이면 그 패키지 `CMakeLists.txt`의 `rosidl_generate_interfaces(...)`에 추가. 다른 패키지 타입을 쓰면 `DEPENDENCIES`와 `package.xml`에도 추가.
3. `colcon build --symlink-install --packages-above <패키지>` — 쓰는 쪽까지 다시 빌드.
4. 받는 쪽·보내는 쪽 담당자에게 알린다 (PR 설명에 바뀐 필드). 담당표는 [lv2_module6/README.md](../lv2_module6/README.md#담당자).
5. 이 문서(API.md)의 해당 표도 같이 고친다. `ros2 interface show <타입>`으로 확인.
