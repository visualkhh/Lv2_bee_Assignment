[목차](README.md) · [◀ 체크리스트·로드맵](14-check.md)

## 초안 대비 바뀐 점 (검토 반영)

> 원래 "컵 가져오기" 예시로 설계를 검토하며 고친 이력입니다 — 지금 미션(퍽 이송·장애물 회피)과 용어가 다른 행이 있습니다. 구조 결정(계층·안전·학습 규칙)은 그대로 유효합니다.

초안 설계서를 ROS 2 관례·실제 패키지 동작 기준으로 검토하고 고친 목록입니다.

| # | 초안 | 최종본 | 이유 |
| --- | --- | --- | --- |
| 1 | switch_controller로 rl_arm_correction만 끄기 | `enabled` 파라미터로 Δ=0 | JTC가 command_joints로 그 reference에 묶여 있어 뒤만 끌 수 없음 |
| 2 | 프리셋이 DWB critic 가중치 변경 | /speed_limit + kinematic 파라미터만 | critic scale은 초기화 때 읽혀 런타임 변경이 반영 안 될 수 있음 |
| 3 | "토픽으로는 controller_server를 바꿀 수 없다" | /speed_limit 토픽 사용 | controller_server가 nav2_msgs/SpeedLimit을 구독 |
| 4 | behavior_server 없음, lifecycle_manager 1개 | behavior_server 추가, 2개로 분리 | 기본 BT가 Spin·BackUp·Wait 호출 |
| 5 | pole_cam depth → depthimage_to_laserscan → /scan | /scan 없이 depth를 PointCloud2로 직접 사용 | 하향 카메라를 스캔으로 바꾸면 바닥을 장애물로 잡음 |
| 6 | camera_info, depth/points 누락 | 1장에 추가 | 3D 투영·포인트클라우드 생성에 필요 |
| 7 | MoveGroup 액션으로 pick | MTC (직선 접근·attach·후퇴) | 직선 이동과 attach는 MoveGroup 하나로 안 됨 |
| 8 | octomap 속 목표 컵 처리 없음 | /clear_octomap + ACM 허용 | 컵 voxel 때문에 접근 계획이 거부됨 |
| 9 | 4축 팔 한계 미반영 | IK 가능 후보만, pick_ik | OM-X는 6D 자세를 못 만듦 |
| 10 | 테이블 높이 0.72~0.75m | 낮은 테이블 0.45m | OM-X를 Waffle에 얹으면 그리퍼 최고 높이 약 0.6m |
| 11 | 바퀴 4.2 · 5.9rad/s | 2.72 · 5.76rad/s | 초안 값은 Burger(간격 0.16m) 기준, Waffle은 0.287m |
| 12 | 프리셋 B 상한 0.15인데 보정 후 0.16 | 보정 후 상한 재클리핑 | residual이 tuner 상한을 넘지 않게 |
| 13 | VLM이 2순위 파지점 선택 | coordinator가 재시도, VLM은 순서만 | 계층 원칙(판단 권한 한 층에 하나) |
| 14 | "폭 센서 값 0" | 그리퍼 관절 위치·GripperCommand 결과 | OM-X에 폭 센서 없음 |
| 15 | 팔 U2D2, controller_manager 2개 | OpenCR, controller_manager 1개 | turtlebot3_manipulation 공식 구성 |
| 16 | odom→base_link를 diff_drive_controller | robot_localization EKF | IMU 융합으로 미끄러짐 기본 보정, RL 비교 기준 |
| 17 | 추종 오차 감지를 직접 작성 | JTC constraints 내장 기능 | 직접 작성은 전류 감지만 |
| 18 | 이동·팔 조율 규칙 없음 | stow, footprint 확장, 정지 시에만 팔 | 모바일 매니퓰레이터 기본 규칙 |
| 19 | rl_nav_tuner를 Isaac Lab RL로 | 규칙으로 시작 | 병렬 환경에 DWB가 없어 프리셋 효과를 시뮬할 수 없음 |
| 20 | slam_toolbox + amcl (LaserScan 기반) | RTAB-Map 매핑·위치추정 | LiDAR가 없어 LaserScan 기반 SLAM·위치추정은 화각 87° 가짜 스캔에 의존하게 됨 |
| 21 | QoS 언급 없음 | QoS 표 추가 | best_effort ↔ reliable 불일치는 연결 자체가 안 됨 |
| 22 | gripper_cam RGB, LiDAR 탑재 가정 | 카메라 2대 모두 Intel RealSense, LiDAR 없음 | 실제 하드웨어 구성 반영 — 근접 점구름은 gripper_cam, 장애물·SLAM은 pole_cam 담당 |
| 23 | 후방 감지 가정 | BackUp 복구 제한, voxel layer 보수적 clearing | 전방 카메라 하나라 옆·뒤 사각지대 |
| 24 | rtabmap Grid/FromDepth | Grid/Sensor 0 + scan_cloud = points_filtered (처음엔 1 depth — 팔이 지도에 벽으로 남아 바꿈) | 현재 rtabmap에서 Grid/FromDepth는 Grid/Sensor(0 레이저, 1 depth, 2 둘 다)로 바뀜 — rtabmap Parameters.h 확인 |
| 25 | 공식 TurtleBot3 플러그인으로 전류(effort) 사용 | 플러그인 fork해서 effort state 추가 | 공식 jazzy 플러그인은 position·velocity만 내보냄 — OpenCR 제어 테이블의 present_current를 읽어 effort로 내보내야 arm_safety_monitor·rl_arm_correction이 동작 |
| 26 | /failure_report (실패할 때만, coordinator → VLM) | /task_status (RUNNING·SUCCEEDED·FAILED + reason) | 실패만이 아니라 진행·완료도 같은 토픽으로 — VLM은 FAILED에 재계획, user_input는 완료 안내·진행 표시에 사용 |

---

[목차](README.md) · [◀ 체크리스트·로드맵](14-check.md) · [원본 페이지](https://dooboostore.github.io/ros2?concept=design&sec=changes)
