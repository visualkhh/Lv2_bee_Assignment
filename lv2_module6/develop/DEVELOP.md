# DEVELOP — 처음 들어온 개발자용

lv2_module6 워크스페이스에서 개발·빌드·테스트하는 방법입니다. 프로젝트 개요·패키지·담당은 [README.md](../README.md), 설계는 [documents/ros2-design](../../documents/ros2-design/README.md), 브랜치·커밋·PR 규칙은 [CONTRIBUTING.md](../../CONTRIBUTING.md)를 봅니다.

- ROS 2 **Lyrical** (Ubuntu 26.04) · **C++** (ament_cmake, rclcpp)
- 개발·테스트는 **도커 테스트베드**에서 합니다 (로컬에 ROS를 설치하지 않아도 됨). CI도 같은 컨테이너에서 같은 검사를 돌립니다.

## 1. 처음 셋팅

필요한 것: Git, [Docker Desktop](https://www.docker.com/products/docker-desktop/) (또는 Linux Docker Engine + compose)

```bash
git clone <저장소>
cd Lv2_bee_Assignment
git switch develop                     # 작업은 develop에서 브랜치를 따서 (CONTRIBUTING.md)

cd docker
docker compose build                   # 이미지: ROS 2 Lyrical + ros2_control + EKF + arduino-cli(OpenCR) (처음 한 번, 몇 분)
docker compose up -d
docker compose exec lyrical bash       # 컨테이너 셸 — 이후 명령은 전부 여기서
test-all                               # 빌드 + 전체 검사. ALL PASS면 셋팅 끝
```

컨테이너는 워크스페이스 전체(`lv2_module6/ros2_ws` → `/ws`)를 마운트합니다. 빌드 결과·로그도 호스트에서 바로 봅니다.
같은 LAN의 다른 팀 스택과 DDS 토픽이 섞이지 않게 `ROS_DOMAIN_ID=8` (docker-compose.yml · `~/.bashrc`) — 컨테이너 안 명령은 compose가 자동 적용, 호스트는 셸 설정.
DDS 환경 (docker-compose · CI 공통): `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` (ROS 기본값 — LOCALHOST면 Fast DDS가 localhost 참가자 번호 몇 개에만 유니캐스트로 discovery를 보내 노드 40여 개 중 일부가 서로 안 보였음: Nav2 관리자 멈춤 · `/map` 발행자 0 · CLI가 노드 못 찾음. 다른 팀과는 `ROS_DOMAIN_ID=8`로 분리) · `FASTDDS_DEFAULT_PROFILES_FILE=…/ros_system/config/fastdds.xml` (Fast DDS TypeLookup 끔 — 없으면 노드 40개에서 UDP 폭주로 컨트롤러가 못 뜸).

| 호스트 | 컨테이너 | 용도 |
| --- | --- | --- |
| `lv2_module6/ros2_ws` | `/ws` | ROS 2 워크스페이스 전체 (src + urdf + build/install/log — 호스트에서 편집·열람) |
| `lv2_module6/firmware` | `/firmware` | OpenCR 펌웨어 스케치 |
| `lv2_module6/debug` | `/debug` | 토픽 기록 · 시뮬레이터 연동 · 가상 시리얼(input/hardware/serial·output/hardware/serial) (gitignore). **양방향**: 호스트→컨테이너(`input/topic/` 명령·수동, `input/hardware/camera/` 카메라 장면, `service/*/request`·`action/*/goal·cancel`)와 컨테이너→호스트(`output/topic/`·`service/*/response·type|info`·`action/*/accept·feedback·result`, `output/hardware/serial`)가 같은 폴더를 쓴다. 펌웨어 빌드 결과는 `firmware/build/` (gitignore). 소유권·권한은 아래 "공유 폴더 권한" 참조 |

Dockerfile·`docker/*.sh`를 고쳤으면 `docker compose build` 후 `docker compose up -d`로 다시 띄웁니다 (스크립트는 이미지에 복사돼 있음).

### 공유 폴더 권한 (`lv2_module6/debug`)

이미지에 `USER` 지정이 없어 컨테이너 안 프로세스는 uid 0(root)으로 돕니다. 바인드 마운트는 호스트 파일시스템을 그대로 공유하고 user namespace remapping을 쓰지 않으므로(기본값), 컨테이너가 만든 파일·디렉토리는 호스트에서 root 소유로 보입니다. `sudo`라서가 아니라 uid 문제입니다.

그래서 양쪽 다 쓸 수 있게 디렉토리는 777로 유지합니다:

- `docker/entrypoint.sh`: 기동 시 `input/`·`input/hardware/camera/`·`output/topic/`·`service/`·`action/`·`interface/`·`output/` 미리 생성 + `chmod -R 777 /debug`
- `faker` 노드(`monitor_manager`·`service_action_monitor`·`service_action_input`): 디렉토리 생성 직후 `chmod 0777` — 실행 중 새로 생기는 `output/topic/<토픽>/`·`service/<이름>/`·`action/<이름>/`도 호스트가 rename(원자적 쓰기)으로 교체 가능
- 파일 자체는 644여도 된다: 읽는 쪽은 읽기만, 쓰는 쪽은 디렉토리 권한으로 rename 하므로

`PermissionError`가 나면 컨테이너에서 소유권 회수 — 호스트 `sudo`는 비밀번호가 필요할 수 있으니 컨테이너(root) 경유가 확실:

```bash
cd docker && docker compose exec -u root lyrical bash -c "chown -R 1000:1000 /debug && chmod -R 777 /debug"
```

참고: `build/`·`install/`·`log/`는 호스트에 그대로 남으므로 재생성 뒤에도 `colcon build` 없이 바로 쓸 수 있습니다. 빌드가 꼬였다 싶으면 `install`·`build` 지우고 처음부터 다시 합니다(지울 때 소유권 주의 — 위 chown 참고).

## 2. 컨테이너 안 명령

| 명령 | 하는 일 |
| --- | --- |
| `test-all` | **push 전 필수.** CI와 같은 검사: test-serial → test-firmware → test-packages → test-hardware → test-bringup → test-faker(정상) → test-faker(파지 실패·재계획) |
| `test-serial` | 가상 시리얼(`/dev/opencr` — 실기 udev와 같은 이름, 컨테이너는 PTY 링크) ↔ `debug/input/hardware/serial`·`output/hardware/serial` 파일 왕복 |
| `test-firmware` | OpenCR 펌웨어 컴파일 → 가상 시리얼로 업로드 → 보낸 바이트가 .bin과 같은지 (결과 `debug/firmware/`) |
| `test-packages [패키지...]` | colcon build + 패키지별 테스트 (`colcon test`). 예) `test-packages plan` |
| `test-hardware` | controller_manager를 실기 모드(`mock_hardware:=false`)로 가상 시리얼에 붙여 펌웨어 프로토콜 확인 (C 명령 송신 · S 상태 → `/joint_states`) |
| `test-bringup` | 🛠 노드가 모두 뜨는지 (perception · user_input · vlm_task_planner · coordinator · arm_safety_monitor · manual_control · body_filter · state_manager) |
| `test-faker [초\|0] [launch 인자]` | 더미로 전체 흐름 → `/task_status` SUCCEEDED면 PASS. `0`이면 띄워 두기 (Ctrl+C 종료, 시뮬레이터 [ROS 재시작] = `/cmd/restart all` → state_manager가 ROS 전부 껐다 `build-ws` 후 같은 인자로 다시 띄움 — 다시 뜬 launch가 끝날 때까지 기다림). 기본은 fake_opencr가 가상 시리얼 반대편 OpenCR 역할 (`mock_hardware:=true`면 OpenCR 없이 mock — 물리 없음), `ml:=real`이면 ML 대역 대신 진짜 ml 노드. 지도 `map:=course`(기본, 코스 지도 외움) · `map:=slam`(RTAB-Map 스캔 → 새 폴더 `ros2_ws/map/<날짜_시각>/rtabmap.db`) · `map:=saved`(저장 지도로 위치추정 — default 폴더, 없으면 가장 최근, `map_name:=<폴더>`로 고름) |
| `ros-kill` | ROS 전부 끄기 + 캐시 정리: launch에 SIGINT → 최대 20초 대기(RTAB-Map DB 닫기) → 안 죽은 것만 SIGKILL → ros2 daemon stop · `/dev/shm/fastdds_*` 삭제 |
| `ros-restart [launch 인자]` | **진짜 bringup** 다시 띄우기: `ros-kill` → `build-ws` → `ros2 launch bringup bringup.launch.py [인자]` (🛠 노드 + ros_system 📦, Ctrl+C 종료, 로그 `/tmp/bringup.log`). 컨테이너엔 RealSense·OpenCR가 없으니 보통 `use_ros_system:=false` |
| `ros-faker-restart [launch 인자]` | **테스트베드(faker)** 다시 띄우기: state_manager가 떠 있으면 `/cmd/restart all [인자]` 요청(시뮬레이터 [ROS 재시작]과 같음 — 끄고 `build-ws` 후 같은 인자로), ROS가 막혀 state_manager가 없으면 `ros-kill` 후 `test-faker 0 [인자]` |
| (기록) | `test-faker` 실행 중 **monitor_manager**(노드 하나)가 남김 (시작할 때 `output/topic/`·`topic/`·`node/`·`pkg/`를 비우고 새로 — 지난 실행 값이 남지 않게, `input/`·`output/hardware/`는 그대로): `output/topic/<토픽>/message`·`image.jpg`, `topic/<토픽>/info`·`type`·`topic/list`, `node/<노드>/info`·`params`·`node/list`, `pkg/<패키지>/prefix`·`executables`·`pkg/list` (예전 test-logger — 토픽마다 ros2 CLI를 띄워 CPU를 먹어서 없앰). 빌드 뒤 `install/` 실파일 복사본은 `debug/install` (`build-ws`) |

```bash
# 띄워 두고 직접 보기 — 셸 두 개 (docker compose exec lyrical bash 를 하나 더)
test-faker 0                                 # 셸 1
ros2 topic list                              # 셸 2
ros2 topic echo /task_status
ros2 run plan user_input                     # 텍스트 명령 직접 입력 (표준입력 한 줄 = 명령 하나 → /cmd/user_command)
# 참고: user_input의 키보드 입력은 터미널에 직접 띄웠을 때만 열린다(isatty).
# launch(test-faker)로 뜨면 stdin이 터미널이 아니라서 안 열리고, launch `command:=` 인자(기본값 "빨간 퍽을 목표 구역으로 옮겨") 한 번만 보낸다.
# 평소엔 시뮬레이터 조작창 prompt(파일 → input_manager → /cmd/user_command)가 유저 입력 역할을 한다.
ros2 control list_controllers                # ros2_control 상태 (joint_state_broadcaster … gripper_controller)
ros2 topic echo /diagnostics_agg             # 노드별 상태 (OK · WARN · ERROR)
ros2 topic pub --once /cmd/operation_mode std_msgs/msg/String '{data: MANUAL}'   # 수동 모드 (AUTO = 자동, state_manager → /state/operation_mode)
ros2 topic pub --once /cmd/map_mode std_msgs/msg/String '{data: MAPPING}'   # 스캔 (LOCALIZATION = 위치추정, map:=slam·saved일 때)
ros2 topic echo /state/map_mode                                            # 상태는 state_manager만 /state/<이름> 으로 발행
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/cmd/manual/cmd_vel   # 키보드 조종
ros2 topic pub --once /cmd/e_stop std_msgs/msg/Bool '{data: true}'             # 비상정지 (false로 해제)
```

### 호스트(로컬 ROS)에서 직접 — 컨테이너 없이

ROS 2 Lyrical이 깔린 리눅스면 `docker/*.sh`를 그대로 씁니다. 경로는 자동 감지라 지정할 것 없음 (`/ws` 있으면 컨테이너, 없으면 레포 안 `ros2_ws`; `debug`도 `/debug`→없으면 `lv2_module6/debug`):

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=8   # 같은 LAN 다른 팀과 분리 (컨테이너 compose·~/.bashrc와 같은 값)
./lv2_module6/ros2_ws/rebuild.sh              # 최초 1회 풀빌드 (build/·install/·log 재생성)
docker/test-faker.sh 0                        # 띄워 두기 — 이후 셸에선 source lv2_module6/ros2_ws/install/setup.bash
```

자주 쓰는 건 `lv2_module6/develop/`에 묶어뒀습니다 (어디서 실행해도 됨):

```bash
lv2_module6/develop/faker.sh                  # = docker/test-faker.sh 0 (인자 그대로 전달)
lv2_module6/develop/rviz.sh                   # RViz2 + 기본 구성(URDF·TF·Map, develop/rviz_default.rviz)
lv2_module6/develop/gazebo.sh                 # Gazebo + 코스 월드(course/worlds/twin_closed_maze.sdf)
lv2_module6/develop/teleop.sh [twist토픽]          # 키보드 조종 (기본 /gz/cmd_vel — testbed 안 건드림)
```

- 컨테이너와 같은 셸 스크립트라 같은 검사 (`test-packages`, `test-bringup`, `test-faker`)가 돕니다. `sh script`로 실행해도 bash로 재실행됩니다.
- 빌드 산출물 소유자가 본인이라 `chown` 문제 없음. 단 `build/`·`install/`·`log/`는 컨테이너와 이름이 같고 CMakeCache에 절대경로가 박히므로 **컨테이너와 호스트에서 동시 빌드 금지** — 한 번에 한 곳에서만.
- 호스트 apt 목록은 §3 참고 (`ros-lyrical-rtabmap-msgs` 포함). rosdep는 lyrical을 몰라서 그냥 apt.
- 가상 시리얼(`/dev/opencr`)이 필요한 검사(`test-serial`, `test-hardware`, 펌웨어 업로드)는 컨테이너(entrypoint의 socat 브릿지)에서만 됩니다. 호스트는 mock 모드(`mock_hardware:=true`, 기본값) 위주로 씁니다. push 전 전체 게이트(`test-all`)·CI 정본은 그대로 컨테이너입니다.

## 3. 빌드

```bash
cd /ws
colcon build --symlink-install                              # 전체
colcon build --symlink-install --packages-select plan       # 한 패키지만
colcon build --symlink-install --packages-up-to plan        # plan과 plan이 의존하는 것까지
source install/setup.bash                                   # 빌드 후 새 셸마다 (컨테이너 .bashrc가 자동으로 함)
```

- `--symlink-install`: launch·yaml·URDF는 다시 빌드하지 않아도 수정이 반영됩니다. C++은 다시 빌드해야 합니다.
- 의존 방향: `perception`, `control`, `manager` ← `ml` ← `plan` ← `bringup` · `faker`. 아래쪽 패키지(메시지 포함)를 고치면 위쪽도 다시 빌드됩니다.
- 호스트에서 빌드할 땐 `ros2_ws/rebuild.sh` (build/·install/·log/ 삭제 후 풀빌드 — 컨테이너와 같은 이름, 한 번에 한 곳에서만).
- 호스트 의존성: ROS 2 Lyrical (`/opt/ros/lyrical`) + 아래 패키지. rosdep는 lyrical을 몰라서 그냥 apt로 받는다.
  ```bash
  sudo apt install -y ros-lyrical-nav2-msgs ros-lyrical-moveit-msgs ros-lyrical-moveit-task-constructor-msgs \
    ros-lyrical-control-msgs ros-lyrical-controller-interface ros-lyrical-hardware-interface \
    ros-lyrical-pluginlib ros-lyrical-std-srvs ros-lyrical-controller-manager \
    ros-lyrical-joint-state-broadcaster ros-lyrical-imu-sensor-broadcaster ros-lyrical-diff-drive-controller \
    ros-lyrical-joint-trajectory-controller ros-lyrical-parallel-gripper-controller \
    ros-lyrical-robot-state-publisher ros-lyrical-xacro ros-lyrical-ros2controlcli ros-lyrical-twist-mux \
    ros-lyrical-diagnostic-updater ros-lyrical-diagnostic-aggregator ros-lyrical-teleop-twist-keyboard \
    ros-lyrical-robot-localization ros-lyrical-rosx-introspection ros-lyrical-cv-bridge \
    ros-lyrical-nav2-lifecycle-manager ros-lyrical-nav2-bt-navigator ros-lyrical-nav2-behavior-tree \
    ros-lyrical-nav2-planner ros-lyrical-nav2-navfn-planner ros-lyrical-nav2-controller \
    ros-lyrical-nav2-dwb-controller ros-lyrical-nav2-behaviors ros-lyrical-nav2-costmap-2d \
    ros-lyrical-nav2-velocity-smoother ros-lyrical-nav2-collision-monitor ros-lyrical-nav2-map-server \
    ros-lyrical-moveit-ros-move-group ros-lyrical-moveit-kinematics ros-lyrical-moveit-planners-ompl \
    ros-lyrical-moveit-simple-controller-manager ros-lyrical-rtabmap-msgs ros-lyrical-rtabmap-ros ros-lyrical-ament-cmake-pytest
  ```
  위 목록 = `docker/Dockerfile`의 apt 목록 (그게 컨테이너 정본 — Dockerfile에 추가하면 여기도 같이 고친다).
  pick_ik(MoveIt IK)는 Lyrical apt에 없어 소스 빌드 (Dockerfile과 같은 커밋):
  ```bash
  sudo apt install -y ros-lyrical-rsl ros-lyrical-generate-parameter-library ros-lyrical-moveit-core ros-lyrical-tf2-kdl librange-v3-dev libfmt-dev libexpected-dev
  mkdir -p /tmp/pick_ik_ws/src && cd /tmp/pick_ik_ws/src && git clone https://github.com/PickNikRobotics/pick_ik.git && (cd pick_ik && git checkout 7c97fe2)
  cd /tmp/pick_ik_ws && source /opt/ros/lyrical/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  sudo cp -r install/pick_ik/. /opt/ros/lyrical/
  ```
  YOLO perception은 pip `torch`(CPU) · `ultralytics`도 필요 (Dockerfile 참고). 호스트 빌드는 참고용, CI·테스트베드 정본은 컨테이너 빌드.

## 4. 커스텀 메시지

별도 interfaces 패키지 없이 **그 메시지를 내보내는 패키지**가 `rosidl_generate_interfaces`로 갖고 있습니다.

| 패키지 | 정의 | 내보내는 노드 |
| --- | --- | --- |
| perception | `msg/TrackedObject.msg` · `msg/TrackedObjectArray.msg` | perception → `/tracked_objects` |
| control | `msg/ArmSafetyEvent.msg` | arm_safety_monitor → `/arm_safety_event` |
| ml | `srv/ScoreGrasps.srv` | grasp_scorer (서버) ← coordinator |
| plan | `msg/SubGoal.msg` · `msg/SubGoalArray.msg` · `msg/TaskStatus.msg` | vlm_task_planner → `/sub_goals`, coordinator → `/task_status` |

```bash
ros2 interface list | grep -E "^(perception|control|ml|plan)/"   # 빌드된 커스텀 타입
ros2 interface show plan/msg/TaskStatus                           # 필드 확인
```

### 필드를 바꾸거나 메시지를 추가할 때

1. `.msg`/`.srv` 파일을 고치거나 만든다. 파일 이름은 CamelCase, 필드는 snake_case.
2. 새 파일이면 그 패키지 `CMakeLists.txt`의 `rosidl_generate_interfaces(...)` 목록에 추가. 다른 패키지 타입(예: `geometry_msgs/Pose`)을 쓰면 `DEPENDENCIES`와 `package.xml`에도 추가.
3. 빌드: `colcon build --symlink-install --packages-above <패키지>` — 그 메시지를 쓰는 패키지까지 다시 빌드 (메시지가 바뀌면 쓰는 쪽도 반드시 다시 빌드).
4. 받는 쪽·보내는 쪽 담당자에게 알린다 (PR 설명에 바뀐 필드).

### C++에서 쓰기

헤더 이름은 타입 이름을 snake_case로: `TrackedObjectArray` → `perception/msg/tracked_object_array.hpp`

```cpp
#include "perception/msg/tracked_object_array.hpp"
perception::msg::TrackedObjectArray msg;
```

| 어디서 쓰나 | package.xml | CMakeLists.txt |
| --- | --- | --- |
| **같은 패키지** 노드 | (이미 있음) | `rosidl_get_typesupport_target(msgs_cpp ${PROJECT_NAME} rosidl_typesupport_cpp)` 후 `target_link_libraries(<노드> ${msgs_cpp})` |
| **다른 패키지** 노드 | `<depend>perception</depend>` | `find_package(perception REQUIRED)` 후 `target_link_libraries(<노드> ${perception_TARGETS})` |

> Lyrical에서는 `ament_target_dependencies`가 없어졌습니다 — 항상 `target_link_libraries` + `<패키지>_TARGETS`.

## 5. 노드 작성 규칙

- 노드 하나 = 파일 세 개, 클래스는 `namespace <패키지>` 안:
  | 파일 | 내용 |
  | --- | --- |
  | `include/<패키지>/<노드>.hpp` | 노드 설명(입출력·담당·TODO) + 클래스 **선언**만 (멤버 함수 선언 · 멤버 변수) |
  | `src/<노드>.cpp` | 멤버 함수 **구현** (생성자 · 콜백 · 로직) |
  | `src/<노드>_main.cpp` | `main`만 — `rclcpp::spin(std::make_shared<<패키지>::<클래스>>())` |
  ros2_control 플러그인(BeebotSystemHardware · RlArmCorrection)도 hpp(선언) + cpp(구현 + PLUGINLIB_EXPORT_CLASS).
- 빌드: 패키지의 노드 구현(`src/<노드>.cpp`)은 라이브러리 `<패키지>_nodes` 하나로 묶고, 실행 파일(`src/<노드>_main.cpp`)과 gtest가 그걸 링크한다 — 테스트가 같은 클래스를 같은 프로세스에 띄울 수 있게.
- 새 노드: 위 세 파일 → `CMakeLists.txt`의 `add_library(${PROJECT_NAME}_nodes …)`에 `src/<노드>.cpp`, `foreach(node …)`·`install(TARGETS …)`에 이름 추가 →
  `bringup/launch/bringup.launch.py`에 `node('<패키지>', '<실행 파일>')` → `docker/test-bringup.sh`의 `NODES`에 노드 이름 → `bringup/config/diagnostics.yaml` 묶음에 이름.
- 토픽·메시지·QoS는 설계 문서 표를 따른다. 센서는 `rclcpp::SensorDataQoS()`(best_effort), 명령은 reliable, 가끔만 바뀌는 상태(`/sub_goals`, `/task_status`)는 reliable + transient_local.
- 학습 노드는 `enabled` 파라미터로 끌 수 있어야 하고, 꺼지거나 모델 파일이 없으면 기본 동작(Δ=0 · 규칙)으로 돌아야 한다.
- **도커·CI에는 GPU·모델이 없다** — 테스트베드는 진짜 ml 노드 대신 **faker ML 대역**(ml 노드와 1:1 — `fake_grasp_scorer` · `fake_rl_nav_tuner` · `fake_rl_nav_correction`, 컨트롤러 `faker/FakeRlArmCorrection`)을 같은 이름·토픽으로 띄운다 (`ml:=fake` 기본, `bringup use_ml:=false`).
  진짜로 돌려 보려면 `test-faker 0 ml:=real`. ml 패키지의 gtest(`colcon test`)는 도커에서 돌므로 **CPU만으로 도는 테스트**로 둔다.
  ml 노드의 토픽·서비스·파라미터를 바꾸면 같은 이름의 대역 `faker/src/fake_<노드>.cpp`도 같이 맞춘다. 하나만 진짜로: `grasp_scorer:=real` (rl_nav_tuner · rl_nav_correction · rl_arm_correction도 같음)
- **상태는 `/diagnostics`로** — 노드 클래스에 `diagnostic_updater::Updater diag_{this};` 를 두고 `diag_.add("상태", …)`에서
  `s.summary(OK|WARN|ERROR, "한 줄")` + `s.add("키", 값)`. 1초마다 발행되고 bringup의 `diagnostic_aggregator`가 묶는다
  (`bringup/config/diagnostics.yaml` — 새 노드는 이름을 해당 묶음 `startswith`에 추가). `test-bringup`이 노드마다 상태가 오는지 확인한다.
- **수동 모드를 존중** — 로봇을 움직이는 노드는 `/state/operation_mode`가 "MANUAL"이면 자기 명령을 멈춘다 (coordinator처럼). 바퀴는 twist_mux가 이미 수동을 우선한다.
- 담당자가 채울 곳은 `TODO(이름)` 주석으로 표시돼 있다.

## 6. 파라미터 (config yaml)

노드 파라미터 값은 코드가 아니라 **패키지 `config/<패키지>.yaml`** 에서 관리합니다 (ROS 2 관례: 노드 이름 → `ros__parameters`).

| 파일 | 노드 |
| --- | --- |
| `perception/config/perception.yaml` | perception |
| `control/config/control.yaml` | arm_safety_monitor · manual_control · body_filter |
| `manager/config/manager.yaml` | state_manager (지도 모드 시작 값 — faker가 `map:=`에 맞춰 덮어씀) |
| `ml/config/ml.yaml` | grasp_scorer · rl_nav_tuner · rl_nav_correction |
| `plan/config/plan.yaml` | user_input · vlm_task_planner · coordinator · explorer(자동 탐사) |
| `faker/config/faker.yaml` | pole_cam · gripper_cam · move_group · fake_opencr · monitor_manager · input_manager (더미) |
| `ros_system/config/moveit/*.yaml` + `beebot.srdf` | 진짜 move_group 설정 (arm 그룹 · IK pick_ik · OMPL · arm_controller) — 팔이 5축이라 KDL은 '위치 + 위→아래' 목표를 못 풀어 pick_ik (소스 빌드, §3) — `use_moveit:=true` |
| `ros_system/config/controllers.yaml` | ros2_control 컨트롤러 (rl_arm_correction 포함) |
| `ros_system/config/ekf.yaml` | robot_localization EKF |

```yaml
coordinator:               # 노드 이름
  ros__parameters:
    max_grasp_tries: 2     # 이름은 코드의 declare_parameter("max_grasp_tries", …)와 같아야 한다
```

- 순서: 코드의 기본값 → yaml → launch 인자(`learning_enabled`, `command`, `fail_count` 등)가 덮어씀.
- 새 파라미터: 코드에 `declare_parameter<타입>("이름", 기본값)` + yaml에 같은 이름. **yaml 이름 오타는 ROS가 조용히 무시**하므로 `test-bringup`이 yaml 키가 노드에 실제로 선언돼 있는지 검사한다.
- yaml은 `--symlink-install`이라 다시 빌드 없이 노드만 다시 띄우면 반영. 실행 중 확인·변경: `ros2 param get /coordinator max_grasp_tries`, `ros2 param set …` (코드가 실행 중 값을 다시 읽는 파라미터만 효과 있음).

## 7. 테스트 작성

ROS 2 권장 방식대로 C++은 **gtest**(`ament_cmake_gtest`)를 `colcon test`로 돌립니다. 패키지마다 기본 테스트가 하나씩 있으니 복사해서 늘리면 됩니다.
Python 노드(perception)는 **pytest**(`ament_cmake_pytest` + `test/test_node.py`) — `colcon test`가 같이 돌립니다.

| 패키지 | 테스트 | 확인하는 것 |
| --- | --- | --- |
| perception | `test/test_node.py` (pytest, YOLO 없이 빈 발행·진단) | 모델 없이도 노드가 떠서 빈 `/tracked_objects`가 나온다 · QR · 색 분류 · 영상 시각 TF(없으면 최신으로) · 두 카메라 융합(오차 축소 · 1:1 배정 · 클래스 분리) |
| control | `test/test_arm_safety_monitor.cpp` | 전류 과다가 0.1초 넘게 이어지면 `/arm_safety_event` |
| control | `test/test_manual_control.cpp` | `/state/operation_mode`가 MANUAL일 때만 `/cmd/manual/cmd_vel` → `/cmd_vel_manual` |
| manager | `test/test_state_manager.cpp` | `/cmd/operation_mode` → `/state/operation_mode` · `/cmd/map_mode` → RTAB-Map 응답 뒤 `/state/map_mode` (서버 없거나 COURSE면 그대로) |
| control | `test/test_body_filter.cpp` | URDF 충돌 상자(+padding) 안의 점은 빠지고 밖의 점만 `points_filtered`로 나간다 |
| ml | `test/test_rl_nav_correction.cpp` | 보정 출력이 `/speed_limit` 상한으로 잘린다 |
| plan | `test/test_explorer.cpp` | 프론티어(빈 칸·모르는 칸 경계)를 덩어리로 묶어 중심 칸을 고르고, 작은 덩어리·모르는 곳 없는 지도는 빈 결과 |
| plan | `test/test_coordinator.cpp` | 허용 안 된 동작이 든 계획은 FAILED로 거부 · 수동 모드로 바뀌면 진행 중 계획을 멈춤(manual_override) |
| ros_system | `CMakeLists.txt` `test_urdf` | URDF가 xacro로 펼쳐지고 트리가 올바르다 (`check_urdf`) |
| bringup · faker | `CMakeLists.txt` `test_launch` | launch 파일이 오류 없이 읽힌다 |

gtest 하나의 모양: 노드 클래스와 `probe` 노드를 같은 프로세스에 띄우고, 입력을 넣으며 원하는 출력이 올 때까지 spin.

```cpp
TEST_F(RosTest, ClipsToSpeedLimit) {
  auto node = std::make_shared<RlNavCorrection>();           // 시험할 노드
  auto probe = std::make_shared<rclcpp::Node>("probe");      // 입력을 넣고 출력을 받는 쪽
  // probe에 publisher(입력) · subscription(출력) 만들기 …
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  ASSERT_TRUE(spin_until(exec, [&]() { /* 입력 publish */ }, [&]() { /* 원하는 출력이 왔나 */ }));
}
```

새 테스트 파일을 추가하면 그 패키지 `CMakeLists.txt`의 `if(BUILD_TESTING)` 블록에 등록합니다:

```cmake
ament_add_gtest(test_foo test/test_foo.cpp TIMEOUT 60 ENV ROS_DOMAIN_ID=104)
target_link_libraries(test_foo <노드가 쓰는 타깃들>)
```

- `ROS_DOMAIN_ID`는 테스트마다 다르게 (perception 101 · control 102·105 · ml 103 · plan 104) — 다른 테스트나 띄워 둔 faker와 토픽이 섞이지 않게.
- 실행: `test-packages <패키지>` 또는 `colcon test --packages-select <패키지> && colcon test-result --verbose`
- 결과: `/ws/build/<패키지>/test_results/`

## 8. 로봇 모델 (URDF)

모델 하나 = 폴더 하나 `urdf/<이름>/urdf.xacro` (+ 그 폴더 `meshes/`) — launch 인자 `urdf:=<이름>`으로 고른다 (기본 `beebot_mesh`, ros_system · bringup · faker 공통. 메시 경로는 launch가 `mesh_uri:=package://ros_system/urdf/<이름>/meshes`로 넘김). 시뮬레이터 [로봇 모델]에서 폴더를 고르면 확인 후 `/cmd/restart urdf:=<이름>`으로 다시 띄운다 (URDF는 기동 때만 읽힘). 빌드가 `urdf/` 전체를 설치하므로 새 폴더·수정은 `build-ws` 뒤에 반영.

`urdf/beebot_mesh/urdf.xacro` — TurtleBot3 Waffle Pi + 앞쪽 상판 위 OpenMANIPULATOR-X(RM-X52) + 손목 roll + 그리퍼(위에 gripper_cam) + 뒤쪽 전원선 가이드 폴(pole_cam) + 바구니(적재함) 충돌체. Fusion 모델에서 뽑은 메시·질량 — 만들기·검수 스크립트와 Isaac Sim용 `beebot.urdf`는 [urdf/README.md](urdf/README.md). 컨테이너에는 `/ws/urdf`로 마운트된다.

```bash
xacro /ws/urdf/beebot_mesh/urdf.xacro mock_hardware:=true > /tmp/beebot.urdf && check_urdf /tmp/beebot.urdf   # 트리 확인
```

- 구동 관절 6개: `joint1~4` (OM-X) · `joint5` (손목 roll, 추가) · `gripper_left_joint` (오른쪽 손가락은 mimic) + 바퀴 2개
- `ros_system`이 빌드할 때 `share/ros_system/urdf/`로 설치하고, launch가 xacro로 펼쳐 `robot_state_publisher`·`controller_manager`에 넘긴다.
- Isaac Sim USD: `xacro`로 펼친 URDF를 Isaac Sim **URDF Importer**로 변환.
- 메시: OM-X(RM-X52-TNM)는 `ros-lyrical-open-manipulator-description`(apt)의 STL을 쓸 수 있다 (TODO).
- 웹 뷰어 (설치 없이 브라우저로 확인):
  - STL 메시: <https://imagetostl.com/kr/view-stl-online> — `urdf/beebot_mesh/meshes` 드래그
  - URDF: <https://viewer.robotsfan.com/> — 위 `xacro`로 펼친 `/tmp/beebot.urdf` + `meshes/` 폴더를 함께 올림 (xacro 원본은 못 읽음)

## 9. SIL 시뮬레이터 (`simulator/`)

실행·화면·파일 규격은 [simulator/README.md](simulator/README.md)를 본다. 요약만 두면:

- 호스트에서 `cd lv2_module6/simulator && .venv/bin/python main.py` (컨테이너 셸에서는 `test-faker 0`으로 띄워 두기 — 컨테이너 접속은 §1 참조)
- **시뮬레이터는 사람이 호스트에서** 켠다 (컨테이너·테스트·CI는 띄우지 않음 — 컨테이너는 `debug/` 파일만 받음). `test-faker`·`test-all`은 시뮬레이터 없이도 배관(joint_states · odom · Nav2 · monitor)을 검사하고, **시뮬레이터가 켜져 있으면** 카메라 → 인지 → 작업 성공 흐름까지 본다 (창 없이 켜려면 `.venv/bin/python main.py --headless --pucks 6`).
- 컨테이너와는 `lv2_module6/debug/` 파일로만 주고받는다 (호스트에 ROS 불필요)
- **지도 만들기 (스캔)**: 컨테이너 `test-faker 0 map:=slam` → 시뮬레이터 [수동 조작] 탭으로 미로를 몰고 다님 (RTAB-Map이 pole_cam 컬러+깊이로 작성. 지도 한 벌 = 폴더 하나 `ros2_ws/map/<날짜_시각>/` — rtabmap.db · map.pgm · map.yaml, 새 스캔은 새 폴더라 덮어쓰지 않음)
  → '지도 (SLAM)' 칸 [지도 저장] (같은 폴더에 `map.pgm`·`map.yaml` — 미리보기용, 폴더 이름은 `/state/map_name`) → 다음부터 `test-faker 0 map:=saved` (default 폴더, 없으면 가장 최근) 또는 `map:=saved map_name:=<폴더>` (rtabmap.db로 위치추정). 실행 중에도 시뮬레이터 지도 목록에서 폴더를 고르면 그 지도로 갈아끼움 (`/cmd/map_name`).
  [스캔 시작]·[위치추정으로]는 `/cmd/map_mode` → state_manager가 RTAB-Map 모드를 재시작 없이 바꿈

### 지도 모드 정리

| 시작 인자 | 뜨는 지도 노드 | `/state/map_mode` | 지도 DB |
| --- | --- | --- | --- |
| `map:=course` (테스트베드 기본) | map_server(코스 지도 파일) + 고정 `map→odom`(항등) — **RTAB-Map 없음** | COURSE | 없음 |
| `map:=slam` | RTAB-Map + map_saver | MAPPING | 새 폴더 `map/<날짜_시각>/rtabmap.db` (빈 DB) |
| `map:=saved [map_name:=<폴더>]` | RTAB-Map + map_saver (slam과 **같은 노드**) | LOCALIZATION | 저장 폴더의 DB (기본 `default` 폴더, 없으면 가장 최근) |

- **재시작 없이 바뀌는 것** (같은 RTAB-Map 안): 스캔 ↔ 위치추정(`/cmd/map_mode` → `/rtabmap/rtabmap/set_mode_*`) · 지도 갈아끼우기(`/cmd/map_name` → `/rtabmap/rtabmap/load_database` + 위치추정).
- **재시작이 필요한 것**: 코스 ↔ RTAB-Map — 노드 구성이 다르고(`/map`·`map→odom`을 map_server와 RTAB-Map이 같이 내면 안 됨) launch는 시작 때 노드를 정한다.
  시뮬레이터에서 코스 모드일 때 [스캔 시작]·[위치추정으로]·지도 고르기를 누르면 확인 창 → `/cmd/restart`(`map:=slam` / `map:=saved [map_name:=…]`).
- 상태: `/state/map_mode` · `/state/map_name`(지금 지도 폴더) · `/state/map_list`(가진 지도 폴더) — 전부 state_manager.
- **카메라 두 대로 지도** (RTAB-Map — `ros_system.launch.py` `rtabmap_nodes()`): 지도를 만들 때(MAPPING · 자동 탐사) coordinator가 팔을
  **스캔 자세**(`ros_system/config/scan_pose.yaml` — 팔을 낮춰 왼쪽 뒤로, gripper_cam이 왼쪽 뒤를 20° 내려다봄 · pole_cam 앞 시야를 안 가림)로
  보내고, `scan_cam_gate`가 팔이 그 자세에 멈춰 있을 때만 gripper_cam depth를 넘김(아니면 0 — 지도를 안 망치고 동기화도 안 끊김).
  pole_cam · gripper_cam → `rgbd_sync` ×2 → `rgbdx_sync` → rtabmap(`rgbd_images`). 점구름도 둘 다: 몸 뺀 pole_cam +
  gripper_cam(게이트 — 스캔 자세일 때만, 카메라 0.5m 안 = 손가락·폴대·적재함은 뺌) → `point_cloud_aggregator` → `/rtabmap/scan_cloud` (격자 `/map` · ICP).
  **MAPPING 동안 팔 수동 조작 금지**: coordinator가 수동 모드여도 스캔 자세로 보냄(스캔은 보통 수동 주행) · `/cmd/manual/arm` 팔 관절(manual_control)과
  시뮬레이터 MoveIt 팔 목표(service_action_input `/move_action`)는 거절 — 그리퍼만 됨. [위치추정으로] 뒤에 조작.
  회전 시험(새 지도 · 제자리 3바퀴 · 전진 · 반대 3바퀴, 실측 2026-10-19): 지도 위 로봇 방향 오차 카메라 하나 −2.4°/−1.1°/−2.1° → 두 대 +0.6°/−0.4°/−0.1°.
  남은 것: 지금 설정이 엄격해 클로저를 거의 받지 않음 — 지도 ≈ EKF(odom). 짧은 매핑엔 좋으나 오래 돌면 자이로 오차가 쌓여도 안 고침.
- **재시작** (`/cmd/restart` → state_manager, 실기·테스트베드 같음 — 결과 `/state/restart`):
  `ros2 topic pub --once /cmd/restart std_msgs/msg/String "{data: 'all'}"` (이 launch 전체, 같은 인자) · `"{data: 'all map:=slam'}"` (그 인자만 바꿔) ·
  `"{data: 'node perception'}"` (그 노드만, 같은 파라미터). 분리된 도우미가 끄고 띄움 — launch에 SIGINT → 3초 안에 안 꺼지면 SIGTERM + 노드에 직접 SIGINT
  (셸이 `&`로 띄운 launch는 SIGINT를 무시하고, SIGTERM을 받은 launch는 노드를 고아로 남김 — 실측) → `restart_wait_s` 넘으면 SIGKILL → 다시.
  `node`로 다시 띄운 노드는 launch 자식이 아니라 `restart_registry` 파일에 적어 두고 다음 `all`·`node` 때 같이 다룸.

## 10. OpenCR 펌웨어 (`firmware/`)

OpenCR 하나가 바퀴 2 · 팔 5축 · 그리퍼 · IMU를 맡고, 호스트의 `control/BeebotSystemHardware`와 USB 시리얼 ASCII 줄 프로토콜로 짝을 이룬다 — 형식·모터 ID·업로드는 [firmware/README.md](firmware/README.md).

- 프로토콜을 바꾸면 **펌웨어(`beebot_opencr.ino`)와 호스트(`beebot_system_hardware.cpp`)를 같이** 바꾼다. `test-hardware`가 호스트 쪽을, `test-firmware`가 컴파일을 확인한다.
- 컨테이너엔 OpenCR 보드가 없어서 업로드는 가상 시리얼로 보낸 바이트가 .bin과 같은지만 본다. 실물 업로드는 `firmware/upload.sh` 또는 배포 패키지의 `firmware-upload.sh`.

## 11. CI · 배포 패키지

`.github/workflows/ci.yml` (module5와 같은 구성):

| 단계 | 내용 |
| --- | --- |
| 트리거 | `develop`·`main` push, 그 둘로 가는 PR |
| testbed (x86_64 · aarch64) | 아키텍처별 러너(`ubuntu-26.04`, `ubuntu-26.04-arm`)에서 이미지 빌드 → 컨테이너에서 `test-all` |
| 배포 패키지 | `ros2_ws-deploy-<arch>.tar.gz` = `install/`(그 아키텍처용) + `firmware/`(스케치 · `build/*.bin`) + `ros2_ws/*.sh` + `ARCH` → Actions artifact (7일) |
| Release | `main`에 병합(push)될 때만 `run-<RUN ID>` Release에 두 패키지 첨부 (로그인 없이 wget) |
| 제출 태그 | `main` 병합 + 검사 통과 커밋에 `lv2-module6-submit` (마지막 병합 = 최종 제출) |
| 실패 시 | launch 로그 · 토픽 기록 · input/hardware/serial·output/hardware/serial · 테스트 결과를 artifact `debug-<arch>`로 |

실기(라즈베리파이·PC)에서 배포 패키지 쓰기:

```bash
./download-artifact.sh                    # 받을 수 있는 RUN ID 목록
./download-artifact.sh <RUN ID>           # 이 장치 아키텍처용을 ./ros2_ws-<RUN ID>/ 에 풂 (토큰 없으면 Release에서)
cd ros2_ws-<RUN ID>
./firmware-upload.sh --setup && ./firmware-upload.sh     # OpenCR 펌웨어 (최초 1회 --setup)
./start.sh                                               # 장치 확인 → bringup (mock_hardware:=true 면 OpenCR 없이)
./stop.sh                                                # 떠 있는 ROS 전부 끄기 (launch · 노드 · ros2 CLI · daemon)
```

## 12. 실기

```bash
./ros2_ws/start.sh                          # 저장소에서 (install/ 없으면 빌드) — OpenCR · RealSense 확인 후 bringup
./ros2_ws/start.sh learning_enabled:=false  # 학습 노드 끄고 비교 기준
./ros2_ws/start.sh mock_hardware:=true use_cameras:=false   # 장치 없이
./ros2_ws/stop.sh                           # 떠 있는 ROS 전부 끄기
```

- 실기 PC에 필요한 apt: 위 §3 목록 + `ros-lyrical-{realsense2-camera,controller-manager,joint-state-broadcaster,imu-sensor-broadcaster,diff-drive-controller,joint-trajectory-controller,parallel-gripper-controller,robot-state-publisher,xacro,moveit-ros-move-group,moveit-kinematics,moveit-planners-ompl,moveit-simple-controller-manager}` + pick_ik 소스 빌드(§3) (MoveIt 설정은 `ros_system/config/moveit/` — SA 재생성 시 이 폴더에 덮음)
- OpenCR 포트 기본 `/dev/opencr` (`port:=` 로 변경) — ACM 번호는 꽂는 순서로 바뀌니 udev 고정 이름을 쓴다. 최초 1회:
  `sudo cp ros2_ws/99-opencr.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger` (OpenCR VID:PID 0483:5740 → `/dev/opencr`). 업로드 중에는 ROS를 끈다 (같은 포트).

### 실기에서 지도 (스캔 · 위치추정) — 시뮬레이터로 조작

시뮬레이터는 ROS 정보를 전부 토픽으로 주고받으므로(`debug/` 파일 ↔ monitor_manager · input_manager · service_action_input) 실기에도 같은 화면·버튼을 쓴다:

1. 로봇(SBC)과 PC를 **같은 네트워크 · 같은 `ROS_DOMAIN_ID=8`** 에 둔다. 로봇은 `ROS_AUTOMATIC_DISCOVERY_RANGE`를 LOCALHOST로 두지 않는다 (기본 SUBNET).
2. PC에서 파일 연동 노드만 띄운다: `monitor_manager` · `input_manager` · `service_action_input` · `service_action_monitor` (`debug_dir` = 시뮬레이터의 `lv2_module6/debug`).
   카메라 원본(무압축 영상)은 네트워크가 무거우니 `monitor_manager raw_images:=false`.
3. 로봇은 `./start.sh` — 버튼은 테스트베드와 같다: [스캔 시작]·[위치추정으로](`/cmd/map_mode`) · 지도 목록(`/cmd/map_name`) · [지도 저장] · 오른쪽 클릭 [여기로 이동](Nav2) · [여기가 로봇 위치](`/initialpose` → RTAB-Map — launch가 `/rtabmap/initialpose`를 `/initialpose`로 리매핑).

지도 처리는 **bringup**에 있다 (실기·테스트베드 공통 — faker는 bringup을 그대로 포함하고 대역 인자만 덮어씀):
```bash
./ros2_ws/start.sh map:=slam                         # 매핑: 새 폴더 ros2_ws/map/<날짜_시각>/ (start.sh가 map_dir:=ros2_ws/map 기본)
./ros2_ws/start.sh                                   # = map:=saved — 기본 지도(default 폴더, 없으면 가장 최근)로 위치추정
./ros2_ws/start.sh map:=saved map_name:=20261017_163731
```
bringup 기본은 실기 값: `use_cameras:=true` · `mock_hardware:=false` · `use_moveit:=true` (예전 ros_system 기본 false라 실기에서 move_group이 안 떴음).

**아직 안 되는 것 (TODO)**:
- 시뮬레이터 [지도 저장]이 저장 경로를 `/ws/map/<폴더>/map`(컨테이너 경로)으로 직접 만듦 → state_manager가 `map_dir`로 정하도록 (`/cmd/map_save`) 바꿔야 실기 경로와 맞음.

## 13. 문제 해결

| 증상 | 원인 · 해결 |
| --- | --- |
| 컨테이너 load가 수십 · spawner가 `/controller_manager/load_controller`를 못 찾아 컨트롤러 0개 · 아무 일 없는 노드도 CPU 10~20% | Fast DDS 3 TypeLookup 폭주 (루프백 UDP 초당 8만 패킷). `FASTDDS_DEFAULT_PROFILES_FILE=…/ros_system/config/fastdds.xml`(type_propagation 끔)이 설정돼 있는지 — docker-compose · CI · start.sh가 지정. 확인: `awk '/^Udp:/{getline;print $2}' /proc/net/snmp`를 5초 간격으로 두 번 → 차이/5가 수천 이하여야 정상 |
| 토픽이 두 번씩 오거나 이상한 값 | 이전 실행의 노드·launch가 남음 → `pkill -f '(/ws/install\|/opt/ros/lyrical)/[^ ]*lib/\|ros2 launch'` (test-* 시작 때 자동으로 같은 정리) 또는 `docker compose restart` |
| `Overrun detected!` 경고 (controller_manager) | 컨테이너는 실시간 스케줄링이 안 돼서 나는 경고 — 테스트에는 문제없음 |
| spawner `Could not contact service` | 기동 직후 CPU가 몰림. `test-faker`는 spawner를 2회까지 자동 재시도, 그래도 안 되면 다른 무거운 일(빌드 등)을 끝내고 다시 |
| `malformed launch argument` | `인자:=""`처럼 빈 값은 안 됨 — 인자를 빼거나 공백이 아닌 값 |
| 메시지 타입을 못 찾음 (`plan/msg/...`) | `source /ws/install/setup.bash` 안 함, 또는 메시지 패키지 빌드 전 |
| 호스트에서 `lv2_module6/debug` 파일을 못 만들거나 못 지움 (`PermissionError`) | 컨테이너(root)가 만든 디렉토리 — §1 "공유 폴더 권한"의 `chown` 명령으로 회수 |
| `docker compose build`에서 `not enough free space` | 다시 빌드할 때마다 이전 이미지가 이름 없이(dangling) 쌓인다 → `docker image prune -f && docker builder prune -f` |
| test-hardware `OpenCR 포트를 열 수 없음` | 가상 시리얼 브릿지가 안 떴음 → `docker compose restart` (entrypoint가 다시 만든다) |
| 시뮬레이터 카메라가 `waiting` | 컨테이너에서 `test-faker 0`이 떠 있어야 로봇 자세(`/odom`·`/robot_description`)가 온다 |
| 시뮬레이터 시리얼이 `대기` | 컨테이너가 `mock_hardware:=true`(mock)로 떠 있으면 C 명령이 안 온다 — 기본(`test-faker 0`)으로 다시 띄울 것 |
| 진단에 `controller_manager … bad periodicity · jitter` ERROR | 도커는 실시간 스케줄링이 안 돼서 — 실기에선 사라져야 정상 |
| 진단에 `ekf_filter_node … No events recorded` ERROR | robot_localization 자체 진단 — 도커에서 EKF 갱신 주기를 못 맞출 때. `/odometry/filtered`는 나오고 있으면 테스트엔 문제없음 |
| 수동 조작이 안 먹음 | `/state/operation_mode`가 MANUAL인지(`[수동 모드]`), `/cmd/e_stop`이 true로 남아 있지 않은지 |
