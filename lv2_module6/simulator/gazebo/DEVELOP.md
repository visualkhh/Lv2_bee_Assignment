# DEVELOP — 처음 들어온 개발자용

lv2_module6 워크스페이스에서 개발·빌드·테스트하는 방법입니다. 프로젝트 개요·패키지·담당은 [README.md](../../README.md), 설계는 [documents/project/ros](../../../documents/project/ros/README.md), 브랜치·커밋·PR 규칙은 [CONTRIBUTING.md](../../../CONTRIBUTING.md)를 봅니다.

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

컨테이너는 워크스페이스 전체(`../../ros2_ws` → `/ws`)를 마운트합니다. 빌드 결과·로그도 호스트에서 바로 봅니다.
같은 LAN의 다른 팀 스택과 DDS 토픽이 섞이지 않게 `ROS_DOMAIN_ID=88` (docker-compose.yml · `~/.bashrc`) — 컨테이너 안 명령은 compose가 자동 적용, 호스트는 셸 설정.
DDS 환경 (docker-compose · CI 공통): `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` (ROS 기본값 — LOCALHOST면 Fast DDS가 localhost 참가자 번호 몇 개에만 유니캐스트로 discovery를 보내 노드 40여 개 중 일부가 서로 안 보였음: Nav2 관리자 멈춤 · `/map` 발행자 0 · CLI가 노드 못 찾음. 다른 팀과는 `ROS_DOMAIN_ID=88`로 분리) · `FASTDDS_DEFAULT_PROFILES_FILE=…/fastdds.xml` (Fast DDS TypeLookup 끔 — 없으면 노드 40개에서 UDP 폭주로 컨트롤러가 못 뜸).

| 호스트 | 컨테이너 | 용도 |
| --- | --- | --- |
| `../../ros2_ws` | `/ws` | ROS 2 워크스페이스 전체 (src + urdf + build/install/log — 호스트에서 편집·열람) |
| `../../firmware` | `/firmware` | OpenCR 펌웨어 스케치 |
| `../../debug` | `/debug` | 토픽 기록 · 시뮬레이터 연동 · 가상 시리얼(input/hardware/serial·output/hardware/serial) (gitignore). **양방향**: 호스트→컨테이너(`input/topic/` 명령·수동, `input/hardware/camera/` 카메라 장면, `service/*/request`·`action/*/goal·cancel`)와 컨테이너→호스트(`output/topic/`·`service/*/response·type|info`·`action/*/accept·feedback·result`, `output/hardware/serial`)가 같은 폴더를 쓴다. 펌웨어 빌드 결과는 `firmware/build/` (gitignore). 소유권·권한은 아래 "공유 폴더 권한" 참조 |

Dockerfile·`docker/*.sh`를 고쳤으면 `docker compose build` 후 `docker compose up -d`로 다시 띄웁니다 (스크립트는 이미지에 복사돼 있음).

### 공유 폴더 권한 (`../../debug`)

이미지에 `USER` 지정이 없어 컨테이너 안 프로세스는 uid 0(root)으로 돕니다. 바인드 마운트는 호스트 파일시스템을 그대로 공유하고 user namespace remapping을 쓰지 않으므로(기본값), 컨테이너가 만든 파일·디렉토리는 호스트에서 root 소유로 보입니다. `sudo`라서가 아니라 uid 문제입니다.

그래서 양쪽 다 쓸 수 있게 디렉토리는 777로 유지합니다:

- `../../../docker/entrypoint.sh`: 기동 시 `input/`·`input/hardware/camera/`·`output/topic/`·`service/`·`action/`·`interface/`·`output/` 미리 생성 + `chmod -R 777 /debug`
- `faker` 노드(`monitor_manager`·`service_action_monitor`·`service_action_input`): 디렉토리 생성 직후 `chmod 0777` — 실행 중 새로 생기는 `output/topic/<토픽>/`·`service/<이름>/`·`action/<이름>/`도 호스트가 rename(원자적 쓰기)으로 교체 가능
- 파일 자체는 644여도 된다: 읽는 쪽은 읽기만, 쓰는 쪽은 디렉토리 권한으로 rename 하므로

`PermissionError`가 나면 컨테이너에서 소유권 회수 — 호스트 `sudo`는 비밀번호가 필요할 수 있으니 컨테이너(root) 경유가 확실:

```bash
cd docker && docker compose exec -u root lyrical bash -c "chown -R 1000:1000 /debug && chmod -R 777 /debug"
```

참고: `build/`·`install/`·`../../../log`는 호스트에 그대로 남으므로 재생성 뒤에도 `colcon build` 없이 바로 쓸 수 있습니다. 빌드가 꼬였다 싶으면 `install`·`build` 지우고 처음부터 다시 합니다(지울 때 소유권 주의 — 위 chown 참고).

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

ROS 2 Lyrical이 깔린 리눅스면 `docker/*.sh`를 그대로 씁니다. 경로는 자동 감지라 지정할 것 없음 (`/ws` 있으면 컨테이너, 없으면 레포 안 `ros2_ws`; `debug`도 `/debug`→없으면 `../../debug`):

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=88  # 같은 LAN 다른 팀과 분리 (컨테이너 compose·~/.bashrc와 같은 값)
./lv2_module6/ros2_ws/rebuild.sh              # 최초 1회 풀빌드 (build/·install/·log 재생성)
docker/test-faker.sh 0                        # 띄워 두기 — 이후 셸에선 source lv2_module6/ros2_ws/install/setup.bash
```

자주 쓰는 건 `../../operation` 아래 **주제별 디렉토리**에 묶었습니다 (어디서 실행해도 됨):

```text
develop/
├── bringup/faker.sh        # = docker/test-faker.sh 0 (use_gz_cameras:=true)
├── gazebo/                 # start·kill·restart·reset + 월드/태그/신호등/동적 장애물
│   ├── generate_world.py / generate_tags.py
│   ├── traffic_lights.py / dynamic_obstacles.py
│   ├── gui_plugin/          # Gazebo 수동 액터 버튼 (사람/박스/삭제)
│   └── bridge.yaml / relay.py / publish_poses.py
├── moveit/                 # start·kill·restart·setup + moveit.rviz
├── rviz/                   # start·kill·restart + default.rviz
├── teleop/                 # start·kill·restart
├── arm/arm.sh              # rqt 팔 슬라이더
└── DEVELOP.md
```

```bash
lv2_module6/develop/bringup/faker.sh                  # = docker/test-faker.sh 0 (인자 그대로 전달)
lv2_module6/develop/rviz/start.sh                     # RViz2 + 기본 구성(URDF·TF·Map, rviz/default.rviz)
lv2_module6/develop/rviz/kill.sh                      # RViz 종료
lv2_module6/develop/rviz/restart.sh                   # kill → start
lv2_module6/develop/moveit/start.sh                   # MoveIt RViz (매니플레이터 Plan → Execute — move_group은 faker가 띄운 것)
lv2_module6/develop/moveit/kill.sh                    # MoveIt RViz만 종료 (기본 RViz는 유지)
lv2_module6/develop/moveit/restart.sh                 # kill → start
lv2_module6/develop/moveit/setup.sh                   # 최초 1회: MoveIt RViz 플러그인 + pick_ik 설치 (sudo)
lv2_module6/develop/gazebo/start.sh                   # Gazebo 클린 스타트 (분산 미션+장애물+도로형 신호등+동적 장애물+스폰+브릿지, Ctrl-C로 종료)
lv2_module6/develop/gazebo/reset.sh [x y z yaw]       # 로봇 원위치 (기본 -1.835 2.757 0.05 0) — 끼임·전복 시
lv2_module6/develop/gazebo/kill.sh                    # Gazebo 전체 종료 (서버·GUI·브릿지·릴레이·씬 마커·신호등·동적 장애물)
lv2_module6/develop/gazebo/restart.sh                 # kill → start (장애물·퍽도 새로 무작위 배치)
lv2_module6/develop/teleop/start.sh [twist토픽]       # 키보드 조종 (기본 /gz/cmd_vel — testbed 안 건드림)
lv2_module6/develop/teleop/kill.sh                    # 텔레옵 종료
lv2_module6/develop/teleop/restart.sh                 # kill → start
lv2_module6/develop/arm/arm.sh [컨트롤러]              # rqt 팔 슬라이더 (기본 arm_controller)
```

- **Gazebo 월드**: `gazebo/start.sh`·`gazebo`가 시작할 때마다 `gazebo`로 ① 30×30×60 mm 중성색 장애물 박스 6개 + 지름 30 mm × 높이 60 mm 실린더 6개를 미로 빈 공간에 무작위 배치하고, ② 퍽 9개를 **네 미션에 3/2/2/2로 나눠 지도 위·중간·아래에 분산**합니다. 각 미션은 자기 구역 근처 벽에 미션 태그를 하나씩 가집니다. 장애물은 미션 앞쪽·태그·신호등 갠트리을 비웁니다.
- **퍽과 태그**: 퍽은 30×30×60 mm 상자이고 4옆면 + 위/아래 = 6면에 AprilTag가 붙습니다. 각 면의 태그 앞면은 바깥을 향하고, 같은 색 퍽은 6면 모두 같은 ID입니다 — **red=1, orange=2, green=3**. 미션 태그는 **SORT_COLOR=10, MOVE_TO_B=11, STAND_UP=12, LAY_DOWN=13**이고, 네 태그가 서로 다른 미션 구역 벽에 분산됩니다 (활성 미션은 RViz에서 초록색·`*ACTIVE*`). B 구역 표지는 **20**입니다. 태그 PNG/OBJ/MTL은 `gazebo/assets/apriltags`에 생성됩니다 (`gazebo`; 필요하면 `--force`로 재생성).
- **미션 분산**: 기본은 `GZ_MISSION_SITES=all`이고, 네 미션의 실제 퍽/구역이 지도에 함께 있습니다. `GZ_MISSION`은 그중 **활성 미션 표시**를 정합니다(기본 random). 위치는 SORT `(0.35,2.75)` 상단, MOVE `(-1.45,1.97)` 중간 위, STAND `(1.45,1.18)` 중간 아래, LAY `(0.0,-2.75)` 하단입니다. `GZ_MISSION_SITES=single`이면 활성 미션만 `GZ_MISSION_CENTER` 한 곳에 만듭니다. SORT는 색별 구역 3개, MOVE는 B 구역 1개를 바닥에 만듭니다. STAND는 분배된 퍽이 눕고, LAY는 모두 서서 시작합니다.
- **도로형 신호등**: 상단 방의 동서 통행로 위에 **갠트리 (상부 빔 + 양쪽 기둥)**를 세우고, 차량 신호등처럼 하우징을 매달았습니다. 렌즈는 로봇이 지나오는 서쪽을 향해 도로를 따라 보입니다. `gazebo/traffic_lights.py`가 `GZ_TRAFFIC_INTERVAL`(기본 3초)마다 **빨강→초록→주황**으로 바꾸고, 색별 발광 구는 미리 만든 뒤 켜진 색만 렌즈 앞으로 옮깁니다. 두 신호등은 위상이 엇갈립니다. `/gz_traffic/markers`, `/gz_traffic/info` 발행.
- **동적 장애물**: `gazebo/dynamic_obstacles.py`는 로봇이 **실제로 이동 중일 때만** 빈 후보 지점에서 **퍽과 구분되는 사람 형상**을 소환해 천천히 움직입니다. `/gz/cmd_vel` 속도 명령이 살아 있을 때만 이동 중으로 보고, 정지하거나 미션 수행 자세로 멈추면 active를 모두 `remove`합니다. `GZ_DYNAMIC_LIFETIME`(기본 12초) 뒤에도 제거하며, 미션 앞쪽/신호등 갠트리 주변은 후보에서 제외하므로 맵에 누적되지 않습니다. `/gz_dynamic/markers`, `/gz_dynamic/info` 발행. `GZ_DYNAMIC_COUNT=0`이면 꺼집니다.
- **수동 액터 GUI**: `gazebo/gui_plugin`이 Gazebo GUI에 **사람/박스/퍽/신호등 소환**과 **수동 모델 삭제** 버튼을 붙입니다. 같은 패널의 **환경** 탭에서 바닥·벽·파노라마를 골라 현재 월드 모델을 즉시 교체할 수 있고, 최초 기본값은 `none`입니다. `start.sh`가 플러그인을 자동 빌드하고 `gui.config`를 `gz sim --gui-config`로 넘깁니다. 소환 버튼은 `/tmp/beebot_scene.json`의 빈 후보 지점에 바로 생성하고, 임의 위치에 직접 놓고 싶으면 Gazebo 기본 **Spawn/드래그 앤 드롭**을 쓰면 됩니다. 수동 모델은 `dynamic_obstacles.py`의 자동 수명 관리 대상이 아니므로 삭제 버튼으로 정리합니다. QML만 수정했으면 `gazebo`를 다시 실행하세요.
- **카메라/점구름 성능**: Gazebo RGBD는 RealSense D435 운용 모드인 **640×360 @ 15 Hz**로 낮췄습니다. 추가로 `perception/body_filter`의 `stride`가 원본 점구름을 솎아(stride점당 1점) Nav2·collision_monitor로 보냅니다. 필요하면 `ros2_ws/src/perception/config/perception.yaml`의 `body_filter.stride`를 조정하고, 카메라 해상도/레이트는 `urdf/default/urdf.xacro`의 RGBD 센서에서 바꾼 뒤 `colcon build`로 `ros_system`을 다시 설치하세요.
- **미션/물체 환경변수**: `GZ_MISSION`, `GZ_MISSION_SITES=all|single`, `GZ_PUCK_COUNT`(총량, 기본 9), `GZ_PUCK_MM`, `GZ_TAG_SIZE_MM`, `GZ_MISSION_TAG_MM`, `GZ_MISSION_CENTER="0.35 2.75"`(single 전용), `GZ_MISSION_WALL=north|south|east|west`, `GZ_MISSION_SEED`, `GZ_OBSTACLE_BOXES`, `GZ_OBSTACLE_CYLINDERS`, `GZ_OBSTACLE_BOX_MM="300 300 600"`(cm로 키울 때), `GZ_OBSTACLE_CYLINDER_MM`, `GZ_OBSTACLE_STATIC=1`, `GZ_OBSTACLE_SEED`, `GZ_TRAFFIC_INTERVAL=3.0`, `GZ_DYNAMIC_COUNT=3`, `GZ_DYNAMIC_LIFETIME=12`, `GZ_DYNAMIC_RESPAWN=1`, `GZ_DYNAMIC_SPEED=0.16`, `GZ_DYNAMIC_MM="220 220 600"`.
- **진짜 위치**: `gazebo/publish_poses.py`가 `/world/default/pose/info`의 실제 모델 pose를 읽어 `/gz_obstacles/markers`(MarkerArray)와 `/gz_obstacles/poses`(PoseArray)로 발행합니다 (frame `world`). 퍽·장애물·벽 태그·목표 구역·신호등 하우징이 들어 있고, 동적 퍽이 밀리면 마커도 실제 pose를 따라갑니다. `base_footprint→world` TF도 같이 내므로 `rviz`의 `Gazebo Obstacles` display에서 기본 fixed frame으로 보입니다. 동적 장애물/신호등은 각각 `Dynamic Obstacles`·`Traffic Lights` display로 분리됩니다. 월드 좌표를 고정해서 보려면 RViz Fixed Frame을 `world`로 두십시오. `rviz`로 새 display를 읽습니다.
- **미션 정보**: `/gz_mission/info` (`std_msgs/String`, JSON, transient_local)에 활성 미션 이름·설명·태그 ID, **분산된 stations 목록**(위치/퍽/태그/구역), 목표 구역, 태그 매핑이 들어 있습니다. 예: `ros2 topic echo --once /gz_mission/info`.
- 로봇 스폰 높이는 0.05 m입니다. 기존 0.3 m는 낙하하며 전복될 수 있어 낮췄습니다 (`gazebo/reset.sh` 기본값도 동일).
- 컨테이너와 같은 셸 스크립트라 같은 검사 (`test-packages`, `test-bringup`, `test-faker`)가 돕니다. `sh script`로 실행해도 bash로 재실행됩니다.
- 빌드 산출물 소유자가 본인이라 `chown` 문제 없음. 단 `build/`·`install/`·`../../../log`는 컨테이너와 이름이 같고 CMakeCache에 절대경로가 박히므로 **컨테이너와 호스트에서 동시 빌드 금지** — 한 번에 한 곳에서만.
- 호스트 apt 목록은 §3 참고 (`ros-lyrical-rtabmap-msgs` 포함). rosdep는 lyrical을 몰라서 그냥 apt.
- 가상 시리얼(`/dev/opencr`)이 필요한 검사(`test-serial`, `test-hardware`, 펌웨어 업로드)는 컨테이너(entrypoint의 socat 브릿지)에서만 됩니다. 호스트는 mock 모드(`mock_hardware:=true`, 기본값) 위주로 씁니다. push 전 전체 게이트(`test-all`)·CI 정본은 그대로 컨테이너입니다.

### MoveIt으로 팔 계획·실행

`develop/bringup/faker.sh`가 이미 진짜 `move_group`과 `arm_controller`(JTC)를 띄웁니다. `develop/moveit/start.sh`는 그 `move_group`에 붙는 RViz MotionPlanning 화면만 엽니다 (`develop/moveit/moveit.rviz`, Planning Group `arm`).

```bash
lv2_module6/develop/gazebo/start.sh   # 터미널 1: 월드 + 브릿지 + relay
lv2_module6/develop/bringup/faker.sh  # 터미널 2: move_group + arm_controller (+ Gazebo 카메라)
lv2_module6/develop/moveit/start.sh   # 터미널 3: RViz — Interactive Marker/Goal State → Plan → Execute
```

- 실행 경로: `/arm_controller/follow_joint_trajectory` → mock 관절 상태 → `/joint_states` → `gazebo/relay.py` → `/gz_arm/<joint>/cmd` → Gazebo 관절별 JPC. **Gazebo도 같이 움직입니다** — joint2 0.3 rad MoveIt 계획·실행 검증: RViz TF `(0.290, 0.001, 0.277)` vs Gazebo `(0.290, 0.001, 0.276)`, 오차 ≤1 mm.
- 전제: `gazebo/relay.py`가 살아 있어야 Gazebo 미러링이 됩니다. 없으면 `moveit`가 경고합니다.
- `moveit/start.sh`는 기존 RViz를 건드리지 않고 MoveIt RViz를 추가로 엽니다. 하나만 쓰려면 `rviz` 후 `moveit`; MoveIt RViz만 닫으려면 `moveit`.
- 최초 1회 `moveit/setup.sh` — `ros-lyrical-moveit-ros-visualization`(RViz 플러그인)과 `pick_ik`(5축 팔 IK, 소스 빌드)를 설치합니다. `pick_ik` 없이 KDL로는 포즈 목표 (특히 위→아래 방향)를 못 풉니다. 설치 후 `faker.sh`를 재시작해야 `move_group`이 새 플러그인을 로드합니다.
- RViz `Planned Path` → `Loop Animation: false` (기본) — 계획 궤적 고스트는 애니메이션이 끝나면 사라집니다. `true`면 마지막 자세가 계속 남습니다. `Show Trail`도 `false`입니다. 단, 이 애니메이션은 실제 컨트롤러 실행 완료와 동기화되지 않습니다(계획 궤적 미리보기).
- 5축 팔은 `Approx IK Solutions` + `pick_ik` `rotation_scale: 0.0`(위치 우선)로 좌우 목표를 계획합니다. 자세(orientation)는 보장되지 않으므로 자세가 중요하면 `Goal State`를 `Joint`로 두고 관절값으로 계획하십시오.

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
    ros-lyrical-moveit-simple-controller-manager ros-lyrical-moveit-ros-visualization \
    ros-lyrical-rtabmap-msgs ros-lyrical-rtabmap-ros ros-lyrical-ament-cmake-pytest
  ```
  위 목록 = `../../../docker/Dockerfile`의 apt 목록 (그게 컨테이너 정본 — Dockerfile에 추가하면 여기도 같이 고친다).
  pick_ik (MoveIt IK)는 Lyrical apt에 없어 소스 빌드 (Dockerfile과 같은 커밋 `7c97fe2`). 아래 스크립트가 apt 의존성·빌드·`/opt/ros/lyrical` 복사를 한 번에 합니다:
  ```bash
  lv2_module6/develop/moveit/setup.sh
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
  `bringup`에 `node('<패키지>', '<실행 파일>')` → `../../../docker/test-bringup.sh`의 `NODES`에 노드 이름 → `bringup` 묶음에 이름.
- 토픽·메시지·QoS는 설계 문서 표를 따른다. 센서는 `rclcpp::SensorDataQoS()`(best_effort), 명령은 reliable, 가끔만 바뀌는 상태(`/sub_goals`, `/task_status`)는 reliable + transient_local.
- 학습 노드는 `enabled` 파라미터로 끌 수 있어야 하고, 꺼지거나 모델 파일이 없으면 기본 동작(Δ=0 · 규칙)으로 돌아야 한다.
- **도커·CI에는 GPU·모델이 없다** — 테스트베드는 진짜 ml 노드 대신 **faker ML 대역**(ml 노드와 1:1 — `fake_grasp_scorer` · `fake_rl_nav_tuner` · `fake_rl_nav_correction`, 컨트롤러 `faker/FakeRlArmCorrection`)을 같은 이름·토픽으로 띄운다 (`ml:=fake` 기본, `bringup use_ml:=false`).
  진짜로 돌려 보려면 `test-faker 0 ml:=real`. ml 패키지의 gtest(`colcon test`)는 도커에서 돌므로 **CPU만으로 도는 테스트**로 둔다.
  ml 노드의 토픽·서비스·파라미터를 바꾸면 같은 이름의 대역 `faker/src/fake_<노드>.cpp`도 같이 맞춘다. 하나만 진짜로: `grasp_scorer:=real` (rl_nav_tuner · rl_nav_correction · rl_arm_correction도 같음)
- **상태는 `/diagnostics`로** — 노드 클래스에 `diagnostic_updater::Updater diag_{this};` 를 두고 `diag_.add("상태", …)`에서
  `s.summary(OK|WARN|ERROR, "한 줄")` + `s.add("키", 값)`. 1초마다 발행되고 bringup의 `diagnostic_aggregator`가 묶는다
  (`bringup` — 새 노드는 이름을 해당 묶음 `startswith`에 추가). `test-bringup`이 노드마다 상태가 오는지 확인한다.
- **수동 모드를 존중** — 로봇을 움직이는 노드는 `/state/operation_mode`가 "MANUAL"이면 자기 명령을 멈춘다 (coordinator처럼). 바퀴는 twist_mux가 이미 수동을 우선한다.
- 담당자가 채울 곳은 `TODO(이름)` 주석으로 표시돼 있다.

## 6. 파라미터 (config yaml)

노드 파라미터 값은 코드가 아니라 **패키지 `config/<패키지>.yaml`** 에서 관리합니다 (ROS 2 관례: 노드 이름 → `ros__parameters`).

| 파일 | 노드 |
| --- | --- |
| `perception/config/perception.yaml` | perception · body_filter |
| `control/config/control.yaml` | arm_safety_monitor · manual_control |
| `manager/config/manager.yaml` | state_manager (지도 모드 시작 값 — faker가 `map:=`에 맞춰 덮어씀) |
| `ml/config/ml.yaml` | grasp_scorer · rl_nav_tuner · rl_nav_correction |
| `plan/config/plan.yaml` | user_input · vlm_task_planner · coordinator · explorer(자동 탐사) |
| `faker/config/faker.yaml` | pole_front_cam · gripper_cam · move_group · fake_opencr · monitor_manager · input_manager (더미) |
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
| perception | `test/test_body_filter.cpp` | URDF 충돌 상자(+padding) 안의 점은 빠지고 밖의 점만 `points_filtered`로 나간다 |
| control | `test/test_arm_safety_monitor.cpp` | 전류 과다가 0.1초 넘게 이어지면 `/arm_safety_event` |
| control | `test/test_manual_control.cpp` | `/state/operation_mode`가 MANUAL일 때만 `/cmd/manual/cmd_vel` → `/cmd_vel_manual` |
| manager | `test/test_state_manager.cpp` | `/cmd/operation_mode` → `/state/operation_mode` · `/cmd/map_mode` → RTAB-Map 응답 뒤 `/state/map_mode` (서버 없거나 COURSE면 그대로) |
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

`urdf/beebot_mesh/urdf.xacro` — TurtleBot3 Waffle Pi + 앞쪽 상판 위 OpenMANIPULATOR-X(RM-X52) + 손목 roll + 그리퍼(위에 gripper_cam) + 뒤쪽 전원선 가이드 폴(pole_front_cam) + 바구니(적재함) 충돌체. Fusion 모델에서 뽑은 메시·질량 — 만들기·검수 스크립트와 Isaac Sim용 `beebot.urdf`는 [urdf/README.md](urdf/README.md). 컨테이너에는 `/ws/urdf`로 마운트된다.

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

## 9. 지도 모드 (launch 인자)

SIL 시뮬레이터(`simulator/`)는 더 이상 사용하지 않는다. 지도 관련 launch 인자만 남긴다:

| 시작 인자 | 뜨는 지도 노드 | `/state/map_mode` | 지도 DB |
| --- | --- | --- | --- |
| `map:=course` (테스트베드 기본) | map_server(코스 지도 파일) + 고정 `map→odom`(항등) — **RTAB-Map 없음** | COURSE | 없음 |
| `map:=slam` | RTAB-Map + map_saver | MAPPING | 새 폴더 `map/<날짜_시각>/rtabmap.db` (빈 DB) |
| `map:=saved [map_name:=<폴더>]` | RTAB-Map + map_saver (slam과 **같은 노드**) | LOCALIZATION | 저장 폴더의 DB (기본 `default` 폴더, 없으면 가장 최근) |

- **재시작 없이 바뀌는 것** (같은 RTAB-Map 안): 스캔 ↔ 위치추정(`/cmd/map_mode` → `/rtabmap/rtabmap/set_mode_*`) · 지도 갈아끼우기(`/cmd/map_name` → `/rtabmap/rtabmap/load_database` + 위치추정).
- **재시작이 필요한 것**: 코스 ↔ RTAB-Map — 노드 구성이 다르고(`/map`·`map→odom`을 map_server와 RTAB-Map이 같이 내면 안 됨) launch는 시작 때 노드를 정한다 (`/cmd/restart` → `map:=slam` / `map:=saved [map_name:=…]`).
- 상태: `/state/map_mode` · `/state/map_name`(지금 지도 폴더) · `/state/map_list`(가진 지도 폴더) — 전부 state_manager.
- **재시작** (`/cmd/restart` → state_manager, 실기·테스트베드 같음 — 결과 `/state/restart`):
  `ros2 topic pub --once /cmd/restart std_msgs/msg/String "{data: 'all'}"` (이 launch 전체, 같은 인자) · `"{data: 'all map:=slam'}"` (그 인자만 바꿔) ·
  `"{data: 'node perception'}"` (그 노드만, 같은 파라미터). 분리된 도우미가 끄고 띄움 — launch에 SIGINT → 3초 안에 안 꺼지면 SIGTERM + 노드에 직접 SIGINT
  (셸이 `&`로 띄운 launch는 SIGINT를 무시하고, SIGTERM을 받은 launch는 노드를 고아로 남김 — 실측) → `restart_wait_s` 넘으면 SIGKILL → 다시.
  `node`로 다시 띄운 노드는 launch 자식이 아니라 `restart_registry` 파일에 적어 두고 다음 `all`·`node` 때 같이 다룸.

## 10. OpenCR 펌웨어 (`firmware/`)

OpenCR 하나가 바퀴 2 · 팔 5축 · 그리퍼 · IMU를 맡고, 호스트의 `control/BeebotSystemHardware`와 USB 시리얼 ASCII 줄 프로토콜로 짝을 이룬다 — 형식·모터 ID·업로드는 [firmware/README.md](firmware/README.md).

- 프로토콜을 바꾸면 **펌웨어(`beebot_opencr.ino`)와 호스트(`beebot_system_hardware.cpp`)를 같이** 바꾼다. `test-hardware`가 호스트 쪽을, `test-firmware`가 컴파일을 확인한다.
- 컨테이너엔 OpenCR 보드가 없어서 업로드는 가상 시리얼로 보낸 바이트가 .bin과 같은지만 본다. 실물 업로드는 `firmware/setup_compile_upload.sh`(또는 setup·compile·upload 개별 실행) 또는 배포 패키지의 `firmware-upload.sh`.

## 11. CI · 배포 패키지

`../../../.github/workflows/ci.yml` (module5와 같은 구성):

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

- 실기 PC에 필요한 apt: 위 §3 목록 + `ros-lyrical-{realsense2-camera,controller-manager,joint-state-broadcaster,imu-sensor-broadcaster,diff-drive-controller,joint-trajectory-controller,parallel-gripper-controller,robot-state-publisher,xacro,moveit-ros-move-group,moveit-kinematics,moveit-planners-ompl,moveit-simple-controller-manager,moveit-ros-visualization}` + pick_ik 소스 빌드(`develop/moveit/setup.sh`, §3) (MoveIt 설정은 `ros_system/config/moveit/` — SA 재생성 시 이 폴더에 덮음)
- OpenCR 포트 기본 `/dev/opencr` (`port:=` 로 변경) — ACM 번호는 꽂는 순서로 바뀌니 udev 고정 이름을 쓴다. 최초 1회:
  `sudo cp ros2_ws/99-opencr.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger` (OpenCR VID:PID 0483:5740 → `/dev/opencr`). 업로드 중에는 ROS를 끈다 (같은 포트).

### 실기에서 지도 (스캔 · 위치추정) — 시뮬레이터로 조작

시뮬레이터는 ROS 정보를 전부 토픽으로 주고받으므로(`debug/` 파일 ↔ monitor_manager · input_manager · service_action_input) 실기에도 같은 화면·버튼을 쓴다:

1. 로봇(SBC)과 PC를 **같은 네트워크 · 같은 `ROS_DOMAIN_ID=88`** 에 둔다. 로봇은 `ROS_AUTOMATIC_DISCOVERY_RANGE`를 LOCALHOST로 두지 않는다 (기본 SUBNET).
2. PC에서 파일 연동 노드만 띄운다: `monitor_manager` · `input_manager` · `service_action_input` · `service_action_monitor` (`debug_dir` = 시뮬레이터의 `../../debug`).
   카메라 원본 (무압축 영상)은 네트워크가 무거우니 `monitor_manager raw_images:=false`.
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
| 컨테이너 load가 수십 · spawner가 `/controller_manager/load_controller`를 못 찾아 컨트롤러 0개 · 아무 일 없는 노드도 CPU 10~20% | Fast DDS 3 TypeLookup 폭주 (루프백 UDP 초당 8만 패킷). `FASTDDS_DEFAULT_PROFILES_FILE=…/fastdds.xml`(type_propagation 끔)이 설정돼 있는지 — docker-compose · CI · start.sh가 지정. 확인: `awk '/^Udp:/{getline;print $2}' /proc/net/snmp`를 5초 간격으로 두 번 → 차이/5가 수천 이하여야 정상 |
| 토픽이 두 번씩 오거나 이상한 값 | 이전 실행의 노드·launch가 남음 → `pkill -f '(/ws/install\|/opt/ros/lyrical)/[^ ]*lib/\|ros2 launch'` (test-* 시작 때 자동으로 같은 정리) 또는 `docker compose restart` |
| `Overrun detected!` 경고 (controller_manager) | 컨테이너는 실시간 스케줄링이 안 돼서 나는 경고 — 테스트에는 문제없음 |
| spawner `Could not contact service` | 기동 직후 CPU가 몰림. `test-faker`는 spawner를 2회까지 자동 재시도, 그래도 안 되면 다른 무거운 일(빌드 등)을 끝내고 다시 |
| `malformed launch argument` | `인자:=""`처럼 빈 값은 안 됨 — 인자를 빼거나 공백이 아닌 값 |
| 메시지 타입을 못 찾음 (`plan/msg/...`) | `source /ws/install/setup.bash` 안 함, 또는 메시지 패키지 빌드 전 |
| 호스트에서 `../../debug` 파일을 못 만들거나 못 지움 (`PermissionError`) | 컨테이너(root)가 만든 디렉토리 — §1 "공유 폴더 권한"의 `chown` 명령으로 회수 |
| `docker compose build`에서 `not enough free space` | 다시 빌드할 때마다 이전 이미지가 이름 없이(dangling) 쌓인다 → `docker image prune -f && docker builder prune -f` |
| test-hardware `OpenCR 포트를 열 수 없음` | 가상 시리얼 브릿지가 안 떴음 → `docker compose restart` (entrypoint가 다시 만든다) |
| 시뮬레이터 카메라가 `waiting` | 컨테이너에서 `test-faker 0`이 떠 있어야 로봇 자세(`/odom`·`/robot_description`)가 온다 |
| 시뮬레이터 시리얼이 `대기` | 컨테이너가 `mock_hardware:=true`(mock)로 떠 있으면 C 명령이 안 온다 — 기본(`test-faker 0`)으로 다시 띄울 것 |
| 진단에 `controller_manager … bad periodicity · jitter` ERROR | 도커는 실시간 스케줄링이 안 돼서 — 실기에선 사라져야 정상 |
| 진단에 `ekf_filter_node … No events recorded` ERROR | robot_localization 자체 진단 — 도커에서 EKF 갱신 주기를 못 맞출 때. `/odometry/filtered`는 나오고 있으면 테스트엔 문제없음 |
| 수동 조작이 안 먹음 | `/state/operation_mode`가 MANUAL인지(`[수동 모드]`), `/cmd/e_stop`이 true로 남아 있지 않은지 |
