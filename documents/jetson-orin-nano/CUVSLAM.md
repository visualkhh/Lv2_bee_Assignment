# cuVSLAM + nvblox (NVIDIA GPU SLAM) — 설치·실행·시험 기록

SLAM은 **cuVSLAM(위치) + nvblox(지도)** 하나다 (RTAB-Map은 브랜치 `feature/cuvslam-nvblox`에서 뺐다). 둘 다 Jetson GPU를 쓴다.
GPU 없는 테스트베드(faker)는 `map:=course`(map_server + 정지 map→odom)만 되고, `map:=slam|saved`는 시작할 때 이유를 알리고 멈춘다.

- 노드: [`ros_system/scripts/cuvslam_slam.py`](../../lv2_module6/ros2_ws/src/ros_system/scripts/cuvslam_slam.py) (위치) ·
  [`nvblox_map.py`](../../lv2_module6/ros2_ws/src/ros_system/scripts/nvblox_map.py) (nvblox 격자 → /map) ·
  [`scan_cam_gate.py`](../../lv2_module6/ros2_ws/src/ros_system/scripts/scan_cam_gate.py) (nvblox depth에서 자기 몸 빼기 — pole_depth_gate · gripper_depth_gate)
- nvblox 설정: [`ros_system/config/nvblox.yaml`](../../lv2_module6/ros2_ws/src/ros_system/config/nvblox.yaml)
- 구조 검사: [`ros_system/test/test_slam_launch.py`](../../lv2_module6/ros2_ws/src/ros_system/test/test_slam_launch.py) — SLAM 노드·지도 파일 경로가 맞게 펼쳐지는지 (test-all)
- launch 인자: [`ros_system.launch.py`](../../lv2_module6/ros2_ws/src/ros_system/launch/ros_system.launch.py) · [`bringup.launch.py`](../../lv2_module6/ros2_ws/src/bringup/launch/bringup.launch.py)
- EKF 입력: [`ros_system/config/ekf.yaml`](../../lv2_module6/ros2_ws/src/ros_system/config/ekf.yaml) `odom1`

## 1. 구성

| 실행 인자 (bringup) | 하는 일 | 지도 파일 |
|---|---|---|
| `map:=slam` (`map_name:=` 생략 시 `<날짜_시각>`) | 매핑 — 새 지도 폴더, 끌 때 nvblox가 자동 저장 | `<map_dir>/<이름>/nvblox.nvblx` |
| `map:=saved map_name:=<이름>` | 위치추정 — nvblox_map이 시작할 때 `/nvblox_node/load_map`으로 불러옴 | 같음 |
| `use_slam:=false` (ros_system) | SLAM 없음 — map→odom · /map 없음 | — |

`map_dir` 기본 `~/.ros/beebot_maps` (실기 start.sh는 `ros2_ws/map`). 폴더에는 사람이 보는 미리보기 `map.pgm` · `map.yaml`도 같이 둔다 (시뮬레이터 [지도 저장]).

위쪽(Nav2 · explorer · state_manager · mission_manager · 시뮬레이터)이 받는 계약:

| 계약 | 내는 쪽 |
|---|---|
| `map→odom` TF | cuvslam_slam (루프 클로저 · 평면 제약) |
| `/map` (OccupancyGrid, latched) | nvblox `/nvblox_node/static_occupancy_grid` → **nvblox_map**이 latched로 다시 냄 (1 Hz) |
| 지도 원점 | 출발 자리 base_footprint |
| `/odometry/filtered` (EKF) | 바퀴 + IMU + 시각 (`/cuvslam/odom`, ekf.yaml `odom1`) |

```text
pole_front_cam 컬러 + 정렬 depth ─▶ cuvslam_slam ─▶ map→odom · /cuvslam/odom ─▶ EKF(odom→base_footprint)
pole_front_cam 정렬 depth ─▶ pole_depth_gate (0.5 m 안 = 자기 팔 → 0) ─┐
gripper_cam 정렬 depth ─▶ gripper_depth_gate (스캔 자세일 때만 · 0.5 m 안 → 0) ─┴▶ nvblox_node ─▶ ~/static_occupancy_grid
                                                                                       └─▶ nvblox_map ─▶ /map (latched)
```

- **자기 몸 빼기**: Nav2 costmap은 몸을 뺀 점구름(body_filter)을 쓰지만, nvblox는 depth 영상을 받는다. 그래서 depth에서 카메라 0.5 m 안 픽셀을 0(측정 없음)으로 바꿔서 넘긴다 (`…/aligned_depth_to_color/image_nvblox`). `scan_cam_gate`의 `depth_min_range` 옵션이 이 일을 한다.
  - 0.5 m 안의 실제 장애물은 nvblox 지도에 안 들어간다. 대신 Nav2 local costmap(몸 뺀 점구름)이 피한다.
- **실행 중 지도 모드·지도 전환**(`/cmd/map_mode` · `/cmd/map_name`): cuVSLAM·nvblox는 실행 중 전환이 없어 state_manager가 launch 전체를 인자만 바꿔 다시 띄운다 (`/cmd/restart`의 `all`과 같은 길) — MAPPING → `map:=slam`(새 폴더) · LOCALIZATION / 지도 고르기 → `map:=saved map_name:=<이름>`. 지도 목록 `/state/map_list` = `nvblox.nvblx`가 있는 폴더.
- **저장 지도로 위치추정(`map:=saved`)**: nvblox 지도를 불러오고, cuVSLAM은 원점부터 새로 시작한다. 그래서 **출발 자리·방향이 저장할 때와 같아야** 맞는다 (운영 규칙과 같음).
  - cuVSLAM 자체 지도 저장·재위치추정은 아직 없다 (6절).
- 필요한 것(cuVSLAM · nvblox)이 설치돼 있지 않으면 시작할 때 무엇이 없는지 알리고 멈춘다 (`check_slam`). SLAM 없이 나머지만 보려면 `use_slam:=false`.

cuVSLAM:

- **RGBD 모드** (D435는 IMU가 없어 VIO 불가). 입력은 pole_front_cam 컬러·정렬 depth라 카메라 설정은 안 바꿈.
- 카메라에서 0.5 m 안 픽셀(화면 가운데 자기 팔)은 마스크로 추적에서 뺌 — 자기 팔이 로봇과 같이 움직여 회전을 덜 잡게 하므로.
- EKF에는 **위치가 아니라 속도**만 넣음 — cuVSLAM 좌표는 추적을 잃으면 새로 시작해 위치를 그대로 넣으면 튐. yaw는 IMU만(기존과 같음).
- `/cuvslam/odom`이 없으면 EKF는 바퀴·IMU만 쓴다.

## 2. 설치 (Jetson Orin Nano · JetPack 7 / L4T R39 · Ubuntu 24.04 · Python 3.12 · CUDA 13)

**배포 휠(`cu13 aarch64`)은 Jetson Thor 전용이다.** ARMv9 SVE 명령이 들어 있어 Orin(Cortex-A78AE)에서는
`import cuvslam`에서 `Illegal instruction (core dumped)`로 죽는다. Orin + JetPack 7은 **소스 빌드**만 된다
(PyCuVSLAM README: 휠은 Orin=JetPack 6/cp310, Thor=JetPack 7/cp312). Isaac ROS apt 저장소에도 Lyrical용
`isaac_ros_visual_slam`은 없다.

```bash
# 1) CUDA 컴파일러·개발 라이브러리 (Jetson 공식 저장소 r39.2, 약 1GB 받기 · 2.2GB 설치)
sudo apt install -y cuda-nvcc-13-2 cuda-cudart-dev-13-2 libcublas-dev-13-2 libcusolver-dev-13-2 \
  libcusparse-dev-13-2 libnvjitlink-dev-13-2 cuda-nvrtc-dev-13-2 cuda-driver-dev-13-2
# (런타임만이면 libcublas-13-2 libcusolver-13-2 libcusparse-13-2 libnvjitlink-13-2 — 위 dev가 끌고 옴)

# 2) 소스 (git-lfs 없이도 빌드됨 — 시험 이미지만 LFS)
GIT_LFS_SKIP_SMUDGE=1 git clone --depth 1 --branch v17.0.1 https://github.com/NVlabs/PyCuVSLAM.git ~/cuvslam/src

# 3) 라이브러리 빌드 — Orin GPU(sm_87)만 · 스왑이 없어 jobs 4 (수십 분)
export PATH=/usr/local/cuda/bin:$PATH
cd ~/cuvslam
CUVSLAM_SRC_DIR=$HOME/cuvslam/src CUVSLAM_DST_DIR=$HOME/cuvslam/build \
  nohup ./src/build_release.sh --build_lib --cuda_arch=87 --jobs=4 > build.log 2>&1 &
tail -f build.log

# 4) 파이썬 바인딩 (pyyaml 말고 의존성 없음 — 시스템 numpy를 안 건드림)
ls ~/cuvslam/build/bin/libcuvslam.so
cd ~/cuvslam/src && CUVSLAM_BUILD_DIR=$HOME/cuvslam/build pip install --user --break-system-packages python/

# 5) 확인
python3 -c "import cuvslam; print(cuvslam.get_version()); cuvslam.warm_up_gpu(); print('gpu ok')"
# → ('17.0.1+51631ba', 17, 0, 1) / gpu ok
```

### 2-1. nvblox (Jetson — 아직 설치 전)

**NVIDIA Isaac ROS 5.0 apt 저장소의 Lyrical nvblox는 x86(amd64)용만 있다** (2026-10-29 확인).
- 확인한 저장소: `isaac.download.nvidia.com/isaac-ros/release-5.0`
- amd64: `ros-lyrical-isaac-ros-nvblox` · `nvblox-ros` · `nvblox-nav2` … 있음
- arm64: 파이썬 패키지 60개뿐

그래서 Jetson에서는 둘 중 하나로 올린다. **먼저 반나절 안에 어느 쪽이 되는지 확인한다.**

| 방법 | 내용 | 확인할 것 |
|---|---|---|
| (a) 소스 빌드 | `isaac_ros_nvblox`(nvblox 코어 포함)와 의존 패키지를 Lyrical 워크스페이스에서 colcon 빌드. CUDA 아키텍처 87 (Orin) | 의존: `nvblox_msgs` · `nvblox_ros_common` · `nvblox_ros_python_utils` · `nvblox_rviz_plugin` · `isaac_ros_common` · `cuda_buffer` (deb Depends 기준). 빌드 시간·메모리 (스왑 없음 → jobs 줄이기) |
| (b) Isaac ROS 컨테이너 | NVIDIA 공식 컨테이너에서 nvblox만 띄우고 나머지는 지금 Lyrical 스택 | 컨테이너가 JetPack 7 + Orin을 지원하는지 · 배포판이 달라도 `nvblox_msgs`·`OccupancyGrid`·TF가 오가는지 |

설치 확인:

```bash
ros2 pkg prefix nvblox_ros && ros2 pkg executables nvblox_ros | grep nvblox_node
python3 -c "import cuvslam; print(cuvslam.get_version())"
```

## 3. 실행

```bash
cd ~/Lv2_bee_Assignment/lv2_module6/ros2_ws
export ROS_DOMAIN_ID=42
export FASTDDS_DEFAULT_PROFILES_FILE=$PWD/fastdds.xml        # PC에서 보려면 (fastdds_local.xml은 Jetson 밖으로 안 나감)
export POLE_FRONT_CAM_SERIAL=261822074109 GRIPPER_CAM_SERIAL=261622075828
./start.sh map:=slam map_name:=<이름> use_moveit:=false
```

- **카메라 시리얼을 꼭 지정**한다. `~/.bashrc`에 있지만 SSH로 명령을 바로 실행(`ssh bee@… './start.sh …'`)하면
  `.bashrc`를 안 읽어 두 노드가 같은 카메라를 잡고 `Device or resource busy`가 난다.
- `gripper_cam_pointcloud`는 SLAM과 무관하다 (grasp_scorer용, 기본 false) — nvblox는 depth 영상을 받는다.
- 끝낼 때는 Ctrl+C 한 번(또는 `pkill -INT -f "ros2 launch bringup"`) — 매핑이면 nvblox가 끌 때 `nvblox.nvblx`를 저장한다 (강제 종료하면 저장 안 됨).

정상 로그 (10초마다):

```text
[cuvslam_slam]: cuVSLAM 17.0.1+51631ba 시작 (단독 SLAM) · 640x360 · 카메라 pole_front_cam_color_optical_frame
[cuvslam_slam]: 추적 1032 · 잃음 0
```

`추적`이 늘지 않으면 카메라 영상이 안 오는 것 — 아래 4절 "카메라·SLAM이 멈춤" 참고.

### 3-1. cuVSLAM + nvblox 실기 시험 순서 (정지 상태부터)

```bash
./start.sh map:=slam map_name:=nvblox_test use_moveit:=false
```

| 단계 | 확인 명령 | 정상 |
|---|---|---|
| 1. 노드 | `ros2 node list \| grep -E "cuvslam\|nvblox\|depth_gate"` | cuvslam_slam · nvblox_node · nvblox_map · pole_depth_gate · gripper_depth_gate |
| 2. cuVSLAM 추적 | cuvslam_slam 로그 | `시작 (단독 SLAM)` · `추적 N · 잃음 0`이 10초마다 늘어남 |
| 3. TF | `ros2 run tf2_ros tf2_echo map base_footprint` | 정지 상태에서 값이 흔들리지 않음 (1 cm · 1° 안) |
| 4. depth 마스크 | `ros2 topic hz /pole_front_cam/aligned_depth_to_color/image_nvblox` | 카메라와 같은 15 Hz. RViz에서 화면 가운데 팔 부분이 검게(0) |
| 5. nvblox 격자 | `ros2 topic hz /nvblox_node/static_occupancy_grid` | 1~5 Hz 예상 (실기에서 확인). **안 나오면 nvblox 쪽 이름·설정부터 확인** |
| 6. /map | `ros2 topic echo /map --once --field info` · nvblox_map 로그 | 해상도 0.05 · `격자 받음 N · /map 냄 N` |
| 7. Nav2 | `ros2 lifecycle get /planner_server` · RViz global costmap | active · 주변 벽이 costmap에 보임 |
| 8. 부하 | `tegrastats` | GPU·RAM 기록 (5절 cuVSLAM 단독 부하와 비교) |
| 9. 손으로 밀기 | 로봇을 손으로 20~30 cm 밀었다 되돌림 | TF가 따라가고, 되돌린 뒤 처음 자리로 돌아옴 |
| 10. 저장 | Ctrl+C 한 번 → `ls ~/.ros/beebot_maps/nvblox_test/` | `nvblox.nvblx` 생김 |
| 11. 불러오기 | 같은 자리에서 `./start.sh map:=saved map_name:=nvblox_test use_moveit:=false` | nvblox_map 로그 `지도 불러오기 … 성공` · 처음부터 /map이 그려져 있음 |

주행 시험(Nav2 목표)은 로봇 주변이 비어 있고 사람이 지켜볼 때 따로 한다.

## 4. 원격 PC에서 보기 · 주의

**PC RViz는 지도·TF·로봇 모델만 받는다** (`RVIZ_CONFIG=light`):

```bash
ROS_DOMAIN_ID=42 RVIZ_CONFIG=light lv2_module6/tool/rviz/start.sh
```

| 증상 | 원인 (실측) | 대처 |
|---|---|---|
| PC RViz를 켜면 카메라·SLAM이 멈춤 (`Did not receive data`, `추적` 안 늘어남), Jetson ping 400ms | PC RViz가 점구름·영상(초당 수 MB)을 Wi-Fi로 받아 Jetson 송신이 막힘. PC RViz를 끄자 바로 회복, ping 11ms | `RVIZ_CONFIG=light` (점구름·카메라 끔). 영상은 Jetson에서 로컬로 녹화 |
| PC에서 토픽은 보이는데 데이터가 안 옴 / Jetson이 PC 노드를 못 봄 | Wi-Fi가 PC→Jetson 멀티캐스트를 못 넘김 (Jetson 무선랜 `power_save on`) | [`develop/README.md`](develop/README.md)의 Discovery Server. 또는 **PC 쪽만** 유니캐스트 initial peers 프로파일 (아래) |
| Jetson에 `ROS_STATIC_PEERS`를 넣었더니 Jetson 노드끼리도 안 보임 | initial peers가 기본 멀티캐스트를 대체 | **Jetson 스택에는 넣지 말 것** |
| 3D 뷰가 검게 깜박임 (GNOME 배율 1.5) | Qt6 소수 배율 (ros2/rviz#1052) | `start.sh`가 `QT_SCALE_FACTOR_ROUNDING_POLICY=Floor`를 넣음 |
| SSH로 띄운 RViz가 밝은 테마 | `XDG_CURRENT_DESKTOP`이 없으면 qt6-gtk-platformtheme이 GNOME 테마를 안 따름 | `XDG_CURRENT_DESKTOP=ubuntu:GNOME DISPLAY=:1`을 붙여 실행 |

PC 쪽만 쓰는 유니캐스트 프로파일 (Jetson 스택은 재시작 안 해도 됨 — domain 42, 참가자 0~79):

```bash
python3 - > ~/fastdds_pc_unicast.xml <<'EOF'
ip, base = '10.2.12.110', 7400 + 250 * 42   # Jetson IP · ROS_DOMAIN_ID
locs = ''.join(f'<locator><udpv4><address>{ip}</address><port>{base + 10 + 2 * i}</port></udpv4></locator>' for i in range(80))
print(f'''<?xml version="1.0" encoding="UTF-8"?><dds xmlns="http://www.eprosima.com"><profiles>
<participant profile_name="pc" is_default_profile="true"><rtps><builtin><initialPeersList>
<locator><udpv4><address>239.255.0.1</address><port>{base}</port></udpv4></locator>{locs}
</initialPeersList></builtin></rtps></participant></profiles></dds>''')
EOF
export FASTDDS_DEFAULT_PROFILES_FILE=~/fastdds_pc_unicast.xml
```

## 5. 시험 기록 (2026-10-27, 실기)

| 시험 | 결과 |
|---|---|
| cuVSLAM 단독, 정지 | 15 Hz, 잃음 0, 위치 흔들림 0.000 m |
| cuVSLAM 단독, 제자리 회전(명령 360°, 실제 약 190°) | 잃음 0. 방향은 바퀴 odom과 3° 이내. 위치는 0.27 m 밀림 (회전만 하는 동작은 시각 SLAM이 약한 구간) |
| RTAB-Map + cuVSLAM 오도메트리, 미로 360°·0.4 m 왕복 | 잃음 0. 지도는 번짐 → 아래 원인 2·3 |
| RTAB-Map만(바퀴+IMU), 미로 재시험 + rosbag | 지도 품질 낮음 → 원인 2. EKF는 주행 중엔 안 튐 |
| 부하 | Jetson CPU 약 50%, GPU 2%, RAM 2.7/7.5 GB (cuVSLAM 단독 시) |

**지도 품질이 낮았던 원인 (확인한 것)**

1. PC RViz가 Wi-Fi를 막아 카메라·RTAB-Map이 멈춤 — 4절. `light`로 해결.
2. **먼 거리 depth 잡음이 격자에 쌓임** — 제자리 회전만 했는데 지도가 22 m × 30 m. D435 depth 오차는 거리 제곱으로 커지고,
   `Grid/RangeMax`가 기본값(5 m)이라 먼 잡음이 장애물로 찍힘. **다음 시험: `--Grid/RangeMax 2.5`** (아직 코드 기본값으로 넣지 않음 — 미로에서 확인 후).
3. **EKF 위치가 기동 직후 한 번 튐** (2.4 m · 5.1 m · 6.1 m, 바퀴 odom은 0.1~0.5 m). rosbag으로 보니 주행 중엔 안 튀고
   기동 시점에만 — cuVSLAM과 무관(넣기 전에도 남). 원점이 옮겨진 것과 같아 지도를 직접 망치진 않지만 원인 조사 필요.
4. 미로 바닥에서 제자리 회전이 명령의 25~30%만 나옴 (0.4 rad/s 명령 → 0.1 rad/s). 바퀴·바닥 마찰 확인 필요.

**그 밖의 실기 메모**

- `use_nav2:=false`면 velocity_smoother·collision_monitor도 꺼져 `/cmd_vel`을 내는 노드가 없다. 수동 주행은 Nav2를 켜고 `/cmd_vel_manual`로 (충돌 감시를 거침).
- collision_monitor는 몸 반경 0.20 m 원 기준이라 벽에 붙으면 **제자리 회전도 막는다** (정상 동작). 0.3 m 안에 점이 있으면 `approach`/`slowdown` 로그.
- 모터 하나(joint5) 과부하가 나면 바퀴 명령이 나가도 바퀴가 안 돌았다 (`/joint_states` 속도 0). 전원 재시작으로 풀림. `/beebotsystem/reboot_motors` 서비스가 공식 복구 경로.
- 공유 메모리 꼬임(`RTPS_TRANSPORT_SHM Error … open_and_lock_file failed`)이 보이면 모든 ROS 프로세스·`ros2 daemon`을 끄고 `/dev/shm/fastdds_*`를 지우거나 재부팅.
- 재부팅 직후 Jetson 시계가 1970년이면 GitHub HTTPS가 실패한다 — NTP 동기화(수십 초)를 기다린다.

## 6. 다음 할 일

1. nvblox를 Jetson에 올리기 (2-1절) → 3-1절 실기 시험 → 5절 미로 코스에서 지도·위치·부하 기록.
2. 먼 거리 depth 잡음(5절 원인 2)이 nvblox 격자에도 쌓이는지 확인 → 쌓이면 `config/nvblox.yaml`의 `projective_integrator_max_integration_distance_m`(지금 3.0 m)을 줄임 (값은 실기에서).
3. EKF 기동 직후 튐 원인 (바퀴 엔코더 첫 값·`/odom` 첫 메시지 의심).
4. cuVSLAM 지도 저장·재위치추정(`save_map`/`localize_in_map`) — 지금 `map:=saved`는 nvblox 지도만 불러오고 cuVSLAM은 원점에서 새로 시작 (출발 자리 고정 전제).
5. (선택) Nav2 costmap에 nvblox ESDF 층(`nvblox::nav2::NvbloxCostmapLayer`, `/nvblox_node/static_map_slice`) — 지금은 /map 격자만 씀 (위쪽 설정을 안 바꾸려고).
6. cuVSLAM 루프 클로저로 map→odom이 크게 튀면 nvblox에 이미 쌓인 복셀은 안 옮겨짐 — 튐이 크면 nvblox `global_frame: odom`으로 바꾸고 격자만 map으로 옮기는 방식 검토.
