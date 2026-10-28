# cuVSLAM (NVIDIA 시각 SLAM) — 설치·실행·시험 기록

브랜치 `feature/cuvslam`. RTAB-Map 대신(또는 함께) NVIDIA cuVSLAM을 쓰는 시험 옵션이다.
**기본 동작은 그대로** (`slam:=rtabmap`, `cuvslam_odom:=false`) — 인자를 주지 않으면 지금까지와 같다.

- 노드: [`ros_system/scripts/cuvslam_slam.py`](../../lv2_module6/ros2_ws/src/ros_system/scripts/cuvslam_slam.py)
- launch 인자: [`ros_system.launch.py`](../../lv2_module6/ros2_ws/src/ros_system/launch/ros_system.launch.py) · [`bringup.launch.py`](../../lv2_module6/ros2_ws/src/bringup/launch/bringup.launch.py)
- EKF 입력: [`ros_system/config/ekf.yaml`](../../lv2_module6/ros2_ws/src/ros_system/config/ekf.yaml) `odom1`

## 1. 방식 3가지 (launch 인자만 바꿈)

| 방식 | 실행 인자 | map→odom | /map 격자 | 용도 |
|---|---|---|---|---|
| RTAB-Map만 (기본) | `map:=slam` | RTAB-Map | O | 지금까지와 같음 |
| **RTAB-Map + cuVSLAM 오도메트리 (권장안)** | `map:=slam cuvslam_odom:=true gripper_cam_pointcloud:=true` | RTAB-Map | O | cuVSLAM 속도를 EKF에 합쳐 바퀴 미끄러짐 보정. 지도·저장·위치추정·Nav2·explorer는 RTAB-Map 그대로 |
| cuVSLAM 단독 SLAM 시험 | `map:=slam slam:=cuvslam use_nav2:=false` | cuVSLAM | **X** | 위치 정확도·루프 클로저만 비교 (격자 없음 → Nav2 끔) |

```text
pole_front_cam 컬러 + 정렬 depth ─▶ cuvslam_slam ─▶ /cuvslam/odom (base_footprint 속도 vx·vy·vyaw)
바퀴 /odom + /imu + /cuvslam/odom ─▶ EKF ─▶ odom→base_footprint
두 카메라 점구름 ─▶ RTAB-Map ─▶ /map · map→odom · rtabmap.db
```

- cuVSLAM은 **RGBD 모드** (D435는 IMU가 없어 VIO 불가). 입력은 RTAB-Map과 같은 pole_front_cam 컬러·정렬 depth라 카메라 설정은 안 바꿈.
- 카메라에서 0.5 m 안 픽셀(화면 가운데 자기 팔)은 마스크로 추적에서 뺌 — RTAB-Map `Vis/MinDepth`와 같은 이유.
- EKF에는 **위치가 아니라 속도**만 넣음 — cuVSLAM 좌표는 추적을 잃으면 새로 시작해 위치를 그대로 넣으면 튐. yaw는 IMU만(기존과 같음).
- `map→odom`은 한 곳(RTAB-Map)에서만 낸다. 둘 다 내면 TF가 충돌한다.
- `/cuvslam/odom`이 없으면 EKF는 예전처럼 바퀴·IMU만 쓴다.

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

## 3. 실행

```bash
cd ~/Lv2_bee_Assignment/lv2_module6/ros2_ws
export ROS_DOMAIN_ID=42
export FASTDDS_DEFAULT_PROFILES_FILE=$PWD/fastdds.xml        # PC에서 보려면 (fastdds_local.xml은 Jetson 밖으로 안 나감)
export POLE_FRONT_CAM_SERIAL=261822074109 GRIPPER_CAM_SERIAL=261622075828
./start.sh map:=slam map_name:=<이름> cuvslam_odom:=true gripper_cam_pointcloud:=true use_moveit:=false
```

- **카메라 시리얼을 꼭 지정**한다. `~/.bashrc`에 있지만 SSH로 명령을 바로 실행(`ssh bee@… './start.sh …'`)하면
  `.bashrc`를 안 읽어 두 노드가 같은 카메라를 잡고 `Device or resource busy`가 난다.
- `gripper_cam_pointcloud:=true` — RTAB-Map 격자는 두 카메라 점구름을 합친 `/rtabmap/scan_cloud`로 만들어서, 이게 꺼져 있으면 `/map`이 안 나온다.
- 끝낼 때는 Ctrl+C 한 번(또는 `pkill -INT -f "ros2 launch bringup"`) — rtabmap.db가 정상 저장된다.

정상 로그 (10초마다):

```text
[cuvslam_slam]: cuVSLAM 17.0.1+51631ba 시작 (시각 오도메트리) · 640x360 · 카메라 pole_front_cam_color_optical_frame
[cuvslam_slam]: 추적 1032 · 잃음 0
```

`추적`이 늘지 않으면 카메라 영상이 안 오는 것 — 아래 4절 "카메라·RTAB-Map이 멈춤" 참고.

## 4. 원격 PC에서 보기 · 주의

**PC RViz는 지도·TF·로봇 모델만 받는다** (`RVIZ_CONFIG=light`):

```bash
ROS_DOMAIN_ID=42 RVIZ_CONFIG=light lv2_module6/tool/rviz/start.sh
```

| 증상 | 원인 (실측) | 대처 |
|---|---|---|
| PC RViz를 켜면 카메라·RTAB-Map이 멈춤 (`Did not receive data`, `추적` 안 늘어남), Jetson ping 400ms | PC RViz가 점구름·영상(초당 수 MB)을 Wi-Fi로 받아 Jetson 송신이 막힘. PC RViz를 끄자 바로 회복, ping 11ms | `RVIZ_CONFIG=light` (점구름·카메라 끔). 영상은 Jetson에서 로컬로 녹화 |
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

1. `Grid/RangeMax 2.5`로 미로 지도 재시험 → 좋아지면 `rtabmap_args` 기본값에 반영.
2. 같은 코스를 `cuvslam_odom:=false` / `true`로 각각 돌려 출발점 복귀 오차·지도 틀어진 각도 비교 (포트폴리오 근거).
3. EKF 기동 직후 튐 원인 (바퀴 엔코더 첫 값·`/odom` 첫 메시지 의심).
4. cuVSLAM 단독 모드에 지도 저장·재위치추정(`save_map`/`localize_in_map`)은 아직 없음 — 단독 모드를 운영에 쓸 때만.
