# OS
1. sdk 실행
2. 리커버리 모드진입 (딥스위치 숏트)
3. 플러그인 선택 x
4. [download](https://developer.nvidia.com/sdk-manager)
5. runtime, ssd 선택
6. ubuntu 유저 세팅 skip
7. display 연결
8. ubuntu 설정 (유저 이름, 비번 설정)
9. sudo usermod -aG dialout,video bee
10. reboot
11. ssh 연결 확인 
- ssh user@jetson.local
```bash
uname -a
cat /etc/os-release
cat /etc/nv_tegra_release 2>/dev/null
dpkg --print-architecture
python3 --version
getconf GNU_LIBC_VERSION
```
```text
Linux bee 6.8.12-1021-tegra #1 SMP PREEMPT Thu Aug  6 21:56:04 PDT 2026 aarch64 aarch64 aarch64 GNU/Linux
PRETTY_NAME="Ubuntu 24.04.4 LTS"
NAME="Ubuntu"
VERSION_ID="24.04"
VERSION="24.04.4 LTS (Noble Numbat)"
VERSION_CODENAME=noble
ID=ubuntu
ID_LIKE=debian
HOME_URL="https://www.ubuntu.com/"
SUPPORT_URL="https://help.ubuntu.com/"
BUG_REPORT_URL="https://bugs.launchpad.net/ubuntu/"
PRIVACY_POLICY_URL="https://www.ubuntu.com/legal/terms-and-policies/privacy-policy"
UBUNTU_CODENAME=noble
LOGO=ubuntu-logo
# R39 (release), REVISION: 2.1, GCID: 46758480, BOARD: generic, EABI: aarch64, DATE: Fri Aug  7 05:54:22 AM UTC 2026
# KERNEL_VARIANT: oot
TARGET_USERSPACE_LIB_DIR=nvidia
TARGET_USERSPACE_LIB_DIR_PATH=usr/lib/aarch64-linux-gnu/nvidia
arm64
Python 3.12.3
glibc 2.39
```


# ros setting
## link
- https://nvidia-isaac-ros.github.io/getting_started/isaac_ros_buildfarm_cdn.html


*ROS 2 Lyrical 설치 가이드 (Ubuntu 24.04 / Jetson JetPack 7.2 공통)*
ROS 공식 저장소에는 arm64용 Lyrical이 없어서 *NVIDIA Isaac ROS 저장소*에서 설치함. 팀 노트북도 같은 방식임.

*1. 기본 도구 · universe*
```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
```

*2. ROS 공식 저장소 등록 (ros2-apt-source)*
```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

*3. Isaac ROS 저장소 등록 (Lyrical은 여기서 받음)*
```bash
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-isaac-ros.gpg
echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-5.0 noble main" | sudo tee /etc/apt/sources.list.d/nvidia-isaac-ros.list
echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/ubuntu/main noble main" | sudo tee /etc/apt/sources.list.d/nvidia-isaac-ros-buildfarm.list
sudo apt update
apt-cache policy ros-lyrical-ros-base
```
• 마지막 줄에서 Candidate에 버전이 나와야 함

*4. ROS 2 · 빌드 도구 설치*
```bash
sudo apt install -y ros-lyrical-ros-base python3-colcon-common-extensions python3-rosdep python3-vcstool python3-pip build-essential cmake
sudo rosdep init
rosdep update
```
• `ros-lyrical-desktop-full`은 설치하지 말 것 (우리 `perception` 패키지와 이름 겹침)

*5. 환경 변수 (~/.bashrc)*
```bash
cat >> ~/.bashrc <<'EOF'

# ROS 2 Lyrical — Jetson 안에서만 (공유 메모리만, 네트워크로 안 나감)
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export FASTDDS_DEFAULT_PROFILES_FILE=$HOME/Lv2_bee_Assignment/lv2_module6/ros2_ws/fastdds_local.xml
EOF
source ~/.bashrc
```
• Jetson은 ROS를 **기기 안에서만** 돌린다(bringup·`tool/run-cli`·`ros2` 명령 모두 Jetson에서). `fastdds_local.xml`이 DDS 전송을 공유 메모리(SHM)로만 제한해 노드 찾기·토픽이 네트워크로 나가지 않는다 — PC docker·시뮬레이터·다른 팀과 안 섞임
• bringup·run-cli·`ros2` 명령이 **모두 같은 프로파일**이어야 서로 보인다 → `~/.bashrc`에 두고 새 터미널에서 실행
• `ROS_LOCALHOST_ONLY`·`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`·`ROS_DISCOVERY_SERVER`는 쓰지 말 것 — LOCALHOST는 노드 일부가 안 보이고, Discovery Server는 서비스 응답이 버려져(`failed to send response … timeout`) lifecycle_manager가 velocity_smoother·collision_monitor를 못 켜 텔레오퍼가 안 움직임 (2026-10-26 실기)
• PC에서 Jetson 토픽을 직접 봐야 할 때만: 프로파일을 `fastdds.xml`(UDP 포함)로 바꾸고 양쪽 `ROS_DOMAIN_ID`를 같게 — 아래 Wi-Fi 주의 참고
• 부트캠프 Wi-Fi는 멀티캐스트가 막혀서 자동 탐색이 안 됨 → `ROS_STATIC_PEERS`에 상대 IP 필수. 여러 대면 `"IP1;IP2"`
• 이름(`xxx.local`) 대신 *IP로* 적을 것. 이름은 상대가 꺼져 있으면 노드 실행 자체가 실패함
• 내 IP 확인: `hostname -I`
• ~/.bashrc 고친 뒤엔 열려 있는 터미널마다 `source ~/.bashrc` (안 하면 예전 설정으로 실행됨)

*6. 확인*
```bash
echo $ROS_DISTRO $ROS_DOMAIN_ID $ROS_STATIC_PEERS
ros2 doctor --report | grep -E "distribution name|middleware name"
```
• `lyrical 17 <IP>` / `lyrical`, `rmw_fastrtps_cpp` 나오면 정상

*7. 통신 테스트*
기기 A
```bash
ros2 topic pub -r 2 /test std_msgs/msg/String "{data: hello}"
```
기기 B
```bash
ros2 topic echo --once /test std_msgs/msg/String
```
• `data: hello` 나오면 성공. 안 되면 양쪽 IP · ROS_STATIC_PEERS · DOMAIN_ID 확인 (Wi-Fi IP는 바뀔 수 있음)






- ros2 development tool
```bash
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep
```
- ros library
```bash
sudo apt-get install -y --no-install-recommends \
    ros-dev-tools \
    python3-colcon-common-extensions \
    ros-lyrical-nav2-msgs \
    ros-lyrical-moveit-msgs \
    ros-lyrical-moveit-task-constructor-msgs \
    ros-lyrical-control-msgs \
    ros-lyrical-controller-interface \
    ros-lyrical-hardware-interface \
    ros-lyrical-pluginlib \
    ros-lyrical-std-srvs \
    ros-lyrical-controller-manager \
    ros-lyrical-joint-state-broadcaster \
    ros-lyrical-imu-sensor-broadcaster \
    ros-lyrical-diff-drive-controller \
    ros-lyrical-joint-trajectory-controller \
    ros-lyrical-parallel-gripper-controller \
    ros-lyrical-robot-state-publisher \
    ros-lyrical-xacro \
    ros-lyrical-ros2controlcli \
    ros-lyrical-twist-mux \
    ros-lyrical-diagnostic-updater \
    ros-lyrical-diagnostic-aggregator \
    ros-lyrical-teleop-twist-keyboard \
    ros-lyrical-robot-localization \
    ros-lyrical-rosx-introspection \
    ros-lyrical-cv-bridge \
    libopencv-dev \
    rapidjson-dev \
    socat \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*
```


```bash
sudo apt-get install -y --no-install-recommends \
    ros-lyrical-nav2-lifecycle-manager \
    ros-lyrical-nav2-bt-navigator \
    ros-lyrical-nav2-behavior-tree \
    ros-lyrical-nav2-planner \
    ros-lyrical-nav2-navfn-planner \
    ros-lyrical-nav2-controller \
    ros-lyrical-nav2-dwb-controller \
    ros-lyrical-nav2-behaviors \
    ros-lyrical-nav2-costmap-2d \
    ros-lyrical-nav2-velocity-smoother \
    ros-lyrical-nav2-collision-monitor \
    ros-lyrical-nav2-map-server \
    && rm -rf /var/lib/apt/lists/*
```

```bash
sudo apt-get update && apt-get install -y --no-install-recommends \
    ros-lyrical-moveit-ros-move-group \
    ros-lyrical-moveit-kinematics \
    ros-lyrical-moveit-planners-ompl \
    ros-lyrical-moveit-simple-controller-manager \
    ros-lyrical-moveit-ros-visualization \
    && rm -rf /var/lib/apt/lists/*
```

```bash
sudo apt-get install -y --no-install-recommends \
    git ros-lyrical-rsl ros-lyrical-generate-parameter-library ros-lyrical-moveit-core ros-lyrical-tf2-kdl \
    librange-v3-dev libfmt-dev libexpected-dev \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /tmp/pick_ik_ws/src && cd /tmp/pick_ik_ws/src \
    && git clone https://github.com/PickNikRobotics/pick_ik.git && cd pick_ik && git checkout 7c97fe2 \
    && cd /tmp/pick_ik_ws && . /opt/ros/lyrical/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && cp -r install/pick_ik/. /opt/ros/lyrical/ \
    && rm -rf /tmp/pick_ik_ws
```

```bash
sudo apt install -y ros-lyrical-moveit-ros-move-group
```

# perception 파이썬 패키지 (YOLO · 표식)

```bash
pip install --break-system-packages ultralytics zxing-cpp
python3 -c "import cv2, zxingcpp; print(cv2.__version__, hasattr(cv2, 'aruco'))"   # aruco True여야 퍽 번호(AprilTag)를 읽음
```

- **AprilTag**(퍽 윗면 16mm 0~11 · 장소 48mm 12~14)는 OpenCV `aruco`(tag36h11)로 읽는다 — 따로 설치할 라이브러리 없음.
  OpenCV 4.6(Ubuntu 24.04 apt)·4.7 이상 둘 다 됨 (perception이 API 차이를 맞춤, 4.6에서 실측). `aruco`가 없는 빌드면 진단 "AprilTag 못 읽음".
- **바코드**(장소 작업 지시 P2-01 …, Code128)는 `zxing-cpp`가 필요 — OpenCV barcode는 EAN·UPC(숫자 상품코드)만 읽어서 발제 바코드를 못 읽음.
- 확인: `ros2 topic echo /diagnostics | grep -A1 "AprilTag\|바코드"`

# RealSense

```bash
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-realsense2-camera
```

# RTAB-Map (Jetson Ubuntu 24.04, ROS 2 Lyrical)

> 실행·로그·오류 추적은 [`DEBUG.md`](../ros/DEBUG.md)에 정리되어 있다.
> cuVSLAM(시각 오도메트리·단독 SLAM 시험) 설치·실행은 [`CUVSLAM.md`](CUVSLAM.md) — Orin은 배포 휠이 안 돼 소스 빌드.

NVIDIA arm64 저장소의 `ros-lyrical-pcl-conversions`는 **PCL 1.15**를 요구한다.
Jetson Ubuntu 24.04에는 **PCL 1.14**만 있으므로 이 패키지는 설치할 수 없다.
`pcl_conversions`는 소스로 빌드하고, `rtabmap_ros`도 소스로 빌드한다.

## 1. 사전 패키지

```bash
sudo apt update
sudo apt install -y \
  libpcl-dev \
  libeigen3-dev \
  ros-lyrical-image-geometry \
  ros-lyrical-cv-bridge \
  ros-lyrical-image-transport \
  ros-lyrical-laser-geometry \
  ros-lyrical-message-filters \
  ros-lyrical-pcl-msgs
```

> `ros-lyrical-pcl-conversions`, `ros-lyrical-pcl-ros`는 설치하지 않는다.  
> 위 PCL 1.15 의존성 때문에 `Unable to correct problems`가 난다.

## 2. 소스 받기

```bash
mkdir -p ~/ros2_rtabmap_ws/src
cd ~/ros2_rtabmap_ws

git clone -b lyrical-devel https://github.com/introlab/rtabmap.git src/rtabmap
git clone -b lyrical-devel https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
```

`pcl_conversions`는 **반드시 `ros2` 브랜치**를 쓴다.  
`jazzy` 브랜치는 Lyrical의 `message_filters` 헤더(.hpp)와 맞지 않아
`message_filters/message_event.h: No such file` 에러가 난다.

```bash
git clone -b ros2 https://github.com/ros-perception/perception_pcl.git src/perception_pcl
```

## 3. 빌드 (순서 중요)

```bash
cd ~/ros2_rtabmap_ws
source /opt/ros/lyrical/setup.bash

# 3-1. PCL 1.14에 맞는 pcl_conversions
colcon build --symlink-install \
  --packages-select pcl_conversions \
  --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash

# 3-2. pcl_conversions를 쓰는 rtabmap_conversions
rm -rf build/rtabmap_conversions install/rtabmap_conversions
colcon build --symlink-install \
  --packages-select rtabmap_conversions \
  --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash

# 3-3. 나머지 rtabmap/rtabmap_ros
export MAKEFLAGS="-j2"
colcon build --symlink-install \
  --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

전체 빌드 후 `rtabmap_launch`만 빠지는 경우가 있다.
이때 `--packages-select rtabmap_launch`만 쓰면 **의존 패키지 `rtabmap_odom`·`rtabmap_viz`가 없어서**
다음 에러가 날 수 있다.

```text
Failed to find ... rtabmap_viz/package.sh
Failed to find ... rtabmap_odom/package.sh
```

이 경우 `rtabmap_launch`의 의존성까지 같이 빌드한다.

```bash
source install/setup.bash

colcon build --symlink-install \
  --packages-up-to rtabmap_launch \
  --parallel-workers 1 \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`rtabmap_launch`는 우리 `ros_system`이 직접 `rtabmap.launch.py`를 쓰지는 않지만,
`ros2 launch`의 패키지 검색에서 빠지지 않도록 설치해 둔다.

## 4. 설치 확인

```bash
source ~/ros2_rtabmap_ws/install/setup.bash

ros2 pkg prefix pcl_conversions
ros2 pkg prefix rtabmap_launch
ros2 pkg prefix rtabmap_slam
ros2 pkg prefix rtabmap_sync
ros2 pkg prefix rtabmap_util
ros2 pkg prefix rtabmap_odom
ros2 pkg prefix rtabmap_viz
```

모두 실제 경로가 나와야 한다.

## 5. 실행

`start.sh`는 `~/ros2_rtabmap_ws/install/setup.bash`를 자동으로 source한다.

```bash
cd ~/Lv2_bee_Assignment/lv2_module6/ros2_ws
./start.sh
```

수동으로 source하려면:

```bash
source ~/ros2_rtabmap_ws/install/setup.bash
```

## 6. 자주 나는 빌드 에러

### `libpcl-common1.15` / `libpcl-io1.15` 없음

`ros-lyrical-pcl-conversions`를 apt로 설치하려 해서 생긴다.  
설치하지 말고 위 2번부터 소스 빌드한다.

### `Failed to find ... rtabmap_viz/package.sh` / `rtabmap_odom/package.sh`

`rtabmap_launch`만 `--packages-select`로 빌드해서 생긴다.  
`--packages-up-to rtabmap_launch`로 의존 패키지까지 빌드한다.

```bash
cd ~/ros2_rtabmap_ws
source /opt/ros/lyrical/setup.bash
source install/setup.bash

colcon build --symlink-install \
  --packages-up-to rtabmap_launch \
  --parallel-workers 1 \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

의존 패키지를 하나씩 먼저 빌드하려면:

```bash
cd ~/ros2_rtabmap_ws
source /opt/ros/lyrical/setup.bash
source install/setup.bash

# 1) rtabmap_odom
colcon build --symlink-install \
  --packages-select rtabmap_odom \
  --parallel-workers 1 \
  --event-handlers console_direct+

# 2) rtabmap_viz (Qt/RViz — 실패하면 에러를 먼저 해결)
colcon build --symlink-install \
  --packages-select rtabmap_viz \
  --parallel-workers 1 \
  --event-handlers console_direct+

# 3) rtabmap_launch
colcon build --symlink-install \
  --packages-select rtabmap_launch \
  --parallel-workers 1 \
  --event-handlers console_direct+
```

또는 의존성까지 한 번에:

```bash
colcon build --symlink-install \
  --packages-up-to rtabmap_launch \
  --parallel-workers 1 \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

확인:

```bash
source ~/ros2_rtabmap_ws/install/setup.bash

ros2 pkg prefix rtabmap_launch
ros2 pkg prefix rtabmap_odom
ros2 pkg prefix rtabmap_viz

cd ~/Lv2_bee_Assignment/lv2_module6/ros2_ws
./start.sh
```

### `Findpcl_conversions.cmake` 없음

```bash
source ~/ros2_rtabmap_ws/install/setup.bash
ros2 pkg prefix pcl_conversions
```

가 안 되면 3-1의 `pcl_conversions` 빌드가 안 된 것이다.

### `message_filters/message_event.h` 없음

`perception_pcl`을 `jazzy` 브랜치로 받은 경우다.

```bash
cd ~/ros2_rtabmap_ws/src/perception_pcl
git fetch origin ros2
git checkout ros2
git pull
```

그 뒤 `pcl_conversions`를 다시 빌드한다.

### `Findpcl_msgs.cmake` 없음

```bash
sudo apt install -y ros-lyrical-pcl-msgs
source /opt/ros/lyrical/setup.bash
ros2 pkg prefix pcl_msgs
```

가 안 되면 `pcl_msgs`도 소스 빌드한다.

```bash
git clone -b jazzy https://github.com/ros-perception/pcl_msgs.git \
  ~/ros2_rtabmap_ws/src/pcl_msgs
```

# 1. Arduino CLI 설치
```bash
mkdir -p "$HOME/bin"
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
| BINDIR="$HOME/bin" sh

export PATH="$HOME/bin:$PATH"
arduino-cli version

# PATH 영구 등록
grep -q 'export PATH="$HOME/bin:$PATH"' ~/.bashrc || \
echo 'export PATH="$HOME/bin:$PATH"' >> ~/.bashrc
# 2. OpenCR 설정값
export OPENCR_VERSION=1.5.3
export OPENCR_SHA256=418656e5e6d99d45d187ffdb28dece0f450c6707da3f6db56769f3ecafdc413c

CLI="$HOME/bin/arduino-cli"
FQBN="OpenCR:OpenCR:OpenCR"
# 3. Arduino CLI 설정 — sudo 금지
"$CLI" config init --overwrite
"$CLI" config add board_manager.additional_urls \
https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json
"$CLI" core update-index
# 4. 시스템 컴파일러 (여기만 sudo)
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
sudo rm -rf /var/lib/apt/lists/*
# 5. Jetson arm64용 OpenCR 코어 수동 설치
#    ROBOTIS 기본 툴체인은 x86용이라 시스템 arm-none-eabi를 링크해서 씁니다.
curl -fsSL \
"https://github.com/ROBOTIS-GIT/OpenCR/releases/download/${OPENCR_VERSION}/opencr.tar.bz2" \
-o /tmp/opencr.tar.bz2

echo "${OPENCR_SHA256}  /tmp/opencr.tar.bz2" | sha256sum -c -

mkdir -p "$HOME/.arduino15/packages/OpenCR/hardware/OpenCR/${OPENCR_VERSION}" \
"$HOME/.arduino15/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin"

rm -rf /tmp/opencr
tar xzf /tmp/opencr.tar.bz2 -C /tmp
cp -a /tmp/opencr/. "$HOME/.arduino15/packages/OpenCR/hardware/OpenCR/${OPENCR_VERSION}/"

rm -rf /tmp/opencr /tmp/opencr.tar.bz2

for f in /usr/bin/arm-none-eabi-*; do
ln -sf "$f" "$HOME/.arduino15/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin/"
done
# 6. Dynamixel 라이브러리
"$CLI" lib install Dynamixel2Arduino

# 7. 확인
"$CLI" board listall | grep -i opencr
```
```text
정상이면:
OpenCR Board    OpenCR:OpenCR:OpenCR
가 보입니다.  
수동 설치라 core list에는 안 보일 수 있는데, board listall에 OpenCR이 보이면 컴파일은 됩니다.
Jetson에서는 업로드가 안 됩니다
OpenCR 업로더 opencr_ld는 x86_64용만 제공됩니다.  
그래서 Jetson에서는:
컴파일   ✅
업로드   ❌
입니다.
Jetson에서 컴파일만:
"$CLI" compile --fqbn "$FQBN" \
/home/bee/.../lv2_module6/firmware/beebot_opencr
.bin 파일은 여기에 생깁니다.
find "$HOME/.cache/arduino/sketches" -name '*.ino.bin'
이 .bin을 OpenCR이 연결된 x86 PC로 복사해서 업로드하거나,  
업로드는 지금처럼 x86 PC에서 Arduino CLI로 하는 것이 안전합니다.
정리
Jetson에서 Arduino CLI 설치       ✅
OpenCR 코어/컴파일                 ✅ (시스템 gcc 사용)
Jetson에서 OpenCR 업로드          ❌ (opencr_ld arm64 없음)
업로드                            x86 PC에서
sudo arduino-cli는 절대 쓰지 말고, $HOME/bin/arduino-cli를 일반 사용자로 실행하세요.
```


# CI 배포 패키지 (Ubuntu 24.04 / Jetson)
- CI가 `aarch64`용을 두 가지로 만든다.

| 패키지 | 대상 |
| --- | --- |
| `ros2_ws-deploy-aarch64.tar.gz` | Ubuntu 26.04 ARM |
| `ros2_ws-deploy-aarch64-noble.tar.gz` | **Ubuntu 24.04 / Jetson Orin Nano** |

Jetson에서는 RUN ID로 받으면 `/etc/os-release`와 `uname -m`을 보고 noble 패키지를 자동 선택한다.

```bash
cd ~/Lv2_bee_Assignment/lv2_module6/ros2_ws
./download-artifact.sh <RUN ID>
```

패키지에는 `TARGET` 파일이 들어 있고, `start.sh`가 장치 OS와 다르면 실행을 막는다.

```bash
cat TARGET
# ubuntu-24.04-aarch64

./start.sh
```

> Ubuntu 24.04용 패키지는 ROS 2 Lyrical이 `/opt/ros/lyrical`에 설치되어 있어야 한다.
> 위 “ROS 2 Lyrical 설치 가이드”를 먼저 따른다.






# gz error
```bash

sudo apt update
sudo apt install ros-$ROS_DISTRO-gz-gui
# 또는 Gazebo 시뮬레이션 관련 메타 패키지 전체 설치
sudo apt install ros-$ROS_DISTRO-ros-gz



sudo apt update
sudo apt install libgz-gui-dev  # 혹은 버전에 따라 libgz-gui8-dev 등
```



# opencr 고정하기
```bash
lsusb
udevadm info -a -n /dev/ttyACM0
```
## editor
```
sudo vi /etc/udev/rules.d/99-opencr.rules
```
내용[99-opencr.rules](../../lv2_module6/ros2_ws/99-opencr.rules): SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="opencr", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"


# RealSense 2대 고정

OpenCR은 `/dev/ttyACM*` 번호가 꽂는 순서로 바뀌어서 udev로 `/dev/opencr`을 고정한다.  
RealSense는 다르다. **드라이버가 serial_no로 장치를 고르므로**, 카메라를 고정하는 데
`/dev/realsense_*` symlink가 **필수는 아니다.** Intel 기본 방식도 serial_no다.

- **2대 구분**: `pole_front_cam_serial` / `gripper_cam_serial`만 지정하면 된다.
- **udev**: USB 접근 권한을 주는 용도다. `/dev` 이름 고정 symlink는 사용하지 않는다.
- **한 대만 연결**: serial 없이도 첫 장치를 잡을 수 있지만, 반대쪽 카메라 노드는 꺼야 한다.

공식/일반적인 RealSense 권한 규칙 예:

```udev
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", MODE="0666", GROUP="plugdev"
```

## 1. serial 번호 확인

두 카메라를 꽂은 상태에서:

```bash
source /opt/ros/lyrical/setup.bash
rs-enumerate-devices -s
```

`rs-enumerate-devices: 명령을 찾을 수 없습니다`가 나오면
`ros-lyrical-librealsense2` 패키지의 도구가 PATH에 없는 것이다.

```bash
sudo apt install -y ros-lyrical-librealsense2
source /opt/ros/lyrical/setup.bash

# PATH에 안 잡히면 직접 실행
/opt/ros/lyrical/bin/rs-enumerate-devices -s
```

패키지를 설치하기 어렵다면 USB sysfs에서 serial만 확인할 수도 있다.

```bash
for d in /sys/bus/usb/devices/*; do
  [ "$(cat "$d/idVendor" 2>/dev/null)" = "8086" ] || continue
  printf '%s  %s  serial=%s\n' \
    "$d" "$(cat "$d/product" 2>/dev/null)" "$(cat "$d/serial" 2>/dev/null)"
done
```

출력의 `Serial Number`를 확인한다.

```text
Intel RealSense D435i
  Serial Number : 123456789012
Intel RealSense D435i
  Serial Number : 987654321098
```

어느 쪽이 `pole_front_cam`인지 `gripper_cam`인지 정한다.  
헷갈리면 한 대를 뺐다 꽂아서 `rs-enumerate-devices -s` 목록에서 사라지는 serial을 확인하면 된다.

## 2. 환경변수로 serial 지정 (권장)

`start.sh`는 아래 환경변수를 읽어서 `ros_system`의 `serial_no`로 자동 전달한다.

```bash
export POLE_FRONT_CAM_SERIAL=123456789012
export GRIPPER_CAM_SERIAL=987654321098

cd lv2_module6/ros2_ws
./start.sh
```

영구 설정은 `~/.bashrc`에 넣는다.

```bash
echo 'export POLE_FRONT_CAM_SERIAL=123456789012' >> ~/.bashrc
echo 'export GRIPPER_CAM_SERIAL=987654321098' >> ~/.bashrc
source ~/.bashrc
```

우선순위:

```text
명령줄 인자 (pole_front_cam_serial:=...)
  > 환경변수 (POLE_FRONT_CAM_SERIAL / GRIPPER_CAM_SERIAL)
```

한 대만 연결한 경우에는 **그 카메라 serial만 설정**하면 `start.sh`가 반대쪽을 자동으로 끈다.
환경변수에 남아 있는 serial이 실제로 감지되지 않아도 그 카메라를 자동으로 끈다.

```bash
bee@bee:~$ rs-enumerate-devices -s
 22/10 17:38:38,510 ERROR [281473406128160] (context.cpp:41) No valid configuration file found at : /home/bee/.realsense-config.json loading defaults
Device Name                   Serial Number       Firmware Version
Intel RealSense D435          261822074109        5.17.3.10
Intel RealSense D435          261622075828        5.17.0.10
```
```bash
# front만 연결
export POLE_FRONT_CAM_SERIAL=261822074109
./start.sh
# WARN: gripper_cam serial 없음 → use_gripper_cam:=false
```

```bash
# gripper만 연결
export GRIPPER_CAM_SERIAL=261622075828
./start.sh
# WARN: pole_front_cam serial 없음 → use_pole_front_cam:=false
```
```bash
#.bashrc 에적어둠 
export POLE_FRONT_CAM_SERIAL=261822074109
export GRIPPER_CAM_SERIAL=261622075828

```
직접 launch 인자로 넘기는 방식도 그대로 된다.

```bash
./start.sh \
  pole_front_cam_serial:=<FRONT_SERIAL> \
  gripper_cam_serial:=<GRIPPER_SERIAL>
```

> `use_cameras:=false`는 카메라 두 대를 모두 끄는 스위치다. 한 대만 쓸 때 쓰면 안 된다.

### Fast DDS 공유 메모리

영상 한 장이 Fast DDS 기본 공유 메모리(512KB)보다 커서, 프로파일 없이는 구독 노드가
영상을 거의 받지 못할 수 있다. `start.sh`는 아래 프로파일을 자동으로 지정한다.

```text
lv2_module6/ros2_ws/fastdds.xml
```

`start.sh` 없이 `ros2 launch`를 직접 실행할 때는:

```bash
export FASTDDS_DEFAULT_PROFILES_FILE=\
  $PWD/fastdds.xml
```

### 적외선 스트림 끄기

두 카메라를 같은 USB 버스에 연결하면 IR 스트림까지 대역폭을 나눠 써서
`overflow video frame detected`와 depth 실패가 날 수 있다. `ros_system`은 IR을
쓰지 않으므로 카메라 파라미터에서 `enable_infra1`·`enable_infra2`를 끈다.

## 3. 확인

```bash
ros2 topic echo /pole_front_cam/color/camera_info --once
ros2 topic echo /gripper_cam/color/camera_info --once
```

또는:

```bash
ros2 topic echo /state/devices --once \
  --qos-reliability reliable --qos-durability transient_local \
  --field data
# pole_front_cam,gripper_cam,opencr,imu,wheels,arm,gripper
```

장치별 상태:

```bash
ros2 topic echo /state/devices/pole_front_cam --once \
  --qos-reliability reliable --qos-durability transient_local \
  --field data
```

```bash
ros2 topic echo /state/devices/gripper_cam --once \
  --qos-reliability reliable --qos-durability transient_local \
  --field data
```

## 원격 접속

Jetson SSH 키 등록, `operator_tool` 원격 실행, x11vnc · GNOME RDP · NoMachine
설정은 [remote/README.md](remote/README.md)에 분리해 두었다. `remote/`에는 Jetson용
`server.sh`와 PC용 `client.sh`도 있다.
