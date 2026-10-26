# ROS 2 Lyrical (Ubuntu 24.04 / Jetson Orin Nano) 테스트베드
# ROS 2 Lyrical은 Ubuntu 26.04 베이스가 기본이지만, Jetson Orin Nano(JetPack 7 / Ubuntu 24.04)는
# NVIDIA Isaac ROS 저장소의 noble arm64 패키지로 같은 배포판을 사용한다.
#   https://nvidia-isaac-ros.github.io/getting_started/isaac_ros_buildfarm_cdn.html
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=lyrical \
    AMENT_PREFIX_PATH=/opt/ros/lyrical \
    CMAKE_PREFIX_PATH=/opt/ros/lyrical \
    PYTHONPATH=/opt/ros/lyrical/lib/python3.12/site-packages
# NOTE: setup.bash는 셸에서 source해야 적용되지만, IDE Docker 툴체인처럼
# entrypoint를 우회하는 실행에서도 find_package가 동작하도록 ENV로 고정.

# NVIDIA Isaac ROS apt 저장소 — Ubuntu 24.04(noble) arm64용 ROS 2 Lyrical 제공
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg \
    && curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
        | gpg --dearmor --yes -o /usr/share/keyrings/nvidia-isaac-ros.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-5.0 noble main" \
        > /etc/apt/sources.list.d/nvidia-isaac-ros.list \
    && echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/ubuntu/main noble main" \
        > /etc/apt/sources.list.d/nvidia-isaac-ros-buildfarm.list \
    && curl -fsSL -o /tmp/ros2-apt-source.deb \
        https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.noble_all.deb \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb \
    && apt-get update \
    && apt-get install -y --no-install-recommends ros-lyrical-ros-base \
    && rm -rf /var/lib/apt/lists/*

# 🛠 노드 빌드에 필요한 메시지·인터페이스 + 도커에서 실제로 돌리는 📦 (ros2_control · robot_state_publisher · EKF)
# + faker 파일 연동(rosx_introspection · OpenCV).
# RTAB-Map·RealSense 본체는 아직 빼고 faker 대역을 쓴다 (실기 PC에만 설치). Nav2·MoveIt은 아래에서 진짜를 설치.
RUN apt-get update && apt-get install -y --no-install-recommends \
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

# 진짜 Nav2 — ros_system이 서버를 직접 띄운다 (nav2_bringup은 시뮬레이션 의존성까지 수백 개라 안 씀).
# faker도 대역 없이 이걸 쓴다. map_server는 시뮬레이터·테스트용 코스 지도(faker/maps) (실기는 RTAB-Map이 지도·map→odom)
RUN apt-get update && apt-get install -y --no-install-recommends \
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

# 진짜 move_group — beebot_moveit_config 설정으로 실기·테스트베드 둘 다 기동 가능.
# 테스트베드도 이걸 쓴다 (mock 컨트롤러로 동작 — 대역 fake_moveit 삭제됨).
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-lyrical-moveit-ros-move-group \
    ros-lyrical-moveit-kinematics \
    ros-lyrical-moveit-planners-ompl \
    ros-lyrical-moveit-simple-controller-manager \
    ros-lyrical-moveit-ros-visualization \
    && rm -rf /var/lib/apt/lists/*

# pick_ik — MoveIt IK 플러그인 (config/moveit/kinematics.yaml). Lyrical apt에 없어 소스로 빌드해 /opt/ros/lyrical에 설치.
#   팔이 5축이라 기본 KDL은 "위치 + 위→아래 방향" 목표를 못 풀었음 (축이 모자란 팔 — pick_ik는 오차 비용을 줄여 해를 찾음).
#   커밋 고정 (재현성): PickNikRobotics/pick_ik 7c97fe2 (2026-09-18)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git ros-lyrical-rsl ros-lyrical-generate-parameter-library ros-lyrical-moveit-core ros-lyrical-tf2-kdl \
    librange-v3-dev libfmt-dev libexpected-dev \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /tmp/pick_ik_ws/src && cd /tmp/pick_ik_ws/src \
    && git clone https://github.com/PickNikRobotics/pick_ik.git && cd pick_ik && git checkout 7c97fe2 \
    && cd /tmp/pick_ik_ws && . /opt/ros/lyrical/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && cp -r install/pick_ik/. /opt/ros/lyrical/ \
    && rm -rf /tmp/pick_ik_ws
# (colcon --install-base /opt/ros/lyrical 는 ROS의 setup.bash를 덮어써서 안 씀 — 패키지 폴더만 복사: lib · share · ament_index)

# RTAB-Map — noble arm64에는 rtabmap_ros 메타패키지가 없고 rtabmap 패키지가 노드를 제공한다.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-lyrical-rtabmap-msgs \
    ros-lyrical-rtabmap \
    && rm -rf /var/lib/apt/lists/*

# YOLO perception (CPU 추론용) — torch CPU + ultralytics. libGL은 cv2용.
# 모델 가중치는 ros2_ws/model/yolo/*.pt 로 커밋 (약 6MB) → perception이 share에 설치.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 libglib2.0-0 python3-pip ros-lyrical-ament-cmake-pytest \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --break-system-packages --ignore-installed --no-cache-dir torch --index-url https://download.pytorch.org/whl/cpu \
    && pip install --break-system-packages --ignore-installed --no-cache-dir "numpy<2" ultralytics pytest

# Arduino CLI (OpenCR 펌웨어 빌드·업로드용, Linux ARM64/x86_64 지원)
# 근거: https://docs.arduino.cc/arduino-cli/installation (install.sh + BINDIR)
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
    | BINDIR=/usr/local/bin sh

# OpenCR 보드 코어 + Dynamixel2Arduino 라이브러리
# 보드 인덱스: ROBOTIS-GIT/OpenCR → arduino/opencr_release/package_opencr_index.json
# FQBN: OpenCR:OpenCR:OpenCR
# 주의: ROBOTIS 툴체인(opencr_gcc·opencr_tools)은 x86_64만 제공 (인덱스 실측).
# arm64(Mac·라즈베리파이) 우회: 보드 패키지를 수동 설치 + 우분투 시스템
# arm-none-eabi-gcc를 툴체인 자리에 연결 → 컴파일 가능 (실측 성공).
# 단 업로더(opencr_tools)는 없어서 업로드는 x86_64 실기 PC에서.
# x86_64도 같은 시스템 gcc를 쓴다: 인덱스의 opencr_gcc는 32비트(i386) 바이너리라
# 64비트 이미지(32비트 로더 /lib/ld-linux.so.2 없음)에서 "fork/exec ... no such file or directory"로 실행 불가.
# NOTE: 릴리스 파일명이 .tar.bz2지만 내용은 gzip (매직바이트 1f8b 실측) → xzf 사용.
ENV ARDUINO_UPDATER_ENABLE_NOTIFICATION=false \
    OPENCR_VERSION=1.5.3 \
    OPENCR_SHA256=418656e5e6d99d45d187ffdb28dece0f450c6707da3f6db56769f3ecafdc413c
RUN arduino-cli config init --overwrite \
    && arduino-cli config add board_manager.additional_urls \
        https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json \
    && arduino-cli core update-index \
    && apt-get update && apt-get install -y --no-install-recommends gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib \
    && rm -rf /var/lib/apt/lists/* \
    && if [ "$(uname -m)" = "x86_64" ]; then \
           arduino-cli core install OpenCR:OpenCR \
           && rm -rf /root/.arduino15/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin \
           && mkdir -p /root/.arduino15/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin \
           && for f in /usr/bin/arm-none-eabi-*; do ln -s "$f" /root/.arduino15/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin/; done \
           && echo "OpenCR ${OPENCR_VERSION} core + system arm-none-eabi-gcc (x86_64: i386 opencr_gcc 대체)"; \
       else \
           curl -fsSL "https://github.com/ROBOTIS-GIT/OpenCR/releases/download/${OPENCR_VERSION}/opencr.tar.bz2" \
               -o /tmp/opencr.tar.bz2 \
           && echo "${OPENCR_SHA256}  /tmp/opencr.tar.bz2" | sha256sum -c - \
           && mkdir -p /root/.arduino15/packages/OpenCR/hardware/OpenCR/${OPENCR_VERSION} \
               /root/.arduino15/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin \
           && tar xzf /tmp/opencr.tar.bz2 -C /tmp \
           && cp -a /tmp/opencr/. /root/.arduino15/packages/OpenCR/hardware/OpenCR/${OPENCR_VERSION}/ \
           && rm -rf /tmp/opencr /tmp/opencr.tar.bz2 \
           && for f in /usr/bin/arm-none-eabi-*; do ln -s "$f" /root/.arduino15/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin/; done \
           && echo "OpenCR ${OPENCR_VERSION} manual install (arm64 workaround)"; \
       fi \
    && arduino-cli lib install Dynamixel2Arduino

WORKDIR /ws

COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/test-serial.sh /usr/local/bin/test-serial
COPY docker/test-firmware.sh /usr/local/bin/test-firmware
COPY docker/test-hardware.sh /usr/local/bin/test-hardware
COPY docker/test-packages.sh /usr/local/bin/test-packages
COPY docker/test-bringup.sh /usr/local/bin/test-bringup
COPY docker/test-faker.sh /usr/local/bin/test-faker
COPY docker/test-all.sh /usr/local/bin/test-all
COPY docker/build-ws.sh /usr/local/bin/build-ws
COPY docker/ros-restart.sh /usr/local/bin/ros-restart
COPY docker/ros-faker-restart.sh /usr/local/bin/ros-faker-restart
COPY docker/ros-kill.sh /usr/local/bin/ros-kill
RUN chmod +x /entrypoint.sh /usr/local/bin/test-* /usr/local/bin/build-ws /usr/local/bin/ros-restart /usr/local/bin/ros-faker-restart /usr/local/bin/ros-kill \
    && echo "source /opt/ros/lyrical/setup.bash" >> /root/.bashrc \
    && echo "if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
