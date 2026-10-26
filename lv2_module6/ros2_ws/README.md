# lv2_module6 / ros2_ws

ROS 2 Lyrical 실기 워크스페이스 — 로봇 bringup, 지도·주행, 팔·그리퍼, 인지·계획 노드.

- 전체 설계: [`../../documents/project/ros/README.md`](../../documents/project/ros/README.md)
- 미션 설명: [`../../documents/mission/MISSION.md`](../../documents/mission/MISSION.md)
- 개발·도커·테스트: [`../simulator/gazebo`](../simulator/gazebo/DEVELOP.md)

## 1. 빠른 시작

```bash
cd lv2_module6/ros2_ws
source /opt/ros/lyrical/setup.bash

# 처음이거나 코드가 바뀌었으면
./start.sh --build

# 이미 빌드되어 있으면
./start.sh
```

`start.sh`는 `/opt/ros/lyrical`을 source하고, `install/`이 없거나 `--build`면
`colcon build --symlink-install`을 실행한 뒤 `bringup` launch를 띄웁니다.

종료:

```bash
./stop.sh
```

## 2. 장치 설정

### OpenCR

기본 포트는 udev 고정 이름 `/dev/opencr`입니다.

```bash
sudo cp 99-opencr.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

`/dev/opencr`이 없으면 `/dev/ttyACM*`를 직접 지정할 수 있습니다.

```bash
./start.sh port:=/dev/ttyACM0
```

OpenCR 없이 노드만 확인:

```bash
./start.sh mock_hardware:=true
```

### RealSense 2대

RealSense ROS 노드는 `/dev/video*`가 아니라 **serial_no**로 카메라를 고릅니다.
serial은 환경변수로 넘깁니다.

```bash
export POLE_FRONT_CAM_SERIAL=<front_serial>
export GRIPPER_CAM_SERIAL=<gripper_serial>
./start.sh
```

확인:

```bash
source /opt/ros/lyrical/setup.bash
rs-enumerate-devices -s
# 안 잡히면:
/opt/ros/lyrical/bin/rs-enumerate-devices -s
```

한 대만 연결한 경우에는 그 카메라 serial만 설정하면 `start.sh`가 반대쪽을 자동으로 끕니다.
환경변수에 남아 있는 serial이 실제로 감지되지 않아도 그 카메라를 자동으로 끕니다.

```bash
export POLE_FRONT_CAM_SERIAL=261822074109
./start.sh
# WARN: gripper_cam serial 없음 → use_gripper_cam:=false
```

serial을 하나도 지정하지 않으면 두 카메라 노드가 같은 첫 장치를 잡을 수 있으므로,
2대를 쓸 때는 반드시 `POLE_FRONT_CAM_SERIAL`·`GRIPPER_CAM_SERIAL`을 지정합니다.

직접 launch 인자로 넘길 수도 있습니다.

```bash
./start.sh \
  pole_front_cam_serial:=<front_serial> \
  gripper_cam_serial:=<gripper_serial>
```

카메라별 on/off:

```bash
./start.sh use_gripper_cam:=false       # front만
./start.sh use_pole_front_cam:=false    # gripper만
./start.sh use_cameras:=false           # 둘 다 끔
```

### 카메라 해상도·FPS

기본 카메라 프로파일은 **640×360 @ 15 fps**이고, RGB/depth에 같은 프로파일을 씁니다.
다만 점군 부하를 줄이기 위해 depth에는 **decimation filter 2배**를 적용해서
PointCloud2 밀도를 320×180 수준으로 낮춥니다. RGB 영상은 640×360 그대로입니다.

```bash
./start.sh \
  pole_front_cam_profile:=640x360x15 \
  gripper_cam_profile:=640x360x15
```

decimation도 launch 인자로 조절한다.

```bash
./start.sh camera_decimation_enable:=false   # 원본 depth/점군
./start.sh camera_decimation_magnitude:=3    # 점 수 약 1/9 (기본 2 = 1/4)
```

- `camera_decimation_enable`: 기본 `true`.
- `camera_decimation_magnitude`: 기본 `2` (2배 decimation, 3이면 3배).

지원 프로파일은 D435/D435i 기준입니다. 예: `424x240x15`, `640x480x15`.
더 줄여야 하면 해상도/FPS를 낮추고 point cloud를 끕니다.

해상도를 바꾸면 카메라 내부 파라미터(fx·fy·cx·cy)도 달라집니다. 고정 표를 쓰지 말고
실제 `/pole_front_cam/color/camera_info`·`/gripper_cam/color/camera_info` 값을 쓰세요.

```bash
./start.sh \
  pole_front_cam_profile:=424x240x15 \
  gripper_cam_profile:=424x240x15
```

- `pole_front_cam_pointcloud`: `body_filter`·Nav2용. 기본 `true`.
- `gripper_cam_pointcloud`: `grasp_scorer`용. 기본 `false` — 그리퍼 작업을 할 때만
  `gripper_cam_pointcloud:=true`로 켭니다.

관련 토픽:

```text
/pole_front_cam/depth/color/points           # RealSense 원본
/pole_front_cam/depth/color/points_filtered  # body_filter가 자기 몸 점을 뺀 것 (Nav2용)
/gripper_cam/depth/color/points              # gripper RealSense 원본
```

> Jetson(ARM)의 `realsense2_camera` 4.57.7은 pointcloud 필터를 NEON 구현으로
> 등록해서 파라미터 이름이 `pointcloud.enable`이 아니라
> **`pointcloud__neon_.enable`**이다. `ros_system.launch.py`가 ARM에서는 이 이름을
> 자동으로 선택한다. 예전처럼 `pointcloud.enable`만 넘기면 publisher가 0개가 된다.

현재 대역폭 확인:

```bash
ros2 topic bw /pole_front_cam/color/image_raw
ros2 topic bw /pole_front_cam/depth/color/points
ros2 topic hz /pole_front_cam/color/image_raw
```

Wi-Fi에서 raw Image/PointCloud2를 여러 개 보내면 쉽게 포화됩니다.  
가능하면 Jetson과 PC를 유선 Ethernet으로 연결하고, 원격 영상만 필요하면 compressed image transport를 쓰세요.

### Fast DDS 공유 메모리

영상 한 장이 Fast DDS 기본 공유 메모리(512KB)보다 커서, 프로파일 없이는 구독 노드가
영상을 거의 받지 못할 수 있습니다. `start.sh`는 아래 파일을 자동으로 지정합니다.

```text
fastdds.xml
```

`start.sh` 없이 `ros2 launch`를 직접 실행할 때는 직접 지정합니다.

```bash
export FASTDDS_DEFAULT_PROFILES_FILE=$PWD/fastdds.xml
```

### 적외선 스트림

두 카메라를 같은 USB 버스에 연결하면 IR 스트림까지 대역폭을 나눠 써서
`overflow video frame detected`와 depth 실패가 날 수 있습니다.  
`ros_system`은 IR을 쓰지 않으므로 카메라 파라미터에서 `enable_infra1`·`enable_infra2`를
끕니다.

### 카메라 QoS

RealSense의 color/depth/pointcloud는 `SENSOR_DATA`(best effort, depth 5)로 발행합니다.
카메라 원본을 최신 값만 쓰는 구독자(body_filter·grasp_scorer·VLM·state_manager)는
queue depth 1로 줄였습니다.

반면 아래 경로는 **일부러 queue를 크게** 둡니다.

- `perception`: color/depth `ApproximateTimeSynchronizer` — 큐 5
- RTAB-Map: 두 카메라·depth·odom approximate sync — 큐 10

이 경로를 1로 줄이면 한 스트림이 먼저 도착했을 때 짝을 찾지 못해
`Did not receive data`가 나거나 지도가 끊길 수 있습니다.

현재 QoS는 실기에서 직접 확인할 수 있습니다.

```bash
ros2 topic info -v /pole_front_cam/color/image_raw
ros2 topic info -v /pole_front_cam/depth/color/points
```

### 다른 PC의 RViz로 볼 때

RViz Image Display에서 다음을 맞춥니다.

```text
Reliability Policy : Best Effort
Durability Policy  : Volatile
Queue Size         : 1
```

RealSense 드라이버의 `*_qos` 프로파일은 발행 큐를 5로 고정한다.
발행 큐를 1로 바꾸는 파라미터는 없으므로, **수신하는 RViz 쪽 큐를 1로**
두는 것이 맞습니다.

네트워크 대역폭이 원인이면 raw Image 대신 compressed transport를 씁니다.
RealSense가 `compressed` 토픽을 발행하는지 확인하고, RViz의 Transport Hint를
`compressed`로 바꿉니다.

```bash
ros2 topic list | grep -E 'image_raw(/(compressed|theora))?$'
ros2 topic bw /pole_front_cam/color/image_raw
```

그래도 느리면 640×360@15에서 424×240@15로 낮추거나 유선 Ethernet을 쓰세요.
큐 1은 **밀린 프레임을 버리는** 설정이지, 네트워크 전송량을 줄이는 설정은 아닙니다.

## 3. 지도·모드

기본은 `map:=saved`입니다. `ros2_ws/map/<이름>/rtabmap.db`가 없으면
launch가 죽지 않고 **RTAB-Map 없이** 뜹니다.

```bash
# 새 지도 작성
./start.sh map:=slam

# 저장된 지도로 위치추정 (기본)
./start.sh
./start.sh map:=saved map_name:=20261017_163731

# 테스트베드 코스 지도 (RTAB-Map 없음)
./start.sh map:=course
```

- 지도 폴더: `ros2_ws/map/` (launch가 `map_dir`를 자동 지정)
- 상태: `/state/map_mode`, `/state/map_name`, `/state/map_list`

조작 모드:

```bash
ros2 topic pub --once /cmd/operation_mode std_msgs/msg/String "{data: MANUAL}"
ros2 topic pub --once /cmd/operation_mode std_msgs/msg/String "{data: AUTO}"
```

## 4. 연결 장치 상태

`state_manager`가 장치 목록과 상태를 발행합니다.

### 장치 이름 목록

```bash
ros2 topic echo /state/devices --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --field data
```

```text
pole_front_cam,gripper_cam,opencr,imu,wheels,arm,gripper
```

### 장치별 상세

```bash
ros2 topic echo /state/devices/pole_front_cam --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --field data
```

```text
camera|active|/pole_front_cam/color/image_raw,...
```

상태 값:

```text
active       = 최근 1초 안에 실제 데이터가 들어옴
disconnected = 장치가 없거나 1초 넘게 데이터가 없음
```

### 전체 토픽

```bash
ros2 topic echo /state/topics --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --field data
```

`/state/devices`에 없는 새 센서도 `/state/topics`에서 토픽 이름·타입·publisher 수를 확인할 수 있습니다.

## 5. bringup 주요 인자

| 인자 | 의미 |
| --- | --- |
| `--build` | `colcon build --symlink-install` 후 실행 |
| `mock_hardware:=true` | OpenCR 없이 제어상태를 mock으로 |
| `port:=/dev/ttyACM0` | OpenCR 포트 |
| `use_cameras:=false` | RealSense 전체 끔 |
| `use_pole_front_cam:=false` | front 카메라만 끔 |
| `use_gripper_cam:=false` | gripper 카메라만 끔 |
| `pole_front_cam_serial:=...` | front serial_no |
| `gripper_cam_serial:=...` | gripper serial_no |
| `pole_front_cam_profile:=640x360x15` | front RGB/depth 프로파일 |
| `gripper_cam_profile:=640x360x15` | gripper RGB/depth 프로파일 |
| `pole_front_cam_pointcloud:=false` | front point cloud 끔 |
| `gripper_cam_pointcloud:=false` | gripper point cloud 끔 |
| `map:=slam\|saved\|course` | 지도 모드 |
| `map_name:=<이름>` | 사용할 지도 폴더 |
| `learning_enabled:=false` | 학습 노드 비활성 |
| `use_ml:=false` | ML 노드 전체 끔 |
| `use_nav2:=false` | Nav2 끔 |
| `use_ekf:=false` | EKF 끔 |
| `use_moveit:=false` | MoveIt move_group 끔 |

### 패키지별 단독 launch

전체 로봇은 `bringup`으로 띄우고, 아래 launch는 해당 패키지 노드만 따로 시험할 때 씁니다.
`ros_system`(하드웨어·Nav2·RTAB-Map)·TF·다른 패키지 노드는 포함되지 않습니다.

```bash
ros2 launch control control.launch.py
ros2 launch manager manager.launch.py map_mode:=MAPPING
ros2 launch ml ml.launch.py learning_enabled:=false
ros2 launch perception perception.launch.py device:=cuda
ros2 launch plan plan.launch.py command:="빨간 퍽을 목표 구역으로 옮겨"
```

`BeebotSystemHardware`와 `rl_arm_correction`은 노드가 아니라 `controller_manager`가
URDF/컨트롤러 설정으로 올리는 플러그인이므로 패키지 launch에 포함되지 않습니다.

## 6. 빌드·테스트

```bash
cd lv2_module6/ros2_ws
source /opt/ros/lyrical/setup.bash

colcon build --symlink-install
colcon test
colcon test-result --verbose
```

패키지 하나만:

```bash
colcon build --packages-select manager
colcon test --packages-select manager
```

전체 클린 빌드:

```bash
./rebuild.sh
```

## 7. OpenCR 펌웨어

저장소에서:

```bash
cd ../firmware
./setup.sh                     # 최초 1회
./compile.sh                   # build/<스케치>/*.ino.bin
./upload.sh /dev/opencr        # 또는 /dev/ttyACM0
./test-firmware                # 시리얼 상태 테스트

# 설치부터 한 번에
./setup_compile_upload.sh /dev/ttyACM0 beebot_opencr
```

CI 배포 패키지에서:

```bash
./firmware-upload.sh --setup
./firmware-upload.sh /dev/opencr
```

## 8. CI 배포 패키지

```bash
cd lv2_module6/ros2_ws

./download-artifact.sh                  # RUN ID 목록
./download-artifact.sh <RUN ID>         # 현재 장치 OS·아키텍처용
```

- x86_64 Ubuntu 26.04: `ros2_ws-deploy-x86_64.tar.gz`
- aarch64 Ubuntu 26.04: `ros2_ws-deploy-aarch64.tar.gz`
- **Jetson Ubuntu 24.04**: `ros2_ws-deploy-aarch64-noble.tar.gz`

Jetson은 ROS 2 Lyrical이 `/opt/ros/lyrical`에 설치되어 있어야 합니다.
자세한 설치: [`../../documents/jetson-orin-nano/SETTING.md`](../../documents/jetson-orin-nano/SETTING.md)

## 9. 자주 막히는 것

### `/opt/ros/lyrical` 없음

```bash
source /opt/ros/lyrical/setup.bash
```

없으면 ROS 2 Lyrical을 먼저 설치합니다.

### `/dev/opencr` 없음

```bash
ls -l /dev/ttyACM* /dev/opencr
```

udev 규칙을 설치하거나 `port:=/dev/ttyACM0`을 씁니다.

### 한쪽 카메라만 잡힘

```bash
rs-enumerate-devices -s
```

serial을 환경변수로 지정하고, 한 대만 쓸 때는 반대쪽 `use_*_cam:=false`가 자동 적용되는지 로그를 확인합니다.

### 지도가 없다는 경고

```text
저장한 지도 없음 ... RTAB-Map 없이 시작
```

`map:=slam`으로 새로 만들거나, 지도 폴더에 `rtabmap.db`가 있는지 확인합니다.

### 네트워크가 터짐

- `gripper_cam_pointcloud:=false`
- `*_cam_profile`을 `424x240x15`로 낮춤
- `ros2 topic bw`로 어떤 토픽이 큰지 확인
- 가능하면 유선 Ethernet, 원격 영상은 compressed transport 사용

## 10. 관련 스크립트

| 스크립트 | 용도 |
| --- | --- |
| `start.sh` | 빌드 확인 후 bringup 실행 |
| `stop.sh` | ROS 프로세스 전체 종료 |
| `rebuild.sh` | `build/`·`install/`·`log/` 삭제 후 클린 빌드 |
| `monitor.sh` | `monitor_manager`만 실행해 debug/ 토픽 기록 |
| `download-artifact.sh` | CI 배포 패키지 다운로드 |
| `firmware-upload.sh` | 배포 패키지에서 OpenCR 펌웨어 업로드 |
| `99-opencr.rules` | OpenCR `/dev/opencr` udev 규칙 |
