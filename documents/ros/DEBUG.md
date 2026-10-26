# Jetson 디버깅·로그 추적

Jetson에서 `start.sh`가 안 뜨거나, 뜨다가 죽거나, 센서가 안 잡힐 때 보는 절차.

- 설치/RTAB-Map/PCL 빌드: [`SETTING.md`](../jetson-orin-nano/SETTING.md)
- 전체 개발·도커 테스트: [`../../lv2_module6/simulator/gazebo`](../../lv2_module6/simulator/gazebo/DEVELOP.md)

## 1. 먼저 구분할 것

로그를 볼 때 이 두 가지를 먼저 구분한다.

### 기동 중 에러

```text
[ERROR] ... package 'X' not found
[ERROR] ... process has died
[ERROR] ... Failed to bring up
```

프로세스가 **처음 올라오는 도중** 실패한 것이다. 이게 진짜 원인이다.

### 종료 중 에러

```text
[INFO] process has finished cleanly
[INFO] sending signal 'SIGINT' ...
[ERROR] failed to terminate ... escalating to SIGKILL
planner_server ... Aborted (Signal sent by tkill)
```

`Ctrl-C`나 SSH 종료로 내려가는 중에 나는 2차 에러다.  
`planner_server`·`realsense`의 abort/SIGKILL이 여기에 해당하면 **런타임 크래시가 아닐 수 있다.**

> 로그의 **맨 처음 ERROR**를 먼저 봐야 한다.  
> 마지막 shutdown 로그만 보면 원인을 잘못 판단한다.

## 2. 로그 위치

### ROS launch / 노드

```bash
ls -lt ~/.ros/log/latest/
find ~/.ros/log/latest -maxdepth 2 -type f | sort
```

- launch가 띄운 노드의 stdout/stderr
- launch 자체 로그
- `~/.ros/log/latest/launch.log` 등

직접 리다이렉트했다면:

```bash
less ~/beebot-start.log
```

### Gazebo / 센서 스택

`../../lv2_module6/simulator/gazebo`가 띄운 경우:

```bash
tail -100 /tmp/gz.log
tail -100 /tmp/gz_bridge.log
tail -100 /tmp/gz_relay.log
tail -100 /tmp/gz_scene_markers.log
tail -100 /tmp/gz_traffic.log
tail -100 /tmp/gz_dynamic.log
```

### 빌드 로그

```bash
cd ~/ros2_rtabmap_ws
ls -lt log/latest_build/
tail -100 log/latest_build/<패키지>/stdout_stderr.log
```

```bash
cd ~/Lv2_bee_Assignment/lv2_module6/ros2_ws
ls -lt log/latest_build/
tail -100 log/latest_build/<패키지>/stdout_stderr.log
```

### 테스트 로그

```bash
cd ~/Lv2_bee_Assignment/lv2_module6/ros2_ws
colcon test-result --verbose
```

### Docker 테스트베드

```bash
docker ps -a
docker logs --tail 200 <컨테이너>
```

## 3. 실행 로그를 파일로 남기기

터미널이 끊겨도 로그를 보려면:

```bash
cd ~/Lv2_bee_Assignment/lv2_module6/ros2_ws
# /tmp는 RAM(tmpfs)이라 무한 증가 로그를 두면 메모리를 먹는다 — 디스크 파일에 시작할 때 비우고 쓴다.
: > ~/beebot-start.log
setsid ./start.sh >> ~/beebot-start.log 2>&1 < /dev/null &
tail -f ~/beebot-start.log
```

종료:

```bash
./stop.sh
```

> `stop.sh`는 같은 머신의 ROS 프로세스를 모두 종료한다.

## 4. 첫 에러 찾기

```bash
grep -n -i -E 'error|exception|traceback|failed|abort|not found|died' ~/beebot-start.log | head -60
```

- `package '...' not found`
- `Exec format error`
- `GLIBC_... not found`
- `process has died`
- `Failed to bring up`
- `Aborted`
- `Killed` / `oom`

첫 ERROR부터 위로 올라가며 원인을 찾는다.

## 5. 환경·버전 확인

```bash
uname -a
cat /etc/os-release
cat /etc/nv_tegra_release 2>/dev/null
dpkg --print-architecture
python3 --version
getconf GNU_LIBC_VERSION

source /opt/ros/lyrical/setup.bash
echo "ROS_DISTRO=$ROS_DISTRO"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_AUTOMATIC_DISCOVERY_RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE"
```

설치된 패키지:

```bash
ros2 pkg prefix rtabmap_launch
ros2 pkg prefix rtabmap_slam
ros2 pkg prefix rtabmap_sync
ros2 pkg prefix rtabmap_util
ros2 pkg prefix pcl_conversions
```

## 6. ROS 그래프 확인

```bash
ros2 node list
ros2 topic list
ros2 topic info -v /tracked_objects
ros2 topic info -v /pole_front_cam/color/image_raw
ros2 topic info -v /state/devices
```

`state_manager` 확인:

```bash
ros2 node list | grep state_manager

ros2 topic echo /state/devices --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --field data
```

장치별 상태:

```bash
ros2 topic echo /state/devices/opencr --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --field data
```

전체 토픽 덤프:

```bash
ros2 topic echo /state/topics --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --field data
```

### QoS 때문에 안 보일 때

`/state/*`는 `transient_local`이다. 기본 `ros2 topic echo`로 안 보이면 위처럼 옵션을 붙인다.

```bash
ros2 topic echo /state/operation_mode --once \
  --qos-durability transient_local
```

### 토픽 주기·대역폭

```bash
ros2 topic hz /pole_front_cam/color/image_raw
ros2 topic bw /pole_front_cam/color/image_raw
ros2 topic bw /pole_front_cam/depth/color/points
```

카메라/포인트클라우드가 Wi-Fi를 포화시키는지 확인할 때 쓴다.

## 7. 컨트롤러·라이프사이클

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
```

정상이라면 컨트롤러가 `active`이고, Nav2 노드가 `active [3]`이다.

```bash
ros2 topic echo /diagnostics_agg --once
# 같은 내용을 CLI로 — 요약 / 항목 하나의 값 전부
~/Lv2_bee_Assignment/lv2_module6/tool/run-cli status --diagnostics
~/Lv2_bee_Assignment/lv2_module6/tool/run-cli status --diagnostics signal_decoder
```

진단(`/diagnostics`)은 사람이 보는 것이다. 노드끼리는 아래 전용 토픽을 쓴다 — 진단 문자열과 토픽 값이 다르면 토픽이 맞다.

### 모터(DYNAMIXEL) 에러

```bash
# 모터별 Hardware Error — 0 정상 · -1 응답 없음 · 비트 1 입력전압 4 과열 8 엔코더 16 전기충격 32 과부하
ros2 topic echo /motor_status --once --qos-durability transient_local
# state_manager가 그걸로 만든 장치 상태 — error + "joint2:과부하"
ros2 topic echo /state/devices/arm --once --qos-durability transient_local --field data
```

과부하·과열은 모터가 토크를 끄고 **재부팅으로만** 풀린다. 원인(걸림·무게)을 없앤 뒤 수동 모드에서:

```bash
~/Lv2_bee_Assignment/lv2_module6/tool/run-cli mode manual
~/Lv2_bee_Assignment/lv2_module6/tool/run-cli arm reboot        # = ros2 topic pub --once /cmd/manual/reboot_motors std_msgs/msg/Empty {}
```

12V가 꺼졌다 켜진 경우(토크 꺼짐)는 펌웨어가 0.5초마다 확인해 천천히 다시 켠다 — [firmware/README.md](../../lv2_module6/firmware/README.md).

### 신호등 미션·계획

```bash
# signal_decoder가 읽은 답 {color, count, sum, dest_tag, raw}
ros2 topic echo /mission/signal --once --qos-durability transient_local
# planner가 낸 계획 · coordinator 진행 (FAILED면 reason)
ros2 topic echo /task/sub_goals --once --qos-durability transient_local
ros2 topic echo /task/status --qos-durability transient_local
# planner 판단 이유 — "신호등 미션"(carry·inspect·wait·fail) · "수납 확인"(look·ok·recarry·missing)
~/Lv2_bee_Assignment/lv2_module6/tool/run-cli status --diagnostics vlm_task_planner
```

- 신호가 안 잡힘: `/tracked_objects`에 `traffic_light`가 있고 `color`가 바뀌는지 (`all` = 세 색 동시 — 단계 구분). 단계 구분 시간은 `plan.yaml` `signal_decoder`.
- 계속 `wait`: 목적지 장소 태그(`location`, tag 12~14)를 아직 못 봤다. 퍽 번호(`details.tag`)는 gripper_cam이 위에서 봐야 읽힌다 (`inspect`).
- 수납 확인은 옮긴 직후 한 번, 그 뒤 `verify_period_s`(기본 60초)마다 AUTO이고 하던 계획이 없을 때만 돈다.

`state_manager` 진단도 확인:

```bash
ros2 param get /state_manager map_mode
ros2 param get /state_manager map_name
```

## 8. RTAB-Map / 빌드 에러

자세한 설치는 [`SETTING.md`](../jetson-orin-nano/SETTING.md)의 **RTAB-Map 설치** 참고.

### `package 'rtabmap_launch' not found`

```bash
source ~/ros2_rtabmap_ws/install/setup.bash
ros2 pkg prefix rtabmap_launch
```

없으면:

```bash
cd ~/ros2_rtabmap_ws
source /opt/ros/lyrical/setup.bash
source install/setup.bash

colcon build --symlink-install \
  --packages-select rtabmap_launch \
  --parallel-workers 1 \
  --event-handlers console_direct+
```

`start.sh`는 `~/ros2_rtabmap_ws/install/setup.bash`를 자동으로 source한다.

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

### `libpcl-common1.15` / `libpcl-io1.15` 없음

Jetson Ubuntu 24.04에는 PCL 1.14만 있다.  
`ros-lyrical-pcl-conversions`를 apt로 설치하지 말고 `SETTING.md`의 소스 빌드를 따른다.

### `message_filters/message_event.h` 없음

`perception_pcl`을 `jazzy` 브랜치로 받은 경우다.

```bash
cd ~/ros2_rtabmap_ws/src/perception_pcl
git fetch origin ros2
git checkout ros2
git pull
```

그 뒤 `pcl_conversions`부터 다시 빌드한다.

### `Findpcl_conversions.cmake` / `Findpcl_msgs.cmake` 없음

```bash
source ~/ros2_rtabmap_ws/install/setup.bash
ros2 pkg prefix pcl_conversions
ros2 pkg prefix pcl_msgs
```

경로가 없으면 해당 패키지를 먼저 빌드/설치한다.

## 9. launch 에러 패턴

### `Exec format error`

다른 CPU용 패키지다.

```bash
uname -m
cat ARCH
```

- `aarch64` ←→ `x86_64` 패키지를 섞으면 안 된다.
- Jetson Ubuntu 24.04는 `aarch64-noble` 패키지다.

### `GLIBC_... not found`

Ubuntu 26.04용 패키지를 24.04에서 실행한 경우다.  
`TARGET` 파일과 `/etc/os-release`를 확인한다.

### `planner_server Aborted` / `SIGKILL`

`SIGINT` 이후에 나오면 **종료 중 2차 에러**다.  
기동 중 abort라면 첫 ERROR와 Nav2 파라미터를 확인한다.

```bash
grep -n -i -E 'planner_server|lifecycle_manager|Failed to bring up' ~/beebot-start.log | head -40
```

### 카메라가 안 잡힘

```bash
source /opt/ros/lyrical/setup.bash
rs-enumerate-devices -s
lsusb -d 8086:
```

```bash
export POLE_FRONT_CAM_SERIAL=<serial>
export GRIPPER_CAM_SERIAL=<serial>
./start.sh
```

한 대만 있으면 연결된 serial만 설정한다.

### OpenCR 포트가 안 잡힘

```bash
ls -l /dev/opencr /dev/ttyACM*
id | grep dialout
```

```bash
sudo usermod -aG dialout "$USER"
# 재로그인
```

### ROS 노드가 서로 안 보임

```bash
echo "$ROS_DOMAIN_ID"
ros2 daemon stop
ros2 topic list
```

- 양쪽 `ROS_DOMAIN_ID` 동일
- Wi-Fi면 `ROS_STATIC_PEERS` 확인
- `ROS_AUTOMATIC_DISCOVERY_RANGE` 확인
- DDS 캐시가 의심되면 `ros2 daemon stop` 후 다시

## 10. 메모리·CPU

```bash
free -h
dmesg -T | tail -60
journalctl -k --since '30 min ago' | tail -60
```

`Out of memory: Killed process`, `oom-kill`이 보이면 메모리 부족이다.

```bash
top -b -n 1 | head -30
pgrep -af 'ros2|realsense|planner|controller|rtabmap'
```

## 11. 디버그 묶음 만들기

```bash
D=/tmp/beebot-debug-$(date +%Y%m%d-%H%M%S)
mkdir -p "$D"

cp ~/beebot-start.log "$D/" 2>/dev/null || true
cp -a ~/.ros/log/latest "$D/ros-log" 2>/dev/null || true

{
  uname -a
  cat /etc/os-release
  cat /etc/nv_tegra_release 2>/dev/null
  echo
  source /opt/ros/lyrical/setup.bash
  echo "ROS_DISTRO=$ROS_DISTRO"
  echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  ros2 pkg prefix rtabmap_launch 2>&1
  ros2 pkg prefix rtabmap_slam 2>&1
  ros2 pkg prefix pcl_conversions 2>&1
} > "$D/env.txt" 2>&1

ros2 node list > "$D/nodes.txt" 2>&1 || true
ros2 topic list > "$D/topics.txt" 2>&1 || true
dmesg -T > "$D/dmesg.txt" 2>&1 || true

echo "$D"
```

> 토큰·비밀번호·개인정보가 들어간 파일은 공유하지 않는다.

## 12. 도움을 요청할 때 같이 보낼 것

1. 실행한 전체 명령
2. `Ctrl-C`를 눌렀는지 여부
3. `~/beebot-start.log`의 **맨 처음 ERROR 30~50줄**
4. `ros2 pkg prefix rtabmap_launch` 결과
5. `uname -m`, Ubuntu 버전, `ROS_DISTRO`, `ROS_DOMAIN_ID`
6. `dmesg -T | tail -30`
