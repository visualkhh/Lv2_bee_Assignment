#!/bin/bash
# 하드웨어 통신 검사 (컨테이너 안에서 실행): test-hardware
#   controller_manager를 실기 모드(mock_hardware:=false)로 띄워 BeebotSystemHardware가 가상 시리얼(/dev/opencr)로
#   OpenCR 펌웨어(lv2_module6/firmware/beebot_opencr)와 같은 프로토콜을 주고받는지 본다.
#   1. SBC → OpenCR: output/hardware/serial에 "C,…" 명령 줄이 쌓이는가 (값 8개)
#   2. OpenCR → SBC: input/hardware/serial에 "S,…" 상태 줄(값 32개, joint1=0.5)을 넣으면 /joint_states joint1이 0.5가 되는가
# NOTE: set -u 사용 금지 — setup.bash가 미설정 변수를 참조해서 죽음.
# shellcheck disable=SC1091
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi   # sh(dash)로 실행해도 bash로 재실행
source /opt/ros/lyrical/setup.bash
BIN_DIR=$(dirname "${BASH_SOURCE[0]}")
[ "$BIN_DIR" = . ] && BIN_DIR="$PWD"   # 형제 스크립트 탐색용 — 컨테이너(/usr/local/bin)·호스트(docker/) 모두 동작
bin_of() {  # 컨테이너는 확장자 없음, 레포는 .sh — 있는 쪽 반환, 둘 다 없으면 PATH에 기대고 bare 이름
  [ -x "$BIN_DIR/$1" ] && echo "$BIN_DIR/$1" && return 0
  [ -x "$BIN_DIR/$1.sh" ] && echo "$BIN_DIR/$1.sh" && return 0
  echo "$1"
}
if [ -z "$WS" ]; then
  if [ -d /ws/src ]; then WS=/ws; else WS="$BIN_DIR/../lv2_module6/ros2_ws"; fi   # 컨테이너면 /ws, 호스트면 레포 안 ros2_ws (머신 경로 하드코딩 없음)
fi
WS=$(realpath -m "$WS")   # ../ 정리 — CMakeCache 절대경로와 문자열 비교하므로 정규화 필수
# DDS: 컨테이너 compose와 동일 (DEVELOP.md §1). 없으면 노드 40개에서 UDP 폭주로 컨트롤러가 못 뜸
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}
export FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE:-$WS/src/ros_system/config/fastdds.xml}
cd "$WS" || { echo "FAIL: $WS 없음"; exit 2; }
[ -e /dev/opencr ] || { echo "FAIL: /dev/opencr 없음 (가상 시리얼 브릿지 미구동)"; exit 2; }
stty -F /dev/opencr raw -echo 2>/dev/null
stty -F /dev/ttyV1 raw -echo 2>/dev/null

KILL_PAT="($WS/install|/opt/ros/lyrical)/[^ ]*lib/|ros2 launch"   # ROS 노드 전부 + launch (패키지를 적으면 새 패키지가 빠져 고아로 남음)
pkill -f "$KILL_PAT" 2>/dev/null || true   # 빌드 전에 — 이전 실행의 노드가 남아 돌면 빌드도 느려짐
"$(bin_of build-ws)" || exit 1   # 빌드 + install → /debug/install (docker/build-ws.sh)
# shellcheck disable=SC1091
source install/setup.bash

if [ -z "$DEBUG_DIR" ]; then
  if [ -d /debug ]; then DEBUG_DIR=/debug; else DEBUG_DIR="$WS/../debug"; fi   # 컨테이너면 /debug, 호스트면 ros2_ws 옆 debug (머신 경로 하드코딩 없음)
fi
DEBUG_DIR=$(realpath -m "$DEBUG_DIR")
mkdir -p "$DEBUG_DIR/output/hardware" "$DEBUG_DIR/input/hardware"
: > "$DEBUG_DIR/output/hardware/serial"

set -m
ros2 launch ros_system ros_system.launch.py mock_hardware:=false \
  use_cameras:=false use_ekf:=false use_rtabmap:=false use_nav2:=false > /tmp/hardware.log 2>&1 & LAUNCH_PID=$!

# OpenCR 흉내: 0.05초마다 상태 줄 (바퀴L 0.1rad, joint1 0.5rad, IMU 정지)
S_LINE="S,0.1,0,0.2,0,0.5,0,0.1,0,0,0.1,0,0,0.1,0,0,0.1,0,0,0.1,0.01,0,0.05,0,0,0,1,0,0,0,0,0,9.81"
(for _ in $(seq 400); do echo "$S_LINE" >> "$DEBUG_DIR/input/hardware/serial"; sleep 0.05; done) & FEED_PID=$!

JS=$(timeout 60 ros2 topic echo --once --filter 'len(m.position) > 0 and abs(m.position[list(m.name).index("joint1")] - 0.5) < 1e-6' \
  /joint_states sensor_msgs/msg/JointState 2>/dev/null)
sleep 1
kill "$FEED_PID" 2>/dev/null
kill -INT "$LAUNCH_PID" 2>/dev/null
timeout 15 tail --pid="$LAUNCH_PID" -f /dev/null
pkill -9 -f "$KILL_PAT" 2>/dev/null || true

C_LINES=$(grep -c '^C,' "$DEBUG_DIR/output/hardware/serial")
C_LAST=$(grep '^C,' "$DEBUG_DIR/output/hardware/serial" | tail -n 1)
echo "--- output/hardware/serial: C 줄 ${C_LINES}개, 마지막: $C_LAST"
[ "$C_LINES" -ge 10 ] || { echo "FAIL: SBC → OpenCR 명령(C) 없음"; grep -E "BeebotSystem|OpenCR|ERROR" /tmp/hardware.log | tail -20; exit 1; }
[ "$(echo "$C_LAST" | tr ',' '\n' | wc -l)" = 9 ] || { echo "FAIL: C 줄 값 개수가 8이 아님"; exit 1; }
[ -n "$JS" ] || { echo "FAIL: OpenCR → SBC 상태(S)가 /joint_states에 안 옴"; grep -E "BeebotSystem|OpenCR|ERROR" /tmp/hardware.log | tail -20; exit 1; }
echo "PASS: hardware (BeebotSystemHardware ↔ 가상 시리얼: C 명령 송신 · S 상태 → /joint_states)"
