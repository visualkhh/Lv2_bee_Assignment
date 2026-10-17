#!/bin/bash
# 하드웨어 통신 검사 (컨테이너 안에서 실행): test-hardware
#   controller_manager를 실기 모드(mock_hardware:=false)로 띄워 BeebotSystemHardware가 가상 시리얼(/dev/opencr)로
#   OpenCR 펌웨어(lv2_module6/firmware/beebot_opencr)와 같은 프로토콜을 주고받는지 본다.
#   1. SBC → OpenCR: output/hardware/serial에 "C,…" 명령 줄이 쌓이는가 (값 8개)
#   2. OpenCR → SBC: input/hardware/serial에 "S,…" 상태 줄(값 32개, joint1=0.5)을 넣으면 /joint_states joint1이 0.5가 되는가
# NOTE: set -u 사용 금지 — setup.bash가 미설정 변수를 참조해서 죽음.
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
cd /ws || { echo "FAIL: /ws 없음"; exit 2; }
[ -e /dev/opencr ] || { echo "FAIL: /dev/opencr 없음 (가상 시리얼 브릿지 미구동)"; exit 2; }
stty -F /dev/opencr raw -echo 2>/dev/null
stty -F /dev/ttyV1 raw -echo 2>/dev/null

KILL_PAT='(/ws/install|/opt/ros/lyrical)/[^ ]*lib/|ros2 launch'   # ROS 노드 전부 + launch (패키지를 적으면 새 패키지가 빠져 고아로 남음)
pkill -f "$KILL_PAT" 2>/dev/null || true   # 빌드 전에 — 이전 실행의 노드가 남아 돌면 빌드도 느려짐
build-ws || exit 1   # 빌드 + install → /debug/install (docker/build-ws.sh)
# shellcheck disable=SC1091
source install/setup.bash

: > /debug/output/hardware/serial

set -m
ros2 launch ros_system ros_system.launch.py mock_hardware:=false \
  use_cameras:=false use_ekf:=false use_rtabmap:=false use_nav2:=false > /tmp/hardware.log 2>&1 & LAUNCH_PID=$!

# OpenCR 흉내: 0.05초마다 상태 줄 (바퀴L 0.1rad, joint1 0.5rad, IMU 정지)
S_LINE="S,0.1,0,0.2,0,0.5,0,0.1,0,0,0.1,0,0,0.1,0,0,0.1,0,0,0.1,0.01,0,0.05,0,0,0,1,0,0,0,0,0,9.81"
(for _ in $(seq 400); do echo "$S_LINE" >> /debug/input/hardware/serial; sleep 0.05; done) & FEED_PID=$!

JS=$(timeout 60 ros2 topic echo --once --filter 'len(m.position) > 0 and abs(m.position[list(m.name).index("joint1")] - 0.5) < 1e-6' \
  /joint_states sensor_msgs/msg/JointState 2>/dev/null)
sleep 1
kill "$FEED_PID" 2>/dev/null
kill -INT "$LAUNCH_PID" 2>/dev/null
timeout 15 tail --pid="$LAUNCH_PID" -f /dev/null
pkill -9 -f "$KILL_PAT" 2>/dev/null || true

C_LINES=$(grep -c '^C,' /debug/output/hardware/serial)
C_LAST=$(grep '^C,' /debug/output/hardware/serial | tail -n 1)
echo "--- output/hardware/serial: C 줄 ${C_LINES}개, 마지막: $C_LAST"
[ "$C_LINES" -ge 10 ] || { echo "FAIL: SBC → OpenCR 명령(C) 없음"; grep -E "BeebotSystem|OpenCR|ERROR" /tmp/hardware.log | tail -20; exit 1; }
[ "$(echo "$C_LAST" | tr ',' '\n' | wc -l)" = 9 ] || { echo "FAIL: C 줄 값 개수가 8이 아님"; exit 1; }
[ -n "$JS" ] || { echo "FAIL: OpenCR → SBC 상태(S)가 /joint_states에 안 옴"; grep -E "BeebotSystem|OpenCR|ERROR" /tmp/hardware.log | tail -20; exit 1; }
echo "PASS: hardware (BeebotSystemHardware ↔ 가상 시리얼: C 명령 송신 · S 상태 → /joint_states)"
