#!/usr/bin/env bash
# 시뮬 리셋: Isaac을 끄지 않고 장면을 처음 자세로 + ROS 노드 재시작 (isaac_sim.sh가 떠 있을 때)
PID_FILE=/tmp/beebot_isaac_sim.pid
[ -e "$PID_FILE" ] && kill -USR1 "$(cat "$PID_FILE")" 2>/dev/null || { echo "isaac_sim.sh가 안 떠 있음"; exit 1; }
echo "리셋 요청함 — isaac_sim.sh 터미널에서 노드가 다시 뜨면 시연 실행"
