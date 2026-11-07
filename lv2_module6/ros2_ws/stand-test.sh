#!/bin/bash
# 팔 작업 테스트(세우기 stand · 집기 pick)용 가벼운 실행 — SLAM·Nav2 없이 팔·카메라·인지·MoveIt·coordinator만
#   (무엇을 빼고 남기는지: src/bringup/launch/stand_test.launch.py)
#
#   ./stand-test.sh                         # 기본
#   ./stand-test.sh use_rl_policy:=true     # 잔차 정책 노드까지 (ros2 param set /coordinator rl_policy true와 함께)
#
# 저장소 가상환경(../../.venv — YOLO ultralytics · torch)이 있으면 켜고, start.sh의 환경 확인(OpenCR · 카메라 serial · MoveIt)을 그대로 거친다.
#
# 시작 전에 깨끗한 상태로 만든다 (끄려면 STAND_NO_CLEAN=1 ./stand-test.sh):
#   ./stop.sh — 떠 있는 ROS(launch · 노드 · ros2 CLI)와 ros2 daemon 끄기 + /dev/shm의 Fast DDS 잔재 삭제
# 같은 머신의 다른 ROS 작업도 같이 꺼진다 (stop.sh와 같음).
HERE="$(cd "$(dirname "$0")" && pwd)"
VENV="$HERE/../../.venv/bin/activate"
if [ -z "${STAND_NO_CLEAN:-}" ]; then
  "$HERE/stop.sh"
fi
if [ -z "${VIRTUAL_ENV:-}" ] && [ -f "$VENV" ]; then
  # shellcheck disable=SC1090
  source "$VENV"
  echo "가상환경: $VIRTUAL_ENV"
fi
# Jetson 한 대 안에서만 도는 테스트라 SHM 전용 프로파일을 기본으로 (start.sh 기본은 fastdds.xml = UDP+SHM).
# 11/04: fastdds.xml로 뜬 스택에서 EKF가 TF를 못 받아 "Failed to meet update rate"로 막히고 odom TF가 안 나왔음.
# 다른 값으로 쓰려면 FASTDDS_DEFAULT_PROFILES_FILE=... ./stand-test.sh
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-$HERE/fastdds_local_only.xml}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
LAUNCH_FILE=stand_test.launch.py exec "$HERE/start.sh" "$@"
