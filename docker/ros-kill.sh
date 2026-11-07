#!/bin/bash
# ROS+Gazebo 스택 전부 죽이기 + 캐시 정리 (컨테이너·호스트 공용): ros-kill
#   ros2 launch + 노드 + 좀비 spawner · stale 데몬 캐시 · Fast DDS 공유 메모리 잔재
#   1) launch에 SIGINT → 최대 20초 기다림 (nvblox가 끌 때 지도를 저장할 시간 — 바로 죽이면 nvblox.nvblx가 안 남음)
#   2) 안 죽은 것만 SIGKILL
#   3) ros2 daemon stop (죽은 launch 그래프 캐시 → 다음 실행에 지난 노드가 보이는 것 방지)
#   4) /dev/shm/fastdds_* 삭제 (강제 종료된 노드의 공유 메모리 통로가 남으면 보내는 쪽이 막힐 수 있음)
# 다시 띄우기: ros-restart(진짜 bringup) · ros-faker-restart(테스트베드). test-faker 0 의 재시작(/cmd/restart)도 이걸 부름.
# NOTE: set -u 사용 금지 — setup.bash가 미설정 변수를 참조해서 죽음.
# shellcheck disable=SC1091
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi   # sh(dash)로 실행해도 bash로 재실행
source /opt/ros/lyrical/setup.bash 2>/dev/null || true

BIN_DIR=$(dirname "${BASH_SOURCE[0]}")
[ "$BIN_DIR" = . ] && BIN_DIR="$PWD"   # 형제 스크립트 탐색용 — 컨테이너(/usr/local/bin)·호스트(docker/) 모두 동작
if [ -z "$WS" ]; then
  if [ -d /ws/src ]; then WS=/ws; else WS="$BIN_DIR/../lv2_module6/ros2_ws"; fi   # 컨테이너면 /ws, 호스트면 레포 안 ros2_ws (머신 경로 하드코딩 없음)
fi
WS=$(realpath -m "$WS")   # ../ 정리

KILL_PAT="($WS/install|/opt/ros/lyrical)/[^ ]*lib/|ros2 launch"   # ROS 노드 전부 + launch (docker/test-*.sh와 같은 기준)
if pgrep -f "$KILL_PAT" >/dev/null; then
  pkill -INT -f "ros2 launch" 2>/dev/null
  for _ in $(seq 40); do
    pgrep -f "$KILL_PAT" >/dev/null || break
    sleep 0.5
  done
  pkill -9 -f "$KILL_PAT" 2>/dev/null || true
fi
# Gazebo 시뮬레이터 + ros_gz_bridge/relay/씬 마커/신호등/동적 장애물
# (develop/gazebo/start.sh가 띄운 것 — ROS KILL_PAT에 안 걸림)
pkill -INT -f "gz-sim-mai[n]" 2>/dev/null
pkill -INT -f "gz-sim-gui-clien[t]" 2>/dev/null
pkill -INT -f "paramete[r]_bridge.*[.]yaml" 2>/dev/null
pkill -INT -f "(gazebo/relay|gz_relay)[.]py" 2>/dev/null
pkill -INT -f "(publish_poses|gz_obstacle_markers)[.]py" 2>/dev/null
pkill -INT -f "gazebo/(traffic_lights|dynamic_obstacles)[.]py" 2>/dev/null
pkill -INT -f "rvi[z]2" 2>/dev/null   # develop/rviz/start.sh (호스트 RViz)
sleep 3
pkill -9 -f "gz-sim-mai[n]" 2>/dev/null || true
pkill -9 -f "gz-sim-gui-clien[t]" 2>/dev/null || true
pkill -9 -f "paramete[r]_bridge.*[.]yaml" 2>/dev/null || true
pkill -9 -f "(gazebo/relay|gz_relay)[.]py" 2>/dev/null || true
pkill -9 -f "(publish_poses|gz_obstacle_markers)[.]py" 2>/dev/null || true
pkill -9 -f "gazebo/(traffic_lights|dynamic_obstacles)[.]py" 2>/dev/null || true
pkill -9 -f "rvi[z]2" 2>/dev/null || true
ros2 daemon stop >/dev/null 2>&1 || true
rm -rf /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null
echo "ROS+Gazebo 스택 정리됨 (남은 노드 $(pgrep -fc "$KILL_PAT")개 · daemon·공유메모리 캐시 삭제)"
