#!/bin/bash
# Gazebo 전체 종료 (호스트): 서버 + GUI + ros_gz_bridge + relay + 실제 pose 마커
# + 분산 미션/신호등/동적 장애물 런타임
#   lv2_module6/develop/gazebo/kill.sh
# NOTE: sh(dash)로 실행해도 bash로 재실행
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi
pkill -INT -f "gz-sim-mai[n]" 2>/dev/null
pkill -INT -f "gz-sim-gui-clien[t]" 2>/dev/null
pkill -INT -f "paramete[r]_bridge.*[.]yaml" 2>/dev/null
pkill -INT -f "(gazebo/relay|gz_relay)[.]py" 2>/dev/null
pkill -INT -f "(publish_poses|gz_obstacle_markers)[.]py" 2>/dev/null
pkill -INT -f "gazebo/(traffic_lights|dynamic_obstacles)[.]py" 2>/dev/null
sleep 3
pkill -9 -f "gz-sim-mai[n]" 2>/dev/null || true
pkill -9 -f "gz-sim-gui-clien[t]" 2>/dev/null || true
pkill -9 -f "paramete[r]_bridge.*[.]yaml" 2>/dev/null || true
pkill -9 -f "(gazebo/relay|gz_relay)[.]py" 2>/dev/null || true
pkill -9 -f "(publish_poses|gz_obstacle_markers)[.]py" 2>/dev/null || true
pkill -9 -f "gazebo/(traffic_lights|dynamic_obstacles)[.]py" 2>/dev/null || true
echo "Gazebo 정리됨"
