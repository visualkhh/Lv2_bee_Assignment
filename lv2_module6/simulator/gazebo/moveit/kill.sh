#!/bin/bash
# MoveIt RViz 종료 (호스트): lv2_module6/develop/moveit/start.sh가 띄운 것만 (기본 RViz는 건드리지 않음)
#   lv2_module6/develop/moveit/kill.sh
# NOTE: sh(dash)로 실행해도 bash로 재실행
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi
pkill -INT -f "rvi[z]2.*moveit[.]rviz" 2>/dev/null
sleep 2
pkill -9 -f "rvi[z]2.*moveit[.]rviz" 2>/dev/null || true
echo "MoveIt RViz 정리됨"
