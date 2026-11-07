#!/bin/bash
# RViz 종료 (호스트): lv2_module6/develop/rviz/start.sh가 띄운 것
#   lv2_module6/develop/rviz/kill.sh
# NOTE: sh(dash)로 실행해도 bash로 재실행
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi
pkill -INT -f "rvi[z]2" 2>/dev/null
sleep 2
pkill -9 -f "rvi[z]2" 2>/dev/null || true
echo "RViz 정리됨"
