#!/bin/bash
# 키보드 텔레옵 (호스트): Gazebo 로봇 조종
#   develop/teleop.sh [twist 토픽]   # 기본 /gz/cmd_vel (testbed 안 건드림)
# 조작: i=전진 ,=후진 j=좌회전 l=우회전 k=정지 · shift=빠르게 · q/z 속도 +- · 종료는 ctrl-c
# NOTE: sh(dash)로 실행해도 bash로 재실행
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi
source /opt/ros/lyrical/setup.bash
: "${ROS_DOMAIN_ID:=8}"; export ROS_DOMAIN_ID
TOPIC="${1:-/gz/cmd_vel}"
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=$TOPIC
