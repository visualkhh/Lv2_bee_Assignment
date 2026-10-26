#!/bin/bash
# Gazebo 로봇 위치 리셋 (끼임·전복 시 원위치): spawn 자리로 되돌림
#   lv2_module6/develop/gazebo/reset.sh [x y z yaw]   # 기본 -1.835 2.757 0.05 0
# NOTE: sh(dash)로 실행해도 bash로 재실행
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi
source /opt/ros/lyrical/setup.bash
: "${ROS_DOMAIN_ID:=88}"; export ROS_DOMAIN_ID
X=${1:--1.835}; Y=${2:-2.757}; Z=${3:-0.05}; YAW=${4:-0}
# yaw → 쿼터니언(z축 회전): x=0 y=0 z=sin(yaw/2) w=cos(yaw/2)
QZ=$(python3 -c "import math; print(math.sin(float('$YAW')/2))")
QW=$(python3 -c "import math; print(math.cos(float('$YAW')/2))")
timeout 30 gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 8000 --req "name: 'beebot', position: {x: $X, y: $Y, z: $Z}, orientation: {x: 0, y: 0, z: $QZ, w: $QW}" 2>&1 | tail -1
# 정지 상태로 (잔류 속도 제거)
timeout 15 gz topic -t /model/beebot/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.0}, angular: {z: 0.0}' >/dev/null 2>&1 || true
echo "리셋: ($X, $Y, $Z, yaw=$YAW)"
