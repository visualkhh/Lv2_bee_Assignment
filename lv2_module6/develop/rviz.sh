#!/bin/bash
# RViz2 실행 (호스트): URDF 오버레이 + TF + Map 기본 구성
#   develop/rviz.sh [rviz 인자...]   (-d 다른 config 지정 가능)
# NOTE: sh(dash)로 실행해도 bash로 재실행
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi
source /opt/ros/lyrical/setup.bash
: "${ROS_DOMAIN_ID:=8}"; export ROS_DOMAIN_ID   # test-faker 돌린 셸과 같은 값
HERE=$(dirname "${BASH_SOURCE[0]}")
[ "$HERE" = . ] && HERE="$PWD"
WS="$HERE/../ros2_ws"
if [ -f "$WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"   # package://ros_system/... 메시 경로 해결용 (안 하면 RViz에 하얀 박스만 나옴)
fi
if [ $# -eq 0 ] && [ -f "$HERE/rviz_default.rviz" ]; then
  exec rviz2 -d "$HERE/rviz_default.rviz"
fi
exec rviz2 "$@"
