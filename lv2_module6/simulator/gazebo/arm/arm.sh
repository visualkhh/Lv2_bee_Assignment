#!/bin/bash
# 팔 슬라이더 GUI (호스트): testbed arm_controller(JTC)를 직접 움직임 — RViz RobotModel에 반영
#   lv2_module6/develop/arm/arm.sh [컨트롤러]   # 기본 arm_controller
# NOTE: sh(dash)로 실행해도 bash로 재실행
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi
source /opt/ros/lyrical/setup.bash
: "${ROS_DOMAIN_ID:=88}"; export ROS_DOMAIN_ID
if ! ros2 pkg list 2>/dev/null | grep -qx rqt_joint_trajectory_controller; then
  echo "필요: sudo apt install -y ros-lyrical-rqt-joint-trajectory-controller"
  echo "설치 후 다시 실행하십시오. (rqt → Plugins → Controllers → Joint trajectory controller 수동 선택도 가능)"
  exit 2
fi
CTRL="${1:-arm_controller}"
# rqt 플러그인에서 $CTRL 선택 (드롭다운)
exec rqt --standalone rqt_joint_trajectory_controller/JointTrajectoryController 2>/dev/null \
  || exec rqt
