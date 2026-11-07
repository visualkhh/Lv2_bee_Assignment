#!/usr/bin/env bash
# rl_nav_controller 기록 — rosbag2 (mcap).
#   ./run record ~/bags/policy
# 정책 ON/OFF 비교: ~/FollowPath/toggle 서비스로 바꿔가며 같은 주행을 두 번.
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/../../../ros2_ws" 2>/dev/null && pwd || true)"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME:-lyrical}/setup.bash"
OUT="${1:?사용법: ./run record <출력 폴더>}"

# shellcheck disable=SC1090
source "$ROS_SETUP"
if [ -n "$WS" ] && [ -f "$WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
fi

exec ros2 bag record -o "$OUT" --topics \
  /cmd_vel_nav /cmd_vel /cmd_vel_mux \
  /odometry/filtered /odom /imu \
  /plan /tf /tf_static \
  /controller_server/FollowPath/debug
