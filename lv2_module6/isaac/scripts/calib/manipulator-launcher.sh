#!/bin/bash
# Manipulator bringup for real-robot tests in this folder: arm, gripper, both RealSense cameras, perception (YOLO),
# hand_relative, MoveIt, coordinator, skill nodes, grasp_scorer — no SLAM / Nav2 (map = where the robot was powered on).
# What is left out: ros2_ws/src/bringup/launch/stand_test.launch.py (run through ros2_ws/start.sh and its env checks:
# OpenCR · camera serials · MoveIt).
#
#   ./manipulator-launcher.sh                                              # default
#   ./manipulator-launcher.sh perception_device:=cuda                      # YOLO on GPU
#
# Starts clean (skip with STAND_NO_CLEAN=1): ros2_ws/stop.sh stops running ROS (launch · nodes · ros2 CLI) and the
# ros2 daemon and removes Fast DDS leftovers in /dev/shm — other ROS work on this machine stops too.
# Turns on the repo venv (.venv — YOLO ultralytics · torch) when there is one.
# Residual policies (arm Δ) are switched only per skill in ros2_ws/model/skills/<skill>/meta.yaml (use: false) + colcon build —
# learning_enabled:=false does not turn them off (it only reaches grasp_scorer, which is rule-based either way).
WS="$(cd "$(dirname "$0")/../../../ros2_ws" && pwd)"
VENV="$WS/../../.venv/bin/activate"
if [ -z "${STAND_NO_CLEAN:-}" ]; then
  "$WS/stop.sh"
fi
if [ -z "${VIRTUAL_ENV:-}" ] && [ -f "$VENV" ]; then
  # shellcheck disable=SC1090
  source "$VENV"
  echo "venv: $VIRTUAL_ENV"
fi
# SHM-only profile by default (one machine; start.sh default fastdds.xml = UDP+SHM). 11/04: with fastdds.xml the EKF got
# no TF ("Failed to meet update rate") and odom TF never came. Other profile: FASTDDS_DEFAULT_PROFILES_FILE=... ./manipulator-launcher.sh
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-$WS/fastdds_local_only.xml}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
LAUNCH_FILE=stand_test.launch.py exec "$WS/start.sh" "$@"
