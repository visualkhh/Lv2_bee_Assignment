#!/bin/bash
# Manipulator bringup for real-robot tests in this folder: arm, gripper, both RealSense cameras, perception (YOLO),
# hand_relative, MoveIt, coordinator, skill nodes, grasp_scorer — no SLAM / Nav2 (map = where the robot was powered on).
# What is left out: ros2_ws/src/bringup/launch/stand_test.launch.py (run through ros2_ws/start.sh and its env checks:
# OpenCR · camera serials · MoveIt).
#
#   ./manipulator-launcher.sh                                              # default
#   ./manipulator-launcher.sh perception_device:=cuda                      # YOLO on GPU
#   ./manipulator-launcher.sh use_sim:=true                                # Isaac Sim beebot (start isaac_sim.sh first, separately)
#
# use_sim:=true — the same manipulator stack against Isaac Sim (isaac/scripts/standalone/isaac_sim.sh, started on its own):
#   waits for Isaac to Play (/tmp/opencr = its virtual OpenCR), stops whatever ROS is up (also the full isaac_system bringup
#   isaac_sim.sh starts with it; Isaac itself keeps running), then launches isaac/scripts/standalone/isaac_manipulator_demo.launch.py
#   (stand_test + port /tmp/opencr, Isaac cameras through camera_adapter.py, use_sim_time on every node). ROS_DOMAIN_ID is
#   taken from the running isaac_sim.sh so both sides match. isaac/scripts/standalone/reset.sh brings back the full bringup —
#   run this again after a reset.
#
# Starts clean (skip with STAND_NO_CLEAN=1): ros2_ws/stop.sh stops running ROS (launch · nodes · ros2 CLI) and the
# ros2 daemon and removes Fast DDS leftovers in /dev/shm — other ROS work on this machine stops too.
# Turns on the repo venv (.venv — YOLO ultralytics · torch) when there is one.
# Residual policies (arm Δ) are switched only per skill in ros2_ws/model/skills/<skill>/meta.yaml (use: false) + colcon build —
# learning_enabled:=false does not turn them off (it only reaches grasp_scorer, which is rule-based either way).
WS="$(cd "$(dirname "$0")/../../../ros2_ws" && pwd)"
STANDALONE="$(cd "$(dirname "$0")/../standalone" && pwd)"
USE_SIM=0
ARGS=()
for a in "$@"; do
  case "$a" in
    use_sim:=true|use_sim:=1) USE_SIM=1 ;;
    use_sim:=*) ;;
    *) ARGS+=("$a") ;;
  esac
done
set -- "${ARGS[@]}"
if [ "$USE_SIM" = 1 ]; then
  SIM_PID=$(cat /tmp/beebot_isaac_sim.pid 2>/dev/null)
  if [ -z "$SIM_PID" ] || ! kill -0 "$SIM_PID" 2>/dev/null; then
    echo "use_sim: isaac_sim.sh is not running — start isaac/scripts/standalone/isaac_sim.sh first" >&2
    exit 1
  fi
  SIM_DOMAIN=$(tr '\0' '\n' < "/proc/$SIM_PID/environ" | sed -n 's/^ROS_DOMAIN_ID=//p')
  export ROS_DOMAIN_ID="${SIM_DOMAIN:-0}"  # same domain as Isaac's ROS bridge (unset there = 0)
  # Isaac's joint twist on/off (opencr_bridge.py): the launch sets twist_state to match (isaac_manipulator_demo.launch.py)
  SIM_TWIST=$(tr '\0' '\n' < "/proc/$SIM_PID/environ" | sed -n 's/^OPENCR_TWIST_SCALE=//p')
  [ -n "$SIM_TWIST" ] && export OPENCR_TWIST_SCALE="$SIM_TWIST"
  echo "use_sim: isaac_sim.sh pid $SIM_PID, ROS_DOMAIN_ID=$ROS_DOMAIN_ID — waiting for Play (/tmp/opencr)"
  until [ -e /tmp/opencr ]; do
    kill -0 "$SIM_PID" 2>/dev/null || { echo "use_sim: isaac_sim.sh exited" >&2; exit 1; }
    sleep 1
  done
fi
VENV="$WS/../../.venv/bin/activate"
if [ -z "${STAND_NO_CLEAN:-}" ]; then
  "$WS/stop.sh"
  # the full isaac_system bringup also runs camera_adapter.py as a plain python3 script — stop.sh does not match it,
  # and a second adapter would publish every camera topic twice ([c] keeps pkill off this shell's own command line)
  [ "$USE_SIM" = 1 ] && pkill -INT -f "[c]amera_adapter.py" && sleep 1
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
if [ "$USE_SIM" = 1 ]; then
  # start.sh checks the real OpenCR / camera serials — not for the sim; launch directly
  # shellcheck disable=SC1091
  source /opt/ros/lyrical/setup.bash
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
  exec ros2 launch "$STANDALONE/isaac_manipulator_demo.launch.py" "$@"
fi
LAUNCH_FILE=stand_test.launch.py exec "$WS/start.sh" "$@"
