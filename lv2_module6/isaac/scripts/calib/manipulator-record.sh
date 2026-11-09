#!/usr/bin/env bash
# Manipulator-only rosbag (MCAP) for arm tests in this folder (pick_loop, stand_loop ...): arm / gripper / skills /
# perception / hand camera residual + colour images of both cameras. No driving topics (cmd_vel, odom, Nav2, map) and no
# depth images — the general recorder with everything is tool/bag/record.sh.
#
#   ./manipulator-record.sh            # -> logs/bag_<date_time>/ (logs/ is git-ignored)
#   ./manipulator-record.sh nocam      # same without the colour images (much smaller)
#   ./manipulator-record.sh motion     # arm motion only: joints / controllers / RL correction / safety / skill stages
#                                      # (no perception, task, coordinator, rosout, images; place_loop.py uses this)
#   BAG_OUT=<dir> BAG_NODE=<name> ./manipulator-record.sh   # pick_loop.py uses this for one bag per round
#   BAG_SIM=1 ./manipulator-record.sh                     # Isaac Sim (isaac_sim.sh): stamp the bag with /clock
# Run while ./manipulator-launcher.sh is up; Ctrl-C to stop. View: PlotJuggler / Lichtblick (open the .mcap).
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/../../../ros2_ws/install/setup.bash"  # before set -u: colcon setup reads unset vars (COLCON_TRACE)
set -euo pipefail
OUT="${BAG_OUT:-$HERE/logs/bag_$(date +%m%d_%H%M%S)}"
mkdir -p "$HERE/logs"
TOPICS='^/(joint_states|tf|tf_static|arm_controller/.*|gripper_controller/.*|rl_arm_correction/debug|skill_context|hand_relative.*|arm_safety_event|motor_status|skills/.*|task/.*|coordinator/.*|perception/.*detections|tracked_objects|rosout)$'
[ "${1:-}" = motion ] && TOPICS='^/(joint_states|tf|tf_static|arm_controller/.*|gripper_controller/.*|rl_arm_correction/debug|skill_context|arm_safety_event|motor_status|skills/.*)$'
[ "${1:-}" = nocam ] || [ "${1:-}" = motion ] || TOPICS="$TOPICS|^/(gripper_cam|pole_front_cam)/color/image_raw/compressed$"
SIM=(); [ -n "${BAG_SIM:-}" ] && SIM=(--use-sim-time)
exec ros2 bag record --node-name "${BAG_NODE:-manipulator_record}" --disable-keyboard-controls "${SIM[@]}" \
  --regex "$TOPICS" --include-hidden-topics -s mcap --storage-preset-profile zstd_fast \
  --max-bag-duration 300 -o "$OUT"
