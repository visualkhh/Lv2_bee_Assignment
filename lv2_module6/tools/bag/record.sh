#!/usr/bin/env bash
# 실기 실험 기록 (rosbag2 · MCAP) — 로봇(또는 같은 ROS_DOMAIN_ID의 PC)에서
#   ./record.sh blackbox [초]   최근 N초(기본 60)만 메모리에 → 실패 때 snap_on_fail.py가 그 구간만 파일로 (늘 켜 둠)
#   ./record.sh full            전부 파일로 (짧은 실험 · 재현할 때)
#   ./record.sh snap            지금 블랙박스 구간을 손으로 저장
# 저장: ${BAG_DIR:-~/bee_bags}/<모드>_<날짜시각>/ — 보기: PlotJuggler · Lichtblick (MCAP 파일 열기)
set -euo pipefail
MODE=${1:-blackbox}
DIR=${BAG_DIR:-$HOME/bee_bags}
NODE=bee_blackbox
mkdir -p "$DIR"
# 팔 · 주행 · 인지 · 미션 · 상태 (영상 원본은 뺌 — 압축 영상·검출만)
TOPICS='^/(joint_states|tf|tf_static|clock|odom.*|cmd_vel.*|arm_controller/.*|gripper_controller/.*|rl_arm_correction/debug|skill_context|hand_relative.*|arm_safety_event|motor_status|state/.*|mission/.*|coordinator/.*|task/.*|cmd/.*|perception/.*detections|tracked_objects|diagnostics_agg|diagnostics_toplevel_state|rosout|plan|local_plan|map)$|compressed$'
SKIP='image_raw$|/points$|depth/image'
case "$MODE" in
  blackbox)
    SEC=${2:-60}
    exec ros2 bag record --node-name "$NODE" --snapshot-mode --max-cache-duration "$SEC" --max-cache-size 400000000 \
      --regex "$TOPICS" --exclude-regex "$SKIP" --include-hidden-topics --disable-keyboard-controls \
      -s mcap -o "$DIR/blackbox_$(date +%m%d_%H%M%S)" ;;
  full)
    exec ros2 bag record --regex "$TOPICS" --exclude-regex "$SKIP" --include-hidden-topics \
      -s mcap --storage-preset-profile zstd_fast --max-bag-duration 300 -o "$DIR/full_$(date +%m%d_%H%M%S)" ;;
  snap)
    exec ros2 service call "/$NODE/snapshot" rosbag2_interfaces/srv/Snapshot ;;
  *) sed -n 2,6p "$0"; exit 1 ;;
esac
