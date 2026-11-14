#!/usr/bin/env bash
# 미션 한 회차 녹화 (현장 진단) — ros2 bag record · MCAP · zstd_fast · 크기로 나눔 (기본 1 GB)
#   ./record_run.sh [미션 회차 폴더]   폴더(예 ~/.ros/mission/20261114_101500)를 주면 그 안 bag/에, 안 주면 ~/.ros/bee_bags/<시각>/
#   ./record_run.sh --regex            녹화하는 토픽 정규식만 찍음 (TUI 미션 메뉴 'x'가 이걸 읽어 같은 목록으로 녹화)
#   끝내기: Ctrl+C (SIGINT — metadata.yaml을 씀). 나누는 크기: BAG_SPLIT_BYTES=500000000 ./record_run.sh
# 토픽: 미션·skill·적재함·인지 결과·진단·TF·odom·속도 명령·관절·충돌 감시·Nav2 경로/진행·/rosout
#   영상은 뺌 — 압축(compressed) 토픽이 없고 원본은 초당 수십 MB (/perception/<cam>/debug_image는 구독자가 있을 때만 그려서 부하도 늘림)
#   영상이 필요하면 tools/field-test.py --camera 또는 tools/bag/record.sh
set -euo pipefail
REGEX='^/(mission/(state|signal|signal_status|qr|markers)|task/.*|coordinator/basket|cmd/.*|tracked_objects|skill_context|arm_safety_event|motor_status|diagnostics_agg|rosout|tf|tf_static|odom|odometry/filtered|cuvslam/odom|cmd_vel.*|joint_states|collision_monitor_state|plan|local_plan|navigate_to_pose/_action/(feedback|status)|controller_server/FollowPath/status|skills/[a-z_]+/_action/(feedback|status))$'
if [ "${1:-}" = "--regex" ]; then echo "$REGEX"; exit 0; fi
if [ -n "${1:-}" ]; then OUT=$1/bag; else OUT=$HOME/.ros/bee_bags/$(date +%Y%m%d_%H%M%S); fi
mkdir -p "$(dirname "$OUT")"
echo "녹화: $OUT (Ctrl+C로 끝냄)"
exec ros2 bag record -s mcap --storage-preset-profile zstd_fast --max-bag-size "${BAG_SPLIT_BYTES:-1000000000}" \
  --regex "$REGEX" --include-hidden-topics --disable-keyboard-controls -o "$OUT"
