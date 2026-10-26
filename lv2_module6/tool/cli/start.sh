#!/usr/bin/env bash
# tool/cli 시작 — ROS 2와 이 워크스페이스 설치를 source한 뒤 Python CLI로 넘긴다.
# set -u 금지 — ROS setup.bash가 미설정 변수를 참조한다 (start.sh와 같은 이유)
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/../../ros2_ws" 2>/dev/null && pwd || true)"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME:-lyrical}/setup.bash"

if [ ! -f "$ROS_SETUP" ]; then
  echo "FAIL: ROS 2 setup 없음: $ROS_SETUP" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "$ROS_SETUP"

if [ -n "$WS" ] && [ -f "$WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
else
  echo "WARN: ros2_ws/install/setup.bash 없음 — 먼저 ros2_ws에서 colcon build" >&2
fi

if [ -n "${ROS_DOMAIN_ID:-}" ]; then
  echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
else
  echo "ROS_DOMAIN_ID 미설정 → 기본값 0 사용"
fi

exec python3 -u "$HERE/operator_tool.py" "$@"
