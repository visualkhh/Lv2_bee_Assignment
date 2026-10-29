#!/usr/bin/env bash
# tool/rviz 시작 — 카메라/점구름을 Best Effort + queue 1로 구독한다.
# Jetson에서 실행하면 로컬, PC에서 실행하면 네트워크 너머의 Jetson을 본다.
#   ./start.sh
#   ROS_DOMAIN_ID=42 ./start.sh
#   RVIZ_CONFIG=mission ./start.sh      # 주사위 미션 (map 고정 · L1~L3 칸 마커 /mission/markers)
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
  # RViz 자체는 커스텀 메시지를 쓰지 않지만, 같은 환경에서 띄울 때 맞춘다.
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
fi

if ! command -v rviz2 >/dev/null; then
  echo "FAIL: rviz2 없음 — ROS 2 RViz를 설치하세요" >&2
  exit 1
fi

if [ -n "${ROS_DOMAIN_ID:-}" ]; then
  echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
else
  echo "ROS_DOMAIN_ID 미설정 → 기본값 0 사용"
fi

if [ "${ROS_LOCALHOST_ONLY:-0}" = "1" ]; then
  echo "WARN: ROS_LOCALHOST_ONLY=1 — 다른 머신과 통신 불가 (원격이면 0으로)" >&2
fi

exec rviz2 -d "$HERE/${RVIZ_CONFIG:-operator}.rviz" "$@"
