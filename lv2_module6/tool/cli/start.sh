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

# nvblox/cuVSLAM overlay — 지도 저장 서비스(nvblox_msgs/srv/FilePath)를 CLI에서 쓰려면 필요
SLAM_WS="${SLAM_WS:-$HOME/isaac_ros_ws}"
if [ -f "$SLAM_WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$SLAM_WS/install/setup.bash"
fi

# bringup이 없을 때의 기본값 — Jetson 실기 로컬 전용 (SHM만, 네트워크 in/out 없음)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-$WS/fastdds_local_only.xml}"

# 실행 중인 bringup과 같은 ROS 도메인/DDS 프로파일을 쓴다.
#   ps에는 ROS 프로세스가 보이는데 `run-cli` 노드 그래프가 비어 보이는 대표 원인:
#   대화형 셸(~/.bashrc)은 ROS_DOMAIN_ID=88 · fastdds_local_only.xml(SHM만)인데,
#   bringup을 비대화형 셸에서 start하면 도메인 0 · fastdds.xml(UDP)로 뜰 수 있다.
#   bringup /proc 환경을 그대로 복사해 어느 셸에서 run-cli를 켜도 같은 그래프를 보게 한다.
_apply_bringup_env() {
  local pid var val
  pid="$(pgrep -f 'ros2 launch bringup bringup.launch.py' | head -n 1 || true)"
  [ -n "$pid" ] && [ -r "/proc/$pid/environ" ] || return 1
  for var in ROS_DOMAIN_ID ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE \
             ROS_STATIC_PEERS ROS_DISCOVERY_SERVER \
             FASTDDS_DEFAULT_PROFILES_FILE FASTRTPS_DEFAULT_PROFILES_FILE \
             RMW_IMPLEMENTATION; do
    val="$(tr '\0' '\n' < "/proc/$pid/environ" | sed -n "s/^${var}=//p" | head -n 1)"
    if [ -n "$val" ]; then
      export "$var=$val"
    else
      unset "$var"   # bringup에 없으면 기본값 — 셸의 다른 값이 끼어들지 않게
    fi
  done
  echo "bringup(pid $pid) ROS 환경 적용: domain ${ROS_DOMAIN_ID:-0} · profile ${FASTDDS_DEFAULT_PROFILES_FILE:-기본}"
}
_apply_bringup_env || true

# 로컬 전용 프로파일이면 외부 discovery 변수를 지운다 (실수로 네트워크로 새는 것 방지)
case "$(basename "${FASTDDS_DEFAULT_PROFILES_FILE:-}")" in
  fastdds_local_only.xml)
    unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER ROS_LOCALHOST_ONLY
    ;;
esac

if [ -n "${ROS_DOMAIN_ID:-}" ]; then
  echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
else
  echo "ROS_DOMAIN_ID 미설정 → 기본값 0 사용"
fi

exec python3 -u "$HERE/operator_tool.py" "$@"
