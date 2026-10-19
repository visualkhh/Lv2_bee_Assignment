#!/bin/bash
# monitor_manager만 실행 — 보이는 토픽의 최신 메시지를 <debug>/topic/<토픽>/ 에 저장 (시뮬레이터 상태·영상이 읽음)
#
#   ./monitor.sh                      # 저장: 이 스크립트 위치의 위 debug/ (저장소: lv2_module6/debug), 원본 영상 제외
#   ./monitor.sh -D 8                 # ROS_DOMAIN_ID 지정 (Pi와 같은 값, --domain 8 도 같음)
#   ./monitor.sh -d ~/lv2_debug       # debug 폴더 지정 (그 아래 topic/에 저장)
#   ./monitor.sh --raw                # 원본 영상(sensor_msgs/Image)도 저장 (같은 컴퓨터에서만 권장)
#   ./monitor.sh -x /pole_cam/color/camera_info -x /tf   # 토픽 제외 (여러 번)
#
# 옵션
#   -d <폴더>   debug 폴더 (기본: <이 스크립트 폴더>/../debug). 없으면 만든다
#   -D, --domain <ID>  ROS_DOMAIN_ID (우선순위: 이 옵션 > 환경변수 ROS_DOMAIN_ID > 0)
#   -p <초>     이미지 저장 간격 period_s (기본 0.2)
#   -x <토픽>   제외할 토픽 (여러 번)
#   --raw       원본 영상 저장 (기본 꺼짐 — Wi-Fi로 받으면 약 27MB/s라 Pi 추적에 영향)
#
# 시작 전에 ros2 daemon stop으로 토픽 목록 캐시를 비운다 (다른 도메인·예전 노드 정보가 남아 있으면 헷갈림).
# 확인: 시작 로그에 저장 경로·ROS_DOMAIN_ID, 5초마다 "토픽 N개 구독 중, 최근 5초 저장 M건".
#       "구독할 토픽 없음"이 계속 나오면 ROS_DOMAIN_ID·네트워크 확인 (DEVELOP.md 문제 해결).
# 시뮬레이터와 함께: cd ../simulator && .venv/bin/python main.py
# NOTE: set -u 사용 금지 — ROS setup.bash가 미설정 변수를 참조해서 죽음.
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-lyrical}"

usage() { awk 'NR > 1 && (/^# NOTE/ || !/^#/) {exit} NR > 1' "$0"; }   # 맨 위 주석 = 도움말

debug_dir=''
domain=''
period=0.2
raw=false
excludes=()
while (($#)); do
  case "$1" in
    -d) debug_dir=$2; shift 2 ;;
    -D | --domain) domain=$2; shift 2 ;;
    -p) period=$2; shift 2 ;;
    -x) excludes+=("$2"); shift 2 ;;
    --raw) raw=true; shift ;;
    -h | --help) usage; exit 0 ;;
    *) echo "모르는 옵션: $1"; usage; exit 2 ;;
  esac
done

# sudo는 환경변수(ROS_DOMAIN_ID 등)를 지우고, 저장 파일을 root 소유로 만든다 → 시뮬레이터가 못 씀
if [ "$(id -u)" = 0 ] && [ -n "${SUDO_USER:-}" ]; then
  echo "WARN: sudo로 실행됨 — 셸의 ROS_DOMAIN_ID가 전달되지 않고, debug/ 파일이 root 소유가 됨"
  echo "      sudo 없이 실행 권장. 폴더 권한 문제면: sudo chown -R $SUDO_USER:$SUDO_USER <debug 폴더>"
  [ -n "$domain" ] || echo "      (도메인을 꼭 지정하려면 -D <ID>)"
fi

# ROS_DOMAIN_ID: 옵션 > 환경변수 > 0
if [ -n "$domain" ]; then
  domain_from="옵션"
elif [ -n "${ROS_DOMAIN_ID:-}" ]; then
  domain=$ROS_DOMAIN_ID
  domain_from="환경변수"
else
  domain=0
  domain_from="기본값"
fi
[[ $domain =~ ^[0-9]+$ ]] || { echo "ROS_DOMAIN_ID는 숫자: $domain"; exit 2; }
export ROS_DOMAIN_ID=$domain

# 기본: 스크립트 위치(ros2_ws)의 위 debug/ → 저장소에선 lv2_module6/debug (시뮬레이터가 읽는 위치)
debug_dir=${debug_dir:-"$HERE/../debug"}
mkdir -p "$debug_dir"
debug_dir="$(cd "$debug_dir" && pwd)"

# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO_NAME/setup.bash"
[ -f "$HERE/install/setup.bash" ] || { echo "FAIL: install/ 없음 — colcon build --packages-select faker"; exit 1; }
# shellcheck disable=SC1091
source "$HERE/install/setup.bash"
if ! ros2 pkg prefix faker >/dev/null 2>&1; then
  echo "FAIL: faker 패키지가 빌드되지 않음"
  echo "      sudo apt install ros-lyrical-rosx-introspection"
  echo "      colcon build --packages-select faker"
  exit 1
fi

# 데몬 캐시 비우기 — ros2 daemon이 예전 도메인·이미 꺼진 노드의 토픽 목록을 들고 있으면 헷갈린다
ros2 daemon stop >/dev/null 2>&1 || true

args=(-p "debug_dir:=$debug_dir" -p "period_s:=$period" -p "raw_images:=$raw")
if [ "${#excludes[@]}" -gt 0 ]; then
  list=$(printf "'%s'," "${excludes[@]}")
  args+=(-p "exclude_topics:=[${list%,}]")
fi

echo "=== monitor_manager | 저장: $debug_dir/topic | ROS_DOMAIN_ID=$domain ($domain_from) | raw_images=$raw"
exec ros2 run faker monitor_manager --ros-args "${args[@]}"
