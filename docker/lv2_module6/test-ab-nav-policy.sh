#!/bin/bash
# Nav2 주행 정책(rl_nav_controller) A/B 시험 (컨테이너 안에서 실행): test-ab-nav-policy [launch 인자...]
#   faker를 띄우고(test-faker 0) → run-cli A/B guide를 그대로 부른다. 이 스크립트는 기동·대기·정리만 한다.
#   점검·주행·판정·리포트는 전부 run-cli가 한다 (tools/cli/op/pages/ab/README.md):
#     사전 점검(Nav2 수명주기 · /map · 경로 계획) → A(정책)·B(DWB)를 같은 출발점에서 command.json route대로 주행
#     → leg마다 result.json(모드가 맞게 돌았나) → report.png · compare.png
#   PASS = guide 종료 코드 0 = 모든 leg가 유효(측정이 맞게 됨). 정책이 목표에 도착했는지는 리포트로 본다.
#   goal 위치: tools/cli/op/pages/ab/rl_nav_controller/command.json (다른 파일은 NAV_AB_CONFIG=<경로>)
#   예) test-ab-nav-policy                       # Isaac 학습 미로 (course:=isaac_maze)
#       test-ab-nav-policy course:=dice_maze
# NOTE: set -u 사용 금지 — setup.bash가 미설정 변수를 참조해서 죽음.
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi   # sh(dash)로 실행해도 bash로 재실행
source /opt/ros/lyrical/setup.bash
BIN_DIR=$(dirname "${BASH_SOURCE[0]}")
[ "$BIN_DIR" = . ] && BIN_DIR="$PWD"   # 컨테이너(/usr/local/bin)·호스트(docker/lv2_module6/) 모두 동작
bin_of() {  # 컨테이너는 확장자 없음, 레포는 .sh — 있는 쪽 반환, 둘 다 없으면 PATH에 기대고 bare 이름
  [ -x "$BIN_DIR/$1" ] && echo "$BIN_DIR/$1" && return 0
  [ -x "$BIN_DIR/$1.sh" ] && echo "$BIN_DIR/$1.sh" && return 0
  echo "$1"
}
if [ -z "$WS" ]; then
  if [ -d /ws/src ]; then WS=/ws; else WS="$BIN_DIR/../../lv2_module6/ros2_ws"; fi   # 컨테이너면 /ws, 호스트면 레포 안 ros2_ws
fi
WS=$(realpath -m "$WS")
TOOL="${TOOL:-$WS/tools}"   # 컨테이너 /ws/tools (레포 lv2_module6/tools 바인드)
[ -x "$TOOL/run-cli" ] || TOOL="$WS/../tools"
TOOL=$(realpath -m "$TOOL")
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}
export FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE:-$WS/fastdds.xml}

case " $* " in
  *\ course:=*) : ;;
  *) set -- "$@" "course:=isaac_maze" ;;   # 정책을 학습한 Isaac 미로
esac
case " $* " in
  *\ use_monitor:=*) : ;;
  *) set -- "$@" "use_monitor:=false" ;;   # 호스트 시뮬레이터 없이 (fake_camera·fake_nvblox가 코스 지도로 센서를 냄)
esac
# 묵은 시뮬레이터 프레임 치우기 — .json이 있으면 fake_camera가 "한 번 받고 끊김"으로 보고 코스 지도 depth를 안 낸다.
CAM_DIR="${CAM_DIR:-/debug/input/hardware/camera}"
[ -d "$CAM_DIR" ] && rm -f "$CAM_DIR"/*.json

# 기동·정리는 test-faker 0 (빌드 · mock 판단 · spawner 재시도 포함) — 끝나면 같이 내린다
"$(bin_of test-faker)" 0 "$@" > /tmp/ab-nav-policy.log 2>&1 & FAKER_PID=$!
stop_faker() {
  # 백그라운드 자식은 SIGINT를 무시한 채 시작하므로(비대화형 셸) TERM → 안 끝나면 ros-kill로 노드·캐시 정리
  kill -TERM "$FAKER_PID" 2>/dev/null
  for _ in $(seq 10); do kill -0 "$FAKER_PID" 2>/dev/null || break; sleep 1; done
  "$(bin_of ros-kill)" > /dev/null 2>&1
}
trap 'stop_faker' EXIT INT TERM

# Nav2·코스 지도가 active가 될 때까지 (빌드 포함 최대 5분) — 나머지 점검은 guide 사전 점검이 한다
echo "faker 기동 중 … (로그 /tmp/ab-nav-policy.log)"
READY=0
BT_AT=0
for i in $(seq 150); do
  kill -0 "$FAKER_PID" 2>/dev/null || { echo "FAIL: test-faker가 끝남"; tail -20 /tmp/ab-nav-policy.log; exit 1; }
  BT="$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null)"
  MS="$(timeout 5 ros2 lifecycle get /map_server 2>/dev/null)"
  case "$BT|$MS" in *"active [3]"*"|"*"active [3]"*) READY=1; break ;; esac
  # Nav2는 떴는데 map_server만 inactive로 남음 = lifecycle_manager_map이 기동 중 응답 시간 초과로 멈춘 것
  #   (실측: change_state 응답이 사라져 configure 뒤로 안 감) — 사람이 하던 'run-cli node activate'를 대신 부른다
  case "$BT" in *"active [3]"*) [ "$BT_AT" = 0 ] && BT_AT=$i ;; esac
  if [ "$BT_AT" != 0 ] && [ $((i - BT_AT)) -ge 15 ]; then
    echo "map_server가 active가 아님 ($MS) — run-cli node activate map_server"
    "$TOOL/run-cli" node activate map_server || true
    BT_AT=$i
  fi
  sleep 2
done
[ "$READY" = 1 ] || { echo "FAIL: bt_navigator·map_server가 active가 안 됨 — run-cli node list로 확인"; exit 1; }

LABEL="ci_$(date +%Y%m%d_%H%M%S)"
"$TOOL/run-cli" ab rl_nav_controller guide --label "$LABEL" ${NAV_AB_CONFIG:+--config "$NAV_AB_CONFIG"}
RC=$?
GROUP=$(ls -td /debug/tools/artifacts/ab/rl_nav_controller/*/ 2>/dev/null | head -1)
echo "=========================="
if [ "$RC" = 0 ]; then
  echo "PASS: ab-nav-policy — 리포트 ${GROUP}compare.png"
else
  echo "FAIL: ab-nav-policy (guide 종료 코드 $RC) — 위 사전 점검·leg 판정, ${GROUP:-산출물 없음}"
fi
exit "$RC"
