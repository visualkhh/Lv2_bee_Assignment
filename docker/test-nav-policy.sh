#!/bin/bash
# Nav2 주행 정책(rl_nav_controller ONNX) faker 검증 (컨테이너 안에서 실행): test-nav-policy
#   faker launch (course 맵 · 진짜 Nav2) → ONNX 로드 확인 → toggle off/on →
#   제자리 근처 goal로 DWB·정책 주행 → /cmd_vel_nav · FollowPath/debug 확인
#   예) test-nav-policy
# NOTE: set -u 사용 금지 — setup.bash가 미설정 변수를 참조해서 죽음.
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi   # sh(dash)로 실행해도 bash로 재실행
SMOKE=0
if [ "${1:-}" = smoke ]; then SMOKE=1; shift; fi   # CI용: 배선 확인까지만, goal 완주 대기 안 함
source /opt/ros/lyrical/setup.bash
BIN_DIR=$(dirname "${BASH_SOURCE[0]}")
[ "$BIN_DIR" = . ] && BIN_DIR="$PWD"   # 컨테이너(/usr/local/bin)·호스트(docker/) 모두 동작
bin_of() {  # 컨테이너는 확장자 없음, 레포는 .sh — 있는 쪽 반환, 둘 다 없으면 PATH에 기대고 bare 이름
  [ -x "$BIN_DIR/$1" ] && echo "$BIN_DIR/$1" && return 0
  [ -x "$BIN_DIR/$1.sh" ] && echo "$BIN_DIR/$1.sh" && return 0
  echo "$1"
}
if [ -z "$WS" ]; then
  if [ -d /ws/src ]; then WS=/ws; else WS="$BIN_DIR/../lv2_module6/ros2_ws"; fi   # 컨테이너면 /ws, 호스트면 레포 안 ros2_ws (머신 경로 하드코딩 없음)
fi
WS=$(realpath -m "$WS")   # ../ 정리
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}
export FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE:-$WS/fastdds.xml}
cd "$WS" || { echo "FAIL: $WS 없음"; exit 2; }

KILL_PAT="($WS/install|/opt/ros/lyrical)/[^ ]*lib/|ros2 launch"
kill_all() {
  local pids
  pids=$(pgrep -f "$KILL_PAT" 2>/dev/null) || return 0
  # shellcheck disable=SC2086
  [ -n "$pids" ] && kill $pids 2>/dev/null
  sleep 2
  pids=$(pgrep -f "$KILL_PAT" 2>/dev/null) || return 0
  # shellcheck disable=SC2086
  [ -n "$pids" ] && kill -KILL $pids 2>/dev/null
  return 0
}
trap 'kill_all' EXIT INT TERM

# 이전에 죽은 실행이 남긴 FastDDS SHM 조각 정리 (가득 차면 discovery·토픽이 멈춤 — 이 테스트 전용 머신이라 전체 삭제)
kill_all 2>/dev/null
rm -f /dev/shm/fastdds_* 2>/dev/null
# 오래된 ros2 데몬 캐시 제거 (stale graph로 서비스·토픽 못 찾는 경우 방지)
pkill -f "ros2-daemon" 2>/dev/null
sleep 2

"$(bin_of build-ws)" || exit 1   # 빌드 + install (모델 share/ml/model 복사 포함)
# shellcheck disable=SC1091
source install/setup.bash

set -- "$@" "use_monitor:=false"   # 모니터 끔 (가벼운 실행 · PASS 판정에서 기록 검사 제외)
case " $* " in
  *\ mock_hardware:=*) : ;;
  *) [ -e /dev/opencr ] || set -- "$@" "mock_hardware:=true" ;;   # 호스트엔 가상 시리얼이 없어서 mock으로
esac
case " $* " in
  *\ course:=*) : ;;
  *) set -- "$@" "course:=twin_closed_maze" ;;   # Nav2 정책 시험은 옛 미로 (기본값은 dice_maze)
esac
set -m
# 묵은 시뮬레이터 프레임 치우기 — .json이 있으면 fake_camera가 "한 번 받고 끊김"으로 판단해
# 더미 모드로 안 가고 영상·점구름이 안 나온다. headless 테스트는 프레임 없이 시작한다.
CAM_DIR="${CAM_DIR:-/debug/input/hardware/camera}"
[ -d "$CAM_DIR" ] && rm -f "$CAM_DIR"/*.json
ros2 launch faker faker.launch.py "$@" > /tmp/nav-policy.log 2>&1 & LAUNCH_PID=$!

# EKF 오도메트리 대기 (fake_opencr → /joint_states → EKF)
timeout 60 ros2 topic echo --once /odometry/filtered nav_msgs/msg/Odometry > /dev/null 2>&1 \
  || { echo "FAIL: /odometry/filtered 없음"; tail -20 /tmp/nav-policy.log; exit 1; }
# Nav2 서버 active 대기 — controller_server(정책 플러그인 주인) + planner_server(경로 계산) + bt_navigator
for _ in $(seq 90); do
  CS="$(timeout 5 ros2 lifecycle get /controller_server 2>/dev/null)"
  PS="$(timeout 5 ros2 lifecycle get /planner_server 2>/dev/null)"
  BT="$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null)"
  [ "$(printf '%s\n%s\n%s' "$CS" "$PS" "$BT" | grep -o 'active \[3\]' | wc -l)" = 3 ] && break
  sleep 1
done
[ "$(printf '%s\n%s\n%s' "$CS" "$PS" "$BT" | grep -o 'active \[3\]' | wc -l)" = 3 ] \
  || { echo "FAIL: Nav2 서버가 active 아님"; grep -E "lifecycle_manager|ERROR" /tmp/nav-policy.log | tail -10; exit 1; }
# ONNX 로드 확인 (모델 없으면 바이패스 → 정책 테스트 불가)
grep -q "ONNX loaded" /tmp/nav-policy.log \
  || { echo "FAIL: ONNX 미로드 — share/ml/model/rl_nav_controller/policy.onnx 확인"; grep -iE "onnx|policy" /tmp/nav-policy.log | tail -10; exit 1; }
echo "OK: ONNX loaded"
# 토글 서비스 확인
timeout 10 ros2 service list 2>/dev/null | grep -q "/controller_server/FollowPath/toggle" \
  || { echo "FAIL: FollowPath/toggle 서비스 없음"; exit 1; }

# 카메라: fake_camera가 시뮬레이터 프레임 또는 더미(회색 영상)를 낸다 — 흐름 확인.
# 더미 모드(depth 0)는 점구름을 일부러 안 낸다(장애물 없음 가정) — 정책은 위 ESDF를 본다.
timeout 15 ros2 topic echo --once /pole_front_cam/color/image_raw sensor_msgs/msg/Image > /dev/null 2>&1 \
  || { echo "FAIL: 카메라 영상 없음 (fake_camera 확인)"; exit 1; }
echo "카메라: 흐름 확인"
# 코스트맵 발행 확인 (planner가 "Costmap timed out" 내지 않게 — map + voxel 초기화 대기)
timeout 30 ros2 topic echo --once /local_costmap/costmap nav_msgs/msg/OccupancyGrid > /dev/null 2>&1 \
  || { echo "FAIL: local costmap 없음"; exit 1; }
timeout 30 ros2 topic echo --once /global_costmap/costmap nav_msgs/msg/OccupancyGrid > /dev/null 2>&1 \
  || { echo "FAIL: global costmap 없음"; exit 1; }
echo "코스트맵: 흐름 확인"
# discovery 안정화 대기 (기동 직후 서비스 호출이 타임아웃 나지 않게)
sleep 20
# ESDF: course 모드에선 fake_nvblox가 /nvblox_node/combined_esdf_pointcloud 발행 (빈 지도 가정).
# 없으면 sensor_ok=false → 정책이 한 번도 안 돌고 DWB 바이패스만 돈다 (테스트가 그걸 못 잡음).
# 예전엔 여기서 인라인 python 더미를 깔았으나 fake_nvblox로 일원화 — 안 뜨면 FAIL.
ESDF_TOPIC="${ESDF_TOPIC:-/nvblox_node/combined_esdf_pointcloud}"
if timeout 15 ros2 topic echo --once "$ESDF_TOPIC" sensor_msgs/msg/PointCloud2 > /dev/null 2>&1; then
  echo "ESDF: fake_nvblox 점구름 확인"
else
  echo "FAIL: ESDF 점구름 없음 ($ESDF_TOPIC) — fake_nvblox가 떴는지 확인"
  exit 1
fi

# run-cli ab guide로 A/B 전체 수행 (토글·녹화·주행·리포트 한 번에)
GUIDE_LABEL="ci_$(date +%Y%m%d_%H%M%S)"
python3 /ws/tool/cli/operator_tool.py ab rl_nav_controller guide --label "$GUIDE_LABEL" || { echo "FAIL: ab guide 실패"; exit 1; }
echo "OK: ab guide 완료 (A=정책, B=DWB)"

# 산출물 확인 (guide가 ab/<노드>/<시작시각>/{A,B}/ + compare.png 생성)
GROUP=$(ls -td /debug/tool/artifacts/ab/rl_nav_controller/*/ 2>/dev/null | head -1)
[ -n "$GROUP" ] || { echo "FAIL: A/B 그룹 폴더 없음"; exit 1; }
[ -f "$GROUP/compare.png" ] || { echo "FAIL: compare.png 없음 ($GROUP)"; exit 1; }
echo "비교 리포트: ${GROUP}compare.png"
for LEG in A B; do
  LEGDIR=$(ls -td "$GROUP"*_$LEG 2>/dev/null | head -1)
  [ -n "$LEGDIR" ] || { echo "FAIL: $LEG leg 폴더 없음"; exit 1; }
  [ -f "$LEGDIR/report.png" ] || { echo "FAIL: $LEG leg 리포트 없음"; exit 1; }
  [ -f "$LEGDIR/raw.csv" ] || { echo "FAIL: $LEG leg raw.csv 없음"; exit 1; }
  echo "leg $LEG: $LEGDIR (report.png + raw.csv)"
done

# 정책 leg bag에 FollowPath/debug가 있어야 실제 정책이 돈 것 (DWB 바이패스 아님)
ABAG=$(ls -td "$GROUP"*_A 2>/dev/null | head -1)
[ -n "$ABAG" ] || { echo "FAIL: 정책 bag 없음"; exit 1; }
DCNT=$(python3 - "$ABAG" <<'PYEOF'
import sys
from pathlib import Path
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
uri = sys.argv[1]
sid = "mcap" if list(Path(uri).glob("*.mcap")) else "sqlite3"
r = SequentialReader()
r.open(StorageOptions(uri=uri, storage_id=sid), ConverterOptions("", ""))
n = 0
while r.has_next():
    topic, _, _ = r.read_next()
    if "FollowPath/debug" in topic:
        n += 1
print(n)
PYEOF
)
[ "${DCNT:-0}" -gt 0 ] 2>/dev/null || { echo "FAIL: 정책 bag에 debug 없음 (바이패스 의심)"; exit 1; }
echo "OK: 정책 bag에 debug 토픽 $DCNT개 (정책 실제 동작)"

echo "=========================="
echo "PASS: nav-policy (run-cli ab guide · 리포트 · debug 확인)"
