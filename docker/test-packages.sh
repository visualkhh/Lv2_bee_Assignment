#!/bin/bash
# 패키지 테스트 (컨테이너 안에서 실행): test-packages [패키지...]
#   colcon build → colcon test → 결과 요약. 패키지를 주면 그것만 (예: test-packages plan control)
#   각 패키지의 테스트는 <패키지>/test/ 와 CMakeLists.txt의 if(BUILD_TESTING) 블록 (lv2_module6/DEVELOP.md 참고)
#   결과 파일: $WS/build/<패키지>/test_results/
# NOTE: set -u 사용 금지 — setup.bash가 미설정 변수를 참조해서 죽음.
# shellcheck disable=SC1091
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi   # sh(dash)로 실행해도 bash로 재실행
source /opt/ros/lyrical/setup.bash
BIN_DIR=$(dirname "${BASH_SOURCE[0]}")
[ "$BIN_DIR" = . ] && BIN_DIR="$PWD"   # 형제 스크립트 탐색용 — 컨테이너(/usr/local/bin)·호스트(docker/) 모두 동작
bin_of() {  # 컨테이너는 확장자 없음, 레포는 .sh — 있는 쪽 반환, 둘 다 없으면 PATH에 기대고 bare 이름
  [ -x "$BIN_DIR/$1" ] && echo "$BIN_DIR/$1" && return 0
  [ -x "$BIN_DIR/$1.sh" ] && echo "$BIN_DIR/$1.sh" && return 0
  echo "$1"
}
if [ -z "$WS" ]; then
  if [ -d /ws/src ]; then WS=/ws; else WS="$BIN_DIR/../lv2_module6/ros2_ws"; fi   # 컨테이너면 /ws, 호스트면 레포 안 ros2_ws (머신 경로 하드코딩 없음)
fi
WS=$(realpath -m "$WS")   # ../ 정리 — CMakeCache 절대경로와 문자열 비교하므로 정규화 필수
cd "$WS" || { echo "FAIL: $WS 없음"; exit 2; }

SELECT=()
[ $# -gt 0 ] && SELECT=(--packages-select "$@")

"$(bin_of build-ws)" "${SELECT[@]}" || exit 1   # 빌드 + install → /debug/install (docker/build-ws.sh)
# 패키지 테스트끼리 토픽이 섞이지 않게 순서대로 (gtest마다 ROS_DOMAIN_ID도 따로 둠)
colcon test --executor sequential --event-handlers console_direct- "${SELECT[@]}"
echo "--- 결과 ---"
if colcon test-result --verbose; then
  echo "PASS: packages"
else
  echo "FAIL: packages (위 실패 목록 · $WS/build/<패키지>/test_results/)"
  exit 1
fi
