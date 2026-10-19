#!/bin/bash
# 전체 검사 (컨테이너 안에서 실행): test-all — push 전 로컬 게이트, CI도 이것만 돌린다.
#   serial(가상 시리얼) → firmware(OpenCR 컴파일·업로드) → packages(패키지별 colcon test)
#   → hardware(BeebotSystemHardware ↔ 시리얼) → bringup(🛠 노드 기동) → faker 정상 흐름 → faker 파지 실패·재계획 흐름
# 하나라도 FAIL이면 즉시 중단 (exit 1).
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi   # sh(dash)로 실행해도 bash로 재실행
run_one() {
  echo "########## $* ##########"
  "$@" || exit 1
}

BIN_DIR=$(dirname "${BASH_SOURCE[0]}")
[ "$BIN_DIR" = . ] && BIN_DIR="$PWD"   # 호스트에서 docker/ 안이어도 형제 스크립트를 찾게 (컨테이너는 /usr/local/bin 그대로)
bin_of() {  # 컨테이너는 확장자 없음, 레포는 .sh — 있는 쪽 반환, 둘 다 없으면 PATH에 기대고 bare 이름
  [ -x "$BIN_DIR/$1" ] && echo "$BIN_DIR/$1" && return 0
  [ -x "$BIN_DIR/$1.sh" ] && echo "$BIN_DIR/$1.sh" && return 0
  echo "$1"
}

run_one "$(bin_of test-serial)"
run_one "$(bin_of test-firmware)"
run_one "$(bin_of test-packages)"
run_one "$(bin_of test-hardware)"
run_one "$(bin_of test-bringup)"
run_one "$(bin_of test-faker)" 1
run_one "$(bin_of test-faker)" 1

echo "=========================="
echo "ALL PASS"
