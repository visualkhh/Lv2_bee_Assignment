#!/bin/bash
# 전체 검사 (컨테이너 안에서 실행): test-all — push 전 로컬 게이트, CI도 이것만 돌린다.
#   serial(가상 시리얼) → firmware(OpenCR 컴파일·업로드) → packages(패키지별 colcon test)
#   → hardware(BeebotSystemHardware ↔ 시리얼) → bringup(🛠 노드 기동) → faker 정상 흐름 → faker 파지 실패·재계획 흐름
# 하나라도 FAIL이면 즉시 중단 (exit 1).
run_one() {
  echo "########## $* ##########"
  "$@" || exit 1
}

run_one test-serial
run_one test-firmware
run_one test-packages
run_one test-hardware
run_one test-bringup
run_one test-faker 1
run_one test-faker 1

echo "=========================="
echo "ALL PASS"
