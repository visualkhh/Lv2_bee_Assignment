#!/bin/bash
# 패키지 테스트 (컨테이너 안에서 실행): test-packages [패키지...]
#   colcon build → colcon test → 결과 요약. 패키지를 주면 그것만 (예: test-packages plan control)
#   각 패키지의 테스트는 <패키지>/test/ 와 CMakeLists.txt의 if(BUILD_TESTING) 블록 (lv2_module6/DEVELOP.md 참고)
#   결과 파일: /ws/build/<패키지>/test_results/
# NOTE: set -u 사용 금지 — setup.bash가 미설정 변수를 참조해서 죽음.
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
cd /ws || { echo "FAIL: /ws 없음"; exit 2; }

SELECT=()
[ $# -gt 0 ] && SELECT=(--packages-select "$@")

build-ws "${SELECT[@]}" || exit 1   # 빌드 + install → /debug/install (docker/build-ws.sh)
# 패키지 테스트끼리 토픽이 섞이지 않게 순서대로 (gtest마다 ROS_DOMAIN_ID도 따로 둠)
colcon test --executor sequential --event-handlers console_direct- "${SELECT[@]}"
echo "--- 결과 ---"
if colcon test-result --verbose; then
  echo "PASS: packages"
else
  echo "FAIL: packages (위 실패 목록 · /ws/build/<패키지>/test_results/)"
  exit 1
fi
