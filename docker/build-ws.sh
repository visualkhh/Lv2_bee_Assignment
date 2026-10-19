#!/bin/bash
# 워크스페이스 빌드 (컨테이너 안): build-ws [colcon build 인자...] — test-packages · test-hardware · test-bringup · test-faker 공용
#   1. 다른 경로(/ws 아닌 곳)에서 만든 빌드 캐시면 비우고 풀빌드 (CMakeCache가 절대경로라 깨짐 — 수동 리셋: ros2_ws/rebuild.sh)
#   2. colcon build --symlink-install
#   3. install을 호스트 공유 폴더 /debug/install 에 실파일로 복사 — install은 컨테이너 안이라 호스트에서 안 보이므로 (share 참고용)
#      (-L: symlink 풀어 실파일로, 수 MB)
# NOTE: set -u 사용 금지 — setup.bash가 미설정 변수를 참조해서 죽음.
# shellcheck disable=SC1091
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi   # sh(dash)로 실행해도 bash로 재실행
source /opt/ros/lyrical/setup.bash
BIN_DIR=$(dirname "${BASH_SOURCE[0]}")
[ "$BIN_DIR" = . ] && BIN_DIR="$PWD"   # 형제 스크립트 탐색용 — 컨테이너(/usr/local/bin)·호스트(docker/) 모두 동작
if [ -z "$WS" ]; then
  if [ -d /ws/src ]; then WS=/ws; else WS="$BIN_DIR/../lv2_module6/ros2_ws"; fi   # 컨테이너면 /ws, 호스트면 레포 안 ros2_ws (머신 경로 하드코딩 없음)
fi
WS=$(realpath -m "$WS")   # ../ 정리 — CMakeCache 절대경로와 문자열 비교하므로 정규화 필수
cd "$WS" || { echo "FAIL: $WS 없음"; exit 2; }
if ls build/*/CMakeCache.txt >/dev/null 2>&1 && \
   grep -rh 'CMAKE_HOME_DIRECTORY:INTERNAL=' build/*/CMakeCache.txt 2>/dev/null | grep -qv "^CMAKE_HOME_DIRECTORY:INTERNAL=$WS/src/"; then
  echo "외부 경로 빌드 캐시 감지 — 풀빌드로 리셋"
  rm -rf build install log
fi
colcon build --symlink-install --event-handlers console_direct- "$@" || { echo "FAIL: colcon build"; exit 1; }
if [ -d /debug ]; then  # 컨테이너만: 호스트 공유 폴더에 install 실파일 복사
  rm -rf /debug/install && cp -rL "$WS/install" /debug/install && echo "install → /debug/install (호스트: lv2_module6/debug/install)"
fi
