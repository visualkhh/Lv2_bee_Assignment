#!/bin/bash
# 풀빌드 (컨테이너·호스트 공통): 이 폴더에서 실행하면 build/·install/·log/를 지우고 처음부터 빌드한다.
# build/ 안 CMakeCache는 절대경로라 컨테이너(/ws/…)와 호스트(~/…)가 공유 불가 — 옮겨서 빌드할 땐 여기로 리셋.
# 용법: ./rebuild.sh [--packages-select plan ...]   (인자는 colcon에 그대로 전달)
set -e
cd "$(dirname "$0")"
if [ -f /opt/ros/lyrical/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/lyrical/setup.bash
fi
rm -rf build install log
colcon build --symlink-install "$@"
