#!/bin/bash
# 최초 1회 설치 (호스트): MoveIt RViz 플러그인 + pick_ik(소스 빌드). sudo 암호를 묻습니다.
#   lv2_module6/develop/moveit/start.sh가 "없음"이라고 하면 이 스크립트를 한 번 실행하십시오.
# NOTE: sh(dash)로 실행해도 bash로 재실행
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi
if [ "$(id -u)" -eq 0 ]; then
  echo "sudo로 직접 실행하지 마십시오 — 일반 사용자로 실행하면 apt/cp만 내부에서 sudo로 실행합니다."
  echo "  lv2_module6/develop/moveit/setup.sh"
  exit 2
fi
# setup.bash는 set -u에서 AMENT_TRACE_SETUP_FILES를 직접 참조하므로 -u를 잠시 끄고 source한다
set +eu
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash || { echo "FAIL: /opt/ros/lyrical/setup.bash"; exit 1; }
set -euo pipefail

if ! command -v colcon >/dev/null 2>&1; then
  echo "FAIL: colcon 없음 — sudo apt install -y python3-colcon-common-extensions"
  exit 1
fi

sudo -v

# Dockerfile의 MoveIt 블록 + pick_ik 빌드 의존성 (DEVELOP.md §3)
sudo apt-get update
sudo apt-get install -y \
  ros-lyrical-moveit-ros-visualization \
  ros-lyrical-rsl \
  ros-lyrical-generate-parameter-library \
  ros-lyrical-moveit-core \
  ros-lyrical-tf2-kdl \
  librange-v3-dev libfmt-dev libexpected-dev

if [ -d /opt/ros/lyrical/share/pick_ik ]; then
  echo "이미 설치: pick_ik"
else
  TMP=$(mktemp -d)
  trap 'rm -rf "$TMP"' EXIT
  echo "pick_ik 빌드: $TMP"
  git clone https://github.com/PickNikRobotics/pick_ik.git "$TMP/src/pick_ik"
  git -C "$TMP/src/pick_ik" checkout 7c97fe2
  (cd "$TMP" && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF)
  sudo cp -r "$TMP/install/pick_ik/." /opt/ros/lyrical/
fi

echo "확인:"
ros2 pkg prefix moveit_ros_visualization
ros2 pkg prefix pick_ik
echo "완료 — faker를 재시작한 뒤(fresh move_group) develop/moveit/start.sh"
