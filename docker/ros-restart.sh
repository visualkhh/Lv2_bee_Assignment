#!/bin/bash
# ROS 재시작 — 진짜 bringup (컨테이너 안에서): ros-restart [launch 인자]
#   ros-kill(끄기 + 캐시 정리) → build-ws → ros2 launch bringup bringup.launch.py [인자]  (🛠 노드 + ros_system 📦, Ctrl+C로 종료)
#   인자 예: use_ros_system:=false (📦 없이 🛠 노드만 — 컨테이너엔 RealSense·OpenCR가 없음) · mock_hardware:=true
#   대역(faker)까지 띄우는 테스트베드는 ros-faker-restart (test-faker 0)
# NOTE: set -u 사용 금지 — setup.bash가 미설정 변수를 참조해서 죽음.
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
cd /ws || { echo "FAIL: /ws 없음"; exit 2; }
ros-kill                 # 끄기 + 캐시 정리 — docker/ros-kill.sh
build-ws || exit 1       # 빌드 + install → /debug/install (docker/build-ws.sh)
# shellcheck disable=SC1091
source install/setup.bash
echo "=== ros2 launch bringup bringup.launch.py $*  (로그도 /tmp/bringup.log)"
exec > >(tee /tmp/bringup.log) 2>&1
exec ros2 launch bringup bringup.launch.py "$@"
