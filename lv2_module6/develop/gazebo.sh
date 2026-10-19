#!/bin/bash
# Gazebo 실행 (호스트): 코스 월드 띄우기
#   develop/gazebo.sh [gz sim 인자...]
# NOTE: sh(dash)로 실행해도 bash로 재실행
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi
source /opt/ros/lyrical/setup.bash
: "${ROS_DOMAIN_ID:=8}"; export ROS_DOMAIN_ID   # ros_gz_bridge 같이 띄울 때 같은 값
HERE=$(dirname "${BASH_SOURCE[0]}")
[ "$HERE" = . ] && HERE="$PWD"
WORLD="$HERE/../course/worlds/twin_closed_maze.sdf"
# model://twin_closed_maze → course/twin_closed_maze (world 파일 주석 참고)
export GZ_SIM_RESOURCE_PATH="$HERE/../course${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
# NVIDIA EGL 직접 지정 — Mesa(dri2)가 NVIDIA 카드를 잡으려다 터짐 (ogre2 eglInitialize 크래시)
# 10_nvidia.json이 없으면(내장그래픽 등) 건드리지 않음. 소프트웨어 렌더링은 최후 수단: LIBGL_ALWAYS_SOFTWARE=1
if [ -z "$__EGL_VENDOR_LIBRARY_FILENAMES" ] && [ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ] && [ -z "$LIBGL_ALWAYS_SOFTWARE" ]; then
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
fi
command -v gz >/dev/null 2>&1 || { echo "FAIL: gz 없음 — sudo apt install -y ros-lyrical-ros-gz-sim"; exit 2; }
[ -f "$WORLD" ] || { echo "FAIL: 월드 없음: $WORLD"; exit 2; }
exec gz sim "$WORLD" "$@"
