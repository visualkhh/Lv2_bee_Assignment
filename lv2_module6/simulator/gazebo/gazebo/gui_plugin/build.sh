#!/bin/bash
# ManualActors Gazebo GUI 플러그인 빌드
#   lv2_module6/develop/gazebo/gui_plugin/build.sh [--clean]
set -eo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/lyrical/setup.bash
cd "$HERE"
if [ "$1" = "--clean" ]; then
  rm -rf build
fi
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release >/tmp/gz_gui_plugin_cmake.log
cmake --build build -j"$(nproc)" | tee /tmp/gz_gui_plugin_build.log
echo "plugin: $HERE/build/libManualActors.so"
