#!/usr/bin/env bash
# urdf.xacro → beebot.urdf (Isaac Sim URDF Importer용).
# 메시 경로를 상대 경로 meshes/ 로 넘겨 ROS 없이도 메시를 찾게 한다.
# 사용: ./make_urdf.sh   (xacro를 고친 뒤 다시 실행)
[ -n "${ROS_DISTRO:-}" ] || source /opt/ros/lyrical/setup.bash
set -euo pipefail
cd "$(dirname "$0")/.."
xacro urdf.xacro mesh_uri:=meshes > beebot.urdf
check_urdf beebot.urdf >/dev/null
echo "생성: $(pwd)/beebot.urdf"
