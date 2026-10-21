#!/usr/bin/env bash
# beebot_mesh → default 배포: urdf.xacro 와 meshes/ 를 default/ 에 덮어쓴다.
# default/urdf.xacro 의 Gazebo 블록(<!-- Gazebo ... 부터 </robot> 앞까지)은 팀 Gazebo 시뮬레이션용이라 그대로 옮겨 붙인다.
# default/meshes/ 에만 있는 파일(예: maze.stl)은 지우지 않는다.
# 사용: ./deploy_default.sh   (beebot_mesh 를 고친 뒤 실행)
[ -n "${ROS_DISTRO:-}" ] || source /opt/ros/lyrical/setup.bash
set -euo pipefail
cd "$(dirname "$0")/../.."
src=beebot_mesh/urdf.xacro
dst=default/urdf.xacro

gz=$(sed -n '/<!-- Gazebo/,/<\/robot>/p' "$dst" | sed '/<\/robot>/d')
[ -n "$gz" ] || { echo "default 에 Gazebo 블록 없음 — 중단 (덮어쓰면 Gazebo 설정을 잃는다)" >&2; exit 1; }

tmp=$(mktemp)
{ sed '/<\/robot>/d' "$src"; echo; echo "$gz"; echo '</robot>'; } > "$tmp"
xacro "$tmp" mesh_uri:=default/meshes > "$tmp.urdf"
check_urdf "$tmp.urdf" >/dev/null
mv "$tmp" "$dst"; rm -f "$tmp.urdf"
cp -r beebot_mesh/meshes/. default/meshes/
echo "배포: $(pwd)/$dst + default/meshes/"
