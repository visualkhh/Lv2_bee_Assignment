#!/bin/bash
# Gazebo에 beebot 스폰 + ros_gz_bridge 시작 (world가 먼저 떠 있어야 함: develop/gazebo.sh)
#   develop/gazebo_spawn.sh
# NOTE: sh(dash)로 실행해도 bash로 재실행
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi
source /opt/ros/lyrical/setup.bash
: "${ROS_DOMAIN_ID:=8}"; export ROS_DOMAIN_ID
HERE=$(dirname "${BASH_SOURCE[0]}")
[ "$HERE" = . ] && HERE="$PWD"
ROOT="$HERE/../.."

# 1) URDF 생성 (package:// → file:// 절대경로, Gazebo가 package://를 못 읽어서)
MESHDIR="$ROOT/lv2_module6/urdf/default/meshes"
XACRO="$ROOT/lv2_module6/urdf/default/urdf.xacro"
[ -f "$XACRO" ] || { echo "FAIL: xacro 없음: $XACRO"; exit 2; }
xacro "$XACRO" "mesh_uri:=file://$MESHDIR" -o /tmp/beebot_gz.urdf || exit 1

# 2) 기존 엔티티 제거 (없으면 무시) → 스폰
timeout 30 gz service -s /world/default/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 8000 --req 'name: "beebot", type: 2' >/dev/null 2>&1
timeout 60 ros2 run ros_gz_sim create -world default -name beebot -file /tmp/beebot_gz.urdf \
  -x -1.835 -y 2.757 -z 0.3 || { echo "FAIL: 스폰 실패 — world가 떴는지 확인 (develop/gazebo.sh)"; exit 1; }

# 3) 브리지 재시작 (config 바뀌면 여기로 반영)
CFG="$ROOT/lv2_module6/ros2_ws/src/ros_system/config/gz_bridge.yaml"
pkill -f "paramete[r]_bridge.*gz_bridge[.]yaml" 2>/dev/null   # 대괄호 트릭: 자기 자신은 안 죽임
sleep 1
setsid nohup ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$CFG \
  > /tmp/gz_bridge.log 2>&1 < /dev/null &
echo "OK: 스폰 + 브리지 (로그 /tmp/gz_bridge.log)"
sleep 3
timeout 20 ros2 topic list 2>/dev/null | grep -E "^/gz/|pole_cam/color/image_raw"
