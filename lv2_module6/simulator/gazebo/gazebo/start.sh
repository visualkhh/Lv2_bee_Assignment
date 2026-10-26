#!/bin/bash
# Gazebo 클린 스타트 (호스트): 남아있는 것 정리 → 월드 + 무작위 장애물 + AprilTag 퍽 미션
# + 지도에 분산된 4개 미션 + 통행로 위 갠트리 신호등 + 이동 중 동적 장애물
# + 수동 액터 GUI 버튼 + beebot 스폰 + ros_gz_bridge + pose 마커
#
#   lv2_module6/develop/gazebo/start.sh [gz sim 인자...]
#
# 기본: 퍽 9개를 네 미션(SORT/MOVE/STAND/LAY)에 3/2/2/2로 나눠 지도에 분산.
#       각 미션 근처 벽에 미션 태그, 상단 방 통행로 위에는 갠트리 신호등 2개.
#   GZ_MISSION=SORT_COLOR|MOVE_TO_B|STAND_UP|LAY_DOWN|random
#   GZ_PUCK_COUNT=9 GZ_PUCK_MM="30 30 60"
#   GZ_MISSION_SITES=all|single  (single이면 GZ_MISSION_CENTER="0.35 2.75")
#   GZ_OBSTACLE_BOXES=6 GZ_OBSTACLE_CYLINDERS=6
#   GZ_DYNAMIC_COUNT=3 (로봇 이동 중에만 나타남)
#   GZ_DYNAMIC_LIFETIME=12 GZ_DYNAMIC_RESPAWN=1 GZ_DYNAMIC_SPEED=0.16
#   GZ_TRAFFIC_INTERVAL=3.0 (빨강→초록→주황)
#   GZ_FLOOR_TEXTURE=floor_wood GZ_WALL_TEXTURE=wall_painted \
#   GZ_BACKGROUND=pano_wooden_lounge  (none이면 기존 재질)
#
# NOTE: sh(dash)로 실행해도 bash로 재실행
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi
source /opt/ros/lyrical/setup.bash
: "${ROS_DOMAIN_ID:=88}"; export ROS_DOMAIN_ID
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)

"$HERE/kill.sh" >/dev/null 2>&1

BASE_WORLD="$ROOT/course/worlds/twin_closed_maze.sdf"
WALLS="$ROOT/course/twin_closed_maze/walls.json"
WORLD=/tmp/beebot_world.sdf
SCENE_STATE=/tmp/beebot_scene.json
SPAWN_MODEL_DIR=/tmp/beebot_spawn_models
ENV_MODEL_DIR=/tmp/beebot_environment_models
export BEEBOT_SPAWN_DIR="$SPAWN_MODEL_DIR"
export BEEBOT_ENV_MODEL_DIR="$ENV_MODEL_DIR"
SPAWN_X=-1.835
SPAWN_Y=2.757
SPAWN_Z=0.05
export GZ_SIM_RESOURCE_PATH="$ROOT/course${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

# 수동 액터 GUI 플러그인 (사람/박스 소환, 수동 모델 삭제)
GUI_PLUGIN_DIR="$HERE/gui_plugin"
if [ ! -f "$GUI_PLUGIN_DIR/build/libManualActors.so" ]; then
  (cd "$GUI_PLUGIN_DIR" && bash build.sh) \
    || echo "WARN: ManualActors GUI 플러그인 빌드 실패 — Gazebo는 기본 GUI로 시작" >&2
fi
export GZ_GUI_PLUGIN_PATH="$GUI_PLUGIN_DIR/build${GZ_GUI_PLUGIN_PATH:+:$GZ_GUI_PLUGIN_PATH}"
# NVIDIA EGL 직접 지정 (Mesa dri2 크래시 회피). 없으면 건드리지 않음
if [ -z "$__EGL_VENDOR_LIBRARY_FILENAMES" ] && [ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]; then
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
fi

[ -f "$BASE_WORLD" ] || { echo "FAIL: 월드 없음: $BASE_WORLD"; exit 2; }
chmod +x "$HERE/generate_world.py" "$HERE/generate_tags.py" \
         "$HERE/relay.py" "$HERE/publish_poses.py" \
         "$HERE/traffic_lights.py" "$HERE/dynamic_obstacles.py" 2>/dev/null || true
python3 "$HERE/generate_world.py" \
  --world-in "$BASE_WORLD" --walls "$WALLS" \
  --world-out "$WORLD" --state-out "$SCENE_STATE" \
  --spawn "$SPAWN_X $SPAWN_Y $SPAWN_Z 0" \
  --spawn-model-dir "$SPAWN_MODEL_DIR" \
  --env-model-dir "$ENV_MODEL_DIR" \
  || { echo "FAIL: 월드/미션 생성 실패"; exit 2; }

setsid nohup gz sim --gui-config "$HERE/gui.config" "$WORLD" "$@" \
  > /tmp/gz.log 2>&1 < /dev/null &
SRV_PID=$!
trap 'kill -INT $SRV_PID 2>/dev/null' INT TERM
echo "월드 기동 중... (로그 /tmp/gz.log)"
for _ in $(seq 1 30); do
  timeout 15 gz topic -l 2>/dev/null | grep -qx "/world/default/clock" && break
  kill -0 "$SRV_PID" 2>/dev/null || { echo "FAIL: gazebo 기동 실패 — /tmp/gz.log 확인"; exit 1; }
  sleep 1
done
timeout 15 gz topic -l 2>/dev/null | grep -qx "/world/default/clock" \
  || { echo "FAIL: 월드 안 뜸 — /tmp/gz.log 확인"; exit 1; }
# 월드가 멈춤 상태로 시작하는 경우가 있음 (센서·물리·odom 전부 정지) — 무조건 재생
timeout 30 gz service -s /world/default/control --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean --timeout 8000 --req 'pause: false' >/dev/null 2>&1 || true

# 로봇 URDF 생성 (package:// → file://, Gazebo가 package://를 못 읽어서)
MESHDIR="$ROOT/urdf/default/meshes"
xacro "$ROOT/urdf/default/urdf.xacro" "mesh_uri:=file://$MESHDIR" -o /tmp/beebot_gz.urdf \
  || { echo "FAIL: xacro 변환 실패"; exit 1; }

# 스폰
timeout 60 ros2 run ros_gz_sim create -world default -name beebot -file /tmp/beebot_gz.urdf \
  -x "$SPAWN_X" -y "$SPAWN_Y" -z "$SPAWN_Z" || { echo "FAIL: 스폰 실패"; exit 1; }
echo "스폰: beebot"

# 브리지 + 릴레이 + 실제 pose 마커 (릴레이: /gz_raw/* 시뮬시간 → 벽시계 찍어 표준 토픽으로)
CFG="$HERE/bridge.yaml"
pkill -f "paramete[r]_bridge.*[.]yaml" 2>/dev/null
pkill -f "(gazebo/relay|gz_relay)[.]py" 2>/dev/null
pkill -f "(publish_poses|gz_obstacle_markers)[.]py" 2>/dev/null
pkill -f "gazebo/(traffic_lights|dynamic_obstacles)[.]py" 2>/dev/null
sleep 1
setsid nohup ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$CFG \
  > /tmp/gz_bridge.log 2>&1 < /dev/null &
setsid nohup python3 "$HERE/relay.py" > /tmp/gz_relay.log 2>&1 < /dev/null &
setsid nohup python3 "$HERE/publish_poses.py" --state "$SCENE_STATE" \
  > /tmp/gz_scene_markers.log 2>&1 < /dev/null &
sleep 1
setsid nohup python3 "$HERE/traffic_lights.py" --state "$SCENE_STATE" \
  > /tmp/gz_traffic.log 2>&1 < /dev/null &
setsid nohup python3 "$HERE/dynamic_obstacles.py" --state "$SCENE_STATE" \
  > /tmp/gz_dynamic.log 2>&1 < /dev/null &
echo "브릿지+릴레이+씬 마커+신호등+동적 장애물 기동"
echo "  로그: /tmp/gz_bridge.log /tmp/gz_relay.log /tmp/gz_scene_markers.log /tmp/gz_traffic.log /tmp/gz_dynamic.log"
trap 'pkill -f "paramete[r]_bridge.*[.]yaml"; pkill -f "(gazebo/relay|gz_relay)[.]py"; pkill -f "(publish_poses|gz_obstacle_markers)[.]py"; pkill -f "gazebo/(traffic_lights|dynamic_obstacles)[.]py"; kill -INT $SRV_PID 2>/dev/null' INT TERM
echo "종료는 Ctrl-C (서버·브리지 같이 끔)"
wait "$SRV_PID" 2>/dev/null || true
