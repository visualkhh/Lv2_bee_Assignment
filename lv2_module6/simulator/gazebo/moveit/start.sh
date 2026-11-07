#!/bin/bash
# 매니플레이터 계획·실행 (호스트): MoveIt RViz. move_group·arm_controller는 faker가 띄운 것을 쓴다.
#   전제: develop/gazebo/start.sh (월드+브리지+릴레이) · develop/bringup/faker.sh (move_group·arm_controller)
#   실행 결과: /arm_controller/follow_joint_trajectory → /joint_states → gazebo/relay.py → /gz_arm/*/cmd → Gazebo
#   최초 1회: develop/moveit/setup.sh (RViz 플러그인 + pick_ik)
#   종료: develop/moveit/kill.sh · 재시작: develop/moveit/restart.sh
# NOTE: sh(dash)로 실행해도 bash로 재실행
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi
source /opt/ros/lyrical/setup.bash
: "${ROS_DOMAIN_ID:=88}"; export ROS_DOMAIN_ID
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS="$HERE/../../ros2_ws"
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
if ! ros2 pkg prefix ros_system >/dev/null 2>&1; then
  echo "ros_system 없음 — 먼저 lv2_module6/ros2_ws/rebuild.sh로 빌드하십시오."
  exit 2
fi

if ! ros2 pkg prefix moveit_ros_visualization >/dev/null 2>&1; then
  echo "MoveIt RViz 플러그인 없음 — 최초 1회 설치:"
  echo "  $HERE/setup.sh"
  exit 2
fi
if ! ros2 pkg prefix pick_ik >/dev/null 2>&1; then
  echo "pick_ik 없음 — IK(포즈 목표) 계획이 실패합니다. 최초 1회 설치:"
  echo "  $HERE/setup.sh"
  exit 2
fi
if ! ros2 node list 2>/dev/null | grep -qx '/move_group'; then
  echo "move_group 없음 — develop/bringup/faker.sh를 먼저 실행하십시오."
  exit 2
fi
if ! pgrep -f "(gazebo/relay|gz_relay)[.]py" >/dev/null 2>&1; then
  echo "경고: relay가 없어 Gazebo 팔 미러링이 꺼져 있습니다 (develop/gazebo/start.sh)."
fi

SHARE=$(ros2 pkg prefix ros_system)/share/ros_system
URDF=$(xacro "$SHARE/urdf/default/urdf.xacro" \
  "mesh_uri:=package://ros_system/urdf/default/meshes" \
  mock_hardware:=false port:=/dev/opencr) || { echo "FAIL: xacro"; exit 1; }
export RV_URDF="$URDF"
export RV_SRDF="$(cat "$SHARE/config/moveit/beebot.srdf")"
export RV_KIN="$(cat "$SHARE/config/moveit/kinematics.yaml")"
export RV_LIM="$(cat "$SHARE/config/moveit/joint_limits.yaml")"

# RViz MoveIt 플러그인은 robot_description·semantic·kinematics·planning을 rviz2 파라미터로 읽는다.
python3 - <<'PY' || { echo "FAIL: RViz 파라미터 생성"; exit 1; }
import os
import yaml


def rosparams(text):
    data = yaml.safe_load(text)
    if isinstance(data, dict) and '/**' in data:
        data = data['/**'].get('ros__parameters', data['/**'])
    return data


params = {'/**': {'ros__parameters': {
    'robot_description': os.environ['RV_URDF'],
    'robot_description_semantic': os.environ['RV_SRDF'],
    'robot_description_kinematics': rosparams(os.environ['RV_KIN']),
    'robot_description_planning': rosparams(os.environ['RV_LIM']),
}}}
with open('/tmp/rviz_moveit_params.yaml', 'w') as f:
    yaml.safe_dump(params, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
PY

setsid nohup rviz2 -d "$HERE/moveit.rviz" --ros-args \
  --params-file /tmp/rviz_moveit_params.yaml > /tmp/rviz_moveit.log 2>&1 < /dev/null &
sleep 3
if pgrep -f "rvi[z]2.*moveit[.]rviz" >/dev/null 2>&1; then
  echo "RViz MoveIt 기동 (로그 /tmp/rviz_moveit.log)"
  echo "  Planning Group: arm — 목표(Interactive Marker/Goal State) → Plan → Execute"
  echo "  실행: /arm_controller/follow_joint_trajectory → /joint_states → gazebo/relay.py → /gz_arm/*/cmd → Gazebo"
else
  echo "FAIL: RViz 기동 실패 — /tmp/rviz_moveit.log 확인"
  tail -5 /tmp/rviz_moveit.log
  exit 1
fi
