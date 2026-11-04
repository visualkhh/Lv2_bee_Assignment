#!/usr/bin/env bash
# 실제와 가까운 미션 시뮬 — Isaac Sim 디지털 트윈 + 로봇과 같은 ROS 전체 (skill 노드 · MoveIt · Nav2 · cuVSLAM · nvblox · YOLO)
#
#   isaac/sim/twin.sh map              1) 지도 만들기: SLAM 매핑 + 자동 탐사(EXPLORE). 다 돌면 Isaac 창을 닫음 → nvblox가 지도 저장
#   isaac/sim/twin.sh mission [seed]   2) 저장한 지도로 위치추정 + 무작위 문제(퍽 · 신호 · 바코드) → 미션 시작 (seed 없으면 무작위)
#   BEE_LYING=0.2 isaac/sim/twin.sh mission 7    누운 퍽 20 %
#
# pa27 데스크톱 터미널에서 (SSH로 띄우면 끊길 때 같이 꺼짐 — 원격이면: systemd-run --user --setenv=DISPLAY=:1 … twin.sh mission)
# 필요: ros2_ws 빌드 · 인지 노드용 venv(torch · ultralytics · zxing-cpp, 기본 ~/bee_twin_venv — --system-site-packages)
# 지도: ~/bee_twin_maps (실기 지도 ~/.ros/beebot_maps와 따로) · 문제·정답: ~/bee_twin_runs/<시각>/problem.json
set -e
HERE=$(dirname "$(realpath "$0")")
MODE=${1:-mission}
VENV=${BEE_TWIN_VENV:-$HOME/bee_twin_venv}
export BEE_MAP_DIR=${BEE_MAP_DIR:-$HOME/bee_twin_maps}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # pa27 ~/.bashrc는 Cyclone(pa-course) — 이 프로젝트는 Fast DDS (ros2_ws/fastdds.xml)
# 로봇을 미션 출발 자리에 — 출발 구역 가운데 (734, 950) mm · −X (운영 규칙 · mission_manager start_pose_arena)
#   scene_sim.usda는 로봇이 1.3 m 동쪽에서 +X를 봄 → 그대로면 지도 좌표가 경기장과 180° 어긋나 미션 목표가 엉뚱한 곳 (11-04)
#   장면 좌표 = 경기장 m + (−2.721, −3.421) (problem_setup.ARENA_TO_SCENE)
export BEE_START_POSE="-1.987,-2.471,3.14159"
export BEE_AUTOPLAY=1                       # Isaac이 장면을 열면 Play (그때 ROS가 뜸)
export PATH="$VENV/bin:$PATH"               # 인지 노드(perception_node)가 venv python3로 — torch · ultralytics
mkdir -p "$BEE_MAP_DIR"
case "$MODE" in
  map)     export BEE_MAP=slam; unset BEE_PROBLEM; AFTER="EXPLORE" ;;
  mission) export BEE_MAP=saved; export BEE_PROBLEM=${2:-random}; AFTER="AUTO"
           export BEE_RUN_DIR=${BEE_RUN_DIR:-$HOME/bee_twin_runs/$(date +%m%d_%H%M%S)} ;;
  *) sed -n 2,12p "$0"; exit 1 ;;
esac
# 노드가 다 뜨면 운영 모드 → (미션이면) 시작 명령. 비동기로 기다림 (isaac_sim.sh가 앞에서 돎)
(
  source /opt/ros/lyrical/setup.bash
  until ros2 node list 2>/dev/null | grep -q '^/skill_pose$' && ros2 node list 2>/dev/null | grep -q '^/mission_manager$'; do sleep 3; done
  sleep 20  # 컨트롤러 · Nav2 · MoveIt 활성화
  ros2 topic pub --once /cmd/operation_mode std_msgs/msg/String "{data: $AFTER}" >/dev/null
  echo "[twin] 운영 모드 $AFTER"
  if [ "$MODE" = mission ]; then
    sleep 5
    ros2 topic pub --once /cmd/mission std_msgs/msg/String "{data: start}" >/dev/null
    echo "[twin] 미션 시작 — 문제·정답 $BEE_RUN_DIR/problem.json · 진행 /mission/state"
  fi
) &
# 실시간 상태 화면 (RViz): 지도 · Nav2 장애물 지도 · 계획 경로 · 로봇 · 카메라 점구름 · cuVSLAM 경로 · 미션 표시 — BEE_RVIZ=0이면 안 띄움
if [ "${BEE_RVIZ:-1}" = 1 ]; then
  (
    source /opt/ros/lyrical/setup.bash
    source "$HERE/../../ros2_ws/install/setup.bash"
    export FASTDDS_DEFAULT_PROFILES_FILE="$HERE/../../ros2_ws/fastdds.xml"
    export QT_SCALE_FACTOR_ROUNDING_POLICY=Floor XDG_CURRENT_DESKTOP=ubuntu:GNOME XDG_SESSION_TYPE=x11  # 3D 깜박임 · 어두운 테마
    until ros2 node list 2>/dev/null | grep -q '^/nvblox_map$'; do sleep 3; done
    exec rviz2 -d "$HERE/scripts/twin.rviz" --ros-args -p use_sim_time:=true
  ) > /tmp/twin_rviz.log 2>&1 &
fi
exec "$HERE/isaac_sim.sh"
