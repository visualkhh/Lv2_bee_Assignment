#!/usr/bin/env bash
# 전체 미션용 Isaac — scene_sim.usda + mission.launch.py (인지 켬 · AUTO). 환경변수는 isaac_sim.sh 그대로 (ISAAC_AUTOPLAY=1 등)
#   isaac/mission/isaac.sh
#   ISAAC_AUTOPLAY=1 isaac/mission/isaac.sh
# 그다음 다른 터미널: python3 isaac/mission/run_mission.py
HERE=$(cd "$(dirname "$0")" && pwd)
ISAAC_ROS_LAUNCH=../../mission/mission.launch.py exec "$HERE/../standalone/isaac_sim.sh" "$@"
