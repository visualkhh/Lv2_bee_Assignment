#!/usr/bin/env bash
# Isaac Sim + ROS 쪽을 한 번에 띄운다. Isaac을 끄면 ROS 쪽도 꺼진다.
#
#   isaac/sim/scripts/isaac_sim.sh
#   ISAACSIM_VENV=<venv 경로> isaac/sim/scripts/isaac_sim.sh   # venv 위치가 다를 때 (기본 ~/pa-course/.venv-isaacsim)
#   필요: ros2_ws 빌드(install/), sudo apt install ros-lyrical-depth-image-proc
#
# 1. Isaac (isaac_startup.py): scene_sim.usda 열기(Physics variant physx, 파일엔 저장 안 됨)
#    · Viewport 왼쪽 Script Editor + opencr_bridge.py·sim_drive.py 탭 · opencr_bridge.py 실행 → /tmp/opencr
# 2. /tmp/opencr가 생기면 ROS (isaac_system.launch.py): ros2_control(port:=/tmp/opencr) + pole_front_cam 점구름
# 3. Isaac에서 Play → 가상 OpenCR가 S를 보내기 시작 (Play 전엔 BeebotSystemHardware가 "S 수신 없음" 경고 — 정상)
HERE=$(dirname "$(realpath "$0")")
PORT=/tmp/opencr
source /opt/ros/lyrical/setup.bash   # Isaac도 이 환경(ROS_DISTRO=lyrical, ROS_DOMAIN_ID)을 물려받는다 — 카메라 토픽은 Isaac 안 bridge가 냄
rm -f "$PORT"                         # 지난 실행의 링크가 남아 있으면 아래 대기가 바로 통과해 버린다
(source "${ISAACSIM_VENV:-$HOME/pa-course/.venv-isaacsim}/bin/activate" && exec isaacsim --exec "$HERE/isaac_startup.py") &
ISAAC_PID=$!
until [ -e "$PORT" ]; do
  kill -0 $ISAAC_PID 2>/dev/null || exit 1
  sleep 1
done
source "$HERE/../../../ros2_ws/install/setup.bash"
ros2 launch "$HERE/isaac_system.launch.py" &
ROS_PID=$!
trap 'kill $ROS_PID $ISAAC_PID 2>/dev/null' EXIT INT TERM
wait $ISAAC_PID
