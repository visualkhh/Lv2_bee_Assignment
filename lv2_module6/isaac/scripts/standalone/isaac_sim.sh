#!/usr/bin/env bash
# Isaac Sim + ROS 쪽을 한 번에 띄운다. Isaac을 끄면 ROS 쪽도 꺼진다.
#
#   isaac/scripts/standalone/isaac_sim.sh
#   ISAACSIM_VENV=<venv 경로> isaac/scripts/standalone/isaac_sim.sh   # venv 위치가 다를 때 (기본 ~/pa-course/.venv-isaacsim)
#   ISAAC_ROS_LAUNCH=isaac_manipulator_demo.launch.py ISAAC_AUTOPLAY=1 isaac/scripts/standalone/isaac_sim.sh  # SLAM·Nav2 없는 팔 시연 (scripts/demo/pick_load_play.py)
#   필요: ros2_ws 빌드(install/), sudo apt install ros-lyrical-depth-image-proc
#
# 1. Isaac (isaac_startup.py): scene_sim.usda 열기(Physics variant physx, 파일엔 저장 안 됨)
#    · opencr_bridge.py 실행 (pty만 열고 /tmp/opencr 링크는 Play 때). Script Editor는 띄우지 않는다
# 2. Play를 누르면 /tmp/opencr가 생기고 그때 ROS (isaac_system.launch.py)를 띄운다 — Play 전에 띄우면
#    BeebotSystemHardware가 S를 3초 안에 못 받아 활성화 실패 → 컨트롤러 전부 안 뜸 (2026-10-29)
# 3. 실행 중 Stop 금지 — 시뮬레이션 시계가 0으로 돌아가 ROS 노드(TF·EKF·cuVSLAM·nvblox)가 꼬인다. 잠깐 멈추려면 Pause, 처음부터는 isaac/scripts/standalone/reset.sh (Stop → Play → ROS 재시작)
HERE=$(dirname "$(realpath "$0")")
PORT=/tmp/opencr
PID_FILE=/tmp/beebot_isaac_sim.pid
RESET_FILE=/tmp/beebot_isaac_reset  # isaac_startup.py와 같은 경로
rm -f "$RESET_FILE"
# 관절 비틀림(opencr_bridge.py)과 그 보정(rl_arm_correction 앞먹임)을 늘 켠다. OPENCR_TWIST_SCALE=0 isaac_sim.sh 로 둘 다 끔
export OPENCR_TWIST_SCALE="${OPENCR_TWIST_SCALE:-1}"
# Fast DDS 프로파일은 ros2_ws/fastdds_local_only.xml (TypeLookup 끔 · SHM 32MB · SHM만 = 이 머신 안에서만) — Isaac·ROS 모두.
# 다른 터미널의 ros2 CLI도 같은 파일이어야 토픽이 보인다 (fastdds.xml(SHM+UDP)과 섞으면 서로 못 찾음, 2026-11-06 실측).
# 개인 프로파일(TypeLookup 켜짐)로 띄우면 노드 40개 규모에서 기동 직후 약 2분간 controller_manager 서비스가 안 보여
# spawner가 실패 → joint_states 없음 → nvblox가 gripper_cam TF를 못 구한 depth에 막혀 지도가 멈췄다 (2026-10-30 실측)
export FASTDDS_DEFAULT_PROFILES_FILE="$HERE/../../../ros2_ws/fastdds_local_only.xml"
source /opt/ros/lyrical/setup.bash   # Isaac도 이 환경(ROS_DISTRO=lyrical, ROS_DOMAIN_ID)을 물려받는다 — 카메라 토픽은 Isaac 안 bridge가 냄

# --- 지난 실행 찌꺼기 정리 (2026-10-30: 남은 노드·같은 도메인의 다른 실행과 이름이 겹쳐 컨트롤러·TF가 꼬였다)
# 1) 지난 isaac_system launch와 그 노드  2) 부모가 죽어 고아가 된 ROS 노드(--ros-args, 부모 = init/systemd --user)
# 3) Fast DDS 공유 메모리 찌꺼기  4) ros2 daemon(옛 노드·토픽 목록을 기억함)
stop_pids() {  # SIGINT로 정상 종료 → 5초 뒤 남은 것만 SIGKILL
  [ -n "$*" ] || return 0
  kill -INT $* 2>/dev/null
  for _ in $(seq 10); do kill -0 $* 2>/dev/null || return 0; sleep 0.5; done
  kill -KILL $* 2>/dev/null
}
OLD=$(pgrep -f "isaac/scripts/standalone/${ISAAC_ROS_LAUNCH:-isaac_system.launch.py}" | tr '\n' ' ')
[ -n "$OLD" ] && echo "지난 isaac_system launch 정리: $OLD" && stop_pids $OLD $(for p in $OLD; do pgrep -g "$(ps -o pgid= -p $p | tr -d ' ')"; done)
ORPHAN_PARENTS="1 $(pgrep -u "$USER" -x systemd | tr '\n' ' ')"
ORPHANS=$(ps -u "$USER" -o pid=,ppid=,args= | awk -v parents=" $ORPHAN_PARENTS " \
  '/--ros-args/ && index(parents, " "$2" ") {print $1}' | tr '\n' ' ')
[ -n "${ORPHANS// /}" ] && echo "고아 ROS 노드 정리: $ORPHANS" && stop_pids $ORPHANS
fastdds shm clean >/dev/null 2>&1
ros2 daemon stop >/dev/null 2>&1
# 같은 ROS_DOMAIN_ID에 다른 실행(예: MoveIt mock)이 있으면 노드 이름이 겹쳐 꼬인다 — 남의 실행이라 끄지 않고 알리고 멈춘다
OTHERS=$(ros2 node list --no-daemon --spin-time 3 2>/dev/null | grep -vE '^/(rviz|transform_listener_impl_|_ros2cli_)')  # RViz는 겹쳐도 무해
if [ -n "$OTHERS" ] && [ "${ALLOW_OTHER_NODES:-0}" != 1 ]; then
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}에 이미 노드가 떠 있음 — 끄거나 다른 도메인으로 옮긴 뒤 다시 실행 (무시: ALLOW_OTHER_NODES=1):"
  echo "$OTHERS" | sed 's/^/  /'
  exit 1
fi

rm -f "$PORT"                         # 지난 실행의 링크가 남아 있으면 아래 대기가 바로 통과해 버린다
# ROS's own pinocchio (ros-lyrical-pinocchio, built for numpy 1.x) sits on the sourced PYTHONPATH ahead of the copy Isaac
# bundles for its pink extension, and Isaac (numpy 2.x) then fails to start (2026-11-02). Put Isaac's copy first.
PINK_PY="${ISAACSIM_VENV:-$HOME/pa-course/.venv-isaacsim}/lib/python3.12/site-packages/isaacsim/exts/isaacsim.robot_motion.pink/pip_prebundle/cmeel.prefix/lib/python3.12/site-packages"
(export PYTHONPATH="$PINK_PY:$PYTHONPATH" && source "${ISAACSIM_VENV:-$HOME/pa-course/.venv-isaacsim}/bin/activate" && if [ -n "$ISAAC_STANDALONE" ]; then exec python3 "$ISAAC_STANDALONE" $ISAAC_STANDALONE_ARGS; else exec isaacsim --exec "$HERE/isaac_startup.py"; fi) &
ISAAC_PID=$!
until [ -e "$PORT" ]; do
  kill -0 $ISAAC_PID 2>/dev/null || exit 1
  sleep 1
done
source "$HERE/../../../ros2_ws/install/setup.bash"
# perception_node runs on system python3, which may lack torch/ultralytics (then YOLO is skipped: empty detections).
# YOLO_VENV=<venv with ultralytics, python3.12, numpy<2> appends its site-packages. It still shadows /usr/lib/python3/dist-packages
# (numpy, cv2), so its numpy must match the system 1.26 that cv_bridge is built against.
[ -n "$YOLO_VENV" ] && export PYTHONPATH="$PYTHONPATH:$YOLO_VENV/lib/python3.12/site-packages"
if ! YOLO_ERR=$(/usr/bin/python3 -c 'import cv_bridge, torch, ultralytics' 2>&1); then
  echo "[isaac_sim] YOLO 사용 불가 — 인식 결과가 비어 있게 된다 (YOLO_VENV='$YOLO_VENV'): $(tail -n 1 <<<"$YOLO_ERR")" >&2
fi
# 자기 프로세스 그룹으로 띄운다(setsid) — 끌 때 그룹 전체(launch + 노드)에 SIGINT를 직접 준다.
# 실측 2026-10-30: 백그라운드 launch는 SIGINT를 무시해 안 꺼졌고, SIGTERM을 주면 launch만 죽고 노드 10개가 고아로 남았다.
# 노드는 SIGINT로 정상 종료한다(nvblox는 매핑이면 이때 지도 저장). 15초 안에 안 끝나면 그룹을 SIGKILL.
start_ros() {
setsid ros2 launch "$HERE/${ISAAC_ROS_LAUNCH:-isaac_system.launch.py}" &
ROS_PID=$!
# 앞먹임·인지 처짐 보정 기본값은 켜짐 — 비틀림을 끈 Isaac에서만 둘 다 끈다.
# shortcut: 끌 때 한 번만 보낸다 — rl_arm_correction·perception만 다시 띄우면 켜지니 그때는 ros2 param set 또는 이 스크립트를 다시 실행
GRAV_PID=
if [ "$OPENCR_TWIST_SCALE" = 0 ]; then  # 비틀림이 없으면 launch가 twist_state도 끔 (isaac_manipulator_demo.launch.py)
  (until ros2 param set /rl_arm_correction gravity_enabled false >/dev/null 2>&1; do sleep 2; done
   echo "[isaac_sim] 중력 앞먹임 끔 (OPENCR_TWIST_SCALE=0)") &
  GRAV_PID=$!
fi
}
stop_ros() {
  [ -n "$GRAV_PID" ] && kill $GRAV_PID 2>/dev/null
  kill -INT -- -$ROS_PID 2>/dev/null
  for _ in $(seq 30); do pgrep -g $ROS_PID >/dev/null || return; sleep 0.5; done
  echo "ROS 노드가 15초 안에 안 꺼짐 — SIGKILL"
  kill -KILL -- -$ROS_PID 2>/dev/null
}
start_ros
cleanup() {
  trap - EXIT INT TERM HUP
  rm -f "$PID_FILE"
  kill $ISAAC_PID 2>/dev/null
  stop_ros
}
# reset.sh → USR1: ROS 끄기 → Isaac Stop · Play (장면 처음 자세로 · 시계 0) → ROS 다시
reset_sim() {
  echo "[isaac_sim] 리셋: ROS 끄기 → Stop · Play → ROS 다시"
  stop_ros
  fastdds shm clean >/dev/null 2>&1
  touch "$RESET_FILE"   # isaac_startup.py가 보고 Stop · Play 후 지움
  while [ -e "$RESET_FILE" ]; do
    kill -0 $ISAAC_PID 2>/dev/null || return
    sleep 0.5
  done
  sleep 2               # 물리 스텝이 돌아 S 줄이 나온 뒤 (BeebotSystemHardware가 3초 안에 받아야 함)
  start_ros
}
RESET_WANTED=0
trap 'RESET_WANTED=1' USR1
echo $$ > "$PID_FILE"
trap cleanup EXIT INT TERM HUP  # HUP: 터미널 창을 닫을 때 — setsid로 띄운 launch는 터미널의 HUP을 못 받는다
while kill -0 $ISAAC_PID 2>/dev/null; do
  wait $ISAAC_PID       # USR1이 오면 여기서 깨어남
  if [ "$RESET_WANTED" = 1 ]; then
    RESET_WANTED=0
    reset_sim
  fi
done
