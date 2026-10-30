#!/usr/bin/env bash
# Isaac Sim + ROS 쪽을 한 번에 띄운다. Isaac을 끄면 ROS 쪽도 꺼진다.
#
#   isaac/sim/isaac_sim.sh
#   ISAACSIM_VENV=<venv 경로> isaac/sim/isaac_sim.sh   # venv 위치가 다를 때 (기본 ~/pa-course/.venv-isaacsim)
#   필요: ros2_ws 빌드(install/), sudo apt install ros-lyrical-depth-image-proc
#
# 1. Isaac (isaac_startup.py): scene_sim.usda 열기(Physics variant physx, 파일엔 저장 안 됨)
#    · opencr_bridge.py 실행 (pty만 열고 /tmp/opencr 링크는 Play 때). Script Editor는 띄우지 않는다
# 2. Play를 누르면 /tmp/opencr가 생기고 그때 ROS (isaac_system.launch.py)를 띄운다 — Play 전에 띄우면
#    BeebotSystemHardware가 S를 3초 안에 못 받아 활성화 실패 → 컨트롤러 전부 안 뜸 (2026-10-29)
# 3. 실행 중 Stop 금지 — 시뮬레이션 시계가 0으로 돌아가 cuVSLAM·nvblox가 멈춘다. 잠깐 멈추려면 Pause, 처음부터는 이 스크립트를 다시 실행
HERE=$(dirname "$(realpath "$0")")
PORT=/tmp/opencr
# Fast DDS 프로파일은 실기 start.sh와 같은 ros2_ws/fastdds.xml (TypeLookup 끔 · SHM 32MB) — Isaac·ROS 모두.
# 개인 프로파일(TypeLookup 켜짐)로 띄우면 노드 40개 규모에서 기동 직후 약 2분간 controller_manager 서비스가 안 보여
# spawner가 실패 → joint_states 없음 → nvblox가 gripper_cam TF를 못 구한 depth에 막혀 지도가 멈췄다 (2026-10-30 실측)
export FASTDDS_DEFAULT_PROFILES_FILE="$HERE/../../ros2_ws/fastdds.xml"
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
OLD=$(pgrep -f "isaac/sim/scripts/isaac_system.launch.py" | tr '\n' ' ')
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
(source "${ISAACSIM_VENV:-$HOME/pa-course/.venv-isaacsim}/bin/activate" && exec isaacsim --exec "$HERE/scripts/isaac_startup.py") &
ISAAC_PID=$!
until [ -e "$PORT" ]; do
  kill -0 $ISAAC_PID 2>/dev/null || exit 1
  sleep 1
done
source "$HERE/../../ros2_ws/install/setup.bash"
# 자기 프로세스 그룹으로 띄운다(setsid) — 끌 때 그룹 전체(launch + 노드)에 SIGINT를 직접 준다.
# 실측 2026-10-30: 백그라운드 launch는 SIGINT를 무시해 안 꺼졌고, SIGTERM을 주면 launch만 죽고 노드 10개가 고아로 남았다.
# 노드는 SIGINT로 정상 종료한다(nvblox는 매핑이면 이때 지도 저장). 15초 안에 안 끝나면 그룹을 SIGKILL.
setsid ros2 launch "$HERE/scripts/isaac_system.launch.py" &
ROS_PID=$!
cleanup() {
  trap - EXIT INT TERM HUP
  kill $ISAAC_PID 2>/dev/null
  kill -INT -- -$ROS_PID 2>/dev/null
  for _ in $(seq 30); do pgrep -g $ROS_PID >/dev/null || return; sleep 0.5; done
  echo "ROS 노드가 15초 안에 안 꺼짐 — SIGKILL"
  kill -KILL -- -$ROS_PID 2>/dev/null
}
trap cleanup EXIT INT TERM HUP  # HUP: 터미널 창을 닫을 때 — setsid로 띄운 launch는 터미널의 HUP을 못 받는다
wait $ISAAC_PID
