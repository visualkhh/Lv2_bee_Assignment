#!/bin/bash
# 더미 전체 흐름 검사 (컨테이너 안에서 실행): test-faker [초|0] [launch 인자...]
#   faker launch = bringup(🛠 노드) + ros_system(controller_manager ↔ 가상 시리얼 ↔ fake_opencr · EKF) + 대역(📦) + 파일 연동 → user_input 명령 한 번
#   /task_status 에 SUCCEEDED(state 1)가 오고 /robot_say 가 나오고,
#   controller_manager(fake_opencr S 상태)가 /joint_states(팔 5축 + 그리퍼)·EKF가 /odometry/filtered 를 내면 PASS
#   초: SUCCEEDED 뒤 더 돌릴 시간 (기본 1). 0이면 Ctrl+C까지 띄워 둠 (판정 없음)
#   예) test-faker                       정상 흐름
#       test-faker 1                        파지까지 전체 흐름 (진짜 move_group)
#       test-faker 0                      띄워 두고 다른 터미널에서 ros2 topic echo … (시뮬레이터 [ROS 재시작] = /cmd/restart → state_manager가 다시 띄움)
#       test-faker 1 mock_hardware:=true    OpenCR 없이 mock (물리 없음)
# 배관(joint_states · odom · Nav2 · monitor)은 항상 본다. 작업 흐름(카메라 → 인지 → 작업 성공)은 사용자가 호스트 시뮬레이터를 켰을 때만.
# 토픽·노드·패키지 기록은 monitor_manager가 debug/topic · node · pkg · output/topic 에 남긴다.
# NOTE: set -u 사용 금지 — setup.bash가 미설정 변수를 참조해서 죽음.
# shellcheck disable=SC1091
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi   # sh(dash)로 실행해도 bash로 재실행
source /opt/ros/lyrical/setup.bash
BIN_DIR=$(dirname "${BASH_SOURCE[0]}")
[ "$BIN_DIR" = . ] && BIN_DIR="$PWD"   # 형제 스크립트 탐색용 — 컨테이너(/usr/local/bin)·호스트(docker/) 모두 동작
bin_of() {  # 컨테이너는 확장자 없음, 레포는 .sh — 있는 쪽 반환, 둘 다 없으면 PATH에 기대고 bare 이름
  [ -x "$BIN_DIR/$1" ] && echo "$BIN_DIR/$1" && return 0
  [ -x "$BIN_DIR/$1.sh" ] && echo "$BIN_DIR/$1.sh" && return 0
  echo "$1"
}
if [ -z "$WS" ]; then
  if [ -d /ws/src ]; then WS=/ws; else WS="$BIN_DIR/../lv2_module6/ros2_ws"; fi   # 컨테이너면 /ws, 호스트면 레포 안 ros2_ws (머신 경로 하드코딩 없음)
fi
WS=$(realpath -m "$WS")   # ../ 정리 — CMakeCache 절대경로와 문자열 비교하므로 정규화 필수
# DDS: 컨테이너 compose와 동일 (DEVELOP.md §1). 없으면 노드 40개에서 UDP 폭주로 컨트롤러가 못 뜸
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}
export FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE:-$WS/fastdds.xml}
cd "$WS" || { echo "FAIL: $WS 없음"; exit 2; }

KILL_PAT="($WS/install|/opt/ros/lyrical)/[^ ]*lib/|ros2 launch"   # ROS 노드 전부 + launch (패키지를 적으면 새 패키지가 빠져 고아로 남음)
kill_pat() {  # $1 = 시그널. ros_gz_bridge(개발 Gazebo 브리지)는 살림 — test-faker 재시작과 무관하게 유지
  local pids bp b
  pids=$(pgrep -f "$KILL_PAT" 2>/dev/null) || return 0
  bp=$(pgrep -f "paramete[r]_bridge" 2>/dev/null) || bp=""
  for b in $bp; do pids=$(echo "$pids" | grep -vx "$b" || true); done
  # shellcheck disable=SC2086
  [ -n "$pids" ] && kill "-$1" $pids 2>/dev/null
  return 0
}
kill_pat TERM   # 빌드 전에 — 이전 실행의 노드가 남아 돌면 빌드도 느려짐
"$(bin_of build-ws)" || exit 1   # 빌드 + install → /debug/install (docker/build-ws.sh)
# shellcheck disable=SC1091
source install/setup.bash


DUR=${1:-1}
shift 2>/dev/null
if [ -z "$DEBUG_DIR" ]; then
  if [ -d /debug ]; then DEBUG_DIR=/debug; else DEBUG_DIR="$WS/../debug"; fi   # 컨테이너면 /debug, 호스트면 ros2_ws 옆 debug (머신 경로 하드코딩 없음)
fi
DEBUG_DIR=$(realpath -m "$DEBUG_DIR")
case " $* " in
  *\ debug_dir:=*) : ;;   # 사용자가 직접 지정하면 그대로
  *) set -- "$@" "debug_dir:=$DEBUG_DIR" ;;
esac
case " $* " in
  *\ mock_hardware:=*) : ;;   # 사용자가 직접 지정하면 그대로
  *) [ -e /dev/opencr ] || set -- "$@" "mock_hardware:=true" ;;   # 호스트엔 가상 시리얼이 없어서 mock으로 (컨테이너는 entrypoint가 만듦)
esac
case " $* " in
  *\ use_monitor:=false*) MONITOR=0 ;;   # 호스트 가벼운 실행용 (도커·CI는 기본 true)
  *) MONITOR=1 ;;
esac
set -m
ros2 launch faker faker.launch.py "$@" > /tmp/faker.log 2>&1 & LAUNCH_PID=$!
JS=$(timeout 60 ros2 topic echo --once /joint_states sensor_msgs/msg/JointState 2>/dev/null)
# spawner가 기동 타이밍을 놓치면(FATAL) 컨트롤러 없이 돈다 — 죽은 spawner 치우고 최대 2회 재시도
for i in 1 2; do
  echo "$JS" | grep -q gripper_left_joint && break
  echo "spawner 재시도 ($i/2)..."
  pkill -f 'controller_manager/spawner' 2>/dev/null || true
  ros2 run controller_manager spawner joint_state_broadcaster imu_sensor_broadcaster diff_drive_controller rl_arm_correction arm_controller gripper_controller --controller-manager /controller_manager --controller-manager-timeout 120 --service-call-timeout 30 --param-file "$WS/install/ros_system/share/ros_system/config/controllers.yaml" --controller-ros-args '-r ~/cmd_vel:=/cmd_vel' --controller-ros-args '-r ~/odom:=/odom' --controller-ros-args '-r ~/imu:=/imu' > /tmp/spawner-retry.log 2>&1 || true
  JS=$(timeout 30 ros2 topic echo --once /joint_states sensor_msgs/msg/JointState 2>/dev/null)
done

# 카메라 = 사용자가 호스트에서 켠 시뮬레이터 (테스트가 띄우지 않음). 켜져 있으면 /robot_description을 받은 뒤 렌더 시작 → 최대 20초 기다림
# 꺼져 있으면 카메라 → 인지 → 작업 흐름 검사는 건너뛰고 배관(joint_states · odom · Nav2 · monitor)만 본다
META=$DEBUG_DIR/input/hardware/camera/pole_front_cam.json
for _ in $(seq 20); do
  [ -f "$META" ] && [ $(( $(date +%s) - $(stat -c %Y "$META") )) -lt 3 ] && break
  sleep 1
done
CAM_OK=0
[ -f "$META" ] && [ $(( $(date +%s) - $(stat -c %Y "$META") )) -lt 3 ] && CAM_OK=1
# Nav2 서버가 다 active인지 — 관리자 둘의 마지막 서버: collision_monitor(lifecycle_manager_base) · bt_navigator(lifecycle_manager_navigation)
NAV2=
for _ in $(seq 60); do
  NAV2="$(timeout 5 ros2 lifecycle get /collision_monitor 2>/dev/null) / $(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null)"
  [ "$(echo "$NAV2" | grep -o 'active \[3\]' | wc -l)" = 2 ] && break
  sleep 1
done

stop_all() {
  kill -INT "$LAUNCH_PID" 2>/dev/null
  timeout 15 tail --pid="$LAUNCH_PID" -f /dev/null
  kill_pat KILL
}

if [ "$DUR" = 0 ]; then
  # 띄워 두기. 재시작은 state_manager가 한다 (/cmd/restart "all [이름:=값 …]" · "node <이름>" — 실기 bringup과 같은 길):
  #   all이면 state_manager의 도우미가 이 launch를 끄고 → build-ws(faker restart_prepare) → 같은 명령줄(바꾼 인자만 새 값)로 다시 띄움.
  #   다시 뜬 launch는 이 스크립트의 자식이 아니므로, 도우미가 도는 동안(MARKER 있음)이나 faker launch가 하나라도 떠 있으면 기다린다.
  #   시뮬레이터 [ROS 재시작] → 파일 debug/input/topic/cmd/restart → input_manager가 발행(읽은 파일은 지움 — 두 번 안 감) → state_manager
  #   ROS가 통째로 막혀 state_manager가 못 받으면: 다른 터미널에서 ros-faker-restart (끄고 test-faker 0 새로)
  MARKER=/tmp/beebot_restart.pending   # manager.yaml restart_marker와 같게
  echo "실행 중 (로그 /tmp/faker.log)... 종료는 Ctrl+C · 재시작 = /cmd/restart (state_manager)"
  trap 'echo; stop_all; exit 0' INT
  while kill -0 "$LAUNCH_PID" 2>/dev/null || pgrep -f "ros2 launch faker" >/dev/null || [ -e "$MARKER" ]; do
    sleep 1
  done
  echo "$(date +%T) faker launch 끝남"
  trap '' INT
  stop_all
  exit 0
fi

# 타입을 명시해야 /task_status가 아직 안 생겼을 때 바로 끝나지 않고 기다림. transient_local로 받아야 놓친 것도 옴
DONE=
[ "$CAM_OK" = 1 ] && DONE=$(timeout 60 ros2 topic echo --once --filter 'm.state == 1' \
  --qos-reliability reliable --qos-durability transient_local /task_status plan/msg/TaskStatus 2>/dev/null)
ODOM=$(timeout 10 ros2 topic echo --once /odometry/filtered nav_msgs/msg/Odometry 2>/dev/null)
sleep "$DUR"
stop_all

echo "--- 흐름 (faker.log) ---"
grep -E '\] \[(user_input|vlm_task_planner|coordinator|grasp_scorer|rl_nav_tuner|controller_server|move_group|arm_safety_monitor|spawner_[a-z_]+)\]' /tmp/faker.log \
  | sed -E 's/^\[[^]]+\] //' | head -60
[ "$(echo "$NAV2" | grep -o 'active \[3\]' | wc -l)" = 2 ] || { echo "FAIL: Nav2 서버가 active 아님 (collision_monitor / bt_navigator: ${NAV2:-응답 없음})"; grep -E "lifecycle_manager|ERROR" /tmp/faker.log | tail -20; exit 1; }
echo "$JS" | grep -q gripper_left_joint || { echo "FAIL: /joint_states에 gripper_left_joint 없음 (controller_manager·BeebotSystemHardware)"; grep -E "controller_manager|spawner|ros2_control" /tmp/faker.log | tail -20; exit 1; }
[ -n "$ODOM" ] || { echo "FAIL: /odometry/filtered 없음 (EKF ← /odom·/imu)"; exit 1; }
if [ "$MONITOR" = 1 ]; then
  [ -s "$DEBUG_DIR/output/topic/joint_states/message" ] || { echo "FAIL: monitor_manager 파일 없음 ($DEBUG_DIR/output/topic/joint_states/message)"; exit 1; }
  echo "기록 (monitor_manager): $DEBUG_DIR/topic · $DEBUG_DIR/node · $DEBUG_DIR/pkg (그래프) · $DEBUG_DIR/output/topic (message·image)"
fi
if [ "$CAM_OK" = 1 ]; then
  [ -n "$DONE" ] || { echo "FAIL: /task_status SUCCEEDED 없음 (60초)"; tail -n 40 /tmp/faker.log; exit 1; }
  grep -q '🔊' /tmp/faker.log || { echo "FAIL: /robot_say 없음"; exit 1; }
  echo "PASS: faker 흐름 (시뮬레이터 카메라 → 인지 → vlm → coordinator → Nav2·grasp·move_group → task_status SUCCEEDED)"
else
  echo "건너뜀: 작업 흐름 — 시뮬레이터 꺼짐 ($META 갱신 없음). 흐름까지 보려면 호스트에서 시뮬레이터를 켜고 다시: cd lv2_module6/simulator && .venv/bin/python main.py"
fi
if [ "$MONITOR" = 1 ]; then
  echo "PASS: faker 배관 (controller_manager ↔ fake_opencr → /joint_states · EKF /odometry/filtered · Nav2 active · monitor_manager)"
else
  echo "PASS: faker 배관 (controller_manager ↔ fake_opencr → /joint_states · EKF /odometry/filtered · Nav2 active — monitor 끔)"
fi
