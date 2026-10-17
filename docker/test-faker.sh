#!/bin/bash
# 더미 전체 흐름 검사 (컨테이너 안에서 실행): test-faker [초|0] [launch 인자...]
#   faker launch = bringup(🛠 노드) + ros_system(controller_manager ↔ 가상 시리얼 ↔ fake_opencr · EKF) + 대역(📦) + 파일 연동 → user_input 명령 한 번
#   /task_status 에 SUCCEEDED(state 1)가 오고 /robot_say 가 나오고,
#   controller_manager(fake_opencr S 상태)가 /joint_states(팔 5축 + 그리퍼)·EKF가 /odometry/filtered 를 내면 PASS
#   초: SUCCEEDED 뒤 더 돌릴 시간 (기본 1). 0이면 Ctrl+C까지 띄워 둠 (판정 없음)
#   예) test-faker                       정상 흐름
#       test-faker 1                        파지까지 전체 흐름 (진짜 move_group)
#       test-faker 0                      띄워 두고 다른 터미널에서 ros2 topic echo … (시뮬레이터 [ROS 재시작] = /cmd/restart 로 다시 띄움)
#       test-faker 1 mock_hardware:=true    OpenCR 없이 mock (물리 없음)
# 배관(joint_states · odom · Nav2 · monitor)은 항상 본다. 작업 흐름(카메라 → 인지 → 작업 성공)은 사용자가 호스트 시뮬레이터를 켰을 때만.
# 토픽·노드·패키지 기록은 monitor_manager가 debug/topic · node · pkg · output/topic 에 남긴다.
# NOTE: set -u 사용 금지 — setup.bash가 미설정 변수를 참조해서 죽음.
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
cd /ws || { echo "FAIL: /ws 없음"; exit 2; }

KILL_PAT='(/ws/install|/opt/ros/lyrical)/[^ ]*lib/|ros2 launch'   # ROS 노드 전부 + launch (패키지를 적으면 새 패키지가 빠져 고아로 남음)
pkill -f "$KILL_PAT" 2>/dev/null || true   # 빌드 전에 — 이전 실행의 노드가 남아 돌면 빌드도 느려짐
build-ws || exit 1   # 빌드 + install → /debug/install (docker/build-ws.sh)
# shellcheck disable=SC1091
source install/setup.bash


DUR=${1:-1}
shift 2>/dev/null
set -m
ros2 launch faker faker.launch.py "$@" > /tmp/faker.log 2>&1 & LAUNCH_PID=$!
JS=$(timeout 60 ros2 topic echo --once /joint_states sensor_msgs/msg/JointState 2>/dev/null)
# spawner가 기동 타이밍을 놓치면(FATAL) 컨트롤러 없이 돈다 — 죽은 spawner 치우고 최대 2회 재시도
for i in 1 2; do
  echo "$JS" | grep -q gripper_left_joint && break
  echo "spawner 재시도 ($i/2)..."
  pkill -f 'controller_manager/spawner' 2>/dev/null || true
  ros2 run controller_manager spawner joint_state_broadcaster imu_sensor_broadcaster diff_drive_controller rl_arm_correction arm_controller gripper_controller --controller-manager /controller_manager --controller-manager-timeout 120 --service-call-timeout 30 --param-file /ws/install/ros_system/share/ros_system/config/controllers.yaml --controller-ros-args '-r ~/cmd_vel:=/cmd_vel' --controller-ros-args '-r ~/odom:=/odom' --controller-ros-args '-r ~/imu:=/imu' > /tmp/spawner-retry.log 2>&1 || true
  JS=$(timeout 30 ros2 topic echo --once /joint_states sensor_msgs/msg/JointState 2>/dev/null)
done

# 카메라 = 사용자가 호스트에서 켠 시뮬레이터 (테스트가 띄우지 않음). 켜져 있으면 /robot_description을 받은 뒤 렌더 시작 → 최대 20초 기다림
# 꺼져 있으면 카메라 → 인지 → 작업 흐름 검사는 건너뛰고 배관(joint_states · odom · Nav2 · monitor)만 본다
META=/debug/input/hardware/camera/pole_cam.json
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
  pkill -9 -f "$KILL_PAT" 2>/dev/null || true
}

if [ "$DUR" = 0 ]; then
  # 띄워 두기 + 재시작 요청 받기: 시뮬레이터가 /cmd/restart 를 보내면 (= debug/input/topic/cmd/restart/message 파일)
  # ROS 전부 정상 종료 → 다시 띄움. ROS가 막혀도 되게 ROS가 아니라 이 스크립트가 파일을 직접 본다.
  # 값(data)이 비면 같은 인자, 있으면 그 인자만 바꾸고 나머지는 유지 (예: "map:=saved map_name:=20261017_163731" → urdf:= 등은 그대로,
  # "urdf:=beebot_normal" → map:= 등은 그대로). 같은 이름이면 새 값, 없던 이름이면 추가
  REQ=/debug/input/topic/cmd/restart/message
  ARGS=("$@")
  last=$(stat -c %Y "$REQ" 2>/dev/null || echo 0)
  echo "실행 중 (로그 /tmp/faker.log)... 종료는 Ctrl+C · 재시작 = /cmd/restart"
  trap 'echo; stop_all; exit 0' INT
  while kill -0 "$LAUNCH_PID" 2>/dev/null; do
    m=$(stat -c %Y "$REQ" 2>/dev/null || echo 0)
    if [ "$m" != "$last" ]; then
      last=$m
      new=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['data'].get('data', ''))" "$REQ" 2>/dev/null)
      read -ra NEW <<< "$new"
      # map:=을 바꾸면 예전 map_name:=은 버림 (안 주면 slam은 새 이름 · saved는 최근 — 예전 이름이 남으면 slam이 '있는 이름'으로 거부)
      [[ " ${NEW[*]}" == *" map:="* ]] && for i in "${!ARGS[@]}"; do [ "${ARGS[$i]%%:=*}" = map_name ] && unset 'ARGS[i]'; done
      for a in "${NEW[@]}"; do
        k=${a%%:=*}
        for i in "${!ARGS[@]}"; do [ "${ARGS[$i]%%:=*}" = "$k" ] && unset 'ARGS[i]'; done
        ARGS+=("$a")
      done
      ARGS=("${ARGS[@]}")
      echo "$(date +%T) 재시작 요청 (/cmd/restart): ${ARGS[*]:-기본}"
      ros-kill   # ROS 전부 정상 종료 + 캐시(공유메모리 잔재·daemon) 삭제 — docker/ros-kill.sh
      build-ws || echo "빌드 실패 — 이전 install 그대로 띄움"   # 고친 코드·설정도 반영 (실기 ros-restart와 같게)
      ros2 launch faker faker.launch.py "${ARGS[@]}" > /tmp/faker.log 2>&1 & LAUNCH_PID=$!
    fi
    sleep 1
  done
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
[ -s /debug/output/topic/joint_states/message ] || { echo "FAIL: monitor_manager 파일 없음 (/debug/output/topic/joint_states/message)"; exit 1; }
echo "기록 (monitor_manager): /debug/topic · /debug/node · /debug/pkg (그래프) · /debug/output/topic (message·image)"
if [ "$CAM_OK" = 1 ]; then
  [ -n "$DONE" ] || { echo "FAIL: /task_status SUCCEEDED 없음 (60초)"; tail -n 40 /tmp/faker.log; exit 1; }
  grep -q '🔊' /tmp/faker.log || { echo "FAIL: /robot_say 없음"; exit 1; }
  echo "PASS: faker 흐름 (시뮬레이터 카메라 → 인지 → vlm → coordinator → Nav2·grasp·move_group → task_status SUCCEEDED)"
else
  echo "건너뜀: 작업 흐름 — 시뮬레이터 꺼짐 ($META 갱신 없음). 흐름까지 보려면 호스트에서 시뮬레이터를 켜고 다시: cd lv2_module6/simulator && .venv/bin/python main.py"
fi
echo "PASS: faker 배관 (controller_manager ↔ fake_opencr → /joint_states · EKF /odometry/filtered · Nav2 active · monitor_manager)"
