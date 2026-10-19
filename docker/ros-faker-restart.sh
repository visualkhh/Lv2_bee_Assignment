#!/bin/bash
# 테스트베드(faker — 대역 포함) 재시작 (컨테이너 안에서): ros-faker-restart
#   진짜 bringup으로 다시 띄우려면 ros-restart
#   ros-faker-restart [launch 인자]   떠 있는 state_manager에 재시작 요청 (/cmd/restart "all [인자]" — 시뮬레이터 [ROS 재시작]과 같은 길),
#                              state_manager가 없으면(ROS 막힘) 여기서 정리하고 test-faker 0 [launch 인자] 를 띄움 (인자 예: map:=saved map_name:=20261017_163731)
# 끄기·캐시 정리는 ros-kill (docker/ros-kill.sh) — 정리만 하려면 ros-kill
# NOTE: set -u 사용 금지 — setup.bash가 미설정 변수를 참조해서 죽음.
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
REQ=/debug/input/topic/cmd/restart/message

if pgrep -f "ros2 launch faker" >/dev/null && pgrep -f "lib/manager/state_manager" >/dev/null; then
  # 떠 있는 state_manager가 받아서 정리 → 같은 명령줄(바꾼 인자만 새 값)로 다시 띄움 — 시뮬레이터 [ROS 재시작]과 같은 길
  # (파일 → input_manager가 /cmd/restart 발행. 읽은 파일은 지우므로 두 번 안 감)
  mkdir -p "$(dirname "$REQ")"
  python3 -c "import json,sys; json.dump({'type': 'std_msgs/msg/String', 'data': {'data': ' '.join(['all'] + sys.argv[2:])}}, open(sys.argv[1] + '.tmp', 'w'))" "$REQ" "$@" \
    && mv -f "$REQ.tmp" "$REQ"
  echo "재시작 요청 → state_manager (/cmd/restart all $*) · 결과 /state/restart · 로그 /tmp/faker.log"
  exit 0
fi

# ROS가 없거나 막혀 state_manager가 없음 → 여기서 정리하고 새로 (띄워 둔 test-faker 0은 launch가 사라지면 끝남)
ros-kill   # 끄기 + 캐시 정리 — docker/ros-kill.sh
exec test-faker 0 "$@"
