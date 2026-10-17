#!/bin/bash
# 테스트베드(faker — 대역 포함) 재시작 (컨테이너 안에서): ros-faker-restart
#   진짜 bringup으로 다시 띄우려면 ros-restart
#   ros-faker-restart [launch 인자]   띄워 둔 test-faker 0 이 있으면 거기에 재시작 요청 (/cmd/restart — 시뮬레이터 [ROS 재시작]과 같은 길),
#                              없으면 여기서 정리하고 test-faker 0 [launch 인자] 를 띄움 (인자 예: map:=saved map_name:=20261017_163731)
# 끄기·캐시 정리는 ros-kill (docker/ros-kill.sh) — 정리만 하려면 ros-kill
# NOTE: set -u 사용 금지 — setup.bash가 미설정 변수를 참조해서 죽음.
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
REQ=/debug/input/topic/cmd/restart/message

if pgrep -f "bin/test-faker 0" >/dev/null; then
  # 띄워 둔 test-faker 0 이 받아서 정리 → 다시 띄움 (파일 = /cmd/restart 토픽과 같은 자리)
  mkdir -p "$(dirname "$REQ")"
  python3 -c "import json,sys; json.dump({'type': 'std_msgs/msg/String', 'data': {'data': ' '.join(sys.argv[2:])}}, open(sys.argv[1] + '.tmp', 'w'))" "$REQ" "$@" \
    && mv -f "$REQ.tmp" "$REQ"
  echo "재시작 요청 → 띄워 둔 test-faker 0 (로그 /tmp/faker.log)"
  exit 0
fi

ros-kill   # 끄기 + 캐시 정리 — docker/ros-kill.sh
exec test-faker 0 "$@"
