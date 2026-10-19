#!/bin/bash
# ROS 스택 전부 죽이기 + 캐시 정리 (컨테이너 안에서 실행): ros-kill
#   ros2 launch + 노드 + 좀비 spawner · stale 데몬 캐시 · Fast DDS 공유 메모리 잔재
#   1) launch에 SIGINT → 최대 20초 기다림 (RTAB-Map이 rtabmap.db를 닫을 시간 — 바로 죽이면 지도 끝부분이 덜 저장됨)
#   2) 안 죽은 것만 SIGKILL
#   3) ros2 daemon stop (죽은 launch 그래프 캐시 → 다음 실행에 지난 노드가 보이는 것 방지)
#   4) /dev/shm/fastdds_* 삭제 (강제 종료된 노드의 공유 메모리 통로가 남으면 보내는 쪽이 막힐 수 있음)
# 다시 띄우기: ros-restart(진짜 bringup) · ros-faker-restart(테스트베드). test-faker 0 의 재시작(/cmd/restart)도 이걸 부름.
# NOTE: set -u 사용 금지 — setup.bash가 미설정 변수를 참조해서 죽음.
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash 2>/dev/null || true

KILL_PAT='(/ws/install|/opt/ros/lyrical)/[^ ]*lib/|ros2 launch'   # ROS 노드 전부 + launch (docker/test-*.sh와 같은 기준)
if pgrep -f "$KILL_PAT" >/dev/null; then
  pkill -INT -f "ros2 launch" 2>/dev/null
  for _ in $(seq 40); do
    pgrep -f "$KILL_PAT" >/dev/null || break
    sleep 0.5
  done
  pkill -9 -f "$KILL_PAT" 2>/dev/null || true
fi
ros2 daemon stop >/dev/null 2>&1 || true
rm -rf /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null
echo "ROS 스택 정리됨 (남은 노드 $(pgrep -fc "$KILL_PAT")개 · daemon·공유메모리 캐시 삭제)"
