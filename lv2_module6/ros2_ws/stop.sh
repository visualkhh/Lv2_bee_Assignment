#!/bin/bash
# 떠 있는 ROS 전부 끄기 (start.sh의 짝): launch · 노드 · ros2 CLI · ros2 daemon
#
#   ./stop.sh          # SIGINT로 정상 종료 → 5초 안에 안 죽으면 SIGKILL
#
# 대상 = 실행 파일 경로가 ROS 설치 폴더(/opt/ros/<distro>/ · 이 워크스페이스·/ws의 install/) 아래인 프로세스 + `ros2 …` CLI.
# 패키지 이름으로 고르지 않는다 (새 패키지가 빠져 고아로 남음 — docker/lv2_module6/test-*.sh와 같은 기준).
# 같은 머신의 다른 ROS 작업도 같이 꺼진다.
# 끝에 /dev/shm의 Fast DDS 공유 메모리 잔재도 지운다 (노드당 35 MB — 강제 종료된 노드가 남기면 RAM을 먹고 SHM 포트 오류가 남).
#   ROS가 하나라도 안 죽었으면 지우지 않는다 (살아 있는 노드의 공유 메모리를 지우면 통신이 끊김).
HERE="$(cd "$(dirname "$0")" && pwd)"
# nvblox/cuVSLAM은 별도 workspace(~ /isaac_ros_ws)에 있어서 여기도 포함해야 한다.
SLAM_WS="${SLAM_WS:-$HOME/isaac_ros_ws}"
# [h] [n] — 패턴 글자가 이 스크립트를 부른 셸 명령줄과 겹쳐도 자기 자신은 안 걸리게
PAT="(/opt/ros/[^/ ]+|$HERE/install|$SLAM_WS/install|/ws/install)/[^ ]*li[b]/|bin/ros2 (launc[h]|ru[n]|topic|service|param|node|action|lifecycle|control)"

count() { pgrep -f "$PAT" | wc -l | tr -d ' '; }

n=$(count)
if [ "$n" = 0 ]; then
  echo "떠 있는 ROS 없음"
else
  echo "ROS 프로세스 ${n}개 — SIGINT"
  pkill -INT -f "$PAT"
  for _ in $(seq 10); do
    [ "$(count)" = 0 ] && break
    sleep 0.5
  done
  n=$(count)
  if [ "$n" != 0 ]; then
    echo "안 죽은 ${n}개 — SIGKILL"
    pkill -9 -f "$PAT"
  fi
fi
# ros2 daemon (그래프 캐시) — 남아 있으면 다음 실행에 지난 노드가 보임
if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
else
  pkill -f "ros2cli.daemon.daemoniz[e]" 2>/dev/null || true
fi
# 강제 종료된 참가자의 Fast DDS SHM 통로가 남으면 다음 state_manager가 포트 lock에 걸려
# publisher가 하나도 안 보일 수 있다 (/state/* publisher count 0). ROS가 모두 꺼진 뒤에만 지운다.
if [ "$(count)" = 0 ]; then
  rm -rf /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null || true
  echo "Fast DDS SHM 잔재 정리"
fi
echo "남은 ROS 프로세스: $(count)개"
