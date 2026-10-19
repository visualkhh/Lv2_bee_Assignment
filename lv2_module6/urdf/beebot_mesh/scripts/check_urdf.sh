#!/usr/bin/env bash
# beebot.urdf 일괄 검수: check_urdf → verify.py → check_xacro.py. 하나라도 실패하면 종료 코드 1.
# 메시 파일 누락은 verify.py가 INFO로만 알린다 (실패 아님).
# 사용: ./check_urdf.sh   (beebot.urdf.xacro가 없으면 make_xacro.py를 먼저 실행)
[ -n "${ROS_DISTRO:-}" ] || source /opt/ros/lyrical/setup.bash
set -euo pipefail
cd "$(dirname "$0")/.."
echo "== check_urdf"
check_urdf beebot.urdf >/dev/null
echo "PASS"
echo "== verify.py"
python3 scripts/verify.py
echo "== check_xacro.py"
python3 scripts/check_xacro.py
echo "== 전체 통과"
