#!/bin/bash
# 테스트베드 띄우기 (호스트): docker/test-faker.sh 로 연결 — 인자 그대로 전달, 없으면 0(상주)
#   develop/faker.sh [초|0] [launch 인자...]
# NOTE: sh(dash)로 실행해도 bash로 재실행
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi
: "${ROS_DOMAIN_ID:=8}"; export ROS_DOMAIN_ID   # test-faker·rviz 셸과 통일 (명시했으면 존중)
HERE=$(dirname "${BASH_SOURCE[0]}")
[ "$HERE" = . ] && HERE="$PWD"
if [ $# -eq 0 ]; then set -- 0; fi
exec "$HERE/../../docker/test-faker.sh" "$@"
