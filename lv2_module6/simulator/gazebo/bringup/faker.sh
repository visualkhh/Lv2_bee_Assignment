#!/bin/bash
# 테스트베드 띄우기 (호스트): docker/test-faker.sh 로 연결 — 인자 그대로 전달, 없으면 0(상주)
#   lv2_module6/develop/bringup/faker.sh [초|0] [launch 인자...]
# NOTE: sh(dash)로 실행해도 bash로 재실행
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi
: "${ROS_DOMAIN_ID:=88}"; export ROS_DOMAIN_ID   # test-faker·rviz 셸과 통일 (명시했으면 존중)
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if [ $# -eq 0 ]; then set -- 0; fi
case " $* " in
  *\ use_monitor:=*) : ;;   # 사용자가 직접 지정하면 그대로
  *) set -- "$@" "use_monitor:=false" ;;   # develop 실행은 가볍게 (도커·CI는 기본 true)
esac
case " $* " in
  *\ use_gz_cameras:=*) : ;;   # 사용자가 직접 지정하면 그대로
  *) set -- "$@" "use_gz_cameras:=true" ;;   # develop 실행은 Gazebo 카메라 (fake_camera 끔 · 도커·CI는 기본 false)
esac
exec "$HERE/../../../docker/test-faker.sh" "$@"
