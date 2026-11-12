#!/usr/bin/env bash
# Isaac 미션 RViz — 어느 폴더에서 실행해도 mission.rviz를 찾는다 (rviz2 -d는 없는 파일이면 말없이 빈 화면으로 뜸) · 시뮬 시계 기준
exec rviz2 -d "$(dirname "$(realpath "$0")")/mission.rviz" --ros-args -p use_sim_time:=true "$@"
