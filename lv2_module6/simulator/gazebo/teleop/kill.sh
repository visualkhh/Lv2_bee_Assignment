#!/bin/bash
# 키보드 텔레옵 종료 (호스트)
#   lv2_module6/develop/teleop/kill.sh
# NOTE: sh(dash)로 실행해도 bash로 재실행
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi
pkill -INT -f "teleop_twist_keyboar[d]" 2>/dev/null
sleep 1
pkill -9 -f "teleop_twist_keyboar[d]" 2>/dev/null || true
echo "텔레옵 정리됨"
