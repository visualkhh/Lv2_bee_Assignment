#!/bin/bash
# 키보드 텔레옵 재시작 (호스트): kill → start
#   lv2_module6/develop/teleop/restart.sh [twist 토픽]
# NOTE: sh(dash)로 실행해도 bash로 재실행
if [ -z "$BASH_VERSION" ]; then exec bash "$0" "$@"; fi
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
"$HERE/kill.sh" >/dev/null 2>&1
exec "$HERE/start.sh" "$@"
