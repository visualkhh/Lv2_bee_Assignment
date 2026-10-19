#!/bin/bash
# 테스트베드 처음부터 재시작 (호스트에서, docker/ 안에서 실행):
#   ./reset-testbed.sh [초]   (test-faker에 넘길 시간, 기본 0 = Ctrl+C까지 띄워 둠)
# 가벼운 재시작(삭제·빌드 없이)은 그냥: docker compose up -d && docker compose exec lyrical test-faker 0
DUR=${1:-0}
set -e
cd "$(dirname "$0")"
docker compose down
docker compose up -d --build
docker compose exec lyrical bash -c 'while [ ! -f /run/testbed-ready ]; do sleep 1; done'
docker compose exec -u root lyrical bash -c 'rm -rf /debug/* /ws/build /ws/install /ws/log'
docker compose exec lyrical bash -c "test-faker $DUR"
