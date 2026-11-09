#!/usr/bin/env bash
# 켜져 있는 RViz(rviz2)를 종료한다.
set -eo pipefail

pids="$(pgrep -x rviz2 || true)"
if [ -z "$pids" ]; then
  echo "실행 중인 RViz 없음"
  exit 0
fi

# shellcheck disable=SC2086
kill $pids 2>/dev/null || true
for _ in $(seq 1 50); do
  sleep 0.1
  [ -z "$(pgrep -x rviz2 || true)" ] && break
done

pids="$(pgrep -x rviz2 || true)"
if [ -n "$pids" ]; then
  echo "정상 종료 안 됨 → 강제 종료: $pids" >&2
  # shellcheck disable=SC2086
  kill -9 $pids 2>/dev/null || true
fi
echo "RViz 종료됨"
