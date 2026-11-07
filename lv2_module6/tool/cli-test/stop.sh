#!/usr/bin/env bash
# 실행 중인 cli-test(cli_test.py)를 종료한다.
set -eo pipefail

pids="$(pgrep -f 'cli-test/cli_test\.py' || true)"
if [ -z "$pids" ]; then
  echo "실행 중인 cli-test 없음"
  exit 0
fi

# shellcheck disable=SC2086
kill $pids 2>/dev/null || true
for _ in $(seq 1 50); do
  sleep 0.1
  [ -z "$(pgrep -f 'cli-test/cli_test\.py' || true)" ] && break
done

pids="$(pgrep -f 'cli-test/cli_test\.py' || true)"
if [ -n "$pids" ]; then
  echo "정상 종료 안 됨 → 강제 종료: $pids" >&2
  # shellcheck disable=SC2086
  kill -9 $pids 2>/dev/null || true
fi
echo "cli-test 종료됨"
