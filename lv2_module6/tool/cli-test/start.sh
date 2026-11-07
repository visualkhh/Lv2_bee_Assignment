#!/usr/bin/env bash
# tool/cli-test 시작 — ROS 없이 가짜 ROS층으로 cli_test.py를 실행한다.
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
if [ -x "$HERE/.venv/bin/python" ]; then
  PY="$HERE/.venv/bin/python"
else
  PY="python3"
fi
exec "$PY" -u "$HERE/cli_test.py" "$@"
