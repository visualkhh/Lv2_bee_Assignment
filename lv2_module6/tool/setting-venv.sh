#!/usr/bin/env bash
# setting-venv.sh — tool/.venv 만들기 + requirements.txt 설치. 처음 한 번 (requirements가 바뀌면 다시).
#   ./setting-venv.sh
# venv는 ROS와 같은 파이썬으로, --system-site-packages로 만든다 —
#   rclpy·메시지(ROS setup.bash의 PYTHONPATH)와 apt로 깐 패키지를 그대로 보고, 없는 것만 venv에 깐다.
# run-cli · run-cli-test는 .venv가 있으면 쓰고, 없으면 시스템 python3로 돈다 (도커 /ws/tool은 읽기 전용이라 .venv 없음 — 이미지에 이미 설치됨).
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
VENV="$HERE/.venv"
PY="${PYTHON:-python3}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME:-lyrical}/setup.bash"
if [ -f "$ROS_SETUP" ] && [ -z "${PYTHON:-}" ]; then
  PY=/usr/bin/python3   # ROS 파이썬 확장(rclpy)은 시스템 파이썬 버전에 맞춰 빌드됨
fi

if [ ! -x "$VENV/bin/python" ]; then
  echo "venv 만들기: $VENV ($("$PY" --version 2>&1))"
  "$PY" -m venv --system-site-packages "$VENV" \
    || { echo "FAIL: venv 생성 실패 — Ubuntu면 sudo apt install python3-venv" >&2; exit 1; }
fi
"$VENV/bin/python" -m pip install --upgrade pip -q
"$VENV/bin/python" -m pip install -r "$HERE/requirements.txt"
"$VENV/bin/python" - <<'PY'
import importlib
for m in ("numpy", "matplotlib", "yaml"):
    mod = importlib.import_module(m)
    print(f"  {m} {getattr(mod, '__version__', '')}")
PY
echo "OK: $VENV — run-cli · run-cli-test가 이걸 씁니다"
