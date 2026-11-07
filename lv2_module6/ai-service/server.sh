#!/bin/bash
# 서버: ./server.sh [--port 8000] [--host 127.0.0.1] [--config config.json]
cd "$(dirname "$0")" || exit 1
exec .venv/bin/python server.py "$@"
