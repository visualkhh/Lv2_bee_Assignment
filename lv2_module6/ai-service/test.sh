#!/bin/bash
# 유닛 테스트: .venv/bin/python -m unittest discover -s tests (외부 호출 없음)
cd "$(dirname "$0")" || exit 1
exec .venv/bin/python -m unittest discover -s tests
