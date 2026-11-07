"""ai-service 유닛 테스트 — 외부 호출 없음 (CLI·HTTP 전부 스텁).

  cd lv2_module6/ai-service && .venv/bin/python -m unittest discover -s tests
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "backends"))
