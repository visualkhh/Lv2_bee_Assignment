#!/usr/bin/env python3
"""퍽 배정 보기(tools/plan/mission_plan.py) 시험 — ROS 없이: python3 tools/cli/tests/test_mission_plan.py"""
import json
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'plan'))   # tools/plan
from mission_plan import latest_run, render  # noqa: E402

with tempfile.TemporaryDirectory() as d:
    log = Path(d)
    assert '회차 없음' in render(latest_run(log))[0]
    run = log / '20261112_101500'
    (run / 'photos').mkdir(parents=True)
    (log / '20261111_090000' / 'photos').mkdir(parents=True)   # 지난 회차 — 최근 것을 골라야 함
    assert latest_run(log) == run
    assert '신호 대기' in '\n'.join(render(run))
    (run / 'signal.json').write_text(json.dumps({
        'barcode': 'ASC', 'ascending': True, 'sequence': ['R', 'Y', 'G', 'G'], 'raw': 'RYGG', 'cycles': 2,
        'counts': {'R': 1, 'Y': 1, 'G': 2}, 'order': 'RYG'}))
    text = '\n'.join(render(run))
    assert 'L1=R(1개)' in text and 'L3=G(2개)' in text and '퍽 배정 대기' in text, text
    (run / 'allocation.json').write_text(json.dumps({
        'pucks': [{'tag': 5, 'color': 'red', 'shape': 'circle', 'box': 2, 'tray_slot': 0},
                  {'tag': 7, 'color': 'green', 'shape': 'square', 'box': 4, 'tray_slot': None}],
        'ok': True, 'why': '',
        'locations': {'L3': {'group': 'G', 'pucks': [{'tag': 7, 'cell': 0}]},
                      'L1': {'group': 'R', 'pucks': [{'tag': 5, 'cell': 4}]}}}))
    (run / 'build.json').write_text(json.dumps({'placed': 1, 'total': 2, 'pucks': [{'tag': 5, 'ok': True}]}))
    lines = render(run)
    text = '\n'.join(lines)
    assert text.index(' L1: R') < text.index(' L3: G'), text
    assert '← 박스 2 (적재함 0) 놓음' in text and '← 박스 4' in text, text
    print(text)
print('PASS test_mission_plan')
