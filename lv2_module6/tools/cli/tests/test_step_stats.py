#!/usr/bin/env python3
"""실기 기록 모아 보기(tools/plan/step_stats.py) 시험 — ROS 없이: python3 tools/cli/tests/test_step_stats.py"""
import json
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'plan'))   # tools/plan
from step_stats import pct, report, runs, skill_stats  # noqa: E402


def line(skill, ok, dur, reason='', retry=0, **ctx):
    return json.dumps({'t0': 0, 't1': dur or 0, 'dur_s': dur, 'phase': 'SURVEY', 'plan': 1, 'step': 1, 'steps': 1,
                       'skill': skill, 'target': 'x', 'relation': '', 'ok': ok, 'reason': reason, 'retry': retry, **ctx})


assert pct([], 0.5) is None and pct([3.0], 0.9) == 3.0
assert pct([1, 2, 3, 4], 0.5) == 2.5 and abs(pct(list(range(11)), 0.9) - 9.0) < 1e-9

with tempfile.TemporaryDirectory() as d:
    log = Path(d)
    assert '회차 없음' in report([log])[0]
    a, b = log / '20261113_100000', log / '20261113_110000'
    a.mkdir()
    b.mkdir()
    (log / 'empty_dir').mkdir()   # 기록 없는 폴더는 회차 아님
    (a / 'steps.jsonl').write_text('\n'.join([
        line('pick', True, 10.0, box=1, tag=3), line('pick', False, 20.0, 'grasp_empty', tag=3),
        line('pick', True, 12.0, retry=1, tag=3), line('navigate', False, None, 'nav_no_progress'),
        line('approach', True, 8.0)]) + '\n{"broken')   # 끊긴 마지막 줄은 버림
    (b / 'steps.jsonl').write_text('\n'.join([line('pick', False, 30.0, 'grasp_empty'), line('load', True, 11.0),
                                              line('approach', False, 9.0, 'approach_blocked')]) + '\n')
    (b / 'phases.jsonl').write_text('\n'.join(json.dumps(x) for x in [
        {'phase': 'BARCODE', 'dur_s': 25.0, 'ok': False, 'reason': 'barcode_missing', 'next': 'READ'},
        {'phase': 'SURVEY', 'dur_s': 300.0, 'ok': True, 'reason': '', 'next': 'BUILD'},
        {'phase': 'MISSION', 'dur_s': 900.0, 'ok': True, 'reason': '', 'results': ['BARCODE: barcode_missing']}]) + '\n')
    assert runs([log]) == [a, b] and runs([a]) == [a]

    rows = [json.loads(x) for r in (a, b) for x in (r / 'steps.jsonl').read_text().splitlines() if not x.startswith('{"b')]
    st = skill_stats(rows)
    p = st['pick']
    assert (p['n'], p['ok']) == (4, 2) and p['rate'] == 0.5, p
    assert p['med'] == 16.0 and p['reasons'] == {'grasp_empty': 2}, p
    assert st['navigate']['med'] is None and st['navigate']['reasons'] == {'nav_no_progress': 1}   # 시간 모름(null)은 뺌

    text = '\n'.join(report([log]))
    assert '회차 2개' in text, text
    assert 'grasp_empty 2' in text and 'pick 1' in text, text           # 실패 이유 · 다시 시도한 단계
    assert 'BARCODE' in text and 'barcode_missing 1' in text, text
    assert 'PROBS.update({"pick": 0.5, "load": 1.0, "repick": 0.5})' in text, text
    assert 'SKILL_S.update({PICK: 16.0, LOAD: 11.0, REPICK: 16.0})' in text, text
    assert 'DOCK_S = 8.5' in text, text
    assert '기록 없음: stand, unload, place' in text, text
print('ok test_step_stats')
