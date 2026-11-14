#!/usr/bin/env python3
"""실기 미션 기록 모아 보기 — skill별 시도 · 성공률 · 걸린 시간(중앙값·p90) · 실패 이유 + 단계별 시간.

  mission_manager가 회차마다 남기는 <log_dir>/<시각>/steps.jsonl (계획 단계마다 한 줄) · phases.jsonl (단계·미션 끝마다)을 읽음.
  실행: python3 tools/plan/step_stats.py [회차 폴더 또는 log_dir …]   (기본 ~/.ros/mission 전부 · 표준 라이브러리만)
  끝에 isaac/learning/plan/mission_sim.py PROBS · SKILL_S에 붙여 넣을 값을 찍음 (이름은 mission_sim 기준 — 실기 skill 이름과 다름).
"""
import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path

LOG_DIR = Path('~/.ros/mission').expanduser()
# mission_sim 이름 ← 실기 skill (PROBS 키, SKILL_S 키). REPICK(놓인 퍽 다시 집기)은 pick과 같은 skill
SIM = {'pick': ('pick', 'PICK'), 'stand': ('stand', 'STAND'), 'load': ('load', 'LOAD'),
       'unload': ('unload', 'UNLOAD'), 'place': ('place_ok', 'PLACE')}


def runs(paths):
    """회차 폴더(steps.jsonl · phases.jsonl이 있는 곳) 목록 — log_dir을 주면 그 아래 회차 전부."""
    out = []
    for p in map(Path, paths):
        if (p / 'steps.jsonl').exists() or (p / 'phases.jsonl').exists():
            out.append(p)
        elif p.is_dir():
            out += sorted(d for d in p.iterdir() if (d / 'steps.jsonl').exists() or (d / 'phases.jsonl').exists())
    return out


def read_jsonl(path):
    rows = []
    try:
        lines = path.read_text().splitlines()
    except OSError:
        return rows
    for line in lines:
        try:
            rows.append(json.loads(line))
        except ValueError:   # 쓰는 중 끊긴 마지막 줄
            pass
    return rows


def pct(values, q):
    """q 분위수 (선형 보간) — 값이 없으면 None."""
    v = sorted(values)
    if not v:
        return None
    k = (len(v) - 1) * q
    i = int(k)
    return v[i] if i + 1 >= len(v) else v[i] + (v[i + 1] - v[i]) * (k - i)


def skill_stats(steps):
    """skill → {n, ok, rate, med, p90, reasons}. 시간은 성공·실패 모든 시도 (mission_sim은 시도마다 SKILL_S를 씀)."""
    by = defaultdict(list)
    for s in steps:
        by[s.get('skill', '?')].append(s)
    out = {}
    for skill, rows in sorted(by.items()):
        durs = [r['dur_s'] for r in rows if isinstance(r.get('dur_s'), (int, float))]
        ok = sum(1 for r in rows if r.get('ok'))
        out[skill] = {'n': len(rows), 'ok': ok, 'rate': ok / len(rows), 'med': pct(durs, 0.5), 'p90': pct(durs, 0.9),
                      'reasons': Counter(r.get('reason') or '?' for r in rows if not r.get('ok'))}
    return out


def phase_stats(phases):
    by = defaultdict(list)
    for p in phases:
        by[p.get('phase', '?')].append(p)
    return {name: {'n': len(rows), 'ok': sum(1 for r in rows if r.get('ok')),
                   'med': pct([r['dur_s'] for r in rows if isinstance(r.get('dur_s'), (int, float))], 0.5),
                   'reasons': Counter(r.get('reason') or '?' for r in rows if not r.get('ok'))}
            for name, rows in by.items()}


def sim_block(stats, n_runs):
    """mission_sim.py에 붙여 넣을 값 — 시도가 없는 skill은 빼고 주석으로."""
    probs, times, missing = {}, {}, []
    for skill, (pkey, skey) in SIM.items():
        s = stats.get(skill)
        if not s:
            missing.append(skill)
            continue
        probs[pkey] = round(s['rate'], 3)
        if s['med'] is not None:
            times[skey] = round(s['med'], 1)
    if 'PICK' in times:
        times['REPICK'] = times['PICK']
    if 'pick' in probs:
        probs['repick'] = probs['pick']
    n = sum(s['n'] for s in stats.values())
    lines = [f'# 실기 {n_runs}회 · 단계 {n}개 (tools/plan/step_stats.py) — 시간은 시도마다 중앙값 [s]',
             f'PROBS.update({json.dumps(probs)})',
             f'SKILL_S.update({{{", ".join(f"{k}: {v}" for k, v in times.items())}}})']
    dock = stats.get('approach')
    if dock and dock['med'] is not None:
        lines.append(f'DOCK_S = {dock["med"]:.1f}  # approach 중앙값 (다가가기 한 번)')
    if missing:
        lines.append(f'# 기록 없음: {", ".join(missing)} (그대로 둠)')
    lines.append('# place_ok = place skill 성공률 (놓은 뒤 10 mm 확인 결과는 build.json)')
    return lines


def fmt_s(v):
    return '-' if v is None else f'{v:.1f}'


def report(paths):
    rs = runs(paths)
    steps, phases = [], []
    for r in rs:
        steps += read_jsonl(r / 'steps.jsonl')
        phases += read_jsonl(r / 'phases.jsonl')
    if not rs:
        return ['회차 없음 (steps.jsonl · phases.jsonl) — ' + ', '.join(map(str, paths))]
    out = [f'회차 {len(rs)}개: {rs[0].name} … {rs[-1].name}', '',
           f'{"skill":<12}{"시도":>5}{"성공":>5}{"성공률":>8}{"중앙 s":>8}{"p90 s":>8}  실패 이유']
    stats = skill_stats(steps)
    for skill, s in stats.items():
        why = ' · '.join(f'{k} {v}' for k, v in s['reasons'].most_common(4))
        out.append(f'{skill:<12}{s["n"]:>5}{s["ok"]:>5}{s["rate"]:>8.0%}{fmt_s(s["med"]):>8}{fmt_s(s["p90"]):>8}  {why}')
    retried = Counter(s.get('skill') for s in steps if s.get('retry'))
    if retried:
        out.append('다시 시도한 단계: ' + ' · '.join(f'{k} {v}' for k, v in retried.most_common()))
    out += ['', f'{"단계":<12}{"횟수":>5}{"성공":>5}{"중앙 s":>8}  실패 이유']
    for name, s in phase_stats(phases).items():
        why = ' · '.join(f'{k} {v}' for k, v in s['reasons'].most_common(3))
        out.append(f'{name:<12}{s["n"]:>5}{s["ok"]:>5}{fmt_s(s["med"]):>8}  {why}')
    return out + ['', '# --- isaac/learning/plan/mission_sim.py (PROBS · SKILL_S 정의 뒤에 붙임)'] + sim_block(stats, len(rs))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('paths', nargs='*', default=[str(LOG_DIR)], help='회차 폴더 또는 log_dir (여러 개)')
    print('\n'.join(report(ap.parse_args().paths)))


if __name__ == '__main__':
    main()
