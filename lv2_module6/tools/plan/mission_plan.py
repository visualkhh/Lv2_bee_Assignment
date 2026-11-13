#!/usr/bin/env python3
"""미션 배정 보기 — 장소(L1·L2·L3)마다 그룹(R/Y/G) · 퍽 수 · 어떤 퍽(tag 색 모양 · 출발 박스)을 놓는지.

  mission_manager가 회차마다 남기는 ~/.ros/mission/<시각>/signal.json · allocation.json · build.json · verify.json
  (plan.yaml log_dir) 중 가장 최근 회차를 2초마다 다시 읽음. 아직 안 정해진 건 '대기'로.
  실행: python3 tools/plan/mission_plan.py [--dir 회차폴더 또는 log_dir] [--once]   (표준 라이브러리만 · ROS 없이)
  TUI 미션 메뉴 'p'가 새 창(tmux 창 · 터미널)으로 띄움.
"""
import argparse
import json
import time
from pathlib import Path

LOG_DIR = Path('~/.ros/mission').expanduser()
GROUP = {'R': '빨강', 'Y': '노랑', 'G': '초록'}


def latest_run(d: Path) -> Path | None:
    """회차 폴더 자체면 그대로, log_dir이면 이름(YYYYmmdd_HHMMSS)이 가장 큰 회차."""
    if (d / 'photos').is_dir() or (d / 'signal.json').exists():
        return d
    runs = sorted(p for p in d.glob('*_*') if p.is_dir()) if d.is_dir() else []
    return runs[-1] if runs else None


def _load(run: Path, name: str):
    try:
        return json.loads((run / name).read_text())
    except (OSError, ValueError):   # 아직 없음 · 쓰는 중
        return None


def render(run: Path | None) -> list[str]:
    if run is None:
        return ['[미션 배정] 회차 없음 — 미션 시작 전 (~/.ros/mission 비었음)']
    out = [f'[미션 배정] 회차 {run.name}  ({run})']
    sig = _load(run, 'signal.json')
    if sig is None:
        out.append(' 신호 대기 — 바코드·신호를 읽으면 그룹별 개수·장소 배정이 나옴')
    elif sig.get('ok') is False:
        out.append(f' 신호 실패: {sig.get("error")} (raw {sig.get("raw") or "-"})')
    else:
        c = sig.get('counts', {})
        asc = '오름차순' if sig.get('ascending') else '내림차순'
        out.append(f' 바코드 {sig.get("barcode") or "-"} ({asc}) · 신호 R{c.get("R", 0)} Y{c.get("Y", 0)} G{c.get("G", 0)}'
                   f' · 순서 {",".join(sig.get("sequence", [])) or "-"} · 연속 {sig.get("cycles", "-")}')
        order = sig.get('order') or ''
        out.append(' 장소 배정 ' + (' · '.join(f'L{i + 1}={g}({c.get(g, 0)}개)' for i, g in enumerate(order))
                                   if order else '없음 (바코드 모름)'))
    alloc = _load(run, 'allocation.json')
    if alloc is None:
        out.append(' 퍽 배정 대기 — 시작 박스 조사(SURVEY)가 끝나면 나옴')
        return out
    pucks = {p['tag']: p for p in alloc.get('pucks', [])}
    out.append(f' 조사한 퍽 {len(pucks)}개 · 배정 {"OK" if alloc.get("ok") else "실패: " + str(alloc.get("why"))}')
    done = {(b.get('tag')): b.get('ok') for b in (_load(run, 'build.json') or {}).get('pucks', [])}
    verify = {v.get('location'): v.get('ok') for v in (_load(run, 'verify.json') or {}).get('locations', [])}
    for name, loc in sorted(alloc.get('locations', {}).items()):
        g = loc.get('group', '?')
        v = verify.get(name)
        out.append(f' {name}: {g} {GROUP.get(g, "")} {len(loc.get("pucks", []))}개'
                   + ('' if v is None else ' · 확인 ' + ('OK' if v else '실패')))
        for j in loc.get('pucks', []):
            p = pucks.get(j['tag'], {})
            slot = p.get('tray_slot')
            st = {True: '놓음', False: '실패'}.get(done.get(j['tag']), '')
            out.append(f'   tag {j["tag"]:>3} {p.get("color", "?"):<7} {p.get("shape", "?"):<10} 칸 {j.get("cell")}'
                       f' ← 박스 {p.get("box", "?")}{"" if slot is None else f" (적재함 {slot})"} {st}'.rstrip())
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--dir', type=Path, default=LOG_DIR, help='log_dir 또는 회차 폴더 (기본 ~/.ros/mission)')
    ap.add_argument('--once', action='store_true', help='한 번만 출력')
    a = ap.parse_args()
    if a.once:
        print('\n'.join(render(latest_run(a.dir.expanduser()))))
        return
    try:
        while True:
            text = '\n'.join(render(latest_run(a.dir.expanduser())))
            print('\033[H\033[J' + text + f'\n\n{time.strftime("%H:%M:%S")} 2초마다 갱신 · Ctrl+C 닫기', flush=True)
            time.sleep(2)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
