#!/usr/bin/env python3
"""README Notion 활동 리포트 생성기.

데이터: Notion 공개 페이지를 Playwright로 긁은 텍스트 + 마지막 로컬 스냅샷.
  - 개인 일지: 작성자·날짜·작업·상태
  - 회의록: 날짜·제목 (작성자 컬럼이 없어 팀 단위)
  - 트러블슈팅: 날짜·상태·문제·상세 (작성자 컬럼이 없어 팀 단위)
  - 투두리스트: 공개 표에 보이는 항목·참여 인원

사용:
  python3 .github/scripts/notion_stats.py            # README.md의 NOTION_STATS 블록 갱신
  python3 .github/scripts/notion_stats.py --stdout   # 화면에만 출력
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import unicodedata
from datetime import datetime, timezone
from pathlib import Path
from zoneinfo import ZoneInfo

KST = ZoneInfo('Asia/Seoul')
NOW = datetime.now(timezone.utc)
BAR_W = 26

START = '<!-- NOTION_STATS:START -->'
END = '<!-- NOTION_STATS:END -->'

NOTION_URL = 'https://app.notion.com/p/teamsparta/B-_BEE-3de2dc3ef51480ce978edf40651c6b35'
NOTION_TEXT = Path(os.environ.get('NOTION_TEXT', '.notion_cache.txt'))
NOTION_SNAPSHOT = Path(os.environ.get('NOTION_SNAPSHOT', 'notion_snapshot.json'))
NOTION_AUTHOR = {
    '정혁': 'cowsjh', '송정혁': 'cowsjh',
    '김현하': 'visualkhh',
    'Hyeongjung Kwon': 'JuneKunst', '권형중': 'JuneKunst',
    '승혜 전': 'SeungHye-J', '전승혜': 'SeungHye-J',
}
NOTION_TEAM = {
    'visualkhh': '김현하',
    'JuneKunst': '권형중',
    'cowsjh': '송정혁',
    'SeungHye-J': '전승혜',
}


def bar(value: float, top: float, width: int = BAR_W) -> str:
    n = 0 if top <= 0 else int(round(value / top * width))
    n = max(0, min(width, n))
    return '█' * n + '·' * (width - n)


def disp_width(s: str) -> int:
    return sum(2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in s)


def pad(s: str, width: int) -> str:
    return s + ' ' * max(0, width - disp_width(s))


FIG = '\u2007'   # figure space — 코드 스팬에서 접히지 않는 숫자 폭 공백


def rjust_fig(s: str, width: int) -> str:
    return s.rjust(width).replace(' ', FIG)


def person_label(p: dict, width: int = 30) -> str:
    name = p.get('name') or ''
    label = f'{p["login"]} ({name})' if name and name != p['login'] else p['login']
    return pad(label, width)


def name_of(p: dict) -> str:
    name = p.get('name') or ''
    return f'{p["login"]} ({name})' if name and name != p['login'] else p['login']


def mini(value: int, top: int, width: int = 6) -> str:
    if not value:
        return '·' * width
    n = max(1, min(width, int(round(value / top * width)))) if top else 0
    return '█' * n + '·' * (width - n)


def read_notion(path: Path) -> dict | None:
    """Playwright로 긁은 Notion innerText → 개인 일지/회의록/트러블슈팅/투두."""
    if not path.exists():
        return None
    raw = path.read_text(encoding='utf-8')
    try:
        text = json.loads(raw) if raw.lstrip().startswith('"') else raw
    except Exception:
        text = raw
    lines = [l.strip() for l in text.splitlines() if l.strip()]
    if not any('개인 일지' in l for l in lines):
        return None

    def idx(marker: str) -> int:
        return next(i for i, l in enumerate(lines) if marker in l)

    date_re = re.compile(r'^(\d{4})년 (\d{1,2})월 (\d{1,2})일(?: → .*)?$')

    def ymd(m: re.Match) -> str:
        return f'{int(m.group(1)):04d}-{int(m.group(2)):02d}-{int(m.group(3)):02d}'

    status_set = {'완료', '진행 중', '진행중', '해결 완료', '해결 필요'}

    # 개인 일지
    js = idx('✦ 개인 일지')
    journal = []
    i = js
    while i < len(lines):
        m = date_re.match(lines[i])
        if not m:
            i += 1
            continue
        raw_date = lines[i]
        date = ymd(m)
        end_m = re.search(r'→\s*(\d{4})년 (\d{1,2})월 (\d{1,2})일', raw_date)
        date_end = (f'{int(end_m.group(1)):04d}-{int(end_m.group(2)):02d}-{int(end_m.group(3)):02d}'
                    if end_m else date)
        i += 1
        while i < len(lines) and lines[i] in ('김', '') and i + 1 < len(lines):
            i += 1
        author = lines[i] if i < len(lines) else ''
        i += 1
        if author == '김' and i < len(lines):
            author = lines[i]
            i += 1
        work = lines[i] if i < len(lines) else ''
        i += 1
        status = ''
        if i < len(lines) and lines[i] in status_set:
            status = lines[i]
            i += 1
        journal.append({
            'date': date, 'date_end': date_end, 'author': author, 'work': work, 'status': status,
            'login': NOTION_AUTHOR.get(author),
        })

    # 회의록
    ms, me = idx('✦ 회의록'), idx('✦ 프로젝트 타임라인')
    meetings = []
    for j in range(ms, me):
        m = date_re.match(lines[j])
        if m and j + 1 < me:
            meetings.append({'date': ymd(m), 'title': lines[j + 1]})

    # 트러블슈팅
    ts, te = idx('✦ 트러블슈팅'), idx('✦ 개인 일지')
    trouble = []
    i = ts
    while i < te:
        m = date_re.match(lines[i])
        if not m:
            i += 1
            continue
        date = ymd(m)
        i += 1
        status = ''
        if i < te and lines[i] in status_set:
            status = lines[i]
            i += 1
        problem = lines[i] if i < te else ''
        i += 1
        details = []
        while i < te and not date_re.match(lines[i]) and lines[i] not in status_set:
            details.append(lines[i])
            i += 1
        trouble.append({'date': date, 'status': status, 'problem': problem, 'details': details})

    # 투두리스트 (공개 표에 보이는 항목만)
    us, ue = idx('✦ 투두리스트'), idx('✦ 트러블슈팅')
    todos = []
    cur = None
    skip = {'투두리스트', '투두리스트 날짜', '회의록', '사람'}
    for j in range(us, ue):
        line = lines[j]
        if line in skip or line.startswith('(복제'):
            continue
        if date_re.match(line):
            continue
        if line in NOTION_AUTHOR or line == '김':
            if line == '김':
                continue
            if cur is not None:
                cur['people'].append(line)
            continue
        if line.endswith('오늘의 투두리스트') or line.endswith('투두리스트'):
            # 날짜별 투두리스트 페이지 = 그 사람들이 참여한 항목
            cur = {'title': line, 'people': []}
            todos.append(cur)
            continue
        cur = {'title': line, 'people': []}
        todos.append(cur)

    return {'journal': journal, 'meetings': meetings, 'trouble': trouble, 'todos': todos}


def read_snapshot(path: Path) -> dict | None:
    """로컬에서 긁어 둔 Notion 스냅샷 (CI 비로그인에서 안 보이는 표 보완)."""
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return None


def canonical_login(login: str) -> str:
    """NOTION_TEAM의 표기(대소문자 포함)로 로그인을 정규화한다."""
    for known in NOTION_TEAM:
        if known.lower() == login.lower():
            return known
    return login


def update_readme(path: Path, report: str) -> None:
    """README의 NOTION_STATS 마커 사이를 리포트로 교체한다."""
    text = path.read_text(encoding='utf-8') if path.exists() else ''
    if START in text and END in text:
        before = text[:text.index(START)]
        after = text[text.index(END) + len(END):]
        text = before.rstrip() + '\n\n' + report + after
    else:
        text = text.rstrip() + '\n\n' + report + '\n'
    path.write_text(text, encoding='utf-8')


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--readme', default='README.md')
    ap.add_argument('--stdout', action='store_true', help='README를 쓰지 않고 출력만')
    args = ap.parse_args()

    try:
        live = read_notion(NOTION_TEXT)
    except Exception as e:
        print(f'Notion 파싱 실패: {type(e).__name__}: {e}', file=sys.stderr)
        live = None
    snap = read_snapshot(NOTION_SNAPSHOT)
    if live is None and snap is None:
        print('Notion 수집/스냅샷 없음 — 리포트를 생성하지 않습니다.', file=sys.stderr)
        return 2
    snapshot_at = (snap or {}).get('captured_at')

    def pick(key: str) -> list:
        if live and live.get(key):
            return live[key]
        if snap and snap.get(key):
            return snap[key]
        return []

    notion = {k: pick(k) for k in ('journal', 'meetings', 'trouble', 'todos')}
    if not any(notion.values()):
        print('Notion 데이터가 비어 있음 — 리포트를 생성하지 않습니다.', file=sys.stderr)
        return 2

    solved = sum(1 for t in notion['trouble'] if t['status'] == '해결 완료')
    need = sum(1 for t in notion['trouble'] if t['status'] == '해결 필요')
    todo_items = [t for t in notion['todos'] if t['people']]

    # 사람 키 = 로그인 (NOTION_AUTHOR 매핑), 이름은 NOTION_TEAM 우선
    people: dict[str, dict] = {}

    def person(login: str, fallback_name: str = '') -> dict:
        canon = canonical_login(login)
        p = people.setdefault(canon.lower(), {
            'login': canon,
            'name': NOTION_TEAM.get(canon) or fallback_name or canon,
            'journal': 0, 'last': None, 'done': 0, 'doing': 0, 'todos': 0,
        })
        return p

    guests = []
    for e in notion['journal']:
        login = e.get('login') or ''
        if not login:
            guests.append(e)
            continue
        p = person(login, e['author'])
        p['journal'] += 1
        p['last'] = max(p['last'] or '', e.get('date_end') or e['date'])
        if e['status'] == '완료':
            p['done'] += 1
        elif e['status'] in ('진행 중', '진행중'):
            p['doing'] += 1

    for t in notion['todos']:
        for author in t['people']:
            login = NOTION_AUTHOR.get(author)
            if not login:
                continue
            person(login, author)['todos'] += 1

    for p in people.values():
        p['weight'] = p['journal'] + p['todos']
    total_weight = sum(p['weight'] for p in people.values())
    for p in people.values():
        p['pct'] = p['weight'] / total_weight * 100 if total_weight else 0.0

    pct_rows = sorted(people.values(), key=lambda p: (-p['pct'], -p['journal'], p['login']))
    user_rows = sorted(people.values(), key=lambda p: (-p['weight'], -p['journal'], p['login']))
    journal_rows = sorted(people.values(), key=lambda p: (-p['journal'], p['login']))
    todo_rows = sorted(people.values(), key=lambda p: (-p['todos'], p['login']))

    top_journal = max((p['journal'] for p in people.values()), default=0)
    top_todos = max((p['todos'] for p in people.values()), default=0)

    lines: list[str] = []
    add = lines.append
    add(START)
    add('## 📝 Notion 활동')
    add('')
    add('### 🧮 기여도 (데이터 기반)')
    add('')
    add('> 개인 일지 + 투두 합계의 **기계적 비율**입니다. 막대는 **1위=100%** 기준.')
    add('')
    if pct_rows:
        add('| 이름 | 기여도 | 그래프 |')
        add('|:---|---:|:---:|')
        top_pct = max((p['pct'] for p in pct_rows), default=100)
        for p in pct_rows:
            add(f'| {name_of(p)} | {p["pct"]:.1f}% | `{bar(p["pct"], top_pct, 16)}` |')
    else:
        add('- 집계 가능한 개인 일지/투두가 없습니다.')
    add('')
    add('### 📊 요약')
    add('')
    add(f'- 개인 일지 **{len(notion["journal"])}** (게스트 {len(guests)}) · '
        f'회의록 **{len(notion["meetings"])}** · 트러블슈팅 **{len(notion["trouble"])}** '
        f'(해결 완료 {solved} / 해결 필요 {need}) · 투두 **{len(todo_items)}**')
    add(f'- 사람별 집계 대상 **{len(people)}명** · 일지+투두 합계 **{total_weight}**')
    add('')
    add(f'> 데이터: Notion 공개 표(투두) + 마지막 로컬 스냅샷({snapshot_at or "없음"}) — '
        f'개인 일지·회의록·트러블슈팅 · 갱신 {NOW.astimezone(KST).strftime("%Y-%m-%d %H:%M")} KST')
    add(f'> 🔗 원본: [B팀_BEE 팀 페이지]({NOTION_URL}) (공개된 기록만 집계)')
    if guests:
        add(f'> 게스트(팀 외) 일지 {len(guests)}건은 사람별 기여도 표에서 제외했습니다.')
    add('')
    add('### 👥 사람별 기록')
    add('')
    add('| 기여자 | 개인 일지 | 최근 작성 | 완료/진행 | 투두 |')
    add('|:---|:---|:---|:---|:---|')
    for p in user_rows:
        jn = rjust_fig(str(p['journal']), 6)
        tn = rjust_fig(str(p['todos']), 6)
        add(f'| {name_of(p)} | `{mini(p["journal"], top_journal)} {jn}` | '
            f'{p["last"] or "-"} | {p["done"]}/{p["doing"]} | '
            f'`{mini(p["todos"], top_todos)} {tn}` |')
    if not user_rows:
        add('| - | - | - | - | - |')
    add('')
    add('> ✅ 투두는 공개 표의 항목별 참여 인원을 각각 세므로, 인원 합계가 항목 수와 다를 수 있습니다.')
    add('')
    add('<details>')
    add(f'<summary>🧯 트러블슈팅 (팀) — {len(notion["trouble"])}건 · 해결 완료 {solved} / 해결 필요 {need}</summary>')
    add('')
    add(f'공개 표의 트러블슈팅 **{len(notion["trouble"])}건** — 해결 완료 **{solved}** / 해결 필요 **{need}**.')
    add('')
    add('| 상태 | 건수 | 그래프 |')
    add('|---|---:|---|')
    top_status = max(solved, need, len(notion['trouble']) - solved - need, 1)
    add(f'| 해결 완료 | {solved} | `{bar(solved, top_status, 8)}` |')
    add(f'| 해결 필요 | {need} | `{bar(need, top_status, 8)}` |')
    other = len(notion['trouble']) - solved - need
    if other:
        add(f'| 기타 | {other} | `{bar(other, top_status, 8)}` |')
    add('')
    for t in sorted(notion['trouble'], key=lambda x: x['date']):
        add(f'- {t["date"]} · **{t["status"]}** · {t["problem"]}')
    if not notion['trouble']:
        add('- 없음')
    add('')
    add('> 회의록·트러블슈팅은 공개 표에 작성자 컬럼이 없어 팀 단위로만 셉니다.')
    add('')
    add('</details>')
    add('')
    add('### 📓 개인 일지')
    add('')
    if journal_rows:
        add('```text')
        for p in journal_rows:
            add(f'{person_label(p)} {bar(p["journal"], top_journal)} {p["journal"]:>4}')
        add('```')
    else:
        add('- 없음')
    add('')
    add('### ✅ 투두 (공개 표)')
    add('')
    if todo_rows:
        add('```text')
        for p in todo_rows:
            add(f'{person_label(p)} {bar(p["todos"], top_todos)} {p["todos"]:>4}')
        add('```')
    else:
        add('- 없음')
    add('')
    if notion['journal']:
        add(f'<details><summary>📓 개인 일지 전체 {len(notion["journal"])}건 — 날짜·작성자·작업·상태</summary>')
        add('')
        for e in sorted(notion['journal'], key=lambda x: (x['date'], x['author'])):
            login = e.get('login')
            canon = canonical_login(login) if login else ''
            who = NOTION_TEAM.get(canon, e['author']) if login else f'{e["author"]} (게스트)'
            suffix = f' ({canon})' if login else ''
            span = e['date'] if e['date_end'] == e['date'] else f'{e["date"]} → {e["date_end"]}'
            add(f'- {span} · {who}{suffix} · {e["work"]} · {e["status"] or "-"}')
        add('')
        add('</details>')
        add('')
    if notion['meetings']:
        add(f'<details><summary>🗓️ 회의록 전체 {len(notion["meetings"])}건 — 날짜·제목</summary>')
        add('')
        for m in sorted(notion['meetings'], key=lambda x: x['date']):
            add(f'- {m["date"]} · {m["title"]}')
        add('')
        add('</details>')
        add('')
    if notion['trouble']:
        add(f'<details><summary>🧯 트러블슈팅 전체 {len(notion["trouble"])}건 '
            f'(해결 완료 {solved} / 해결 필요 {need})</summary>')
        add('')
        for t in sorted(notion['trouble'], key=lambda x: x['date']):
            add(f'- {t["date"]} · {t["status"]} · {t["problem"]}')
            if t['details']:
                add(f'  - {" / ".join(t["details"])}')
        add('')
        add('</details>')
        add('')
    if todo_items:
        add(f'<details><summary>✅ 투두리스트 공개 항목 {len(todo_items)}건</summary>')
        add('')
        for t in todo_items:
            add(f'- {t["title"]} — {", ".join(t["people"])}')
        add('')
        add('</details>')
        add('')
    add(END)

    report = '\n'.join(lines)
    if args.stdout:
        print(report)
        return 0

    path = Path(args.readme)
    update_readme(path, report)
    print(f'updated {path} ({len(report)} bytes)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
