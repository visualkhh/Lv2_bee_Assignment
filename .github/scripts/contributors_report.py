#!/usr/bin/env python3
"""README 기여자 활동 리포트 생성기.

⚠️ 이 스크립트는 **AI가 작성한 자동 생성기**입니다. 사람이 수치를 직접 쓰지 않고,
   아래 데이터를 기계적으로 집계해 README의 CONTRIBUTORS 블록을 갱신합니다.

데이터: `git log --all` (모든 원격 브랜치) + GitHub API (이슈·PR·리뷰) + Notion 공개 페이지(선택).
  - 커밋/증감/마지막 커밋: git log --all (merge 제외)
  - 이슈 처리량·할당·누적·정체: GitHub issues API (댓글 없는 닫힘은 처리량에서 제외)
  - PR·머지·리뷰 참여: GitHub pulls/reviews API
  - Notion 개인 일지·회의록·트러블슈팅·투두: Playwright로 긁은 공개 페이지 텍스트
  - 봇(Copilot 등) 제외

사용:
  python3 .github/scripts/contributors_report.py            # README.md 갱신
  python3 .github/scripts/contributors_report.py --stdout   # 화면에만 출력
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import unicodedata
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timedelta, timezone
from pathlib import Path
from zoneinfo import ZoneInfo

KST = ZoneInfo('Asia/Seoul')
NOW = datetime.now(timezone.utc)
STALE_DAYS = 14
BAR_W = 26
SPARK = '▁▂▃▄▅▆▇█'

START = '<!-- CONTRIBUTORS:START -->'
END = '<!-- CONTRIBUTORS:END -->'

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

# git 이메일 → GitHub 로그인 (같은 사람이 여러 이름/이메일로 커밋해도 한 명으로 합친다)
EMAIL_TO_LOGIN = {
    'visualkhh@gmail.com': 'visualkhh',
    'hjkwon9401@gmail.com': 'JuneKunst',
    'ef70024@gmail.com': 'cowsjh',
    'ivory0130@gmail.com': 'SeungHye-J',
}
# README 담당자 표기와 맞춘 이름 (GitHub 프로필 이름이 비어 있거나 영어만 있을 때)
NAME_OVERRIDE = {
    'visualkhh': '김현하',
    'JuneKunst': '권형중',
    'cowsjh': '송정혁',
    'SeungHye-J': '전승혜',
}
# noreply 주소는 정규식으로 login을 뽑는다


def is_bot(login: str | None, user: dict | None = None) -> bool:
    if user and user.get('type') == 'Bot':
        return True
    if not login:
        return False
    return login.endswith('[bot]') or login in {'Copilot', 'copilot-swe-agent'}


def gh_json(endpoint: str) -> list:
    """gh api --paginate --slurp → 모든 페이지를 펼친 list."""
    proc = subprocess.run(['gh', 'api', '--paginate', '--slurp', endpoint],
                          text=True, capture_output=True)
    if proc.returncode != 0:
        raise RuntimeError(f'gh api 실패: {endpoint}\n{proc.stderr.strip()}')
    raw = proc.stdout.strip()
    data = json.loads(raw) if raw else []
    out = []
    for page in data if isinstance(data, list) else [data]:
        if isinstance(page, list):
            out.extend(page)
        elif page:
            out.append(page)
    return out


def gh_obj(endpoint: str) -> dict:
    proc = subprocess.run(['gh', 'api', endpoint], text=True, capture_output=True)
    if proc.returncode != 0:
        raise RuntimeError(f'gh api 실패: {endpoint}\n{proc.stderr.strip()}')
    return json.loads(proc.stdout)


def iso(s: str | None) -> datetime | None:
    if not s:
        return None
    try:
        return datetime.fromisoformat(s.replace('Z', '+00:00'))
    except ValueError:
        return None


def kst(dt: datetime | None, fmt: str = '%Y-%m-%d') -> str:
    return dt.astimezone(KST).strftime(fmt) if dt else '-'


def bar(value: float, top: float, width: int = BAR_W) -> str:
    n = 0 if top <= 0 else int(round(value / top * width))
    n = max(0, min(width, n))
    return '█' * n + '·' * (width - n)


def spark(vals: list[int]) -> str:
    top = max(vals) if vals else 0
    if top <= 0:
        return '·' * len(vals)
    return ''.join(SPARK[min(len(SPARK) - 1, int(round(v / top * (len(SPARK) - 1))))] for v in vals)


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


def one_liner(p: dict, tops: dict) -> str:
    """로그 수치만으로 만드는 중립적인 한 줄 요약 (사람 평가 아님)."""
    metrics = [
        ('커밋', p['commits'], tops['commits'], ''),
        ('PR', p['prs_opened'], tops['prs'], f'머지 {p["prs_merged"]}'),
        ('리뷰', p['reviews'], tops['reviews'], ''),
        ('이슈 처리', p['issues_closed'], tops['closed'], ''),
        ('Notion 일지', p['notion_journal'], tops['journal'], ''),
    ]
    ranked = []
    for label, val, top, extra in metrics:
        if val <= 0:
            continue
        ratio = val / top if top else 0
        ranked.append((ratio, label, val, top, extra))
    ranked.sort(reverse=True)
    picks = []
    for ratio, label, val, top, extra in ranked:
        if len(picks) >= 3:
            break
        if len(picks) < 2 or ratio >= 0.7:
            mark = ' 1위' if top and val == top else (' 상위' if ratio >= 0.7 else '')
            tail = f'({extra})' if extra else ''
            picks.append(f'{label} {val}{tail}{mark}')
    if not picks:
        picks = [f'커밋 {p["commits"]}']
    line = ' · '.join(picks)
    notes = []
    if p['stale'] > 0:
        notes.append(f'담당 열린 이슈 {p["assigned_open"]}건 중 {p["stale"]}건 {STALE_DAYS}일+ 정체 — 진행 공유가 필요할 수 있음')
    if p['notion_journal'] <= 1:
        notes.append(f'Notion 일지 {p["notion_journal"]}건(기록 습관 지표)')
    if notes:
        line += ' — ' + ' / '.join(notes)
    return line


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


def repo_slug() -> str:
    slug = os.environ.get('GITHUB_REPOSITORY')
    if slug:
        return slug
    out = subprocess.check_output(['gh', 'repo', 'view', '--json', 'nameWithOwner',
                                   '-q', '.nameWithOwner'], text=True).strip()
    return out


def login_for_email(email: str) -> str | None:
    email = email.strip().lower()
    if email in EMAIL_TO_LOGIN:
        return EMAIL_TO_LOGIN[email]
    m = re.match(r'^(?:\d+\+)?([^@]+)@users\.noreply\.github\.com$', email)
    if m:
        return m.group(1)
    return None


def read_git_all() -> dict:
    """git log --all --no-merges --numstat → 사람별 커밋/증감/주간/마지막 날짜."""
    fmt = '@@%ae\t%an\t%aI'
    proc = subprocess.run(['git', 'log', '--all', '--no-merges', '--numstat',
                           f'--format={fmt}'], text=True, capture_output=True)
    if proc.returncode != 0:
        raise RuntimeError(proc.stderr.strip())
    per = defaultdict(lambda: {'commits': 0, 'add': 0, 'del': 0,
                               'last': None, 'weeks': [0] * 12, 'names': defaultdict(int)})
    cur = None
    total = 0
    first = None
    last = None
    for line in proc.stdout.splitlines():
        if line.startswith('@@'):
            email, name, date = line[2:].split('\t', 2)
            dt = iso(date)
            cur = email.strip().lower()
            if cur:
                p = per[cur]
                p['commits'] += 1
                p['names'][name.strip()] += 1
                if dt:
                    p['last'] = max(p['last'], dt) if p['last'] else dt
                    first = min(first, dt) if first else dt
                    last = max(last, dt) if last else dt
                    weeks = int((NOW - dt).days // 7)
                    if 0 <= weeks < 12:
                        p['weeks'][11 - weeks] += 1
                total += 1
        elif cur and line.strip():
            parts = line.split('\t')
            if len(parts) >= 3 and parts[0].isdigit() and parts[1].isdigit():
                per[cur]['add'] += int(parts[0])
                per[cur]['del'] += int(parts[1])
    return {'per_email': per, 'total': total, 'first': first, 'last': last}


def read_snapshot(path: Path) -> dict | None:
    """로컬에서 긁어 둔 Notion 스냅샷 (CI 비로그인에서 안 보이는 표 보완)."""
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--readme', default='README.md')
    ap.add_argument('--stdout', action='store_true', help='README를 쓰지 않고 출력만')
    ap.add_argument('--require-notion', action='store_true',
                    help='Notion 수집 실패 시 README를 덮어쓰지 않고 실패 (CI용)')
    args = ap.parse_args()

    repo = repo_slug()
    git = read_git_all()
    branch_count = len([b for b in subprocess.check_output(
        ['git', 'branch', '-r'], text=True).splitlines() if 'HEAD' not in b])

    issues = [i for i in gh_json(f'repos/{repo}/issues?state=all&per_page=100')
              if 'pull_request' not in i]
    pulls = gh_json(f'repos/{repo}/pulls?state=all&per_page=100')

    def reviews(n: int) -> list:
        return gh_json(f'repos/{repo}/pulls/{n}/reviews?per_page=100')

    with ThreadPoolExecutor(max_workers=8) as ex:
        review_lists = list(ex.map(reviews, [p['number'] for p in pulls]))

    # 사람 키 = GitHub login (git 이메일이 매핑되면 login, 아니면 git 이름/이메일)
    people: dict[str, dict] = {}

    def person(login: str | None, name: str | None = None) -> dict | None:
        if not login or is_bot(login):
            return None
        key = login if login.startswith('git:') else login.lower()
        p = people.setdefault(key, {
            'login': login, 'name': name or login,
            'commits': 0, 'add': 0, 'del': 0, 'weeks': [0] * 12, 'last_commit': None,
            'issues_opened': 0, 'issues_closed': 0,
            'assigned_total': 0, 'assigned_open': 0, 'stale': 0, 'assigned_items': [],
            'prs_opened': 0, 'prs_merged': 0, 'prs_reviewed': set(), 'reviews': 0,
            'last_issue': None, 'last_pr': None, 'last_review': None,
        })
        if name and (not p['name'] or p['name'] == p['login']):
            p['name'] = name
        return p

    # git 커밋
    for email, g in git['per_email'].items():
        if 'copilot' in email or email.endswith('[bot]') or 'bot@' in email:
            continue
        login = login_for_email(email)
        name = max(g['names'], key=g['names'].get) if g['names'] else email
        if login:
            p = person(login, name)
        else:
            p = person(f'git:{name}', name)
        if p is None:
            continue
        p['commits'] += g['commits']
        p['add'] += g['add']
        p['del'] += g['del']
        p['weeks'] = [a + b for a, b in zip(p['weeks'], g['weeks'])]
        if g['last'] and (not p['last_commit'] or g['last'] > p['last_commit']):
            p['last_commit'] = g['last']

    # 이슈
    for i in issues:
        author = (i.get('user') or {}).get('login')
        p = person(author)
        if p:
            p['issues_opened'] += 1
            p['last_issue'] = max(filter(None, [p['last_issue'], iso(i.get('updated_at')), iso(i.get('created_at'))]),
                                  default=None)
        for a in i.get('assignees') or []:
            p = person(a.get('login'))
            if not p:
                continue
            p['assigned_total'] += 1
            if i.get('state') == 'open':
                p['assigned_open'] += 1
                created = iso(i.get('created_at'))
                upd = iso(i.get('updated_at'))
                p['assigned_items'].append((created, upd, i['number'], i.get('title', '')))
                if upd and NOW - upd > timedelta(days=STALE_DAYS):
                    p['stale'] += 1
            p['last_issue'] = max(filter(None, [p['last_issue'], iso(i.get('updated_at'))]), default=None)
        closed_by = (i.get('closed_by') or {}).get('login')
        p = person(closed_by)
        if p and i.get('state') == 'closed' and i.get('comments', 0) > 0:
            p['issues_closed'] += 1
            p['last_issue'] = max(filter(None, [p['last_issue'], iso(i.get('closed_at'))]), default=None)

    # PR + 리뷰
    for pr, rvs in zip(pulls, review_lists):
        author = (pr.get('user') or {}).get('login')
        p = person(author)
        if p:
            p['prs_opened'] += 1
            if pr.get('merged_at'):
                p['prs_merged'] += 1
            p['last_pr'] = max(filter(None, [p['last_pr'], iso(pr.get('updated_at')),
                                             iso(pr.get('merged_at')), iso(pr.get('created_at'))]), default=None)
        for rv in rvs:
            login = (rv.get('user') or {}).get('login')
            p = person(login)
            if not p:
                continue
            p['reviews'] += 1
            p['prs_reviewed'].add(pr['number'])
            p['last_review'] = max(filter(None, [p['last_review'], iso(rv.get('submitted_at'))]), default=None)

    # GitHub 프로필 이름 (차트는 login, 표는 이름 병기)
    for key, p in people.items():
        if key.startswith('git:'):
            continue
        try:
            u = gh_obj(f'users/{p["login"]}')
            if u.get('login'):
                p['login'] = u['login']          # 대소문자 정규화된 GitHub 표기
            if u.get('name'):
                p['name'] = u['name']
        except Exception:
            pass
        p['name'] = NAME_OVERRIDE.get(p['login'], p['name'] or p['login'])

    for p in people.values():
        last = max(filter(None, [p['last_commit'], p['last_issue'], p['last_pr'], p['last_review']]), default=None)
        p['last'] = last
        p['score'] = int(round(p['commits'] + 3 * p['prs_merged'] + 2 * p['reviews']
                               + 2 * p['issues_closed'] + 0.5 * p['issues_opened']))
        p['assigned_items'].sort(key=lambda x: (x[0] or NOW, x[2]))

    rows = sorted(people.values(), key=lambda p: (-p['score'], -p['commits'], p['login']))
    top = max((p['score'] for p in rows), default=0)
    top_commits = max((p['commits'] for p in rows), default=0)
    top_reviews = max((p['reviews'] for p in rows), default=0)
    top_closed = max((p['issues_closed'] for p in rows), default=0)

    total_issues_closed = sum(1 for i in issues
                              if i.get('state') == 'closed' and i.get('comments', 0) > 0)
    inc_commits = sum(p['commits'] for p in rows)
    inc_prs = sum(p['prs_opened'] for p in rows)
    inc_merged = sum(p['prs_merged'] for p in rows)
    total_reviews = sum(p['reviews'] for p in rows)
    all_weeks = [0] * 12
    for p in rows:
        all_weeks = [a + b for a, b in zip(all_weeks, p['weeks'])]

    live = read_notion(NOTION_TEXT)
    snap = read_snapshot(NOTION_SNAPSHOT)
    if live is None and snap is None and args.require_notion:
        print('Notion 수집/스냅샷 없음 — README를 덮어쓰지 않습니다.', file=sys.stderr)
        return 2
    snapshot_at = (snap or {}).get('captured_at')

    def pick(key: str) -> list:
        if live and live.get(key):
            return live[key]
        if snap and snap.get(key):
            return snap[key]
        return []

    notion = None
    if live or snap:
        notion = {k: pick(k) for k in ('journal', 'meetings', 'trouble', 'todos')}
        if not any(notion.values()):
            notion = None
    solved = sum(1 for t in notion['trouble'] if t['status'] == '해결 완료') if notion else 0
    need = sum(1 for t in notion['trouble'] if t['status'] == '해결 필요') if notion else 0
    nstats: dict[str, dict] = {}
    if notion:
        for e in notion['journal']:
            login = (e.get('login') or '').lower()
            if not login:
                continue
            s = nstats.setdefault(login, {'journal': 0, 'last': None, 'done': 0, 'doing': 0, 'todos': 0})
            s['journal'] += 1
            s['last'] = max(s['last'] or '', e.get('date_end') or e['date'])
            if e['status'] == '완료':
                s['done'] += 1
            elif e['status'] in ('진행 중', '진행중'):
                s['doing'] += 1
        for t in notion['todos']:
            for author in t['people']:
                login = NOTION_AUTHOR.get(author)
                if login:
                    s = nstats.setdefault(login.lower(), {'journal': 0, 'last': None, 'done': 0, 'doing': 0, 'todos': 0})
                    s['todos'] += 1
    for p in rows:
        s = nstats.get(p['login'].lower(), {})
        p['notion_journal'] = s.get('journal', 0)
        p['notion_last'] = s.get('last')
        p['notion_done'] = s.get('done', 0)
        p['notion_doing'] = s.get('doing', 0)
        p['notion_todos'] = s.get('todos', 0)

    notion_rows = sorted(rows, key=lambda p: (-p['notion_journal'], -p['score'], p['login']))

    top_open_issues = max((p['issues_opened'] for p in rows), default=0)
    top_closed_issues = max((p['issues_closed'] for p in rows), default=0)
    top_prs = max((p['prs_opened'] for p in rows), default=0)
    top_merged = max((p['prs_merged'] for p in rows), default=0)
    top_journal = max((p['notion_journal'] for p in rows), default=0)
    top_todos = max((p['notion_todos'] for p in rows), default=0)
    top_stale = max((p['stale'] for p in rows), default=0)
    top_add = max((p['add'] for p in rows), default=0)
    top_del = max((p['del'] for p in rows), default=0)
    top_assigned_open = max((p['assigned_open'] for p in rows), default=0)
    top_assigned_total = max((p['assigned_total'] for p in rows), default=0)
    top_issue_items = max((len(p['assigned_items']) for p in rows), default=0)
    tops = {
        'commits': top_commits, 'prs': top_prs, 'reviews': top_reviews,
        'closed': top_closed_issues, 'journal': top_journal, 'todos': top_todos,
    }

    lines: list[str] = []
    add = lines.append
    add(START)
    add('## 🤖 기여자 활동량 리포트 (자동 생성)')
    add('')
    add('> **⚠️🙏 사람이 쓴 평가가 아닙니다 — AI/CI 자동 생성 리포트입니다.**')
    add('>')
    add('> 🤖 이 문서는 **AI가 작성한 생성기 + CI(GitHub Actions)가 `git log --all`, GitHub API(이슈·PR·리뷰), '
        'Notion 공개 표(투두) + 마지막 로컬 스냅샷(비공개 표)을 기계적으로 집계**해 매일 자동 갱신합니다.')
    add('> 📊 **성과·실력·기여도·인사 평가가 아니며, 팀 내 서열화가 아닙니다.** '
        '커밋·이슈·PR·일지 수는 **업무량/기록량의 대리 지표**일 뿐이고, 설계·디버깅·리뷰·멘토링·문서화·회의·테스트·'
        '외부 대기 같은 기여는 숫자에 제대로 잡히지 않습니다.')
    add('> 🫶 **숫자가 적다고 기여가 적은 것이 아니고, 많다고 더 잘한 것도 아닙니다.** 재미와 회고용 참고 지표로만 봐 주세요.')
    add('> ⏳ **`장기 정체`는 "이슈를 받고도 안 한다"는 뜻이 아닙니다.** 장기 작업·블로킹·외부 대기·'
        '다른 우선순위일 수 있으니 참고만 하세요. **이 리포트로 사람을 평가하지 마세요.**')
    add(f'> 🔗 Notion 원본: [B팀_BEE 팀 페이지]({NOTION_URL}) (공개된 기록만 집계)')
    add(f'> 🧾 비공개 표(회의록·트러블슈팅·개인 일지)는 CI 비로그인에서 안 보여서 '
        f'마지막 로컬 스냅샷({snapshot_at or "없음"})을 사용 · 투두는 공개 표에서 매일 갱신')
    add('> 🕓 **매일 04:00 KST 자동 갱신** (GitHub Actions schedule) · 수동 실행: workflow_dispatch')
    add('>')
    add(f'> 기준: **전체 원격 브랜치 {branch_count}개** (main만 아님) · 커밋 {inc_commits}개(merge·봇 제외) · '
        f'기간 {kst(git["first"])} ~ {kst(git["last"])} · 갱신 {kst(NOW, "%Y-%m-%d %H:%M")} KST')
    add(f'> 가중치: 커밋×1 + 머지 PR×3 + 리뷰×2 + 닫은 이슈×2 + 연 이슈×0.5 · '
        f'장기 정체 = 열린 이슈 + 할당됨 + {STALE_DAYS}일 이상 갱신 없음 · '
        '**댓글 없는 닫힘은 이슈 처리량에서 제외**')
    add(f'> 생성기: [`.github/scripts/contributors_report.py`](.github/scripts/contributors_report.py) · '
        f'워크플로: [`.github/workflows/contributors.yml`](.github/workflows/contributors.yml)')
    add('')
    add('### 📊 요약')
    add('')
    add(f'- 기여자 **{len(rows)}명** · 커밋 **{inc_commits}** · 이슈 **{len(issues)}** (댓글 있는 닫힘 {total_issues_closed}) · '
        f'PR **{inc_prs}** (머지 {inc_merged}) · 리뷰 **{total_reviews}**')
    if notion:
        add(f'- 📓 Notion 개인 일지 **{len(notion["journal"])}** · 회의록 **{len(notion["meetings"])}** · '
            f'트러블슈팅 **{len(notion["trouble"])}** (완료 {solved} / 필요 {need}) · '
            f'투두 **{len([t for t in notion["todos"] if t["people"]])}**')
    add(f'- 최근 12주 전체 커밋: `{spark(all_weeks)}`  (왼쪽이 11주 전, 오른쪽이 이번 주)')
    add('')
    add('### 🏅 활동량 점수 (기계적 집계 · 서열 아님)')
    add('')
    add('```text')
    for p in rows:
        add(f'{person_label(p)} {bar(p["score"], top)} {p["score"]:>5}')
    add('```')
    add('')
    add('### 💻 커밋 (전체 브랜치)')
    add('')
    add('```text')
    for p in rows:
        add(f'{person_label(p)} {bar(p["commits"], top_commits)} {p["commits"]:>5}')
    add('```')
    add('')
    add('### 🚀 GitHub PR (올림/머지)')
    add('')
    add('```text')
    for p in rows:
        add(f'{person_label(p)} {bar(p["prs_opened"], top_prs)} {p["prs_opened"]:>4}  (머지 {p["prs_merged"]})')
    add('```')
    add('')
    add('### 🔍 PR 리뷰 참여')
    add('')
    add('```text')
    for p in rows:
        add(f'{person_label(p)} {bar(p["reviews"], top_reviews)} {p["reviews"]:>4}  (PR {len(p["prs_reviewed"])}개)')
    add('```')
    add('')
    add('### 🧾 이슈 (열음 / 댓글 있는 닫힘)')
    add('')
    add('```text')
    for p in rows:
        add(f'{person_label(p)} 열음 {bar(p["issues_opened"], top_open_issues)} {p["issues_opened"]:>3} · '
            f'닫음 {bar(p["issues_closed"], top_closed_issues)} {p["issues_closed"]:>3}')
    add('```')
    add('')
    add('댓글 없이 닫힌 이슈는 처리량에서 제외했습니다.')
    add('')
    add('')
    add('### 📋 상세 (정확한 수치)')
    add('')
    add('| 기여자 | 커밋 | +/− | 이슈 열음/닫음 | 할당 열림/누적 | PR 올림/머지 | 리뷰 | 정체 | Notion 일지(완/진) | 마지막 작업 | 점수 |')
    add('|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|')
    for p in rows:
        name = name_of(p)
        add(f'| {name} | {p["commits"]} | {p["add"]:,}/{p["del"]:,} | '
            f'{p["issues_opened"]}/{p["issues_closed"]} | {p["assigned_open"]}/{p["assigned_total"]} | '
            f'{p["prs_opened"]}/{p["prs_merged"]} | {p["reviews"]} | '
            f'{p["stale"] if p["stale"] else "-"} | '
            f'{p["notion_journal"]} ({p["notion_done"]}/{p["notion_doing"]}) | {kst(p["last"])} | {p["score"]} |')
    add('')
    add('### 📊 상세 비교 (블록 그래프 + 숫자)')
    add('')
    add('그래프는 왼쪽, 숫자는 오른쪽에 붙입니다. 막대는 항목별 최대값 기준입니다. (`█` = 큼, `·` = 작음)')
    add('')
    add('| 항목 | ' + ' | '.join(
        f'{p["login"]}<br>({p["name"]})' if p['name'] != p['login'] else p['login'] for p in rows) + ' |')
    add('|---|' + '|'.join([':---'] * len(rows)) + '|')

    def metric_row(label: str, vals: list[int], topv: int) -> None:
        cells = [f'`{bar(val, topv, 8)} {rjust_fig(f"{val:,}", 8)}`' for val in vals]
        add(f'| {label} | ' + ' | '.join(cells) + ' |')

    metric_row('커밋', [p['commits'] for p in rows], top_commits)
    metric_row('증감 +', [p['add'] for p in rows], top_add)
    metric_row('증감 −', [p['del'] for p in rows], top_del)
    metric_row('이슈 열음', [p['issues_opened'] for p in rows], top_open_issues)
    metric_row('이슈 닫음(댓글)', [p['issues_closed'] for p in rows], top_closed_issues)
    metric_row('담당 열린', [p['assigned_open'] for p in rows], top_assigned_open)
    metric_row('누적 할당', [p['assigned_total'] for p in rows], top_assigned_total)
    metric_row('PR 올림', [p['prs_opened'] for p in rows], top_prs)
    metric_row('PR 머지', [p['prs_merged'] for p in rows], top_merged)
    metric_row('리뷰', [p['reviews'] for p in rows], top_reviews)
    metric_row('정체', [p['stale'] for p in rows], top_stale)
    metric_row('Notion 일지', [p['notion_journal'] for p in rows], top_journal)
    metric_row('투두', [p['notion_todos'] for p in rows], top_todos)
    metric_row('점수', [p['score'] for p in rows], top)
    add('')
    add('### 🗣️ 참여자 한줄평 (기계적 로그 집계)')
    add('')
    for p in rows:
        add(f'- **{name_of(p)}** — {one_liner(p, tops)}')
    add('')
    if notion:
        add('### 📝 Notion 활동 (공개 표 + 로컬 스냅샷)')
        add('')
        add('> 🤖 **AI/CI 자동 수집** — **공개 표(투두)**는 매일 읽고, **비공개 표(회의록·트러블슈팅·개인 일지)**는 '
            f'마지막 로컬 스냅샷({snapshot_at or "없음"})을 사용합니다. 사람이 수치를 고치지 않습니다.')
        add('> 🙏 **개인 일지 개수는 "기록 습관" 지표**일 뿐, 실제 업무량·기여도가 아닙니다. '
            '일지를 적게 쓴 사람도 코드·회의·디버깅·멘토링·테스트에서 기여했습니다.')
        add('> 🫶 이 페이지는 **사람을 평가하려는 것이 아니라**, 프로젝트 기록을 투명하게 모아 보려는 것입니다.')
        add('> 📌 회의록·트러블슈팅은 작성자 컬럼이 공개 표에 없어 팀 단위로만 셉니다. 투두는 공개 표에서 매일 갱신합니다.')
        add('')
        add('| 기여자 | 개인 일지 | 최근 작성 | 완료/진행 | 투두(공개) | GitHub PR (올림/머지) |')
        add('|:---|:---|:---|:---|:---|:---|')
        for p in notion_rows:
            jn = rjust_fig(str(p['notion_journal']), 6)
            tn = rjust_fig(str(p['notion_todos']), 6)
            pn = rjust_fig(f'{p["prs_opened"]}/{p["prs_merged"]}', 6)
            add(f'| {name_of(p)} | `{mini(p["notion_journal"], top_journal)} {jn}` | '
                f'{p["notion_last"] or "-"} | {p["notion_done"]}/{p["notion_doing"]} | '
                f'`{mini(p["notion_todos"], top_todos)} {tn}` | '
                f'`{mini(p["prs_opened"], top_prs)} {pn}` |')
        add('')
        add('> ✅ 투두는 공개 표의 항목별 참여 인원을 각각 세므로, 인원 합계가 항목 수와 다를 수 있습니다.')
        add('')
        add('### 🧯 트러블슈팅 (팀)')
        add('')
        add(f'공개 표의 트러블슈팅 **{len(notion["trouble"])}건** — 해결 완료 **{solved}** / 해결 필요 **{need}**.')
        add('')
        add('| 상태 | 건수 | 그래프 |')
        add('|---|---:|---|')
        top_status = max(solved, need, 1)
        add(f'| 해결 완료 | {solved} | `{bar(solved, top_status, 8)}` |')
        add(f'| 해결 필요 | {need} | `{bar(need, top_status, 8)}` |')
        add('')
        for t in sorted(notion['trouble'], key=lambda x: x['date']):
            add(f'- {t["date"]} · **{t["status"]}** · {t["problem"]}')
        add('')
        add('### 📓 Notion 개인 일지')
        add('')
        add('```text')
        for p in notion_rows:
            add(f'{person_label(p)} {bar(p["notion_journal"], top_journal)} {p["notion_journal"]:>4}')
        add('```')
        add('')
        add('### ✅ Notion 투두 (공개 표)')
        add('')
        add('```text')
        for p in notion_rows:
            add(f'{person_label(p)} {bar(p["notion_todos"], top_todos)} {p["notion_todos"]:>4}')
        add('```')
        add('')
        add(f'<details><summary>📓 개인 일지 전체 {len(notion["journal"])}건 — 날짜·작성자·작업·상태</summary>')
        add('')
        for e in sorted(notion['journal'], key=lambda x: (x['date'], x['author'])):
            login = e.get('login')
            who = NOTION_TEAM.get(login, e['author']) if login else f'{e["author"]} (게스트)'
            suffix = f' ({login})' if login else ''
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
            solved = sum(1 for t in notion['trouble'] if t['status'] == '해결 완료')
            need = sum(1 for t in notion['trouble'] if t['status'] == '해결 필요')
            add(f'<details><summary>🧯 트러블슈팅 전체 {len(notion["trouble"])}건 (해결 완료 {solved} / 해결 필요 {need})</summary>')
            add('')
            for t in sorted(notion['trouble'], key=lambda x: x['date']):
                add(f'- {t["date"]} · {t["status"]} · {t["problem"]}')
                if t['details']:
                    add(f'  - {" / ".join(t["details"])}')
            add('')
            add('</details>')
            add('')
        todo_rows = [t for t in notion['todos'] if t['people']]
        if todo_rows:
            add(f'<details><summary>✅ 투두리스트 공개 항목 {len(todo_rows)}건</summary>')
            add('')
            for t in todo_rows:
                add(f'- {t["title"]} — {", ".join(t["people"])}')
            add('')
            add('</details>')
            add('')
    else:
        add('### 📝 Notion 활동')
        add('')
        add('> ⚠️ 이번 실행에서는 Notion 공개 페이지를 읽지 못했습니다. '
            '다음 CI 실행에서 다시 시도합니다. (페이지가 비공개/로그인 필요로 바뀌면 수집이 실패할 수 있습니다.)')
        add('')
    add('### ⏳ 할당된 열린 이슈 (개수·링크·경과)')
    add('')
    add('할당된 사람이 있는 **열린 이슈 전부**입니다. 괄호 안은 **개설 후 경과일 · 마지막 갱신 후 경과일**이고, '
        f'**정체** 표시는 {STALE_DAYS}일 이상 갱신이 없는 항목입니다. '
        '"이슈를 받고도 안 한다"는 뜻이 아니며, 장기 작업·블로킹·외부 대기일 수 있습니다.')
    add('')
    total_open = 0
    total_stale = 0
    for p in rows:
        if not p['assigned_items']:
            continue
        total_open += len(p['assigned_items'])
        stale_n = sum(1 for _, upd, _, _ in p['assigned_items']
                      if upd and NOW - upd > timedelta(days=STALE_DAYS))
        total_stale += stale_n
        add(f'- **{name_of(p)}** {len(p["assigned_items"])}건 {bar(len(p["assigned_items"]), top_issue_items, 12)} (정체 {stale_n})')
        for created, upd, num, title in p['assigned_items']:
            safe = title.replace('[', '\\[').replace(']', '\\]').replace('|', '\\|')
            cd = f'{(NOW - created).days}' if created else '?'
            ud = f'{(NOW - upd).days}' if upd else '?'
            flag = ' · **정체**' if upd and NOW - upd > timedelta(days=STALE_DAYS) else ''
            add(f'  - [#{num} {safe}](https://github.com/{repo}/issues/{num}) — 개설 {cd}일 · 갱신 {ud}일 전{flag}')
    if total_open == 0:
        add('- 없음')
    else:
        add('')
        add(f'총 **{total_open}건** (정체 **{total_stale}건**).')
    add('')
    add('### 데이터 한계')
    add('')
    add('- 커밋은 **모든 원격 브랜치의 merge 제외** 이력입니다. 같은 커밋이 여러 브랜치에 있어도 한 번만 셉니다.')
    add('- 이슈 할당 이력은 현재/닫힌 이슈의 assignee 기준입니다. 과거에 assignee가 바뀐 이력은 GitHub 이벤트 API 없이는 정확히 복원하지 않습니다.')
    add('- **댓글 없이 닫힌 이슈는 이슈 처리량에서 제외**합니다. 처리량은 댓글이 1개 이상 있는 닫힌 이슈만 셉니다.')
    add('- 장기 정체의 경과일은 **마지막 갱신(updated_at) 기준**입니다. 할당된 시점과 다를 수 있습니다.')
    add('- 봇(Copilot 등)은 제외합니다. PR 리뷰는 submitted review만 셉니다(코멘트-only 리뷰 제외).')
    add('- Notion 개인 일지 개수는 **기록 습관 지표**입니다. 일지를 적게 쓴 사람도 코드·회의·디버깅·멘토링·테스트에서 기여할 수 있습니다.')
    add('- Notion 회의록·트러블슈팅은 공개 표에 작성자 컬럼이 없어 팀 단위로만 셉니다.')
    add(f'- Notion 비공개 표(회의록·트러블슈팅·개인 일지)는 CI 비로그인에서 안 보여서 **`notion_snapshot.json`(마지막 로컬 스크랩, {snapshot_at or "없음"})**을 사용합니다.')
    add('- 투두는 공개 표에서 매일 갱신합니다. 비공개 표 스냅샷은 로컬에서 다시 스크랩하면 갱신됩니다.')
    add('- 점수는 활동량 비교용이며, 난이도·품질·기여도를 대변하지 않습니다. **이 리포트로 사람을 평가하지 마세요.**')
    add(END)

    report = '\n'.join(lines)
    if args.stdout:
        print(report)
        return 0

    path = Path(args.readme)
    text = path.read_text(encoding='utf-8') if path.exists() else ''
    if START in text and END in text:
        before = text[:text.index(START)]
        after = text[text.index(END) + len(END):]
        text = before.rstrip() + '\n\n' + report + after
    else:
        text = text.rstrip() + '\n\n' + report + '\n'
    path.write_text(text, encoding='utf-8')
    print(f'updated {path} ({len(report)} bytes)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
