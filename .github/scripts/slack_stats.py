#!/usr/bin/env python3
"""Slack 덤프 → README의 SLACK_STATS 블록 생성기.

수동 실행 전용이다. GitHub Actions에서 돌리지 않는다 (덤프에 비공개 메시지 포함).
매일 도는 contributors_report.py는 CONTRIBUTORS 블록만 갱신하고,
이 스크립트는 그와 별개인 SLACK_STATS 블록만 갱신한다.

사용:
  python3 .github/scripts/slack_stats.py \
    --dump ~/Downloads/slackdump_20261113_195914.zip \
    --readme README.md

  # README를 건드리지 않고 결과만 보기
  python3 .github/scripts/slack_stats.py --dump ... --stdout

덤프는 Slackdump `export` ZIP (channels.json · users.json · <채널>/YYYY-MM-DD.json)을 쓴다.
"""
from __future__ import annotations

import argparse
import collections
import json
import re
import statistics
import zipfile
from datetime import datetime, timedelta, timezone
from pathlib import Path

KST = timezone(timedelta(hours=9))
START = '<!-- SLACK_STATS:START -->'
END = '<!-- SLACK_STATS:END -->'
CONTRIB_START = '<!-- CONTRIBUTORS:START -->'

# Slack 시스템 메시지 — 사람 글/댓글로 세지 않는다.
SYSTEM_SUBTYPES = {
    'channel_join', 'channel_leave', 'channel_name', 'channel_purpose', 'channel_topic',
    'channel_archive', 'channel_unarchive', 'bot_add', 'bot_remove',
    'pinned_item', 'unpinned_item', 'file_comment',
}
THUMB_NAMES = {'+1', 'thumbsup', 'thumbsup_all'}
SPARK = '▁▂▃▄▅▆▇█'


def kst(ts: float) -> datetime:
    return datetime.fromtimestamp(float(ts), KST)


def fmt_dt(dt: datetime) -> str:
    return dt.strftime('%Y-%m-%d %H:%M')


def md(s: str) -> str:
    return str(s).replace('|', '\\|').replace('\n', ' ').strip()


def spark(vals: list[int]) -> str:
    if not vals:
        return ''
    lo, hi = min(vals), max(vals)
    if hi == lo:
        return SPARK[3] * len(vals)
    return ''.join(SPARK[round((v - lo) / (hi - lo) * (len(SPARK) - 1))] for v in vals)


def bar(n: int, top: int, width: int = 24) -> str:
    if top <= 0:
        return ''
    if n <= 0:
        return '·'
    return '█' * max(1, round(n / top * width))


def display_name(uid: str, users: dict) -> str:
    u = users.get(uid, {})
    name = (u.get('real_name') or u.get('profile', {}).get('display_name')
            or u.get('name') or uid)
    # 팀 표기(PA_1기)는 이름을 읽기 쉽게 생략한다.
    name = re.sub(r'\s*\(PA_1기\)\s*', '', name)
    return name.strip() or uid


def is_bot(uid: str, users: dict, m: dict | None = None) -> bool:
    u = users.get(uid, {})
    if u.get('is_bot'):
        return True
    if m and (m.get('bot_id') or m.get('subtype') == 'bot_message'):
        return True
    return False


def load_dump(path: Path):
    if not path.exists():
        raise SystemExit(f'덤프 파일 없음: {path}')
    if path.stat().st_size < 1024:
        raise SystemExit(f'덤프가 너무 작음({path.stat().st_size}B) — 빈 export 아닌지 확인: {path}')
    try:
        z = zipfile.ZipFile(path)
    except zipfile.BadZipFile as exc:
        raise SystemExit(f'ZIP이 아님: {path} ({exc})') from exc
    with z:
        names = z.namelist()
        if 'channels.json' not in names or 'users.json' not in names:
            raise SystemExit('Slack export 형식 아님 — channels.json / users.json 없음')

        channels = json.loads(z.read('channels.json'))
        users = {u['id']: u for u in json.loads(z.read('users.json')) if u.get('id')}

        # 메시지 JSON이 있는 채널을 고른다. 보통 export는 채널 1개다.
        candidates = []
        for c in channels:
            cname = c.get('name')
            if not cname:
                continue
            files = [n for n in names if n.startswith(cname + '/') and n.endswith('.json')]
            if files:
                candidates.append((len(files), c, files))
        if not candidates:
            raise SystemExit('메시지 JSON이 있는 채널이 없음 — export 내용 확인')
        candidates.sort(key=lambda x: x[0], reverse=True)
        _, channel, files = candidates[0]

        msgs: list[dict] = []
        for n in files:
            try:
                data = json.loads(z.read(n))
            except Exception:
                continue
            if isinstance(data, list):
                msgs.extend(data)
        return channel, users, msgs, path


def build(channel: dict, users: dict, msgs: list[dict], path: Path) -> str:
    people: dict[str, dict] = {}
    bots: collections.Counter[str] = collections.Counter()
    given: collections.Counter[str] = collections.Counter()
    reaction_names: collections.Counter[str] = collections.Counter()
    content = []  # (message, is_reply, uid)
    system = 0

    for m in msgs:
        if m.get('type') not in (None, 'message'):
            continue
        if m.get('subtype') in SYSTEM_SUBTYPES:
            system += 1
            continue
        ts = m.get('ts')
        if not ts:
            continue
        uid = m.get('user') or m.get('bot_id') or m.get('username') or '?'
        is_reply = bool(m.get('thread_ts') and m.get('thread_ts') != m.get('ts'))
        if is_bot(uid, users, m):
            bots[display_name(uid, users)] += 1
            content.append((m, is_reply, uid, True))
            continue
        t = kst(ts)
        s = people.setdefault(uid, {
            'posts': 0, 'comments': 0, 'react': 0, 'thumb': 0,
            'first': t, 'last': t,
        })
        if is_reply:
            s['comments'] += 1
        else:
            s['posts'] += 1
        s['first'] = min(s['first'], t)
        s['last'] = max(s['last'], t)
        for r in m.get('reactions') or []:
            cnt = int(r.get('count') or len(r.get('users') or []) or 0)
            rn = str(r.get('name') or '')
            s['react'] += cnt
            reaction_names[rn] += cnt
            if rn in THUMB_NAMES:
                s['thumb'] += cnt
            for ru in r.get('users') or []:
                if not is_bot(ru, users):
                    given[ru] += 1
        content.append((m, is_reply, uid, False))

    if not people:
        raise SystemExit('사람 메시지가 없음 — 채널/덤프 확인')

    for uid, n in given.items():
        if uid in people:
            people[uid]['given'] = n
    for s in people.values():
        s.setdefault('given', 0)

    # 스레드 — 사람 댓글 기준 첫 응답 시간
    parents = set()
    replies: dict[str, list[float]] = collections.defaultdict(list)
    for m, is_reply, uid, bot in content:
        if bot:
            continue
        ts = float(m['ts'])
        if m.get('thread_ts') == m.get('ts') or int(m.get('reply_count') or 0) > 0:
            parents.add(m['ts'])
        if is_reply and m.get('thread_ts'):
            replies[m['thread_ts']].append(ts)
    thread_parents = [p for p in parents if replies.get(p)]
    first_lat = [min(replies[p]) - float(p) for p in thread_parents if min(replies[p]) >= float(p)]
    comments = sum(s['comments'] for s in people.values())
    posts = sum(s['posts'] for s in people.values())
    react = sum(s['react'] for s in people.values())
    thumbs = sum(s['thumb'] for s in people.values())
    react_msgs = sum(1 for m, is_reply, uid, bot in content
                     if not bot and (m.get('reactions') or []))
    bot_msgs = sum(bots.values())
    first = min(s['first'] for s in people.values())
    last = max(s['last'] for s in people.values())
    days = sorted({kst(m['ts']).date() for m, is_reply, uid, bot in content
                   if not bot and m.get('ts')})

    # 월/주 버킷
    def bucket_counts(key_fn):
        out = {}
        for m, is_reply, uid, bot in content:
            if bot or not m.get('ts'):
                continue
            t = kst(m['ts'])
            key = key_fn(t)
            b = out.setdefault(key, {'posts': 0, 'comments': 0, 'react': 0, 'users': set(), 'by': collections.Counter()})
            if is_reply:
                b['comments'] += 1
            else:
                b['posts'] += 1
            b['users'].add(uid)
            b['by'][uid] += 1
            for r in m.get('reactions') or []:
                b['react'] += int(r.get('count') or len(r.get('users') or []) or 0)
        return out

    monthly = bucket_counts(lambda t: t.strftime('%Y-%m'))
    weekly = bucket_counts(lambda t: (t.date() - timedelta(days=t.weekday())).isoformat())

    # 빈 주/월도 채워 타임라인을 끊기지 않게 한다.
    def fill_weeks():
        start = min(days) - timedelta(days=min(days).weekday())
        end = max(days) - timedelta(days=max(days).weekday())
        out, d = [], start
        while d <= end:
            out.append(d.isoformat())
            d += timedelta(days=7)
        return out
    week_keys = fill_weeks()
    month_keys = []
    d = min(days).replace(day=1)
    while d <= max(days):
        month_keys.append(d.strftime('%Y-%m'))
        d = (d.replace(day=28) + timedelta(days=4)).replace(day=1)

    def top_name(b):
        if not b or not b['by']:
            return '-'
        mx = max(b['by'].values())
        names = [display_name(u, users) for u, n in b['by'].items() if n == mx]
        return ' · '.join(sorted(names))

    dump_at = datetime.fromtimestamp(path.stat().st_mtime, KST)
    lines: list[str] = []
    add = lines.append
    add(START)
    add('## 💬 PA1 MissionBot Slack 활동 통계 (자동 갱신)')
    add('')
    add('> **⚠️ 사람이 쓴 평가가 아닙니다 — Slack에서 직접 긁은 스냅샷을 스크립트가 기계적으로 집계한 결과입니다.**')
    add('>')
    add(f'> 채널: **`{md(channel.get("name", "?"))}`** (비공개, 멤버 {channel.get("num_members", "?")}명) · '
        f'기간: **{first:%Y-%m-%d %H:%M} ~ {last:%Y-%m-%d %H:%M} KST** · '
        f'Slack 스냅샷: {dump_at:%Y-%m-%d %H:%M} KST')
    add('> 글 = 최상위 메시지 · 댓글 = 스레드 답글 · 반응 = 메시지에 달린 이모지(받은 수). '
        '시스템 메시지와 봇(Claude·polly)은 사람 표에서 제외.')
    add('> 이 블록은 **GitHub Actions `slack-stats.yml`**이 매일 09:30 KST 갱신합니다 (수동 실행도 가능). '
        '매일 도는 contributors action은 아래 CONTRIBUTORS 블록만 갱신합니다.')
    add('> **활동량 대리 지표이며, 성과·실력·기여도 평가가 아닙니다.**')
    add('')
    add('### 📊 종합')
    add('')
    add(f'- 사람 메시지 **{posts + comments}** (글 **{posts}** · 댓글 **{comments}**) · '
        f'봇 메시지 **{bot_msgs}** · 시스템 메시지 **{system}**')
    add(f'- 스레드 **{len(thread_parents)}** · 댓글 **{comments}** · 첫 응답 중앙값 '
        f'**{statistics.median(first_lat) / 60:.1f}분**' if first_lat else
        f'- 스레드 **{len(thread_parents)}** · 댓글 **{comments}**')
    add(f'- 반응이 달린 메시지 **{react_msgs}** · 받은 반응 **{react}** · 👍 **{thumbs}**')
    add(f'- 활동일 **{len(days)}일** · 하루 평균 글+댓글 **{(posts + comments) / max(1, len(days)):.1f}**')
    add('')
    add('### 🧑‍🤝‍🧑 사람별 종합')
    add('')
    add('| 이름 | 글 | 댓글 | 받은 반응 | 👍 | 반응 보냄 | 첫 활동 | 마지막 활동 |')
    add('|:---|---:|---:|---:|---:|---:|:---|:---|')
    for uid, s in sorted(people.items(), key=lambda kv: (-(kv[1]['posts'] + kv[1]['comments']),
                                                         -kv[1]['react'], display_name(kv[0], users))):
        add(f'| {md(display_name(uid, users))} | {s["posts"]} | {s["comments"]} | {s["react"]} | '
            f'{s["thumb"]} | {s["given"]} | {s["first"]:%m-%d} | {s["last"]:%m-%d} |')
    add(f'| **합계** | **{posts}** | **{comments}** | **{react}** | **{thumbs}** | '
        f'**{sum(s["given"] for s in people.values())}** | | |')
    if bots:
        add('')
        add(f'> 봇 메시지: ' + ' · '.join(f'{md(k)} {v}' for k, v in bots.most_common()) +
            ' — 사람 표/그래프에서 제외.')
    add('')
    add('### 📈 글+댓글 많은 순 (ASCII)')
    add('')
    add('```text')
    top = max(s['posts'] + s['comments'] for s in people.values())
    for uid, s in sorted(people.items(), key=lambda kv: -(kv[1]['posts'] + kv[1]['comments'])):
        total = s['posts'] + s['comments']
        add(f'{display_name(uid, users)[:14]:<16} {bar(total, top)} {total}')
    add('```')
    add('')
    add('### 🗓 월별 추이')
    add('')
    add('| 월 | 글 | 댓글 | 받은 반응 | 활동자 | 가장 많이 쓴 사람 |')
    add('|:---|---:|---:|---:|---:|:---|')
    for k in month_keys:
        b = monthly.get(k, {'posts': 0, 'comments': 0, 'react': 0, 'users': set(), 'by': {}})
        add(f'| {k} | {b["posts"]} | {b["comments"]} | {b["react"]} | {len(b["users"])} | '
            f'{md(top_name(b))} |')
    add('')
    add('### 📅 주별 타임라인')
    add('')
    add('| 주 시작 (월) | 글 | 댓글 | 받은 반응 | 활동자 | 가장 많이 쓴 사람 |')
    add('|:---|---:|---:|---:|---:|:---|')
    for k in week_keys:
        b = weekly.get(k, {'posts': 0, 'comments': 0, 'react': 0, 'users': set(), 'by': {}})
        add(f'| {k} | {b["posts"]} | {b["comments"]} | {b["react"]} | {len(b["users"])} | '
            f'{md(top_name(b))} |')
    add('')
    add('```text')
    add(f'글   {spark([weekly.get(k, {}).get("posts", 0) for k in week_keys])}')
    add(f'댓글 {spark([weekly.get(k, {}).get("comments", 0) for k in week_keys])}')
    add(f'반응 {spark([weekly.get(k, {}).get("react", 0) for k in week_keys])}')
    add(f'     {week_keys[0]} ~ {week_keys[-1]}  (주 시작 기준)')
    add('```')
    add('')
    add('### ❤️ 반응')
    add('')
    add(f'- 반응이 달린 메시지 **{react_msgs}개** · 총 **{react}회** · 👍 **{thumbs}회**')
    add('')
    add('| 반응 (Slack 이름) | 횟수 |')
    add('|:---|---:|')
    for name, n in reaction_names.most_common(12):
        add(f'| `{md(name)}` | {n} |')
    rest = sum(n for name, n in reaction_names.items() if name not in dict(reaction_names.most_common(12)))
    if rest:
        add(f'| 기타 | {rest} |')
    add('')
    add('### 🧾 데이터 한계')
    add('')
    add('- 스냅샷 시점 이후의 메시지·수정·삭제는 반영되지 않는다. 반응 수는 스냅샷 시점 기준이다.')
    add('- `channel_join`·`channel_leave` 같은 시스템 메시지는 글/댓글에서 제외했다.')
    add('- 봇(Claude·polly)은 사람 표와 그래프에서 제외하고 별도로만 표기했다.')
    add('- 이름은 `users.json`의 `real_name` 기준이며 `(PA_1기)` 표기는 생략했다.')
    add('- 이 수치는 활동량의 대리 지표일 뿐이며, **이 리포트로 사람을 평가하지 마세요.**')
    add(END)
    return '\n'.join(lines)


def update_readme(path: Path, block: str) -> bool:
    text = path.read_text(encoding='utf-8') if path.exists() else ''
    if START in text and END in text:
        before = text[:text.index(START)]
        after = text[text.index(END) + len(END):]
        new = before.rstrip() + '\n\n' + block + '\n\n' + after.lstrip('\n')
    elif CONTRIB_START in text:
        idx = text.index(CONTRIB_START)
        new = text[:idx].rstrip() + '\n\n' + block + '\n\n' + text[idx:]
    else:
        new = text.rstrip() + '\n\n' + block + '\n'
    if new == text:
        return False
    path.write_text(new, encoding='utf-8')
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description='Slack 덤프 → README SLACK_STATS 블록')
    ap.add_argument('--dump', type=Path,
                    default=Path(__file__).resolve().parent / 'slack.zip',
                    help='Slackdump export ZIP 경로 (기본 .github/scripts/slack.zip)')
    ap.add_argument('--readme', default=Path('README.md'), type=Path, help='README 경로 (기본 README.md)')
    ap.add_argument('--stdout', action='store_true', help='README를 쓰지 않고 블록만 출력')
    args = ap.parse_args()

    channel, users, msgs, path = load_dump(args.dump)
    block = build(channel, users, msgs, path)
    if args.stdout:
        print(block)
        return 0
    changed = update_readme(args.readme, block)
    print(f'{args.readme} ' + ('갱신' if changed else '변경 없음'))
    print(f'채널 {channel.get("name")} · 메시지 {len(msgs)} · 블록 {len(block)} bytes')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
