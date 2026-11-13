#!/usr/bin/env python3
"""Slack 덤프 → README의 SLACK_STATS 블록 생성기.

GitHub Actions(.github/workflows/slack-stats.yml)와 로컬 수동 실행에서 모두 쓴다.
- 기간: 기본 2026-11-04 ~ (KST, 이전 데이터 제외)
- 글 = 최상위 메시지(단독 + 스레드 시작), 댓글 = 스레드 답글
- 이모지 반응 통계는 넣지 않는다.
- 이 스크립트는 SLACK_STATS 블록만 갱신한다. GitHub/Notion은 각자 스크립트가 맡는다.

사용:
  python3 .github/scripts/slack_stats.py --dump ~/Downloads/slackdump_....zip --readme README.md
  python3 .github/scripts/slack_stats.py --stdout
  python3 .github/scripts/slack_stats.py --since 2026-11-04
"""
from __future__ import annotations

import argparse
import collections
import json
import re
import zipfile
from datetime import datetime, timedelta, timezone
from pathlib import Path

KST = timezone(timedelta(hours=9))
START = '<!-- SLACK_STATS:START -->'
END = '<!-- SLACK_STATS:END -->'
DEFAULT_SINCE = '2026-11-04'   # 이 날짜(KST) 이전 메시지는 집계에서 제외

# Slack 시스템 메시지 — 사람 글/댓글로 세지 않는다.
SYSTEM_SUBTYPES = {
    'channel_join', 'channel_leave', 'channel_name', 'channel_purpose', 'channel_topic',
    'channel_archive', 'channel_unarchive', 'bot_add', 'bot_remove',
    'pinned_item', 'unpinned_item', 'file_comment',
}

# 업무 관련 메시지 필터 — 생활 잡담/인사/짧은 리액션을 뺀다.
# 규칙은 여기 두 목록만 고치면 된다. (대소문자 무시)
WORK_KEYWORDS = (
    '로봇', '로보틱스', '코드', '코딩', '구현', '개발', '빌드', '컴파일', '테스트', '에러', '오류', '버그',
    '디버그', '디버깅', 'pr', '풀리퀘', '풀리퀘스트', '커밋', '브랜치', '머지', '리뷰', '이슈', '배포',
    '서버', 'api', 'db', '데이터', '모델', '학습', '추론', '알고리즘', '경로', '플래너', '내비', '네비',
    'nav2', 'ros', 'ros2', 'slam', '인지', '비전', '카메라', '라이다', '센서', '모터', '제어', '컨트롤',
    '속도', '각도', '좌표', '지도', '맵', '시뮬', '시뮬레이션', '하드웨어', '펌웨어', '젯슨', '오린',
    '미션', '태스크', '스케줄', '회의', '회의록', '일정', '문서', '보고서', '발표', '데모', 'qa',
    '테스트베드', '도커', '컨테이너', '깃', 'git', '깃허브', 'github', '액션', '워크플로', 'ci', 'cd',
    '로그', '모니터', '성능', '최적화', '튜닝', '파라미터', '설정', '스택', '트레이스', '요구사항',
    '기획', '설계', '아키텍처', '인터페이스', '프로토콜', '토픽', '노드', '패키지', '릴리즈',
    '리팩토링', '리팩터링', '확인 부탁', '확인해', '확인 필요', '공유', '정리', '수정', '반영',
    '점검', 'http', 'https', '.py', '.cpp', '.hpp', '.yaml', '.json', '.sh', '스크린샷', '첨부',
    'cuvslam', 'cuvslsam', 'vslam', 'nvblox', 'realsense', '리얼센스', '퍽', 'pick_far',
    'manipulator', '매니퓰레이터', '캘리브레이션', 'calibration', 'tsdf', 'decay_tsdf',
    '프린터', '3d', 'planner', '계층적', '강화학습', 'ontology',
)

SOCIAL_PATTERNS = [
    # 예: '오', '늦게 도착', '오세요(유)', '몸이' → 제외.
    # '오'는 '오늘·오류·오프' 등에 흔해서 패턴에 넣지 않는다 — 한 글자 잡담은 아래 길이 규칙이 거른다.
    re.compile(r'^(넵|네+|예|ㅇㅋ|ㅋㅋ+|ㅎㅎ+|굿|좋아요?|감사|감사합니다|고생|수고|수고하셨|'
               r'안녕|안녕하세요?|하이|hello|hi|확인|확인했습니다|확인했어요|알겠|알겠습니다)[\s!?.~]*$',
               re.IGNORECASE),
    re.compile(r'늦게\s*도착|도착(했|합니다|이요)?|출근|퇴근|오세요|오세유|오셨어요|오셨', re.IGNORECASE),
    re.compile(r'몸이|몸조리|아파|아프|병원|점심|저녁|밥|커피|간식|야식|날씨|비\s*온|눈\s*온|더워|추워',
               re.IGNORECASE),
    re.compile(r'집\s*(가|갑|도착)|가는\s*중|먼저\s*(가|갑)|생일|축하|축하해|피곤|졸려|힘들|'
               r'조금\s*늦|늦어서|늦을|금방\s*가', re.IGNORECASE),
    # 생활 잡담 — 업무 키워드가 없을 때만 적용된다 (키워드가 있으면 업무 글로 남긴다).
    re.compile(r'배고|배불|졸리|주말|쉬고|쉬는|놀러|게임|영화|드라마|맛집|카페|소주|맥주|'
               r'오늘\s*(뭐|점심|저녁|야식)|뭐\s*먹|ㅠㅠ|ㅜㅜ|죄송|미안|다행', re.IGNORECASE),
]

# 명백한 개인사는 업무 키워드보다 먼저 제외한다.
PERSONAL_OVERRIDES = re.compile(
    r'몸이|병원|노로바이러스|쾌차|손이\s*떨|떨려|긴장|일이\s*끝나|들어가유|들어갑니다|퇴근|'
    r'연차|휴가|반차|병가|집안일정|늦게\s*도착|알람을\s*못|알람\s*못|늦게\s*일어났|'
    r'지각할|지각했|지각임|지각이야',
    re.IGNORECASE,
)


def is_work_message(m: dict) -> bool:
    """업무 메시지인가 — 개인사/생활 잡담은 제외. 첨부·링크·업무 키워드가 있어야 업무 글로 본다."""
    text = ' '.join(str(m.get('text') or '').split())
    low = text.lower()
    if PERSONAL_OVERRIDES.search(low):
        return False
    if m.get('files') or m.get('attachments'):
        return True
    if any(k in low for k in WORK_KEYWORDS):
        return True
    if any(p.search(low) for p in SOCIAL_PATTERNS):
        return False
    # 예전에는 10자 이상이면 업무 글로 포함했다. 그 구멍으로 일상 잡담이 길다는
    # 이유만으로 새어 들어갔으므로, 이제는 명시적 업무 신호가 없으면 제외한다.
    return False


def kst(ts: float) -> datetime:
    return datetime.fromtimestamp(float(ts), KST)


def md(s: str) -> str:
    return str(s).replace('|', '\\|').replace('\n', ' ').strip()


def bar(n: int, top: int, width: int = 24) -> str:
    if top <= 0:
        return '·' * width
    fill = 0 if n <= 0 else max(1, round(n / top * width))
    fill = min(fill, width)
    return '█' * fill + '·' * (width - fill)


def display_name(uid: str, users: dict) -> str:
    u = users.get(uid, {})
    name = (u.get('real_name') or u.get('profile', {}).get('display_name')
            or u.get('name') or uid)
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


def build(channel: dict, users: dict, msgs: list[dict], path: Path, since: datetime) -> str:
    people: dict[str, dict] = {}
    bots: collections.Counter[str] = collections.Counter()
    content = []  # (message, is_reply, uid, is_bot)
    system = 0
    excluded = 0  # 업무 필터에서 제외된 사람 메시지 수
    excluded_by_uid: collections.Counter[str] = collections.Counter()

    for m in msgs:
        if m.get('type') not in (None, 'message'):
            continue
        ts = m.get('ts')
        if not ts:
            continue
        t = kst(ts)
        if t < since:
            continue
        if m.get('subtype') in SYSTEM_SUBTYPES:
            system += 1
            continue
        uid = m.get('user') or m.get('bot_id') or m.get('username') or '?'
        is_reply = bool(m.get('thread_ts') and m.get('thread_ts') != m.get('ts'))
        if is_bot(uid, users, m):
            bots[display_name(uid, users)] += 1
            content.append((m, is_reply, uid, True))
            continue
        if not is_work_message(m):
            excluded += 1
            excluded_by_uid[uid] += 1
            continue
        s = people.setdefault(uid, {
            'posts': 0, 'comments': 0,
            'standalone': 0, 'thread_starts': 0,
            'first': t, 'last': t,
        })
        if not is_reply:
            s['posts'] += 1
        s['first'] = min(s['first'], t)
        s['last'] = max(s['last'], t)
        content.append((m, is_reply, uid, False))

    if not people:
        raise SystemExit('해당 기간에 사람 메시지가 없음 — --since/채널/덤프 확인')

    # 같은 스레드에서 같은 사람이 연속으로 단 답글은 1개 댓글로 센다.
    replies_by_thread: dict[str, list[tuple[float, dict, str]]] = collections.defaultdict(list)
    for m, is_reply, uid, bot in content:
        if not bot and is_reply and m.get('thread_ts'):
            replies_by_thread[m['thread_ts']].append((float(m['ts']), m, uid))
    deduped_reply_ts: set[str] = set()
    for items in replies_by_thread.values():
        items.sort(key=lambda x: x[0])
        last_uid = None
        for _, m, uid in items:
            if uid != last_uid:
                people[uid]['comments'] += 1
                deduped_reply_ts.add(str(m['ts']))
                last_uid = uid

    counted = []  # 사람 글 + (연속 답글을 합친) 사람 댓글
    for m, is_reply, uid, bot in content:
        if bot:
            continue
        if is_reply and str(m['ts']) not in deduped_reply_ts:
            continue
        counted.append((m, uid, is_reply))

    for uid in people:
        people[uid]['first'] = None
        people[uid]['last'] = None
    active_days: dict[str, set] = collections.defaultdict(set)
    for m, uid, is_reply in counted:
        t = kst(m['ts'])
        s = people[uid]
        s['first'] = t if s['first'] is None else min(s['first'], t)
        s['last'] = t if s['last'] is None else max(s['last'], t)
        active_days[uid].add(t.date())
    for uid, s in people.items():
        s['active_days'] = len(active_days[uid])

    # 스레드 — 사람 댓글이 있는 스레드 시작 글
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
    parent_set = set(thread_parents)

    # 글 중 실제 스레드를 시작한 글 / 단독 글
    for s in people.values():
        s['thread_starts'] = 0
    for m, is_reply, uid, bot in content:
        if not bot and not is_reply and m.get('ts') in parent_set:
            people[uid]['thread_starts'] += 1
    for s in people.values():
        s['standalone'] = s['posts'] - s['thread_starts']

    comments = sum(s['comments'] for s in people.values())
    posts = sum(s['posts'] for s in people.values())
    total_standalone = sum(s['standalone'] for s in people.values())
    total_thread_starts = sum(s['thread_starts'] for s in people.values())
    bot_msgs = sum(bots.values())
    last = max(s['last'] for s in people.values())
    days = sorted({d for ds in active_days.values() for d in ds})

    # 날짜×사람별 계 (사람 글 + 연속 답글을 합친 댓글)
    daily_user: dict[tuple[str, str], dict] = {}
    for m, uid, is_reply in counted:
        key = kst(m['ts']).date().isoformat()
        du = daily_user.setdefault((key, uid), {
            'standalone': 0, 'thread_starts': 0, 'posts': 0, 'comments': 0,
        })
        if is_reply:
            du['comments'] += 1
        else:
            du['posts'] += 1
            if str(m['ts']) in parent_set:
                du['thread_starts'] += 1
            else:
                du['standalone'] += 1

    # 최근 날짜 → 과거. 활동 없는 날도 넣는다.
    day_keys: list[str] = []
    d = max(days)
    while d >= since.date():
        day_keys.append(d.isoformat())
        d -= timedelta(days=1)

    dump_at = datetime.fromtimestamp(path.stat().st_mtime, KST)
    lines: list[str] = []
    add = lines.append
    add(START)
    add('## 💬 PA1 MissionBot Slack 활동 통계 (자동 갱신)')
    add('')
    add(f'> 채널: **`{md(channel.get("name", "?"))}`** (비공개, 멤버 {channel.get("num_members", "?")}명) · '
        f'기간: **{since:%Y-%m-%d} ~ {last:%Y-%m-%d %H:%M} KST** (이전 데이터 제외) · '
        f'Slack 스냅샷: {dump_at:%Y-%m-%d %H:%M} KST')
    add('> 글 = 최상위 **업무** 메시지(단독 + 스레드 시작) · 댓글 = 스레드 답글. '
        '생활 잡담/인사/짧은 글은 제외. 시스템/봇 제외. **이모지 반응은 집계하지 않는다.**')
    add('> 같은 스레드에서 같은 사람이 연속으로 단 답글은 1개 댓글로 집계. '
        '**단독 글이 기준**, 스레드 시작·댓글은 참고.')
    add('> 업무 필터: 인사·도착/출근·감사/확인·식사/날씨·생활 잡담 제외. '
        '**첨부/링크/업무 키워드가 있어야 포함** (`WORK_KEYWORDS` / `SOCIAL_PATTERNS`).')
    if excluded:
        top_excluded = sorted(excluded_by_uid.items(), key=lambda kv: (-kv[1], display_name(kv[0], users)))[:6]
        detail = ' · '.join(f'{md(display_name(uid, users))} {n}' for uid, n in top_excluded if n)
        add(f'> 업무 필터로 제외: **{excluded}개** — {detail}')
    add('> 이 블록은 **GitHub Actions `slack-stats.yml`**이 매일 09:30 KST 갱신합니다 (수동 실행도 가능).')
    add('')
    add('### 🧮 기여도 (데이터 기반)')
    add('')
    add('> 단독 글 기준 **기계적 비율**입니다. 막대는 **1위=100%** 기준.')
    add('')
    add('| 이름 | 기여도 | 그래프 |')
    add('|:---|---:|:---:|')
    pct_rows = []
    for uid, s in sorted(people.items(), key=lambda kv: (-kv[1]['standalone'], display_name(kv[0], users))):
        pct_rows.append((uid, s['standalone'] / max(total_standalone, 1) * 100))
    top_pct = max((pct for _, pct in pct_rows), default=100)
    for uid, pct in pct_rows:
        add(f'| {md(display_name(uid, users))} | {pct:.1f}% | `{bar(pct, top_pct, 16)}` |')
    add('')
    add('### 🧑‍🤝‍🧑 사람별 통계 (단독 글 기준)')
    add('')
    people_sorted = sorted(people.items(), key=lambda kv: (
        -kv[1]['standalone'], -kv[1]['thread_starts'], display_name(kv[0], users)))
    top_standalone = max((s['standalone'] for _, s in people_sorted), default=1)
    add('| 이름 | 단독 글 | 그래프 (단독) | 스레드 시작 | 글 합계 | 댓글 | 활동일 |')
    add('|:---|---:|:---:|---:|---:|---:|---:|')
    for uid, s in people_sorted:
        add(f'| {md(display_name(uid, users))} | {s["standalone"]} | '
            f'`{bar(s["standalone"], top_standalone, 16)}` | {s["thread_starts"]} | '
            f'{s["posts"]} | {s["comments"]} | {s["active_days"]} |')
    add('')
    add('> **단독 글이 기준**. `스레드 시작`·`글 합계`·`댓글`은 참고용. '
        '막대 = 단독 글, 사람별 최대값 기준. (`█` 채움 · `·` 빈칸)')
    add('> 같은 스레드에서 같은 사람이 연속으로 단 답글은 1개 댓글로 집계.')
    add('')
    add('<details>')
    add('<summary>📆 날짜별 타임라인 (y=사람, x=날짜, 최근→과거)</summary>')
    add('')
    max_standalone = max((du['standalone'] for du in daily_user.values()), default=1)
    max_comments = max((du['comments'] for du in daily_user.values()), default=1)
    add('**단독 글**')
    add('')
    add('| 사람 | ' + ' | '.join(k[5:] for k in day_keys) + ' |')
    add('|:---|' + '|'.join([':---:'] * len(day_keys)) + '|')
    for uid, s in people_sorted:
        cells = []
        for k in day_keys:
            n = daily_user.get((k, uid), {'standalone': 0, 'comments': 0})['standalone']
            cells.append(f'`{bar(n, max_standalone, 5)}` {n}')
        add(f'| {md(display_name(uid, users))} | ' + ' | '.join(cells) + ' |')
    add('')
    add('**댓글**')
    add('')
    add('| 사람 | ' + ' | '.join(k[5:] for k in day_keys) + ' |')
    add('|:---|' + '|'.join([':---:'] * len(day_keys)) + '|')
    for uid, s in people_sorted:
        cells = []
        for k in day_keys:
            n = daily_user.get((k, uid), {'standalone': 0, 'comments': 0})['comments']
            cells.append(f'`{bar(n, max_comments, 5)}` {n}')
        add(f'| {md(display_name(uid, users))} | ' + ' | '.join(cells) + ' |')
    add('')
    add('</details>')
    add('')
    add(END)
    return '\n'.join(lines)


def update_readme(path: Path, block: str) -> bool:
    text = path.read_text(encoding='utf-8') if path.exists() else ''
    if START in text and END in text:
        before = text[:text.index(START)]
        after = text[text.index(END) + len(END):]
        new = before.rstrip() + '\n\n' + block + '\n\n' + after.lstrip('\n')
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
    ap.add_argument('--since', default=DEFAULT_SINCE,
                    help=f'집계 시작일 (KST, YYYY-MM-DD) — 기본 {DEFAULT_SINCE}')
    ap.add_argument('--stdout', action='store_true', help='README를 쓰지 않고 블록만 출력')
    args = ap.parse_args()

    try:
        since = datetime.strptime(args.since, '%Y-%m-%d').replace(tzinfo=KST)
    except ValueError as exc:
        raise SystemExit(f'--since 날짜 형식 오류: {args.since} (YYYY-MM-DD)') from exc

    channel, users, msgs, path = load_dump(args.dump)
    block = build(channel, users, msgs, path, since)
    if args.stdout:
        print(block)
        return 0
    changed = update_readme(args.readme, block)
    print(f'{args.readme} ' + ('갱신' if changed else '변경 없음'))
    print(f'채널 {channel.get("name")} · 메시지 {len(msgs)} · 블록 {len(block)} bytes')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
