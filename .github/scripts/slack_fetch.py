#!/usr/bin/env python3
"""Slack 사용자 토큰(xoxc) + 쿠키(xoxd)로 채널을 긁어 Slackdump 호환 ZIP을 만든다.

GitHub Actions(.github/workflows/slack-stats.yml)와 로컬 수동 실행에서 모두 쓴다.
- GitHub Actions: `SLACK_TOKEN` / `SLACK_COOKIE` 환경변수(Repository Secrets)를 읽는다.
- 로컬: slack_secrets.txt(gitignore)를 읽는다. 환경변수가 있으면 그쪽이 우선.
slack.zip(gitignore)을 만든 뒤 slack_stats.py가 README의 SLACK_STATS 블록을 갱신한다.

사용:
  python3 .github/scripts/slack_fetch.py
  python3 .github/scripts/slack_fetch.py --secrets ~/Downloads/slack_secrets.txt
  python3 .github/scripts/slack_fetch.py --channel C0C2CFTCWAH --out .github/scripts/slack.zip

slack_secrets.txt:
  SLACK_TOKEN=xoxc-...
  SLACK_COOKIE=xoxd-...
  SLACK_CHANNEL_ID=C0C2CFTCWAH       # 선택 (기본값 있음)
  SLACK_WORKSPACE=2026kant           # 선택 (있으면 그 워크스페이스 API 사용)
  SLACK_COOKIE_DS=xoxd-...           # 선택 (d-s 쿠키가 있으면)
"""
from __future__ import annotations

import argparse
import json
import os
import ssl
import time
import urllib.error
import urllib.parse
import urllib.request
import zipfile
from pathlib import Path

try:  # macOS python.org 빌드는 시스템 CA를 못 찾는다 — certifi가 있으면 그걸 쓴다.
    import certifi
    SSL_CONTEXT = ssl.create_default_context(cafile=certifi.where())
except Exception:
    SSL_CONTEXT = ssl.create_default_context()

DEFAULT_CHANNEL = 'C0C2CFTCWAH'
DEFAULT_CHANNEL_NAME = '프로젝트-pa1-mission-bot'
TIMEOUT = 30


SECRET_KEYS = (
    'SLACK_TOKEN', 'SLACK_COOKIE', 'SLACK_CHANNEL_ID',
    'SLACK_WORKSPACE', 'SLACK_COOKIE_DS',
)


def load_secrets(path: Path) -> dict[str, str]:
    # GitHub Actions는 환경변수(Secrets)로, 로컬은 slack_secrets.txt로.
    # 둘 다 있으면 환경변수가 우선한다.
    out: dict[str, str] = {k: os.environ[k] for k in SECRET_KEYS if os.environ.get(k)}
    if path.exists():
        for line in path.read_text(encoding='utf-8').splitlines():
            line = line.strip()
            if not line or line.startswith('#') or '=' not in line:
                continue
            k, v = line.split('=', 1)
            out.setdefault(k.strip(), v.strip().strip('"').strip("'"))
    if not out.get('SLACK_TOKEN') or not out.get('SLACK_COOKIE'):
        raise SystemExit(
            'SLACK_TOKEN / SLACK_COOKIE 가 없다.\n'
            'GitHub Actions면 Repository Secrets에 넣고, '
            '로컬이면 slack_secrets.txt에 넣어라.'
        )
    return out


def api(method: str, params: dict, secrets: dict[str, str]) -> dict:
    token = secrets.get('SLACK_TOKEN', '')
    cookie = secrets.get('SLACK_COOKIE', '')
    if not token or not cookie:
        raise SystemExit('SLACK_TOKEN / SLACK_COOKIE 가 없다 — Secrets/환경변수 또는 slack_secrets.txt 확인')

    workspace = secrets.get('SLACK_WORKSPACE', '').strip()
    bases = []
    if workspace:
        bases.append(f'https://{workspace}.slack.com/api/')
    bases.append('https://slack.com/api/')

    query = urllib.parse.urlencode(params)
    headers = {
        'Authorization': f'Bearer {token}',
        'User-Agent': 'beebot-slack-stats/1.0',
        'Cookie': f'd={cookie}',
    }
    ds = secrets.get('SLACK_COOKIE_DS', '').strip()
    if ds:
        headers['Cookie'] += f'; d-s={ds}'

    last_error = 'unknown'
    for base in bases:
        for attempt in range(5):
            req = urllib.request.Request(base + method + '?' + query, headers=headers)
            try:
                with urllib.request.urlopen(req, timeout=TIMEOUT, context=SSL_CONTEXT) as resp:
                    data = json.load(resp)
            except urllib.error.HTTPError as exc:
                if exc.code == 429:
                    wait = int(exc.headers.get('Retry-After', '5') or 5)
                    time.sleep(min(wait, 60))
                    continue
                last_error = f'HTTP {exc.code}'
                break
            except Exception as exc:
                last_error = str(exc)
                break

            if data.get('ok'):
                return data
            err = str(data.get('error') or 'unknown')
            last_error = err
            if err in ('ratelimited', 'rate_limited'):
                time.sleep(5)
                continue
            if err in ('invalid_auth', 'not_authed', 'account_inactive'):
                break
            raise SystemExit(f'{method} 실패: {err}')
    raise SystemExit(
        f'{method} 실패: {last_error}\n'
        'Slack 웹에서 토큰/쿠키가 만료됐을 수 있다 — OAuth & Permissions가 아니라 '
        '브라우저 DevTools에서 xoxc 토큰과 d 쿠키를 다시 복사해라.'
    )


def fetch_users(secrets: dict[str, str]) -> list[dict]:
    users: list[dict] = []
    cursor = ''
    while True:
        params = {'limit': 200}
        if cursor:
            params['cursor'] = cursor
        try:
            data = api('users.list', params, secrets)
        except SystemExit as exc:
            print(f'[warn] users.list 실패 — 이름 없이 진행: {exc}')
            return []
        users.extend(data.get('members') or [])
        cursor = (data.get('response_metadata') or {}).get('next_cursor') or ''
        if not cursor:
            break
    return users


def fetch_channel(channel_id: str, secrets: dict[str, str]) -> dict:
    try:
        data = api('conversations.info', {'channel': channel_id, 'include_num_members': 'true'}, secrets)
        return data.get('channel') or {}
    except SystemExit as exc:
        print(f'[warn] conversations.info 실패 — 기본 이름 사용: {exc}')
        return {}


def fetch_history(channel_id: str, secrets: dict[str, str]) -> list[dict]:
    messages: list[dict] = []
    seen: set[str] = set()
    cursor = ''
    while True:
        params = {'channel': channel_id, 'limit': 200}
        if cursor:
            params['cursor'] = cursor
        data = api('conversations.history', params, secrets)
        for m in data.get('messages') or []:
            ts = str(m.get('ts') or '')
            if ts and ts not in seen:
                seen.add(ts)
                messages.append(m)
        cursor = (data.get('response_metadata') or {}).get('next_cursor') or ''
        if not cursor:
            break
    return messages


def fetch_threads(channel_id: str, messages: list[dict], secrets: dict[str, str]) -> int:
    seen = {str(m.get('ts')) for m in messages}
    replies = 0
    parents = [m for m in messages
               if str(m.get('ts')) == str(m.get('thread_ts') or '')
               and int(m.get('reply_count') or 0) > 0]
    for parent in parents:
        cursor = ''
        thread_ts = str(parent.get('ts'))
        while True:
            params = {'channel': channel_id, 'ts': thread_ts, 'limit': 200}
            if cursor:
                params['cursor'] = cursor
            data = api('conversations.replies', params, secrets)
            for m in data.get('messages') or []:
                ts = str(m.get('ts') or '')
                if ts and ts not in seen:
                    seen.add(ts)
                    messages.append(m)
                    replies += 1
            cursor = (data.get('response_metadata') or {}).get('next_cursor') or ''
            if not cursor:
                break
    return replies


def write_zip(out: Path, channel: dict, users: list[dict], messages: list[dict]) -> None:
    ch_name = str(channel.get('name') or DEFAULT_CHANNEL_NAME).replace('/', '-')
    channel_row = {
        'id': str(channel.get('id') or ''),
        'name': ch_name,
        'is_private': bool(channel.get('is_private', True)),
        'num_members': channel.get('num_members') or len(channel.get('members') or []) or '?',
    }
    messages.sort(key=lambda m: float(m.get('ts') or 0))
    out.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(out, 'w', zipfile.ZIP_DEFLATED) as z:
        z.writestr('channels.json', json.dumps([channel_row], ensure_ascii=False))
        z.writestr('users.json', json.dumps(users, ensure_ascii=False))
        z.writestr(f'{ch_name}/messages.json', json.dumps(messages, ensure_ascii=False))
    print(f'저장: {out} ({out.stat().st_size / 1024:.0f} KB)')


def main() -> int:
    here = Path(__file__).resolve().parent
    default_secrets = here / 'slack_secrets.txt'
    if not default_secrets.exists():
        alt = Path.home() / 'Downloads' / 'slack_secrets.txt'
        if alt.exists():
            default_secrets = alt

    ap = argparse.ArgumentParser(description='Slack 채널 → Slackdump 호환 ZIP (로컬 전용)')
    ap.add_argument('--secrets', type=Path, default=default_secrets,
                    help='SLACK_TOKEN / SLACK_COOKIE 파일 (기본 .github/scripts/slack_secrets.txt)')
    ap.add_argument('--channel', default='', help='채널 ID (기본 SLACK_CHANNEL_ID 또는 C0C2CFTCWAH)')
    ap.add_argument('--workspace', default='', help='워크스페이스 서브도메인 (예: 2026kant)')
    ap.add_argument('--out', type=Path, default=here / 'slack.zip', help='출력 ZIP (기본 .github/scripts/slack.zip)')
    args = ap.parse_args()

    secrets = load_secrets(args.secrets)
    if args.workspace:
        secrets['SLACK_WORKSPACE'] = args.workspace
    channel_id = args.channel or secrets.get('SLACK_CHANNEL_ID') or DEFAULT_CHANNEL

    print(f'채널 ID: {channel_id}')
    channel = fetch_channel(channel_id, secrets)
    users = fetch_users(secrets)
    messages = fetch_history(channel_id, secrets)
    replies = fetch_threads(channel_id, messages, secrets)
    print(f'메시지 {len(messages)}개 (스레드 답글 {replies}개) · 사용자 {len(users)}명')
    write_zip(args.out, channel, users, messages)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
