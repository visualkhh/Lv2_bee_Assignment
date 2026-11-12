#!/usr/bin/env python3
"""Notion 공개 팀 페이지 스크래퍼 (AI가 작성).

⚠️ 이 스크립트는 **AI가 작성한 자동 수집기**입니다. 사람이 글을 옮겨 적지 않고,
   Playwright로 공개 페이지의 표를 읽어 `.notion_cache.txt`에 저장합니다.
   - Notion은 내부 스크롤(`.notion-scroller.vertical`)을 쓰므로 **500px씩 내려가며**
     각 구간(회의록·투두·트러블슈팅·개인 일지)을 읽는다. 한 번에 맨 아래로 점프하면
     중간 표가 lazy-load되지 않아 0건으로 잡힌다 (실측).
   - 구간별로 가장 길게 읽힌 텍스트를 채택해 합친다.
   - 실패해도 CI는 계속 진행하되, 리포트는 README를 덮어쓰지 않는다.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

from playwright.sync_api import sync_playwright

URL = 'https://app.notion.com/p/teamsparta/B-_BEE-3de2dc3ef51480ce978edf40651c6b35'
OUT = Path(os.environ.get('NOTION_TEXT', '.notion_cache.txt'))

SECTIONS = {
    'meetings': ('✦ 회의록', '✦ 프로젝트 타임라인'),
    'todo': ('✦ 투두리스트', '✦ 트러블슈팅'),
    'trouble': ('✦ 트러블슈팅', '✦ 개인 일지'),
    'journal': ('✦ 개인 일지', None),
}

# Notion마다 스크롤 요소 class가 달라질 수 있어, 실제 overflow 스크롤 요소를 찾는다.
FIND_SC = """(() => {
  const list = [...document.querySelectorAll('*')].filter(e => {
    const s = getComputedStyle(e);
    return (s.overflowY === 'auto' || s.overflowY === 'scroll')
      && e.scrollHeight > e.clientHeight + 100 && e.clientHeight > 300;
  });
  list.sort((a, b) => b.scrollHeight - a.scrollHeight);
  return list[0] || null;
})()"""


def scroller_text(page) -> str:
    return page.evaluate(
        f"() => {{ const sc = {FIND_SC}; return sc ? sc.innerText : document.body.innerText; }}"
    )


def scroll_height(page) -> int:
    return int(page.evaluate(
        f"() => {{ const sc = {FIND_SC}; return sc ? sc.scrollHeight : document.body.scrollHeight; }}"
    ) or 0)


def scroll_to(page, y: int) -> None:
    page.evaluate(
        f"(y) => {{ const sc = {FIND_SC}; if (sc) sc.scrollTop = y; else window.scrollTo(0, y); }}",
        y,
    )


def section(text: str, start: str, end: str | None) -> str | None:
    i = text.find(start)
    if i < 0:
        return None
    j = text.find(end, i + len(start)) if end else -1
    return text[i:j] if j >= 0 else text[i:]


def main() -> int:
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        # Notion 표는 화면에 들어올 때 행을 lazy-load한다. 뷰포트를 페이지 높이만큼 크게 잡아
        # 회의록·투두·트러블슈팅·개인 일지를 한 번에 렌더시킨다 (1200px에서는 아래 표가 0건).
        page = browser.new_page(viewport={'width': 1600, 'height': 10_000})
        try:
            page.goto(URL, wait_until='domcontentloaded', timeout=90_000)
            page.wait_for_timeout(5_000)

            best = {key: '' for key in SECTIONS}
            y = 0
            height = scroll_height(page)
            while y <= height + 1_500 and y < 30_000:
                scroll_to(page, y)
                page.wait_for_timeout(350)
                text = scroller_text(page)
                for key, (start, end) in SECTIONS.items():
                    sec = section(text, start, end)
                    if sec and len(sec) > len(best[key]):
                        best[key] = sec
                y += 500
                height = max(height, scroll_height(page))
        finally:
            browser.close()

    text = (best['meetings'] + '\n✦ 프로젝트 타임라인\n'
            + best['todo'] + '\n' + best['trouble'] + '\n' + best['journal'])
    if '개인 일지' not in text or not best['journal']:
        print('Notion 페이지에서 개인 일지를 찾지 못했습니다 (로그인/비공개/로딩 실패).', file=sys.stderr)
        return 1
    OUT.write_text(text, encoding='utf-8')
    print(f'wrote {OUT} ({len(text)} chars)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
