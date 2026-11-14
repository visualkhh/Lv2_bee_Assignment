#!/usr/bin/env python3
"""README의 GitHub/Notion/Slack 기여도 표를 읽어 맨 위 OVERALL_STATS 블록을 만든다.

세 섹터 데이터를 다시 API로 긁지 않는다. 각 섹터 워크플로가 갱신한
`### 🧮 기여도 (데이터 기반)` 표를 읽어 사람별로 합산한다.
종합 % = 세 섹터 %의 단순 평균 (자료가 없는 섹터는 0%로 계산).

사용:
  python3 .github/scripts/overall_stats.py
  python3 .github/scripts/overall_stats.py --stdout
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

START = '<!-- OVERALL_STATS:START -->'
END = '<!-- OVERALL_STATS:END -->'
GITHUB_START = '<!-- GITHUB_STATS:START -->'
NOTION_START = '<!-- NOTION_STATS:START -->'
SLACK_START = '<!-- SLACK_STATS:START -->'
SECTORS = (
    ('GitHub', GITHUB_START, '<!-- GITHUB_STATS:END -->'),
    ('Notion', NOTION_START, '<!-- NOTION_STATS:END -->'),
    ('Slack', SLACK_START, '<!-- SLACK_STATS:END -->'),
)


def bar(value: float, top: float, width: int = 16) -> str:
    if top <= 0:
        return '·' * width
    fill = 0 if value <= 0 else max(1, round(value / top * width))
    fill = min(fill, width)
    return '█' * fill + '·' * (width - fill)


def normalize_name(name: str) -> str:
    """`visualkhh (김현하)` → `김현하`, `김현하` → `김현하`."""
    m = re.match(r'^(.*?)\s*\(([^()]*)\)\s*$', name)
    return (m.group(2) if m else name).strip()


def parse_sector(text: str, start: str, end: str) -> dict[str, float]:
    if start not in text or end not in text:
        return {}
    block = text[text.index(start):text.index(end)]
    lines = block.splitlines()
    # `### 🧮 기여도` 아래 첫 표를 읽는다.
    for i, line in enumerate(lines):
        if line.strip().startswith('###') and '기여도' in line:
            j = i + 1
            while j < len(lines) and not lines[j].lstrip().startswith('| 이름'):
                j += 1
            out: dict[str, float] = {}
            for row in lines[j + 1:]:
                if not row.lstrip().startswith('|'):
                    break
                cells = [c.strip() for c in row.strip().strip('|').split('|')]
                if len(cells) < 2 or not cells[1].endswith('%'):
                    continue
                try:
                    pct = float(cells[1].rstrip('%'))
                except ValueError:
                    continue
                name = normalize_name(cells[0])
                if name:
                    out[name] = pct
            return out
    return {}


def build(text: str) -> str:
    data: dict[str, dict[str, float]] = {}
    for sector, start, end in SECTORS:
        for name, pct in parse_sector(text, start, end).items():
            data.setdefault(name, {})[sector] = pct
    if not data:
        raise SystemExit('README에서 GitHub/Notion/Slack 기여도 표를 찾지 못했습니다.')

    rows = []
    for name, vals in data.items():
        g = vals.get('GitHub', 0.0)
        n = vals.get('Notion', 0.0)
        s = vals.get('Slack', 0.0)
        overall = (g + n + s) / 3.0
        rows.append((name, g, n, s, overall))
    rows.sort(key=lambda r: (-r[4], r[0]))

    lines = [
        START,
        '## 🧮 종합 기여도 (GitHub · Notion · Slack)',
        '',
        '> 세 섹터의 **기계적 비율을 단순 평균**한 값입니다. 자료가 없는 섹터는 0%로 계산합니다. '
        '막대는 **종합 1위=100%** 기준.',
        '',
        '| 이름 | GitHub | Notion | Slack | 종합 | 그래프 |',
        '|:---|---:|---:|---:|---:|:---:|',
    ]
    top_overall = max((r[4] for r in rows), default=100.0)
    for name, g, n, s, overall in rows:
        lines.append(
            f'| {name} | {g:.1f}% | {n:.1f}% | {s:.1f}% | **{overall:.1f}%** | '
            f'`{bar(overall, top_overall)}` |'
        )
    lines += ['', END]
    return '\n'.join(lines)


def update_readme(path: Path, block: str) -> bool:
    text = path.read_text(encoding='utf-8') if path.exists() else ''
    if START in text and END in text:
        before = text[:text.index(START)]
        after = text[text.index(END) + len(END):]
        new = before.rstrip() + '\n\n' + block + '\n\n' + after.lstrip('\n')
    elif GITHUB_START in text:
        idx = text.index(GITHUB_START)
        new = text[:idx].rstrip() + '\n\n' + block + '\n\n' + text[idx:]
    else:
        new = text.rstrip() + '\n\n' + block + '\n'
    if new == text:
        return False
    path.write_text(new, encoding='utf-8')
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description='README 종합 기여도 블록')
    ap.add_argument('--readme', default=Path('README.md'), type=Path)
    ap.add_argument('--stdout', action='store_true')
    args = ap.parse_args()

    text = args.readme.read_text(encoding='utf-8')
    block = build(text)
    if args.stdout:
        print(block)
        return 0
    changed = update_readme(args.readme, block)
    print(f'{args.readme} ' + ('갱신' if changed else '변경 없음'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
