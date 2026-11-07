"""MenuBar — 메뉴 항목(Component) 여러 개를 세로로 늘어놓는 막대. ↑↓ · 숫자 · 글자로 포커스."""
from __future__ import annotations

import time

from ..component import Component

# 메뉴에서 항목 키로 못 쓰는 글자 (내비게이션 전용 — Window가 먼저 가져감)
NAV_KEYS = ('b', 'm', 'q', 'r')
HIGHLIGHT = '\033[7m\033[36m\033[1m'   # 포커스된 줄 (반전 · 청록 · 굵게)


class MenuBar(Component):
    """items = 각자 key·label()·activate()를 가진 Component (MenuItem · ChoiceItem).
    ↑↓ 이동 · 숫자(1초 안에 이어 치면 10·11…) · 글자(항목 key)로 포커스."""

    focusable = True

    def __init__(self):
        self.items: list = []
        self.idx = 0
        self._dbuf, self._dat = '', 0.0

    def set_items(self, items: list, idx: int | None = None) -> None:
        self.items = list(items)
        if idx is not None:
            self.idx = idx
        self.idx = self.idx % len(self.items) if self.items else 0

    @property
    def focused_item(self):
        return self.items[self.idx] if self.items else None

    def lines(self, focused: bool = True) -> list[str]:
        out = []
        for i, it in enumerate(self.items):
            row = f"  {'❯' if i == self.idx else ' '} {it.key}) {it.label()}"
            out.append(self._paint(row) if i == self.idx and focused else row)
        return out

    def _paint(self, row: str) -> str:
        return HIGHLIGHT + row + '\033[0m'

    def on_key(self, k) -> bool:
        if not self.items:
            return False
        if k in ('up', 'down'):
            self.idx = (self.idx - 1 if k == 'up' else self.idx + 1) % len(self.items)
            self._dbuf = ''
            return True
        keys = [it.key for it in self.items]
        if isinstance(k, str) and len(k) == 1 and k.isdigit():
            now = time.monotonic()
            if now - self._dat > 1.0:
                self._dbuf = ''
            self._dbuf += k
            self._dat = now
            exact = [i for i, key in enumerate(keys) if key == self._dbuf]
            longer = any(key.startswith(self._dbuf) and key != self._dbuf for key in keys)
            if exact:
                self.idx = exact[0]
                if not longer:
                    self._dbuf = ''
                return True
            self._dbuf = ''
            return False
        self._dbuf = ''
        if isinstance(k, str) and len(k) == 1 and k.lower() not in NAV_KEYS:
            hit = [i for i, key in enumerate(keys) if key == k.lower()]
            if hit:
                self.idx = hit[0]
                return True
        return False
