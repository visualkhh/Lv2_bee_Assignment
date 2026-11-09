"""PagerDialog — Dialog 하위 클래스."""
from __future__ import annotations

from ..defs import STAY
from ..terminal import DIM
from .dialog import CONTINUE, Dialog

class PagerDialog(Dialog):
    """결과 페이저 — ↑↓ 스크롤 · Enter/Esc/◀ 닫기(메뉴·종료 이동 없음)."""

    def __init__(self, ui, title: str, body: list[str]):
        super().__init__(ui, title, body or ['(출력 없음)'])
        import shutil
        try:
            _, rows = shutil.get_terminal_size()
        except Exception:
            rows = 24
        self.h = max(5, min(20, rows - 10))
        self.off = 0

    def _clamp(self):
        self.off = max(0, min(self.off, max(0, len(self.body) - self.h)))

    def content(self):
        self._clamp()
        vis = self.body[self.off:self.off + self.h]
        pos = (f'{self.off + 1}-{min(self.off + self.h, len(self.body))}/{len(self.body)} · '
               if len(self.body) > self.h else '')
        return [self.title, ''] + vis + [self.ui.term.paint(f'{pos}↑↓ 스크롤 · Enter/Esc 닫기', DIM)]

    def on_key(self, k):
        if k in ('enter', 'esc', 'left', 'eof'):
            return STAY
        step = {'up': -1, 'down': 1, 'pgup': -self.h, 'pgdn': self.h}.get(k)
        if step is not None:
            self.off += step
        elif k == 'home':
            self.off = 0
        elif k == 'end':
            self.off = len(self.body)
        self._clamp()
        return CONTINUE

    def fallback(self):
        self.ui.console.write('\n'.join(self.body) + '\n')
        self.ui.console.flush()
        return STAY
