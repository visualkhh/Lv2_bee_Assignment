"""KeyDialog — Dialog 하위 클래스."""
from __future__ import annotations

from .dialog import CONTINUE, Dialog

class KeyDialog(Dialog):
    """단일 키 확인 — Enter='enter' · Esc/◀=None · 문자는 그대로 (confirm의 y/n 등)."""

    def on_key(self, k):
        if k == 'enter':
            return 'enter'
        if k in ('esc', 'left', 'eof'):
            return None
        if isinstance(k, str) and len(k) == 1 and k.isprintable():
            return k.lower()
        return CONTINUE

    def fallback(self):
        try:
            raw = input(f'{self.title} (Enter/b/m/q): ').strip().lower()
            return {'': 'enter', 'b': 'b', 'm': 'm', 'q': 'q'}.get(raw, raw or 'enter')
        except EOFError:
            return None
