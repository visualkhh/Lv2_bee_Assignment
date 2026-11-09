"""InputDialog — Dialog 하위 클래스."""
from __future__ import annotations

from ..terminal import DIM, RED
from .dialog import CONTINUE, Dialog

class InputDialog(Dialog):
    """한 줄 입력 — Enter 확인(빈칸=default) · Esc/◀ 취소(None).
    validate(text) → 거부 사유 문자열(None=통과). 거부면 창을 닫지 않고 사유를 보여준다."""

    def __init__(self, ui, title: str, body: list[str], prompt: str, default: str = '', validate=None):
        super().__init__(ui, title, body)
        self.prompt, self.default, self.validate = prompt, default, validate
        self.buf, self.error = '', ''

    def content(self):
        return super().content() + [self.prompt + ' ' + self.buf + '▏',
                                    self.ui.term.paint(self.error, RED) if self.error else '',
                                    self.ui.term.paint('Enter 확인 · Esc 취소', DIM)]

    def extra_width(self):
        return self.ui.term.disp_len(self.prompt) + 2

    def on_key(self, k):
        if k == 'enter':
            value = self.buf if self.buf else self.default
            self.error = (self.validate(value) or '') if self.validate else ''
            return value if not self.error else CONTINUE
        if k in ('esc', 'left', 'eof'):
            return None
        self.error = ''
        if k == 'backspace':
            self.buf = self.buf[:-1]
        elif k == 'ctrl_u':
            self.buf = ''
        elif isinstance(k, str) and len(k) == 1 and k.isprintable():
            self.buf += k
        return CONTINUE

    def fallback(self):
        try:
            raw = input(f'{self.prompt} ').strip()
            return raw if raw else self.default
        except EOFError:
            return None
