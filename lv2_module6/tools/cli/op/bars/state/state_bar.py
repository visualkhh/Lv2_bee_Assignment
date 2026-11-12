"""StateBar — State 여러 개를 줄(row)로 받아 그리는 상태 막대 (Component). add()로 칸·줄을 더한다 (add-on)."""
from __future__ import annotations

from ..bar import Bar
from .state import State

FOCUS = '\033[7m'   # 포커스된 칸 (반전)


class StateBar(Bar):
    """rows = [[State, …], …] — 줄마다 보이는 칸을 ' | '로 잇는다. 칸이 하나도 안 보이는 줄은 빠진다.
    focusable 칸이 하나라도 있으면 막대도 포커스 가능 — ←→로 칸 이동, activate()는 그 칸의 activate()."""

    sep = ' | '
    indent = '  '

    def __init__(self, ui, rows: list[list[State]] | None = None):
        super().__init__()
        self.ui = ui
        self.rows: list[list[State]] = [list(r) for r in (rows or [])]

    def add(self, state: State, row: int | None = None) -> 'StateBar':
        """칸 추가 — row가 None이면 새 줄, 숫자면 그 줄 끝에."""
        if row is None or row >= len(self.rows):
            self.rows.append([state])
        else:
            self.rows[row].append(state)
        return self

    def focus_states(self) -> list[State]:
        return [s for row in self.rows for s in row if s.focusable]

    @property
    def focusable(self) -> bool:
        return bool(self.focus_states())

    @property
    def focused_state(self) -> State | None:
        fs = self.focus_states()
        return fs[self.idx % len(fs)] if fs else None

    cont_indent = '    '   # 줄바꿈으로 이어지는 줄의 들여쓰기

    def _wrap(self, parts: list[str], width: int) -> list[str]:
        """칸(part)을 sep로 이으며 폭을 넘으면 다음 줄로 — 칸 하나는 쪼개지 않는다 (칸 하나가 폭보다 길면 …로 자름)."""
        term = self.ui.term
        lines, cur, cur_w = [], self.indent, term.disp_len(self.indent)
        first = True
        for p in parts:
            pw = term.disp_len(p)
            add = pw if first else term.disp_len(self.sep) + pw
            if not first and cur_w + add > width:           # 이 칸이 안 들어감 → 새 줄
                lines.append(cur)
                cur, cur_w, first = self.cont_indent, term.disp_len(self.cont_indent), True
                add = pw
            if first and cur_w + pw > width:                 # 칸 하나가 폭보다 김 → 자름
                p = term.fit(p, max(8, width - cur_w)).rstrip()
                pw = term.disp_len(p)
                add = pw
            cur += (p if first else self.sep + p)
            cur_w += add
            first = False
        lines.append(cur)
        return lines

    def lines(self, focused: bool = False) -> list[str]:
        cur = self.focused_state if focused else None
        width = self.ui.term.size()[0] - 1
        out = []
        for row in self.rows:
            parts = []
            for s in row:
                t = s.text()
                if t:
                    parts.append(FOCUS + t + '\033[0m' if s is cur else t)
            if parts:
                out.extend(self._wrap(parts, width))
        return out

    def on_key(self, k) -> bool:
        n = len(self.focus_states())
        if n and k in ('left', 'right'):
            self.idx = (self.idx + (1 if k == 'right' else -1)) % n
            return True
        return False

    def activate(self):
        s = self.focused_state
        return s.activate() if s is not None else None
