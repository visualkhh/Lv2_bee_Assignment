"""적재함 화면 — 상단 '적재함' 칸에서 Enter. /coordinator/basket 장부를 칸 0~11 순서로 보여준다.

  왼쪽 = 칸 목록(빈 칸 포함) · 오른쪽 = 사용 칸 수 · 손에 든 퍽 · 실은 순서.
"""
from __future__ import annotations

from ...defs import STAY
from ...menus import Menu, MenuItem
from ..page import Page


def _bits(s) -> str:
    parts = [f'tag {s.tag}' if s.tag >= 0 else 'tag ?']
    if s.color:
        parts.append(s.color)
    if s.shape:
        parts.append(s.shape)
    return '  '.join(parts)


def _slot_text(s) -> str:
    """칸 한 줄 — 번호 · 태그 · 색/모양 · 확인 필요."""
    if not s.filled:
        return f'{s.slot:2d}  · 비어 있음'
    return f'{s.slot:2d}  ● {_bits(s)}' + ('   ! 확인 필요' if s.check else '')


class _SlotRow(MenuItem):
    """보기 전용 한 줄 — Enter는 아무 일도 안 함."""
    key = ''

    def __init__(self, app, text: str):
        super().__init__(app)
        self._text = text

    def label(self) -> str:
        return self._text

    def run(self) -> str:
        return STAY


class BasketPage(Page):
    title = '적재함'

    def items(self):
        b = self.state.basket
        if b is None:
            return [_SlotRow(self.app, '적재함 장부 없음 — coordinator가 떠 있는지 확인')]
        return [_SlotRow(self.app, _slot_text(s)) for s in sorted(b.slots, key=lambda s: s.slot)]

    def center(self):
        b = self.state.basket
        if b is None:
            return ['[적재함]', ' /coordinator/basket 을 아직 못 받았습니다.']
        filled = [s for s in b.slots if s.filled]
        check = any(s.check for s in filled)
        out = ['[적재함]', f' {len(filled)}/{len(b.slots)}칸 사용' + ('  — ! 확인 필요' if check else '')]
        held = b.held
        out.append(' 손: ' + (_bits(held) if held.filled else '빈손'))
        out.append(' 실은 순서: ' + (', '.join(map(str, b.order)) if b.order else '-'))
        out.append('')
        out.append(' 번호  내용')
        return out

    def show(self) -> str:
        return self.ui.window.run(self.title, self.items, self.center, back=True, main=True)
