"""BasketState — 상단 '적재함' 칸. /coordinator/basket 장부에서 찬 칸 수를 센다. Enter → 적재함 화면."""
from __future__ import annotations

from ...terminal import BOLD, DIM, GREEN, RED, YELLOW
from .state import State


def basket_counts(b) -> tuple[int, int, bool]:
    """(찬 칸 수, 전체 칸 수, 확인 필요 있음)."""
    if b is None:
        return 0, 0, False
    filled = [s for s in b.slots if s.filled]
    return len(filled), len(b.slots), any(s.check for s in filled)


class BasketState(State):
    focusable = True

    def activate(self):
        return self.ui.window.navigate('basket')

    def text(self) -> str:
        b = self.state.basket
        if b is None:
            return ''
        n, total, check = basket_counts(b)
        color = DIM if n == 0 else YELLOW if check else GREEN
        held = ' · 손' if b.held.filled else ''
        return ('적재함 ' + self.term.paint(f'{n}/{total}', color) + held
                + (' ' + self.term.paint('!', RED + BOLD) if check else ''))
