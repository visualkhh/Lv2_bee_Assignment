#!/usr/bin/env python3
"""적재함 칸·화면 단위 시험 — ROS 없이: python3 tools/cli/tests/test_basket_view.py

확인: ① 칸 문구 '적재함 2/12 · 손' + 확인 필요 '!' ② Enter → 'basket' 화면 ③ 화면 줄 = 칸 0~11 전부
④ 빈 칸/찬 칸 표기 ⑤ 장부 없으면 안내
"""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # tools/cli
from op.bars.state.basket_state import BasketState  # noqa: E402
from op.pages.basket import BasketPage  # noqa: E402


def slot(n, filled=False, tag=-1, color='', shape='box', check=False):
    return NS(slot=n, filled=filled, tag=tag, color=color, shape=shape, check=check)


def basket(slots, held=None, order=None):
    return NS(slots=slots, held=held or slot(-1), order=order or [])


class _Win:
    went = None
    def navigate(self, name, **kw):
        _Win.went = name
        return 'back'


def cell(b):
    ui = NS(window=_Win(), term=NS(paint=lambda s, _c: s), state=NS(basket=b))
    c = BasketState(ui)
    c.state = ui.state
    c.term = ui.term
    return c


# ① 칸 문구
b = basket([slot(i, filled=i < 2, tag=i if i < 2 else -1, color='red' if i == 0 else '')
            for i in range(12)],
           held=slot(-1, filled=True, tag=5, color='green'), order=[0, 1])
c = cell(b)
assert c.text() == '적재함 2/12 · 손', c.text()
assert c.focusable and c.activate() == 'back' and _Win.went == 'basket'

# ② 확인 필요 표시
b2 = basket([slot(0, filled=True, tag=3, check=True), *[slot(i) for i in range(1, 12)]])
assert cell(b2).text() == '적재함 1/12 !', cell(b2).text()

# ③ 화면 — 칸 0~11 전부, 빈 칸/찬 칸 표기
class _App:
    state = NS(basket=b)
    bus = NS()
    ui = NS()
    term = NS()
    def get_clock(self):
        raise RuntimeError

page = BasketPage(_App())
rows = page.items()
assert len(rows) == 12, len(rows)
assert rows[0].label().startswith(' 0  ● tag 0  red  box'), rows[0].label()
assert rows[2].label() == ' 2  · 비어 있음', rows[2].label()
center = page.center()
assert center[1].startswith(' 2/12칸 사용'), center
assert ' 손: tag 5  green  box' in center, center
assert center[3] == ' 실은 순서: 0, 1', center

# ④ 장부 없음
empty = BasketPage(NS(state=NS(basket=None), bus=NS(), ui=NS(), term=NS()))
assert '장부 없음' in empty.items()[0].label()
assert cell(None).text() == ''

print('PASS test_basket_view')
