"""화면 틀 — Window가 상태바(op/state_bar)·메뉴 막대(op/menu_bar)·중앙·힌트를 조합해 그리고 키 루프·신호를 돌린다.

  Window      window.py      최상위 — 조합·그리기·키 루프·신호(tick · navigate)

부품은 모두 Component(op/component.py) 하위: MenuItem(pages/base.py) · State · StateBar(state_bar/) · MenuBar · ChoiceItem(menu_bar/).
"""
from .window import Window

__all__ = ['Window']
