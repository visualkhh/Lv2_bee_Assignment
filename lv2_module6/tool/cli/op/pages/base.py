"""Page(기반) → Menu(선택 루프) → 각 메뉴 화면.

메뉴 항목은 MenuItem 객체다: 라벨·노출 조건·실행을 캡슐화해서
키 문자열 if-chain 없이 다형성으로 동작한다.

흐름 규약 (모든 메뉴 공통):
  - 항목 run()은 BACK/MAIN/QUIT/STAY만 돌려준다. 화면 이동은 반환값으로만.
    BACK은 그 항목이 연 하위 화면이 닫혔다는 뜻 — 부모 메뉴는 그대로 남는다.
  - 실행 중 print()는 전부 Menu가 가로채 결과창(pager)에 보여준다 — 다이얼로그보다 먼저
    알려야 할 경고·맥락은 print가 아니라 confirm(body=...)/modal_input(body=...)에 넣는다.
  - 취소(Esc·아니오)는 아무 출력 없이 STAY.
"""
from __future__ import annotations

from abc import ABC, abstractmethod

import rclpy

from ..defs import BACK, MAIN, QUIT, STAY, TIMEOUT


class Page(ABC):
    """한 화면. show()는 BACK/MAIN/QUIT/STAY 중 하나를 돌려준다."""

    title: str = ''

    def __init__(self, app):
        self.app = app
        self.state = app.state
        self.bus = app.bus
        self.ui = app.ui
        self.term = app.term

    @abstractmethod
    def show(self) -> str:
        ...


class MenuItem(ABC):
    """메뉴 항목 한 개 — 라벨·노출·실행을 한 몸에."""

    key: str = ''

    def __init__(self, app):
        self.app = app
        self.state = app.state
        self.bus = app.bus
        self.ui = app.ui
        self.term = app.term

    def label(self) -> str:
        return ''

    def visible(self) -> bool:
        return True

    @abstractmethod
    def run(self) -> str:
        """BACK/MAIN/QUIT/STAY — 프레임은 Menu가, 이동은 반환값으로."""
        ...


class GoMenu(MenuItem):
    """하위 화면 진입 항목."""

    def __init__(self, app, key: str, label: str, page):
        super().__init__(app)
        self.key = key
        self._label = label
        self._page = page

    def label(self) -> str:
        return self._label

    def run(self) -> str:
        return self._page.show() if self._page is not None else STAY


class InfoItem(MenuItem):
    """고를 게 없을 때 보여주는 안내 한 줄 (Enter는 아무 일도 안 함)."""

    def __init__(self, app, text: str, key: str = '1'):
        super().__init__(app)
        self.key = key
        self._text = text

    def label(self) -> str:
        return self._text

    def run(self) -> str:
        return STAY


class Menu(Page):
    """방향키 메뉴 — items()의 MenuItem들을 보여주고 고른 항목의 run()을 실행."""

    #: 최상위 메뉴 — 뒤로·메인 없음
    root = False

    def items(self) -> list[MenuItem]:
        return []

    def center(self) -> list[str]:
        return []

    def show(self) -> str:
        self.ui.reset_selection()  # 메뉴 진입은 항상 첫 항목에 포커스
        while rclpy.ok():
            self.app.poll(0.2)
            visible = [i for i in self.items() if i.visible()]
            if not visible:
                return STAY
            keys = [(it.key, it.label()) for it in visible]
            choice = self.ui.select(self.title, keys, self.center(),
                                    back=not self.root, main=not self.root)
            if choice == TIMEOUT:
                continue
            if choice in (BACK, MAIN, QUIT):
                return choice
            item = next((it for it in visible if it.key == choice), None)
            if item is None:
                continue
            # 실행 중 print는 화면을 스크롤시키므로 가로채서 결과창에 넣는다.
            with self.ui.capture() as (buf_out, buf_err):
                result = item.run()
            out = (buf_out.getvalue() + buf_err.getvalue()).strip('\n')
            if out:
                self.ui.pager(item.label(), out)
            # 하위 화면의 BACK은 '그 화면을 닫음'이라 여기는 그대로 남는다 (두 단계 뒤로 가지 않게)
            if result in (MAIN, QUIT):
                return result
        return QUIT
