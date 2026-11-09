"""메뉴 항목 — MenuItem(공통) · ActionItem · GoMenu · InfoItem."""
from __future__ import annotations

from abc import ABC, abstractmethod

from ..component import Component
from ..defs import STAY


class MenuItem(Component, ABC):
    """메뉴 항목 한 개 — 라벨·노출·실행을 한 몸에. LeftSideMenuBar에 들어가는 Component."""

    key: str = ''
    focusable = True
    shows_busy = True   # 누르면 '처리 중' 창 (하위 화면으로 들어가는 항목은 바로 열리니 False)

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
        """BACK/MAIN/QUIT/STAY — 프레임은 Window가, 이동은 반환값으로."""
        ...

    def activate(self) -> str:
        """Enter로 골랐을 때 · 소스에서 직접 부를 때 — '처리 중' 창 · print 가로채 결과창 · run()."""
        return self.ui.window.perform(self.label(), self.run, self.shows_busy)


class ActionItem(MenuItem):
    """메뉴의 동작 한 줄 — label(문자열 또는 함수) · fn() 실행 후 STAY (바로 끝나니 '처리 중' 창 없음). 화면 안 동작 메뉴용."""
    shows_busy = False

    def __init__(self, app, key: str, label, fn):
        super().__init__(app)
        self.key, self._label, self._fn = key, label, fn

    def label(self) -> str:
        return self._label() if callable(self._label) else self._label

    def run(self) -> str:
        self._fn()
        return STAY


class GoMenu(MenuItem):
    """하위 화면 진입 항목."""
    shows_busy = False  # 하위 화면을 연다

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
