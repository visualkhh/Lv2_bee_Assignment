"""Page — 화면 하나의 최상위 부모. show()는 BACK/MAIN/QUIT/STAY 중 하나를 돌려준다."""
from __future__ import annotations

from abc import ABC, abstractmethod


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
