"""State — 상단 상태바 한 칸의 최상위 부모 (Component). 칸마다 하위 클래스가 text()를 정한다."""
from __future__ import annotations

from ..component import Component


class State(Component):
    """상태바 한 칸. text() = 그 칸 문자열 (색 포함) — 빈 문자열이면 그 칸은 안 보인다.
    focusable = True인 칸은 Tab으로 상태바에 들어가 ←→로 고르고 Enter → activate() (예: DiagState → 진단 화면).
    ui = Ui (term · state(StateStore) · window · 재연결 상태를 쓴다)."""

    def __init__(self, ui):
        self.ui = ui
        self.term = ui.term
        self.state = ui.state

    def text(self) -> str:
        raise NotImplementedError
