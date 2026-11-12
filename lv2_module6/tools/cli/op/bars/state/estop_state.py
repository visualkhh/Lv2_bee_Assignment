"""EstopState — 비상정지 칸."""
from __future__ import annotations

from ...terminal import BOLD, GREEN, RED
from .state import State


class EstopState(State):
    def text(self) -> str:
        on = self.state.emergency
        return 'E-STOP ' + (self.term.paint('ON', RED + BOLD) if on else self.term.paint('OFF', GREEN))
