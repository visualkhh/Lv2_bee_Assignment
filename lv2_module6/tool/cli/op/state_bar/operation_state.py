"""OperationState — 조작 모드 칸."""
from __future__ import annotations

from ..terminal import BOLD, CYAN, GREEN, RED, YELLOW
from .state import State


class OperationState(State):
    COLORS = {'AUTO': GREEN, 'MANUAL': YELLOW, 'EXPLORE': CYAN, 'TEACH': RED}

    def text(self) -> str:
        op = self.state.operation or '?'
        return '조작 ' + self.term.paint(op, self.COLORS.get(op, RED) + BOLD)
