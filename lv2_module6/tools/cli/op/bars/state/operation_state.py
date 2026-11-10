"""OperationState — 조작 모드 칸 (TEACH는 MANUAL의 토크 꺼짐 상태로 표시)."""
from __future__ import annotations

from ...terminal import BOLD, CYAN, GREEN, RED, YELLOW
from .state import State


class OperationState(State):
    COLORS = {'AUTO': GREEN, 'MANUAL': YELLOW}

    def text(self) -> str:
        op = self.state.operation or '?'
        label = op
        return '조작 ' + self.term.paint(label, self.COLORS.get(op, RED) + BOLD)
