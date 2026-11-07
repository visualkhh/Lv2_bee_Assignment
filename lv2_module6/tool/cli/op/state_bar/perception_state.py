"""PerceptionState — 인지(추적 물체 수) 칸."""
from __future__ import annotations

from .state import State


class PerceptionState(State):
    def text(self) -> str:
        return f'인지 {self.state.tracked_count:3d}개'
