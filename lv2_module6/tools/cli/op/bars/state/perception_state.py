"""PerceptionState — 지금 보이는 물체 수 칸 (/tracked_objects — 2 s 안 보이면 빠짐)."""
from __future__ import annotations

from .state import State


class PerceptionState(State):
    def text(self) -> str:
        return f'보이는 물체 {self.state.tracked_count}'
