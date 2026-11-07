"""SayState — 마지막 발화 칸 (없으면 안 보임)."""
from __future__ import annotations

from .state import State


class SayState(State):
    def text(self) -> str:
        say = self.state.say_last or ''
        if not say:
            return ''
        return '발화 ' + (say if len(say) <= 22 else say[:21] + '…')
