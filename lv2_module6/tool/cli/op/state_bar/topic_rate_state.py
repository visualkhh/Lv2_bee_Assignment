"""TopicRateState — 토픽 하나의 수신 Hz 칸 (키·라벨·기준 Hz를 받아 여러 개 둔다)."""
from __future__ import annotations

from ..terminal import GREEN, RED, YELLOW
from .state import State


class TopicRateState(State):
    """StateStore.rates[key] — 없으면 ✗, 3초 넘게 끊기면 빨강, low_hz 미만이면 노랑."""

    def __init__(self, ui, key: str, label: str, low_hz: float):
        super().__init__(ui)
        self.key, self.label, self.low_hz = key, label, low_hz

    def text(self) -> str:
        rate = self.state.rates.get(self.key)
        age = rate.age() if rate else None
        if age is None:
            return self.term.paint(f'{self.label} ✗', RED)
        if age > 3.0:
            return self.term.paint(f'{self.label} {age:.0f}s 끊김', RED)
        hz = rate.hz()
        return self.term.paint(f'{self.label} {hz:.0f}Hz', GREEN if hz >= self.low_hz else YELLOW)
