"""ConnectionState — 연결 칸 (마지막 /state/* 수신 뒤 경과 · 재연결 대기 중이면 남은 초)."""
from __future__ import annotations

import time

from ..terminal import BOLD, GREEN, RED, YELLOW
from .state import State


class ConnectionState(State):
    def text(self) -> str:
        last = self.state.state_last_update
        if last <= 0:
            return self.term.paint('연결 ✗ 상태 없음 — bringup 확인', RED + BOLD)
        age = time.monotonic() - last
        status = getattr(self.ui, '_reconnect_status', None)
        retry = status() if status is not None else None
        if retry is not None:
            return self.term.paint(f'연결 ✗ {age:.0f}s — {retry}', RED + BOLD)
        if age <= 2.0:
            return self.term.paint('연결 ● 정상', GREEN)
        if age <= 5.0:
            return self.term.paint(f'연결 ▲ {age:.0f}s 지연', YELLOW)
        return self.term.paint(f'연결 ✗ {age:.0f}s 끊김 — bringup 확인', RED + BOLD)
