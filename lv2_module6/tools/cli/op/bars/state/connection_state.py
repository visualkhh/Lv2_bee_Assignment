"""ConnectionState — 연결 칸 (마지막 /state/* 수신 뒤 경과 · 벽시계 시각 · 재연결 대기 중이면 남은 초)."""
from __future__ import annotations

import time

from ...terminal import BOLD, GREEN, RED, YELLOW
from .state import State


class ConnectionState(State):
    def text(self) -> str:
        last = self.state.state_last_update
        if last <= 0:
            return self.term.paint('연결 ✗ 상태 없음 — bringup 확인', RED + BOLD)
        age = time.monotonic() - last
        wall = getattr(self.state, 'state_last_wall', 0.0)
        when = time.strftime('%H:%M:%S', time.localtime(wall)) if wall > 0 else '-'
        status = getattr(self.ui, '_reconnect_status', None)
        retry = status() if status is not None else None
        if retry is not None:
            return self.term.paint(f'연결 ✗ {age:.0f}s — {retry} · 마지막 {when}', RED + BOLD)
        if age <= 3.0:   # /state/devices 하트비트 2s + 여유
            return self.term.paint(f'연결 ● 정상 {age:.1f}s · 마지막 {when}', GREEN)
        if age <= 6.0:
            return self.term.paint(f'연결 ▲ {age:.1f}s 지연 · 마지막 {when}', YELLOW)
        return self.term.paint(f'연결 ✗ {age:.0f}s 끊김 · 마지막 {when} — bringup 확인', RED + BOLD)
