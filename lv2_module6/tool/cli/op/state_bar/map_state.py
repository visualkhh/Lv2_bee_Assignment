"""MapState — 지도 모드 + 지도 이름 칸."""
from __future__ import annotations

from ..terminal import DIM, GREEN, YELLOW
from .state import State


class MapState(State):
    COLORS = {'MAPPING': YELLOW, 'LOCALIZATION': GREEN}

    def text(self) -> str:
        mode = self.state.map_mode or '?'
        color = self.COLORS.get(mode)
        txt = self.term.paint(mode, color) if color else self.term.paint(mode, DIM)
        name = (self.state.values.get('map_name') or '').strip()
        if name:
            txt += ' ' + self.term.paint(name if len(name) <= 20 else name[:19] + '…', DIM)
        return '지도 ' + txt
