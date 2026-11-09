"""조작 — 모드 · 수동 주행 · 팔 · 그리퍼 · 바퀴 · 비상정지 · 비프음."""
from __future__ import annotations

from ...menus import GoMenu, Menu
from ..manual import ArmMenu, GripperMenu, ManualMenu, WheelMenu
from ..mode import EstopMenu, ModeMenu
from ..status import BeepMenu


class ControlMenu(Menu):
    title = '조작'

    def items(self):
        return [
            GoMenu(self.app, '1', '모드', ModeMenu(self.app)),
            GoMenu(self.app, '2', '수동 주행 / 텔레오퍼', ManualMenu(self.app)),
            GoMenu(self.app, '3', '팔', ArmMenu(self.app)),
            GoMenu(self.app, '4', '그리퍼', GripperMenu(self.app)),
            GoMenu(self.app, '5', '바퀴', WheelMenu(self.app)),
            GoMenu(self.app, '6', '비상정지', EstopMenu(self.app)),
            GoMenu(self.app, '7', '비프음 (OpenCR 부저)', BeepMenu(self.app)),
        ]

    def center(self):
        st = self.state
        return ['[조작]', f' 모드: {st.operation or "-"}',
                f' E-STOP: {"ON" if st.emergency else "OFF"}',
                f' 지도: {st.map_mode or "-"}']


__all__ = ['ControlMenu']
