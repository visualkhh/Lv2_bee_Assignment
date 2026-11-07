"""DeviceState — 장치 active 수 칸."""
from __future__ import annotations

from ..terminal import DIM, GREEN, YELLOW
from .state import State


class DeviceState(State):
    def text(self) -> str:
        devices = self.state.devices
        if not devices:
            return '장치 ' + self.term.paint('0/0', DIM)
        active = sum(1 for n in devices if self.state.device_info(n)[1] == 'active')
        return '장치 ' + self.term.paint(f'{active}/{len(devices)}', GREEN if active == len(devices) else YELLOW)
