"""PoseState — 로봇의 지금 map 위치 칸 (TF map → base_footprint). TF가 없으면 '위치 -'."""
from __future__ import annotations

import math

from ...terminal import DIM
from .state import State


class PoseState(State):
    def text(self) -> str:
        p = self.state.map_pose
        if p is None:
            return '위치 ' + self.term.paint('-', DIM)
        return f'위치 x={p[0]:+.2f} y={p[1]:+.2f} yaw={math.degrees(p[2]):+.0f}°'
