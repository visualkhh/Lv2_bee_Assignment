"""NoDataState — ROS 상태를 하나도 못 받았을 때만 보이는 경고 칸."""
from __future__ import annotations

from ...terminal import YELLOW
from .state import State


class NoDataState(State):
    def text(self) -> str:
        st = self.state
        if st.values or st.devices or st.joints:
            return ''
        return self.term.paint('상태 없음 — bringup이 떠 있는지 확인', YELLOW)
