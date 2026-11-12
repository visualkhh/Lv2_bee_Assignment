"""SlamState — cuVSLAM odom Hz + map→odom 점프 횟수·마지막 크기 (TF를 0.5 s마다 보고 TUI에서 셈)."""
from __future__ import annotations

from ...terminal import GREEN, RED, YELLOW
from .state import State

LOW_HZ = 7   # cuVSLAM odom이 이보다 느리면 노랑 (카메라 15 Hz의 절반)


class SlamState(State):
    def text(self) -> str:
        st = self.state
        rate = st.rates.get('cuvslam')
        if st.map_odom is None and rate is None:
            return ''   # SLAM 안 띄움 (지도 NONE · fake)
        age = rate.age() if rate else None
        if age is None or age > 3.0:
            hz = self.term.paint('끊김', RED)
        else:
            v = rate.hz()
            hz = self.term.paint(f'{v:.0f}Hz', GREEN if v >= LOW_HZ else YELLOW)
        if not st.slam_last_jump:
            return f'SLAM {hz} 점프 0'
        d, deg, at = st.slam_last_jump
        return f'SLAM {hz} ' + self.term.paint(f'점프 {st.slam_jumps} (최근 {d:.2f}m {deg:.0f}° {at})', YELLOW)
