"""state_manager — 조작·지도 모드의 주인. /state/*와 /diagnostics를 한 화면에."""
from __future__ import annotations

from .feature import NodeFeature


class StateManagerFeature(NodeFeature):
    node = 'state_manager'

    def status_lines(self, bus, name: str, summary: bool = False) -> list[str]:
        st = bus.state
        op = st.operation or '-'
        mm = st.map_mode or '-'
        mn = st.values.get('map_name') or '-'
        if summary:
            return [f' 상태 {op} · {mm}' + (f' · {mn}' if mn != '-' else '')]

        lines = ['[상태 관리]']
        lines.append(f' 조작 {op} · 지도 {mm} · {mn}')
        lines.append(f' 비상정지 {"ON" if st.emergency else "OFF"}')

        for diag_name, values in st.diag_values.items():
            if 'state_manager' in diag_name:
                d = dict(values)
                for key in ('끊긴 장치', '지도 경로', '재시작 준비', '재시작', '탐사'):
                    v = d.get(key)
                    if v and v != '-':
                        lines.append(f' {key}: {v}')
                break

        devs = (st.values.get('devices') or '').strip()
        if devs:
            offline = []
            for dev in devs.split(','):
                dev = dev.strip()
                detail = st.device_details.get(dev) or ''
                if 'disconnected' in detail:
                    offline.append(dev)
            lines.append(f' 장치 {devs}' + (f' · 끊김 {", ".join(offline)}' if offline else ''))

        if st.values.get('restart'):
            lines.append(f' 최근 재시작: {st.values["restart"]}')
        return lines
