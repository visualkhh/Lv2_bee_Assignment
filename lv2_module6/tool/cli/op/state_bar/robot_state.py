"""RobotState — 로봇(OpenCR) 한 줄: 연결 진단 · 모터 에러 · 배터리 · 최고 온도 · 팔/그리퍼 전류."""
from __future__ import annotations

from ..consts import ARM_JOINTS, GRIPPER_JOINT
from ..terminal import BOLD, GREEN, RED, YELLOW
from .state import State


class RobotState(State):
    def text(self) -> str:
        st = self.state
        rate = st.rates.get('joint_states')
        age = rate.age() if rate else None
        if age is None:
            return self.term.paint('로봇 ? 상태 대기 중 (/joint_states 없음 — bringup·OpenCR 확인)', YELLOW)
        if age > 3.0:
            return self.term.paint(f'로봇 ✗ {age:.0f}초 끊김 (/joint_states)', RED + BOLD)
        opencr = next(((lv, msg) for name, (lv, msg) in st.diagnostics.items() if 'OpenCR' in name), None)
        errors = [f'{n} {t}' for n, t in st.motor_text.items() if t not in ('', '정상')]
        level = max([opencr[0] if opencr else 0, 2 if errors else 0])
        temps = {n: t for n, t in st.motor_temp.items() if t >= 0}
        hot = max(temps, key=temps.get) if temps else None
        arm_a = sum(abs(st.joint_effort.get(j, 0.0)) for j in ARM_JOINTS)
        grip = st.joint_effort.get(GRIPPER_JOINT)
        bat = st.battery_v
        if bat == bat and bat < 11.0:  # NaN 아님 · 낮음 → 경고
            level = max(level, 1)
        parts = [errors and '모터 에러 ' + ', '.join(errors) or (opencr[1] if opencr else 'OpenCR 진단 -'),
                 f'배터리 {st.battery_summary()}',
                 f'최고 온도 {temps[hot]}°C({hot})' if hot else '온도 -',
                 f'팔 전류 {arm_a:.2f}A',
                 f'그리퍼 {grip * 1000:.0f}mA' if grip is not None else '그리퍼 전류 -']
        mark, color = {0: ('●', GREEN), 1: ('▲', YELLOW)}.get(level, ('✗', RED + BOLD))
        hint = ' → 팔 → 5) 재부팅' if errors else ''
        return self.term.paint(f'로봇 {mark} ' + ' · '.join(parts) + hint, color)
