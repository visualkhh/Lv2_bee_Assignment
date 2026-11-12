"""RobotState · PowerState — 로봇(OpenCR) 두 줄.
  모터 줄  : 바퀴 · 팔 · 그리퍼를 *그룹별로 한 칸씩* 요약 (모터마다 나열하지 않음 — 팔 6개가 "응답 없음"이어도 한 칸)
  전원 줄  : 배터리 · 팔 전류 · 그리퍼 전류
예전엔 한 줄에 전부 이어 붙여 화면 폭을 넘고 뒤가 잘렸다."""
from __future__ import annotations

from ...consts import ARM_JOINTS, GRIPPER_JOINT, WHEEL_JOINTS
from ...terminal import BOLD, DIM, GREEN, RED, YELLOW
from .state import State

NO_REPLY = '응답 없음'                         # /motor_status 텍스트 (firmware E 줄 −1)
GROUPS = (('바퀴', WHEEL_JOINTS, True), ('팔', ARM_JOINTS, False), ('그리퍼', (GRIPPER_JOINT,), False))  # (이름, 모터, 없으면 에러?)


class _LinkState(State):
    """공통 — /joint_states가 없거나 끊기면 안내 한 줄 (모터 줄만 보임), 정상이면 None."""

    def link_text(self) -> str | None:
        rate = self.state.rates.get('joint_states')
        age = rate.age() if rate else None
        if age is None:
            return self.term.paint('로봇 ? 상태 대기 중 (/joint_states 없음 — bringup·OpenCR 확인)', YELLOW)
        if age > 3.0:
            return self.term.paint(f'로봇 ✗ {age:.0f}초 끊김 (/joint_states)', RED + BOLD)
        return None


class RobotState(_LinkState):
    """모터 줄 — 바퀴 ● 33°C · 팔 ✗ 응답 없음 · 그리퍼 ✗ 응답 없음"""

    def group(self, label: str, names, must: bool) -> tuple[int, str]:
        """(심각도 0 정상 · 1 주의 · 2 에러, 칸 글자). must = 이 그룹이 없으면 에러 (바퀴)."""
        st = self.state
        texts = [st.motor_text.get(n, '') for n in names]
        dead = [n for n, t in zip(names, texts) if t == NO_REPLY]
        errs = [f'{t}({n})' for n, t in zip(names, texts) if t not in ('', '정상', NO_REPLY)]
        if errs:
            return 2, f'{label} ✗ ' + ', '.join(errs)
        if len(dead) == len(names):
            return (2 if must else 1), f'{label} ✗ {NO_REPLY}'
        if dead:
            return 1, f'{label} ▲ 일부 응답 없음 ({len(dead)}/{len(names)})'
        temps = [st.motor_temp.get(n, -1) for n in names]
        hot = max(temps)
        return 0, f'{label} ● ' + (f'{hot}°C' if hot >= 0 else '정상')

    def text(self) -> str:
        down = self.link_text()
        if down:
            return down
        opencr = next(((lv, msg) for name, (lv, msg) in self.state.diagnostics.items() if 'OpenCR' in name), None)
        parts, worst, errors = [], 0, False
        for label, names, must in GROUPS:
            level, txt = self.group(label, names, must)
            worst = max(worst, level)
            errors = errors or (level == 2 and '✗' in txt and NO_REPLY not in txt)
            parts.append(self.term.paint(txt, {0: GREEN, 1: YELLOW}.get(level, RED + BOLD)))
        # OpenCR 진단 메시지는 *연결 자체* 문제(포트 닫힘 · S 끊김)일 때만 — 모터 에러 목록까지 붙은 긴 문장을 그대로 싣지 않는다
        #   (모터 상태는 위 그룹 칸이 이미 말해 준다)
        if opencr and opencr[0] >= 2 and ('포트 닫힘' in opencr[1] or '수신 없음' in opencr[1]):
            parts.insert(0, self.term.paint(opencr[1].split(';')[0].split(' → ')[0], RED + BOLD))
            worst = 2
        hint = self.term.paint('  → 팔 → 5) 재부팅', DIM) if errors else ''
        mark, color = {0: ('●', GREEN), 1: ('▲', YELLOW)}.get(worst, ('✗', RED + BOLD))
        return self.term.paint(f'모터 {mark} ', color) + ' · '.join(parts) + hint


class PowerState(_LinkState):
    """전원 줄 — 배터리 12.1V (80% 추정) · 팔 전류 0.00A · 그리퍼 0mA  (연결이 없으면 안 보임 — 모터 줄이 안내)"""

    def text(self) -> str:
        if self.link_text():
            return ''
        st = self.state
        parts = [f'배터리 {st.battery_summary()}']
        arm_alive = any(st.motor_text.get(j, '') not in ('', NO_REPLY) for j in ARM_JOINTS)
        if arm_alive:
            parts.append(f'팔 전류 {sum(abs(st.joint_effort.get(j, 0.0)) for j in ARM_JOINTS):.2f}A')
        grip = st.joint_effort.get(GRIPPER_JOINT)
        if grip is not None and st.motor_text.get(GRIPPER_JOINT, '') not in ('', NO_REPLY):
            parts.append(f'그리퍼 {grip * 1000:.0f}mA')
        bat = st.battery_v
        low = bat == bat and bat < 11.0   # NaN 아님 · 낮음 → 경고
        return self.term.paint('전원 ' + ('▲ ' if low else '● ') + ' · '.join(parts), YELLOW if low else GREEN)
