"""ControllerState — 주행 제어기 RL/DWB 칸 · CollisionState — 충돌 감시 켬/끔 + 지금 동작 칸."""
from __future__ import annotations

import time

from ...defs import STAY
from ...terminal import BOLD, DIM, GREEN, RED, YELLOW
from .state import State

# nav2_msgs/CollisionMonitorState action_type
COLLISION_ACTIONS = {0: '', 1: '정지', 2: '감속', 3: '접근정지', 4: '속도제한'}


def controller_text(st) -> str:
    """/controller_server/FollowPath/status → 'RL' · 'DWB(점구름/스캔 없음·stale)' — 2 s 넘게 없으면 ''."""
    from ...pages.nodes.controller_server import INNER, REASONS
    s = st.rl_status
    if not s or time.monotonic() - st.rl_status_time > 2.0:
        return ''
    if s[0] >= 0.5:
        return 'RL'
    inner = INNER.get(int(s[9]), '기본') if len(s) > 9 else '기본'
    why = REASONS.get(int(s[1]), '') if len(s) > 1 else ''
    return f'{inner}({why})' if why else inner


class ControllerState(State):
    def text(self) -> str:
        t = controller_text(self.state)
        return f'주행 {self.term.paint(t, GREEN if t == "RL" else YELLOW)}' if t else ''


def collision_text(st) -> tuple[str, str]:
    """(켬/끔 문자열, 지금 동작) — 동작은 collision_monitor가 바뀔 때만 발행하니 마지막 값."""
    action = ''
    if st.collision is not None:
        (kind, polygon), _ = st.collision
        action = COLLISION_ACTIONS.get(kind, str(kind))
        action = f'{action}({polygon})' if action and polygon else action
    return st.collision_enabled(), action


class CollisionState(State):
    focusable = True

    def activate(self):
        """칸에서 Enter — 지금 상태의 반대로 바꿀지 '현재: · 적용:'을 보여 주고 확인하면 바로 적용한다.
        켬/끔을 모르면(-·거절) 묻지도 바꾸지도 않는다. 적용은 ./run-cli collision on|off와 같다 (/cmd/collision)."""
        cur = self.state.collision_enabled()
        short = cur.split(' ')[0]   # '켬 (자동)' · '끔 (MANUAL — …)' · '거절: …' · '-'
        if short not in ('켬', '끔'):
            self.ui.modal_key('충돌 감시', [
                f'상태를 모름 ({cur})',
                'Nav2(collision_monitor)와 manual_control이 떠 있는지 확인하세요.'])
            return STAY
        want_on = short == '끔'
        apply = '켬' if want_on else '끔 (근접 정지 없이 통과 — 주의)'
        body = [f'현재: {cur}', f'적용: {apply}', '']
        if not want_on:
            body.insert(2, '끄면 벽·물체 앞에서 자동으로 서지 않습니다. 직접 몰 때만 쓰세요.')
        body.append('직접 바꾸면 모드가 바뀔 때까지 유지됩니다 (MANUAL·MAPPING은 자동으로 끔)')
        if not self.ui.confirm('충돌 감시를 켤까요?' if want_on else '충돌 감시를 끌까요?', default=want_on, body=body):
            return STAY
        try:
            ok, text = self.ui.bus.set_collision(want_on)
        except Exception as exc:   # 서비스 응답 형식이 달라도 TUI가 죽지 않게
            ok, text = False, str(exc)
        self.ui.modal_key('충돌 감시', [('적용: ' if ok else '실패: ') + text])
        return STAY

    def text(self) -> str:
        on, action = collision_text(self.state)
        if on == '-' and not action:
            return ''
        short = on.split(' ')[0]   # '켬 (자동)' · '끔 (MANUAL — …)' · '거절: …' → 앞 낱말만 (자세히는 미션 화면)
        on_s = self.term.paint(short, DIM if on == '-' else GREEN if short == '켬' else YELLOW)
        return '충돌감시 ' + on_s + (' ' + self.term.paint(action, RED + BOLD) if action else '')
