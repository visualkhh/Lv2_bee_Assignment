"""ControllerState — 주행 제어기 RL/DWB 칸 · CollisionState — 충돌 감시 켬/끔 + 지금 동작 칸."""
from __future__ import annotations

import time

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
    def text(self) -> str:
        on, action = collision_text(self.state)
        if on == '-' and not action:
            return ''
        short = on.split(' ')[0]   # '켬 (자동)' · '끔 (MANUAL — …)' · '거절: …' → 앞 낱말만 (자세히는 미션 화면)
        on_s = self.term.paint(short, DIM if on == '-' else GREEN if short == '켬' else YELLOW)
        return '충돌감시 ' + on_s + (' ' + self.term.paint(action, RED + BOLD) if action else '')
