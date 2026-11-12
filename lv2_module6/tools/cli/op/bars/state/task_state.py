"""TaskState — 미션 칸: 미션 단계+경과 · 지금 skill+경과 · 마지막 실패 이유(빨강). Enter → 미션 화면."""
from __future__ import annotations

import time

from ...terminal import BOLD, DIM, GREEN, RED, YELLOW
from .state import State

FAIL_SHOW_S = 300.0   # 마지막 실패 이유를 상단바에 이만큼 남김 (이력은 미션 화면)


class TaskState(State):
    focusable = True

    def activate(self):
        return self.ui.window.navigate('9')

    def text(self) -> str:
        st, now = self.state, time.monotonic()
        parts = []
        m = st.mission
        if m is not None:
            color = RED + BOLD if m.phase == 'ERROR' else GREEN if m.phase == 'DONE' else YELLOW if m.phase != 'IDLE' else DIM
            parts.append(self.term.paint(f'{m.phase} {now - st.mission_since:.0f}s', color))
        task = st.task
        if task is not None:
            plan_id, step, state, reason = task
            skill = st.current_skill()
            name = f'{skill[0]} {skill[1]}'.strip() if skill else f'step {step}'
            if state == 0:
                parts.append(f'{name} {now - st.task_since:.0f}s')
            elif state == 2:
                pass   # 실패는 아래 ✗ 이유에 step·skill까지 나옴
            else:
                parts.append(self.term.paint(f'{name} 끝', DIM))
        if st.failures and now - st.failures[-1][1] < FAIL_SHOW_S:
            _, t, _, reason = st.failures[-1]
            parts.append(self.term.paint(f'✗ {reason} ({now - t:.0f}s)', RED + BOLD))
        return '미션 ' + (' · '.join(parts) if parts else '-')
