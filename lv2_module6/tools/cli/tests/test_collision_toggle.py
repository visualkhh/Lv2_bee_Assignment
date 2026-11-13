#!/usr/bin/env python3
"""상단 '충돌감시' 칸 토글 단위 시험 — ROS 없이: python3 tools/cli/tests/test_collision_toggle.py

확인: ① 칸이 포커스 가능하고 Enter → 다이얼로그(화면 이동 없음) ② 켬이면 '끌까요?' · 현재/적용 문구 · 예 → cmd_collision(off) · 아니오 → 호출 없음
③ 끔이면 '켤까요?' · 예 → on ④ 상태를 모르면(-·거절) 확인창도 호출도 없음 ⑤ 끄기만 기본값 '아니오'(실수로 Enter 쳐도 안 꺼짐), 켜기는 기본 '예'
"""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # tools/cli
from ros.fake.FakeRos import FakeRos  # noqa: E402

FakeRos.install()
from op.bars.state.nav_state import CollisionState  # noqa: E402
from op.defs import STAY  # noqa: E402


class _Win:
    went = None
    def navigate(self, name, **kw):
        _Win.went = name
        return 'back'


def run(cur, answer):
    calls, asked, notes = [], [], []
    ui = NS(
        bus=NS(set_collision=lambda want: (calls.append('on' if want else 'off'), (True, '켬' if want else '끔'))[1]),
        confirm=lambda prompt, default=False, body=None: (asked.append((prompt, default, list(body or []))), answer)[1],
        modal_key=lambda title, lines: notes.append((title, list(lines))),
        window=_Win(), term=NS())
    cell = CollisionState(NS(window=ui.window, term=ui.term, state=NS(collision_enabled=lambda: cur)))
    cell.ui = ui
    r = cell.activate()
    return r, calls, asked, notes


# ① 포커스 가능 · 화면 이동 없음
r, calls, asked, notes = run('켬 (자동)', False)
assert r == STAY and _Win.went is None, '다이얼로그로 처리 — 화면 이동 없음'

# ② 켬 → 끄기 제안
r, calls, asked, _ = run('켬 (자동)', True)
assert r == STAY and calls == ['off'], (r, calls)
prompt, default, body = asked[0]
assert prompt == '충돌 감시를 끌까요?' and default is False, 'Enter만 눌러도 안 꺼짐'
assert body[0] == '현재: 켬 (자동)' and body[1].startswith('적용: 끔') and any('끄면' in b for b in body), body
r, calls, _, _ = run('켬 (자동)', False)
assert calls == [], '아니오면 호출 없음'

# ③ 끔 → 켜기 제안
r, calls, asked, _ = run('끔 (MANUAL — 근접 정지 없이 통과)', True)
assert calls == ['on'], calls
prompt, default, body = asked[0]
assert prompt == '충돌 감시를 켤까요?' and default is True
assert body[0].startswith('현재: 끔 (MANUAL') and body[1] == '적용: 켬', body

# ④ 모르는 상태 — 확인창·호출 없이 안내만
for cur in ('-', '거절: toggle failed', ''):
    r, calls, asked, notes = run(cur, True)
    assert r == STAY and calls == [] and asked == [], (cur, calls, asked)
    assert notes and any('모름' in line for line in notes[0][1]), (cur, notes)

print('PASS test_collision_toggle')
