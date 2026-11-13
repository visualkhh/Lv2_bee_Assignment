#!/usr/bin/env python3
"""새 창을 못 열 때 TUI 안에서 따라 보기 시험 — ROS 없이: python3 tools/cli/tests/test_follow_view.py

확인: ① tmux 밖 · 화면 없음이면 open_window가 FollowDialog로 (안내 문구 포함) ② 출력 줄이 쌓이고 맨 아래를 따라감 ·
화면 지우기(\\033[J)면 새 화면만 ③ ↑로 멈춤 · End로 다시 따라감 · q로 닫힘 ④ 닫으면 명령(자기 세션)이 끝남
"""
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # tools/cli
from ros.fake.FakeRos import FakeRos  # noqa: E402

FakeRos.install()
from op.defs import STAY  # noqa: E402
from op.dialog import CONTINUE, FollowDialog  # noqa: E402
from op.pages import mission  # noqa: E402
from op.terminal import Terminal  # noqa: E402


class Ui:
    term = Terminal(color=False)

    def tick(self, _s):
        pass


def settle(dlg, want, s=3.0):
    end = time.monotonic() + s
    while time.monotonic() < end:
        dlg.on_tick()
        if want(dlg.body):
            return
        time.sleep(0.02)
    raise AssertionError(dlg.body)


# ① 폴백
seen = {}
mission.FollowDialog = lambda ui, title, cmd, note: type('D', (), {'run': lambda self: seen.update(title=title, cmd=cmd, note=note)})()
for k in ('TMUX', 'DISPLAY', 'WAYLAND_DISPLAY'):
    os.environ.pop(k, None)
assert mission.open_window(Ui(), 'logs', ['echo', 'x'], '최근 로그', '설명') == STAY
assert seen['cmd'] == ['echo', 'x'] and '새 창을 못 열어' in seen['note'] and '설명' in seen['note'], seen
mission.FollowDialog = FollowDialog

# ② 줄 쌓기 · 따라가기 · 화면 지우기
py = sys.executable
dlg = FollowDialog(Ui(), 't', [py, '-c', 'for i in range(30): print(i)'], 'n')
dlg.h = 5
settle(dlg, lambda b: b and b[-1].startswith('(명령 끝남'))
assert dlg.body[:30] == [str(i) for i in range(30)] and dlg.follow and dlg.off == len(dlg.body) - 5
assert 'n' in dlg.content()[1]
# ③ 스크롤
assert dlg.on_key('up') is CONTINUE and not dlg.follow and dlg.off == len(dlg.body) - 6
assert dlg.on_key('end') is CONTINUE and dlg.follow
assert dlg.on_key('q') == STAY
dlg.stop()

dlg = FollowDialog(Ui(), 't', [py, '-c', r'print("old"); print("\033[H\033[Jnew1\nnew2")'], '')
settle(dlg, lambda b: b and b[-1].startswith('(명령 끝남'))
assert dlg.body[:2] == ['new1', 'new2'], dlg.body

# ④ 닫으면 명령도 끝
dlg = FollowDialog(Ui(), 't', [py, '-c', 'import time\nwhile True: print(1, flush=True); time.sleep(0.05)'], '')
settle(dlg, lambda b: len(b) > 2)
dlg.stop()
assert dlg.proc.poll() is not None, '닫았는데 명령이 계속 돎'
print('PASS')
