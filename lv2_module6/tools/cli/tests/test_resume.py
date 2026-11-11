#!/usr/bin/env python3
"""재연결(exec) 뒤 보던 화면 복원 — python3 tools/cli/tests/test_resume.py
열린 화면 항목의 키가 resume_path로 쌓이고, 새 프로세스가 resume_queue로 같은 경로를 되짚는지."""
import sys
from pathlib import Path
from types import SimpleNamespace as N

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from ros.fake.FakeRos import FakeRos  # noqa: E402

FakeRos.install()
import op.pages  # noqa: E402,F401 — menus↔pages 순환 import 순서
from op.defs import MAIN, QUIT, STAY  # noqa: E402
from op.menus.item import MenuItem  # noqa: E402
from op.window.window import Window  # noqa: E402

seen = {}


class Leaf(MenuItem):
    shows_busy = False

    def __init__(self, app, key, **kw):
        self.app, self.key = app, key
        self.ui = N(window=N(perform=lambda label, fn, busy: fn()))
        for k, v in kw.items():
            setattr(self, k, v)

    def label(self):
        return 'x'

    def run(self):
        seen['path'] = list(self.app.resume_path)
        return STAY


app = N(resume_path=[])
# 1) 화면 항목이 열려 있는 동안만 키가 쌓이고, 끝나면 빠진다
Leaf(app, '4').activate()
assert seen['path'] == ['4'] and app.resume_path == []
# 2) 동작 항목(shows_busy)·resumable=False는 안 쌓음 — 재시작을 부르는 동작을 다시 보내지 않게
Leaf(app, '3', shows_busy=True).activate()
assert seen['path'] == []
Leaf(app, '1', resumable=False).activate()
assert seen['path'] == []

# 3) 복원 — 진짜 Window.run이 resume_queue의 키 항목을 자동으로 연다 (없는 키면 복원 중단 후 평소처럼 키 대기)
class Item:
    def __init__(self, key, log):
        self.key, self._log = key, log

    def label(self):
        return self.key

    def activate(self):
        self._log.append(self.key)
        return STAY

    def visible(self):
        return True


def make_window(queue):
    w = Window.__new__(Window)
    w.resume_queue, w._once_idx = list(queue), {}
    w.emit = lambda *a: []
    w._wait_key = lambda *a: 'q'          # 복원이 끝나면 평소 키 대기 — q로 빠져나옴
    return w


sys.stdin = N(isatty=lambda: True)
try:
    log = []
    w = make_window(['4'])
    assert w.run('t', lambda: [Item('4', log), Item('8', log)], lambda: []) == QUIT
    assert log == ['4'] and w.resume_queue == [], (log, w.resume_queue)
    log = []
    w = make_window(['9', '1'])            # 없는 키 — 아무것도 열지 않고 중단
    assert w.run('t', lambda: [Item('4', log)], lambda: []) == QUIT
    assert log == [] and w.resume_queue == []
finally:
    sys.stdin = sys.__stdin__
print('PASS test_resume')
