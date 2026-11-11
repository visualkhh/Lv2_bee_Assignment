#!/usr/bin/env python3
"""Ui.choose 시험 — ROS 없이: python3 tools/cli/tests/test_ui_choose.py

예전 문제(2026-11-10): 컨트롤러 고르기 화면이 select 를 한 번만 불렀다. select 는 키가 없으면 MENU_REFRESH_S(2 s)마다
TIMEOUT 을 돌려주는데 그걸 '취소'로 처리해 화면이 2초 뒤 이전 페이지로 돌아갔다.
확인: ① TIMEOUT 이 몇 번 와도 계속 기다렸다가 고른 key 를 돌려줌 ② BACK·MAIN·QUIT 은 그대로 돌려줌.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # tools/cli
from ros.fake.FakeRos import FakeRos  # noqa: E402

FakeRos.install()
from op.defs import BACK, TIMEOUT  # noqa: E402
from op.ui import Ui  # noqa: E402


def make_ui(answers):
    ui = Ui.__new__(Ui)
    seq = list(answers)
    calls = []

    def select(title, items, center, back=True, main=True, timeout=None):
        calls.append(title)
        return seq.pop(0)
    ui.select = select
    return ui, calls


def test_waits_through_timeouts():
    ui, calls = make_ui([TIMEOUT, TIMEOUT, TIMEOUT, '2'])
    assert ui.choose('t', [('1', 'a'), ('2', 'b')], []) == '2'
    assert len(calls) == 4, calls   # TIMEOUT 3번 뒤 선택 — 중간에 빠져나가지 않음


def test_back_passes_through():
    ui, calls = make_ui([TIMEOUT, BACK])
    assert ui.choose('t', [('1', 'a')], []) == BACK
    assert len(calls) == 2


if __name__ == '__main__':
    test_waits_through_timeouts()
    test_back_passes_through()
    print('PASS test_ui_choose')
