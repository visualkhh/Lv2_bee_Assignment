#!/usr/bin/env python3
"""StateStore 미션 디버그 단위 시험 — ROS 없이: python3 tools/cli/tests/test_mission_state.py"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # tools/cli
from ros.fake.FakeRos import FakeRos  # noqa: E402

FakeRos.install()
from op.state import StateStore  # noqa: E402
from op.terminal import Terminal  # noqa: E402
from op.pages.mission import mission_lines  # noqa: E402
from ros.msgs import _Msg  # noqa: E402

st = StateStore()
st.on_sub_goals(_Msg(plan_id=9, goals=[_Msg(step=1, skill='navigate', target='1,2')]))
st.on_task(9, 1, 0, '')
assert st.current_skill() == ('navigate', '1,2')
st.on_task(9, 1, 2, 'nav_no_progress')
st.on_task(9, 1, 2, 'nav_no_progress')          # latched 재수신 — 이력에 한 번만
assert [f[3] for f in st.failures] == ['navigate#1: nav_no_progress'], st.failures
st.on_mission(_Msg(phase='ERROR', error='skill_unavailable:pick', note='', barcode='', count_r=0, count_y=0,
                   count_g=0, order='', placed=0, total=0, results=[], tray=[]))
assert st.failures[-1][2:] == ('미션', 'skill_unavailable:pick')

st.on_map_odom(0.0, 0.0, 0.0)
st.on_map_odom(0.01, 0.0, 0.0)                    # 조금 — 점프 아님
st.on_map_odom(0.30, 0.0, math.radians(1))        # 큰 보정 — 점프
assert st.slam_jumps == 1 and abs(st.slam_last_jump[0] - 0.29) < 1e-6

text = '\n'.join(mission_lines(st, Terminal(color=False)))
for want in ('[미션] ERROR', 'skill_unavailable:pick', 'map→odom 점프 1회', 'nav_no_progress'):
    assert want in text, (want, text)
print('PASS test_mission_state')
