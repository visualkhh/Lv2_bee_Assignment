#!/usr/bin/env python3
"""현장 진단 미션 + 녹화 (미션 메뉴 x) 시험 — ROS 없이: python3 tools/cli/tests/test_diag_run.py

확인: ① record_run.sh --regex 토픽 목록 (미션·skill·TF … 들어감 · 영상 빠짐) ② bagrec extra 옵션이 명령줄에 들어감
③ 미션이 끝나면(DONE·ERROR) 우리 녹화만 끔 · 시작 안 하면(거부) 바로 끔 · 다른 녹화로 바뀌었으면 안 건드림
"""
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # tools/cli
from ros.fake.FakeRos import FakeRos  # noqa: E402

FakeRos.install()
from op import bagrec  # noqa: E402
from op.pages.mission import RECORD_OPTS, RECORD_RUN, MissionMenu, stop_bag_when_done  # noqa: E402

# ① 토픽 목록
regex = re.compile(subprocess.run([str(RECORD_RUN), '--regex'], capture_output=True, text=True, check=True).stdout.strip())
for t in ('/mission/state', '/mission/signal_status', '/task/status', '/task/current_plan', '/task/execute_plan/_action/feedback',
          '/coordinator/basket', '/tracked_objects', '/diagnostics_agg', '/tf', '/tf_static', '/odom', '/odometry/filtered',
          '/cmd_vel', '/cmd_vel_nav', '/joint_states', '/collision_monitor_state', '/plan', '/cmd/mission',
          '/navigate_to_pose/_action/feedback', '/skills/pick/_action/status', '/rosout'):
    assert regex.match(t), t
for t in ('/pole_front_cam/color/image_raw', '/perception/gripper_cam/debug_image', '/mission/dice_image', '/map',
          '/coordinator/basket_clear', '/tfx'):
    assert not regex.match(t), t

# ② bagrec 명령줄
cmd = bagrec.bag_command(Path('/x/b'), regex='^/a$', extra=RECORD_OPTS)
assert cmd[cmd.index('-o') - 4:cmd.index('-o')] == RECORD_OPTS and cmd[-3:] == ['--regex', '^/a$', '--include-hidden-topics'], cmd


# ③ 자동 끄기
class Mission:
    def __init__(self, phase):
        self.phase = phase


class State:
    mission = Mission('DONE')   # 지난 회차


class Rec:
    def __init__(self, bag):
        self.bag, self.stopped = bag, 0

    def current(self):
        return {'bag': self.bag} if not self.stopped else None

    def stop(self):
        self.stopped += 1


def run_async(state, rec, **kw):
    out = {}
    th = threading.Thread(target=lambda: out.setdefault('why', stop_bag_when_done(state, '/b/1', rec, poll_s=0.01, **kw)))
    th.start()
    return th, out


st, rec = State(), Rec('/b/1')
th, out = run_async(st, rec)
time.sleep(0.05)
st.mission = Mission('GO_OBSERVE')    # 시작됨
time.sleep(0.05)
assert rec.stopped == 0
st.mission = Mission('SURVEY')
time.sleep(0.05)
st.mission = Mission('ERROR')        # abort·치명 → ERROR
th.join(2)
assert out['why'] == 'ERROR' and rec.stopped == 1, (out, rec.stopped)

st, rec = State(), Rec('/b/1')       # 시작 거부 — 끝 단계 그대로
th, out = run_async(st, rec, start_wait_s=0.1)
th.join(2)
assert out['why'] == 'not_started' and rec.stopped == 1

st, rec = State(), Rec('/b/other')   # 사람이 다른 녹화로 바꿈 — 안 건드림
st.mission = Mission('BUILD')
th, out = run_async(st, rec)
time.sleep(0.05)
st.mission = Mission('DONE')
th.join(2)
assert out['why'] == 'DONE' and rec.stopped == 0

# 메뉴 키 — 한 번만 · develop 키와 안 겹침 (#199 l·d·a도 피함)
app = type('App', (), {'state': type('S', (), {'failures': []})(), 'bus': None, 'ui': None, 'term': None})()
menu = MissionMenu.__new__(MissionMenu)
menu.app, menu.state = app, app.state
keys = [i.key for i in MissionMenu.items(menu)]
assert keys.count('x') == 1 and not {'l', 'd', 'a'} & set(keys), keys
print('ok test_diag_run')
