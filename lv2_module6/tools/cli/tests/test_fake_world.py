#!/usr/bin/env python3
"""fake 월드 시험 — ROS 없이: python3 tools/cli/tests/test_fake_world.py
가짜 bringup을 띄워 명령(수동 주행 · goal · 미션 · skill)에 가짜 월드가 반응하는지 본다."""
import runpy
import sys
import time
from pathlib import Path

CLI = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(CLI))
sys.argv = ['cli', '--source', 'fake', 'status']
g = runpy.run_path(str(CLI / '__main__.py'), run_name='fake_world_test')   # 대역 설치 + OperatorTool 패치만 (main은 안 탐)
ot = g['ot']
app = ot.OperatorTool()
w = app.world
bus = app.bus


def run(sec):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        app.poll(0.05)


run(0.3)
assert app.state.rates['joint_states'].hz() > 5, '가짜 joint_states가 흘러야 함'
assert app.state.rates['odom'].hz() > 5

# 수동 주행: MANUAL에서 cmd_vel을 보내면 로봇이 움직임 (벽에 막히면 멈추니 시작 위치 기준으로 비교)
app.state.values['operation_mode'] = 'MANUAL'
x0, yaw0 = w.x, w.yaw
for _ in range(10):
    bus._send_velocity(0.0, 0.8)
    run(0.05)
assert abs(w.yaw - yaw0) > 0.1, '제자리 회전이 반영돼야 함'
run(0.8)
assert w.wz == 0.0, '명령이 끊기면 정지'

# AUTO 아니면 goal 거부 · AUTO면 경로가 생기고 도착
try:
    w.send_nav_goal({'x': 0.0, 'y': 0.0})
    raise SystemExit('MANUAL에서 goal이 받아들여짐')
except RuntimeError:
    pass
app.state.values['operation_mode'] = 'AUTO'
w.set_robot_pose(w.x, w.y, 0.0)
tx, ty = w.x + 0.6, w.y
w.send_nav_goal({'x': tx, 'y': ty})
assert w.path, '경로가 있어야 함'
run(0.7)
assert w._cell(w.x, w.y) is not None
for _ in range(80):
    run(0.1)
    if not w.path:
        break
assert abs(w.x - tx) < 0.15 and abs(w.y - ty) < 0.15, (w.x, w.y, tx, ty)

# goal의 방향(yaw) — 도착하면 제자리에서 그 방향으로 돌아 서야 함 (왼쪽 +90°)
import math  # noqa: E402
w.set_robot_pose(w.x, w.y, 0.0)
w.send_nav_goal({'x': w.x + 0.4, 'y': w.y, 'yaw_deg': 90.0})
for _ in range(120):
    run(0.1)
    if not w.nav_active:
        break
assert not w.nav_active and abs(((w.yaw - math.radians(90.0)) + math.pi) % (2 * math.pi) - math.pi) < 0.1, math.degrees(w.yaw)

# 미션 시작 → 단계가 /mission/state로 나옴 · abort → ERROR
bus.cmd_mission('start')
run(0.3)
assert app.state.mission is not None and app.state.mission.phase == 'GO_OBSERVE', app.state.mission
bus.cmd_mission('abort')
run(0.2)
assert app.state.mission.phase == 'ERROR' and app.state.mission.error == 'aborted'

# 세부 명령 (coordinator 흉내) — pose는 성공, 모르는 skill은 실패
bus.cmd_skill('pose', 'scan')
run(0.2)
assert 'RUNNING' in app.state.task_status, app.state.task_status
run(2.0)
assert 'SUCCEEDED' in app.state.task_status, app.state.task_status
bus.cmd_skill('nope', 'x')
run(0.2)
assert 'FAILED' in app.state.task_status and 'unknown_skill' in app.state.task_status, app.state.task_status
app.state.values['operation_mode'] = 'MANUAL'
bus.cmd_skill('pose', 'stow')
run(0.2)
assert 'FAILED' in app.state.task_status, 'MANUAL에서는 거부돼야 함'
print('PASS test_fake_world')

# 지도 화면에 쓰는 토픽 — costmap(c)·경로(p)가 fake에서도 흘러온다 (MapPage.show와 같은 구독)
from nav_msgs.msg import Path as PathMsg  # noqa: E402
from op.pages.map import MapPage  # noqa: E402
from op.views.map import CostGrid  # noqa: E402
from ros.fake.FakeRos import LATEST  # noqa: E402
app.state.values['operation_mode'] = 'AUTO'
w.set_robot_pose(w.x, w.y, 0.0)
page = MapPage(app, mode='view')
for kind, topic in (('plan', '/plan'), ('local_plan', '/local_plan')):
    app.create_subscription(PathMsg, topic, lambda m, k=kind: page._on_path(k, m), None)
from plan.msg import SubGoalArray  # noqa: E402
app.create_subscription(SubGoalArray, '/task/sub_goals', page._on_goals, None)
w.send_nav_goal({'x': w.x + 1.0, 'y': w.y})
run(1.2)
page.plan_on = True
bus.cmd_skill('navigate', f'{w.x + 0.5:.2f},{w.y:.2f}')   # 경유점 — 실행 중인 navigate 목표가 번호로 찍힘
run(0.3)
assert len(page._waypoint_pts()) == 1 and '경유점' in page._plan_lines()[1], page._plan_lines()
lines, info = page._plan_lines()
assert len(lines) == 2 and '전역' in info and '지역' in info, info
cg = CostGrid.from_msg(LATEST['/global_costmap/costmap'])
assert any(cg.code(*w._world(c, r)) == 2 for r in range(0, w.h, 3) for c in range(0, w.w, 3)), 'costmap에 벽(내접)이 있어야 함'
assert LATEST['/local_costmap/costmap'].info.width > 10
print('PASS test_fake_world (map topics)')

# 저장 지도 폴더 = ros/fake/map/<이름> — 목록 · 보기(map.yaml) · 전환 · 저장
import shutil  # noqa: E402
from op.views.map import MapGrid  # noqa: E402
from ros.fake.FakeRos import CURRENT, MAP_DIR, fake_map_names  # noqa: E402
assert bus.default_map_dir() == MAP_DIR, bus.default_map_dir()
assert bus.map_names() == fake_map_names() and 'default' in bus.map_names(), bus.map_names()
assert MapGrid.from_yaml(MAP_DIR / 'default' / 'map.yaml').w > 0   # 지도 목록 → 보기가 읽는 파일
app.state.values['operation_mode'] = 'MANUAL'
bus.cmd_map(bus.map_args('switch', name='20261017_223015'))
app.poll(0.3)
assert CURRENT['name'] == '20261017_223015', CURRENT
tmp = MAP_DIR / '_test_saved'
try:
    assert bus.cmd_map(bus.map_args('save', file=str(tmp / 'map'), localize=False)) == 0
    assert (tmp / 'map.yaml').is_file() and (tmp / 'map.pgm').is_file(), '저장하면 폴더에 지도가 생겨야 함'
finally:
    shutil.rmtree(tmp, ignore_errors=True)
    CURRENT['name'] = 'default'
print('PASS test_fake_world (map store)')

# 가짜 state_manager 생존 신호 — 계속 오지 않으면 CLI가 '연결 ✗'로 보고 스스로 재시작함
run(1.3)
assert time.monotonic() - app.state.state_last_update < 2.0, '/state/* 생존 신호가 끊김'
print('PASS test_fake_world (heartbeat)')
