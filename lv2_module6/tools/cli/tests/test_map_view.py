#!/usr/bin/env python3
"""map_view 단위 시험 — ROS 없이: python3 tools/cli/tests/test_map_view.py"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))   # tools/cli
from ros.fake.FakeRos import FakeRos  # noqa: E402

FakeRos.install()
from op.views.map import FREE, OCC, OUT, MapGrid, MapView, Viewport  # noqa: E402

# 8×8 빈 지도에 벽 한 칸 — 아무리 축소해도 벽이 안 사라져야 한다 (벽 우선 피라미드)
data = [0] * 64
data[3 * 8 + 5] = 100
g = MapGrid(8, 8, 0.1, -0.4, -0.4, data)
assert g.code(0, -0.4 + 5.5 * 0.1, -0.4 + 3.5 * 0.1) == OCC
assert g.code(0, 0.0, 0.0) == FREE and g.code(0, 9.0, 9.0) == OUT
assert all(g.code(k, -0.4 + 5.5 * 0.1, -0.4 + 3.5 * 0.1) == OCC for k in (1, 2, 3)), '축소해도 벽이 남아야 함'
assert g.level(3)[:2] == (1, 1)

# 전체 보기 — 지도가 화면에 들어옴 · 확대는 중심(화면 한가운데)이 그대로
vp = Viewport(g)
vp.fit(40, 20)
assert vp.fits(40, 20)
c0 = (vp.cx, vp.cy)
assert vp.zoom(+1) and (vp.cx, vp.cy) == c0
vp.k = -2
assert not vp.zoom(+1), '확대 한계'
vp.pan(2, 0)
assert abs(vp.cx - (c0[0] + 2 * vp.scale)) < 1e-9

# 그리기 — 글자 모드: 줄 수·폭이 맞고, goal 번호·십자선·로봇이 보임
view = MapView(g, color=False)
view.vp.fit(30, 20)
view.robot = (0.2, 0.2)
view.goals = [{'x': -0.3, 'y': -0.3, 'yaw_deg': 0}]
lines = view.render(30, 10)
assert len(lines) == 10 and all(len(l) == 30 for l in lines), (len(lines), [len(l) for l in lines])
txt = ''.join(lines)
assert '1' in txt and '+' in txt and '@' in txt and '#' in txt

# 로봇 따라가기 — 한 화면에 다 보이면 지도 고정, 확대해서 안 보이면 로봇이 중심
view.following = True
view.render(30, 10)
assert (view.vp.cx, view.vp.cy) == (0.0, 0.0), '지도가 다 보이면 지도 가운데'
view.vp.k = -2
view.robot = (0.3, -0.25)
view.render(30, 10)
assert (view.vp.cx, view.vp.cy) == (0.3, -0.25), '확대하면 로봇이 중심'
# 방향키로 밀면 따라가기가 풀리고 · r이면 다시 로봇 중심
assert view.on_key('left') and not view.following
cx = view.vp.cx
view.robot = (0.35, -0.25)
view.render(30, 10)
assert view.vp.cx == cx, '자유 이동 중엔 로봇을 안 따라감'
assert view.on_key('r') and view.following
view.render(30, 10)
assert (view.vp.cx, view.vp.cy) == (0.35, -0.25)
assert not view.on_key('x')   # 모르는 키는 안 먹음

# 방향선 — 찍는 모드(show_yaw)에서만 십자선에서 yaw 방향으로 선이 나감 (글자 모드는 ':')
view2 = MapView(g, color=False)
view2.vp.fit(30, 20)
view2.yaw = 0.0
assert ':' not in ''.join(view2.render(30, 10))
view2.show_yaw = True
assert ':' in ''.join(view2.render(30, 10))
rows0 = view2.render(30, 10)
view2.yaw = 90.0
assert view2.render(30, 10) != rows0, '방향이 바뀌면 선도 바뀜'

# 색 모드 — 방향선은 점자(⠀~⣿)로 가늘게 · 십자선은 분홍 바탕 흰 +
import re  # noqa: E402
cv = MapView(g, color=True)
cv.vp.fit(30, 20)
cv.show_yaw = True
cv.yaw = 0.0
cell_txt = re.sub(r'\x1b\[[0-9;]*m', '', ''.join(cv.render(30, 10)))
assert any(0x2800 < ord(ch) <= 0x28FF for ch in cell_txt), '방향선(점자)이 있어야 함'
assert '+' in cell_txt
print('PASS test_map_view')
