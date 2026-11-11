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


# M · O · 로봇의 방향 화살표 — 색 모드에서 점자(U+2800~28FF)로 그려지고, 방향이 바뀌면 다른 칸에 그려진다
def braille_cells(view, cols=40, rows=20):
    out = view.render(cols, rows)
    return {(c, r) for r, line in enumerate(out) for c, ch in enumerate(_strip(line)) if 0x2800 <= ord(ch) <= 0x28FF}


def _strip(line):
    import re
    return re.sub(r'\x1b\[[0-9;]*m', '', line)


big = MapGrid(40, 40, 0.1, -2.0, -2.0, [0] * 1600)
cv = MapView(big, color=True)
cv.vp.fit(40, 40)
cv.show_map = cv.show_odom = True
base = braille_cells(cv)                       # M 의 방향선(0°)만 있을 때
cv.odom, cv.odom_yaw = (0.8, 0.5), 0.0
east = braille_cells(cv)
cv.odom_yaw = 90.0
north = braille_cells(cv)
assert base and len(east) > len(base), 'O 방향선이 그려져야 함'
assert east != north, 'odom 방향이 바뀌면 화살표가 다른 칸에 그려져야 함'
cv.odom_yaw = None
assert braille_cells(cv) == base, 'odom 방향을 모르면 O 화살표는 안 그림'
cv.robot, cv.robot_yaw = (-0.6, -0.4), 180.0
assert len(braille_cells(cv)) > len(base), '로봇 진행 방향 화살표'
cv.show_map = cv.show_odom = False
off = braille_cells(cv)
assert off and len(off) < len(braille_cells(cv.__class__(big, color=True)) | off) + 1, '로봇 화살표는 남음'
cv.robot = None
assert not braille_cells(cv), 'M·O가 꺼지고 로봇도 없으면 화살표 없음 (M·O 방향선은 숨김)'
cv.show_map = True
assert braille_cells(cv), 'M 화살표는 켜면 다시 나옴'
cv2 = MapView(big, color=True)
assert cv2.show_map is False and cv2.show_odom is False, '처음에는 M·O 숨김'
# M과 O는 따로 — M만 켜면 O 글자는 안 나옴
cv2.vp.fit(40, 40)
cv2.odom, cv2.odom_yaw = (0.8, 0.5), 0.0
cv2.show_map = True
t_ = '\n'.join(cv2.render(40, 20))
assert 'M' in t_ and 'O' not in t_, 'M만 켰는데 O가 나옴'
cv2.show_map, cv2.show_odom = False, True
t_ = '\n'.join(cv2.render(40, 20))
assert 'O' in t_ and 'M' not in t_, 'O만 켰는데 M이 나옴'
print('PASS test_map_view')


# Nav2 경로 겹침(p) — 점선(:)이 늘고 끝점 E가 보임, 경로가 없으면 그대로
view = MapView(g, color=False)
view.vp.fit(30, 20)
base = ''.join(view.render(30, 10))
view.plans = [([(-0.3, -0.3), (0.0, 0.0), (0.3, 0.3)], (60, 230, 200))]
with_plan = ''.join(view.render(30, 10))
assert 'G' in with_plan and with_plan.count(':') > base.count(':'), '경로가 그려져야 함'
view.plans = []
assert ''.join(view.render(30, 10)) == base
print('map_view plan overlay ok')

# 십자선·방향 화살표가 지도 밖에 있어도 검은 네모(배경 48;2;0;0;0)를 칠하지 않는다 — 터미널 바탕 그대로
view = MapView(g, color=True)
view.vp.fit(30, 20)
view.vp.cy += 5.0        # 십자선을 지도 위쪽 밖으로
view.following = False
view.show_yaw = True
out = '\n'.join(view.render(30, 10))
assert '\033[48;2;0;0;0m' not in out, '지도 밖에 검은 배경이 칠해짐'
print('map_view off-map ok')

# 화살표·방향선·글자 밑 배경 — 위·아래 칸 중 한쪽이라도 벽이면 벽 색 (회색으로 지워지지 않음)
from op.views.map.map_view import RGB, _cell_bg  # noqa: E402
assert _cell_bg(OCC, FREE, RGB[OCC], RGB[FREE]) == RGB[OCC]
assert _cell_bg(FREE, OCC, RGB[FREE], RGB[OCC]) == RGB[OCC]
assert _cell_bg(FREE, FREE, RGB[FREE], RGB[FREE]) == RGB[FREE]
assert _cell_bg(OUT, OUT, None, None) is None, '지도 밖은 칠하지 않음'
print('map_view wall under arrow ok')

# 경유점 — 주황 번호로 찍힘 · 경로선 색은 밝은 회색 지도(205,205,210)와 구분되는 색 (흰색이면 안 보임)
from op.views.map.map_view import LOCAL_PLAN_RGB, PLAN_RGB, WAYPOINT_RGB  # noqa: E402
for rgb in (PLAN_RGB, LOCAL_PLAN_RGB, WAYPOINT_RGB):
    assert max(abs(a - b) for a, b in zip(rgb, (205, 205, 210))) > 80, rgb
view = MapView(g, color=True)
view.vp.fit(30, 20)
view.waypoints = [(0.2, 0.2)]   # (0,0)은 M 글자가 덮음
assert '\033[1;38;2;255;110;0m' in '\n'.join(view.render(30, 10)), '경유점이 주황으로 찍혀야 함'
print('map_view waypoint ok')

# 1/4 블록 지도(기본) — 빈틈 없는 블록 · 벽이 한 칸이라도 있으면 앞색은 벽 · costmap을 켜면 반블록으로 자동 전환
from op.views.map.map_view import QUAD, QUAD_BIT, _quad_cell  # noqa: E402
sub4 = [((dx, dy), FREE) for dy in (0, 1) for dx in (0, 1)]
assert _quad_cell(sub4)[0] == 0 and _quad_cell(sub4)[2] == RGB[FREE], '한 종류면 배경만'
w4 = [((dx, dy), OCC if (dx, dy) == (1, 0) else FREE) for dy in (0, 1) for dx in (0, 1)]
bits, fg, bg, wall = _quad_cell(w4)
assert QUAD[bits] == '▝' and fg == RGB[OCC] and bg == RGB[FREE] and wall, '벽 한 칸이 블록으로 남아야 함'
col = [((dx, dy), OCC if dx == 0 else FREE) for dy in (0, 1) for dx in (0, 1)]
assert QUAD[_quad_cell(col)[0]] == '▌', '세로 벽 = 왼쪽 반 블록 (이어진 선)'
edge = [((dx, dy), FREE if dx == 0 else OUT) for dy in (0, 1) for dx in (0, 1)]
bits, fg, bg, wall = _quad_cell(edge)
assert fg == RGB[FREE] and bg is None and not wall, '지도 가장자리는 바탕 그대로'
from ros.fake.RealWorld import RealWorld  # noqa: E402
_W, _H, _ox, _oy, _occ = RealWorld('isaac_maze').occupancy(0.05)   # 실세계 미로 — 벽이 이어진 선으로 나오는지
view = MapView(MapGrid(_W, _H, 0.05, _ox, _oy, [100 if _occ[r][c] else 0 for r in range(_H) for c in range(_W)]), color=True)
view.vp.fit(70, 60)
hi = '\n'.join(view.render(70, 30))
assert any(ch in QUAD[1:] for ch in hi), '블록으로 그려져야 함'
view.show_cost = True
lo = '\n'.join(view.render(70, 30))
assert not any(ch in '▘▝▖▗▌▐▚▞▛▜▙▟' for ch in lo), 'costmap 모드는 반블록(▀▄)만'
assert any(ch in '▘▝▖▗▌▐▚▞▛▜▙▟' for ch in hi), '기본은 1/4 블록'
print('map_view quad base ok')

# 경로 끝점 E — 흰 글자 + 경로 색 바탕 (경로선과 같은 색 글자라 안 보이던 것)
view = MapView(g, color=True)
view.vp.fit(30, 20)
view.plans = [([(-0.3, -0.3), (0.3, 0.3)], (20, 90, 255))]
out = '\n'.join(view.render(30, 10))
assert '\033[1;38;2;255;255;255m\033[48;2;20;90;255mG' in out, '끝점 G는 흰 글자 + 경로 색 바탕'
view.plans = [([(-0.3, -0.3), (0.3, 0.3)], (170, 0, 255))]
assert 'L' in ''.join(view.render(30, 10)).replace('\033', ''), '지역 경로 끝은 L'
print('map_view plan end marker ok')

# 글자가 겹치면 — 우선순위가 높은 쪽이 그 칸, 진 쪽은 옆 칸에 (M·O·로봇이 한 칸이어도 셋 다 보임)
view = MapView(g, color=False)
view.vp.fit(30, 20)
view.following = False
view.vp.cx, view.vp.cy = 0.35, 0.35                  # 십자선을 겹침에서 멀리
view.show_map = view.show_odom = True                  # M·O는 켜야 보임 (기본 숨김)
view.robot = view.odom = (0.0, 0.0)                    # 로봇 · O · M이 모두 세계 (0,0)
txt = '\n'.join(view.render(30, 10))
assert '@' in txt and 'O' in txt and 'M' in txt, '겹쳐도 로봇·O·M이 모두 보여야 함'
rows_ = [l for l in txt.split('\n') if '@' in l]
assert rows_ and all(ch in rows_[0] for ch in 'OM@') and max(rows_[0].index(ch) for ch in 'OM@') - min(rows_[0].index(ch) for ch in 'OM@') <= 4, rows_
print('map_view overlap ok')

# 경로 사이사이 점 o — 전역 경로를 1 m마다 (지도 8×8칸은 작아 0.8 m를 넘는 경로로)
g2 = MapGrid(80, 80, 0.1, -4.0, -4.0, [0] * 6400)
view = MapView(g2, color=False)
view.vp.fit(60, 30)
view.plans = [([(-3.0 + i * 0.1, 0.0) for i in range(61)], (20, 90, 255))]   # 6 m 직선
txt = '\n'.join(view.render(60, 15))
assert txt.count('o') >= 2, '경로 사이사이에 점이 찍혀야 함'
view.plans = [([(-3.0 + i * 0.1, 0.0) for i in range(61)], (170, 0, 255))]   # 지역 경로에는 안 찍음
assert '\n'.join(view.render(60, 15)).count('o') == 0
print('map_view path points ok')

# 지도 회전 키 — 왼쪽 키([ {)는 좌회전(+) · 오른쪽 키(] })는 우회전(−)
from op.pages.map import MapPage  # noqa: E402
pg = MapPage.__new__(MapPage)
pg.mode, pg.yaw, pg.msg, pg.view = 'pose', 0.0, '', None
for key, want in (('[', 15.0), (']', -15.0), ('{', 1.0), ('}', -1.0)):
    pg.yaw = 0.0
    assert pg._extra_key(key) and abs(pg.yaw - want) < 1e-9, (key, pg.yaw)
print('map rotate keys ok')

# m = M(map 원점) · o = O(odom 원점) — 각자 따로 토글
pg = MapPage.__new__(MapPage)
pg.mode, pg.msg = 'view', ''
pg.view = MapView(g, color=True)
assert pg._extra_key('m') and pg.view.show_map and not pg.view.show_odom, 'm은 M만'
assert pg._extra_key('o') and pg.view.show_map and pg.view.show_odom, 'o는 O만'
assert pg._extra_key('m') and not pg.view.show_map and pg.view.show_odom
assert pg._extra_key('o') and not pg.view.show_map and not pg.view.show_odom
print('map frame toggles ok')

# goal 웨이포인트 가운데 스냅 — 벽 사이 중앙으로, 벽 너머 다른 통로로는 안 넘어감
w21, h41 = 21, 41
data = [0] * (w21 * h41)
for r in range(h41):
    for c in (0, 20):
        data[r * w21 + c] = 100
g3 = MapGrid(w21, h41, 0.1, -1.05, -2.05, data)
pg = MapPage.__new__(MapPage)
pg.view = MapView(g3, color=False)
sx, sy, clear = pg._center_point(0.80, 0.0)   # 오른쪽 벽에 붙은 점
assert abs(sx) < 0.15 and abs(sy) < 0.15, (sx, sy)
assert clear is not None and clear > 0.9, clear
print('map goal snap center ok')

data = [0] * (w21 * h41)
for r in range(h41):
    for c in (0, 10, 20):
        data[r * w21 + c] = 100
g4 = MapGrid(w21, h41, 0.1, -1.05, -2.05, data)
pg.view = MapView(g4, color=False)
sx, sy, clear = pg._center_point(-0.20, 0.0)   # 왼쪽 통로의 벽에 붙은 점
assert abs(sx + 0.5) < 0.15 and abs(sy) < 0.15 and clear > 0.4, (sx, sy, clear)
print('map goal snap stays in corridor ok')

# d = 웨이포인트 전체 지우기 · s = 가운데 스냅 토글
class _Nav:
    def nav_progress(self):
        return None

    def cancel_nav(self):
        return True


class _Bus:
    nav = _Nav()


pg.mode, pg.msg, pg.snap, pg.goals, pg.bus = 'goal', '', True, [{'x': 0.0, 'y': 0.0}], _Bus()
assert pg._extra_key('d') and not pg.goals, 'd = 전체 지우기'
assert pg._extra_key('s') and not pg.snap, 's = 스냅 끄기'
assert pg._extra_key('s') and pg.snap
print('map goal keys ok')

# x = 주행 취소 — 경로(p)·경유점·웨이포인트 표시를 모두 지우고 '취소'를 남긴다
pg._plans = {'plan': (1.0, [(0.0, 0.0)]), 'local_plan': (1.0, [(0.0, 0.0)])}
pg._waypoints = (1, [(0.0, 0.0)])
pg.plan_on, pg._canceled, pg.msg, pg._show_goals = True, False, '', True
pg.goals = [{'x': 0.0, 'y': 0.0}]
assert pg._extra_key('x')
assert pg._plans == {'plan': None, 'local_plan': None}, pg._plans
assert pg._waypoints is None and not pg.plan_on and pg._canceled, (pg._waypoints, pg.plan_on, pg._canceled)
assert pg.goals and not pg._show_goals, (pg.goals, pg._show_goals)   # 목록은 남기고 지도 표시만 숨김
assert '취소' in pg.msg, pg.msg

# Backspace = 마지막 웨이포인트 취소 — 취소 뒤에도 목록이 남아 있어 바로 고칠 수 있다
pg.goals = [{'x': 0.0, 'y': 0.0}, {'x': 1.0, 'y': 0.0}]
pg._show_goals, pg.msg = False, ''
assert pg._extra_key('backspace')
assert len(pg.goals) == 1 and pg._show_goals and '웨이포인트' in pg.msg, (pg.goals, pg._show_goals, pg.msg)
print('map goal cancel/backspace ok')

# 연속 같은 자리 찍기는 추가하지 않는다 (through-poses 계획에 중복 점이 들어가지 않게)
pg = MapPage.__new__(MapPage)
pg.view = MapView(g3, color=False)
pg.mode, pg.snap, pg.yaw, pg.goals, pg.msg = 'goal', False, 0.0, [], ''
pg._canceled, pg._show_goals = True, False
pg.pick()
assert len(pg.goals) == 1 and pg._show_goals, (pg.goals, pg._show_goals)
pg.msg = ''
pg.pick()
assert len(pg.goals) == 1 and '같은 자리' in pg.msg, (pg.goals, pg.msg)
print('map goal duplicate skip ok')
