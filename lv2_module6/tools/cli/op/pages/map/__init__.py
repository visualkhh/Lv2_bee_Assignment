"""MapPage — 터미널 지도 화면 (Menu 하위). 중앙 = MapView(포커스 가능), 왼쪽 = 동작 메뉴. 웹 페이지처럼 파라미터를 받는다.

  window.navigate('map', mode=…, source=…, …)  또는  MapPage(app, …)
    mode    'view'  지도 보기만 (기본)
            'goals' A/B 경로 goal 찍기 — start·goals(이미 있는 경로) · 저장하면 on_save(goals)
            'goal'  이동 목표 (Nav2) — 여러 웨이포인트를 찍어 g로 순서대로 보낸다 (구간 경로는 Nav2 planner)
            'pose'  로봇 위치 지정 — 십자선 위치(+방향)를 on_pose(x, y, yaw_deg)로 보냄
    source  None = 지금 /map(live · 로봇 점 표시) · 경로 = 저장된 지도 yaml(map_server 형식)
    title   화면 제목 (기본 = mode에 따라)

열면 지도를 받아 중앙에 올린다 (Window.run의 center_comp) — 처음엔 지도가 포커스를 받아 방향키로 지도 이동 · +/- 확대·축소 ·
Enter = mode의 동작(goal 추가 · 위치 지정). Tab/Esc로 왼쪽 메뉴(같은 동작)와 오간다. Esc/b = 나가기.
goals 모드는 고친 게 있으면 저장 여부를 묻는다.
"""
from __future__ import annotations

import math
import sys
import time
from collections import deque
from pathlib import Path

from ...defs import BACK, MAIN, QUIT, STAY
from ...dialog import fmt_pose
from ...views.map import FREE, OCC, UNKNOWN, CostGrid, MapGrid, MapView
from ...views.map.map_view import LOCAL_PLAN_RGB, PLAN_RGB
PREVIEW_RGB = (0, 200, 120)   # planner에 계산만 요청한 예상 경로 (초록)
from ...terminal import DIM
from ...menus import ActionItem, Menu

YAW_STEP = 15.0
CELL_TXT = {UNKNOWN: '미확인', FREE: '빈 곳', OCC: '벽'}
SNAP_SEARCH_M = 1.2      # 웨이포인트를 옮길 최대 거리 — 벽 사이 가운데까지
SNAP_MARGIN_M = 1.2      # 벽까지 거리 계산 여유 (검색 반경 밖 벽도 거리 계산에 포함)
SNAP_LAMBDA = 0.7        # 가운데 선호 vs 찍은 자리 유지 — 클수록 가운데로 더 간다
SNAP_MIN_GAIN_M = 0.05   # 이만큼도 안 넓어지면 찍은 자리 그대로


def _onoff(flag: bool) -> str:
    """메뉴에서 한눈에 보이게 — 켜짐 ● / 꺼짐 ○."""
    return '● 켜짐' if flag else '○ 꺼짐'
TITLES = {'view': '지도 보기', 'goals': 'A/B 경로 — 지도에서 찍기', 'pose': '로봇 위치 지정', 'goal': '이동 목표 (Nav2)'}


class MapPage(Menu):
    def __init__(self, app, mode: str = 'view', source=None, start: dict | None = None, goals: list[dict] | None = None,
                 on_save=None, on_pose=None, title: str | None = None):
        super().__init__(app)
        if mode not in TITLES:
            raise ValueError(f'mode는 {"/".join(TITLES)} 중 하나: {mode!r}')
        self.mode, self.source, self.on_save, self.on_pose = mode, (Path(source) if source else None), on_save, on_pose
        self.title = title or (TITLES[mode] + (f' · {self.source.parent.name}' if self.source else ''))
        self.start = start
        self.goals = [dict(g) for g in (goals or [])]
        self._initial = [dict(g) for g in self.goals]
        self.saved = False
        self.view: MapView | None = None
        self.yaw = 0.0
        self.msg = ''
        self._robot = None
        self._robot_yaw = None
        self._odom_yaw = None
        self._odom = None
        self._robot_at = 0.0
        self._fitted = False
        self.cost_mode = 0            # 0 OFF · 1 global_costmap · 2 local_costmap
        self.snap = True              # goal 웨이포인트를 벽 사이 가운데로 (s로 끄기)
        self._canceled = False        # x로 취소 — 상태줄에 '취소'를 남긴다
        self._show_goals = True       # 취소하면 지도 표시만 숨기고 목록은 남긴다 (Backspace로 수정)
        self._cost: dict = {'global': None, 'local': None}
        self._cost_at: dict = {'global': 0.0, 'local': 0.0}
        self._cost_inbox: dict = {'global': [], 'local': []}
        self._cost_subs: list = []
        self.plan_on = False          # p — Nav2 전역(/plan)·지역(/local_plan) 경로 겹침
        self._plans: dict = {'plan': None, 'local_plan': None}   # 종류 → (받은 시각, [(x, y)…])
        self._waypoints: tuple[int, list[tuple[float, float]]] | None = None   # (plan_id, 경유점) — /task/sub_goals의 navigate·approach 목표
        self._preview: list[tuple[float, float]] | None = None   # v — planner에 계산만 요청한 예상 경로

    # ---------------------------------------------------------------- 동작 (키 · 메뉴 공용)
    def pick(self) -> None:
        """십자선 위치에서 mode의 동작 — goals/goal: 웨이포인트 추가 · pose: 로봇 위치 지정 · view: 없음."""
        x, y = self.view.center
        if self.mode == 'view':
            self.msg = '보기 전용 — 찍는 화면은 A/B 경로 · 위치 지정에서'
        elif self.view.grid.code(self.view.vp.k, x, y) == OCC:
            self.msg = '벽 위에는 못 찍음 — 십자선을 옮기세요'
        elif self.mode == 'goals':
            self.goals.append({'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(self.yaw, 1)})
            self.msg = f'G{len(self.goals)} 추가 {fmt_pose(self.goals[-1])}'
        elif self.mode == 'goal':
            self._canceled = False   # 새로 찍기 시작 — 지난 취소 표시는 지운다
            self._show_goals = True
            note = ''
            if self.snap:
                sx, sy, clear = self._center_point(x, y)
                if round(sx, 3) != round(x, 3) or round(sy, 3) != round(y, 3):
                    x, y = sx, sy
                    note = f' → 가운데 (벽 {clear:.2f} m)' if clear is not None else ' → 가운데'
            goal = {'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(self.yaw, 1)}
            if self.goals:
                last = self.goals[-1]
                if math.hypot(goal['x'] - last['x'], goal['y'] - last['y']) < 0.08:
                    self.msg = '바로 전 웨이포인트와 같은 자리 — 추가 안 함'
                    return
            self.goals.append(goal)
            self._preview = None   # 웨이포인트가 바뀌었다 — 예상 경로는 다시 계산
            self.msg = f'웨이포인트 {len(self.goals)}개 추가 {fmt_pose(self.goals[-1])}{note} — 전송: g'
        else:
            self._send_pose(x, y)

    def _send_pose(self, x: float, y: float) -> None:
        """위치 지정(initialpose) — faker는 텔레포트, 실기는 /initialpose."""
        pose = {'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(self.yaw, 1)}
        if not self.ui.confirm('여기를 로봇 위치로 지정할까요?', body=[fmt_pose(pose)]):
            self.msg = '취소'
            return
        try:
            with self.ui.capture() as (out, err):
                self.on_pose(pose['x'], pose['y'], pose['yaw_deg'])
            self.msg = f'위치 지정 {fmt_pose(pose)}'
        except Exception as exc:
            self.msg = f'FAIL: {exc}'

    def set_initial_pose_here(self) -> None:
        """십자선 위치·방향을 로봇 초기 위치로 — /initialpose (faker는 텔레포트). 위치를 잃었을 때 여기서 바로."""
        x, y = self.view.center
        pose = {'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(self.yaw, 1)}
        if not self.ui.confirm('여기를 로봇 초기 위치로 지정할까요?',
                               body=[fmt_pose(pose), '위치추정(LOCALIZATION)에서 위치를 잃었을 때 쓰세요']):
            self.msg = '취소'
            return
        try:
            with self.ui.capture():
                self.bus.nav.set_robot_pose(pose['x'], pose['y'], pose['yaw_deg'])
            self.msg = f'초기 위치 {fmt_pose(pose)}'
        except Exception as exc:
            self.msg = f'FAIL: {exc}'

    def undo_goal(self) -> None:
        p = self.bus.nav.nav_progress() if hasattr(self.bus.nav, 'nav_progress') else None
        if p and p['state'] == 'running':
            self.msg = '주행 중에는 목록을 고칠 수 없음 — 먼저 취소(x)'
            return
        if self.goals:
            self.goals.pop()
            self._preview = None
            self._show_goals = True
            self.msg = ('마지막 웨이포인트 취소' if self.mode == 'goal' else '마지막 goal 취소')
        else:
            self.msg = '취소할 웨이포인트 없음' if self.mode == 'goal' else '취소할 goal 없음'

    def clear_goals(self) -> None:
        p = self.bus.nav.nav_progress()
        if p and p['state'] == 'running':
            self.msg = '주행 중에는 못 지움 — 먼저 취소(x)'
            return
        self.goals.clear()
        self._preview = None
        self._show_goals = True
        self.msg = '웨이포인트 전체 지움'

    def send_waypoints(self) -> None:
        """찍어 둔 웨이포인트를 순서대로 Nav2에 보낸다 — 구간 경로는 planner가 채운다."""
        if not self.goals:
            self.msg = '웨이포인트 없음 — Enter로 먼저 찍으세요'
            return
        cg = self._cost_grid('global')
        bad = [f'{i}) {fmt_pose(g)}' for i, g in enumerate(self.goals, 1)
               if cg is not None and cg.code(g['x'], g['y']) == 2]
        body = [f'{i}) {fmt_pose(g)}' for i, g in enumerate(self.goals, 1)]
        if bad:
            body.append('WARN: costmap 내접/치명 웨이포인트 — planner가 거부할 수 있습니다')
            body += bad
        if not self.ui.confirm(f'웨이포인트 {len(self.goals)}개를 순서대로 보낼까요?',
                               default=not bad, body=body):
            self.msg = '취소' + (' (costmap 내접)' if bad else '')
            return
        try:
            with self.ui.capture() as (out, err):
                self.bus.nav.send_nav_waypoints(self.goals, 'map')
            self._canceled = False
            self._show_goals = True
            self.msg = f'웨이포인트 {len(self.goals)}개 전송 — Nav2가 구간 경로를 계획'
        except Exception as exc:
            self.msg = f'FAIL: {exc}'

    def cancel_nav(self) -> None:
        """주행 취소 + 지도에 남은 경로·웨이포인트 표시를 모두 지운다 (x)."""
        active = self.bus.nav.cancel_nav()
        self._canceled = True
        self._plans = {'plan': None, 'local_plan': None}
        self._waypoints = None
        self.plan_on = False
        self.goals.clear()   # 목록도 비운다 — 남겨 두면 다음 Enter에서 이전 점들이 되살아나 g로 같이 전송됐다 (실기 2026-11-12)
        self._show_goals = True
        self.msg = ('Nav2 주행 취소 — 경로·웨이포인트 모두 지움' if active
                    else '주행 중인 웨이포인트 없음 — 경로·웨이포인트 모두 지움')

    def toggle_snap(self) -> None:
        self.snap = not self.snap
        self.msg = ('웨이포인트 가운데 스냅 ● 켜짐 — Enter로 찍을 때 벽 사이 가운데로'
                    if self.snap else '웨이포인트 가운데 스냅 ○ 꺼짐 — 찍은 자리 그대로')

    def _center_point(self, x: float, y: float) -> tuple[float, float, float | None]:
        """찍은 점 근처에서 '벽까지 가장 먼, 갈 수 있는' 칸으로 — 벽 사이 가운데.
        지도 가장자리·모르는 칸은 벽으로 본다. 벽 너머로는 BFS가 못 건너가게 한다.
        반환: (x, y, 벽까지 거리[m]) — 못 옮기면 찍은 점 그대로."""
        g = self.view.grid if self.view else None
        if g is None or g.res <= 0:
            return x, y, None
        w, h, raw = g.level(0)
        c0 = math.floor((x - g.ox) / g.res)
        r0 = math.floor((y - g.oy) / g.res)
        if not (0 <= c0 < w and 0 <= r0 < h):
            return x, y, None
        if raw[r0 * w + c0] != FREE:
            return x, y, None
        sr = max(1, int(round(SNAP_SEARCH_M / g.res)))       # 옮길 수 있는 반경
        mr = max(sr + 1, int(round(SNAP_MARGIN_M / g.res)))  # 거리 계산 여유
        win = sr + mr
        ca, ra = c0 - win, r0 - win
        ww = wh = 2 * win + 1
        INF = 1e9
        dist = [INF] * (ww * wh)

        def code_at(cc: int, rr: int) -> int:
            gc, gr = ca + cc, ra + rr
            return raw[gr * w + gc] if 0 <= gc < w and 0 <= gr < h else OCC

        for rr in range(wh):
            for cc in range(ww):
                if code_at(cc, rr) != FREE:   # 벽·모름·지도 밖 = 장애물
                    dist[rr * ww + cc] = 0.0
        s2 = math.sqrt(2.0)
        for rr in range(wh):   # 1패스 — 왼쪽/위에서
            base = rr * ww
            for cc in range(ww):
                i = base + cc
                d = dist[i]
                if d == 0.0:
                    continue
                if rr:
                    up = i - ww
                    d = min(d, dist[up] + 1.0)
                    if cc:
                        d = min(d, dist[up - 1] + s2)
                    if cc + 1 < ww:
                        d = min(d, dist[up + 1] + s2)
                if cc:
                    d = min(d, dist[i - 1] + 1.0)
                dist[i] = d
        for rr in range(wh - 1, -1, -1):   # 2패스 — 오른쪽/아래에서 (근사 유클리드)
            base = rr * ww
            for cc in range(ww - 1, -1, -1):
                i = base + cc
                d = dist[i]
                if d == 0.0:
                    continue
                if rr + 1 < wh:
                    dn = i + ww
                    d = min(d, dist[dn] + 1.0)
                    if cc:
                        d = min(d, dist[dn - 1] + s2)
                    if cc + 1 < ww:
                        d = min(d, dist[dn + 1] + s2)
                if cc + 1 < ww:
                    d = min(d, dist[i + 1] + 1.0)
                dist[i] = d

        # 찍은 칸에서 갈 수 있는 빈 칸만 (벽 너머·다른 통로로 순간이동 금지)
        start = (r0 - ra, c0 - ca)
        seen = {start}
        q = deque([start])
        while q:
            rr, cc = q.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nr, nc = rr + dr, cc + dc
                if not (0 <= nr < wh and 0 <= nc < ww) or (nr, nc) in seen:
                    continue
                if abs(nr - start[0]) > sr or abs(nc - start[1]) > sr:
                    continue
                if code_at(nc, nr) != FREE:
                    continue
                seen.add((nr, nc))
                q.append((nr, nc))

        best = None
        for rr, cc in seen:
            i = rr * ww + cc
            if dist[i] <= 0.0:
                continue
            wx = g.ox + (ca + cc + 0.5) * g.res
            wy = g.oy + (ra + rr + 0.5) * g.res
            moved = math.hypot(wx - x, wy - y)
            if moved > SNAP_SEARCH_M:
                continue
            clear = min(dist[i], mr) * g.res
            score = clear - SNAP_LAMBDA * moved
            if best is None or score > best[0]:
                best = (score, wx, wy, clear, moved)
        click_clear = min(dist[start[0] * ww + start[1]], mr) * g.res
        if best is None:
            return x, y, click_clear
        _, bx, by, bclear, bmoved = best
        if bmoved < 1e-9 or bclear - click_clear < SNAP_MIN_GAIN_M:
            return x, y, click_clear
        return bx, by, bclear

    def turn(self, delta: float) -> None:
        self.yaw = (self.yaw + delta + 180) % 360 - 180

    def toggle_follow(self) -> None:
        self.view.following = not self.view.following

    def zoom(self, d: int) -> None:
        if not self.view.vp.zoom(d):
            self.msg = '더 이상 ' + ('확대' if d > 0 else '축소') + '할 수 없음'

    def fit(self) -> None:
        self.view.on_key('f')

    def _save(self) -> None:
        self.saved = True

    def _extra_key(self, k) -> bool:
        """지도가 모르는 키."""
        if k in ('enter', ' '):
            self.pick()
        elif k == 'backspace' and self.mode in ('goals', 'goal'):
            self.undo_goal()
        elif k in ('g', 'G') and self.mode == 'goal':
            self.send_waypoints()
        elif k in ('d', 'D') and self.mode == 'goal':
            self.clear_goals()
        elif k in ('s', 'S') and self.mode == 'goal':
            self.toggle_snap()
        elif k in ('x', 'X') and self.mode == 'goal':
            self.cancel_nav()
        elif k in ('i', 'I') and self.mode == 'goal':
            self.set_initial_pose_here()
        elif k in ('v', 'V') and self.mode == 'goal':
            self.preview_path()
        elif k in ('[', ']', '{', '}') and self.mode != 'view':   # [ ] = 15° · { } = 1° (미세) — 왼쪽 키([ {)는 좌회전(반시계, +) · 오른쪽 키(] })는 우회전(시계, −)
            self.turn({'[': YAW_STEP, ']': -YAW_STEP, '{': 1.0, '}': -1.0}[k])
        elif k in ('c', 'C'):
            self.toggle_cost()
        elif k in ('p', 'P'):
            self.toggle_plan()
        elif k in ('o', 'O'):
            self.toggle_odom()
        elif k in ('m', 'M'):   # 지도에 포커스가 있을 때만 — 왼쪽 메뉴에서는 m = 메인 메뉴
            self.toggle_map_origin()
        else:
            return False
        return True

    COST_LABELS = {0: 'OFF', 1: 'GLOBAL (전역 계획)', 2: 'LOCAL (로봇 주변)'}

    def toggle_cost(self) -> None:
        self.cost_mode = (self.cost_mode + 1) % 3
        self.msg = 'costmap 겹침 ' + self.COST_LABELS[self.cost_mode]

    def toggle_map_origin(self) -> None:
        self.view.show_map = not self.view.show_map
        self.msg = 'M(map 원점) ' + ('● 표시' if self.view.show_map else '○ 숨김')
        if self.view.show_map and not self._on_screen((0.0, 0.0)):
            self.msg += ' — 화면 밖, f 전체 보기'

    def toggle_odom(self) -> None:
        self.view.show_odom = not self.view.show_odom
        self.msg = 'O(odom 원점) ' + ('● 표시' if self.view.show_odom else '○ 숨김')
        if not self.view.show_odom:
            return
        odom = getattr(self, '_odom', None)
        if odom is None:
            self.msg += ' — odom 데이터 대기'
        elif not self._on_screen(odom):
            self.msg += ' — 화면 밖, f 전체 보기'

    def _on_screen(self, p: tuple[float, float]) -> bool:
        """지도 중심 p가 지금 화면 안에 있나 — M·O 토글 때 '안 보이는 이유'를 알려 주려고."""
        cols, rows = self.view.size
        c, r = self.view.vp.to_screen(p[0], p[1], cols, rows * 2)
        return 0 <= c < cols and 0 <= r < rows * 2

    PLAN_FRESH_S = 15.0   # Nav2는 주행이 끝나도 빈 경로를 안 내서 — 이보다 오래된 경로는 안 그림

    def toggle_plan(self) -> None:
        if self.source:
            self.msg = '저장 지도에는 경로가 없음 — 지금 지도(live)에서만'
            return
        self.plan_on = not self.plan_on
        self.msg = '경로 겹침 ' + ('ON — 전역(파랑 G) · 지역(보라 L) · 주황 o = 경로 1 m 간격 · 주황 번호 = 작업 경유점' if self.plan_on else 'OFF')

    def _on_goals(self, m) -> None:
        """/task/sub_goals — navigate·approach 목표 좌표(\"x,y[,yaw]\")만 경유점으로 (물체 id 목표는 좌표를 모름)."""
        pts = []
        for g in m.goals:
            if g.skill in ('navigate', 'approach') and ',' in g.target:
                try:
                    x, y = (float(t) for t in g.target.split(',')[:2])
                except ValueError:
                    continue
                pts.append((x, y))
        self._waypoints = (int(m.plan_id), pts) if pts else None

    def _waypoint_pts(self):
        """그리는 경유점 — 그 계획이 지금 실행 중(/task/status RUNNING)일 때만 (끝난 계획의 점이 남아 보이지 않게)."""
        wp = self._waypoints
        if not (self.plan_on and wp):
            return []
        ts = self.state_task()
        return wp[1] if ts.startswith(f'plan={wp[0]} ') and 'RUNNING' in ts else []

    def state_task(self) -> str:
        return self.app.state.task_status or ''

    def _plan_lines(self):
        """겹칠 경로 [(점들, 색)] + 상태 글자 (끝점 G = 전역 · L = 지역 · 미리보기 = 초록). 15초 넘게 안 온 경로는 버림 (지난 주행 것이 남아 보이지 않게)."""
        if not self.plan_on and not self._preview:
            return [], ''
        now, out, info = time.monotonic(), [], []
        if self._preview and len(self._preview) > 1:
            out.append((self._preview, PREVIEW_RGB))
            info.append(f'미리보기(초록) {len(self._preview)}점')
        for kind, label, rgb in (('plan', '전역(파랑 G)', PLAN_RGB), ('local_plan', '지역(보라 L)', LOCAL_PLAN_RGB)):
            if not self.plan_on:
                break
            got = self._plans.get(kind)
            if got and now - got[0] <= self.PLAN_FRESH_S and len(got[1]) > 1:
                out.append((got[1], rgb))
                info.append(f'{label} {len(got[1])}점')
        wps = self._waypoint_pts() if self.plan_on else []
        if wps:
            info.append(f'경유점(주황 번호) {len(wps)}개')
        if not info:
            return [], f'경로(p): 없음 — 주행 중이 아니거나 {self.PLAN_FRESH_S:.0f}초 넘게 안 옴'
        return out, '경로: ' + ' · '.join(info)

    def preview_path(self) -> None:
        """찍어 둔 웨이포인트로 planner에 경로만 계산 (v) — 로봇은 움직이지 않는다. 전송은 g."""
        if not self.goals:
            self.msg = '웨이포인트 없음 — Enter로 먼저 찍으세요'
            return
        if self.source:
            self.msg = '저장 지도에는 planner가 없음 — 지금 지도(live)에서만'
            return
        try:
            with self.ui.capture():
                pts = self.bus.nav.preview_path(self.goals, 'map')
            if len(pts) < 2:
                self.msg = 'planner가 경로를 내지 않음 (출발/목표 확인)'
                return
            self._preview = pts
            self.msg = f'예상 경로 {len(pts)}점 — 초록 · 전송은 g'
        except Exception as exc:  # noqa: BLE001 — 화면에 이유를 남긴다
            self.msg = f'FAIL: {exc}'

    # ---------------------------------------------------------------- 화면
    def items(self):
        if self.saved:   # 저장했으면 보이는 항목이 없다 → Window.run이 끝남 (기존 규약: 항목이 없으면 화면 종료)
            return []
        v, act = self.view, ActionItem
        rows = {'goals': [('goal 추가 — 십자선 위치 (Enter)', self.pick), ('마지막 goal 취소 (Backspace)', self.undo_goal)],
                'pose': [('로봇 위치로 지정 — 십자선 위치 (Enter)', self.pick)],
                'goal': [('웨이포인트 추가 — 십자선 위치 (Enter)', self.pick),
                         ('마지막 웨이포인트 취소 (Backspace)', self.undo_goal),
                         ('웨이포인트 전송 — 순서대로 주행 (g)', self.send_waypoints),
                         ('웨이포인트 전체 지우기 (d)', self.clear_goals),
                         ('예상 경로 미리보기 — 계산만 (v)', self.preview_path),
                         ('로봇 초기 위치 지정 — 십자선 위치 (i)', self.set_initial_pose_here),
                         (lambda: f'가운데 스냅 (s) — {_onoff(self.snap)}', self.toggle_snap),
                         ('Nav2 주행 취소 (x)', self.cancel_nav)],
                'view': []}[self.mode]
        rows += [('확대 (+)', lambda: self.zoom(+1)), ('축소 (-)', lambda: self.zoom(-1)), ('전체 보기 (f)', self.fit),
                 (lambda: f'로봇 따라가기 (r) — {_onoff(v.following)}', self.toggle_follow),
                 (lambda: 'costmap 겹쳐 보기 (c) — ' + self.COST_LABELS[self.cost_mode], self.toggle_cost),
                 (lambda: f'Nav2 경로 겹쳐 보기 (p) — {_onoff(self.plan_on)}', self.toggle_plan),
                 (lambda: f'M map 원점 표시 (m) — {_onoff(v.show_map)}', self.toggle_map_origin),
                 (lambda: f'O odom 원점 표시 (o) — {_onoff(v.show_odom)}', self.toggle_odom)]
        if self.mode != 'view':
            rows += [(f'좌회전 +{YAW_STEP:.0f}° ([) · 미세 +1° = {{', lambda: self.turn(YAW_STEP)), (f'우회전 -{YAW_STEP:.0f}° (]) · 미세 -1° = }}', lambda: self.turn(-YAW_STEP))]
        if self.mode == 'goals':
            rows += [('저장하고 나가기', self._save)]
        return [act(self.app, str(i), label, fn) for i, (label, fn) in enumerate(rows, 1)]

    def _pose(self):
        """로봇 자세 — 0.3초에 한 번만 (막히지 않는 조회). 저장 지도도 map 프레임이면 로봇 점을 표시한다."""
        now = time.monotonic()
        if now - self._robot_at >= 0.3:
            self._robot_at = now
            p = self.bus.nav.robot_pose()
            self._robot = (p['x'], p['y']) if p else None
            self._robot_yaw = p['yaw_deg'] if p else None
            odom = self.bus.nav.frame_pose('odom')
            self._odom = odom[:2] if odom else None
            self._odom_yaw = odom[2] if odom else None
        return self._robot

    def _on_path(self, kind: str, m) -> None:
        """/plan·/local_plan 수신 — map 프레임만 (그 밖이면 좌표가 안 맞아 버림)."""
        if m.header.frame_id not in ('map', ''):
            return
        self._plans[kind] = (time.monotonic(), [(p.pose.position.x, p.pose.position.y) for p in m.poses])

    def _cost_grid(self, kind: str | None = None):
        """최근 costmap — 1초 캐시. kind 없으면 지금 모드(off/global/local). 저장 지도는 live 없음."""
        if self.source:
            return None
        if kind is None:
            kind = {0: None, 1: 'global', 2: 'local'}[self.cost_mode]
        if kind is None:
            return None
        now = time.monotonic()
        cached = self._cost.get(kind)
        if cached is not None and now - self._cost_at[kind] < 1.0:
            return cached
        inbox = self._cost_inbox[kind]
        if inbox:
            grid = CostGrid.from_msg(inbox[-1])
            if grid.frame_id not in ('', 'map'):
                # local costmap은 odom 프레임 — map 기준 자세로 변환해 겹쳐야 로봇·지도와 맞는다
                parent = self.bus.nav.frame_pose(grid.frame_id)
                if parent is not None:
                    grid.set_parent(parent[0], parent[1], math.radians(parent[2]))
            self._cost[kind] = grid
            self._cost_at[kind] = now
        return self._cost.get(kind)

    def center(self) -> list[str]:
        v = self.view
        cw, ch = self.ui.window.center_size()
        rows = max(3, ch - 7)
        v.show_yaw = self.mode != 'view'
        v.odom = self._odom
        v.odom_yaw, v.robot_yaw = self._odom_yaw, self._robot_yaw
        v.cost = self._cost_grid()
        v.show_cost = self.cost_mode != 0 and v.cost is not None
        v.plans, plan_info = self._plan_lines()
        v.waypoints = self._waypoint_pts()
        v.robot, v.start, v.goals, v.yaw = (self._pose(), ((self.start['x'], self.start['y']) if self.start else None),
                                            self.goals if self._show_goals else [], self.yaw)
        if not self._fitted:
            if self.mode in ('pose', 'goal') and self._robot:
                # initialpose/goal은 지도 한가운데가 아니라 로봇의 실제 map 자세에서 시작한다.
                v.vp.cx, v.vp.cy = self._robot
                v.vp.snap()
                self.yaw = self._robot_yaw if self._robot_yaw is not None else 0.0
                v.following = False
                self._fitted = True
            elif self.mode not in ('pose', 'goal'):
                v.vp.fit(cw, rows * 2)
                self._fitted = True
        lines = v.render(cw, rows)
        x, y = v.center
        code = v.grid.code(v.vp.k, x, y)
        status = [f'십자선 ({x:+.2f}, {y:+.2f}) m · {CELL_TXT.get(code, "지도 밖")}',
                  f'한 칸 = {v.vp.scale * 100:.0f} cm · {"● 따라가기" if v.following else "○ 자유이동"}',
                  (f'방향 {self.yaw:+.0f}°' if self.mode == 'pose' else
                   f'방향 {self.yaw:+.0f}° · goal {len(self.goals)}개' if self.mode == 'goals' else
                   f'방향 {self.yaw:+.0f}° · 웨이포인트 {len(self.goals)}개 (Enter 추가 · g 전송)' if self.mode == 'goal'
                   else '보기 전용')]
        if plan_info:
            status.append(plan_info)
        cg = self._cost_grid('global')   # goal/시작 판정은 항상 전역 costmap
        if cg is not None:
            cc = cg.code(x, y)
            tag = {0: '자유', 1: '팽창', 2: '내접/치명'}.get(cc, '?')
            status.append(f'cost(global) 십자선: {tag}' + (' — goal 불가' if cc == 2 and self.mode == 'goal' else ''))
            if self._robot:
                rc = cg.code(*self._robot)
                if rc == 2:
                    status.append('cost(global) 로봇: 내접/치명 — 시작 자세를 벽에서 떨어뜨리세요')
        if self.cost_mode:
            status.append('겹침: ' + self.COST_LABELS[self.cost_mode]
                          + (' ON' if v.show_cost else ' 데이터 대기')
                          + ' · 주황=팽창 · 빨강=내접/치명')
        if self.mode == 'goal':
            if self._canceled:
                status.append('Nav2 주행 취소 — 웨이포인트·경로 표시 지움')
            else:
                p = self.bus.nav.nav_progress()
                if p is not None:
                    head = f'Nav2 웨이포인트 {p["index"]}/{p["total"]}'
                    if p['state'] == 'running':
                        if p.get('recoveries') is not None:
                            head += (f' · 회복 {p["recoveries"]}회'
                                     f' · 남은 {p["distance_remaining"]:.2f} m ({p["age"]:.1f}s)')
                        elif p.get('distance_remaining') is not None:
                            head += f' · 남은 {p["distance_remaining"]:.2f} m'
                        else:
                            head += ' · feedback 대기'
                    elif p['state'] == 'done':
                        head = f'Nav2 웨이포인트 완료 {p["total"]}/{p["total"]}'
                    elif p['state'] == 'failed':
                        head += f' 실패: {p.get("error") or "이유 없음"}'
                    elif p['state'] == 'canceled':
                        head += ' 취소'
                    status.append(head)
        legend = self.term.paint('● 로봇(base_footprint) 짧은 화살표 = 앞(+x) 방향  + 십자선  S 출발  1 2 goal'
                                 + ('  M map원점' if v.show_map else '') + ('  O odom원점' if v.show_odom else '')
                                 + ('' if v.show_map and v.show_odom else '  (' + ('' if v.show_map else 'm = M') + ('' if v.show_map or v.show_odom else ' · ') + ('' if v.show_odom else 'o = O') + ' 표시)'), DIM)
        frames = (('프레임' + (': map 원점 M(0,0) 방향 +0°' if v.show_map else '')
                   + ((f' · odom O({self._odom[0]:+.2f},{self._odom[1]:+.2f}) 방향 {self._odom_yaw:+.0f}°' if self._odom else ' · odom TF 없음')
                      if v.show_odom else ''))
                  + (f' · base_footprint ●({self._robot[0]:+.2f},{self._robot[1]:+.2f}) 방향 {self._robot_yaw:+.0f}°' if self._robot and self._robot_yaw is not None
                     else f' · base_footprint ●({self._robot[0]:+.2f},{self._robot[1]:+.2f})' if self._robot else ' · base_footprint TF 없음'))
        return lines + status + [frames, self.msg, legend]

    # ---------------------------------------------------------------- 실행
    def _load_grid(self, timeout: float = 10.0):
        if self.source:
            return MapGrid.from_yaml(self.source)
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        inbox: list = []
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        sub = self.app.create_subscription(OccupancyGrid, '/map', inbox.append, qos)
        try:
            end = time.monotonic() + timeout
            while not inbox and time.monotonic() < end:
                self.ui.progress(f'/map 기다리는 중 … {max(0, end - time.monotonic()):.0f}초')
                self.app.poll(0.2)
            return MapGrid.from_msg(inbox[-1]) if inbox else None
        finally:
            self.app.destroy_subscription(sub)
            self.ui._busy = None

    @staticmethod
    def _warm(grid: MapGrid) -> None:
        """축소 단계(벽 우선 피라미드)를 미리 다 만들어 둔다 — 열린 뒤 확대·축소가 끊기지 않게."""
        k = 0
        while grid.level(k)[:2] != (1, 1):
            k += 1

    def show(self) -> str:
        self.ui.busy('지도 여는 중 …')   # 지도 받기·읽기가 몇 초 걸릴 수 있다 — 화면이 뜨기 전까지 busy 창
        try:
            grid = self._load_grid()
            if grid is not None:
                self._warm(grid)
        except RuntimeError as exc:
            print(f'FAIL: {exc}', file=sys.stderr)
            return STAY
        finally:
            self.ui._busy = None   # 지도 화면이 그려지면 가려짐 · 뒤에 여는 확인창이 busy를 다시 그리지 않게
        if grid is None:
            print('FAIL: /map을 못 받음 — 지도 서버(map_server)·Nav가 떠 있는지 확인', file=sys.stderr)
            return STAY
        self.view = MapView(grid, self.term.color, extra_key=self._extra_key)
        if self.mode == 'goal':
            self.view.hints += ' · g 전송 · v 예상 경로 · d 전체 지우기 · i 초기 위치 · s 중앙 스냅 · x 취소 · Backspace 마지막 취소'
        if self.source is None:  # live 지도일 때만 costmap 겹침 — 저장 지도에는 현재 costmap이 없다
            from nav_msgs.msg import OccupancyGrid
            from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
            qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            for kind, topic in (('global', '/global_costmap/costmap'), ('local', '/local_costmap/costmap')):
                self._cost_inbox[kind] = []
                self._cost_subs.append(self.app.create_subscription(
                    OccupancyGrid, topic, self._cost_inbox[kind].append, qos))
            try:   # Nav2 경로 — 메시지가 없는 환경(fake)이면 p만 빈 화면
                from nav_msgs.msg import Path as PathMsg
                for kind, topic in (('plan', '/plan'), ('local_plan', '/local_plan')):
                    self._cost_subs.append(self.app.create_subscription(
                        PathMsg, topic, lambda m, k=kind: self._on_path(k, m),
                        QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)))
            except Exception:
                pass
            try:   # 경유점 — plan 메시지가 없는 환경이면 건너뜀
                from plan.msg import SubGoalArray
                self._cost_subs.append(self.app.create_subscription(
                    SubGoalArray, '/task/sub_goals', self._on_goals, qos))
            except Exception:
                pass
        try:
            while True:
                result = self.ui.window.run(self.title, self.items, self.center, back=True, main=True,
                                            center_comp=self.view, focus_center=True)
                if result in (MAIN, QUIT):
                    return result
                if self.saved:
                    if self.on_save:
                        self.on_save(self.goals)
                    return BACK
                if self.mode != 'goals' or self.goals == self._initial or self.ui.confirm(
                        '저장하지 않고 나갈까요?', default=False,
                        body=[f'고친 goal {len(self.goals)}개가 사라집니다.', '저장하려면 아니오 → 왼쪽 메뉴의 저장하고 나가기']):
                    return BACK
        finally:
            for sub in self._cost_subs:
                self.app.destroy_subscription(sub)
            self._cost_subs.clear()
