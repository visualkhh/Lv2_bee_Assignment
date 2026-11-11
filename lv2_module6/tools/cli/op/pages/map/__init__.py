"""MapPage — 터미널 지도 화면 (Menu 하위). 중앙 = MapView(포커스 가능), 왼쪽 = 동작 메뉴. 웹 페이지처럼 파라미터를 받는다.

  window.navigate('map', mode=…, source=…, …)  또는  MapPage(app, …)
    mode    'view'  지도 보기만 (기본)
            'goals' A/B 경로 goal 찍기 — start·goals(이미 있는 경로) · 저장하면 on_save(goals)
            'pose'  로봇 위치 지정 — 십자선 위치(+방향)를 on_pose(x, y, yaw_deg)로 보냄
    source  None = 지금 /map(live · 로봇 점 표시) · 경로 = 저장된 지도 yaml(map_server 형식)
    title   화면 제목 (기본 = mode에 따라)

열면 지도를 받아 중앙에 올린다 (Window.run의 center_comp) — 처음엔 지도가 포커스를 받아 방향키로 지도 이동 · +/- 확대·축소 ·
Enter = mode의 동작(goal 추가 · 위치 지정). Tab/Esc로 왼쪽 메뉴(같은 동작)와 오간다. Esc/b = 나가기.
goals 모드는 고친 게 있으면 저장 여부를 묻는다.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

from ...defs import BACK, MAIN, QUIT, STAY
from ...dialog import fmt_pose
from ...views.map import FREE, OCC, UNKNOWN, CostGrid, MapGrid, MapView
from ...views.map.map_view import LOCAL_PLAN_RGB, PLAN_RGB
from ...terminal import DIM
from ...menus import ActionItem, Menu

YAW_STEP = 15.0
CELL_TXT = {UNKNOWN: '미확인', FREE: '빈 곳', OCC: '벽'}
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
        self._cost: dict = {'global': None, 'local': None}
        self._cost_at: dict = {'global': 0.0, 'local': 0.0}
        self._cost_inbox: dict = {'global': [], 'local': []}
        self._cost_subs: list = []
        self.plan_on = False          # p — Nav2 전역(/plan)·지역(/local_plan) 경로 겹침
        self._plans: dict = {'plan': None, 'local_plan': None}   # 종류 → (받은 시각, [(x, y)…])
        self._waypoints: tuple[int, list[tuple[float, float]]] | None = None   # (plan_id, 경유점) — /task/sub_goals의 navigate·approach 목표

    # ---------------------------------------------------------------- 동작 (키 · 메뉴 공용)
    def pick(self) -> None:
        """십자선 위치에서 mode의 동작 — goals: goal 추가 · pose: 로봇 위치 지정 · view: 없음."""
        x, y = self.view.center
        if self.mode == 'view':
            self.msg = '보기 전용 — 찍는 화면은 A/B 경로 · 위치 지정에서'
        elif self.view.grid.code(self.view.vp.k, x, y) == OCC:
            self.msg = '벽 위에는 못 찍음 — 십자선을 옮기세요'
        elif self.mode == 'goals':
            self.goals.append({'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(self.yaw, 1)})
            self.msg = f'G{len(self.goals)} 추가 {fmt_pose(self.goals[-1])}'
        else:
            self._send_pose(x, y)

    def _send_pose(self, x: float, y: float) -> None:
        pose = {'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(self.yaw, 1)}
        goal = self.mode == 'goal'
        prompt = '여기를 이동 목표로 보낼까요?' if goal else '여기를 로봇 위치로 지정할까요?'
        cg = self._cost_grid('global')  # goal 검증은 전역 costmap 기준
        bad = bool(goal and cg and cg.code(x, y) == 2)   # 내접/치명 — planner가 거부하는 자리
        body = [fmt_pose(pose)]
        if bad:
            body.append('WARN: costmap 내접/치명 — planner가 거부할 가능성이 큽니다')
            body.append('회색(자유) 칸을 고르세요. 지금 보내면 backup(후진)으로 복구할 수 있습니다')
        if not self.ui.confirm(prompt, default=not bad, body=body):
            self.msg = '취소' + (' (costmap 내접)' if bad else '')
            return
        try:
            with self.ui.capture() as (out, err):
                self.on_pose(pose['x'], pose['y'], pose['yaw_deg'])
            self.msg = f'이동 목표 전송 {fmt_pose(pose)}' if goal else f'위치 지정 {fmt_pose(pose)}'
        except Exception as exc:
            self.msg = f'FAIL: {exc}'

    def undo_goal(self) -> None:
        self.msg = '마지막 goal 취소' if self.goals and self.goals.pop() else '취소할 goal 없음'

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
        elif k == 'backspace' and self.mode == 'goals':
            self.undo_goal()
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
        self.msg = 'M(map 원점) ' + ('표시' if self.view.show_map else '숨김')

    def toggle_odom(self) -> None:
        self.view.show_odom = not self.view.show_odom
        self.msg = 'O(odom 원점) ' + ('표시' if self.view.show_odom else '숨김')

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
        """겹칠 경로 [(점들, 색)] + 상태 글자 (끝점 G = 전역 · L = 지역). 15초 넘게 안 온 경로는 버림 (지난 주행 것이 남아 보이지 않게)."""
        if not self.plan_on:
            return [], ''
        now, out, info = time.monotonic(), [], []
        for kind, label, rgb in (('plan', '전역(파랑 G)', PLAN_RGB), ('local_plan', '지역(보라 L)', LOCAL_PLAN_RGB)):
            got = self._plans.get(kind)
            if got and now - got[0] <= self.PLAN_FRESH_S and len(got[1]) > 1:
                out.append((got[1], rgb))
                info.append(f'{label} {len(got[1])}점')
        wps = self._waypoint_pts()
        if wps:
            info.append(f'경유점(주황 번호) {len(wps)}개')
        return out, ('경로(p): ' + ' · '.join(info) if info else f'경로(p): 없음 — 주행 중이 아니거나 {self.PLAN_FRESH_S:.0f}초 넘게 안 옴')

    # ---------------------------------------------------------------- 화면
    def items(self):
        if self.saved:   # 저장했으면 보이는 항목이 없다 → Window.run이 끝남 (기존 규약: 항목이 없으면 화면 종료)
            return []
        v, act = self.view, ActionItem
        rows = {'goals': [('goal 추가 — 십자선 위치 (Enter)', self.pick), ('마지막 goal 취소 (Backspace)', self.undo_goal)],
                'pose': [('로봇 위치로 지정 — 십자선 위치 (Enter)', self.pick)],
                'goal': [('이동 목표로 — 십자선 위치 (Enter)', self.pick)], 'view': []}[self.mode]
        rows += [('확대 (+)', lambda: self.zoom(+1)), ('축소 (-)', lambda: self.zoom(-1)), ('전체 보기 (f)', self.fit),
                 (lambda: f'로봇 따라가기 (r) — {"켜짐" if v.following else "꺼짐"}', self.toggle_follow),
                 (lambda: 'costmap 겹쳐 보기 (c) — ' + self.COST_LABELS[self.cost_mode], self.toggle_cost),
                 (lambda: f'Nav2 경로 겹쳐 보기 (p) — {"켜짐" if self.plan_on else "꺼짐"}', self.toggle_plan),
                 (lambda: f'M map 원점 표시 (m) — {"켜짐" if v.show_map else "꺼짐"}', self.toggle_map_origin),
                 (lambda: f'O odom 원점 표시 (o) — {"켜짐" if v.show_odom else "꺼짐"}', self.toggle_odom)]
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
            self._cost[kind] = CostGrid.from_msg(inbox[-1])
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
                                            self.goals, self.yaw)
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
                  f'한 칸 = {v.vp.scale * 100:.0f} cm · {"따라가기" if v.following else "자유이동"}',
                  (f'방향 {self.yaw:+.0f}°' if self.mode == 'pose' else
                   f'방향 {self.yaw:+.0f}° · goal {len(self.goals)}개' if self.mode == 'goals' else '보기 전용')]
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
