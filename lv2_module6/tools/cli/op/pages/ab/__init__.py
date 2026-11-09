"""A/B 테스트 — Nav2 주행 정책 ON/OFF를 같은 주행으로 비교.

메뉴 13 → 노드 목록 → 그 노드의 A/B 화면. 지금은 rl_nav_controller 하나이고,
새 노드가 생기면 ab_node_pages()에 한 줄 추가한다.

기록은 백그라운드로 돌고, 메뉴에서 '녹화 종료'를 누르면 rosbag2가 정상 마무리된다.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

from ...defs import STAY
from ...dialog import RVIZ_GOAL_TOPIC, RouteRvizDialog, fmt_pose
from ...terminal import GREEN, RED
from ...menus import GoMenu, Menu, MenuItem
from .core import AB_NODES


class AbStatusItem(MenuItem):
    key = '1'

    def label(self):
        return '상태 (보정 ON/OFF · 모델 · 녹화)'

    def run(self):
        print('\n'.join(self.bus.ab.status_lines()))
        return STAY


class AbToggleItem(MenuItem):
    key = '2'

    def label(self):
        st = self.bus.ab.status()
        if st['enabled'] is None:
            return '정책 ON/OFF — controller_server 응답 없음'
        return f'정책 {"끄기" if st["enabled"] else "켜기"} (현재 {"ON" if st["enabled"] else "OFF"})'

    def run(self):
        st = self.bus.ab.status()
        if st['enabled'] is None:
            print('FAIL: controller_server 응답 없음 — bringup 확인', file=sys.stderr)
            return STAY
        on = not st['enabled']
        if not self.ui.confirm(
                f'Nav2 주행 정책을 {"ON" if on else "OFF"}로 바꿀까요?', default=True,
                body=['OFF = inner DWB (비교 기준)', 'A/B는 주행과 주행 사이에 바꾸세요.']):
            return STAY
        ok, msg = self.bus.ab.toggle(on)
        print(('ok: ' if ok else 'FAIL: ') + msg)
        return STAY


class AbRecordItem(MenuItem):
    key = '3'

    def label(self):
        return '녹화 시작 (rosbag2 · 백그라운드)'

    def run(self):
        if self.bus.ab.current():
            print('FAIL: 이미 녹화 중 — 4) 녹화 종료를 먼저 누르세요', file=sys.stderr)
            return STAY
        label = self.ui.modal_input(
            'A/B 녹화', ['라벨 (예: A, B, on1, off1)', '저장: tools/artifacts/ab/<시각>_<라벨>'], '라벨>', default='A')
        if label is None:
            return STAY
        try:
            rec = self.bus.ab.start(label)
        except RuntimeError as exc:
            print(f'FAIL: {exc}', file=sys.stderr)
            return STAY
        print(f'녹화 시작: {rec["bag"]}')
        print('주행을 마친 뒤 4) 녹화 종료를 누르세요.')
        return STAY


class AbStopItem(MenuItem):
    key = '4'

    def label(self):
        return '녹화 종료 (rosbag2 마무리)'

    def run(self):
        rec = self.bus.ab.stop()
        if rec is None:
            print('녹화 중 아님')
            return STAY
        dur = max(0.0, float(rec.get('stopped', time.time())) - float(rec.get('started', time.time())))
        print(f'녹화 종료: {rec["bag"]} ({dur:.1f}s)')
        return STAY


class AbReportItem(MenuItem):
    key = '5'

    def label(self):
        return f'최근 A/B 리포트 (녹화 {len(self.bus.ab.history())}개)'

    def run(self):
        hist = self.bus.ab.history()
        if len(hist) < 2:
            print('FAIL: 녹화가 2개 미만 — A/B를 먼저 기록하세요', file=sys.stderr)
            return STAY
        pair = hist[-2:]
        body = [f'{h.get("label", "?")}: {h["bag"]}' for h in pair]
        if not self.ui.confirm('최근 두 녹화로 리포트를 만들까요?', default=True, body=body):
            return STAY
        out = str(Path(pair[0]['bag']).parent / f'report_{time.strftime("%Y%m%d_%H%M%S")}.png')
        labels = [h.get('label', f'RUN{i + 1}') for i, h in enumerate(pair)]
        rc = self.bus.ab.report([h['bag'] for h in pair], labels, out)
        if rc == 0:
            print(f'저장: {out}')
        return STAY


class AbGuideItem(MenuItem):
    key = '2'

    def label(self):
        return '② 자동 A/B 실행 (A 정책 → B DWB · 녹화·리포트 자동)'

    def run(self):
        label = self.ui.modal_input(
            '자동 A/B',
            [self.bus.ab.plan_summary(),
             'A(정책 ON) → 시작점 복귀 → B(정책 OFF=DWB)를 같은 경로로 주행 · 녹화 · 리포트까지 알아서 합니다.',
             '주행 중: s = 지금 leg 중지하고 다음으로 · q = 전체 중단',
             'Nav2 /navigate_to_pose · AUTO · E-STOP OFF 필요 — 라벨을 입력하세요'],
            '라벨>', default=time.strftime('%Y%m%d_%H%M%S'))
        if label is None:
            return STAY
        rc = self.bus.ab.guide(label)
        if rc != 0:
            print(f'FAIL: 자동 A/B 실패 (rc={rc})', file=sys.stderr)
        return STAY


# ------------------------------------------------------------------ 경로 편집 (route.json)
class _RouteItem(MenuItem):
    """경로 편집 공통 — route.json(있으면) 또는 빈 경로를 고친다. command.json(기본)은 건드리지 않는다."""

    def _route(self) -> tuple[dict | None, list[dict]]:
        from .core import ROUTE
        if not ROUTE.is_file():
            return None, []
        plan = self.bus.ab.load_plan()
        return plan['start'], list(plan['route'])

    def _save(self, start, route) -> None:
        path = self.bus.ab.save_route(start, route)
        print(f'저장: {path} — 출발 {fmt_pose(start)} · goal {len(route)}개')

    def _here(self) -> dict | None:
        try:
            return self.bus.ab.current_pose()
        except RuntimeError as exc:
            print(f'FAIL: {exc}', file=sys.stderr)
            return None


class RouteMapItem(_RouteItem):
    key = '1'
    shows_busy = False   # 지도 화면(MapPage)이 자기 화면을 그린다 — busy 창은 run 안에서

    def label(self):
        return '지도에서 찍기 (터미널 · 확대/축소 · 로봇 중심)'

    def run(self):
        self.ui.busy('지도 화면 여는 중 … (로봇 위치 확인)')   # 위치 조회가 최대 5초 걸린다
        here = None
        try:
            here = self.bus.ab.current_pose()   # 출발점 (TF 리스너도 이때 켜짐 — 최대 5초)
        except RuntimeError:
            pass  # 위치를 모르면 실행할 때의 자세를 출발점으로
        old_start, route = self._route()
        start = old_start if route else here   # 이미 있는 경로는 출발점을 그대로
        self.ui.window.navigate('map', mode='goals', start=start, goals=route,
                                on_save=lambda goals: goals and self._save(start, goals))
        return STAY


class RouteRvizItem(_RouteItem):
    key = '2'

    def label(self):
        return f'RViz로 지정 (2D Goal Pose → {RVIZ_GOAL_TOPIC})'

    def run(self):
        start = None
        try:
            start = self.bus.ab.current_pose()
        except RuntimeError:
            pass  # TF 없으면 실행할 때의 자세를 출발점으로
        goals = RouteRvizDialog(self.ui, self.bus, start).run()
        if goals:
            self._save(start, goals)
        return STAY


class RouteStartHereItem(_RouteItem):
    key = '3'

    def label(self):
        return '지금 위치를 출발점으로'

    def run(self):
        here = self._here()
        if here is not None:
            _start, route = self._route()
            if not route:
                print('goal이 아직 없음 — 출발점만 저장할 수 없어 goal을 먼저 추가하세요 (4·5)', file=sys.stderr)
                return STAY
            self._save(here, route)
        return STAY


class RouteAddHereItem(_RouteItem):
    key = '4'

    def label(self):
        return '텔레오퍼로 이동하여 goal 추가 (g/Enter = 지금 위치)'

    def run(self):
        import argparse
        start, route = self._route()
        if not route:  # 새 경로 — 텔레오퍼를 시작하는 지금 자리가 출발점
            start = self._here()
            if start is None:
                return STAY
        added = []

        def mark() -> str:
            try:
                here = self.bus.ab.current_pose()
            except RuntimeError as exc:
                return self.term.paint(f'위치를 모름 — {exc}', RED)
            n = len(route) + 1
            if not self.ui.confirm(f'지금 위치를 G{n}로 추가할까요?', default=True,
                                   body=[f'G{n}: {fmt_pose(here)}', f'출발점: {fmt_pose(start)}']):
                return '추가 안 함'
            route.append(here)
            added.append(here)
            self.bus.ab.save_route(start, route)
            return self.term.paint(f'G{n} 추가 {fmt_pose(here)} — 계속 몰고 가서 g · 끝나면 Esc', GREEN)

        self.bus.cmd_teleop(argparse.Namespace(linear=self.bus.linear, angular=self.bus.angular, force=False),
                            on_mark=mark)
        if not added:
            print('추가한 goal 없음')
            return STAY
        print(f'goal {len(added)}개 추가 — 경로: 출발 {fmt_pose(start)} · goal {len(route)}개')
        print('로봇은 그대로 두면 됨 — ② 자동 A/B 실행이 시작할 때 출발점으로 데려간다 (Nav2·DWB)')
        return STAY


class RouteAddTypedItem(_RouteItem):
    key = '5'

    def label(self):
        return '좌표 입력으로 goal 추가 (x y yaw°)'

    def run(self):
        def parse(text):
            v = text.replace(',', ' ').split()
            if len(v) not in (2, 3):
                return 'x y [yaw°] — 예: 1.9 0.85 0'
            try:
                [float(t) for t in v]
            except ValueError:
                return '숫자만'
            return None
        raw = self.ui.modal_input('goal 추가', ['map 좌표 [m]와 방향 [°] — 예: 1.9 0.85 0'], 'x y yaw>', validate=parse)
        if raw is None:
            return STAY
        v = [float(t) for t in raw.replace(',', ' ').split()]
        start, route = self._route()
        self._save(start, route + [{'x': v[0], 'y': v[1], 'yaw_deg': v[2] if len(v) > 2 else 0.0}])
        return STAY


class RouteUndoItem(_RouteItem):
    key = '6'

    def label(self):
        return '마지막 goal 삭제'

    def run(self):
        start, route = self._route()
        if not route:
            print('route.json에 goal이 없음')
            return STAY
        if len(route) == 1:
            self.bus.ab.clear_route()
            print('마지막 goal을 지워 route.json 삭제 — command.json 기본 경로를 씀')
            return STAY
        self._save(start, route[:-1])
        return STAY


class RouteResetItem(_RouteItem):
    key = '7'

    def label(self):
        return '경로 지우기 → command.json 기본'

    def run(self):
        if not self.ui.confirm('지정한 경로(route.json)를 지울까요?', default=False,
                               body=['지우면 command.json의 기본 경로로 A/B를 돌립니다.']):
            return STAY
        print('route.json 삭제' if self.bus.ab.clear_route() else 'route.json 없음 — 이미 기본 경로')
        return STAY


class RouteCheckItem(_RouteItem):
    key = '8'

    def label(self):
        return '경로 점검 (Nav2 planner로 구간마다 경로 계획)'

    def run(self):
        try:
            plan = self.bus.ab.load_plan()
            start = plan['start'] or self.bus.ab.current_pose(plan['frame_id'])
            self.bus.ab._preflight(plan, start)
            print(f'OK: {self.bus.ab.plan_summary()}')
        except RuntimeError as exc:
            print(f'FAIL: {exc}', file=sys.stderr)
        return STAY


class AbRouteMenu(Menu):
    title = 'A/B 경로 편집 · rl_nav_controller'

    def items(self):
        return [RouteMapItem(self.app), RouteRvizItem(self.app), RouteStartHereItem(self.app), RouteAddHereItem(self.app),
                RouteAddTypedItem(self.app), RouteUndoItem(self.app), RouteResetItem(self.app),
                RouteCheckItem(self.app)]

    def center(self):
        from .core import ROUTE
        lines = ['[A/B 경로]', ' ' + self.bus.ab.plan_summary()]
        try:
            plan = self.bus.ab.load_plan()
            lines.append(f' 출발점: {fmt_pose(plan["start"])}')
            lines += [f'  G{i}: {fmt_pose(g)}' for i, g in enumerate(plan['route'], 1)]
        except RuntimeError as exc:
            lines.append(f' 오류: {exc}')
        lines += ['', f' 저장: {ROUTE}' if ROUTE.is_file() else ' 지정한 경로 없음 — command.json 기본 경로 사용 중',
                  '', ' 여기서는 경로만 정한다 — 주행·녹화는 안 함 (② 자동 A/B 실행이 함)',
                  ' 1) 터미널 지도에서 찍기 (+/- 확대·축소) · 실기: 4) 텔레오퍼 g · 또는 RViz(RVIZ_CONFIG=ab) 2)',
                  ' A·B는 같은 경로를 그대로 주행 (공정한 비교)']
        return lines


class AbResultItem(MenuItem):
    key = '3'

    def label(self):
        return '③ 최근 결과 보기 (판정 · compare.png)'

    def run(self):
        import json
        from .core import BAG_DIR
        groups = sorted((d for d in BAG_DIR.glob('2*') if d.is_dir() and any(d.glob('*/result.json'))),
                        key=lambda d: d.name)
        if not groups:
            print('아직 자동 A/B 결과가 없음 — ② 자동 A/B 실행')
            return STAY
        g = groups[-1]
        print(f'최근 A/B: {g.name}')
        for leg in sorted(g.glob('*/result.json')):
            r = json.loads(leg.read_text())
            goals = f' · 목표 {r["goals_reached"]}/{r["goals_total"]}' if 'goals_total' in r else ''
            print(f'  {r["leg"]} ({"정책" if r["leg"] == "A" else "DWB"}): {r["verdict"]}{goals} · {r["duration_s"]:.1f}s'
                  + (f'\n      {r["error"]}' if r.get('error') else ''))
        cmp_png = g / 'compare.png'
        print(f'\n비교 리포트: {cmp_png if cmp_png.is_file() else "(없음)"}')
        print('  호스트: lv2_module6/debug/tools/artifacts/ab/rl_nav_controller/' + g.name + '/compare.png')
        return STAY


class AbManualMenu(Menu):
    """직접 주행하며 기록할 때만 — 자동 A/B는 이걸 전부 알아서 한다."""
    title = 'A/B 수동 (고급) · rl_nav_controller'

    def items(self):
        return [AbStatusItem(self.app), AbToggleItem(self.app), AbRecordItem(self.app),
                AbStopItem(self.app), AbReportItem(self.app)]

    def center(self):
        return ['[수동 A/B — 보통은 필요 없음]',
                ' 자동 A/B(②)가 정책 ON/OFF · 녹화 시작/종료 · 리포트를 알아서 한다.',
                ' 여기는 사람이 직접 몰면서(텔레오퍼·RViz goal) 기록할 때:',
                '  2) 정책 ON → 3) 녹화 시작 → 주행 → 4) 녹화 종료',
                '  2) 정책 OFF → 3) 녹화 시작 → 같은 주행 → 4) 녹화 종료 → 5) 리포트'] + \
            [''] + self.bus.ab.status_lines()


class RlNavControllerAbMenu(Menu):
    title = 'A/B 테스트 · rl_nav_controller'
    REFRESH_S = 1.0

    def __init__(self, app):
        super().__init__(app)
        self._lines: list[str] = []
        self._at = 0.0

    def items(self):
        return [GoMenu(self.app, '1', '① 경로 정하기 (지도 · RViz · 텔레오퍼 · 좌표)', AbRouteMenu(self.app)),
                AbGuideItem(self.app), AbResultItem(self.app),
                GoMenu(self.app, '4', '수동 (고급) — 상태 · 정책 ON/OFF · 녹화 · 리포트', AbManualMenu(self.app))]

    def center(self):
        now = time.monotonic()
        if not self._lines or now - self._at >= self.REFRESH_S:
            self._lines = self.bus.ab.status_lines()
            self._at = now
        return (['[A/B 하는 법]',
                 ' ① 경로 정하기 — RViz에서 goal 찍기 또는 텔레오퍼로 몰고 가서 \'지금 위치 추가\' (한 번)',
                 ' ② 자동 A/B 실행 — 사전 점검 → A(정책) 녹화·주행 → 복귀 → B(DWB) 녹화·주행 → 리포트',
                 '     주행 중 s = 이 leg 중지 · q = 전체 중단',
                 ' ③ 최근 결과 보기 — 판정 · compare.png', ''] + self._lines)


def ab_node_pages(app) -> list[tuple[str, str, Menu]]:
    """AB_NODES의 각 노드 → 그 노드의 A/B 화면. 새 노드는 화면 매핑에 추가한다."""
    pages = {'rl_nav_controller': RlNavControllerAbMenu}
    return [(name, desc, pages[name](app)) for name, desc in AB_NODES]


class AbMenu(Menu):
    """A/B 테스트 최상위 — 노드 이름으로 하위 메뉴를 고른다."""
    title = 'A/B 테스트 (노드별)'

    def items(self):
        return [GoMenu(self.app, str(i + 1), f'{name} — {desc}', page)
                for i, (name, desc, page) in enumerate(ab_node_pages(self.app))]

    def center(self):
        nodes = ab_node_pages(self.app)
        lines = ['[A/B 대상 노드]', f' {len(nodes)}개 — 노드를 고르면 ON/OFF·기록·리포트 화면']
        lines.append('')
        lines += [f' {name}: {desc}' for name, desc, _ in nodes]
        lines += ['', ' 새 rl_nav_* 노드는 ab_node_pages()에 추가']
        return lines
