"""A/B 테스트 — Nav2 주행 정책 ON/OFF를 같은 주행으로 비교.

메뉴 14 → 노드 목록 → 그 노드의 A/B 화면. 지금은 rl_nav_controller 하나이고,
새 노드가 생기면 ab_node_pages()에 한 줄 추가한다.

기록은 백그라운드로 돌고, 메뉴에서 '녹화 종료'를 누르면 rosbag2가 정상 마무리된다.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

from ...defs import STAY
from ..base import GoMenu, Menu, MenuItem
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
            'A/B 녹화', ['라벨 (예: A, B, on1, off1)', '저장: tool/artifacts/ab/<시각>_<라벨>'], '라벨>', default='A')
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
    key = '6'

    def label(self):
        return '자동 A/B (command.json → Nav2 주행)'

    def run(self):
        label = self.ui.modal_input(
            '자동 A/B',
            ['command.json의 start·route대로 A(ON) → 복귀 → B(OFF)를 주행합니다.',
             'Nav2 /navigate_to_pose · AUTO · E-STOP OFF 필요',
             '라벨을 입력하세요'], '라벨>', default=time.strftime('%Y%m%d_%H%M%S'))
        if label is None:
            return STAY
        rc = self.bus.ab.guide(label)
        if rc != 0:
            print(f'FAIL: 자동 A/B 실패 (rc={rc})', file=sys.stderr)
        return STAY


class RlNavControllerAbMenu(Menu):
    title = 'A/B 테스트 · rl_nav_controller'
    REFRESH_S = 1.0

    def __init__(self, app):
        super().__init__(app)
        self._lines: list[str] = []
        self._at = 0.0

    def items(self):
        return [AbStatusItem(self.app), AbToggleItem(self.app), AbRecordItem(self.app),
                AbStopItem(self.app), AbReportItem(self.app), AbGuideItem(self.app)]

    def center(self):
        now = time.monotonic()
        if not self._lines or now - self._at >= self.REFRESH_S:
            self._lines = self.bus.ab.status_lines()
            self._at = now
        return self._lines + ['', ' A/B는 같은 출발점·같은 goal로 번갈아 여러 번 (정책 ON vs inner DWB)']


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
