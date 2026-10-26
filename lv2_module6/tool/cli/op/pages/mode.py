"""조작 모드·비상정지·재시작 메뉴."""
from __future__ import annotations

import argparse

from ..defs import STAY
from ..terminal import BOLD, GREEN, RED
from .base import InfoItem, Menu, MenuItem


class ModeItem(MenuItem):
    """조작 모드 전환 한 가지."""

    def __init__(self, app, key: str, target: str, desc: str):
        super().__init__(app)
        self.key = key
        self._target = target
        self._desc = desc

    def label(self):
        return f'{self._target:<10} — {self._desc}'

    def run(self):
        body = [f'현재: {self.state.operation or "-"}']
        if self._target == 'AUTO':
            body.append('AUTO로 바꾸면 자동 주행이 시작될 수 있습니다.')
        if self._target == 'EXPLORE' and self.state.map_mode != 'MAPPING':
            body.append(f'주의: EXPLORE는 지도 모드 MAPPING에서만 허용됩니다 (지금 {self.state.map_mode or "-"}).')
        if self._target != 'MANUAL' and not self.ui.confirm(
                f'{self._target}로 전환할까요?', default=False, body=body):
            return STAY
        self.bus.cmd_mode(argparse.Namespace(mode=self._target.lower()))
        return STAY


class ModeMenu(Menu):
    title = '조작 모드'

    def items(self):
        return [ModeItem(self.app, '1', 'AUTO', '자동 주행'),
                ModeItem(self.app, '2', 'MANUAL', '수동 주행/팔/그리퍼'),
                ModeItem(self.app, '3', 'EXPLORE', 'MAPPING 중 탐사')]

    def center(self):
        st = self.state
        return [
            '[현재 모드]',
            f' 조작: {st.operation or "-"}',
            f' E-STOP: {"ON" if st.emergency else "OFF"}',
            f' 지도: {st.map_mode or "-"}',
            '',
            ' AUTO 전환 시 자동 주행 시작 주의',
        ]


class EstopRefreshItem(MenuItem):
    key = '1'

    def label(self):
        return '상태 새로고침'

    def run(self):
        self.bus.cmd_estop(argparse.Namespace(state='status'))
        return STAY


class EstopOnItem(MenuItem):
    key = '2'

    def label(self):
        return '비상정지 ON'

    def run(self):
        # 멈추는 쪽은 묻지 않는다 — 확인창이 정지를 늦추면 안 됨
        self.bus.cmd_estop(argparse.Namespace(state='on'))
        return STAY


class EstopOffItem(MenuItem):
    key = '3'

    def label(self):
        return '비상정지 OFF'

    def run(self):
        if not self.ui.confirm('비상정지를 해제할까요?', default=False,
                               body=['해제하면 바퀴·팔 명령이 다시 통합니다. 주변을 확인하세요.']):
            return STAY
        self.bus.cmd_estop(argparse.Namespace(state='off'))
        return STAY


class EstopMenu(Menu):
    title = '비상정지'

    def items(self):
        return [EstopRefreshItem(self.app), EstopOnItem(self.app), EstopOffItem(self.app)]

    def center(self):
        est = self.term.paint('ON', RED + BOLD) if self.state.emergency else self.term.paint('OFF', GREEN)
        return ['[비상정지]', f' 현재: {est}', '', ' ON은 바로 적용 (확인 없음) · 바퀴 명령 차단',
                ' OFF는 확인 후 해제']


class RestartItem(MenuItem):
    """재시작 대상 한 개 (노드 이름)."""

    def __init__(self, app, key: str, target: str):
        super().__init__(app)
        self.key = key
        self._target = target

    def label(self):
        return self._target

    def run(self):
        if not self.ui.confirm(f'{self._target} 노드를 재시작할까요?', default=False):
            return STAY
        self.bus.cmd_restart(argparse.Namespace(target=self._target, yes=True))
        return STAY


class RestartAllItem(MenuItem):
    key = 'a'

    def label(self):
        return '전체 스택 재시작'

    def run(self):
        confirm = self.ui.modal_input(
            '전체 스택 재시작',
            ['전체 스택이 재시작됩니다. 운영 중인 로봇에서는 주의하세요.',
             '계속하려면 RESTART를 입력'], '확인>',
            validate=lambda t: None if t.strip() == 'RESTART' else 'RESTART를 정확히 입력하세요')
        if confirm is None:
            return STAY
        self.bus.cmd_restart(argparse.Namespace(target='all', yes=True))
        return STAY


class RestartNameItem(MenuItem):
    key = 'n'

    def label(self):
        return '노드 이름 직접 입력'

    def run(self):
        target = self.ui.modal_input('노드 이름', ['재시작할 노드 (all=전체)'], '이름>')
        if target is not None:
            target = target.strip()
        if not target:
            return STAY
        if target == 'all':
            return RestartAllItem(self.app).run()
        if not self.ui.confirm(f'{target} 노드를 재시작할까요?', default=False):
            return STAY
        self.bus.cmd_restart(argparse.Namespace(target=target, yes=True))
        return STAY


class RestartMenu(Menu):
    title = '재시작'

    def items(self):
        restartable = [line.split()[0] for line in
                       (self.state.values.get('restartable') or '').splitlines() if line.split()]
        items: list[MenuItem] = [RestartItem(self.app, str(i + 1), n)
                                 for i, n in enumerate(restartable)]
        if not items:
            items = [InfoItem(self.app, '재시작 가능한 노드 없음')]
        return items + [RestartAllItem(self.app), RestartNameItem(self.app)]

    def center(self):
        note = self.state.values.get('restart') or '-'
        return ['[재시작]', f' 최근 요청: {note}', '',
                ' 전체 재시작은 RESTART 입력 필요']
