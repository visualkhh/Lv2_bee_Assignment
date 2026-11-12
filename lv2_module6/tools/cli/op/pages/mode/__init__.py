"""조작 모드·비상정지·재시작 메뉴."""
from __future__ import annotations

import argparse

from ...defs import STAY
from ...terminal import BOLD, GREEN, RED
from ...menus import Menu, MenuItem


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
            body.append('AUTO = coordinator가 skill 명령을 받는 상태 (미션 시작은 메인 → 0 미션)')
        if self._target != 'MANUAL' and not self.ui.confirm(
                f'{self._target}로 전환할까요?', default=False, body=body):
            return STAY
        self.bus.cmd_mode(argparse.Namespace(mode=self._target.lower()))
        return STAY


class ModeMenu(Menu):
    title = '모드'

    def items(self):
        return [ModeItem(self.app, '1', 'AUTO', '자동 (coordinator가 skill 실행 · 미션 시작은 0 미션)'),
                ModeItem(self.app, '2', 'MANUAL', '수동 주행/팔/그리퍼')]

    def center(self):
        st = self.state
        return [
            '[현재 모드]',
            f' 조작: {st.operation or "-"}',
            f' E-STOP: {"ON" if st.emergency else "OFF"}',
            f' 지도: {st.map_mode or "-"}',
            '',
            ' AUTO = 자동 실행 허용 · 미션 시작/중지는 메인 → 0 미션',
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


class CollisionStatusItem(MenuItem):
    key = '1'

    def label(self):
        return '상태 새로고침'

    def run(self):
        self.bus.cmd_collision(argparse.Namespace(state='status'))
        return STAY


class CollisionOnItem(MenuItem):
    key = '2'

    def label(self):
        return 'collision_monitor 켜기'

    def run(self):
        self.bus.cmd_collision(argparse.Namespace(state='on'))
        return STAY


class CollisionOffItem(MenuItem):
    key = '3'

    def label(self):
        return 'collision_monitor 끄기'

    def run(self):
        if not self.ui.confirm('collision_monitor를 끌까요?', default=False,
                               body=['끄면 벽·물체 앞에서 자동으로 서지 않습니다. 직접 몰 때만 쓰세요.',
                                     'MANUAL·MAPPING이면 이미 자동으로 꺼져 있습니다.']):
            return STAY
        self.bus.cmd_collision(argparse.Namespace(state='off'))
        return STAY


class CollisionMenu(Menu):
    title = '충돌 감시 (collision_monitor)'

    def items(self):
        return [CollisionStatusItem(self.app), CollisionOnItem(self.app), CollisionOffItem(self.app)]

    def center(self):
        return ['[collision_monitor]', f' 반영 상태: {self.bus.collision_status()}', '',
                ' 자동: MANUAL · 지도 만드는 중(MAPPING)이면 끔, 그 밖엔 켬',
                ' 직접 바꾸면 모드가 바뀔 때까지 유지됩니다']


class EstopMenu(Menu):
    title = '비상정지'

    def items(self):
        return [EstopRefreshItem(self.app), EstopOnItem(self.app), EstopOffItem(self.app)]

    def center(self):
        est = self.term.paint('ON', RED + BOLD) if self.state.emergency else self.term.paint('OFF', GREEN)
        return ['[비상정지]', f' 현재: {est}', '', ' ON은 바로 적용 (확인 없음) · 바퀴 명령 차단',
                ' OFF는 확인 후 해제']
