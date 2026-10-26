"""장치 메뉴 + 장치 상세."""
from __future__ import annotations

import sys

from ..consts import DIAG_LEVELS
from ..defs import STAY
from ..terminal import GREEN, RED, YELLOW
from .base import InfoItem, Menu, MenuItem
from .params import NodeItem, NodeParamMenu


class DeviceItem(MenuItem):
    """장치 하나 — Enter가 상세 화면."""

    def __init__(self, app, key: str, name: str):
        super().__init__(app)
        self.key = key
        self._name = name

    def label(self):
        kind, state, _ = self.state.device_info(self._name)
        if state == 'active':
            mark = self.term.paint('●', GREEN)
        elif state:
            mark = self.term.paint('○', RED)
        else:
            mark = self.term.paint('?', YELLOW)
        return f'{mark} {self._name} ({kind})'

    def run(self):
        return DeviceDetailMenu(self.app, self._name).show()


class DevicesMenu(Menu):
    title = '장치'

    def items(self):
        if not self.state.devices:
            return [InfoItem(self.app, '장치 목록 없음 (/state/devices 대기 중)')]
        return [DeviceItem(self.app, str(i + 1), n)
                for i, n in enumerate(self.state.devices)]

    def center(self):
        return ['[장치 상세는 Enter]'] + self.ui.center_summary()


class DeviceParamItem(MenuItem):
    """이 장치를 내는 노드의 파라미터 — 노드별 파라미터 화면과 같은 화면을 연다."""

    key = '1'

    def __init__(self, app, topics):
        super().__init__(app)
        self._topics = topics

    def label(self):
        return '파라미터 (발행 노드)'

    def run(self):
        cands = self.bus.param_target_nodes(self._topics)
        if not cands:
            print('거부: 이 장치의 발행 노드를 찾지 못했습니다 — bringup/토픽 확인',
                  file=sys.stderr)
            return STAY
        if len(cands) == 1:
            return NodeParamMenu(self.app, *cands[0]).show()
        return NodeChoiceMenu(self.app, cands).show()


class NodeChoiceMenu(Menu):
    """발행 노드가 여럿일 때 고르기."""

    title = '노드 선택'

    def __init__(self, app, nodes):
        super().__init__(app)
        self._nodes = nodes

    def items(self):
        return [NodeItem(self.app, str(i + 1), n, ns) for i, (n, ns) in enumerate(self._nodes)]

    def center(self):
        return ['[파라미터 대상 노드]']


class ShowFullItem(MenuItem):
    key = '2'

    def __init__(self, app, detail: 'DeviceDetailMenu'):
        super().__init__(app)
        self._detail = detail

    def label(self):
        return '전체 상세 보기'

    def run(self):
        self._detail.show_full()
        return STAY


class DeviceDetailMenu(Menu):
    def __init__(self, app, name: str):
        super().__init__(app)
        self._name = name
        self.title = f'장치 · {name}'

    def items(self):
        topics = self._topics()
        return [DeviceParamItem(self.app, topics), ShowFullItem(self.app, self)]

    def _topics(self):
        _, _, topics = self.state.device_info(self._name)
        return topics

    def center(self):
        kind, state, topics = self.state.device_info(self._name)
        if state == 'active':
            state_txt = self.term.paint('● active', GREEN)
        elif state:
            state_txt = self.term.paint(f'○ {state}', RED)
        else:
            state_txt = self.term.paint('? 상세 정보 없음', YELLOW)
        center = [f'[장치 · {self._name}]', f' 종류: {kind}', f' 상태: {state_txt}',
                  f' 토픽/관절: {len(topics)}개']
        center += [f'  - {t}' for t in topics[:6]]
        if len(topics) > 6:
            center.append(f'  … 외 {len(topics) - 6}개')
        related = []
        for diag_name, (level, message) in self.state.diagnostics.items():
            if self._name in diag_name or any(t in diag_name for t in topics):
                related.append((diag_name, level, message))
        self._related = sorted(related)[:3]
        if related:
            center.append(' 관련 진단:')
            for diag_name, level, message in self._related:
                center.append(f'  - {diag_name}: {DIAG_LEVELS.get(level, level)}')
        else:
            center.append(' 관련 진단: -')
        self._state_txt = state_txt
        self._kind = kind
        return center

    def show_full(self) -> None:
        """장치 전체 상세 (잘림 없이 전부)."""
        self.app.poll(0.45)
        print(f'  종류   : {self._kind}')
        print(f'  상태   : {self._state_txt}')
        print('  세부 토픽/관절:')
        topics = self._topics()
        if topics:
            for topic in topics:
                print(f'    - {topic}')
        else:
            print('    -')
        if self._related:
            print('  관련 진단:')
            for diag_name, level, message in self._related:
                print(f'    - {diag_name}: {level} {message}')
        else:
            print('  관련 진단: -')
