"""노드 메뉴 — 전체 재시작 · 노드별 상태 · 재시작 · 수명주기 활성화."""
from __future__ import annotations

import argparse
import time

from ..defs import STAY
from ..terminal import BOLD, DIM, GREEN, RED, YELLOW
from .base import InfoItem, Menu, MenuItem


class RestartAllItem(MenuItem):
    key = 'a'

    def label(self):
        return '전체 재시작 (bringup launch 전체를 같은 명령으로 다시 띄움)'

    def run(self):
        confirm = self.ui.modal_input(
            '전체 재시작',
            ['bringup 전체(카메라·컨트롤러·Nav2·모든 노드)를 끄고 같은 명령으로 다시 띄웁니다.',
             '팔은 시작 자세로 다시 갑니다. 운영 중인 로봇에서는 주의하세요.',
             '계속하려면 RESTART를 입력'], '확인>',
            validate=lambda t: None if t.strip() == 'RESTART' else 'RESTART를 정확히 입력하세요')
        if confirm is None:
            return STAY
        self.bus.cmd_restart(argparse.Namespace(target='all', yes=True))
        print('결과: 메인 → 상태의 "재시작" 줄 · 다시 뜨는 데 30초쯤')
        return STAY


class NodeStatusItem(MenuItem):
    key = '1'

    def __init__(self, app, name: str):
        super().__init__(app)
        self._name = name

    def label(self):
        return '상태 보기'

    def run(self):
        print('\n'.join(self.bus.node_status_lines(self._name)))
        return STAY


class NodeRestartItem(MenuItem):
    key = '2'

    def __init__(self, app, name: str, lifecycle: bool):
        super().__init__(app)
        self._name = name
        self._lifecycle = lifecycle

    def label(self):
        return '재시작' + (' (다시 뜬 뒤 active까지)' if self._lifecycle else '')

    def run(self):
        if not self.ui.confirm(f'{self._name} 노드를 재시작할까요?', default=False,
                               body=['같은 명령줄·파라미터로 프로세스를 다시 띄웁니다.']):
            return STAY
        self.bus.node_restart(self._name)
        return STAY


class NodeActivateItem(MenuItem):
    key = '3'

    def __init__(self, app, name: str):
        super().__init__(app)
        self._name = name

    def label(self):
        return '활성화 (unconfigured·inactive → active)'

    def run(self):
        self.bus.lifecycle_activate(self._name)
        return STAY


class NodeActionMenu(Menu):
    def __init__(self, app, row: dict):
        super().__init__(app)
        self._row = row
        self.title = f'노드 · {row["name"]}'

    def items(self):
        items: list[MenuItem] = [NodeStatusItem(self.app, self._row['name'])]
        if self._row['pid']:
            items.append(NodeRestartItem(self.app, self._row['name'], self._row['lifecycle']))
        if self._row['lifecycle']:
            items.append(NodeActivateItem(self.app, self._row['name']))
        return items

    def center(self):
        r = self._row
        lines = [f'[{r["name"]}]', f' pid: {r["pid"] or "- (재시작 목록 밖)"}',
                 f' 그래프: {"보임" if r["alive"] else "안 보임"}']
        if r['lifecycle']:
            lines.append(' 수명주기 노드 — active가 아니면 3) 활성화')
        return lines


class NodeItem(MenuItem):
    def __init__(self, app, key: str, row: dict, life: str | None):
        super().__init__(app)
        self.key = key
        self._row = row
        self._life = life

    def label(self):
        r = self._row
        mark = self.term.paint('●', GREEN) if r['alive'] else self.term.paint('✗', RED + BOLD)
        text = f'{mark} {r["name"]}'
        if r['lifecycle']:
            life = self._life or '?'
            text += ' ' + self.term.paint(f'[{life}]', GREEN if life == 'active' else RED + BOLD)
        if not r['pid']:
            text += self.term.paint('  (재시작 불가)', DIM)
        return text

    def run(self):
        return NodeActionMenu(self.app, self._row).show()


class NodesMenu(Menu):
    title = '노드 (상태 · 재시작)'
    REFRESH_S = 3.0  # 노드 목록·수명주기 상태 조회 주기 (메뉴는 0.2초마다 다시 그림)

    def __init__(self, app):
        super().__init__(app)
        self._rows: list[dict] = []
        self._life: dict[str, str | None] = {}
        self._at = 0.0

    def _refresh(self):
        if time.monotonic() - self._at < self.REFRESH_S:
            return
        self._rows = self.bus.node_rows()
        self._life = {r['name']: self.bus.lifecycle_state(r['name'], 0.5) for r in self._rows if r['lifecycle']}
        self._at = time.monotonic()

    def items(self):
        self._refresh()
        items: list[MenuItem] = [RestartAllItem(self.app)]
        if not self._rows:
            return items + [InfoItem(self.app, '보이는 노드 없음 (bringup 확인)')]
        return items + [NodeItem(self.app, str(i + 1), r, self._life.get(r['name']))
                        for i, r in enumerate(self._rows)]

    def center(self):
        dead = [r['name'] for r in self._rows if not r['alive']]
        idle = [n for n, s in self._life.items() if s != 'active']
        note = self.state.values.get('restart') or '-'
        lines = ['[노드]', f' 보이는 노드 {sum(r["alive"] for r in self._rows)}개']
        lines.append(' ' + (self.term.paint('안 보임: ' + ', '.join(dead), RED) if dead else '안 보이는 노드 없음'))
        lines.append(' ' + (self.term.paint('active 아님: ' + ', '.join(idle), YELLOW) if idle
                            else '수명주기 노드 전부 active'))
        lines += ['', f' 최근 재시작: {note}', ' a) 전체 재시작은 RESTART 입력 필요']
        return lines
