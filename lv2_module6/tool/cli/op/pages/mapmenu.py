"""지도 메뉴 + 전환 + 저장."""
from __future__ import annotations

from ..defs import STAY
from .base import InfoItem, Menu, MenuItem, Page


class MapStatusItem(MenuItem):
    key = '1'

    def label(self):
        return '지도 상태'

    def run(self):
        self.bus.cmd_map(self.bus.map_args('status'))
        return STAY


class MapListItem(MenuItem):
    key = '2'

    def label(self):
        return '지도 목록'

    def run(self):
        self.bus.cmd_map(self.bus.map_args('list'))
        return STAY


class MapModeItem(MenuItem):
    def __init__(self, app, key: str, mode: str, desc: str):
        super().__init__(app)
        self.key = key
        self._mode = mode
        self._desc = desc

    def label(self):
        return f'{self._desc} 모드'

    def run(self):
        if not self.ui.confirm(f'{self._desc} 모드로 전환할까요?', default=False,
                              body=[f'현재: {self.state.map_mode or "-"}']):
            return STAY
        self.bus.cmd_map(self.bus.map_args('mode', name=self._mode))
        return STAY


class MapMenu(Menu):
    title = '지도'

    def items(self):
        return [MapStatusItem(self.app), MapListItem(self.app),
                MapModeItem(self.app, '3', 'mapping', 'MAPPING'),
                MapModeItem(self.app, '4', 'localization', 'LOCALIZATION'),
                MapSwitchItem(self.app), MapSaveItem(self.app)]

    def center(self):
        st = self.state
        maps = self.bus.map_names()
        return (['[지도]',
                 f' 모드: {st.map_mode or "-"}',
                 f' 이름: {st.values.get("map_name") or "-"}',
                 f' 목록: {len(maps)}개'] + [f'  - {n}' for n in maps[:8]])


class MapSwitchItem(MenuItem):
    key = '5'

    def label(self):
        return '지도 전환'

    def run(self):
        return MapSwitchMenu(self.app).show()


class MapSaveItem(MenuItem):
    key = '6'

    def label(self):
        return '지도 저장'

    def run(self):
        return MapSavePage(self.app).show()


class SwitchTargetItem(MenuItem):
    """전환할 지도 한 개."""

    def __init__(self, app, key: str, name: str):
        super().__init__(app)
        self.key = key
        self._name = name

    def label(self):
        return self._name

    def run(self):
        if not self.ui.confirm(f'지도 "{self._name}"(으)로 전환할까요?', default=False,
                              body=[f'현재: {self.state.values.get("map_name") or "-"}']):
            return STAY
        self.bus.cmd_map(self.bus.map_args('switch', name=self._name))
        return STAY


class SwitchNameItem(MenuItem):
    key = 'n'

    def label(self):
        return '이름 직접 입력'

    def run(self):
        name = self.ui.modal_input('지도 이름', ['전환할 지도 폴더 이름'], '이름>',
                                   validate=lambda t: None if t.strip() else '이름이 비었습니다')
        if name is None:
            return STAY
        name = name.strip()
        if not self.ui.confirm(f'지도 "{name}"(으)로 전환할까요?', default=False,
                               body=[f'현재: {self.state.values.get("map_name") or "-"}']):
            return STAY
        self.bus.cmd_map(self.bus.map_args('switch', name=name))
        return STAY


class MapSwitchMenu(Menu):
    title = '지도 전환'

    def items(self):
        maps = self.bus.map_names()
        items: list[MenuItem] = [SwitchTargetItem(self.app, str(i + 1), n)
                                 for i, n in enumerate(maps)]
        if not items:
            items = [InfoItem(self.app, '지도 목록 없음')]
        return items + [SwitchNameItem(self.app)]

    def center(self):
        st = self.state
        return ['[지도 전환]', f' 현재: {st.values.get("map_name") or "-"}',
                f' 모드: {st.map_mode or "-"}']


class MapSavePage(Page):
    title = '지도 저장'

    def show(self) -> str:
        name = self.state.values.get('map_name') or ''
        default = self.bus.default_map_dir() / name / 'map' if name else None
        raw = self.ui.modal_input(
            '지도 저장',
            [f'현재 지도: {name or "-"}',
             f'기본 경로: {default or "(없음 — 경로를 입력해야 함)"}',
             '현재 /map을 이 경로로 저장합니다 (빈칸 Enter = 기본 경로)'], '경로>',
            validate=lambda t: None if t.strip() or name else '현재 지도 이름이 없습니다 — 경로를 입력하세요')
        if raw is None:
            return STAY
        self.bus.cmd_map(self.bus.map_args('save', file=raw.strip() or None))
        return STAY
