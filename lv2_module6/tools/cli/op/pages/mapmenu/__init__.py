"""지도/위치 메뉴 — 지도 보기(터미널 지도 화면 MapPage) · 위치 지정 · 목록 · 전환 · 저장."""
from __future__ import annotations

import argparse
import sys

from ...defs import STAY
from ...menus import GoMenu, InfoItem, Menu, MenuItem
from ..manual import ManualMenu, TeleopItem
from ..page import Page


def _teleop_item(app, key: str):
    """매핑하며 직접 몰 때 쓰는 텔레오퍼 — 수동 주행 메뉴의 항목을 키만 바꿔 재사용."""
    item = TeleopItem(app)
    item.key = key
    return item


class MapStatusItem(MenuItem):
    key = '1'

    def label(self):
        return '지도 상태'

    def run(self):
        self.bus.cmd_map(self.bus.map_args('status'))
        return STAY


class MapListItem(MenuItem):
    shows_busy = False  # 하위 화면을 연다
    key = '2'

    def label(self):
        return '지도 목록 → 보기'

    def run(self):
        return MapListMenu(self.app).show()


class MapViewItem(MenuItem):
    """지도 화면(MapPage)을 'view' 모드로 연다 — name이 없으면 지금 /map, 있으면 저장된 지도 파일."""
    shows_busy = False

    def __init__(self, app, key: str, name: str | None = None):
        super().__init__(app)
        self.key, self._name = key, name

    def label(self):
        return self._name or '지도 보기 (지금 /map · 로봇 위치)'

    def run(self):
        if self._name is None:
            return self.ui.window.navigate('map', mode='view')
        yaml = self.bus.default_map_dir() / self._name / 'map.yaml'
        if not yaml.is_file():
            print(f'이 지도는 보기용 map.yaml이 없음: {yaml.parent}', file=sys.stderr)
            return STAY
        return self.ui.window.navigate('map', mode='view', source=yaml)


class MapPoseItem(MenuItem):
    """초기 위치(initialpose) — faker는 텔레포트, 실기는 /initialpose 토픽."""
    shows_busy = False
    key = '1'

    def label(self):
        return '초기 위치 (initialpose · 지도에서 찍기)'

    def run(self):
        return self.ui.window.navigate('map', mode='pose', on_pose=self.bus.nav.set_robot_pose)


class MapGoalItem(MenuItem):
    """Nav2 이동 목표(GOAL) — 지도 위에서 한 점을 찍어 보낸다 (도착은 로봇이 알아서)."""
    shows_busy = False
    key = '2'

    def label(self):
        return '이동 목표 (goal · 지도에서 찍기)'

    def run(self):
        if self.state.operation and self.state.operation != 'AUTO':   # 모르면(빈 값) 묻지 않음 — 재연결 직후 거짓 확인창 방지
            if not self.ui.confirm('AUTO로 전환하고 이동 목표를 찍을까요?', default=True,
                                   body=['Nav2 이동 목표는 AUTO에서 실행됩니다.']):
                return STAY
            self.bus.cmd_mode(argparse.Namespace(mode='auto'))
        return self.ui.window.navigate('map', mode='goal', on_pose=self._send_goal)

    def _send_goal(self, x: float, y: float, yaw_deg: float) -> None:
        self.bus.nav.send_nav_goal({'x': x, 'y': y, 'yaw_deg': yaw_deg}, 'map')


class MapPoseMenuItem(MenuItem):
    """지도/위치 메뉴에서 initialpose·goal을 고르는 하위 메뉴."""
    shows_busy = False
    key = '8'

    def label(self):
        return '위치 지정 (initialpose / goal)'

    def run(self):
        return MapPoseMenu(self.app).show()


class MapPoseMenu(Menu):
    title = '위치 지정'

    def items(self):
        return [MapPoseItem(self.app), MapGoalItem(self.app)]

    def center(self):
        return ['[위치 지정]', f' 조작: {self.state.operation or "-"}',
                ' initialpose = 로봇 초기 위치 (faker 텔레포트)',
                ' goal = Nav2 이동 목표']


class MapListMenu(Menu):
    title = '지도 목록 → 보기'

    def items(self):
        maps = self.bus.map_names()
        return ([MapViewItem(self.app, str(i + 1), n) for i, n in enumerate(maps)]
                or [InfoItem(self.app, '지도 목록 없음')])

    def center(self):
        return ['[저장된 지도]', ' Enter = 터미널 지도 화면으로 열기', f' 폴더: {self.bus.default_map_dir()}', ' <이름>/map.yaml + map.pgm 이 있는 지도만 볼 수 있음']


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


class MapRvizItem(MenuItem):
    """매핑 전용 RViz(mapping.rviz) — 지도·/scan·TF·로봇을 위에서 내려다본다."""
    shows_busy = False
    key = '9'

    def label(self):
        return '매핑 전용 RViz 열기 (mapping.rviz)'

    def run(self):
        from ..status import MappingRvizPage
        return MappingRvizPage(self.app).show()


class NavRvizItem(MenuItem):
    """목표 이동(Nav2) 전용 RViz(nav.rviz) — 코스트맵·경로·footprint 위에서 2D Goal Pose로 목표를 준다."""
    shows_busy = False
    key = 'c'

    def label(self):
        return '목표 이동 전용 RViz 열기 (nav.rviz — 2D Goal Pose)'

    def run(self):
        from ..status import NavRvizPage
        return NavRvizPage(self.app).show()


class SlamRvizItem(MenuItem):
    """SLAM 전용 RViz 열기 — nvblox(3D 메시·ESDF·코스트맵) / cuVSLAM(경로·odom) 항목만 다르고 같다."""
    shows_busy = False

    def __init__(self, app, key: str, label: str, page: str):
        super().__init__(app)
        self.key, self._label, self._page = key, label, page

    def label(self):
        return self._label

    def run(self):
        from .. import status
        return getattr(status, self._page)(self.app).show()


class MapMenu(Menu):
    title = '지도/위치'

    def items(self):
        return [MapStatusItem(self.app), MapListItem(self.app),
                MapModeItem(self.app, '0', 'none', 'NONE (지도 없음)'),
                MapModeItem(self.app, '3', 'mapping', 'MAPPING'),
                MapModeItem(self.app, '4', 'localization', 'LOCALIZATION'),
                MapSwitchItem(self.app), MapSaveItem(self.app),
                MapViewItem(self.app, '7'), MapPoseMenuItem(self.app),
                MapRvizItem(self.app),
                NavRvizItem(self.app),
                SlamRvizItem(self.app, 'd', 'nvblox 전용 RViz 열기 (nvblox.rviz — 3D 메시 · ESDF · 코스트맵)', 'NvbloxRvizPage'),
                SlamRvizItem(self.app, 'e', 'cuVSLAM 전용 RViz 열기 (cuvslam.rviz — 경로 · odom 비교)', 'CuvslamRvizPage'),
                _teleop_item(self.app, 'a'),
                GoMenu(self.app, 'b', '수동 주행 메뉴 (전진·후진·회전·속도 설정)', ManualMenu(self.app))]

    def center(self):
        st = self.state
        maps = self.bus.map_names()
        lines = ['[지도/위치]',
                 f' 모드: {st.map_mode or "-"}',
                 f' 이름: {st.values.get("map_name") or "-"}',
                 f' 목록: {len(maps)}개']
        if st.map_mode == 'MAPPING' and st.values.get('map_name'):
            lines.append(' 작성 중 — 실제 nvblox는 LOCALIZATION 전환/종료 때 저장')
        return lines + [f'  - {n}' for n in maps[:8]]


class MapSwitchItem(MenuItem):
    shows_busy = False  # 하위 화면을 연다
    key = '5'

    def label(self):
        return '지도 전환'

    def run(self):
        return MapSwitchMenu(self.app).show()


class MapSaveItem(MenuItem):
    shows_busy = False  # 하위 화면을 연다
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
        rc = self.bus.cmd_map(self.bus.map_args('save', file=raw.strip() or None, localize=False))
        if rc == 0 and self.state.map_mode == 'MAPPING':
            if self.ui.confirm('LOCALIZATION 모드로 전환할까요?', default=True,
                               body=['지도 미리보기 저장 완료.',
                                     '전환하면 지도를 확정하고 위치추정 모드로 다시 뜹니다 (실제 nvblox는 nvblox.nvblx 저장).',
                                     '다시 뜨는 데 수십 초 걸립니다 — 주행 중이면 먼저 정지하세요.']):
                self.bus.localize_saved()
            else:
                print('LOCALIZATION 전환 생략 — nvblox.nvblx는 나중에 전환/종료할 때 저장됩니다.')
        return STAY
