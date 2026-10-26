"""메인 메뉴."""
from __future__ import annotations

from ..defs import MAIN
from .base import GoMenu, Menu

ENTRIES = [
    ('1', '상태'),
    ('2', '장치'),
    ('3', '조작 모드'),
    ('4', '수동 주행 / 텔레오퍼'),
    ('5', '팔'),
    ('6', '그리퍼'),
    ('7', '지도'),
    ('8', '비상정지'),
    ('9', '재시작'),
    ('10', 'RViz 열기 (Best Effort QoS)'),
    ('11', '파라미터 (노드별)'),
]


class MainMenu(Menu):
    title = '메인 메뉴'
    root = True

    def items(self):
        return [GoMenu(self.app, key, label, self.app.screen(key)) for key, label in ENTRIES]

    def center(self):
        return self.ui.center_summary()

    def show(self) -> int:
        self.app.poll(0.6)
        while super().show() == MAIN:  # 하위 화면의 m = 여기로 돌아옴
            pass
        self.term.clear()
        print('operator_tool 종료')
        return 0
