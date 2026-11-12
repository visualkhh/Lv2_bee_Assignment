"""보이는 물체 화면 — 상단 '보이는 물체' 칸에서 Enter. 왼쪽 = /tracked_objects 목록, 포커스한 항목의 상세가 오른쪽에.

  ↑↓ 목록에서 물체 고르기 (Enter 없이 오른쪽 상세가 바뀜) · Tab = 본문으로 포커스 → ↑↓·PgUp/PgDn·Home/End 스크롤
  목록은 2초마다 다시 그린다. 인지(perception_node)가 2초 안에 못 본 물체는 메시지에서 빠지므로 목록에서도 빠진다.
"""
from __future__ import annotations

import math
import time

from ...defs import STAY
from ...menus import InfoItem, Menu, MenuItem
from ...views import View
from ..page import Page


def _yaw_deg(q) -> float:
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def _get(o, name, default=''):
    return getattr(o, name, default)


def _seq(o, name) -> list:
    """rosidl 배열(고정 크기·numpy)을 list로 — `value or []`는 numpy에서 ambiguous하다."""
    v = getattr(o, name, None)
    if v is None:
        return []
    try:
        return list(v)
    except TypeError:
        return []


def object_label(i: int, o) -> str:
    """목록 한 줄 — #id 클래스 (색 · 태그)."""
    det = {kv.key: kv.value for kv in _seq(o, 'details')}
    extra = ' '.join(x for x in (_get(o, 'color', ''), ('태그 ' + det['tag']) if 'tag' in det else '') if x and x != 'unknown')
    return f'#{_get(o, "id", i)} {_get(o, "class_name", "") or "?"}' + (f' ({extra})' if extra else '')


def object_lines(o, now_s: float | None = None) -> list[str]:
    """물체 하나의 상세 — 위치·방향·크기·신뢰도·관측·클래스별 속성 전부."""
    out = [f'[{object_label(0, o)}]', '']
    out.append(f' 클래스    {_get(o, "class_name", "") or "?"}   신뢰도 {float(_get(o, "confidence", 0.0)):.2f}')
    pose = _get(o, 'pose', None)
    pp = _get(pose, 'pose', None) if pose is not None else None
    if pp is not None:
        p, q = pp.position, pp.orientation
        out.append(f' 위치(map) x {p.x:+.3f}  y {p.y:+.3f}  z {p.z:+.3f} m')
        out.append(f' 방향      yaw {_yaw_deg(q):+.1f}°')
        cov = _seq(pose, 'covariance')
        if len(cov) >= 8 and cov[0] >= 0 and cov[7] >= 0:
            out.append(f' 위치 오차 ±x {math.sqrt(cov[0]):.3f}  ±y {math.sqrt(cov[7]):.3f} m (공분산 1σ)')
    sz = _get(o, 'size', None)
    if sz is not None:
        out.append(f' 크기      {sz.x:.3f} × {sz.y:.3f} × {sz.z:.3f} m (영상 박스 환산 · z=클래스 기본값)')
    col = _get(o, 'color', '')
    if col:
        out.append(f' 색        {col}')
    ls = _get(o, 'last_seen', None)
    if ls is not None and now_s is not None:
        age = now_s - (ls.sec + ls.nanosec * 1e-9)
        out.append(f' 마지막 관측 {age:.1f} s 전')
    cams = _seq(o, 'cameras')
    if cams:
        out.append(f' 본 카메라  {", ".join(cams)}  (마지막 = {cams[-1]})')
    px = _seq(o, 'px')
    if len(px) == 2:
        out.append(f' 영상 픽셀  u {px[0]}  v {px[1]}')
    det = _seq(o, 'details')
    if det:
        out += ['', ' 속성 (details)']
        out += [f'   {kv.key:<14} {kv.value}' for kv in det]
    return out


class ScrollView(View):
    """줄 목록을 창 높이만큼 잘라 보여주는 본문 — 포커스(Tab)하면 ↑↓ · PgUp/PgDn · Home/End로 스크롤."""
    hints = '↑↓ 스크롤 · PgUp/PgDn 한 쪽 · Home/End 처음/끝'   # 뒤에 Window가 'Tab/Esc 메뉴로 · q 종료'를 붙인다

    def __init__(self):
        super().__init__()
        self.lines: list[str] = []
        self.top = 0
        self.page = 10
        self._key = None

    def set_lines(self, key, lines: list[str]) -> None:
        if key != self._key:      # 다른 물체로 옮기면 처음부터
            self._key, self.top = key, 0
        self.lines = lines

    def on_key(self, k) -> bool:
        step = {'up': -1, 'down': 1, 'pgup': -self.page, 'pgdn': self.page}.get(k)
        if step is not None:
            self.top += step
        elif k == 'home':
            self.top = 0
        elif k == 'end':
            self.top = len(self.lines)
        else:
            return False
        self._clamp()
        return True

    def _clamp(self) -> None:
        self.top = max(0, min(self.top, max(0, len(self.lines) - self.page)))

    def render(self, height: int = 20) -> list[str]:
        self.page = max(1, height - 1)
        self._clamp()
        view = self.lines[self.top:self.top + self.page]
        more = len(self.lines) - self.top - self.page
        foot = (f' ↓ 아래 {more}줄 (Tab → 본문에서 ↓)' if more > 0 else '') or (f' ↑ 위 {self.top}줄' if self.top else '')
        return view + ([foot] if foot else [])


class ObjectItem(MenuItem):
    def __init__(self, app, key: str, index: int, obj):
        super().__init__(app)
        self.key, self.index, self.obj = key, index, obj

    def label(self) -> str:
        return object_label(self.index, self.obj)

    def run(self) -> str:
        return STAY   # 상세는 포커스만으로 오른쪽에 뜬다 — Tab으로 본문에 가서 스크롤


class ObjectsPage(Page):
    title = '보이는 물체'

    def __init__(self, app):
        super().__init__(app)
        self.view = ScrollView()

    def _objects(self) -> list:
        objs = list(self.state.tracked)
        return sorted(objs, key=lambda o: _get(o, 'id', 0))

    def items(self):
        objs = self._objects()
        if not objs:
            return [InfoItem(self.app, '보이는 물체 없음 (2초 안에 본 것이 없거나 인지가 꺼져 있음)')]
        return [ObjectItem(self.app, str(i + 1), i, o) for i, o in enumerate(objs)]

    def _now_s(self) -> float:
        try:
            return self.app.get_clock().now().nanoseconds * 1e-9
        except Exception:   # docker·fake 모드는 ROS 시계가 없다 — 실기·시뮬 모두 last_seen이 시스템 시각이라 벽시계로 대신한다
            return time.time()

    def center(self):
        item = self.ui.window.menu_bar.focused_item
        if isinstance(item, ObjectItem):
            lines = object_lines(item.obj, self._now_s())
            key = _get(item.obj, 'id', item.index)
        else:
            lines = ['[보이는 물체]', ' 인지가 지금 /tracked_objects에 낸 물체가 없습니다.',
                     ' (use_mission:=false · use_perception:=false로 띄웠으면 항상 비어 있음)']
            key = None
        self.view.set_lines(key, lines)
        _, h = self.ui.window.center_size()
        return self.view.render(h)

    def show(self) -> str:
        return self.ui.window.run(self.title, self.items, self.center, back=True, main=True, center_comp=self.view)
