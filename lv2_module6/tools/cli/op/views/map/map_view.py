"""MapView — 지도를 터미널 글자로 그리는 포커스 가능한 부품 (Component). 색이 되면 반블록 ▀(한 글자에 두 칸), 아니면 # . 문자.

화면 한가운데 = 십자선(+) = 찍을 자리. 로봇은 빨간 점(●). 로봇 따라가기(following)가 켜져 있으면 중심이 로봇을 따라간다
(지도가 한 화면에 다 보이는 확대에서는 지도를 고정하고 로봇 점이 움직인다). 방향키로 지도를 밀면 따라가기가 풀리고, r로 다시 켠다.
"""
from __future__ import annotations

import math

from ..view import View
from .grid import FREE, OCC, OUT, UNKNOWN, CostGrid, MapGrid
from .viewport import Viewport

RGB = {UNKNOWN: (70, 70, 78), FREE: (205, 205, 210), OCC: (18, 18, 24)}   # 칸 색 — OUT은 색 없음 (터미널 바탕)
MAP_RGB, ODOM_RGB = (90, 220, 255), (255, 150, 40)   # map 원점 M · odom 원점 O
PATH_RGB, ROBOT_RGB, GOAL_RGB, START_RGB, CURSOR_RGB = (240, 190, 40), (235, 60, 60), (60, 200, 90), (70, 150, 255), (255, 80, 220)
COST_INFLATED, COST_LETHAL = (125, 85, 25), (175, 35, 35)   # 코스트맵 겹침 — 팽창 / 내접·치명
BRAILLE = {(0, 0): 0x01, (0, 1): 0x02, (0, 2): 0x04, (1, 0): 0x08, (1, 1): 0x10, (1, 2): 0x20, (0, 3): 0x40, (1, 3): 0x80}
YAW_LEN, YAW_GAP, HEAD_LEN = 16, 3, 5   # 방향선 길이 · 십자선에서 비우는 거리 · 화살표 머리 길이 (점 단위 — 점자 한 글자 = 2×4점)
PAN_KEYS = {'up': (0, 1), 'down': (0, -1), 'left': (-1, 0), 'right': (1, 0)}


def _yaw_dots(cx: int, cy: int, yaw_deg: float) -> set[tuple[int, int]]:
    """점자 점 좌표(가로 2점/칸 · 세로 4점/줄 — 점은 정사각형)에서 (cx, cy)로부터 yaw 방향 선 + 화살표 머리."""
    rad = math.radians(yaw_deg)
    pts = set()

    def ray(x0, y0, ang, a, b):
        t = a
        while t <= b:
            pts.add((int(round(x0 + t * math.cos(ang))), int(round(y0 - t * math.sin(ang)))))
            t += 0.5
    ray(cx, cy, rad, YAW_GAP, YAW_LEN)
    tip = (cx + YAW_LEN * math.cos(rad), cy - YAW_LEN * math.sin(rad))
    for side in (-1, 1):   # 머리 = 끝에서 뒤로 ±28° 두 줄
        ray(tip[0], tip[1], rad + math.pi + side * math.radians(28), 0, HEAD_LEN)
    return pts


def _line(a: tuple[int, int], b: tuple[int, int]):
    """두 화면 칸 사이를 잇는 칸들 (브레젠험)."""
    (x0, y0), (x1, y1) = a, b
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx + dy
    for _ in range(dx - dy + 1):
        yield x0, y0
        if (x0, y0) == (x1, y1):
            return
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


class MapView(View):
    """grid를 vp(Viewport)로 잘라 cols × rows 글자 영역으로 그린다. robot·start·goals는 밖(페이지)이 갱신, 키는 on_key."""

    focusable = True
    hints = '↑↓←→ 지도 이동 · +/- 확대·축소 · f 전체 · r 로봇 따라가기 · c costmap · [ ] 방향 15° · { } 1° · Enter 찍기'

    def __init__(self, grid: MapGrid, color: bool = True, extra_key=None):
        self.grid, self.color = grid, color
        self.vp = Viewport(grid)
        self.following = True
        self.robot: tuple[float, float] | None = None
        self.start: tuple[float, float] | None = None
        self.odom: tuple[float, float] | None = None   # map 기준 odom 프레임 원점 (None = TF 없음)
        self.show_frames = True      # map 원점 M · odom 원점 O (로봇 ● = base_footprint)
        self.goals: list[dict] = []
        self.yaw = 0.0
        self.show_yaw = False       # True면 십자선에서 방향(yaw)선을 뻗어 그림 — 찍는 모드(goal · 위치 지정)
        self.size = (60, 20)        # 마지막으로 그린 (cols, rows) — 키 처리(전체 보기)가 쓴다
        super().__init__(extra_key)  # 지도가 모르는 키(goal 추가 등)를 페이지로 넘긴다
        self._key = None
        self._codes: list[list[int]] = []
        self.cost: CostGrid | None = None   # 겹쳐 볼 코스트맵 (None = 없음)
        self.show_cost = False
        self._cost_codes: list[list[int]] | None = None

    @property
    def center(self) -> tuple[float, float]:
        """십자선이 가리키는 세계 좌표 = 화면 한가운데."""
        return self.vp.cx, self.vp.cy

    # ---------------------------------------------------------------- 키
    def on_key(self, k) -> bool:
        cols, rows = self.size
        vp = self.vp
        if k in PAN_KEYS:
            self.following = False
            vp.pan(*PAN_KEYS[k])
        elif k in ('+', '='):
            vp.zoom(+1)
        elif k in ('-', '_'):
            vp.zoom(-1)
        elif k in ('f', 'F'):
            vp.fit(cols, rows * 2)
            self.following = True
        elif k in ('r', 'R'):
            self.following = True
        else:
            return self.forward_extra(k)
        return True

    # ---------------------------------------------------------------- 그리기
    def _follow(self, cols: int, sub: int) -> None:
        vp = self.vp
        if not self.following:
            return
        if vp.fits(cols, sub):
            vp.center_on_map()
        elif self.robot:
            vp.cx, vp.cy = self.robot
            vp.snap()

    def _base(self, cols: int, subrows: int) -> list[list[int]]:
        key = (self.vp.k, round(self.vp.cx, 6), round(self.vp.cy, 6), cols, subrows,
               self.show_cost, id(self.cost) if self.cost is not None else None)
        if key != self._key:
            vp, g = self.vp, self.grid
            self._codes = [[g.code(vp.k, *vp.to_world(c + 0.5, r + 0.5, cols, subrows)) for c in range(cols)]
                           for r in range(subrows)]
            if self.show_cost and self.cost is not None:
                self._cost_codes = [[self.cost.code(*vp.to_world(c + 0.5, r + 0.5, cols, subrows))
                                     for c in range(cols)] for r in range(subrows)]
            else:
                self._cost_codes = None
            self._key = key
        return self._codes

    def render(self, cols: int, rows: int) -> list[str]:
        """rows줄 (각 cols칸). 안 그려지는 칸(지도 밖)은 공백."""
        self.size = (cols, rows)
        sub = rows * 2
        vp = self.vp
        self._follow(cols, sub)
        codes = self._base(cols, sub)
        cost_codes = self._cost_codes
        dots: dict[tuple[int, int], tuple] = {}      # 화면 칸 → 색 (경로 점)
        labels: dict[tuple[int, int], tuple] = {}    # 글자 칸 → (글자, 색)

        def scr(p):
            c, r = vp.to_screen(p[0], p[1], cols, sub)
            return int(math.floor(c)), int(math.floor(r))

        pts = ([self.start] if self.start else []) + [(g['x'], g['y']) for g in self.goals]
        for a, b in zip(pts, pts[1:]):
            for c, r in _line(scr(a), scr(b)):
                if 0 <= c < cols and 0 <= r < sub:
                    dots[(c, r)] = PATH_RGB

        def mark(p, ch, rgb):
            c, r = scr(p)
            if 0 <= c < cols and 0 <= r < sub:
                labels[(c, r // 2)] = (ch, rgb)
        if self.start:
            mark(self.start, 'S', START_RGB)
        for i, g in enumerate(self.goals, 1):
            mark((g['x'], g['y']), str(i % 10), GOAL_RGB)
        if self.show_frames:
            mark((0.0, 0.0), 'M', MAP_RGB)
            if self.odom:
                mark(self.odom, 'O', ODOM_RGB)
        robot_cell = None
        if self.robot:
            c, r = scr(self.robot)
            if 0 <= c < cols and 0 <= r < sub:
                robot_cell = (c, r // 2)
                labels[robot_cell] = ('●' if self.color else '@', ROBOT_RGB)
        cur = (cols // 2, (sub // 2) // 2)
        braille: dict[tuple[int, int], int] = {}   # 글자 칸 → 점자 점 비트 (방향선)
        if self.show_yaw:
            for dx, dy in _yaw_dots(cur[0] * 2 + 1, cur[1] * 4 + 2, self.yaw):
                c, r = dx // 2, dy // 4
                if 0 <= c < cols and 0 <= r < rows and (c, r) != cur:
                    braille[(c, r)] = braille.get((c, r), 0) | BRAILLE[(dx % 2, dy % 4)]
        # 십자선 = 화면 한가운데 (찍을 자리). 로봇도 같은 칸이면 ⊕로 둘 다 보이게 한다.
        labels[cur] = (('⊕', ROBOT_RGB) if robot_cell == cur else ('+', CURSOR_RGB))

        out = []
        for tr in range(rows):
            row = []
            cur_sgr = None   # 직전에 낸 색 — 같은 색이 이어지면 코드를 다시 안 붙여 출력을 줄인다 (꾹 누를 때 밀리지 않게)

            def put(sgr: str, ch: str):
                nonlocal cur_sgr
                row.append(ch if sgr == cur_sgr else sgr + ch)
                cur_sgr = sgr
            for c in range(cols):
                top, bot = codes[2 * tr][c], codes[2 * tr + 1][c]
                lab = labels.get((c, tr))
                if self.color:
                    tc = dots.get((c, 2 * tr)) or (RGB.get(top) if top != OUT else None)
                    bc = dots.get((c, 2 * tr + 1)) or (RGB.get(bot) if bot != OUT else None)
                    if cost_codes and (c, 2 * tr) not in dots and (c, 2 * tr + 1) not in dots:
                        ct = max(cost_codes[2 * tr][c], cost_codes[2 * tr + 1][c])
                        if ct and top != OCC and bot != OCC:   # 벽은 지도 색 그대로, 빈 칸의 팽창만 덮는다
                            tc = bc = COST_LETHAL if ct == 2 else COST_INFLATED
                    if lab:
                        bg = bc or tc or (0, 0, 0)
                        f = lab[1]
                        if (c, tr) == cur:   # 십자선 = 분홍 바탕에 흰 +  (로봇과 같은 칸이면 빨간 바탕)
                            bg, f = (ROBOT_RGB if (c, tr) == robot_cell else CURSOR_RGB), (255, 255, 255)
                        put(f'\033[1;38;2;{f[0]};{f[1]};{f[2]}m\033[48;2;{bg[0]};{bg[1]};{bg[2]}m', lab[0])
                    elif (c, tr) in braille:   # 방향선 — 지도 바탕 위에 분홍 점자
                        bg = bc or tc or (0, 0, 0)
                        put(f'\033[1;38;2;{CURSOR_RGB[0]};{CURSOR_RGB[1]};{CURSOR_RGB[2]}m'
                            f'\033[48;2;{bg[0]};{bg[1]};{bg[2]}m', chr(0x2800 + braille[(c, tr)]))
                    elif tc is None and bc is None:
                        put('\033[0m', ' ')
                    else:
                        t, b = tc or (0, 0, 0), bc or (0, 0, 0)
                        put(f'\033[22;38;2;{t[0]};{t[1]};{t[2]}m\033[48;2;{b[0]};{b[1]};{b[2]}m', '▀')
                else:
                    code = max(top, bot) if OUT not in (top, bot) else min(top, bot)
                    on_path = (c, 2 * tr) in dots or (c, 2 * tr + 1) in dots or (c, tr) in braille
                    ch = lab[0] if lab else (':' if on_path else {OCC: '#', FREE: '.', UNKNOWN: ' ', OUT: ' '}[code])
                    if cost_codes and not lab and not on_path:
                        ct = max(cost_codes[2 * tr][c], cost_codes[2 * tr + 1][c])
                        if ct and code != OCC:
                            ch = 'x' if ct == 2 else '~'
                    row.append(ch)
            out.append(''.join(row) + ('\033[0m' if self.color else ''))
        return out
