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
PLAN_RGB, LOCAL_PLAN_RGB, WAYPOINT_RGB = (20, 90, 255), (170, 0, 255), (255, 110, 0)   # 전역 경로(/plan) 파랑 · 지역 경로(/local_plan) 보라 · 경유점 주황 — 밝은 회색 지도·검은 벽·분홍 십자선·빨강 로봇과 겹쳐도 구분됨
COST_INFLATED, COST_LETHAL = (125, 85, 25), (175, 35, 35)   # 코스트맵 겹침 — 팽창 / 내접·치명
BRAILLE = {(0, 0): 0x01, (0, 1): 0x02, (0, 2): 0x04, (1, 0): 0x08, (1, 1): 0x10, (1, 2): 0x20, (0, 3): 0x40, (1, 3): 0x80}
YAW_LEN, YAW_GAP, HEAD_LEN = 16, 3, 5   # 방향선 길이 · 십자선에서 비우는 거리 · 화살표 머리 길이 (점 단위 — 점자 한 글자 = 2×4점)
FRAME_LEN, FRAME_GAP, FRAME_HEAD = 10, 3, 4   # M·O·로봇의 방향 화살표 — 십자선 방향선보다 짧게
PAN_KEYS = {'up': (0, 1), 'down': (0, -1), 'left': (-1, 0), 'right': (1, 0)}


def _yaw_dots(cx: int, cy: int, yaw_deg: float, length: int = YAW_LEN, gap: int = YAW_GAP,
              head: int = HEAD_LEN) -> set[tuple[int, int]]:
    """점자 점 좌표(가로 2점/칸 · 세로 4점/줄 — 점은 정사각형)에서 (cx, cy)로부터 yaw 방향 선 + 화살표 머리."""
    rad = math.radians(yaw_deg)
    pts = set()

    def ray(x0, y0, ang, a, b):
        t = a
        while t <= b:
            pts.add((int(round(x0 + t * math.cos(ang))), int(round(y0 - t * math.sin(ang)))))
            t += 0.5
    ray(cx, cy, rad, gap, length)
    tip = (cx + length * math.cos(rad), cy - length * math.sin(rad))
    for side in (-1, 1):   # 머리 = 끝에서 뒤로 ±28° 두 줄
        ray(tip[0], tip[1], rad + math.pi + side * math.radians(28), 0, head)
    return pts


def _cell_bg(top: int, bot: int, tc, bc):
    """글자 한 칸(위·아래 두 칸)의 배경색 — 배경은 한 색뿐이라, 벽이 반쪽이라도 있으면 벽 색 (화살표가 벽을 회색으로 지우지 않게)."""
    return RGB[OCC] if OCC in (top, bot) else (bc or tc)


# 1/4 블록 — 글자 한 칸 = 가로 2 × 세로 2칸, 빈틈 없이 채워져서 가는 벽도 이어진 선으로 보임 (점자는 점 사이가 비어 점선처럼 보였음)
QUAD = ' ▘▝▀▖▌▞▛▗▚▐▜▄▙▟█'   # 비트: 왼위 1 · 오른위 2 · 왼아래 4 · 오른아래 8
QUAD_BIT = {(0, 0): 1, (1, 0): 2, (0, 1): 4, (1, 1): 8}


def _quad_cell(sub: list):
    """글자 한 칸의 1/4 블록 4칸(((dx, dy), 코드) 4개) → (블록 비트, 앞색, 배경색, 벽 있음).
    앞색 = 벽 > 모름 > 빈 곳 순으로 있는 가장 센 종류, 배경 = 나머지 중 가장 많은 종류 (지도 밖이면 None = 터미널 바탕)."""
    cnt: dict[int, int] = {}
    for _, cd in sub:
        cnt[cd] = cnt.get(cd, 0) + 1
    wall = OCC in cnt
    if len(cnt) == 1:
        (cd,) = cnt
        return 0, None, RGB.get(cd), wall
    fg_cls = next(c for c in (OCC, UNKNOWN, FREE) if c in cnt)
    rest = {c: n for c, n in cnt.items() if c != fg_cls}
    bg_cls = max(rest, key=rest.get)
    bits = 0
    for pos, cd in sub:
        if cd == fg_cls:
            bits |= QUAD_BIT[pos]
    return bits, RGB[fg_cls], RGB.get(bg_cls), wall


def _bg(rgb) -> str:
    """배경색 SGR — 지도 밖(None)은 칠하지 않고 터미널 바탕 그대로 (검은 네모가 안 생기게)."""
    return f'\033[48;2;{rgb[0]};{rgb[1]};{rgb[2]}m' if rgb else '\033[49m'


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
    hints = '↑↓←→ 지도 이동 · +/- 확대·축소 · f 전체 · r 로봇 따라가기 · c costmap · p 경로 · m M · o O · [ ] 좌·우 회전 15° · { } 1° · Enter 찍기'

    def __init__(self, grid: MapGrid, color: bool = True, extra_key=None):
        self.grid, self.color = grid, color
        self.vp = Viewport(grid)
        self.following = True
        self.robot: tuple[float, float] | None = None
        self.start: tuple[float, float] | None = None
        self.odom: tuple[float, float] | None = None   # map 기준 odom 프레임 원점 (None = TF 없음)
        self.odom_yaw: float | None = None   # map 기준 odom 프레임 방향 [deg] — O에서 뻗는 화살표 (None이면 안 그림)
        self.robot_yaw: float | None = None  # 로봇 진행 방향 [deg] — ●에서 뻗는 화살표
        self.show_map = False        # map 원점 M — 처음엔 숨김, m 키로 켬 (로봇 ●과 방향 화살표는 늘 보임)
        self.show_odom = False       # odom 원점 O — 처음엔 숨김, o 키로 켬
        self.goals: list[dict] = []
        self.yaw = 0.0
        self.show_yaw = False       # True면 십자선에서 방향(yaw)선을 뻗어 그림 — 찍는 모드(goal · 위치 지정)
        self.size = (60, 20)        # 마지막으로 그린 (cols, rows) — 키 처리(전체 보기)가 쓴다
        super().__init__(extra_key)  # 지도가 모르는 키(goal 추가 등)를 페이지로 넘긴다
        self._key = None
        self._codes: list[list[int]] = []
        self.cost: CostGrid | None = None   # 겹쳐 볼 코스트맵 (None = 없음)
        self.show_cost = False
        self.hires = True            # 기본 지도를 1/4 블록(▘▝▖▗…)으로 — costmap을 겹칠 때는 색이 정확한 반블록으로 자동 전환
        self._bkey = None
        self._bcells: dict = {}
        self.waypoints: list[tuple[float, float]] = []   # 경유점 (navigate·approach 목표) — 번호가 주황으로 찍힘
        self.plans: list[tuple[list[tuple[float, float]], tuple]] = []   # 겹쳐 그릴 Nav2 경로들 (점 목록, 색) — 끝점은 G·L
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

    def _hires_cells(self, cols: int, rows: int) -> dict:
        """1/4 블록 지도 — 글자 칸 (c, r) → _quad_cell 결과. 가로는 반블록의 2배로 촘촘히, 세로는 풀 칸 1~2개를 합쳐 샘플링. 키·확대가 같으면 재사용."""
        vp, g = self.vp, self.grid
        key = (vp.k, round(vp.cx, 6), round(vp.cy, 6), cols, rows)
        if key == self._bkey:
            return self._bcells
        lv = max(vp.k - 1, 0)
        w, h, raw = g.level(lv)
        size, s2 = g.res * (1 << lv), vp.scale / 2
        xs = [math.floor((vp.cx + (i + 0.5 - cols) * s2 - g.ox) / size) for i in range(cols * 2)]
        rowc = []   # 샘플 줄 하나 = 세로 scale = 풀 칸 1~2개 — 그 칸들을 합쳐서(벽 우선) 가는 벽이 샘플 사이로 빠지지 않게
        for j in range(rows * 2):
            top = (vp.cy + (rows - j) * vp.scale - g.oy) / size
            lo, hi = math.floor(top - vp.scale / size + 1e-9), math.floor(top - 1e-9)
            rowc.append(range(lo, hi + 1))

        def pix(j, x):
            best = OUT
            for y in rowc[j]:
                v = raw[y * w + x] if 0 <= x < w and 0 <= y < h else OUT
                if v != OUT and (best == OUT or v > best):
                    best = v
            return best
        cells = {}
        for r in range(rows):
            for c in range(cols):
                cells[(c, r)] = _quad_cell([((dx, dy), pix(2 * r + dy, xs[2 * c + dx]))
                                            for dy in (0, 1) for dx in (0, 1)])
        self._bkey, self._bcells = key, cells
        return cells

    def render(self, cols: int, rows: int) -> list[str]:
        """rows줄 (각 cols칸). 안 그려지는 칸(지도 밖)은 공백."""
        self.size = (cols, rows)
        sub = rows * 2
        vp = self.vp
        self._follow(cols, sub)
        hires = self.color and not self.show_cost and self.hires
        codes = None if hires else self._base(cols, sub)
        bcells = self._hires_cells(cols, rows) if hires else None
        bdots: dict[tuple[int, int], list] = {}   # 점자 모드의 경로선 — 글자 칸 → [점 비트, 색]
        cost_codes = self._cost_codes
        dots: dict[tuple[int, int], tuple] = {}      # 화면 칸 → 색 (경로 점)
        labels: dict[tuple[int, int], tuple] = {}    # 글자 칸 → (글자, 색)

        def scr(p):
            c, r = vp.to_screen(p[0], p[1], cols, sub)
            return int(math.floor(c)), int(math.floor(r))

        def scrb(p):   # 점자 점 좌표 (가로 2점/칸 · 세로 4점/줄) — 점 하나 = scale/2
            s2 = vp.scale / 2
            return (int(math.floor((p[0] - vp.cx) / s2 + cols)), int(math.floor(rows * 2 - (p[1] - vp.cy) / s2)))

        def draw(a, b, rgb):
            if hires:
                for c, r in _line(scrb(a), scrb(b)):
                    if 0 <= c < cols * 2 and 0 <= r < rows * 4:
                        cell = bdots.setdefault((c // 2, r // 4), [0, rgb])
                        cell[0] |= BRAILLE[(c % 2, r % 4)]
                        cell[1] = rgb
            else:
                for c, r in _line(scr(a), scr(b)):
                    if 0 <= c < cols and 0 <= r < sub:
                        dots[(c, r)] = rgb
        pts = ([self.start] if self.start else []) + [(g['x'], g['y']) for g in self.goals]
        for a, b in zip(pts, pts[1:]):
            draw(a, b, PATH_RGB)

        for pl, rgb in self.plans:   # Nav2 경로 — 목표 줄(노랑) 위에
            for a, b in zip(pl, pl[1:]):
                draw(a, b, rgb)

        # 글자 표시 — 같은 칸에 여럿이 겹치면 우선순위가 높은 쪽이 그 칸, 진 쪽은 바로 옆 빈 칸에 나란히 (번갈아 그릴 애니메이션이 없어서)
        #   우선순위: 로봇 ● > 경로 끝 G·L > 경유점 번호 > goal 번호 > S > O > M. 십자선 칸은 비워 둠 (십자선이 덮으니)
        pend: list[tuple[int, tuple[int, int], str, tuple, tuple | None]] = []   # (우선순위, 칸, 글자, 글자색, 바탕색)

        def mark(p, ch, rgb, prio=50, bg=None):
            c, r = scr(p)
            if 0 <= c < cols and 0 <= r < sub:
                pend.append((prio, (c, r // 2), ch, rgb, bg))
        endbg: dict[tuple[int, int], tuple] = {}   # 경로 끝점 G(전역)·L(지역) — 흰 글자에 경로 색 바탕 (경로선 위에 겹쳐도 보임)
        for pl, rgb in self.plans:
            if pl:
                mark(pl[-1], 'L' if rgb == LOCAL_PLAN_RGB else 'G', (255, 255, 255), 90, rgb)   # G = 전역 경로 끝(목표) · L = 지역 경로 끝(컨트롤러가 앞서 보는 점)
        for pl, rgb in self.plans:   # 전역 경로 사이사이 점 'o' — 1 m마다 (줄여 보면 점이 겹치지 않게 간격을 넓힘). 번호 경유점·끝점보다 낮은 순위
            if rgb == PLAN_RGB and len(pl) > 2:
                step, run, nxt = max(1.0, 6 * vp.scale), 0.0, max(1.0, 6 * vp.scale)
                for a, b in zip(pl, pl[1:-1]):
                    run += math.hypot(b[0] - a[0], b[1] - a[1])
                    if run >= nxt:
                        mark(b, 'o', WAYPOINT_RGB, 20)
                        nxt += step
        for i, p in enumerate(self.waypoints, 1):
            mark(p, str(i % 10), WAYPOINT_RGB, 80)
        if self.start:
            mark(self.start, 'S', START_RGB, 60)
        for i, g in enumerate(self.goals, 1):
            mark((g['x'], g['y']), str(i % 10), GOAL_RGB, 70)
        if self.show_map:
            mark((0.0, 0.0), 'M', MAP_RGB, 40)
        if self.show_odom:
            if self.odom:
                mark(self.odom, 'O', ODOM_RGB, 50)
        robot_cell = None
        if self.robot:
            c, r = scr(self.robot)
            if 0 <= c < cols and 0 <= r < sub:
                robot_cell = (c, r // 2)
        cur = (cols // 2, (sub // 2) // 2)
        if robot_cell:   # 로봇이 십자선 칸이면 아래에서 ⊕ 하나로 — 그 밖은 일반 글자처럼 겹침 처리
            pend.append((100, robot_cell, '●' if self.color else '@', ROBOT_RGB, None))
        taken = {cur}
        for prio, pos, ch, rgb, bg in sorted(pend, key=lambda t: -t[0]):
            if pos == cur and ch in ('●', '@'):
                continue
            for dc, dr in ((0, 0), (1, 0), (-1, 0), (0, -1), (0, 1), (2, 0), (-2, 0)):
                q = (pos[0] + dc, pos[1] + dr)
                if q not in taken and 0 <= q[0] < cols and 0 <= q[1] < rows:
                    taken.add(q)
                    labels[q] = (ch, rgb)
                    if bg:
                        endbg[q] = bg
                    break
        # M·O·로봇의 방향 화살표 — map x축(M은 늘 0°) · odom 프레임 방향 · 로봇 진행 방향. 글자(M·O·●)가 있는 칸은 글자가 우선.
        rays: dict[tuple[int, int], tuple[int, tuple]] = {}   # 글자 칸 → (점자 비트, 색)
        if True:
            sources = [((0.0, 0.0), 0.0, MAP_RGB)] if self.show_map else []
            if self.show_odom and self.odom and self.odom_yaw is not None:
                sources.append((self.odom, self.odom_yaw, ODOM_RGB))
            if self.robot and self.robot_yaw is not None:
                sources.append((self.robot, self.robot_yaw, ROBOT_RGB))
            for p, yaw_deg, rgb in sources:
                pc, pr = scr(p)
                if not (0 <= pc < cols and 0 <= pr < sub):
                    continue
                for dx, dy in _yaw_dots(pc * 2 + 1, (pr // 2) * 4 + 2, yaw_deg, FRAME_LEN, FRAME_GAP, FRAME_HEAD):
                    c, r = dx // 2, dy // 4
                    if 0 <= c < cols and 0 <= r < rows and (c, r) not in labels and (c, r) != cur:
                        bits = rays.get((c, r), (0, rgb))[0] | BRAILLE[(dx % 2, dy % 4)]
                        rays[(c, r)] = (bits, rgb)
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
                lab = labels.get((c, tr))
                if hires:   # 점자 지도 — 글자(M·O·●·+) > 방향선 > 화살표 > 경로선 > 지도 순으로 얹음. 벽이 한 점이라도 있으면 배경은 벽 색
                    bits, fg, bgc, wall = bcells[(c, tr)]
                    over = RGB[OCC] if wall else bgc
                    if lab:
                        f = lab[1]
                        if (c, tr) == cur:
                            over, f = (ROBOT_RGB if (c, tr) == robot_cell else CURSOR_RGB), (255, 255, 255)
                        over = endbg.get((c, tr), over)
                        put(f'\033[1;38;2;{f[0]};{f[1]};{f[2]}m{_bg(over)}', lab[0])
                    elif (c, tr) in braille:
                        put(f'\033[1;38;2;{CURSOR_RGB[0]};{CURSOR_RGB[1]};{CURSOR_RGB[2]}m{_bg(over)}', chr(0x2800 + braille[(c, tr)]))
                    elif (c, tr) in rays:
                        rb, rgb = rays[(c, tr)]
                        put(f'\033[1;38;2;{rgb[0]};{rgb[1]};{rgb[2]}m{_bg(over)}', chr(0x2800 + rb))
                    elif (c, tr) in bdots:
                        db, rgb = bdots[(c, tr)]
                        put(f'\033[1;38;2;{rgb[0]};{rgb[1]};{rgb[2]}m{_bg(over)}', chr(0x2800 + db))
                    elif bits and fg:
                        put(f'\033[22;38;2;{fg[0]};{fg[1]};{fg[2]}m{_bg(bgc)}', QUAD[bits])
                    else:
                        put(f'\033[0m{_bg(bgc)}', ' ')
                    continue
                top, bot = codes[2 * tr][c], codes[2 * tr + 1][c]
                if self.color:
                    tc = dots.get((c, 2 * tr)) or (RGB.get(top) if top != OUT else None)
                    bc = dots.get((c, 2 * tr + 1)) or (RGB.get(bot) if bot != OUT else None)
                    if cost_codes and (c, 2 * tr) not in dots and (c, 2 * tr + 1) not in dots:
                        ct = max(cost_codes[2 * tr][c], cost_codes[2 * tr + 1][c])
                        if ct and top != OCC and bot != OCC:   # 벽은 지도 색 그대로, 빈 칸의 팽창만 덮는다
                            tc = bc = COST_LETHAL if ct == 2 else COST_INFLATED
                    if lab:
                        bg = endbg.get((c, tr)) or _cell_bg(top, bot, tc, bc)
                        f = lab[1]
                        if (c, tr) == cur:   # 십자선 = 분홍 바탕에 흰 +  (로봇과 같은 칸이면 빨간 바탕)
                            bg, f = (ROBOT_RGB if (c, tr) == robot_cell else CURSOR_RGB), (255, 255, 255)
                        put(f'\033[1;38;2;{f[0]};{f[1]};{f[2]}m{_bg(bg)}', lab[0])
                    elif (c, tr) in braille:   # 방향선 — 지도 바탕 위에 분홍 점자
                        bg = _cell_bg(top, bot, tc, bc)
                        put(f'\033[1;38;2;{CURSOR_RGB[0]};{CURSOR_RGB[1]};{CURSOR_RGB[2]}m{_bg(bg)}', chr(0x2800 + braille[(c, tr)]))
                    elif (c, tr) in rays:   # M·O·로봇 방향 화살표 — 각자 색의 점자
                        bits, rgb = rays[(c, tr)]
                        bg = _cell_bg(top, bot, tc, bc)
                        put(f'\033[1;38;2;{rgb[0]};{rgb[1]};{rgb[2]}m{_bg(bg)}', chr(0x2800 + bits))
                    elif tc is None and bc is None:
                        put('\033[0m', ' ')
                    else:
                        if tc is None:   # 윗칸만 지도 밖 — 아래 반블록 ▄ + 터미널 바탕
                            put(f'\033[22;38;2;{bc[0]};{bc[1]};{bc[2]}m\033[49m', '▄')
                        else:
                            put(f'\033[22;38;2;{tc[0]};{tc[1]};{tc[2]}m{_bg(bc)}', '▀')
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
