"""Viewport — 지도의 어느 부분을 얼마나 확대해서 보여 줄지 (중심 · 확대 단계). 화면 한 칸(subcell) = scale [m]. 중심 = 화면 한가운데."""
from __future__ import annotations

K_MIN, K_MAX = -2, 12   # scale = res × 2^k — -2 = 지도 한 칸을 4×4로 키움 · 12 = 4096칸을 한 칸에


class Viewport:
    def __init__(self, grid):
        self.grid = grid
        self.k = 0
        self.cx = grid.ox + grid.w * grid.res / 2
        self.cy = grid.oy + grid.h * grid.res / 2

    @property
    def scale(self) -> float:
        return self.grid.res * (2.0 ** self.k)

    def fits(self, cols: int, subrows: int) -> bool:
        """지금 확대에서 지도 전체가 화면에 다 들어오나."""
        g, s = self.grid, self.scale
        return g.w * g.res / s <= cols and g.h * g.res / s <= subrows

    def fit(self, cols: int, subrows: int) -> None:
        """지도 전체가 화면에 들어오는 가장 큰 확대 · 중심 = 지도 가운데."""
        self.k = K_MAX
        for k in range(K_MIN, K_MAX + 1):
            self.k = k
            if self.fits(cols, subrows):
                break
        self.center_on_map()
        self.snap()

    def center_on_map(self) -> None:
        g = self.grid
        self.cx, self.cy = g.ox + g.w * g.res / 2, g.oy + g.h * g.res / 2

    def to_screen(self, wx: float, wy: float, cols: int, subrows: int) -> tuple[float, float]:
        s = self.scale
        return (wx - self.cx) / s + cols / 2, subrows / 2 - (wy - self.cy) / s

    def to_world(self, col: float, sub: float, cols: int, subrows: int) -> tuple[float, float]:
        s = self.scale
        return self.cx + (col - cols / 2) * s, self.cy + (subrows / 2 - sub) * s

    def zoom(self, delta: int) -> bool:
        """확대(+)/축소(-) — 중심(화면 한가운데)은 그대로. 한계면 False."""
        k = max(K_MIN, min(K_MAX, self.k - delta))
        if k == self.k:
            return False
        self.k = k
        self.snap()
        return True

    def pan(self, dx: int, dy: int) -> None:
        """화면 한 칸 × (dx, dy)만큼 중심을 옮김 (dy > 0 = 위쪽)."""
        self.cx += dx * self.scale
        self.cy += dy * self.scale
        self.snap()

    def snap(self) -> None:
        """중심을 화면 한 칸(scale)에 맞춘다.

        로봇 따라가기로 중심이 로봇의 소수 자세를 그대로 따라가면, 매 프레임
        지도 샘플링 경계가 반 칸씩 밀려 벽이 지글거린다. 화면 격자에 스냅하면
        로봇 점만 움직이고 지도는 한 칸 단위로만 이동한다.
        """
        s = self.scale
        if s <= 0.0:
            return
        self.cx = round(round(self.cx / s) * s, 9)
        self.cy = round(round(self.cy / s) * s, 9)
