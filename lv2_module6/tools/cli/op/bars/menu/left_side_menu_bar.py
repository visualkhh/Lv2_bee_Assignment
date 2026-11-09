"""LeftSideMenuBar — 화면 왼쪽 메뉴 막대 (MenuBar 하위 클래스). 폭은 라벨에 맞춘다."""
from __future__ import annotations

from .menu_bar import MenuBar


class LeftSideMenuBar(MenuBar):
    """왼쪽 열 — 최소 24칸, 최대 화면 절반. 화면보다 길면 포커스 줄이 보이게 스크롤(Window가)."""

    def width(self, cols: int, term, lines: list[str]) -> int:
        return max(24, min(cols // 2, max([term.disp_len(l) for l in lines] + [0]) + 1))
