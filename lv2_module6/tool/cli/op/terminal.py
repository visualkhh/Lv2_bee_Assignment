"""터미널 저수준 — 크기·색·표시 너비. ROS·메뉴를 모르는 순수 클래스."""
from __future__ import annotations

import os
import re
import shutil
import sys
import unicodedata


RESET = '\033[0m'
BOLD = '\033[1m'
DIM = '\033[2m'
REV = '\033[7m'
RED = '\033[31m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
CYAN = '\033[36m'


class Terminal:
    """터미널 입출력 원자 연산. color=False면(NO_COLOR) 색 없이."""

    _ANSI_RE = re.compile(r'\033\[[0-9;]*m')

    def __init__(self, color: bool | None = None):
        if color is None:
            color = sys.stdout.isatty() and os.environ.get('NO_COLOR') is None
        self.color = color
        self.out = sys.stdout  # 화면 지우기용 — Ui가 console로 교체 (capture에 안 잡히게)

    def paint(self, text: str, color: str) -> str:
        return f'{color}{text}{RESET}' if self.color else text

    def clear(self) -> None:
        if self.out.isatty():
            print('\033[2J\033[H', end='', flush=True, file=self.out)

    @classmethod
    def char_cols(cls, ch: str) -> int:
        # 터미널 표시 너비: 한글·한자(W/F)는 2칸, 나머지는 1칸
        return 2 if unicodedata.east_asian_width(ch) in ('W', 'F') else 1

    @classmethod
    def disp_len(cls, s: str) -> int:
        return sum(cls.char_cols(ch) for ch in cls._ANSI_RE.sub('', s))

    @classmethod
    def fit(cls, s: str, w: int) -> str:
        """표시 너비 기준 잘라내기/채우기 (ANSI 색 코드는 길이에서 뺌)."""
        w = max(1, w)
        if cls.disp_len(s) <= w:
            return s + ' ' * (w - cls.disp_len(s)) + RESET
        out: list[str] = []
        vis = 0
        pos = 0
        done = False
        for m in cls._ANSI_RE.finditer(s):
            if done:
                break
            for ch in s[pos:m.start()]:
                if vis + cls.char_cols(ch) > w - 1:
                    done = True
                    break
                out.append(ch)
                vis += cls.char_cols(ch)
            if done:
                break
            out.append(m.group())
            pos = m.end()
        if not done:
            for ch in s[pos:]:
                if vis + cls.char_cols(ch) > w - 1:
                    break
                out.append(ch)
                vis += cls.char_cols(ch)
        out.append('…')
        vis += 1
        out.append(RESET)
        return ''.join(out) + ' ' * max(0, w - vis)

    @staticmethod
    def size() -> tuple[int, int]:
        try:
            size = shutil.get_terminal_size()
            return max(60, size.columns), max(16, size.lines)
        except Exception:
            return 80, 24
