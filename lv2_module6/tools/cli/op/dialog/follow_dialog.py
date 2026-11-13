"""FollowDialog — PagerDialog 하위 클래스."""
from __future__ import annotations

import os
import re
import signal
import subprocess
import threading

from ..defs import STAY
from ..terminal import DIM
from .dialog import CONTINUE
from .pager_dialog import PagerDialog

ANSI = re.compile(r'\x1b\[[0-9;?]*[A-Za-z]')
CLEAR = '\x1b[J'      # 화면 지우기 — mission_plan.py가 2초마다 '\033[H\033[J' + 새 화면을 찍음
MAX_LINES = 2000


class FollowDialog(PagerDialog):
    """명령 출력을 TUI 안 큰 창에서 따라 봄 (tail -f처럼) — 새 창(tmux·터미널)을 못 열 때 open_window가 씀.

    맨 아래면 새 줄을 따라 내려가고, ↑·PgUp으로 올리면 멈춤 · End로 다시 따라감 · q/Esc/Enter 닫기 (명령도 끔).
    출력에 화면 지우기(\\033[J)가 오면 그때까지 줄을 버림 — 2초마다 다시 찍는 화면도 그대로 보임.
    """
    draw_period = 0.5   # 로그가 쏟아져도 0.5초에 한 번만 그림

    def __init__(self, ui, title: str, cmd: list[str], note: str = ''):
        super().__init__(ui, title, [])
        self.body, self.note, self.follow = [], note, True
        _, rows = ui.term.size()
        self.h = max(5, rows - 8)
        self._lines: list[str] = []
        self._lock = threading.Lock()
        self._new = False
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                     text=True, errors='replace', env={**os.environ, 'PYTHONUNBUFFERED': '1'},
                                     start_new_session=True)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        for ln in self.proc.stdout:
            self._add(ln.rstrip('\n'))
        self._add(f'(명령 끝남 · 종료 코드 {self.proc.wait()})')

    def _add(self, ln: str) -> None:
        with self._lock:
            if CLEAR in ln:
                self._lines, ln = [], ln.rsplit(CLEAR, 1)[1]
            self._lines.append(ANSI.sub('', ln))
            del self._lines[:-MAX_LINES]
            self._new = True

    def content(self):
        self._clamp()
        vis = self.body[self.off:self.off + self.h]
        pos = f'{self.off + 1}-{min(self.off + self.h, len(self.body))}/{len(self.body)} · ' if self.body else ''
        foot = f'{pos}{"따라가는 중" if self.follow else "멈춤 (End 따라가기)"} · ↑↓ PgUp/PgDn · q/Esc 닫기'
        return [self.title, self.ui.term.paint(self.note, DIM), ''] + (vis or ['(출력 기다리는 중…)']) \
            + [self.ui.term.paint(foot, DIM)]

    def on_tick(self) -> None:
        super().on_tick()
        with self._lock:
            if not self._new:
                return
            self.body, self._new = list(self._lines), False
        if self.follow:
            self.off = len(self.body)
            self._clamp()

    def on_key(self, k):
        if k in ('q', 'Q', 'enter', 'esc', 'left', 'eof'):
            return STAY
        if k in ('up', 'pgup', 'home'):
            self.follow = False
        super().on_key(k)
        if k in ('down', 'pgdn', 'end'):
            self.follow = self.off >= len(self.body) - self.h
        self.invalidate()
        return CONTINUE

    def run(self):
        try:
            return super().run()
        finally:
            self.stop()

    def fallback(self):
        return STAY

    def stop(self) -> None:
        """명령 끄기 — 자기 세션이라 그룹째 SIGTERM, 1초 안에 안 끝나면 SIGKILL."""
        if self.proc.poll() is not None:
            return
        for sig in (signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(self.proc.pid, sig)
                self.proc.wait(1.0)
                return
            except subprocess.TimeoutExpired:
                continue
            except (ProcessLookupError, PermissionError):
                return
