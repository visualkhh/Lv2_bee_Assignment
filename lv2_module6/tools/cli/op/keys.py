"""키 읽기 — cbreak 모드 · 방향키·UTF-8 해석. 창(op/dialog)·Window(op/window)·Ui.poll_key가 같이 쓴다."""
from __future__ import annotations

import os
import select
import sys
import termios


def set_cbreak(fd: int) -> None:
    """tty.setcbreak와 같은 모드지만 대기 중인 입력을 버리지 않는다.

    tty.setcbreak()는 기본값이 TCSAFLUSH라, 메뉴/다이얼로그에 들어갈 때마다
    그 사이에 누른 방향키까지 지워 버린다 (키가 가끔 씹히는 원인).
    """
    attrs = termios.tcgetattr(fd)
    attrs[3] &= ~(termios.ECHO | termios.ICANON)
    attrs[3] |= getattr(termios, 'IEXTEN', 0)
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


class KeyReader:
    """cbreak 터미널에서 키 하나를 읽는다. 방향키·UTF-8도 지원."""

    def __init__(self, term, flush: bool = True):
        self.term = term
        self.flush = flush
        self.fd = None
        self._old = None

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self.fd)
        set_cbreak(self.fd)   # 입력을 버리지 않는 cbreak (tty.setcbreak는 TCSAFLUSH)
        # 이전 화면에서 누른 키가 새 다이얼로그에 새지 않게 (y가 확인창을 미리 넘기는 등).
        # 메뉴는 flush=False — 2초마다 다시 그릴 때마다 비우면 그 사이 누른 키가 사라진다.
        if self.flush:
            try:
                termios.tcflush(self.fd, termios.TCIFLUSH)
            except Exception:
                pass
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._old is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self._old)
            except Exception:
                pass

    def _more(self, timeout: float = 0.08) -> bytes:
        """이미 ESC를 읽은 뒤 시퀀스 나머지 한 바이트 — 없으면 b''."""
        try:
            ready, _, _ = select.select([self.fd], [], [], timeout)
        except (OSError, ValueError):
            return b''
        if not ready:
            return b''
        try:
            return os.read(self.fd, 1)
        except OSError:
            return b''

    @staticmethod
    def _csi(seq: bytes) -> str:
        """ESC [ … 시퀀스 → 키 이름 (수식키가 붙어도 마지막 글자로 판단)."""
        final = seq[-1:]
        if final == b'A':
            return 'up'
        if final == b'B':
            return 'down'
        if final == b'C':
            return 'right'
        if final == b'D':
            return 'left'
        if final == b'H':
            return 'home'
        if final == b'F':
            return 'end'
        if final == b'Z':   # Shift+Tab (CSI Z · 일부 터미널은 CSI 1;2 Z)
            return 'shift+tab'
        if final == b'~':
            code = seq[1:-1].split(b';')[0]
            return {b'1': 'home', b'4': 'end', b'5': 'pgup',
                    b'6': 'pgdn', b'7': 'home', b'8': 'end'}.get(code, 'unknown')
        return 'unknown'

    def pending(self) -> bool:
        """아직 안 읽은 키가 있나 — 기다리지 않음 (키를 꾹 누를 때 밀린 키를 먼저 처리하려고)."""
        if self.fd is None:
            return False
        try:
            return bool(select.select([self.fd], [], [], 0)[0])
        except (OSError, ValueError):
            return False

    def read(self, timeout=0.1):
        """키 하나 → ('enter'|'esc'|'up'|'down'|'left'|'right'|'pgup'|'pgdn'|
        'home'|'end'|'backspace'|'ctrl_u'|'tab'|'shift+tab'|<문자>) · 없으면 None."""
        if self.fd is None:
            return None
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if not ready:
            return None
        try:
            raw = os.read(self.fd, 1)
        except OSError:
            return None
        if not raw:
            return 'eof'
        if raw in (b'\n', b'\r'):
            return 'enter'
        if raw in (b'\x7f', b'\x08'):
            return 'backspace'
        if raw == b'\x15':
            return 'ctrl_u'
        if raw == b'\t':
            return 'tab'
        if raw == b'\x1b':
            # ESC 단독과 방향키(ESC [ A · ESC O A)를 넉넉히 구분한다.
            # 시퀀스는 CSI 종료 바이트(0x40~0x7e)까지 전부 소비 — 남은 '[A'가
            # 다음 키로 읽히거나 화면에 찍히는 일을 막는다.
            nxt = self._more(0.08)
            if not nxt:
                return 'esc'
            if nxt == b'[':
                seq = b'['
                while len(seq) < 8:
                    b = self._more(0.08)
                    if not b:
                        break
                    seq += b
                    if 0x40 <= b[0] <= 0x7E:
                        break
                return self._csi(seq)
            if nxt == b'O':  # application cursor mode (일부 터미널·SSH)
                b = self._more(0.08)
                return {b'A': 'up', b'B': 'down', b'C': 'right', b'D': 'left',
                        b'H': 'home', b'F': 'end'}.get(b, 'unknown')
            return 'unknown'  # Alt+글자 등 — 남은 한 바이트는 버린다
        try:
            n = (raw[0] & 0xF0)
            need = 1 if raw[0] < 0x80 else 2 if n == 0xC0 else 3 if n == 0xE0 else 4 if n == 0xF0 else 1
            buf = raw
            while len(buf) < need:
                buf += os.read(self.fd, 1)
            return buf.decode('utf-8', errors='replace')
        except (OSError, UnicodeDecodeError):
            return None
