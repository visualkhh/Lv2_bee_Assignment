"""단일 키 입력·다이얼로그 코어.

Ui.select/modal_input/modal_key/pager가 모두 이 구현을 공유한다.
규약 (모든 화면 공통):
  메뉴   ↑↓ 이동 · Enter 실행 · 항목 키(숫자·글자) 포커스 · Esc/◀/b 뒤로 · m 메인 · r 새로고침 · q 종료
  확인   y 예 · n/Esc/◀ 아니오 · Enter 기본값
  입력   Enter 확인(검증 실패면 창 안에 사유, 그대로 다시 입력) · Esc/◀ 취소
  결과창 ↑↓/PgUp/PgDn 스크롤 · Enter/Esc/◀ 닫기
"""
from __future__ import annotations

import os
import select
import sys
import termios
import time
import tty

from .terminal import DIM, RED
from .defs import BACK, MAIN, MENU_REFRESH_S, QUIT, STAY, TIMEOUT

# 메뉴에서 항목 키로 못 쓰는 글자 (내비게이션 전용)
NAV_KEYS = ('b', 'm', 'q', 'r')


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
        tty.setcbreak(self.fd)
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

    def read(self, timeout=0.1):
        """키 하나 → ('enter'|'esc'|'up'|'down'|'left'|'right'|'pgup'|'pgdn'|
        'home'|'end'|'backspace'|'ctrl_u'|<문자>) · 없으면 None."""
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
        if raw == b'\x1b':
            seq = b''
            for _ in range(3):
                r, _, _ = select.select([self.fd], [], [], 0.04)
                if not r:
                    break
                try:
                    seq += os.read(self.fd, 1)
                except OSError:
                    break
                if seq in (b'[A', b'[B', b'[C', b'[D', b'[H', b'[F') or seq.endswith(b'~'):
                    break
            mapping = {
                b'[A': 'up', b'[B': 'down', b'[C': 'right', b'[D': 'left',
                b'[H': 'home', b'[F': 'end',
                b'[5~': 'pgup', b'[6~': 'pgdn',
            }
            if seq:
                return mapping.get(seq, 'unknown')
            return 'esc'
        try:
            n = (raw[0] & 0xF0)
            need = 1 if raw[0] < 0x80 else 2 if n == 0xC0 else 3 if n == 0xE0 else 4 if n == 0xF0 else 1
            buf = raw
            while len(buf) < need:
                buf += os.read(self.fd, 1)
            return buf.decode('utf-8', errors='replace')
        except (OSError, UnicodeDecodeError):
            return None


class Dialog:
    """박스 하나 + KeyReader로 처리하는 모달. 텍스트 흐름은 Ui.capture가 가로채도 출력은 console로."""

    def __init__(self, ui, title: str, body: list[str]):
        self.ui = ui
        self.title = title
        self.body = list(body)
        self.lines = [title, ''] + self.body
        self._geom = None

    def _width(self, extra: int = 0):
        try:
            import shutil
            cols = shutil.get_terminal_size().columns
        except Exception:
            cols, _ = self.ui.term.size()
        content = max([self.ui.term.disp_len(s) for s in self.lines] + [20]) + extra + 4
        return min(max(cols - 4, 20), max(20, content))

    def draw(self, mode: str, buf: str = '', prompt: str = '', off: int = 0,
             h: int = 0, error: str = '') -> None:
        cols, rows = self.ui.term.size()
        extra = self.ui.term.disp_len(prompt) + 2 if mode == 'input' else 0
        w = self._width(extra)
        lines = self.lines
        if mode == 'pager':
            vis = self.body[off:off + h]
            pos = (f'{off + 1}-{min(off + h, len(self.body))}/{len(self.body)} · '
                   if len(self.body) > h else '')
            content = [self.title, ''] + vis + [self.ui.term.paint(
                f'{pos}↑↓ 스크롤 · Enter/Esc 닫기', DIM)]
        elif mode == 'input':
            content = lines + [prompt + ' ' + buf + '▏',
                               self.ui.term.paint(error, RED) if error else '',
                               self.ui.term.paint('Enter 확인 · Esc 취소', DIM)]
        else:
            content = lines
        left = max(1, (cols - w) // 2 + 1)
        hbox = len(content) + 1
        top = max(1, (rows - hbox - 1) // 2 + 1)
        if self._geom is not None:
            old_l, old_t, old_w, old_h = self._geom
            if (left, top, w, hbox) != (old_l, old_t, old_w, old_h):
                self.ui._erase_rect(old_l, old_t, old_w, old_h)
        self._geom = (left, top, w, hbox)
        self.ui.console.write(f'\033[{top};{left}H┌' + '─' * (w - 2) + '┐')
        for i, s in enumerate(content):
            self.ui.console.write(f'\033[{top + 1 + i};{left}H│' + self.ui.term.fit(s, w - 2) + '│')
        self.ui.console.write(f'\033[{top + hbox};{left}H└' + '─' * (w - 2) + '┘')
        self.ui.console.flush()

    def input(self, prompt: str, default: str = '', validate=None) -> str | None:
        """한 줄 입력 — Enter 확인(빈칸=default) · Esc/◀ 취소(None).

        validate(text) → 거부 사유 문자열(None=통과). 거부면 창을 닫지 않고 사유를 보여준다.
        """
        if not sys.stdin.isatty():
            try:
                raw = input(f'{prompt} ').strip()
                return raw if raw else default
            except EOFError:
                return None
        buf, error = '', ''
        with KeyReader(self.ui.term) as keys:
            while True:
                self.draw('input', buf, prompt, error=error)
                k = keys.read(0.1)
                self.ui._poll(0.01)
                if k is None:
                    continue
                if k == 'enter':
                    value = buf if buf else default
                    error = (validate(value) or '') if validate else ''
                    if not error:
                        return value
                    continue
                if k in ('esc', 'left'):
                    return None
                error = ''
                if k == 'backspace':
                    buf = buf[:-1]
                elif k == 'ctrl_u':
                    buf = ''
                elif k == 'eof':
                    return None
                elif isinstance(k, str) and len(k) == 1 and k.isprintable():
                    buf += k

    def key(self) -> str | None:
        """단일 키 확인 — Enter='enter' · Esc/◀=None · 문자는 그대로."""
        if not sys.stdin.isatty():
            try:
                raw = input(f'{self.title} (Enter/b/m/q): ').strip().lower()
                return {'': 'enter', 'b': 'b', 'm': 'm', 'q': 'q'}.get(raw, raw or 'enter')
            except EOFError:
                return None
        with KeyReader(self.ui.term) as keys:
            while True:
                self.draw('key')
                k = keys.read(0.1)
                self.ui._poll(0.01)
                if k is None:
                    continue
                if k == 'enter':
                    return 'enter'
                if k in ('esc', 'left'):
                    return None
                if k == 'eof':
                    return None
                if isinstance(k, str) and len(k) == 1 and k.isprintable():
                    return k.lower()

    def pager(self) -> str:
        """결과 페이저 — ↑↓ 스크롤 · Enter/Esc/◀ 닫기(메뉴·종료 이동 없음)."""
        import shutil
        lines = self.body or ['(출력 없음)']
        if not sys.stdin.isatty():
            self.ui.console.write('\n'.join(lines) + '\n')
            self.ui.console.flush()
            return STAY
        off = 0
        try:
            _, rows = shutil.get_terminal_size()
        except Exception:
            rows = 24
        h = max(5, min(20, rows - 10))
        with KeyReader(self.ui.term) as keys:
            while True:
                off = max(0, min(off, max(0, len(lines) - h)))
                self.draw('pager', off=off, h=h)
                k = keys.read(0.1)
                self.ui._poll(0.01)
                if k is None:
                    continue
                if k in ('enter', 'esc', 'left', 'eof'):
                    return STAY
                if k == 'up':
                    off = max(0, off - 1)
                elif k == 'down':
                    off = min(max(0, len(lines) - h), off + 1)
                elif k == 'pgup':
                    off = max(0, off - h)
                elif k == 'pgdn':
                    off = min(max(0, len(lines) - h), off + h)
                elif k == 'home':
                    off = 0
                elif k == 'end':
                    off = max(0, len(lines) - h)


def run_select(ui, title: str, items: list[tuple[str, str]], center: list[str],
               back: bool = True, main: bool = True,
               timeout: float | None = None) -> str:
    """메뉴 선택 — 규약은 모듈 docstring. 반환: 항목 key | BACK | MAIN | QUIT | TIMEOUT."""
    if not items:
        return BACK
    if timeout is None:
        timeout = MENU_REFRESH_S
    idx = getattr(ui, '_sel_idx', 0) % len(items)
    if not sys.stdin.isatty():
        return items[0][0]
    dbuf = ''
    ddeadline = 0.0
    with KeyReader(ui.term, flush=False) as keys:
        end = time.monotonic() + timeout
        while True:
            left = []
            for i, (key, label) in enumerate(items):
                row = f"  {'❯' if i == idx else ' '} {key}) {label}"
                left.append(ui.term.paint(row, '\033[7m\033[36m\033[1m') if i == idx else row)
            ui.draw(title, left, center, back=back, main=main, poll=False)
            ui._sel_idx = idx
            while True:
                remain = end - time.monotonic()
                if remain <= 0:
                    return TIMEOUT
                k = keys.read(min(0.1, remain))
                ui._poll(0.02)
                if k is None:
                    continue
                if k == 'enter':
                    return items[idx][0]
                if k in ('q', 'Q', 'eof'):
                    return QUIT
                if k in ('b', 'B', 'esc', 'left'):
                    return BACK if back else TIMEOUT
                if k in ('m', 'M'):
                    return MAIN if main else TIMEOUT
                if k in ('r', 'R'):
                    return TIMEOUT
                if k in ('up', 'down'):
                    idx = (idx - 1 if k == 'up' else idx + 1) % len(items)
                    break
                if isinstance(k, str) and len(k) == 1 and k.isdigit():
                    now = time.monotonic()
                    if now - ddeadline > 1.0:
                        dbuf = ''
                    dbuf += k
                    ddeadline = now
                    exact = [i for i, (key, _) in enumerate(items) if key == dbuf]
                    longer = any(key.startswith(dbuf) and key != dbuf for key, _ in items)
                    if exact and not longer:
                        idx = exact[0]
                        dbuf = ''
                        break
                    if exact:
                        idx = exact[0]
                        break
                    dbuf = ''
                elif isinstance(k, str) and len(k) == 1 and k.lower() not in NAV_KEYS:
                    dbuf = ''
                    hit = [i for i, (key, _) in enumerate(items) if key == k.lower()]
                    if hit:
                        idx = hit[0]
                        break
                else:
                    dbuf = ''
