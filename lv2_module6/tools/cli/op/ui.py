"""Ui — 터미널 모드·창(dialog) 띄우기·print 가로채기. 화면 틀(조합·그리기·키 루프)은 self.window (op/window).

term=Terminal, state=StateStore(상태바용). ROS 폴링·재연결·화면 이동은 window 신호(tick · navigate)로 앱이 연결.
"""
from __future__ import annotations

import atexit
import math
import os
import signal
import sys
import time

from .consts import ARM_JOINTS, DIAG_LEVELS, GRIPPER_JOINT, WHEEL_JOINTS
from .defs import TIMEOUT
from .dialog import BusyDialog, InputDialog, KeyDialog, PagerDialog
from .keys import KeyReader, set_cbreak
from .terminal import CYAN, DIM, GREEN, RED, YELLOW, Terminal
from .window import Window


class Ui:
    def __init__(self, term: Terminal, state):
        self.term = term
        # 화면 그리기 전용 — capture()가 sys.stdout을 가로채도 UI는 여기로 쓴다
        self.term.out = self.console = sys.stdout
        self.state = state
        self._busy = None   # 지금 떠 있는 처리 중 창 (메뉴 항목 실행 동안)
        self._reconnect_status = None  # OperatorTool.reconnect_status — 상태바 '재연결 Ns' 표시
        self.window = Window(self)  # 화면 틀 — 상태바·메뉴 막대·중앙·힌트 조합 · 키 루프 · 신호
        # 화면 아래에서 깜빡이는 실제 커서를 숨긴다 (입력은 다이얼로그의 '▏'로 표시).
        self._cursor_hidden = False
        self._term_fd = None
        self._term_attrs = None
        if self.console.isatty() and sys.stdin.isatty():
            self._enter_key_mode()
            self.term.hide_cursor()
            self._cursor_hidden = True
            atexit.register(self.close)
            self._install_cursor_signals()

    def _enter_key_mode(self) -> None:
        """TUI가 떠 있는 동안 cbreak·에코 없음을 유지한다.

        KeyReader는 다이얼로그/메뉴 선택 동안만 모드를 잡는다. 그 사이(메뉴 재그리기·
        ROS 폴링)에 눌린 키가 조리된 모드로 돌아가 에코되면 화면 맨 아래에 '[A'처럼
        찍히며 커서가 움직인다 — 세션 전체를 no-echo로 고정해 막는다.
        """
        import termios
        try:
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            set_cbreak(fd)
        except Exception:
            return
        self._term_fd, self._term_attrs = fd, old

    def _install_cursor_signals(self) -> None:
        """SIGTERM/SIGHUP(CLI stop 등)에서도 커서를 되살린다 — atexit은 실행되지 않는다."""
        def _restore(signum, _frame):
            self.close()
            try:
                signal.signal(signum, signal.SIG_DFL)
                os.kill(os.getpid(), signum)
            except Exception:
                raise SystemExit(128 + signum)
        for name in ('SIGTERM', 'SIGHUP'):
            sig = getattr(signal, name, None)
            if sig is None:
                continue
            try:
                signal.signal(sig, _restore)
            except (ValueError, OSError):
                pass

    def close(self) -> None:
        """종료·예외 시 실제 커서·터미널 모드 복구 (여러 번 불러도 안전)."""
        if self._term_attrs is not None:
            try:
                import termios
                termios.tcsetattr(self._term_fd, termios.TCSADRAIN, self._term_attrs)
            except Exception:
                pass
            self._term_fd = None
            self._term_attrs = None
        if self._cursor_hidden:
            self.term.show_cursor()
            self._cursor_hidden = False

    @property
    def operation(self) -> str:
        return self.state.operation

    @property
    def map_mode(self) -> str:
        return self.state.map_mode

    def capture(self):
        """with 블록 안의 print()를 가로챈다. 메뉴 실행 결과를 결과창에 넣기용.
        (Ui 자체 그리기는 self.console이라 영향 없음)"""
        import contextlib
        import io

        @contextlib.contextmanager
        def _cap():
            buf_out, buf_err = io.StringIO(), io.StringIO()
            with contextlib.redirect_stdout(buf_out), contextlib.redirect_stderr(buf_err):
                yield buf_out, buf_err

        return _cap()

    def confirm(self, prompt: str, default: bool = False, body: list[str] | None = None) -> bool:
        """예/아니오 — y 예 · n/Esc/◀ 아니오 · Enter 기본값. 경고·맥락은 body에 (확인 전에 보이게)."""
        suffix = 'Y/n' if default else 'y/N'
        lines = list(body or []) + ([''] if body else []) + [
            prompt, self.term.paint(f'[{suffix}]  y 예 · n/Esc 아니오 · Enter 기본', DIM)]
        key = self.modal_key('확인', lines)
        if key == 'enter':
            return default
        return key == 'y'

    def read_float(self, prompt: str, current: float,
                   positive: bool = False) -> tuple[float, bool]:
        """숫자 한 개 — (값, 확정). Enter 빈칸=현재값 유지 · Esc=(current, False)."""
        def check(raw: str) -> str | None:
            if not raw.strip():
                return None
            try:
                value = float(raw)
            except ValueError:
                return f'숫자가 아님: {raw!r}'
            if not math.isfinite(value):
                return '유한한 숫자가 아님'
            if positive and value <= 0.0:
                return '0보다 커야 합니다'
            if not positive and value < 0:
                return '0 이상이어야 합니다'
            return None

        raw = self.modal_input('값 입력', [prompt, f'현재: {current:g} (빈칸 Enter=유지)'],
                               '값>', validate=check)
        if raw is None:
            return current, False
        return (float(raw) if raw.strip() else current), True

    def _erase_rect(self, left: int, top: int, w: int, h: int) -> None:
        """박스 잔상 지우기 (리사이즈 전 위치)."""
        blank = ' ' * w
        for r in range(top, top + h + 1):
            self.console.write(f'\033[{r};{left}H{blank}')
        self.console.flush()

    def modal_input(self, title: str, body: list[str], prompt: str,
                    default: str = '', validate=None) -> str | None:
        """한 줄 입력 — Enter 확인 · Esc/◀ 취소(None). validate는 InputDialog 참고."""
        return InputDialog(self, title, body, prompt, default, validate).run()

    def busy(self, label: str) -> None:
        """누르자마자 '처리 중' 창 — 기다리지 않음 (ROS 대기로 몇 초 걸려도 눌린 걸 알게)."""
        if not self.console.isatty():   # 명령줄(run-cli <명령>)·파이프면 창 없음
            return
        self._busy = BusyDialog(self, label)
        self._busy.draw()

    def step(self, text: str) -> None:
        """처리 중 창의 단계 줄 (예: '1/2 다시 뜨기 기다림') — 여러 단계를 거치는 작업용. 메뉴 밖이면 무시."""
        dlg = self._busy
        if dlg is None:
            return
        dlg.step(self.term.paint(f'▶ {text}', CYAN))

    def progress(self, text: str) -> None:
        """처리 중 창 맨 아래 줄 갱신 (오래 걸리는 단계가 지금 뭘 기다리는지 · 회전 표시). 메뉴 밖이면 무시."""
        dlg = self._busy
        if dlg is None:
            return
        spin = '⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏'[int(time.monotonic() * 4) % 10]
        self.refresh_top()   # 상태바 먼저 → 창이 위에
        dlg.progress(self.term.paint(f'{spin} {text}', DIM))

    def tick(self, seconds: float) -> None:
        """창(결과·확인·입력)이 떠 있는 동안: ROS 폴링 + 상단 상태바 1초마다 갱신 (창 밑 화면이 멈춰 보이지 않게)."""
        self.window.emit('tick', seconds)
        self.window.refresh_top()

    def refresh_top(self) -> None:
        self.window.refresh_top()

    def refresh_bar(self) -> None:
        self.window.refresh_bar()

    def refresh_center(self, geom: tuple | None = None) -> None:
        self.window.refresh_center(geom)

    def select(self, title: str, items: list[tuple[str, str]], center: list[str],
               back: bool = True, main: bool = True, timeout: float | None = None) -> str:
        """(key, label) 고르기 — Window.select (키 규약은 op/window/window.py docstring)."""
        return self.window.select(title, items, center, back, main, timeout)

    def choose(self, title: str, items: list[tuple[str, str]], center: list[str],
               back: bool = True, main: bool = True) -> str:
        """select 를 사람이 고를 때까지 — select 는 갱신 시간(MENU_REFRESH_S)마다 TIMEOUT 을 돌려주므로
        그걸 '취소'로 다루면 화면이 몇 초 뒤 이전 페이지로 돌아간다. 고른 key | BACK | MAIN | QUIT."""
        while True:
            choice = self.select(title, items, center, back, main)
            if choice != TIMEOUT:
                return choice

    def poll_key(self) -> str | None:
        """오래 걸리는 작업 중 눌린 키 하나 — 기다리지 않음 (없으면 None). TTY가 아니면(명령줄·파이프) 늘 None.
        예: A/B 자동 주행 중 s(이 leg 중지)·q(전체 중단)."""
        if not (self.console.isatty() and sys.stdin.isatty()):
            return None
        with KeyReader(self.term, flush=False) as keys:
            return keys.read(0)

    def modal_key(self, title: str, body: list[str]) -> str | None:
        """단일 키 확인 — Enter='enter' · Esc/◀=None · 문자 그대로."""
        return KeyDialog(self, title, body).run()

    def pager(self, title: str, text: str) -> str:
        """결과 페이저 — ↑↓ 스크롤 · Enter/Esc/◀ 닫기 (메뉴·종료 이동 없음)."""
        return PagerDialog(self, title, text.splitlines()).run()

    def center_summary(self) -> list[str]:
        """중앙 공용 요약: 장치·임무·관절·진단 (왼쪽이 메뉴일 때 오른쪽에 보여줌)."""
        lines = ['[상태 요약]']
        if not self.state.devices:
            lines.append(' 장치: - (대기 중)')
        else:
            for name in self.state.devices[:8]:
                kind, state, topics = self.state.device_info(name)
                if state == 'active':
                    mark = self.term.paint('●', GREEN)
                elif state:
                    mark = self.term.paint('○', RED)
                else:
                    mark = self.term.paint('?', YELLOW)
                lines.append(f' {mark} {name} ({kind})')
            if len(self.state.devices) > 8:
                lines.append(f' … 외 {len(self.state.devices) - 8}개')
        lines.append('')
        lines.append(f' 임무: {self.state.task_status or "-"}')
        wheels = [f'{n}={self.state.joints[n]:.2f}' for n in WHEEL_JOINTS
                  if n in self.state.joints]
        arms = [f'{n}={self.state.joints[n]:+.2f}/{math.degrees(self.state.joints[n]):+.0f}°'
                for n in (*ARM_JOINTS, GRIPPER_JOINT) if n in self.state.joints]
        if wheels:
            lines.append(' 바퀴: ' + ' '.join(wheels))
        if arms:
            lines.append(' 관절: ' + ' '.join(arms[:3]))
            if len(arms) > 3:
                lines.append(' ' + ' '.join(arms[3:]))
        if not wheels and not arms:
            lines.append(' 관절: -')
        bad = [(n, lv, m) for n, (lv, m) in sorted(self.state.diagnostics.items()) if lv >= 1][:5]
        if not self.state.diagnostics:
            lines.append(' 진단: -')
        elif not bad:
            lines.append(self.term.paint(f' 진단 OK ({len(self.state.diagnostics)}개)', GREEN))
        else:
            lines.append(f' 진단 주의 {len(bad)}:')
            for n, lv, m in bad:
                lines.append(f'  - {n}: {DIAG_LEVELS.get(lv, lv)} {m}')
        return lines
