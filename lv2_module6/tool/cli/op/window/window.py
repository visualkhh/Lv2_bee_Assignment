"""Window — 화면 최상위. TopStateBar · LeftSideMenuBar · 중앙 · 힌트를 조합해 그리고, 키 루프 하나로 포커스·activate를 돌린다.

키 규약 (모든 화면 공통):
  메뉴     ↑↓ 이동 · Enter 실행 · 항목 키(숫자·글자) 포커스 · Esc/◀/b 뒤로 · m 메인 · r 새로고침 · q 종료
  상태바   Tab 들어가기 · ←→ 칸 이동 · Enter 열기 · Tab/Esc 메뉴로  (focusable 칸이 있을 때만)
  확인     y 예 · n/Esc/◀ 아니오 · Enter 기본값
  입력     Enter 확인(검증 실패면 창 안에 사유, 그대로 다시 입력) · Esc/◀ 취소
  결과창   ↑↓/PgUp/PgDn 스크롤 · Enter/Esc/◀ 닫기

신호 (connect/emit) — Window는 ROS·화면 목록을 모른다, 앱(OperatorTool)이 연결한다:
  'tick'      (seconds)  ROS 콜백 돌리기 + 재연결 판단 — 메뉴 대기 중·창이 떠 있는 동안 계속
  'navigate'  (name)     이름으로 화면 열기 → page.show() 결과 (예: DiagState → 'diagnostics')
"""
from __future__ import annotations

import sys
import time

from ..defs import BACK, MAIN, MENU_REFRESH_S, QUIT, STAY, TIMEOUT
from ..keys import KeyReader
from ..terminal import BOLD, RESET
from ..menu_bar import ChoiceItem, LeftSideMenuBar
from ..state_bar import TopStateBar


class Window:
    def __init__(self, ui):
        self.ui = ui
        self.term = ui.term
        self.console = ui.console
        self.top_bar = TopStateBar(ui)
        self.menu_bar = LeftSideMenuBar()
        self.focus = self.menu_bar          # 지금 키를 받는 부품 (menu_bar | top_bar)
        self._signals: dict[str, list] = {}
        self.title: str | None = None
        self.center_fn = None               # 창이 떠 있는 동안 중앙 실시간 갱신용
        self.back = self.main = True
        self._frame_size = None
        self._top_at = 0.0
        self._once_idx: dict[str, int] = {}  # select(once) 화면별 포커스 — 2초 갱신마다 첫 줄로 튀지 않게

    # ------------------------------------------------------------- 신호
    def connect(self, signal: str, fn) -> None:
        self._signals.setdefault(signal, []).append(fn)

    def emit(self, signal: str, *args) -> list:
        return [fn(*args) for fn in self._signals.get(signal, [])]

    def navigate(self, name: str):
        """화면 이동 신호 — 받은 쪽의 첫 결과(page.show() 반환), 받을 쪽이 없으면 STAY."""
        res = self.emit('navigate', name)
        return res[0] if res else STAY

    # ------------------------------------------------------------- 실행
    def perform(self, label: str, fn, busy: bool = True):
        """고른 동작 하나 실행 — 누르면 바로 '처리 중' 창 · 실행 중 print는 결과창(pager)으로.
        실행 뒤 이 Window의 메뉴 막대·중앙·포커스를 되돌린다 (하위 화면이 바꿔 놓으므로)."""
        saved = self._save()
        if busy:
            self.ui.busy(label)
        try:
            with self.ui.capture() as (buf_out, buf_err):
                result = fn()
        finally:
            self.ui._busy = None
            self._restore(saved)
        out = (buf_out.getvalue() + buf_err.getvalue()).strip('\n')
        if out:
            self.ui.pager(label, out)
        return result

    def _save(self) -> tuple:
        return self.menu_bar, self.center_fn, self.focus, self.back, self.main, self.title

    def _restore(self, saved: tuple) -> None:
        self.menu_bar, self.center_fn, self.focus, self.back, self.main, self.title = saved

    def run(self, title: str, items_fn, center_fn, back: bool = True, main: bool = True,
            timeout: float | None = None, once: bool = False):
        """화면 하나의 키 루프. items_fn() = Component(key·label()·activate()) 목록 (보이는 것만), center_fn() = 중앙 줄.

        once=False (Menu): 고른 항목을 activate하고 계속 · MAIN/QUIT/BACK(뒤로 키)만 돌려준다.
        once=True (select): 고른 항목의 activate() 값을 그대로 돌려준다 · 갱신 시간이 지나면 TIMEOUT.
        """
        timeout = MENU_REFRESH_S if timeout is None else timeout
        bar = LeftSideMenuBar()
        if once:
            bar.idx = self._once_idx.get(title, 0)
        self.menu_bar, self.focus, self.center_fn = bar, bar, center_fn
        self.back, self.main = back, main
        tty = sys.stdin.isatty()
        while True:
            self.emit('tick', 0.2)
            items = items_fn()
            if not items:
                return BACK if once else STAY
            bar.set_items(items)
            if not tty:   # 파이프 — 첫 항목
                result = items[0].activate()
                if once or result in (MAIN, QUIT):
                    return result
                continue
            k = self._wait_key(title, center_fn, timeout)
            if once:
                self._once_idx[title] = bar.idx
            if k is None:            # 갱신 시간 — 항목·중앙 다시 계산
                if once:
                    return TIMEOUT
                continue
            if k == 'enter' and self.focus is self.top_bar:   # 상태 칸 activate (예: 진단 화면)
                state = self.top_bar.focused_state
                result = self.perform(state.text(), self.top_bar.activate, busy=False)
                if result in (MAIN, QUIT):
                    return result
                continue
            if k == 'enter':   # 메뉴 항목 activate — MenuItem은 perform으로, ChoiceItem은 key를 돌려줌
                result = bar.focused_item.activate()
                if once or result in (MAIN, QUIT):
                    return result
                continue
            if k in ('q', 'Q', 'eof'):
                return QUIT
            if k in ('m', 'M') and main:
                return MAIN
            if k in ('b', 'B', 'esc', 'left') and back:
                return BACK
            # r · 뒤로/메인이 없는 화면의 그 키 → 다시 그리기만

    def _wait_key(self, title: str, center_fn, timeout: float):
        """그리기 → 키 대기. 포커스 이동 키는 여기서 처리하고 다시 그린다.
        반환: 처리 못 한 키(enter·q·b·m·r…) | None(timeout 동안 아무것도 안 눌림)."""
        center = center_fn()
        with KeyReader(self.term, flush=False) as keys:
            while True:
                # 키를 하나 처리할 때마다 갱신 시계를 새로 시작 — 리드로우 직후 키가 씹히는 느낌을 막는다.
                end = time.monotonic() + timeout
                self.draw(title, center)
                while True:
                    remain = end - time.monotonic()
                    if remain <= 0:
                        return None
                    k = keys.read(min(0.1, remain))
                    self.emit('tick', 0.02)
                    if k is None:
                        continue
                    if k == 'tab' and self.top_bar.focusable:
                        self.focus = self.top_bar if self.focus is self.menu_bar else self.menu_bar
                        break
                    if self.focus is self.top_bar:
                        if k == 'esc':
                            self.focus = self.menu_bar
                            break
                        if self.top_bar.on_key(k):
                            break
                        if k in ('enter', 'q', 'Q', 'eof'):
                            return k
                        continue   # 상태바 포커스 중 다른 키는 무시 (뒤로·메인은 Tab/Esc로 메뉴 돌아간 뒤)
                    if self.menu_bar.on_key(k):
                        break
                    if k in ('enter', 'q', 'Q', 'eof', 'b', 'B', 'esc', 'left', 'm', 'M', 'r', 'R'):
                        return k

    def select(self, title: str, items: list[tuple[str, str]], center: list[str],
               back: bool = True, main: bool = True, timeout: float | None = None) -> str:
        """(key, label) 고르기 — 고른 key | BACK | MAIN | QUIT | TIMEOUT. 키 규약은 모듈 docstring."""
        if not items:
            return BACK
        choices = [ChoiceItem(k, t) for k, t in items]
        saved = self._save()
        try:
            return self.run(title, lambda: choices, lambda: center, back, main, timeout, once=True)
        finally:
            self._restore(saved)

    # ------------------------------------------------------------- 그리기
    def narrow(self, cols: int, rows: int) -> bool:
        return cols < 70 or rows < 18

    def top_lines(self, title: str, width: int = 64) -> list[str]:
        """상단 고정 영역: 타이틀 3줄 + 상태바(TopStateBar, 보통 4줄) + 구분선 1줄."""
        bold = BOLD if self.term.color else ''
        reset = RESET if self.term.color else ''
        return ([f'{bold}{"=" * width}{reset}',
                 f'{bold}  BeeBot Operator Console  ·  {title}{reset}',
                 f'{bold}{"=" * width}{reset}']
                + self.top_bar.lines(focused=self.focus is self.top_bar) + ['-' * width])

    def hints_line(self) -> str:
        """하단 고정 키 안내 — 포커스된 막대에 맞춰 (최상위면 뒤로·메인 없음)."""
        if self.focus is self.top_bar:
            return '  ' + ' · '.join(['←→ 칸 이동', 'Enter 열기', 'Tab/Esc 메뉴로', 'q 종료'])
        hints = ['↑↓ 이동', 'Enter 실행', '번호 포커스']
        if self.top_bar.focusable:
            hints.append('Tab 상태바')
        if self.back:
            hints.append('Esc/◀ 뒤로')
        if self.main:
            hints.append('m 메인')
        hints += ['r 새로고침', 'q 종료']
        return '  ' + ' · '.join(hints)

    def _left_width(self, cols: int, left: list[str]) -> int:
        return self.menu_bar.width(cols, self.term, left)

    def draw(self, title: str, center: list[str]) -> None:
        """4분할 화면: 상단 타이틀+상태바 고정 · 왼쪽 메뉴 막대 · 중앙 내용 · 하단 힌트.

        터미널 크기에 맞춰 다시 계산하므로 창을 늘리면 다음 그리기부터 넓어진다. 좁으면(<70) 세로로 쌓는다.
        전체 지우기 없이 커서를 맨 위로 옮겨 제자리에서 다시 그려 깜빡이지 않는다 (처음·크기 변경 때만 전체 지우기).
        왼쪽 메뉴가 화면보다 길면 포커스 줄이 보이게 위아래로 스크롤한다.
        """
        self.title = title
        left = self.menu_bar.lines(focused=self.focus is self.menu_bar)
        cols, rows = self.term.size()
        size = (cols, rows)
        if self.narrow(cols, rows):
            self.term.clear()
            for line in self.top_lines(title, cols) + left + list(center) + ['-' * cols, self.hints_line()]:
                self.console.write(self.term.fit(line, cols) + '\n')
            self.console.flush()
            self._frame_size = size
            return
        if self._frame_size != size:
            self.term.clear()
            self._frame_size = size
        else:
            self.console.write('\033[H')
            self.console.flush()
        top = [self.term.fit(ln, cols) for ln in self.top_lines(title, cols)]
        lw = self._left_width(cols, left)
        cw = cols - lw - 4
        body_h = max(1, rows - 1 - len(top) - 2)
        left_lines = list(left)
        center_lines = list(center)
        if len(left_lines) > body_h:
            left_lines = self._scroll(left_lines, body_h, self.menu_bar.idx)
        if len(center_lines) > body_h:
            center_lines = center_lines[:body_h - 1] + [f' … 외 {len(center_lines) - body_h + 1}줄']
        frame = list(top)
        for i in range(body_h):
            l = self.term.fit(left_lines[i] if i < len(left_lines) else '', lw)
            c = self.term.fit(center_lines[i] if i < len(center_lines) else '', cw)
            frame.append(f' {l}| {c}')
        frame.append('-' * cols)
        frame.append(self.term.fit(self.hints_line(), cols))
        # 항상 rows-1줄로 정규화 → 짧아진 프레임의 남은 줄 흔적이 남지 않는다.
        frame = frame[:rows - 1] + [''] * max(0, rows - 1 - len(frame))
        for ln in frame:
            self.console.write('\033[K' + ln if ln else '\033[K')
            self.console.write('\n')
        self.console.flush()

    @staticmethod
    def _scroll(lines: list[str], height: int, focus: int | None) -> list[str]:
        """height줄 창으로 자르기 — focus 줄이 창 안에 오게, 잘린 쪽은 '↑ 위 N줄'·'↓ 아래 N줄'로 표시."""
        if height < 3:
            return lines[:height]
        f = 0 if focus is None else max(0, min(focus, len(lines) - 1))
        room = height - 2  # 위·아래 표시 줄 자리
        start = max(0, min(f - room // 2, len(lines) - room))
        view = lines[start:start + room]
        above, below = start, len(lines) - start - len(view)
        return ([f'  ↑ 위 {above}줄' if above else ''] + view +
                [f'  ↓ 아래 {below}줄 (↓로 내리기)' if below else ''])

    def _framed(self) -> bool:
        cols, rows = self.term.size()
        return self.title is not None and self._frame_size == (cols, rows) and not self.narrow(cols, rows)

    def refresh_top(self) -> None:
        """창이 떠 있는 동안 상단(타이틀+상태바)만 1초마다 다시 그린다."""
        if not self._framed():
            return
        now = time.monotonic()
        if now - self._top_at < 1.0:
            return
        self._top_at = now
        cols, _ = self.term.size()
        for i, ln in enumerate(self.top_lines(self.title, cols)):
            self.console.write(f'\033[{i + 1};1H\033[K' + self.term.fit(ln, cols))
        self.console.flush()

    def refresh_bar(self) -> None:
        """다이어로그가 떠 있는 동안 상태바 줄만 다시 그린다 (전체를 그리면 다이어로그와 겹쳐 깜빡임)."""
        if not self._framed():
            return
        cols, rows = self.term.size()
        top = [self.term.fit(ln, cols) for ln in self.top_lines(self.title, cols)]
        # 0-based: 0~2 타이틀, 3~ 상태바, 마지막 구분선 → 상태바만
        for i in range(3, len(top) - 1):
            if i + 1 >= rows - 1:  # 다이어로그·힌트 줄과 겹치지 않게
                break
            self.console.write(f'\033[{i + 1};1H\033[K' + top[i])
        self.console.flush()

    def refresh_center(self, geom: tuple | None = None) -> None:
        """다이어로그에 가리지 않은 중앙 줄만 다시 그린다 (바퀴 표 실시간 갱신 등).
        geom = 다이어로그 사각형 (left, top, width, height) — 그 행들은 건드리지 않는다."""
        if self.center_fn is None or not self._framed():
            return
        try:
            center = self.center_fn() or []
        except Exception:
            return
        cols, rows = self.term.size()
        lw = self._left_width(cols, self.menu_bar.lines())
        cw = cols - lw - 4
        top_n = len(self.top_lines(self.title, cols))
        body_h = max(1, rows - 1 - top_n - 2)
        for i in range(min(body_h, len(center))):
            screen_row = top_n + 1 + i
            if geom is not None:
                _, gtop, _, gh = geom
                if gtop <= screen_row < gtop + gh:  # 다이어로그에 가린 줄
                    continue
            self.console.write(f'\033[{screen_row};{lw + 4}H' + self.term.fit(center[i], cw))
        self.console.flush()
