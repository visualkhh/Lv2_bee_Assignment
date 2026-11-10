"""Dialog — 모든 창의 최상위 부모. 박스 그리기·크기·지우기·키 루프(run).

규약 (모든 화면 공통):
  메뉴   ↑↓ 이동 · Enter 실행 · 항목 키(숫자·글자) 포커스 · Esc/◀/b 뒤로 · m 메인 · r 새로고침 · q 종료
  확인   y 예 · n/Esc/◀ 아니오 · Enter 기본값
  입력   Enter 확인(검증 실패면 창 안에 사유, 그대로 다시 입력) · Esc/◀ 취소
  결과창 ↑↓/PgUp/PgDn 스크롤 · Enter/Esc/◀ 닫기
"""
from __future__ import annotations

import sys
import time

from ..keys import KeyReader

CONTINUE = object()  # on_key가 이걸 돌려주면 창이 계속 열려 있음 (그 밖의 값 = 결과, 창 닫음)


class Dialog:
    """모달 창 기반 — 박스 그리기·크기·화면에서 지우기·키 루프(run)는 여기서, 내용과 키 처리는 하위 클래스가.

    Page → Menu → 각 화면처럼, 창도 Dialog → InputDialog · KeyDialog · PagerDialog · BusyDialog ·
    TeleopDialog(bus.py) · RouteRvizDialog(pages/ab) 로 상속해서 만든다. 밖에서 내부 줄을 직접 바꾸지 않는다.
      content()    박스 안 줄 목록 (기본: 제목 · 빈 줄 · body)
      on_key(k)    키 하나 → CONTINUE(계속) 또는 결과(창 닫고 run()이 돌려줌)
      on_tick()    키가 있든 없든 매 주기 — ROS 폴링 등 (기본: ui.tick — 상단 상태바)
      fallback()   TTY가 아닐 때(명령줄·파이프) run()의 결과
    """

    read_timeout = 0.1  # 키를 기다리는 한 주기 [s] — 이 주기로 on_tick (키 반응 속도의 한계)
    draw_period = 0.0   # 박스를 이보다 자주 다시 그리지 않음 [s] (0 = 매 주기). 화면 갱신은 값이 바뀌는 속도(사람이 읽는 속도)에 맞춘다

    def __init__(self, ui, title: str, body: list[str] | None = None):
        self.ui = ui
        self.title = title
        self.body = list(body or [])
        self._geom = None  # (left, top, w, h) — 마지막으로 그린 자리 (지우기·뒤 화면 갱신용)
        self._drawn_at = 0.0

    # ---------------------------------------------------------------- 하위 클래스가 정함
    def content(self) -> list[str]:
        return [self.title, ''] + self.body

    def extra_width(self) -> int:
        return 0

    def on_key(self, k):
        return CONTINUE

    def on_tick(self) -> None:
        self.ui.tick(0.01)

    def fallback(self):
        return None

    # ---------------------------------------------------------------- 공통
    def run(self):
        """창을 띄우고 on_key가 결과를 돌려줄 때까지 키 루프. 닫히면 박스를 지운다."""
        if not sys.stdin.isatty():
            return self.fallback()
        with self._modal() as keys:
            while True:
                self.draw()
                k = keys.read(self.read_timeout)
                self.on_tick()
                if k is None:
                    continue
                result = self.on_key(k)
                if result is not CONTINUE:
                    return result

    def _modal(self):
        """키 입력 동안 열려 있고, 닫히면 박스를 지움 (뒤이은 처리 중 창·메뉴에 잔상 안 남게)."""
        import contextlib

        @contextlib.contextmanager
        def _cm():
            try:
                with KeyReader(self.ui.term) as keys:
                    yield keys
            finally:
                if self._geom is not None:
                    self.ui._erase_rect(*self._geom)
                    self._geom = None
                busy = getattr(self.ui, '_busy', None)   # 확인창 밑에 깔린 처리 중 창을 바로 다시 (빈 화면 방지)
                if busy is not None and busy is not self:
                    busy.draw()
        return _cm()

    def _width(self, lines: list[str]) -> int:
        try:
            import shutil
            cols = shutil.get_terminal_size().columns
        except Exception:
            cols, _ = self.ui.term.size()
        content = max([self.ui.term.disp_len(s) for s in lines] + [20]) + self.extra_width() + 4
        return min(max(cols - 4, 20), max(20, content))

    def invalidate(self) -> None:
        """다음 주기에 바로 다시 그리게 한다 (키를 눌렀을 때 등 — draw_period를 기다리지 않음)."""
        self._drawn_at = 0.0

    def draw(self) -> None:
        """content()를 화면 가운데 박스로. 크기·자리가 바뀌면 예전 자리를 지운다. draw_period 안이면 건너뜀."""
        now = time.monotonic()
        if self.draw_period and now - self._drawn_at < self.draw_period:
            return
        self._drawn_at = now
        cols, rows = self.ui.term.size()
        content = self.content()
        w = self._width(content)
        left = max(1, (cols - w) // 2 + 1)
        hbox = len(content) + 1
        top = max(1, (rows - hbox - 1) // 2 + 1)
        if self._geom is not None and (left, top, w, hbox) != self._geom:
            self.ui._erase_rect(*self._geom)
        self._geom = (left, top, w, hbox)
        self.ui.console.write(f'\033[{top};{left}H┌' + '─' * (w - 2) + '┐')
        for i, s in enumerate(content):
            self.ui.console.write(f'\033[{top + 1 + i};{left}H│' + self.ui.term.fit(s, w - 2) + '│')
        self.ui.console.write(f'\033[{top + hbox};{left}H└' + '─' * (w - 2) + '┘')
        self.ui.console.flush()
