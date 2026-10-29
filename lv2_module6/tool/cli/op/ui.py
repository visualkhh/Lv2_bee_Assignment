"""화면 전부 — 4분할 프레임 + 방향키 선택 + 모달 입력.

term=Terminal, state=StateStore(상태바용), poll=ROS 콜백 돌리기.
"""
from __future__ import annotations

import math
import sys
import time

from .consts import ARM_JOINTS, DIAG_LEVELS, GRIPPER_JOINT, WHEEL_JOINTS
from .dialogs import Dialog, run_select
from .terminal import BOLD, CYAN, DIM, GREEN, RED, RESET, YELLOW, Terminal


class Ui:
    def __init__(self, term: Terminal, state, poll):
        self.term = term
        # 화면 그리기 전용 — capture()가 sys.stdout을 가로채도 UI는 여기로 쓴다
        self.term.out = self.console = sys.stdout
        self.state = state
        self._poll = poll
        self._sel_idx = 0
        self._frame_size = None

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

    def reset_selection(self) -> None:
        """메뉴 진입 시 포커스 초기화 (항상 첫 항목)."""
        self._sel_idx = 0

    def _erase_rect(self, left: int, top: int, w: int, h: int) -> None:
        """박스 잔상 지우기 (리사이즈 전 위치)."""
        blank = ' ' * w
        for r in range(top, top + h + 1):
            self.console.write(f'\033[{r};{left}H{blank}')
        self.console.flush()

    def modal_input(self, title: str, body: list[str], prompt: str,
                    default: str = '', validate=None) -> str | None:
        """한 줄 입력 — Enter 확인 · Esc/◀ 취소(None). validate는 Dialog.input 참고."""
        return Dialog(self, title, body).input(prompt, default, validate)

    def modal_key(self, title: str, body: list[str]) -> str | None:
        """단일 키 확인 — Enter='enter' · Esc/◀=None · 문자 그대로."""
        return Dialog(self, title, body).key()

    def pager(self, title: str, text: str) -> str:
        """결과 페이저 — ↑↓ 스크롤 · Enter/Esc/◀ 닫기 (메뉴·종료 이동 없음)."""
        return Dialog(self, title, text.splitlines() or ['(출력 없음)']).pager()

    def select(self, title: str, items: list[tuple[str, str]], center: list[str],
                   back: bool = True, main: bool = True,
                   timeout: float | None = None) -> str:
        """메뉴 선택 — 키 규약은 dialogs.py docstring."""
        return run_select(self, title, items, center, back, main, timeout)

    def draw(self, title: str, left: list[str], center: list[str],
                 back: bool = True, main: bool = True, poll: bool = True, focus: int | None = None):
        """4분할 화면: 상단 타이틀+상태바 고정 · 왼쪽 메뉴 · 중앙 내용 · 하단 힌트+입력.

        터미널 크기에 맞춰 다시 계산하므로 창을 늘리면 다음 그리기부터 넓어진다.
        폭이 좁으면(<70) 세로로 쌓는다.
        전체 지우기 없이 커서를 맨 위로 옮겨 제자리에서 다시 그려 깜빡이지 않는다
        (처음·크기 변경 때만 전체 지우기).
        poll=False면 ROS 폴링을 건너뛴다 (방향키 연속 입력 때 0.3초 지연 방지).
        focus: 왼쪽 메뉴에서 고른 줄 — 메뉴가 화면보다 길면 그 줄이 보이게 위아래로 스크롤한다.
        """
        if poll:
            self._poll(0.3)
        cols, rows = self.term.size()
        size = (cols, rows)
        if cols < 70 or rows < 18:
            self.term.clear()
            for line in self.top_lines(title, cols):
                self.console.write(self.term.fit(line, cols) + '\n')
            for line in left:
                self.console.write(self.term.fit(line, cols) + '\n')
            for line in center:
                self.console.write(self.term.fit(line, cols) + '\n')
            self.console.write('-' * cols + '\n')
            self.console.write(self.term.fit(self.hints_line(back=back, main=main), cols) + '\n')
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
        # 왼쪽(메뉴) 폭은 라벨에 맞춘다 — 최소 24, 최대 화면 절반
        lw = max(24, min(cols // 2, max([self.term.disp_len(l) for l in left] + [0]) + 1))
        cw = cols - lw - 4
        body_h = max(1, rows - 1 - len(top) - 2)
        left_lines = list(left)
        center_lines = list(center)
        if len(left_lines) > body_h:
            left_lines = self._scroll(left_lines, body_h, focus)
        if len(center_lines) > body_h:
            center_lines = center_lines[:body_h - 1] + [f' … 외 {len(center_lines) - body_h + 1}줄']
        frame = list(top)
        for i in range(body_h):
            l = self.term.fit(left_lines[i] if i < len(left_lines) else '', lw)
            c = self.term.fit(center_lines[i] if i < len(center_lines) else '', cw)
            frame.append(f' {l}| {c}')
        frame.append('-' * cols)
        frame.append(self.term.fit(self.hints_line(back=back, main=main), cols))
        # 항상 rows-1줄로 정규화 → 짧아진 프레임의 남은 줄 흔적이 남지 않는다.
        if len(frame) > rows - 1:
            frame = frame[:rows - 1]
        else:
            frame += [''] * (rows - 1 - len(frame))
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

    def hints_line(self, back: bool = True, main: bool = True) -> str:
        """하단 고정 키 안내 — 모든 메뉴 공통 (최상위면 뒤로·메인 없음)."""
        hints = ['↑↓ 이동', 'Enter 실행', '번호 포커스']
        if back:
            hints.append('Esc/◀ 뒤로')
        if main:
            hints.append('m 메인')
        hints += ['r 새로고침', 'q 종료']
        return '  ' + ' · '.join(hints)

    def top_lines(self, title: str, width: int = 64) -> list[str]:
        """상단 고정 영역 6줄: 타이틀 3 + 상태바 2 + 구분선 1."""
        bold = BOLD if self.term.color else ''
        reset = RESET if self.term.color else ''
        lines = [
            f'{bold}{"=" * width}{reset}',
            f'{bold}  BeeBot Operator Console  ·  {title}{reset}',
            f'{bold}{"=" * width}{reset}',
        ]
        op = self.operation or '?'
        mode = self.map_mode or '?'
        op_c = {'AUTO': GREEN, 'MANUAL': YELLOW, 'EXPLORE': CYAN, 'TEACH': RED}.get(op, RED)
        mode_c = {'MAPPING': YELLOW, 'LOCALIZATION': GREEN}.get(mode, '')
        op_txt = self.term.paint(op, op_c + BOLD)
        mode_txt = self.term.paint(mode, mode_c) if mode_c else self.term.paint(mode, DIM)
        if self.state.emergency:
            estop = self.term.paint('ON', RED + BOLD)
        else:
            estop = self.term.paint('OFF', GREEN)
        active = 0
        for name in self.state.devices:
            _, state, _ = self.state.device_info(name)
            if state == 'active':
                active += 1
        if self.state.devices:
            dev_txt = self.term.paint(f'{active}/{len(self.state.devices)}',
                                     GREEN if active == len(self.state.devices) else YELLOW)
        else:
            dev_txt = self.term.paint('0/0', DIM)
        # 갱신시각 (그리는 시점 벽시계 + 마지막 수신 뒤 경과)
        now_wall = time.strftime('%m-%d %H:%M:%S', time.localtime())
        if self.state.last_update <= 0:
            upd_txt = self.term.paint('갱신 없음', YELLOW)
        else:
            age = time.monotonic() - self.state.last_update
            if age <= 5.0:
                upd_txt = self.term.paint(f'갱신 {now_wall} ({age:.1f}초 전)', DIM)
            else:
                upd_txt = self.term.paint(f'갱신 {now_wall} ({age:.0f}초 전 — 끊김?)', YELLOW)
        lines.append(f'  조작 {op_txt} | 지도 {mode_txt} | E-STOP {estop} | '
                     f'장치 {dev_txt} | {upd_txt}')
        # 2단 상시 상태바: 인지·임무·발화 (GUI 보드와 같은 내용, 1줄 요약)
        task = (self.state.task_status or '-')
        if len(task) > 38:
            task = task[:37] + '…'
        say = (self.state.say_last or '')
        if len(say) > 22:
            say = say[:21] + '…'
        diag_levels = [lv for _, (lv, _) in self.state.diagnostics.items() if lv >= 1]
        if not self.state.diagnostics:
            diag_txt = self.term.paint('진단 -', DIM)
        elif not diag_levels:
            diag_txt = self.term.paint(f'진단 OK({len(self.state.diagnostics)})', GREEN)
        else:
            worst = max(diag_levels)
            diag_txt = self.term.paint(f'진단 주의 {len(diag_levels)}',
                                      RED + BOLD if worst >= 2 else YELLOW)
        lines.append(f'  인지 {self.state.tracked_count:3d}개 | 임무 {task} | {diag_txt}'
                     + (f' | 발화 {say}' if say else ''))
        if not self.state.values and not self.state.devices and not self.state.joints:
            lines.append(f'  {self.term.paint("상태 없음 — bringup이 떠 있는지 확인", YELLOW)}')
        lines.append('-' * width)
        return lines

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
