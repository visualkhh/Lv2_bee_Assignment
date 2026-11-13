"""상태·진단·RViz 화면."""
from __future__ import annotations

import argparse

import os
import subprocess
import threading
import time
from pathlib import Path

from ...defs import BACK, MAIN, QUIT, STAY
from ...terminal import DIM, GREEN, RED, YELLOW
from ...menus import GoMenu, InfoItem, Menu, MenuItem
from ..page import Page


class ShowAllItem(MenuItem):
    key = '1'

    def label(self):
        return '전체 요약'

    def run(self):
        self.bus.cmd_status(self.bus.status_args())
        return STAY


class ShowOpStateItem(MenuItem):
    key = '2'

    def label(self):
        return '조작·지도 상태'

    def run(self):
        self.bus.show_operation_state()
        return STAY


class ShowJointsItem(MenuItem):
    key = '4'

    def label(self):
        return '관절 상태'

    def run(self):
        self.bus.show_joints()
        return STAY


class ShowTopicsItem(MenuItem):
    key = '6'

    def label(self):
        return '토픽 (/state/topics)'

    def run(self):
        self.bus.cmd_status(self.bus.status_args(topics=True))
        return STAY


class StatusMenu(Menu):
    title = '상태'

    def items(self):
        return [ShowAllItem(self.app), ShowOpStateItem(self.app),
                GoMenu(self.app, '3', '장치 상태 / 상세', self.app.screen('2')),
                ShowJointsItem(self.app),
                GoMenu(self.app, '5', '진단 (항목별 상세)', DiagnosticsMenu(self.app)),
                ShowTopicsItem(self.app),
                GoMenu(self.app, '7', '부하 (노드별 CPU·메모리)', LoadPage(self.app))]

    def center(self):
        st = self.state
        return [
            '[조작·지도]',
            f' 조작: {(st.operation or "-")}',
            f' 지도: {(st.map_mode or "-")}',
            f' 이름: {st.values.get("map_name") or "-"}',
            f' E-STOP: {"ON" if st.emergency else "OFF"}',
            '',
        ] + self.ui.center_summary()


class DiagItem(MenuItem):
    """진단 항목 하나 — Enter가 상세 화면."""
    shows_busy = False  # 하위 화면을 연다

    def __init__(self, app, key: str, name: str, level: int, message: str):
        super().__init__(app)
        self.key = key
        self._name = name
        self._level = level
        self._message = message

    def label(self):
        lv = self._level
        mark = self.term.paint('●', GREEN if lv == 0 else YELLOW if lv == 1 else RED if lv == 2 else DIM)
        return f'{mark} {self.bus.diag_short(self._name)} — {self._message}'

    def run(self):
        return DiagDetailPage(self.app, self._name).show()


class DiagnosticsMenu(Menu):
    """진단 목록 (문제 있는 것 먼저) → 번호를 고르면 그 항목의 key-value 상세."""

    title = '진단'

    def items(self):
        entries = self.bus.diag_entries()
        if not entries:
            return [InfoItem(self.app, '진단 없음 (/diagnostics_agg 대기 중 — bringup 확인)')]
        return [DiagItem(self.app, str(i), name, lv, msg)
                for i, (name, lv, msg) in enumerate(entries, 1)]

    def center(self):
        entries = self.bus.diag_entries()
        bad = sum(1 for _, lv, _ in entries if lv >= 1)
        return ([f'[진단 {len(entries)}개 · 주의 {bad}]', ' Enter = 그 항목 상세 (값 전부, 2초마다 갱신)',
                 ' 레벨: ● OK · ● WARN · ● ERROR · ● STALE(보고 끊김)']
                + self.ui.center_summary())


class DiagDetailPage(Page):
    """진단 항목 하나 — 2초마다 다시 그려 값이 바뀌는 게 보임."""

    def __init__(self, app, name: str):
        super().__init__(app)
        self._name = name
        self.title = f'진단 · {app.bus.diag_short(name)}'

    def show(self) -> str:
        while True:
            choice = self.ui.select(self.title, [('r', '새로고침 (자동 갱신)')],
                                    self.bus.diag_detail_lines(self._name))
            if choice in (BACK, MAIN, QUIT):
                return choice


class LoadPage(Page):
    """노드(프로세스)별 CPU %·RSS — 3초마다 다시 잼. CPU %는 코어 하나 = 100 (코어 수 × 100이 전부)."""
    title = '부하 · 노드별'

    def show(self) -> str:
        from ... import fielddebug as fd
        fd.proc_usage()   # 첫 호출은 기준값만 잡는다
        while True:
            choice = self.ui.select(self.title, [('r', '새로고침 (자동 갱신)')], self.lines(fd))
            if choice in (BACK, MAIN, QUIT):
                return choice

    def lines(self, fd) -> list[str]:
        p = self.term.paint
        lr = fd.load_ratio()
        rows = fd.proc_usage()
        out = []
        if lr:
            load, cores = lr
            out.append('[부하] ' + p(f'{load:.1f}/{cores}코어', RED if load / cores > fd.LOAD_RED else GREEN)
                       + f' · CPU 합 {sum(r[0] for r in rows):.0f}% / {cores * 100}%')
        avail, swap = _mem_available_mb(), _swap_used_mb()
        if avail is not None:
            out.append('[메모리] ' + p(f'여유 {avail} MB', RED if avail < 500 else YELLOW if avail < 1000 else GREEN)
                       + (f' · 스왑 {swap} MB' if swap is not None else ''))
        out.append(' load = CPU를 쓰거나 기다리는 스레드 수(1분 평균) · 코어 수보다 크면 줄 서서 기다림')
        if not rows:
            return out + ['', ' (재는 중 — 3초 뒤 갱신)']
        out += ['', f' {"CPU%":>6} {"MB":>6}  노드/프로세스']
        for pct, rss, name in rows[:25]:
            out.append(p(f' {pct:6.1f} {rss:6d}  {name}', RED if pct >= 80 else YELLOW if pct >= 40 else ''))
        return out


def _swap_used_mb() -> int | None:
    """/proc/meminfo SwapTotal - SwapFree [MB] (없으면 None)."""
    try:
        m = {l.split(':')[0]: int(l.split()[1]) for l in Path('/proc/meminfo').read_text().splitlines()}
        return (m['SwapTotal'] - m['SwapFree']) // 1024
    except (OSError, ValueError, KeyError):
        return None


def _mem_available_mb() -> int | None:
    """/proc/meminfo의 MemAvailable [MB] (없으면 None)."""
    try:
        for line in Path('/proc/meminfo').read_text().splitlines():
            if line.startswith('MemAvailable:'):
                return int(line.split()[1]) // 1024
    except (OSError, ValueError):
        pass
    return None


def _tail(path: Path, n: int = 2) -> str:
    try:
        return ' | '.join(path.read_text(errors='replace').splitlines()[-n:])[:160]
    except OSError:
        return '(로그 없음)'


class RvizPage(Page):
    title = 'RViz'
    config: str | None = None  # None = run-rviz 기본(operator.rviz), 아니면 tools/rviz/<config>.rviz

    def show(self) -> str:
        if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            print('거부: DISPLAY/WAYLAND_DISPLAY가 없습니다 — GUI가 있는 터미널에서 실행하세요.')
            return STAY
        script = Path(__file__).resolve().parents[4] / 'run-rviz'
        if not script.is_file():
            print(f'거부: 실행 파일 없음: {script}')
            return STAY
        if not self.ui.confirm('RViz를 새 창으로 열까요?', default=True,
                               body=['카메라/점구름은 Best Effort, queue 1로 구독합니다.',
                                     f'실행: {script}']):
            return STAY
        log = Path('/tmp/operator_tool_rviz.log')
        try:
            with log.open('ab') as out:
                proc = subprocess.Popen(
                    [str(script)], cwd=str(script.parent),
                    env={**os.environ, **({'RVIZ_CONFIG': self.config} if self.config else {})},
                    stdout=out, stderr=out, start_new_session=True)
        except OSError as exc:
            print(f'RViz 실행 실패: {exc}')
            return STAY
        threading.Thread(target=proc.wait, daemon=True).start()  # 끝난 RViz가 좀비로 남지 않게 거둔다
        print(f'RViz 시작: pid {proc.pid} · 로그 {log}')
        time.sleep(3)  # 메모리가 모자라면 오류 한 줄 없이 곧 죽는다 (실측 2026-11-10) — 말없이 안 뜨지 않게 알림
        if proc.poll() is not None:
            print(f'⚠ RViz가 바로 종료됨 (코드 {proc.returncode}) — 로그 끝: {_tail(log)}')
        avail = _mem_available_mb()
        if avail is not None and avail < 400:
            print(f'⚠ 여유 메모리 {avail} MB — RViz(약 260–320 MB)가 안 뜨거나 곧 죽을 수 있음. 다른 RViz·앱을 닫으세요.')
        return STAY


class MappingRvizPage(RvizPage):
    title = '매핑 전용 RViz'
    config = 'mapping'


class NavRvizPage(RvizPage):
    title = '목표 이동 전용 RViz'
    config = 'nav'


class NvbloxRvizPage(RvizPage):
    title = 'nvblox 전용 RViz'
    config = 'nvblox'


class CuvslamRvizPage(RvizPage):
    title = 'cuVSLAM 전용 RViz'
    config = 'cuvslam'


class BeepNowItem(MenuItem):
    key = '1'

    def label(self):
        return '삑 (기본음 2000 Hz · 150 ms)'

    def run(self):
        self.bus.cmd_beep(None)
        return STAY


class BeepToneItem(MenuItem):
    key = '2'

    def label(self):
        return '높이·길이 지정'

    def run(self):
        raw = self.ui.modal_input('부저 톤', ['Hz 100~8000 · ms 10~2000 (예: 2000 300)'], 'Hz ms>')
        if raw is None or not raw.strip():
            return STAY
        self.bus.cmd_beep(argparse.Namespace(value=raw.split()))
        return STAY


class BeepMenu(Menu):
    title = '비프음'

    def items(self):
        return [BeepNowItem(self.app), BeepToneItem(self.app)]

    def center(self):
        return ['[비프음]', ' /cmd/beep (std_msgs/Int32MultiArray — [] 기본음 · [Hz, ms])',
                ' → BeebotSystemHardware → OpenCR B 줄 · 모드와 무관']
