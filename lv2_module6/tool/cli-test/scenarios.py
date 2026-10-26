#!/usr/bin/env python3
"""cli-test 시나리오 — 실제 pty 키 입력으로 UI 흐름을 검증한다.

실행: python3 tool/cli-test/scenarios.py [시나리오 이름 ...]
각 시나리오는 가짜 상태를 초기화한 뒤 시작한다.
"""
from __future__ import annotations

import os
import re
import select
import signal
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
CLI = HERE.parent / 'cli'
STATE = HERE / '.fake_state.json'

UP, DOWN, LEFT, RIGHT = b'\x1b[A', b'\x1b[B', b'\x1b[D', b'\x1b[C'
ENTER, ESC, BACKSPACE = b'\r', b'\x1b', b'\x7f'


def plain(data: bytes) -> str:
    text = data.decode('utf-8', errors='replace')
    return re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', text)


class Session:
    def __init__(self, name: str):
        self.name = name
        self.out = bytearray()
        STATE.unlink(missing_ok=True)
        self.pid, self.fd = os.forkpty() if hasattr(os, 'forkpty') else (None, None)
        if self.pid == 0:
            os.chdir(HERE.parent.parent)
            os.execv(sys.executable, [sys.executable, str(HERE / 'cli_test.py'), 'menu'])
        if self.pid is None:
            import pty
            self.pid, self.fd = pty.fork()
            if self.pid == 0:
                os.chdir(HERE.parent.parent)
                os.execv(sys.executable, [sys.executable, str(HERE / 'cli_test.py'), 'menu'])
        self.closed = False
        self.mark = 0
        self.expect('메인 메뉴')  # 첫 화면이 뜬 뒤부터 키를 보낸다

    def _read(self, timeout=0.2) -> bool:
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return False
        try:
            data = os.read(self.fd, 65536)
        except OSError:
            self.closed = True
            return False
        if not data:
            self.closed = True
            return False
        self.out.extend(data)
        return True

    def send(self, data: bytes) -> None:
        while self._read(0.05):  # 앞 화면 출력은 다 받아 두고
            pass
        self.mark = len(self.out)  # expect/absent는 이 키를 보낸 뒤 출력만 본다
        os.write(self.fd, data)

    def expect(self, marker: str, timeout=8.0) -> None:
        target = marker.encode('utf-8')
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if target in self.out[self.mark:]:
                return
            self._read(max(0.0, min(0.2, end - time.monotonic())))
        raise AssertionError(f'[{self.name}] 기대 문자열 없음: {marker!r}')

    def absent(self, marker: str, wait=0.8) -> None:
        end = time.monotonic() + wait
        while time.monotonic() < end:
            self._read(0.1)
        if marker.encode('utf-8') in self.out[self.mark:]:
            raise AssertionError(f'[{self.name}] 나오면 안 되는 문자열: {marker!r}')

    def step(self, data: bytes, marker: str | None = None, wait=0.7) -> None:
        self.send(data)
        if marker is not None:
            self.expect(marker)
        else:
            time.sleep(wait)
            while self._read(0.05):
                pass

    def finish(self, marker='operator_tool 종료', timeout=8.0) -> None:
        self.step(b'q', marker, wait=0) if not self.closed else None
        if not self.closed:
            self.expect(marker, timeout)

    def close(self) -> None:
        if self.closed:
            return
        try:
            os.kill(self.pid, signal.SIGKILL)
        except OSError:
            pass
        self.closed = True
        try:
            os.close(self.fd)
        except OSError:
            pass
        try:
            os.waitpid(self.pid, 0)
        except ChildProcessError:
            pass

    def assert_clean(self) -> None:
        text = plain(bytes(self.out))
        if 'Traceback' in text:
            raise AssertionError(f'[{self.name}] Traceback:\n{text[-2000:]}')


SCENARIOS = []


def scenario(name: str):
    def deco(fn):
        def wrapped():
            s = Session(name)
            try:
                fn(s)
                s.assert_clean()
                print(f'PASS {name}')
            except Exception:
                (HERE / f'.scenario-{name}.bin').write_bytes(bytes(s.out))
                raise
            finally:
                s.close()
        wrapped.__name__ = name
        SCENARIOS.append(wrapped)
        return wrapped
    return deco


@scenario('menu-arrows-esc')
def _(s):
    s.step(DOWN, wait=0.5)
    s.step(DOWN, wait=0.5)
    s.step(ENTER, '조작 모드')
    s.step(ESC, '메인 메뉴')
    s.finish()


@scenario('manual-cancel-no-extra-dialog')
def _(s):
    s.step(b'4', wait=0.4)
    s.step(ENTER, '수동 주행 / 텔레오퍼')
    s.step(DOWN, wait=0.4)
    s.step(ENTER, 'MANUAL 모드로 전환할까요?')
    s.step(ESC, '수동 주행 / 텔레오퍼')
    if '일시정지'.encode() in s.out:
        raise AssertionError('취소 후 일시정지 모달이 뜸')
    s.finish()


@scenario('drive-confirm-execute')
def _(s):
    s.step(b'4', wait=0.4)
    s.step(ENTER, '수동 주행 / 텔레오퍼')
    s.step(DOWN, wait=0.4)
    s.step(ENTER, 'MANUAL 모드로 전환할까요?')
    s.step(b'y', '/cmd/operation_mode')
    s.expect('forward: linear=0.15')
    s.absent('거부: 현재 조작 모드')  # 전환에 동의했으면 거부 문구가 남으면 안 됨
    s.step(ENTER, '수동 주행 / 텔레오퍼')
    s.finish()


@scenario('status-pager-scroll-close')
def _(s):
    s.step(ENTER, '전체 요약')
    s.step(ENTER, 'Enter/Esc 닫기')
    s.step(DOWN, wait=0.4)
    s.step(ESC, '조작·지도 상태')  # 결과창 닫힘 → 상태 메뉴 그대로
    s.step(ESC, '메인 메뉴')
    s.finish()


@scenario('map-mode-course-restart')
def _(s):
    s.step(b'7', wait=0.4)
    s.step(ENTER, '지도')
    s.step(DOWN, wait=0.4)
    s.step(DOWN, wait=0.4)
    s.step(ENTER, 'MAPPING 모드로 전환할까요?')
    s.step(b'y', '재시작할까요?')
    s.expect('COURSE(정지 지도)')  # 이유가 확인창 안에 보여야 함
    s.step(b'y', '/cmd/restart')
    s.step(ESC, '지도 저장')
    s.finish()


@scenario('arm-value-modal-cancel')
def _(s):
    s.step(b'5', wait=0.4)
    s.step(ENTER, '팔')
    s.step(DOWN, wait=0.4)
    s.step(ENTER, 'MANUAL 모드로 전환할까요?')
    s.step(b'y', '관절 선택')
    s.step(ENTER, '값 입력')
    s.step(b'abc', 'abc')
    s.step(ENTER, '숫자가 아님')  # 검증 실패는 창 안에서 — 닫히지 않음
    s.step(ESC, '관절 선택')
    s.step(b'b', '조작 모드 → MANUAL')  # 실제로 바뀐 모드 전환만 결과창에 (거부 문구 없이)
    s.absent('거부')
    s.step(ESC, '관절 델타 이동')
    s.finish()


@scenario('teleop-hud')
def _(s):
    s.step(b'4', wait=0.4)
    s.step(ENTER, '수동 주행 / 텔레오퍼')
    s.step(ENTER, 'MANUAL 모드로 전환할까요?')
    s.step(b'y', 'q/Esc 종료')
    s.step(b'w', '키 w')
    s.step(UP, '키 w ×2')  # 방향키 = w
    s.step(ESC, '텔레오퍼 종료')
    s.step(ENTER, '속도/시간 설정')
    s.finish()


@scenario('nested-back-one-level')
def _(s):
    s.step(ENTER, '전체 요약')
    s.step(b'3', wait=0.3)
    s.step(ENTER, '장치 상세는 Enter')
    s.step(ESC, '조작·지도 상태')  # 장치 → 상태 (메인까지 두 단계 가면 안 됨)
    s.step(b'm', '·  메인 메뉴')
    s.step(ESC, '·  메인 메뉴')  # 최상위 Esc는 무시
    s.absent('·  상태')
    s.finish()


@scenario('estop-on-immediate-off-confirm')
def _(s):
    s.step(b'8', wait=0.3)
    s.step(ENTER, '비상정지 ON')
    s.step(DOWN, wait=0.3)
    s.step(ENTER, '비상정지 → ON')  # 확인창 없이 바로
    s.step(ESC, 'ON은 바로 적용')
    s.step(DOWN, wait=0.3)
    s.step(ENTER, '비상정지를 해제할까요?')
    s.step(b'y', '비상정지 → OFF')
    s.step(ESC, '비상정지 OFF')
    s.finish()


@scenario('letter-key-focus-restart-validate')
def _(s):
    s.step(b'9', wait=0.3)
    s.step(ENTER, '전체 스택 재시작')
    s.step(b'a', wait=0.3)
    s.step(ENTER, 'RESTART를 입력')
    s.step(b'nope' + ENTER, 'RESTART를 정확히 입력하세요')
    s.step(ESC, '노드 이름 직접 입력')
    s.absent('/cmd/restart')
    s.finish()


@scenario('settings-float-validate')
def _(s):
    s.step(b'4', wait=0.3)
    s.step(ENTER, '속도/시간 설정')
    s.step(b'7', wait=0.3)
    s.step(ENTER, '선속도')
    s.step(ENTER, '값 입력')
    s.step(b'-1' + ENTER, '0 이상이어야 합니다')
    s.step(BACKSPACE * 2 + b'0.3' + ENTER, '선속도 : 0.30 m/s')
    s.finish()


def main() -> int:
    only = sys.argv[1:]
    run = [fn for fn in SCENARIOS if not only or fn.__name__ in only]
    failed = []
    for fn in run:
        try:
            fn()
        except Exception as e:
            failed.append((fn.__name__, str(e)))
            print(f'FAIL {fn.__name__}: {e}')
    print(f'\n{len(run) - len(failed)}/{len(run)} 통과')
    return 1 if failed or not run else 0


if __name__ == '__main__':
    raise SystemExit(main())
