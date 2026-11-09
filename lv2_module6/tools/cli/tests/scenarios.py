#!/usr/bin/env python3
"""CLI 시나리오 — 실제 pty 키 입력으로 UI 흐름을 검증한다.

실행: python3 tools/cli/tests/scenarios.py [시나리오 이름 ...]
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

HERE = Path(__file__).resolve().parent          # tools/cli/tests
CLI = HERE.parent                                # tools/cli
ENTRY = CLI / '__main__.py'
STATE = Path(os.environ.get('CLI_FAKE_STATE')
             or (Path.home() / '.cache' / 'beebot' / 'cli-fake-state.json'))
os.environ['CLI_FAKE_STATE'] = str(STATE)       # 자식 __main__이 같은 상태 파일을 쓰게

UP, DOWN, LEFT, RIGHT = b'\x1b[A', b'\x1b[B', b'\x1b[D', b'\x1b[C'
ENTER, ESC, BACKSPACE = b'\r', b'\x1b', b'\x7f'


def plain(data: bytes) -> str:
    text = data.decode('utf-8', errors='replace')
    return re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', text)


class Session:
    def __init__(self, name: str, seed: dict | None = None):
        self.name = name
        self.out = bytearray()
        STATE.unlink(missing_ok=True)
        if seed:  # 가짜 상태 시작 값 (cli.__main__._seed 뒤에 덮어씀)
            import json
            STATE.write_text(json.dumps({'values': seed}, ensure_ascii=False))
        argv = [sys.executable, str(ENTRY), '--source', 'fake', 'menu']
        self.pid, self.fd = os.forkpty() if hasattr(os, 'forkpty') else (None, None)
        if self.pid == 0:
            os.chdir(HERE.parents[2])
            os.execv(sys.executable, argv)
        if self.pid is None:
            import pty
            self.pid, self.fd = pty.fork()
            if self.pid == 0:
                os.chdir(HERE.parents[2])
                os.execv(sys.executable, argv)
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
            end = time.monotonic() + wait  # 기다리는 동안도 읽음 — 안 읽으면 pty가 차서 CLI가 출력에서 멈추고,
            while time.monotonic() < end:  #   뒤에 보낸 키가 ESC와 붙어 한 시퀀스로 먹힘 (상태바 1초 갱신 뒤 manual-cancel 실패)
                self._read(0.05)
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


def scenario(name: str, seed: dict | None = None):
    def deco(fn):
        def wrapped():
            s = Session(name, seed)
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
    s.step(ENTER, '조작')
    s.step(ESC, '메인 메뉴')
    s.finish()


@scenario('calibrate-menu-wheel')
def _(s):
    s.step(b'7', wait=0.4)   # 7) 칼리브레이션
    s.step(ENTER, '칼리브레이션')
    s.step(ENTER, '바퀴 칼리브레이션')
    s.step(ESC, wait=0.4)
    s.step(ESC, wait=0.4)
    s.finish()


@scenario('manual-cancel-no-extra-dialog')
def _(s):
    s.step(b'3', wait=0.4)
    s.step(ENTER, '조작')
    s.step(b'2', wait=0.4)
    s.step(ENTER, '수동 주행 / 텔레오퍼')
    s.step(DOWN, wait=0.4)
    s.step(ENTER, 'MANUAL 모드로 전환할까요?')
    s.step(ESC, wait=1.5)  # 제목은 창이 떠 있어도 상단 상태바 갱신(1초)으로 나와서 표식으로 못 씀
    if '일시정지'.encode() in s.out:
        raise AssertionError('취소 후 일시정지 모달이 뜸')
    s.finish()


@scenario('drive-confirm-execute')
def _(s):
    s.step(b'3', wait=0.4)
    s.step(ENTER, '조작')
    s.step(b'2', wait=0.4)
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


@scenario('map-mode-mapping', seed={'map_mode': 'LOCALIZATION', 'map_name': 'default'})
def _(s):
    s.step(b'4', wait=0.4)
    s.step(ENTER, '지도')
    s.step(DOWN, wait=0.4)
    s.step(DOWN, wait=0.4)
    s.step(ENTER, 'MAPPING 모드로 전환할까요?')
    s.step(b'y', '/cmd/map_mode')       # state_manager가 launch를 map:=slam으로 다시 띄움
    s.expect('바꾼 인자 map:=slam')      # /state/restart 응답
    s.step(ESC, '지도 저장')
    s.finish()


@scenario('map-mode-slam-restart-reply', seed={'map_mode': 'MAPPING', 'map_name': '20261017_223015'})
def _(s):
    # SLAM 중 모드 전환 = launch 재시작 — 몇 초 안에 map_mode가 안 바뀌어도 실패가 아님. 재시작 응답을 보여 줌
    s.step(b'4', wait=0.4)
    s.step(ENTER, '지도')
    for _ in range(3):
        s.step(DOWN, wait=0.4)
    s.step(ENTER, 'LOCALIZATION 모드로 전환할까요?')
    s.step(b'y', '재시작 중')
    s.absent('요청 실패')
    s.step(ESC, '지도 저장')
    s.finish()


@scenario('map-switch-missing-map', seed={'map_mode': 'LOCALIZATION', 'map_name': 'default'})
def _(s):
    # 없는 지도 이름 — state_manager는 진단에만 남기므로 CLI가 먼저 거절하고 있는 지도를 보여 줌
    s.step(b'4', wait=0.4)
    s.step(ENTER, '지도')
    s.step(b'5', wait=0.4)
    s.step(ENTER, '이름 직접 입력')
    s.step(b'n', wait=0.4)
    s.step(ENTER, '전환할 지도 폴더 이름')
    s.step(b'nope', 'nope')
    s.step(ENTER, '"nope"(으)로 전환할까요?')
    s.step(b'y', '지도 없음 "nope"')
    s.expect('있는 지도: default, 20261017_223015')
    s.step(ESC, '이름 직접 입력')  # 결과창 닫힘 → 지도 전환 메뉴
    s.step(ESC, '지도 상태')
    s.finish()


@scenario('arm-value-modal-cancel')
def _(s):
    s.step(b'3', wait=0.4)
    s.step(ENTER, '조작')
    s.step(b'3', wait=0.4)
    s.step(ENTER, '팔')
    s.step(b'2', wait=0.4)   # 2) 관절 목표 설정 (번호로 — 메뉴 순서가 바뀌어도 안 깨짐)
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
    s.step(b'3', wait=0.4)
    s.step(ENTER, '조작')
    s.step(b'2', wait=0.4)
    s.step(ENTER, '수동 주행 / 텔레오퍼')
    s.step(ENTER, 'MANUAL 모드로 전환할까요?')
    s.step(b'y', 'Esc 종료')
    s.step(b'w', '키 w')
    s.step(b'q', '키 q')  # q = 사선 (종료 아님)
    s.step(b'b', '키 b')  # 삑 (발행 출력은 종료 뒤 결과창으로)
    s.step(UP, '키 w ×4')  # 방향키 = w (w·q·b 다음 4번째)
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
    s.step(b'3', wait=0.3)
    s.step(ENTER, '조작')
    s.step(b'6', wait=0.3)
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
    s.step(b'5', wait=0.3)
    s.step(ENTER, '전체 재시작')
    s.step(b'a', wait=0.3)
    s.step(ENTER, 'RESTART를 입력')
    s.step(b'nope' + ENTER, 'RESTART를 정확히 입력하세요')
    s.step(ESC, '노드 (상태 · 파라미터 · 진단 · 재시작)')
    s.absent('/cmd/restart')
    s.finish()


@scenario('settings-float-validate')
def _(s):
    s.step(b'3', wait=0.3)
    s.step(ENTER, '조작')
    s.step(b'2', wait=0.3)
    s.step(ENTER, '수동 주행 / 텔레오퍼')
    s.step(ENTER, '속도/시간 설정')
    s.step(b'7', wait=0.3)
    s.step(ENTER, '선속도')
    s.step(ENTER, '값 입력')
    s.step(b'-1' + ENTER, '0 이상이어야 합니다')
    s.step(BACKSPACE * 2 + b'0.3' + ENTER, '선속도 : 0.30 m/s')
    s.finish()


@scenario('state-bar-tab-diag')
def _(s):
    s.step(b'r', wait=0.5)                 # 첫 화면 키 루프가 뜰 때까지 (새로고침 = 아무 일 없음)
    s.step(b'\t', 'Tab/Esc 메뉴로')        # Tab → 상태바 포커스 (진단 칸)
    s.step(ENTER, '·  진단')               # 진단 칸 Enter → 진단 화면 (navigate)
    s.step(ESC, 'Tab/Esc 메뉴로')           # 진단 닫으면 메인 · 포커스는 들어가기 전 그대로 (상태바)
    s.step(ESC, 'Tab 상태바')               # Esc → 메뉴로 포커스 복귀
    s.step(b'\t', 'Tab/Esc 메뉴로')        # Tab 다시 → 상태바
    s.step(b'\t', 'Tab 상태바')            # Tab → 메뉴
    s.finish()


@scenario('ab-route-map-pick')
def _(s):
    ROUTE = HERE.parents[1] / 'artifacts' / 'ab' / 'rl_nav_controller' / 'route.json'   # core.ROUTE (ROS 없이 import 못 해서 경로만)
    if ROUTE.exists():
        print('SKIP ab-route-map-pick: 지정한 route.json이 이미 있음 — 지우지 않으려고 건너뜀')
        return
    try:
        s.step(b'8', wait=0.4)                               # 8) A/B
        s.step(ENTER, 'A/B 테스트 (노드별)')
        s.step(ENTER, 'A/B 테스트 · rl_nav_controller')
        s.step(b'1', wait=0.2)
        s.step(ENTER, 'A/B 경로 편집')
        s.step(b'1', wait=0.2)                               # 1) 지도에서 찍기
        s.send(ENTER)
        s.expect('A/B 경로 — 지도에서 찍기', timeout=20)     # 현재 위치 TF(최대 5초) · /map을 받은 뒤에 열림
        s.step(b'+', '한 칸 =')                              # 지도가 포커스를 받아 바로 확대 (중앙 = 지도)
        s.step(ENTER, 'G1 추가')                             # 십자선 위치(지도 가운데)에 goal
        s.step(RIGHT * 2, '자유이동')                       # 방향키 = 지도를 밂 → 로봇 따라가기가 풀림
        s.step(b']', wait=0.2)                               # 방향 +15°
        s.step(ENTER, 'G2 추가')
        s.step(b'-', wait=0.3)                               # 축소
        s.step(b'r', '따라가기')                         # 다시 로봇 중심
        s.step(ESC, 'Tab 중앙')                              # Esc → 왼쪽 메뉴로 (힌트에 'Tab 중앙')
        s.step(b'9', wait=0.3)
        s.step(ENTER, '저장:')                               # 9) 저장하고 나가기 → route.json 저장
        import json
        goals = json.loads(ROUTE.read_text())['route']
        assert len(goals) == 2 and goals[1]['yaw_deg'] == 15.0, goals
    finally:
        ROUTE.unlink(missing_ok=True)


@scenario('map-pose-pick-sim-only')
def _(s):
    s.step(b'4', wait=0.3)                    # 4) 지도/위치
    s.step(ENTER, '·  지도/위치')
    s.step(b'8', wait=0.2)                    # 8) 위치 지정 (지도에서 찍기)
    s.send(ENTER)
    s.expect('로봇 위치 지정', timeout=20)
    s.step(ENTER, '로봇 위치로 지정할까요?')
    s.step(b'y', 'initialpose')              # faker 없으면 /initialpose로 (구독자 없으면 거부)
    s.step(ESC, 'Tab 중앙')
    s.step(ESC, '·  지도/위치')
    s.finish()


@scenario('arm-page-confirm')
def _(s):
    s.step(b'3', wait=0.3)                        # 3) 조작
    s.step(ENTER, '조작')
    s.step(b'3', wait=0.3)                        # 3) 팔
    s.step(ENTER, '·  팔')
    s.step(b'0', wait=0.2)                        # 0) 팔 조작 화면
    s.send(ENTER)
    s.expect('팔 조작', timeout=10)
    s.step(RIGHT * 2, '바꾼 관절 1개')            # ←→는 목표만 바꿈 — 아직 안 움직임
    s.step(ENTER, '팔을 움직일까요?')             # Enter → 이동량을 보여 주는 확인창 (기본 아니오)
    s.step(b'n', '취소 — 보내지 않음')
    s.absent('팔 목표')                           # 아니오면 아무것도 안 보냄
    s.step(ENTER, '팔을 움직일까요?')
    s.step(b'y', 'MANUAL 모드로 전환할까요?')     # cmd_arm의 기존 안전장치 그대로
    s.step(b'y', '보냈음')
    s.step(b'z', '현재값으로')                    # z = 목표를 현재값으로
    s.step(ESC, 'Tab 중앙')                       # Esc → 왼쪽 메뉴 · 한 번 더 → 팔 메뉴
    s.step(ESC, '·  팔')
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
