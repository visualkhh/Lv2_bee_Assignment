"""도커 모드에서 호스트 SIL 시뮬레이터 GUI를 같이 띄운다.

시뮬레이터(lv2_module6/simulators/vazebo/simple.py)는 호스트에서 돌며
  - 카메라 장면을 debug/input/hardware/camera/ 에 써서 컨테이너 fake_camera가 토픽으로 발행하고
  - 실환경 3D(로봇 추적 시점)와 카메라 "보내기 전(SIM) / 받은 후(ROS)" × RGB·DEPTH·POINT를 tkinter 창으로 보여 준다.
컨테이너 쪽 `test-faker 0`은 이미 떠 있어야 한다 (이 런처는 CLI만 관리한다).

  docker + 메뉴      기본으로 GUI 시작
  docker + status    안 띄움 (일회성 명령)
  --sim / --no-sim   강제 on/off
  BEEBOT_SIM_HEADLESS=1  창 없이 --headless (CI/원격)
"""
from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

CLI = Path(__file__).resolve().parents[2]      # tools/cli
ROOT = CLI.parents[1]                          # lv2_module6
SIM_DIR = Path(__file__).resolve().parent / 'vazebo'
SIM_MAIN = SIM_DIR / 'simple.py'   # 없으면 main.py로 폴백
SIM_PY = Path(__file__).resolve().parent / '.venv' / 'bin' / 'python'
LOG = Path.home() / '.cache' / 'beebot' / 'simulator.log'
PID_ENV = 'BEEBOT_SIM_PID'
ASSETS = Path(__file__).resolve().parent / 'assets'


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except (ProcessLookupError, ValueError):
        return False
    except PermissionError:
        return True
    return True


def _pgrep() -> list[int]:
    try:
        out = subprocess.check_output(['pgrep', '-f', r'vazebo/(simple|main)\.py'], text=True)
    except (OSError, subprocess.CalledProcessError):
        return []
    return [int(x) for x in out.split() if x.isdigit()]


def _tail(path: Path, n: int = 8) -> str:
    try:
        return '\n'.join(path.read_text(errors='replace').splitlines()[-n:])
    except OSError:
        return ''


class Simulator:
    """시뮬레이터 프로세스 하나. start()가 실패해도 CLI는 계속 돈다."""

    def __init__(self, debug_dir=None, headless: bool | None = None):
        self.debug_dir = Path(debug_dir).resolve() if debug_dir else None
        self.headless = (headless if headless is not None
                         else os.environ.get('BEEBOT_SIM_HEADLESS') == '1')
        self.proc: subprocess.Popen | None = None
        self.pid: int | None = None
        self.owned = False

    # ------------------------------------------------------------- 시작
    def start(self) -> 'Simulator':
        # 재연결 exec로 다시 들어온 경우: 앞 프로세스가 띄운 시뮬레이터를 이어받는다
        old = os.environ.get(PID_ENV)
        if old and old.isdigit() and _alive(int(old)):
            self.pid = int(old)
            self.owned = True
            print(f'[sim] 기존 시뮬레이터 사용 (pid {self.pid})', file=sys.stderr)
            return self

        others = [p for p in _pgrep() if p != os.getpid()]
        if others:
            print(f'[sim] 시뮬레이터가 이미 실행 중 (pid {others[0]}) — 그대로 쓴다', file=sys.stderr)
            return self

        main = SIM_MAIN if SIM_MAIN.is_file() else (SIM_DIR / 'main.py')
        if not main.is_file():
            print(f'[sim] 시뮬레이터 없음: {SIM_MAIN}', file=sys.stderr)
            return self
        if not SIM_PY.is_file():
            print(f'[sim] 시뮬레이터 venv 없음: {SIM_PY}\n'
                  '      cd lv2_module6/simulators && ./setting-venv.sh', file=sys.stderr)
            return self
        if sys.platform.startswith('linux') and not (
                os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            print('[sim] DISPLAY 없음 — GUI를 띄울 수 없다 (창 없이 쓰려면 BEEBOT_SIM_HEADLESS=1)',
                  file=sys.stderr)
            return self

        env = os.environ.copy()
        env['PYTHONUNBUFFERED'] = '1'
        if self.debug_dir:
            env['BEEBOT_DEBUG_DIR'] = str(self.debug_dir)   # vazebo/bridge.py가 읽는다
        if ASSETS.is_dir():
            env['BEEBOT_SIM_ASSETS'] = str(ASSETS)          # 원본 isaac/ 대신 복사본 사용
        argv = [str(SIM_PY), str(main)] + (['--headless'] if self.headless else [])

        LOG.parent.mkdir(parents=True, exist_ok=True)
        with open(LOG, 'ab', buffering=0) as log:
            self.proc = subprocess.Popen(
                argv, cwd=str(SIM_DIR), env=env, stdin=subprocess.DEVNULL,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        os.environ[PID_ENV] = str(self.proc.pid)
        self.pid = self.proc.pid
        self.owned = True

        time.sleep(0.4)  # 바로 죽는 경우(의존성·tkinter·에셋)를 조기에 알린다
        if self.proc.poll() is not None:
            print(f'[sim] 시뮬레이터가 바로 종료됨 (exit {self.proc.returncode}) — {LOG}',
                  file=sys.stderr)
            tail = _tail(LOG)
            if tail:
                print(tail, file=sys.stderr)
            self.owned = False
            os.environ.pop(PID_ENV, None)
            return self

        print(f'[sim] SIL 뷰어 시작 (pid {self.pid}) — 로그 {LOG}', file=sys.stderr)
        return self

    # ------------------------------------------------------------- 종료
    def stop(self) -> None:
        if not self.owned or not self.pid:
            return
        if _alive(self.pid):
            try:
                if self.proc is not None:
                    self.proc.terminate()
                else:
                    os.kill(self.pid, 15)
            except ProcessLookupError:
                pass
            end = time.monotonic() + 3.0
            while time.monotonic() < end and _alive(self.pid):
                time.sleep(0.05)
            if _alive(self.pid):
                try:
                    if self.proc is not None:
                        self.proc.kill()
                    else:
                        os.kill(self.pid, 9)
                except ProcessLookupError:
                    pass
        os.environ.pop(PID_ENV, None)
        self.owned = False
        print('[sim] SIL 뷰어 종료', file=sys.stderr)
