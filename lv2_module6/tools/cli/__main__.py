#!/usr/bin/env python3
"""CLI 진입점 — ROS 층(real/fake/docker)을 고르고 operator_tool을 실행한다.

  python -m cli status                      # real (cli/start.sh가 ROS 환경을 source한 뒤)
  python -m cli --source fake status        # 프로세스 안 가짜 ROS
  python -m cli --source docker status      # 도커 test-faker의 debug/ 파일 브리지
  python -m cli --source docker menu        # + 호스트 SIL 뷰어(GUI) 자동 시작
  python -m cli --source docker --debug-dir ../debug menu
  python -m cli --source docker --no-sim status   # 뷰어 없이
  python -m cli --source docker --no-faker status # 컨테이너 test-faker 자동 시작 끔

--source fake/docker는 operator_tool을 import하기 전에 가짜 rclpy를 sys.modules에 넣는다.
operator_tool은 파일 맨 위에서 import rclpy를 하므로, 그 전에 설치가 끝나야 한다.
--source 없음/real이면 아무것도 바꾸지 않고 진짜 rclpy를 쓴다.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent          # tools/cli
sys.path.insert(0, str(HERE))

STATE_FILE = Path(os.environ.get('CLI_FAKE_STATE')
                  or (Path.home() / '.cache' / 'beebot' / 'cli-fake-state.json'))


def _parse(argv: list[str]):
    """--source/--debug-dir만 뽑고 나머지는 operator_tool에 넘긴다."""
    source = os.environ.get('CLI_SOURCE', 'real')
    debug_dir = os.environ.get('CLI_DEBUG_DIR') or None
    sim = None   # --sim True · --no-sim False · None = 자동
    no_faker = False
    rest: list[str] = []
    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg == '--source':
            i += 1
            if i >= len(argv):
                raise SystemExit('--source 값이 필요합니다 (real | fake | docker)')
            source = argv[i]
        elif arg.startswith('--source='):
            source = arg.split('=', 1)[1]
        elif arg == '--debug-dir':
            i += 1
            if i >= len(argv):
                raise SystemExit('--debug-dir 값이 필요합니다')
            debug_dir = argv[i]
        elif arg.startswith('--debug-dir='):
            debug_dir = arg.split('=', 1)[1]
        elif arg == '--sim':
            sim = True
        elif arg == '--no-sim':
            sim = False
        elif arg == '--no-faker':
            no_faker = True
        else:
            rest.append(arg)
        i += 1
    source = {'dummy': 'fake'}.get(source, source)   # dummy = 옛 이름
    if source not in ('real', 'fake', 'docker'):
        raise SystemExit(f'--source: real | fake | docker (받은 값 {source!r})')
    if source == 'real' and debug_dir:
        raise SystemExit('--debug-dir는 --source fake/docker에서만 쓴다')
    if source != 'real' and rest and rest[0] in ('start', 'stop', 'restart', 'build'):
        raise SystemExit(f"'{rest[0]}'은 실기(bringup) 명령 — --source {source}에서는 쓸 수 없음")
    if sim is True and source != 'docker':
        raise SystemExit('--sim은 --source docker에서만 쓴다 (SIL 뷰어는 도커 스택을 본다)')
    return source, debug_dir, sim, no_faker, rest


SOURCE, DEBUG_DIR, SIM, NO_FAKER, ARGS = _parse(sys.argv[1:])
sys.argv = [sys.argv[0], *ARGS]
# check_cli_reconnect가 os.execv로 다시 띄울 때도 같은 모드/경로를 쓰게
os.environ['CLI_SOURCE'] = SOURCE
if DEBUG_DIR:
    os.environ['CLI_DEBUG_DIR'] = str(DEBUG_DIR)

DEBUG_PATH = None
if SOURCE != 'real':
    from ros import default_debug_dir, install  # noqa: E402

    DEBUG_PATH = Path(DEBUG_DIR).resolve() if DEBUG_DIR else default_debug_dir()
    install(SOURCE, DEBUG_PATH)
    if SOURCE == 'docker' and not NO_FAKER:
        from ros.docker.FakerStack import FakerStack
        if not FakerStack(DEBUG_PATH).ensure():
            raise SystemExit(2)
else:
    try:
        import rclpy  # noqa: F401 — real 모드는 ROS 환경이 있어야 한다
    except ImportError as exc:
        raise SystemExit('real 모드는 ROS 2 환경이 필요합니다 — ./run-cli로 실행하거나 '
                         '--source fake|docker를 쓰세요') from exc

import operator_tool as ot  # noqa: E402

# 도커 모드 + 메뉴면 호스트 SIL 뷰어를 같이 띄운다 (--no-sim으로 끔, --sim으로 강제)
if SIM is None:
    SIM = SOURCE == 'docker' and (not ARGS or ARGS[0] == 'menu') and sys.stdin.isatty()

# 재연결 exec가 operator_tool.py가 아니라 이 진입점을 다시 지나게 (대역 설치 유지)
ot.__file__ = str(Path(__file__).resolve())


def _load_saved() -> dict:
    if SOURCE != 'fake':
        return {}
    try:
        return json.loads(STATE_FILE.read_text())
    except Exception:
        return {}


def _save(state) -> None:
    if SOURCE != 'fake':
        return
    try:
        STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
        STATE_FILE.write_text(json.dumps({
            'values': state.values,
            'emergency': state.emergency,
        }, ensure_ascii=False))
    except Exception:
        pass


def _seed(app: 'ot.OperatorTool'):
    """가짜 bringup 상태 — status/menu가 볼 화면용."""
    st = app.state
    st.values.update({
        'operation_mode': 'MANUAL',
        'map_mode': 'NONE',
        'map_name': '',
        'map_list': 'default\n20261017_223015',
        'devices': 'pole_front_cam, gripper_cam, opencr, imu, wheels, arm, gripper',
        'topics': '/joint_states · /odom · /tracked_objects (42개)',
        'restartable': 'all · perception · coordinator · manual_control',
        'restart': '',
    })
    st.values.update(_load_saved().get('values', {}))
    st.emergency = _load_saved().get('emergency', False)
    st.on_emergency(False)
    st.on_devices(['pole_front_cam', 'gripper_cam', 'opencr', 'imu', 'wheels', 'arm', 'gripper'])
    st.on_device_detail('pole_front_cam', 'camera|active|/pole_front_cam/color/image_raw')
    st.on_device_detail('gripper_cam', 'camera|active|/gripper_cam/color/image_raw')
    st.on_device_detail('opencr', 'controller|active|/joint_states,/odom')
    st.on_device_detail('imu', 'sensor|active|/imu')
    st.on_joints(
        ['joint1', 'joint2', 'joint3', 'joint4', 'joint5',
         'gripper_left_joint', 'wheel_left_joint', 'wheel_right_joint'],
        [0.0, -0.5, 0.8, 0.4, 0.0, 0.026, 1.2, 1.2],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.35, 0.0, 0.0])
    from ros.msgs import DiagnosticStatus, KeyValue
    st.on_diagnostics([
        DiagnosticStatus(level=0, name='beebot: 상태', message='정상',
                         values=[KeyValue(key='모드', value='AUTO')]),
        DiagnosticStatus(level=1, name='beebot: opencr', message='모터 에러 없음 · S 줄 지연',
                         values=[KeyValue(key='받은 S 줄', value='48 Hz')])])
    try:
        st.on_objects([1, 2, 3])
    except Exception:  # noqa: BLE001
        pass
    try:
        st.on_task(7, 3, 1, '')
    except Exception:  # noqa: BLE001
        pass


_orig_init = ot.OperatorTool.__init__
_APPS: dict = {}


def _init(self):
    _orig_init(self)
    _APPS['node'] = self
    if SOURCE == 'fake':
        _seed(self)


ot.OperatorTool.__init__ = _init


if __name__ == '__main__':
    viewer = None
    if SIM:
        from ros.docker.Simulator import Simulator
        viewer = Simulator(DEBUG_PATH).start()
    try:
        code = ot.main()
    finally:
        if viewer is not None:
            try:
                viewer.stop()
            except KeyboardInterrupt:
                pass  # Ctrl+C로 CLI를 끌 때 종료 중에 한 번 더 들어오는 경우 — 스택은 그대로 둔다
    node = _APPS.get('node')
    if node is not None:
        _save(node.state)
    raise SystemExit(code)
