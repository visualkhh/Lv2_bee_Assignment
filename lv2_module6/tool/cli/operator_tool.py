#!/usr/bin/env python3
"""운영자 수동 조작 CLI.

bringup이 떠 있는 상태에서 상태 확인, 자동/수동 전환, 비상정지, 바퀴·팔·그리퍼
수동 명령, 지도 모드/지도 전환을 한 곳에서 실행한다.

  ./tool/run-cli              # 대화형 메뉴
  ./tool/run-cli status       # 한 번만 출력 (자동화용)
  ./tool/run-cli mode manual
  ./tool/run-cli teleop
  ./tool/run-cli arm set joint1=0.0 joint2=0.4
  ./tool/run-cli map save

모든 수동 명령은 먼저 `mode manual`이어야 하고, 비상정지 중에는 거부한다.
실제 명령은 기존 control/manual_control 노드가 받아 안전 체인으로 보낸다.

구조 (OOP 분리):
  op/terminal.py  터미널 원자 연산
  op/ui.py        화면 전부 (프레임 + 방향키 선택 + 모달 입력)
  op/state.py     StateStore — ROS 상태 보관
  op/bus.py       Bus — 발행·전송·대기·명령 실행
  op/pages/       Page → Menu → 각 메뉴 화면
  이 파일         OperatorTool(Node): 배선 + argparse + main
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from op.bus import Bus
from op.consts import _nonnegative_float, _positive_float
from op.pages import MainMenu
from op.state import StateStore
from op.terminal import Terminal
from op.ui import Ui

LATCHED = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
LIVE = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

STATE_TOPICS = {
    'operation_mode': '/state/operation_mode',
    'map_mode': '/state/map_mode',
    'map_name': '/state/map_name',
    'map_list': '/state/map_list',
    'devices': '/state/devices',
    'topics': '/state/topics',
    'restartable': '/state/restartable',
    'restart': '/state/restart',
}


class OperatorTool(Node):
    """배선 전용 노드 — 상태·화면·버스는 op/ 패키지가 맡는다."""

    def __init__(self):
        super().__init__('operator_tool')
        self.term = Terminal()
        self.state = StateStore()
        self.ui = Ui(self.term, self.state, self.poll)
        self.bus = Bus(self, self.state, self.ui, self.term)
        self._device_subs: dict[str, object] = {}
        self._wire()

    # ------------------------------------------------------------- ROS 배선
    def poll(self, seconds: float = 0.2):
        """콜백을 잠깐 돌려 상태를 갱신한다."""
        end = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.01, end - time.monotonic())))

    def _wire(self):
        for key, topic in STATE_TOPICS.items():
            self.create_subscription(
                String, topic,
                lambda msg, key=key: self.state.on_state_value(key, msg.data),
                LATCHED)
        self.create_subscription(Bool, '/state/emergency',
                                 lambda m: self.state.on_emergency(m.data), LATCHED)
        self.create_subscription(String, '/state/devices', self._on_devices, LATCHED)
        self.create_subscription(JointState, '/joint_states',
                                 lambda m: self.state.on_joints(m.name, m.position, m.velocity), 10)
        self.create_subscription(DiagnosticArray, '/diagnostics_agg',
                                 lambda m: self.state.on_diagnostics(m.status), LIVE)
        self.create_subscription(String, '/robot_say',
                                 lambda m: self.state.on_say(m.data), 10)
        # 커스텀 msg가 빌드 전이면 해당 줄만 비활성 (cli 동작에는 지장 없음)
        try:
            from perception.msg import TrackedObjectArray  # type: ignore
            self.create_subscription(
                TrackedObjectArray, '/tracked_objects',
                lambda m: self.state.on_objects(m.objects), LIVE)
        except Exception:
            pass
        try:
            from plan.msg import TaskStatus  # type: ignore
            self.create_subscription(
                TaskStatus, '/task_status',
                lambda m: self.state.on_task(m.plan_id, m.step, m.state, m.reason),
                LATCHED)
        except Exception:
            pass

    def _on_devices(self, msg: String):
        names = [n.strip() for n in msg.data.split(',') if n.strip()]
        for name in self.state.on_devices(names):
            # 장치 목록은 state_manager가 latched로 계속 보낸다. 새 장치가 나타나면
            # /state/devices/<이름>을 그때 구독한다 (장치가 나중에 꽂혀도 상세를 본다).
            self._device_subs[name] = self.create_subscription(
                String, f'/state/devices/{name}',
                lambda detail, n=name: self.state.on_device_detail(n, detail.data),
                LATCHED)

    def screen(self, name: str):
        """메뉴 화면 인스턴스 (진단·RViz는 진입 시 생성)."""
        if name == '1':
            from op.pages import StatusMenu
            return StatusMenu(self)
        if name == '2':
            from op.pages import DevicesMenu
            return DevicesMenu(self)
        if name == '3':
            from op.pages import ModeMenu
            return ModeMenu(self)
        if name == '4':
            from op.pages import ManualMenu
            return ManualMenu(self)
        if name == '5':
            from op.pages import ArmMenu
            return ArmMenu(self)
        if name == '6':
            from op.pages import GripperMenu
            return GripperMenu(self)
        if name == '7':
            from op.pages import MapMenu
            return MapMenu(self)
        if name == '8':
            from op.pages import EstopMenu
            return EstopMenu(self)
        if name == '9':
            from op.pages import NodesMenu
            return NodesMenu(self)
        if name == '10':
            from op.pages import RvizPage
            return RvizPage(self)
        if name == '11':
            from op.pages import ParamMenu
            return ParamMenu(self)
        if name == 'diagnostics':
            from op.pages import DiagnosticsMenu
            return DiagnosticsMenu(self)
        return None

    # ------------------------------------------------------------- CLI 진입
    def cmd_menu(self, args) -> int:
        from op.pages import MainMenu
        return int(MainMenu(self).show() or 0)

    def __getattr__(self, name: str):
        # cmd_* 한 번 명령은 Bus에 위임 (run-cli status/mode/drive/... 그대로 동작)
        if name.startswith('cmd_') or name in ('map_args', 'status_args'):
            try:
                bus = object.__getattribute__(self, 'bus')
            except AttributeError:
                raise AttributeError(name) from None
            return getattr(bus, name)
        raise AttributeError(name)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='operator_tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''예시:
  ./tool/run-cli status
  ./tool/run-cli menu
  ./tool/run-cli
  ./tool/run-cli mode manual
  ./tool/run-cli drive forward --speed 0.15 --duration 1
  ./tool/run-cli teleop
  ./tool/run-cli arm set joint1=0.0 joint2=0.4
  ./tool/run-cli gripper open
  ./tool/run-cli map status
  ./tool/run-cli map save
''')
    sub = parser.add_subparsers(dest='command', required=True)

    sub.add_parser('menu', help='대화형 메뉴 (인자 없이 실행했을 때 기본값)')

    status = sub.add_parser('status', help='조작·지도·장치 상태')
    status.add_argument('--diagnostics', nargs='?', const='', default=None, metavar='이름',
                        help='/diagnostics_agg 요약 · 이름 일부를 주면 그 항목 상세(값 전부) — 예: --diagnostics signal_decoder')
    status.add_argument('--topics', action='store_true', help='/state/topics 전체 출력')

    mode = sub.add_parser('mode', help='조작 모드 AUTO/MANUAL/EXPLORE/TEACH 전환 (TEACH = 팔 토크 끄고 손으로 자세)')
    mode.add_argument('mode', type=str.lower, choices=('auto', 'manual', 'explore', 'teach'))

    estop = sub.add_parser('estop', help='비상정지 on/off/status')
    estop.add_argument('state', type=str.lower, choices=('on', 'off', 'status'))

    drive = sub.add_parser('drive', help='바퀴 수동 주행 (MANUAL 모드)')
    drive.add_argument('direction', type=str.lower, choices=('forward', 'back', 'left', 'right', 'stop'))
    drive.add_argument('--speed', type=_nonnegative_float, default=0.15, help='선속도 [m/s] (기본 0.15)')
    drive.add_argument('--angular', type=_nonnegative_float, default=0.8, help='각속도 [rad/s] (기본 0.8)')
    drive.add_argument('--duration', type=_positive_float, default=1.0, help='주행 시간 [s] (기본 1.0)')
    drive.add_argument('--force', action='store_true', help='MANUAL 상태 확인을 건너뜀 (위험)')

    teleop = sub.add_parser('teleop', help='터미널 키보드 수동 주행 (MANUAL 모드)')
    teleop.add_argument('--linear', type=_nonnegative_float, default=0.15, help='전후진 속도 [m/s]')
    teleop.add_argument('--angular', type=_nonnegative_float, default=0.8, help='회전 속도 [rad/s]')
    teleop.add_argument('--force', action='store_true', help='MANUAL 상태 확인을 건너뜀 (위험)')

    arm = sub.add_parser('arm', help='팔 관절 수동 명령 (MANUAL 모드)')
    arm.add_argument('action', type=str.lower, choices=('show', 'set', 'delta', 'home', 'initial', 'reboot', 'capture'),
                     help='initial = plan.yaml 시작 자세(initial_pose·initial_gripper)로 · '
                          'reboot = Hardware Error(과부하 등) 난 모터만 재부팅 (/cmd/manual/reboot_motors) · '
                          'capture = 지금 팔·그리퍼 자세를 plan.yaml initial_pose·initial_gripper 형식으로 출력 (보통 mode teach로 손으로 잡은 뒤)')
    arm.add_argument('targets', nargs='*', help='관절 목표: joint1=0.0 joint2=0.4 (all=0.0 가능)')
    arm.add_argument('--force', action='store_true', help='MANUAL/MAPPING 확인을 건너뜀 (위험)')

    gripper = sub.add_parser('gripper', help='그리퍼 수동 명령 (MANUAL 모드)')
    gripper.add_argument('action', type=str.lower, choices=('open', 'close', 'position', 'show'))
    gripper.add_argument('value', nargs='?', help='position일 때 목표 [m] (0=닫힘, 0.0265=열림)')
    gripper.add_argument('--force', action='store_true', help='MANUAL 상태 확인을 건너뜀 (위험)')

    map_parser = sub.add_parser('map', help='지도 모드·지도 전환·저장')
    map_parser.add_argument('action', type=str.lower, choices=('status', 'list', 'mode', 'switch', 'save'))
    map_parser.add_argument('name', nargs='?', help='mode: mapping/localization · switch: 지도 이름')
    map_parser.add_argument('--file', help='save 출력 경로 (기본: <map_dir>/<현재 지도>/map)')
    map_parser.add_argument('--map-dir', help='save 기본 지도 폴더 (기본: ros2_ws/map)')
    map_parser.add_argument('--format', default='pgm', choices=('pgm', 'png', 'bmp'),
                            help='save 이미지 형식 (기본 pgm)')
    map_parser.add_argument('--topic', default='/map', help='save할 지도 토픽 (기본 /map)')

    node = sub.add_parser('node', help='노드 상태·재시작·수명주기 활성화 (메뉴: 9 노드)')
    node.add_argument('action', type=str.lower, choices=('list', 'status', 'restart', 'activate'),
                      help='list = 전체 · status <이름> · restart <이름> --yes (수명주기 노드는 active까지) · '
                           'activate <이름> = 수명주기 노드 unconfigured·inactive → active')
    node.add_argument('name', nargs='?', help='노드 이름 (예: velocity_smoother)')
    node.add_argument('--yes', action='store_true', help='restart 실제 실행')

    restart = sub.add_parser('restart', help='state_manager에 재시작 요청 (위험) — all = bringup 전체')
    restart.add_argument('target', help='all 또는 노드 이름')
    restart.add_argument('--yes', action='store_true', help='실제로 재시작')

    return parser


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:  # 인자 없이 터미널에서 실행하면 대화형 메뉴
        argv = ['menu'] if sys.stdin.isatty() else ['status']
    parser = build_parser()
    args = parser.parse_args(argv)
    rclpy.init(args=[])
    node = OperatorTool()
    try:
        method = getattr(node, f'cmd_{args.command}')
        return int(method(args) or 0)
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
