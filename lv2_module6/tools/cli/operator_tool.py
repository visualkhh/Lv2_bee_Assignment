#!/usr/bin/env python3
"""운영자 수동 조작 CLI.

bringup이 떠 있는 상태에서 상태 확인, 자동/수동 전환, 비상정지, 바퀴·팔·그리퍼
수동 명령, 지도 모드/지도 전환을 한 곳에서 실행한다.

  ./tools/run-cli              # 대화형 메뉴
  ./tools/run-cli status       # 한 번만 출력 (자동화용)
  ./tools/run-cli mode manual
  ./tools/run-cli teleop
  ./tools/run-cli arm set joint1=0.0 joint2=0.4
  ./tools/run-cli map save

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
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import rclpy
import rclpy.time
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Imu, JointState, PointCloud2
from std_msgs.msg import Bool, String

from op.bus import Bus
from op.consts import _nonnegative_float, _positive_float
from op.defs import QUIT, STAY
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
    'collision': '/state/collision',
}


class OperatorTool(Node):
    """배선 전용 노드 — 상태·화면·버스는 op/ 패키지가 맡는다."""

    def __init__(self):
        super().__init__('operator_tool')
        self.term = Terminal()
        self.state = StateStore()
        self.ui = Ui(self.term, self.state)
        self.bus = Bus(self, self.state, self.ui, self.term)
        self.ui.bus = self.bus   # 상태 칸(예: 충돌감시)이 Enter에서 바로 명령을 보낼 때
        self._device_subs: dict[str, object] = {}
        self._restart_note = ''
        self._restart_armed_at = 0.0
        self.resume_path: list[str] = []   # 지금 열려 있는 화면 항목 키 경로 (예: ['4','8','1']) — 재연결 exec 뒤 이 경로로 복원
        # 화면 틀(Window) 신호 — ROS 폴링·재연결은 'tick', 이름으로 화면 열기는 'navigate'
        self.ui.window.connect('tick', self._on_tick)
        self.ui.window.connect('navigate', self._on_navigate)
        self.ui._reconnect_status = self.reconnect_status
        self._wire()

    # ------------------------------------------------------------- ROS 배선
    def poll(self, seconds: float = 0.2):
        """콜백을 잠깐 돌려 상태를 갱신한다."""
        end = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.01, end - time.monotonic())))

    def _on_tick(self, seconds: float):
        if not rclpy.ok():   # 종료 신호 — 메뉴 루프를 끝낸다
            raise SystemExit(0)
        self.poll(seconds)
        self.bus.nav.tick()   # docker 웨이포인트 result 파일 — 다음 goal을 바로 보낸다
        self.check_cli_reconnect()

    def _on_navigate(self, name: str, params: dict):
        page = self.screen(name, **params)
        return page.show() if page is not None else STAY

    def reconnect_status(self) -> str | None:
        """상태바용 — 재연결을 기다리는 중이면 남은 초, 아니면 None."""
        if not self._restart_armed_at:
            return None
        if self.state.state_last_update > self._restart_armed_at + 0.5:
            return None
        remain = max(0, int(12.0 - (time.monotonic() - self._restart_armed_at) + 0.999))
        return f'재연결 {remain}s'

    def check_cli_reconnect(self):
        """launch 재시작 뒤 /state/*가 끊긴 채면 CLI를 새 프로세스로 다시 띄운다.

        state_manager가 launch 전체를 재시작하면 기존 rclpy 참여자가 FastDDS
        재탐색을 못 해 화면이 계속 '끊김'으로 남는 경우가 있다. 새 프로세스는
        새 DDS 참여자로 붙으므로, 메뉴가 한가할 때(명령 실행·다이얼로그 밖)
        스스로 exec한다. bringup은 건드리지 않는다.
        """
        note = (self.state.values.get('restart') or '').strip()
        now = time.monotonic()
        if note and note != self._restart_note:
            # 시작할 때 읽은 /state/restart(latched)는 arm하지 않는다.
            # 과거 실행의 note로 재연결 12초 exec가 도는 것을 막고, CLI 실행 중 note가
            # 실제로 바뀐(=launch 재시작) 경우에만 재연결을 기다린다.
            if self._restart_note:
                self._restart_armed_at = now
            self._restart_note = note
        # /state/restart 없이 스택이 죽었다 살아난 경우(통신 끊김)도 같은 재연결 — 한 번이라도 받은 뒤 15s 침묵이면 arm.
        # exec 뒤 새 프로세스는 state_last_update=0이라 스택이 계속 죽어 있어도 반복 exec하지 않는다.
        last = self.state.state_last_update
        if not self._restart_armed_at and last > 0 and now - last > 15.0:
            self._restart_armed_at = now
        if not self._restart_armed_at:
            return
        # state_manager가 다시 살아 /state/*를 보내면 재연결 불필요.
        # NOTE: _restart_note는 지우지 않는다 — /state/restart는 latched라 값이 계속 남아 있고,
        # 지우면 다음 tick에 같은 note로 다시 arm돼 '재연결 12s'가 영원히 반복된다.
        if self.state.state_last_update > self._restart_armed_at + 0.5:
            self._restart_armed_at = 0.0
            return
        if now - self._restart_armed_at < 12.0:
            return
        self.ui.close()
        os.environ['BEE_RESUME_PATH'] = ','.join(self.resume_path)   # 보던 화면 그대로 — 메뉴 키 경로를 새 프로세스가 되짚는다
        os.execv(sys.executable, [sys.executable, '-u', str(Path(__file__).resolve()), 'menu'])

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
                                 lambda m: self.state.on_joints(m.name, m.position, m.velocity, m.effort), 10)
        # 상단 생존 표시용 — 수신 시각만 (카메라는 영상 대신 가벼운 camera_info)
        for key, topic, typ in (('odom', '/odom', Odometry), ('imu', '/imu', Imu),
                                ('front_cam', '/pole_front_cam/color/camera_info', CameraInfo),
                                ('gripper_cam', '/gripper_cam/color/camera_info', CameraInfo),
                                ('cuvslam', '/cuvslam/odom', Odometry)):
            self.create_subscription(typ, topic, lambda _m, key=key: self.state.tick(key), qos_profile_sensor_data)
        # 점구름은 크니 풀지 않고(raw) 수신 시각만 — raw를 못 받는 대역(fake/docker)이면 이 칸은 ✗
        try:
            self.create_subscription(PointCloud2, '/pole_front_cam/depth/color/points_filtered',
                                     lambda _m: self.state.tick('points'), qos_profile_sensor_data, raw=True)
        except TypeError:
            pass
        self.create_subscription(DiagnosticArray, '/diagnostics_agg',
                                 lambda m: self.state.on_diagnostics(m.status), LIVE)
        # 커스텀 msg가 빌드 전이면 해당 줄만 비활성 (cli 동작에는 지장 없음)
        try:
            from perception.msg import TrackedObjectArray  # type: ignore
            self.create_subscription(
                TrackedObjectArray, '/tracked_objects',
                lambda m: self.state.on_objects(m.objects), LIVE)
        except Exception:
            pass
        try:
            from control.msg import MotorStatus  # type: ignore
            self.create_subscription(
                MotorStatus, '/motor_status',
                lambda m: self.state.on_motor_status(m.joints, m.temperature, m.text,
                                                     getattr(m, 'battery_voltage', float('nan'))), LATCHED)
        except Exception:
            pass
        try:
            from plan.msg import TaskStatus  # type: ignore
            self.create_subscription(
                TaskStatus, '/task/status',
                lambda m: self.state.on_task(m.plan_id, m.step, m.state, m.reason),
                LATCHED)
        except Exception:
            pass
        try:
            from plan.msg import MissionState  # type: ignore
            self.create_subscription(MissionState, '/mission/state', self.state.on_mission, LATCHED)
        except Exception:
            pass
        self._wire_mission()
        # 로봇 map 위치 — tf2_ros가 없는 호스트면 칸이 '-'로 남는다
        try:
            from tf2_ros import Buffer, TransformListener
            self._tf_buf = Buffer()
            self._tf_listener = TransformListener(self._tf_buf, self)
            self.create_timer(0.5, self._update_map_pose)
        except Exception:
            pass
        # 노드별 추가 상태·기능 (예: controller_server RL/DWB)
        from op.pages.nodes import wire_all
        wire_all(self)

    def _wire_mission(self):
        """미션 화면 디버그 줄·상단 미션 칸용 구독 — msg가 없는 환경이면 그 줄만 빠진다."""
        subs = (  # (패키지.msg, 클래스, 토픽, QoS, 콜백) — QoS는 발행 쪽과 맞춤 (mission_manager.cpp · coordinator.cpp)
            ('plan.msg', 'SubGoalArray', '/task/sub_goals', LATCHED, self.state.on_sub_goals),
            ('plan.msg', 'BasketState', '/coordinator/basket', LATCHED, self.state.on_basket),
            ('perception.msg', 'MissionSignal', '/mission/signal_status', LIVE, self.state.on_signal_status),
            ('ml.msg', 'SkillContext', '/skill_context', LIVE, self.state.on_skill_context),
            # nav2 collision_monitor — 동작이 바뀔 때만 발행
            ('nav2_msgs.msg', 'CollisionMonitorState', '/collision_monitor_state', LIVE, self.state.on_collision_state),
        )
        import importlib
        for mod, cls, topic, qos, cb in subs:
            try:
                typ = getattr(importlib.import_module(mod), cls)
            except Exception:
                continue
            self.create_subscription(typ, topic, cb, qos)
        try:  # Nav2 주행 진행 (navigate skill이 보낸 goal)
            from nav2_msgs.action import NavigateToPose  # type: ignore
            self.create_subscription(NavigateToPose.Impl.FeedbackMessage, '/navigate_to_pose/_action/feedback',
                                     lambda m: self.state.on_nav_feedback(m.feedback), LIVE)
        except Exception:
            pass

    def _lookup(self, parent: str, child: str):
        """TF parent → child (x, y, yaw[rad]) — 없으면 None."""
        try:
            tr = self._tf_buf.lookup_transform(parent, child, rclpy.time.Time())
        except Exception:
            return None
        t, q = tr.transform.translation, tr.transform.rotation
        return (t.x, t.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def _update_map_pose(self):
        self.state.map_pose = self._lookup('map', 'base_footprint')
        mo = self._lookup('map', 'odom')   # SLAM 보정 — 튀면 cuVSLAM이 위치를 고친 것 (루프 클로저 · 재위치)
        if mo is not None:
            self.state.on_map_odom(*mo)

    def _on_devices(self, msg: String):
        names = [n.strip() for n in msg.data.split(',') if n.strip()]
        for name in self.state.on_devices(names):
            # 장치 목록은 state_manager가 latched로 계속 보낸다. 새 장치가 나타나면
            # /state/devices/<이름>을 그때 구독한다 (장치가 나중에 꽂혀도 상세를 본다).
            self._device_subs[name] = self.create_subscription(
                String, f'/state/devices/{name}',
                lambda detail, n=name: self.state.on_device_detail(n, detail.data),
                LATCHED)

    def screen(self, name: str, **params):
        """메뉴 화면 인스턴스 (진단·RViz는 진입 시 생성). params = 화면에 넘기는 값 (웹 쿼리처럼 — 예: map 화면의 mode·source)."""
        if name == 'map':
            from op.pages import MapPage
            return MapPage(self, **params)
        if name == 'arm':
            from op.pages import ArmControlPage
            return ArmControlPage(self)
        if name == '1':
            from op.pages import StatusMenu
            return StatusMenu(self)
        if name == '2':
            from op.pages import DevicesMenu
            return DevicesMenu(self)
        if name == '3':
            from op.pages import ControlMenu
            return ControlMenu(self)
        if name == '4':
            from op.pages import MapMenu
            return MapMenu(self)
        if name == '5':
            from op.pages import NodesMenu
            return NodesMenu(self)
        if name == '6':
            from op.pages import RvizPage
            return RvizPage(self)
        if name == '7':
            from op.pages import CalibrateMenu
            return CalibrateMenu(self)
        if name == '8':
            from op.pages import TopicBagMenu
            return TopicBagMenu(self)
        if name == '9':
            from op.pages import MissionMenu
            return MissionMenu(self)
        if name == 'objects':
            from op.pages import ObjectsPage
            return ObjectsPage(self)
        if name == 'basket':
            from op.pages import BasketPage
            return BasketPage(self)
        if name == 'diagnostics':
            from op.pages import DiagnosticsMenu
            return DiagnosticsMenu(self)
        return None

    # ------------------------------------------------------------- CLI 진입
    def cmd_menu(self, args) -> int:
        from op.pages import MainMenu
        path = [k for k in os.environ.pop('BEE_RESUME_PATH', '').split(',') if k]
        if path:   # 재연결로 다시 뜸 — 먼저 /state/*(latched)를 받는다: 모르는 상태로 항목을 열면 '모드가 AUTO가 아님' 같은 확인창이 거짓으로 뜸
            end = time.monotonic() + 6.0
            while not self.state.values.get('operation_mode') and time.monotonic() < end:
                self.poll(0.2)
        if path:   # 재연결로 다시 뜸 — 메인 메뉴에서 같은 키를 차례로 눌러 보던 화면까지 되돌아간다 (Window.run이 소비)
            self.ui.window.resume_queue = path
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
        description='bee 운영자 CLI — 인자 없이 실행하면 대화형 메뉴, 명령을 주면 한 번 실행하고 끝난다. '
                    '메뉴 번호: 1 상태 · 2 장치 · 3 조작 · 4 지도/위치 · 5 노드 · 6 RViz · 7 칼리브레이션 · 8 토픽·bag · 9 미션',
        epilog='''run-cli가 먼저 받는 명령 (bringup 수명주기·빌드 — real 전용):
  ./tools/run-cli start                  # bringup 시작 (백그라운드, 로그 ./bringup.log)
  ./tools/run-cli stop                   # 떠 있는 ROS 전부 끄기
  ./tools/run-cli restart [all|<노드>]   # 전체 또는 그 노드만 다시 띄움
  ./tools/run-cli build [colcon 인자…]   # colcon build --symlink-install (예: --packages-select ml)

ROS 층 선택 (모든 명령 앞에 붙임):
  --source real     진짜 rclpy — bringup·실기 (기본)
  --source fake     프로세스 안 가짜 ROS + 움직이는 가짜 세계 — ROS·도커 없이 화면 시험
                    (수동 주행·goal·미션·skill·costmap(c)·경로(p)가 명령에 반응. 값은 전부 더미 — ros/fake/FakeWorld.py)
  --source docker   도커 test-faker가 /debug에 쓰는 파일 브리지 (--debug-dir 경로 · --no-sim · --no-faker)
  docker는 plan 메시지를 몰라서 미션 상태(/mission/state)는 안 보임

예시:
  ./tools/run-cli mission status
  ./tools/run-cli mission start
  ./tools/run-cli --source fake menu
  ./tools/run-cli status
  ./tools/run-cli menu
  ./tools/run-cli
  ./tools/run-cli mode manual
  ./tools/run-cli drive forward --speed 0.15 --duration 1
  ./tools/run-cli teleop
  ./tools/run-cli arm set joint1=0.0 joint2=0.4
  ./tools/run-cli gripper open
  ./tools/run-cli map status
  ./tools/run-cli map save
  ./tools/run-cli node status controller_server   # RL/DWB 상태
''')
    sub = parser.add_subparsers(dest='command', required=True)

    sub.add_parser('menu', help='대화형 메뉴 (인자 없이 실행했을 때 기본값)')

    status = sub.add_parser('status', help='조작·지도·장치 상태')
    status.add_argument('--diagnostics', nargs='?', const='', default=None, metavar='이름',
                        help='/diagnostics_agg 요약 · 이름 일부를 주면 그 항목 상세(값 전부) — 예: --diagnostics signal_decoder')
    status.add_argument('--topics', action='store_true', help='/state/topics 전체 출력')

    mode = sub.add_parser('mode', help='조작 모드 AUTO/MANUAL 전환')
    mode.add_argument('mode', type=str.lower, choices=('auto', 'manual'))

    estop = sub.add_parser('estop', help='비상정지 on/off/status')
    estop.add_argument('state', type=str.lower, choices=('on', 'off', 'status'))

    collision = sub.add_parser('collision', help='collision_monitor 켜기/끄기/상태 (MANUAL·MAPPING이면 자동으로 꺼짐 — 직접 바꿀 때)')
    collision.add_argument('state', type=str.lower, choices=('on', 'off', 'status'))

    bag = sub.add_parser('bag', help='bag 녹화 + CPU·메모리·GPU 부하 기록 — start | stop | status (메뉴 8)')
    bag.add_argument('action', type=str.lower, choices=('start', 'stop', 'status'))
    bag.add_argument('--label', '-l', default=None, help='start: 폴더 이름에 붙일 라벨 (예: asis_manual)')
    bag.add_argument('--topics', nargs='*', default=None, help='start: 이 토픽만 (생략하면 진단 프리셋)')
    bag.add_argument('--regex', default=None, help='start: 토픽 정규식 (프리셋 대신)')
    bag.add_argument('--no-sys', action='store_true', help='start: 부하(sys.csv) 기록 끔')

    drive = sub.add_parser('drive', help='바퀴 수동 주행 (MANUAL 모드)')
    drive.add_argument('direction', type=str.lower, choices=('forward', 'back', 'left', 'right', 'stop'))
    drive.add_argument('--speed', type=_nonnegative_float, default=0.15, help='선속도 [m/s] (기본 0.15)')
    drive.add_argument('--angular', type=_nonnegative_float, default=0.8, help='각속도 [rad/s] (기본 0.8)')
    drive.add_argument('--duration', type=_positive_float, default=1.0, help='주행 시간 [s] (기본 1.0)')
    drive.add_argument('--force', action='store_true', help='MANUAL 상태 확인을 건너뜀 (위험)')

    calibrate = sub.add_parser('calibrate', help='칼리브레이션 (메뉴 7) — wheel: 계획을 수동 경로로 재생하며 응답을 raw.csv로 기록 (로봇이 움직임)')
    cal_sub = calibrate.add_subparsers(dest='part', required=True)
    cal_wheel = cal_sub.add_parser('wheel', help='바퀴 응답 (MANUAL 모드 — 계획 JSON 재생)')
    cal_wheel.add_argument('plan', nargs='?', default=str(Path(__file__).resolve().parent / 'op' / 'pages' / 'calibration' / 'wheel' / 'command.json'),
                           help='계획 파일 {"dt": 0.05, "steps": [[v, w], ...]} (기본 tools/cli/op/pages/calibration/wheel/command.json = sim과 같은 네모 54 s)')
    cal_wheel.add_argument('--out', default=None, help='raw.csv 경로 (기본: tools/artifacts/calibration/<시각>_wheel/raw.csv)')
    cal_wheel.add_argument('--check-only', action='store_true', help='안 움직이고 준비만 확인')
    cal_wheel.add_argument('--force', action='store_true', help='MANUAL 상태 확인을 건너뜀 (위험)')

    cal_report = cal_sub.add_parser('report', help='wheel raw.csv 리포트/비교 (생략 시 최근 1~2개)')
    cal_report.add_argument('csv', nargs='*', help='raw.csv 경로 1개(요약) 또는 2개(비교)')
    cal_report.add_argument('--labels', nargs='*', default=None, help='범례 이름 (예: SIM REAL)')
    cal_report.add_argument('--out', default=None, help='출력 PNG (기본: 첫 CSV 옆 report_*.png)')

    teleop = sub.add_parser('teleop', help='터미널 키보드 수동 주행 (MANUAL 모드)')
    teleop.add_argument('--linear', type=_nonnegative_float, default=0.15, help='전후진 속도 [m/s]')
    teleop.add_argument('--angular', type=_nonnegative_float, default=0.8, help='회전 속도 [rad/s]')
    teleop.add_argument('--force', action='store_true', help='MANUAL 상태 확인을 건너뜀 (위험)')

    arm = sub.add_parser('arm', help='팔 관절 수동 명령 (MANUAL 모드)')
    arm.add_argument('action', type=str.lower, choices=('show', 'set', 'delta', 'home', 'initial', 'reboot', 'capture'),
                     help='initial = skills.yaml(skill_pose) 시작 자세(initial_pose·initial_gripper)로 · '
                          'reboot = Hardware Error(과부하 등) 난 모터만 재부팅 (/cmd/manual/reboot_motors) · '
                          'capture = 지금 팔·그리퍼 자세를 skills.yaml(skill_pose) initial_pose·initial_gripper 형식으로 출력')
    arm.add_argument('targets', nargs='*', help='관절 목표: joint1=0.0 joint2=0.4 (all=0.0 가능)')
    arm.add_argument('--force', action='store_true', help='MANUAL/MAPPING 확인을 건너뜀 (위험)')

    gripper = sub.add_parser('gripper', help='그리퍼 수동 명령 (MANUAL 모드)')
    gripper.add_argument('action', type=str.lower, choices=('open', 'close', 'position', 'show'))
    gripper.add_argument('value', nargs='?', help='position일 때 목표 [m] (0=닫힘, 0.0265=열림)')
    gripper.add_argument('--current', type=_positive_float, default=None, metavar='A',
                         help='쥐는 힘 한계 [A] (펌웨어가 0.00269~1.2로 자름) — 생략하면 gripper_controller max_effort (0.6)')
    gripper.add_argument('--force', action='store_true', help='MANUAL 상태 확인을 건너뜀 (위험)')

    map_parser = sub.add_parser('map', help='지도 모드·지도 전환·저장')
    map_parser.add_argument('action', type=str.lower, choices=('status', 'list', 'mode', 'switch', 'save', 'delete', 'rename'))
    map_parser.add_argument('name', nargs='?', help='mode: mapping/localization · switch·delete·rename: 지도 이름')
    map_parser.add_argument('--to', help='rename 새 이름')
    map_parser.add_argument('--file', help='save 출력 경로 (기본: <map_dir>/<현재 지도>/map)')
    map_parser.add_argument('--map-dir', help='save 기본 지도 폴더 (기본: ros2_ws/map)')
    map_parser.add_argument('--format', default='pgm', choices=('pgm', 'png', 'bmp'),
                            help='save 이미지 형식 (기본 pgm)')
    map_parser.add_argument('--topic', default='/map', help='save할 지도 토픽 (기본 /map)')
    map_parser.add_argument('--localize', dest='localize', action='store_true', default=None,
                            help='save 후 LOCALIZATION으로 자동 전환 (nvblox.nvblx 저장)')
    map_parser.add_argument('--no-localize', dest='localize', action='store_false',
                            help='save만 하고 MAPPING 유지')

    beep = sub.add_parser('beep', help='OpenCR 부저 — 인자 없으면 기본음, beep <Hz 100~8000> <ms 10~2000> (메뉴 1 상태 → 삑)')
    beep.add_argument('value', nargs='*', help='Hz ms (예: 4000 300)')

    mission = sub.add_parser('mission', help='주사위 미션 — start | abort | status (메뉴 9). mission_manager의 /cmd/mission · /mission/state')
    mission.add_argument('action', type=str.lower, choices=('start', 'abort', 'status', 'skill', 'stop'),
                         help='start = 시작 (적재함 장부가 남았으면 거부) · abort = 중지 · status = 지금 단계·진행 · '
                              'skill <이름> <target> = 세부 명령 한 단계 (예: skill pick 5 · skill navigate 1.5,0.8,0) · stop = 진행 중 전부 취소')
    mission.add_argument('args', nargs='*', help='skill: <skill 이름> <target>')

    sub.add_parser('recover', help='조작이 안 될 때 — 모터 에러 재부팅 + 꺼진 컨트롤러 다시 켜기 (메뉴: 수동·팔·그리퍼 → z)')

    node = sub.add_parser('node', help='노드 상태·재시작·수명주기 활성화 (메뉴 5 노드)')
    node.add_argument('action', type=str.lower, choices=('list', 'status', 'restart', 'activate'),
                      help='list = 전체 · status <이름> · restart <이름> (수명주기 노드는 active까지) · '
                           'activate <이름> = 수명주기 노드 unconfigured·inactive → active')
    node.add_argument('name', nargs='?', help='노드 이름 (예: velocity_smoother)')

    restart = sub.add_parser('restart', help='state_manager에 재시작 요청 (위험) — all = bringup 전체')
    restart.add_argument('target', help='all 또는 노드 이름')

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
