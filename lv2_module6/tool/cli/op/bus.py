"""로봇 동작 전부 — 발행·전송·대기·게이트·CLI 명령 실행.

node=rclpy 노드, state=StateStore, prompt=Ui 입력, term=Terminal.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.parameter import Parameter

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String

from .consts import (ARM_JOINTS, DIAG_LEVELS, GRIPPER_CLOSE, GRIPPER_JOINT,
                    GRIPPER_OPEN, WHEEL_JOINTS)
from .terminal import BOLD, DIM, GREEN, RED, YELLOW, Terminal

try:
    from nav2_msgs.srv import SaveMap as SaveMapSrv
except ImportError:  # nav2_msgs가 없는 환경에서도 나머지 명령은 동작
    SaveMapSrv = None


class Bus:
    def __init__(self, node, state, prompt, term: Terminal):
        self.node = node
        self.state = state
        self.prompt = prompt
        self.term = term
        self.linear = 0.15
        self.angular = 0.8
        self.duration = 1.0
        self.pub_operation = node.create_publisher(String, "/cmd/operation_mode", 10)
        self.pub_estop = node.create_publisher(Bool, "/cmd/e_stop", 10)
        self.pub_vel = node.create_publisher(TwistStamped, "/cmd/manual/cmd_vel", 10)
        self.pub_arm = node.create_publisher(JointState, "/cmd/manual/arm", 10)
        self.pub_map_mode = node.create_publisher(String, "/cmd/map_mode", 10)
        self.pub_map_name = node.create_publisher(String, "/cmd/map_name", 10)
        self.pub_restart = node.create_publisher(String, "/cmd/restart", 10)
        self.pub_reboot = node.create_publisher(Empty, "/cmd/manual/reboot_motors", 10)

    def _poll(self, seconds: float = 0.2):
        """콜백을 잠깐 돌려 상태를 갱신한다."""
        end = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=min(0.05, max(0.01, end - time.monotonic())))

    def _publish(self, publisher, msg):
        self._poll(0.2)  # DDS discovery
        count = publisher.get_subscription_count()
        if count == 0:
            print(f'WARN: {publisher.topic_name} 구독자 없음 — bringup이 떠 있는지 확인', file=sys.stderr)
        publisher.publish(msg)
        self._poll(0.15)

    def _send_velocity(self, linear: float, angular: float):
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.pub_vel.publish(msg)

    def _send_arm(self, targets: dict[str, float]):
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(targets)
        msg.position = [float(v) for v in targets.values()]
        self._publish(self.pub_arm, msg)

    def _send_gripper(self, position: float):
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = [GRIPPER_JOINT]
        msg.position = [float(position)]
        self._publish(self.pub_arm, msg)

    def _send_map_mode(self, mode: str):
        self._publish(self.pub_map_mode, String(data=mode))

    def _send_map_name(self, name: str):
        self._publish(self.pub_map_name, String(data=name))

    # ---------------------------------------------------------------- status
    @staticmethod

    @staticmethod
    def _rad_deg(value: float) -> str:
        return f'{value:+.3f} rad ({math.degrees(value):+.1f}°)'

    def _wait_value(self, key: str, expected: str, timeout: float = 5.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            self._poll(0.1)
            if self.state.values.get(key, '').upper() == expected.upper():
                return True
        return False

    def _wait_emergency(self, expected: bool, timeout: float = 2.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            self._poll(0.1)
            if self.state.emergency == expected:
                return True
        return False

    # -------------------------------------------------------------- 상태/안전
    def _wait_joints(self, timeout: float = 6.0) -> bool:
        """/joint_states가 올 때까지 대기 (Jetson discovery 느림 대응)."""
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            self._poll(0.2)
            if self.state.joints:
                return True
        return bool(self.state.joints)

    def _wait_operation(self) -> None:
        # discovery가 느린 Jetson에서도 state 토픽을 놓치지 않게 최대 8초 대기
        # (status는 5초. 모드 gate는 더 길게 — 못 보면 거부라 재시도만 하면 됨)
        end = time.monotonic() + 8.0
        while time.monotonic() < end and rclpy.ok():
            self._poll(0.2)
            if self.state.operation:
                break

    def ensure_manual(self, force: bool = False) -> bool:
        self._wait_operation()
        if self.state.emergency:
            print('거부: 비상정지 중입니다 — 먼저 `estop off`', file=sys.stderr)
            return False
        op = self.state.operation
        if op != 'MANUAL':
            if not force:
                print(f'거부: 현재 조작 모드 {op or "알 수 없음"} — 먼저 `mode manual`', file=sys.stderr)
                return False
            print(f'WARN: 현재 {op or "알 수 없음"} 모드 — --force로 명령을 보냅니다', file=sys.stderr)
        return True

    def ensure_arm(self, force: bool = False) -> bool:
        if not self.ensure_manual(force):
            return False
        if self.state.map_mode == 'MAPPING' and not force:
            print('거부: MAPPING 중에는 팔 명령을 manual_control이 거절합니다 '
                  '(스캔 자세 고정) — gripper만 조작 가능', file=sys.stderr)
            return False
        return True

    def ensure_manual_or_switch(self, force: bool = False) -> bool | None:
        """수동 명령 전 모드 확보. TTY면 MANUAL 전환을 먼저 묻고 바꾼다.

        자동화(비TTY)·--force는 기존 동작 유지 (거부 또는 확인 생략 — manual_control이 최종 판단).
        True=진행 · False=차단(사유 출력됨) · None=사용자 취소(조용히).
        """
        if force:
            return self.ensure_manual(True)
        return self._switch_then(self.ensure_manual, ask=True)

    def ensure_arm_or_switch(self, force: bool = False) -> bool | None:
        if force:
            return self.ensure_arm(True)
        # MAPPING 중 팔 거부는 모드 전환으로 풀리지 않으므로 묻지 않는다 (ensure_arm이 사유 출력)
        self._wait_operation()
        return self._switch_then(self.ensure_arm, ask=self.state.map_mode != 'MAPPING')

    def _switch_then(self, check, ask: bool) -> bool | None:
        # 거부 문구는 전환을 묻기 전이 아니라 최종 판정(check)에서만 찍는다 —
        # 전환에 동의했는데 결과창에 '거부: 현재 조작 모드…'가 남지 않게.
        self._wait_operation()
        if (ask and sys.stdin.isatty() and not self.state.emergency
                and self.state.operation != 'MANUAL'):
            if not self.prompt.confirm('MANUAL 모드로 전환할까요?', default=True,
                                       body=[f'현재 조작 모드: {self.state.operation or "알 수 없음"}']):
                return None
            self.cmd_mode(argparse.Namespace(mode='manual'))
        return check(False)

    # -------------------------------------------------------------- 명령 전송

    def cmd_status(self, args) -> int:
        if isinstance(args.diagnostics, str) and args.diagnostics:  # 진단 항목 상세만 (상태 출력 없이)
            return self.print_diag_detail(args.diagnostics)
        # discovery가 느린 Jetson에서도 잡히게: /state/* 가 오거나 5초까지 대기.
        # joint_states(100Hz)가 먼저 와도 state 토픽 매칭까지 기다린다.
        end = time.monotonic() + 5.0
        while time.monotonic() < end and rclpy.ok():
            self._poll(0.2)
            if self.state.values or self.state.devices:
                break
        if self.state.devices:
            self._poll(0.5)  # /state/devices/<이름> 상세

        if not self.state.values and not self.state.devices and not self.state.joints and not self.state.diagnostics:
            print('상태 없음 — bringup이 떠 있는지 확인하세요.', file=sys.stderr)
            return 1

        print(f'조작 모드   : {self.state.operation or "-"}')
        print(f'지도 모드   : {self.state.map_mode or "-"}')
        print(f'지도 이름   : {self.state.values.get("map_name") or "-"}')
        print(f'비상정지    : {"ON" if self.state.emergency else "OFF"}')
        print(f'장치        : {", ".join(self.state.devices) if self.state.devices else "-"}')
        if self.state.devices:
            for name in self.state.devices:
                print(f'  {name:16s} {self.state.device_details.get(name, "(상세 없음)")}')

        maps = [n for n in (self.state.values.get('map_list') or '').splitlines() if n]
        if maps:
            print('지도 목록   :')
            for name in maps:
                print(f'  - {name}')

        restartable = [n for n in (self.state.values.get('restartable') or '').splitlines() if n]
        if restartable:
            print('재시작 가능 : ' + ', '.join(line.split()[0] for line in restartable))

        if self.state.joints:
            wheels = [f'{name}={self.state.joints[name]:.3f}' for name in WHEEL_JOINTS
                      if name in self.state.joints]
            arms = [f'{name}={self.state.joints[name]:+.3f}/{math.degrees(self.state.joints[name]):+.1f}°'
                    for name in (*ARM_JOINTS, GRIPPER_JOINT) if name in self.state.joints]
            if wheels:
                print('바퀴        : ' + '  '.join(wheels))
            if arms:
                print('관절        : ' + '  '.join(arms))
            if not wheels and not arms:
                print('관절        : (알 수 없는 관절명: ' +
                      ', '.join(sorted(self.state.joints)) + ')')

        if args.diagnostics not in (None, False):
            if not self.state.diagnostics:
                print('진단        : /diagnostics_agg 없음')
            elif args.diagnostics in (True, ''):
                print('/diagnostics_agg (상세: status --diagnostics <이름 일부>):')
                for name, (level, message) in sorted(self.state.diagnostics.items()):
                    print(f'  {name}: {DIAG_LEVELS.get(level, level)} {message}')


        if args.topics:
            print('/state/topics:')
            print(self.state.values.get('topics', '(없음)'))
        return 0

    # ------------------------------------------------------------------ mode

    def cmd_mode(self, args) -> int:
        target = args.mode.upper()
        if target == 'EXPLORE' and self.state.map_mode not in ('MAPPING', ''):
            print('WARN: EXPLORE는 지도 모드 MAPPING에서만 허용됩니다', file=sys.stderr)
        self._publish(self.pub_operation, String(data=target))
        if self._wait_value('operation_mode', target, 2.5):
            print(f'조작 모드 → {target}')
            return 0
        print(f'요청 실패: 현재 조작 모드 {self.state.operation or "알 수 없음"} (state_manager 로그 확인)',
              file=sys.stderr)
        return 1

    # ----------------------------------------------------------------- estop

    def cmd_estop(self, args) -> int:
        if args.state == 'status':
            self._poll(0.5)
            print(f'비상정지: {"ON" if self.state.emergency else "OFF"}')
            return 0

        engaged = args.state == 'on'
        self._publish(self.pub_estop, Bool(data=engaged))
        if engaged:
            # 정지 명령을 한 번 더 보낸다 (twist_mux 잠금과 별개로 manual_control watchdog도 즉시)
            self._send_velocity(0.0, 0.0)
        if self._wait_emergency(engaged, 2.0):
            print(f'비상정지 → {"ON" if engaged else "OFF"}')
            return 0
        print(f'요청 실패: 현재 비상정지 {"ON" if self.state.emergency else "OFF"}', file=sys.stderr)
        return 1

    # ------------------------------------------------------------------- drive

    def cmd_drive(self, args) -> int:
        if args.direction == 'stop':
            self._send_velocity(0.0, 0.0)
            print('바퀴 정지 명령')
            return 0
        if args.duration <= 0.0:
            print('거부: --duration은 0보다 커야 합니다', file=sys.stderr)
            return 2
        if not self.ensure_manual_or_switch(args.force):
            return 1
        if self.pub_vel.get_subscription_count() == 0:
            print('WARN: /cmd/manual/cmd_vel 구독자 없음 — manual_control 확인', file=sys.stderr)

        linear = args.speed if args.direction == 'forward' else -args.speed if args.direction == 'back' else 0.0
        angular = args.angular if args.direction == 'left' else -args.angular if args.direction == 'right' else 0.0
        print(f'{args.direction}: linear={linear:.2f} m/s angular={angular:.2f} rad/s '
              f'for {args.duration:.2f}s')
        end = time.monotonic() + args.duration
        try:
            while time.monotonic() < end and rclpy.ok():
                self._send_velocity(linear, angular)
                self._poll(0.05)
        finally:
            self._send_velocity(0.0, 0.0)
            self._poll(0.15)
        return 0

    # ------------------------------------------------------------------ teleop

    def cmd_teleop(self, args) -> int:
        if not self.ensure_manual_or_switch(args.force):
            return 1
        if not sys.stdin.isatty():
            print('거부: teleop은 터미널에서 실행해야 합니다', file=sys.stderr)
            return 2

        from .dialogs import KeyReader
        linear = 0.0
        angular = 0.0
        lin_speed = args.linear
        ang_speed = args.angular
        last_send = 0.0
        key_count = 0
        last_key = '-'
        con = getattr(self.term, 'out', None) or sys.stdout  # 캡처를 우회해 화면에 직접
        # 방향키도 w/x/a/d와 같게
        alias = {'up': 'w', 'down': 'x', 'left': 'a', 'right': 'd', ' ': 's'}
        try:
            with KeyReader(self.term) as keys:  # cbreak + 시작 전 눌린 키 비우기
                while rclpy.ok():
                    cols, rows = self.term.size()
                    w = min(cols - 4, 62)
                    left = max(1, (cols - w) // 2 + 1)
                    top = max(1, (rows - 9) // 2 + 1)
                    wl = self.state.joint_vel.get('wheel_left_joint')
                    wr = self.state.joint_vel.get('wheel_right_joint')
                    actual = (f'실제 좌{wl:+.2f} 우{wr:+.2f} rad/s'
                              if wl is not None and wr is not None else '실제 ? (joint_states 무응답)')
                    estop = ' · E-STOP' if self.state.emergency else ''
                    key_txt = f'키 {last_key} ×{key_count}' if key_count else '키 입력 대기 중'
                    sent_at = time.strftime('%H:%M:%S', time.localtime(last_send)) if last_send else '--:--:--'
                    rows_txt = ['텔레오퍼',
                                f'모드: {self.state.operation or "-"}{estop}',
                                f'명령 v={linear:+.2f} m/s ω={angular:+.2f} rad/s',
                                actual,
                                f'{key_txt} · 전송 {sent_at}',
                                f'설정 선속도 {lin_speed:.2f} m/s',
                                self.term.paint('w/x/a/d·방향키 이동 · s/space 정지 · e/c 속도 · q/Esc 종료', DIM)]
                    con.write(f'\033[{top};{left}H┌' + '─' * (w - 2) + '┐')
                    for i, line in enumerate(rows_txt):
                        con.write(f'\033[{top + 1 + i};{left}H│' + self.term.fit(line, w - 2) + '│')
                    con.write(f'\033[{top + 1 + len(rows_txt)};{left}H└' + '─' * (w - 2) + '┘')
                    con.flush()
                    now = time.monotonic()
                    if now - last_send >= 0.1:
                        self._send_velocity(linear, angular)
                        last_send = now
                    key = keys.read(0.1)
                    self._poll(0.01)
                    if key is None:
                        continue
                    if key in ('q', 'Q', 'esc', 'eof', '\x03'):
                        break
                    key = alias.get(key, key)
                    key_count += 1
                    last_key = 'space' if key == 's' else key
                    if key == 'w':
                        linear, angular = lin_speed, 0.0
                    elif key == 'x':
                        linear, angular = -lin_speed, 0.0
                    elif key == 'a':
                        linear, angular = 0.0, ang_speed
                    elif key == 'd':
                        linear, angular = 0.0, -ang_speed
                    elif key == 's':
                        linear = angular = 0.0
                    elif key == 'e':
                        lin_speed = min(lin_speed + 0.05, 0.50)
                    elif key == 'c':
                        lin_speed = max(lin_speed - 0.05, 0.02)
        finally:
            self._send_velocity(0.0, 0.0)
            self._poll(0.2)
            print('텔레오퍼 종료 — 바퀴 정지')
        return 0

    # --------------------------------------------------------------------- arm
    @staticmethod

    @staticmethod
    def _parse_targets(tokens: list[str]) -> dict[str, float]:
        targets = {}
        for token in tokens:
            if '=' not in token:
                raise ValueError(f'관절 목표는 이름=값 형식: {token}')
            name, raw = token.split('=', 1)
            name = name.strip()
            try:
                value = float(raw)
            except ValueError as exc:
                raise ValueError(f'숫자가 아님: {token}') from exc
            if not math.isfinite(value):
                raise ValueError(f'유한한 숫자가 아님: {token}')
            if name in ('all', 'arm'):
                for joint in ARM_JOINTS:
                    targets[joint] = value
            elif name in ARM_JOINTS:
                targets[name] = value
            else:
                raise ValueError(f'모르는 관절: {name} (사용: {", ".join(ARM_JOINTS)} 또는 all)')
        return targets

    def cmd_reboot(self, force: bool = False) -> int:
        """Hardware Error(과부하·과열 등) 난 DYNAMIXEL 재부팅 — /cmd/manual/reboot_motors (MANUAL, 에러 난 모터만)."""
        if not self.ensure_manual_or_switch(force):
            return 1
        self._poll(2.5)  # /state/devices/<장치> 갱신 (2초 주기)
        before = self.state.motor_errors()
        print('모터 에러: ' + (', '.join(before) if before else '없음 (/state/devices 기준)'))
        if not before:
            print('재부팅할 모터가 없어 보내지 않습니다 (응답 없는 모터는 재부팅 불가 — 12V·배선 확인)')
            return 0
        self._publish(self.pub_reboot, Empty())
        print('재부팅 요청 보냄 → 에러 난 모터만 재부팅, 팔은 천천히 명령 자세로 돌아감')
        end = time.monotonic() + 8.0
        while time.monotonic() < end and rclpy.ok():
            self._poll(0.5)
            if not self.state.motor_errors():
                print('OK: 모터 에러 풀림')
                return 0
        print('FAIL: 아직 에러: ' + ', '.join(self.state.motor_errors()) +
              ' — manual_control 진단 "마지막 모터 재부팅"·기구 걸림 확인', file=sys.stderr)
        return 1

    def cmd_arm(self, args) -> int:
        if args.action == 'reboot':
            return self.cmd_reboot(args.force)
        if args.action == 'show':
            self._wait_joints()
            values = [f'{name}={self._rad_deg(self.state.joints[name])}' for name in ARM_JOINTS if name in self.state.joints]
            if not values:
                print('/joint_states 없음 — bringup/OpenCR 확인', file=sys.stderr)
                return 1
            print('  '.join(values))
            return 0

        if not self.ensure_arm_or_switch(args.force):
            return 1

        if args.action == 'home':
            targets = {name: 0.0 for name in ARM_JOINTS}
        else:
            try:
                targets = self._parse_targets(args.targets)
            except ValueError as exc:
                print(f'거부: {exc}', file=sys.stderr)
                return 2
            if not targets:
                print('거부: 관절 목표가 없습니다 (예: joint1=0.0 joint2=0.4)', file=sys.stderr)
                return 2
            if args.action == 'delta':
                self._wait_joints()
                for name, delta in list(targets.items()):
                    if name not in self.state.joints:
                        print(f'거부: {name} 현재 각을 모름 (/joint_states 대기)', file=sys.stderr)
                        return 1
                    targets[name] = self.state.joints[name] + delta

        # manual_control은 목표에 없는 관절을 /joint_states의 현재 각으로 채운다.
        # 그 관절의 현재 각이 없으면 명령 전체를 보류하므로 여기서 먼저 막는다.
        self._wait_joints()
        missing = [name for name in ARM_JOINTS if name not in targets and name not in self.state.joints]
        if missing:
            print('/joint_states에 없는 팔 관절: ' + ', '.join(missing) +
                  ' — bringup/OpenCR/controller_manager 확인', file=sys.stderr)
            return 1

        self._send_arm(targets)
        print('팔 목표: ' + '  '.join(f'{name}={value:.3f}' for name, value in targets.items()))
        return 0

    # ----------------------------------------------------------------- gripper

    def cmd_gripper(self, args) -> int:
        if args.action == 'show':
            self._wait_joints()
            if GRIPPER_JOINT not in self.state.joints:
                print('/joint_states에 gripper_left_joint 없음', file=sys.stderr)
                return 1
            print(f'{GRIPPER_JOINT}={self.state.joints[GRIPPER_JOINT]:.4f}')
            return 0

        if args.action == 'open':
            position = GRIPPER_OPEN
        elif args.action == 'close':
            position = GRIPPER_CLOSE
        else:
            if args.value is None:
                print('거부: gripper position에는 값이 필요합니다 (예: 0.01)', file=sys.stderr)
                return 2
            try:
                position = float(args.value)
            except ValueError:
                print(f'거부: 숫자가 아님: {args.value}', file=sys.stderr)
                return 2
            if not math.isfinite(position):
                print('거부: 유한한 숫자가 아님', file=sys.stderr)
                return 2
            if not 0.0 <= position <= GRIPPER_OPEN:
                print(f'WARN: gripper 위치 {position:.4f}은 일반 범위(0~{GRIPPER_OPEN}) 밖입니다',
                      file=sys.stderr)

        if not self.ensure_manual_or_switch(args.force):
            return 1
        self._send_gripper(position)
        print(f'그리퍼 목표: {position:.4f} m')
        return 0

    # --------------------------------------------------------------------- map
    def map_names(self) -> list[str]:
        self._poll(0.5)
        maps = [n for n in (self.state.values.get('map_list') or '').splitlines() if n]
        if not maps:
            maps = self.local_maps(self.default_map_dir())
        return maps

    @staticmethod
    def map_args(action: str, name: str | None = None, file: str | None = None):
        return argparse.Namespace(
            action=action, name=name, file=file, map_dir=None,
            format='pgm', topic='/map')

    @staticmethod
    def default_map_dir(explicit: str | None = None) -> Path:
        if explicit:
            return Path(explicit).expanduser()
        env = os.environ.get('BEEBOT_MAP_DIR')
        if env:
            return Path(env).expanduser()
        # tool/cli/operator_tool.py → lv2_module6/ros2_ws/map
        return Path(__file__).resolve().parents[2] / 'ros2_ws' / 'map'

    @staticmethod

    @staticmethod
    def local_maps(map_dir: Path) -> list[str]:
        if not map_dir.is_dir():
            return []
        return sorted(p.name for p in map_dir.iterdir()
                      if p.is_dir() and (p / 'rtabmap.db').is_file())

    def cmd_map(self, args) -> int:
        map_dir = self.default_map_dir(args.map_dir)

        if args.action == 'status':
            self._poll(0.6)
            print(f'지도 모드: {self.state.map_mode or "-"}')
            print(f'지도 이름: {self.state.values.get("map_name") or "-"}')
            maps = [n for n in (self.state.values.get('map_list') or '').splitlines() if n]
            if not maps:
                maps = self.local_maps(map_dir)
            print('지도 목록: ' + (', '.join(maps) if maps else '-'))
            return 0

        if args.action == 'list':
            self._poll(0.5)
            maps = [n for n in (self.state.values.get('map_list') or '').splitlines() if n]
            if not maps:
                maps = self.local_maps(map_dir)
            if maps:
                for name in maps:
                    print(name)
            else:
                print(f'지도 없음 ({map_dir})', file=sys.stderr)
            return 0

        if args.action == 'mode':
            target = args.name.upper() if args.name else ''
            if target not in ('MAPPING', 'LOCALIZATION'):
                print('거부: map mode는 mapping 또는 localization', file=sys.stderr)
                return 2
            self._poll(0.5)
            if self.state.map_mode == 'COURSE':
                # COURSE엔 RTAB-Map이 없어 전환 불가 — 재시작으로 갈아탐
                return self._map_restart_for(target, None)
            self._send_map_mode(target)
            if self._wait_value('map_mode', target, 4.0):
                print(f'지도 모드 → {target}')
                return 0
            print(f'요청 실패: 현재 지도 모드 {self.state.map_mode or "알 수 없음"} '
                  '(RTAB-Map이 꺼져 있거나 전환 중)', file=sys.stderr)
            return 1

        if args.action == 'switch':
            if not args.name:
                print('거부: map switch에는 지도 이름이 필요합니다', file=sys.stderr)
                return 2
            self._poll(0.5)
            if self.state.map_mode == 'COURSE':
                # COURSE엔 RTAB-Map이 없어 갈아끼우기 불가 — 재시작으로 갈아탐
                return self._map_restart_for('LOCALIZATION', args.name)
            self._send_map_name(args.name)
            if self._wait_value('map_name', args.name, 5.0):
                print(f'지도 → {args.name}')
                return 0
            print(f'요청 실패: 현재 지도 {self.state.values.get("map_name") or "-"} '
                  '(없는 지도이거나 RTAB-Map/state_manager 확인)', file=sys.stderr)
            return 1

        if args.action == 'save':
            return self._map_save(args, map_dir)

        print(f'모르는 map action: {args.action}', file=sys.stderr)
        return 2

    def _map_restart_for(self, mode: str, name: str | None) -> int:
        """COURSE에서 MAPPING/LOCALIZATION·지도 전환 요청이 오면 launch 재시작으로 갈아탐.
        (RTAB-Map이 꺼져 있어 /cmd/map_mode·/cmd/map_name이 통하지 않음)"""
        if mode == 'MAPPING':
            launch_args, desc = 'map:=slam', 'MAPPING(스캔)'
        elif name:
            launch_args, desc = f'map:=saved map_name:={name}', f'LOCALIZATION(지도 {name})'
        else:
            launch_args, desc = 'map:=saved', 'LOCALIZATION(최근 지도)'
        if not self.prompt.confirm('재시작할까요?', default=False, body=[
                '지금은 COURSE(정지 지도)라 바로 전환이 안 됩니다.',
                f'launch 재시작으로 {desc} 모드로 갈아탑니다: all {launch_args}']):
            return 1
        return self._map_do_restart(launch_args)

    def _map_do_restart(self, launch_args: str) -> int:
        # cmd_restart는 target만 받으므로 full 텍스트를 직접 보낸다
        from std_msgs.msg import String as _String
        pub = self.node.create_publisher(_String, '/cmd/restart', 10)
        self._poll(0.2)
        msg = _String()
        msg.data = f'all {launch_args}'
        pub.publish(msg)
        print(f'요청: /cmd/restart "{msg.data}" (재시작 중이면 /state/restart 확인)')
        return 0

    def _map_save(self, args, map_dir: Path) -> int:
        if SaveMapSrv is None:
            print('거부: nav2_msgs가 없어 map save를 사용할 수 없습니다', file=sys.stderr)
            return 2
        name = self.state.values.get('map_name') or ''
        if args.file:
            path = Path(args.file).expanduser()
        else:
            if not name:
                print('거부: 현재 지도 이름이 없습니다 — --file로 경로를 지정하세요', file=sys.stderr)
                return 2
            path = map_dir / name / 'map'
        if path.suffix.lower() in ('.yaml', '.pgm'):
            path = path.with_suffix('')
        path = path.resolve()
        path.parent.mkdir(parents=True, exist_ok=True)

        client = self.node.create_client(SaveMapSrv, '/map_saver/save_map')
        if not client.wait_for_service(timeout_sec=2.0):
            print('거부: /map_saver/save_map 서비스 없음 (RTAB-Map/지도 저장 서버 확인)', file=sys.stderr)
            self.node.destroy_client(client)
            return 1

        req = SaveMapSrv.Request()
        req.map_topic = args.topic
        req.map_url = path.as_uri()
        req.image_format = args.format
        req.map_mode = 'trinary'
        req.free_thresh = 0.25
        req.occupied_thresh = 0.65
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        if not future.done():
            print('요청 실패: /map_saver/save_map 응답 없음', file=sys.stderr)
            self.node.destroy_client(client)
            return 1
        response = future.result()
        self.node.destroy_client(client)
        if not response or not response.result:
            print('요청 실패: map_saver가 저장하지 못했습니다', file=sys.stderr)
            return 1
        print(f'지도 저장 → {path}.pgm · {path}.yaml')
        return 0

    # ----------------------------------------------------------------- restart

    def cmd_restart(self, args) -> int:
        if not args.yes:
            print('거부: 재시작은 전체 스택에 영향을 줍니다 — 확인 후 --yes를 붙이세요',
                  file=sys.stderr)
            return 2
        data = 'all' if args.target == 'all' else f'node {args.target}'
        self._publish(self.pub_restart, String(data=data))
        print(f'재시작 요청: {data}')
        return 0

    # ------------------------------------------------------------------- menu

    def param_nodes(self) -> list[tuple[str, str]]:
        """파라미터 변경 대상 노드 목록 — 디스커버리 + 토픽 발행자 역추적 합침.
        (DDS 디스커버리가 깨진 환경에서도 토픽으로 찾은 노드는 나온다)"""
        found: dict[tuple[str, str], None] = {}
        try:
            pairs = self.node.get_node_names_and_namespaces()
            for n, ns in pairs:
                key = (str(n), str(ns or '/'))
                if key[0] != self.node.get_name():
                    found[key] = None
        except Exception:
            try:
                for n in self.node.get_node_names():
                    key = (str(n), '/')
                    if key[0] != self.node.get_name():
                        found[key] = None
            except Exception:
                pass
        me = self.node.get_name()
        for topic in [t.strip().split(' ')[0]
                      for t in (self.state.values.get('topics') or '').replace('·', ',').split(',')]:
            if not topic.startswith('/'):
                continue
            try:
                infos = self.node.get_publishers_info_by_topic(topic)
            except Exception:
                continue
            for info in infos or []:
                try:
                    key = (str(info.node_name), str(info.node_namespace or '/'))
                except Exception:
                    continue
                if key[0] != me:
                    found[key] = None
        return sorted(found)

    def param_client(self, node_name: str, node_ns: str):
        """대상 노드의 파라미터 클라이언트 (없으면 None). 재사용 캐시."""
        try:
            from rclpy.parameter_client import AsyncParameterClient
        except ImportError:
            return None
        key = (node_name, node_ns)
        cli = self.state.param_clients.get(key)
        if cli is None:
            try:
                # Lyrical AsyncParameterClient는 namespace 인자가 없고
                # 원격 노드 FQN('/ns/이름')으로 서비스명을 만든다.
                fqn = f'/{node_name}' if node_ns in ('/', '') else f'{node_ns}/{node_name}'
                cli = AsyncParameterClient(self.node, fqn)
            except Exception:
                return None
            self.state.param_clients[key] = cli
        return cli

    def param_target_nodes(self, entries: list[str]) -> list[tuple[str, str]]:
        """장치 세부 항목(토픽/관절)에서 파라미터 대상 노드 찾기.

        토픽 발행자를 역추적한다. 관절 이름만 있으면 /joint_states 발행자로.
        """
        topics = [e for e in entries if isinstance(e, str) and e.startswith('/')]
        if not topics:
            topics = ['/joint_states']
        found: list[tuple[str, str]] = []
        seen = set()
        me = self.node.get_name()
        for topic in topics:
            try:
                infos = self.node.get_publishers_info_by_topic(topic)
            except Exception:
                continue
            for info in infos or []:
                key = (info.node_name, info.node_namespace or '/')
                if key in seen or info.node_name == me:
                    continue
                seen.add(key)
                found.append(key)
        return found

    def param_wait(self, future, timeout: float = 5.0):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            return None
        try:
            return future.result()
        except Exception:
            return None

    def param_list_names(self, cli) -> list[str] | None:
        try:
            res = self.param_wait(cli.list_parameters())
        except Exception:
            return None
        if res is None or res.result is None:
            return None
        try:
            return sorted(res.result.names)
        except Exception:
            return None

    def param_describe(self, cli, names: list[str]):
        try:
            res = self.param_wait(cli.describe_parameters(names))
        except Exception:
            return None
        if res is None or getattr(res, 'descriptors', None) is None:
            return None
        return list(res.descriptors)

    def param_get_vals(self, cli, names: list[str]):
        try:
            res = self.param_wait(cli.get_parameters(names))
        except Exception:
            return None
        if res is None or getattr(res, 'values', None) is None:
            return None
        return list(res.values)

    def param_act_reset(self, node_name: str, node_ns: str) -> int | None:
        """yaml 기본값으로 되돌리기 — config 파일의 ros__parameters를 그대로 올림.
        중첩 dict 등 rclpy 파라미터에 못 들어가는 값은 건너뛴다."""
        from . import configref as _cfg
        path, defaults = _cfg.node_defaults(node_name)
        if path is None:
            print(f'거부: {node_name}의 config yaml을 못 찾음 (패키지 config/*.yaml 확인)', file=sys.stderr)
            return 1
        if not defaults:
            print(f'거부: {path}에 {node_name} 기본값 없음', file=sys.stderr)
            return 1
        cli = self.param_client(node_name, node_ns)
        if cli is None or not cli.wait_for_services(timeout_sec=5.0):
            print('거부: 파라미터 서비스 무응답', file=sys.stderr)
            return 1
        params = []
        skipped = []
        for k, v in defaults.items():
            if isinstance(v, (bool, int, float, str)) or (
                    isinstance(v, list) and all(isinstance(x, (bool, int, float, str)) for x in v)):
                params.append(Parameter(k, v))
            else:
                skipped.append(k)
        if not params:
            print('거부: 올릴 수 있는 값이 없음', file=sys.stderr)
            return 1
        body = [f'파일: {path} ({len(defaults)}개)']
        if skipped:
            body.append(f'건너뜀 (복합 타입): {", ".join(skipped)}')
        if not self.prompt.confirm(f'{node_name} 파라미터 {len(params)}개를 yaml 기본값으로 되돌릴까요?',
                                   default=False, body=body):
            return None  # Esc/아니오=그냥 닫기
        try:
            res = self.param_wait(cli.set_parameters(params))
        except Exception as exc:
            print(f'거부: 설정 호출 실패: {exc}', file=sys.stderr)
            return 1
        ok = True
        if res is not None and getattr(res, 'results', None):
            for r in res.results:
                if not r.successful:
                    ok = False
                    print(f'  실패: {getattr(r, "reason", "?")}', file=sys.stderr)
        print(f'  {"yaml 기본값으로 복원 완료" if ok else "일부 실패 (위 참조)"}')
        return 0 if ok else 1

    @staticmethod
    def status_args(diagnostics: bool = False, topics: bool = False):
        return argparse.Namespace(diagnostics=diagnostics, topics=topics)

    def print_diag_detail(self, text: str) -> int:
        """status --diagnostics <이름 일부> — 이름에 그 글자가 든 진단 항목의 레벨·메시지·값 전부"""
        end = time.monotonic() + 5.0  # /diagnostics_agg는 1초마다 — discovery 느린 Jetson 고려
        while time.monotonic() < end and rclpy.ok():
            self._poll(0.2)
            if any(text.lower() in n.lower() for n in self.state.diagnostics):
                break
        names = [n for n, _, _ in self.diag_entries() if text.lower() in n.lower()]
        if not names:
            print(f'진단: "{text}"이 들어간 항목 없음 (/diagnostics_agg {len(self.state.diagnostics)}개)', file=sys.stderr)
            return 1
        for name in names:
            print('\n'.join(self.diag_detail_lines(name, color=sys.stdout.isatty())) + '\n')
        return 0

    @staticmethod

    @staticmethod
    def diag_short(name: str) -> str:
        """/diagnostics_agg 이름 '/Beebot/계획·임무/signal_decoder: 상태' → '계획·임무 / signal_decoder: 상태'"""
        parts = [p for p in name.split('/') if p and p != 'Beebot']
        return ' / '.join(parts) or name

    def diag_entries(self) -> list[tuple[str, int, str]]:
        """진단 항목 (문제 있는 것 먼저 · 같은 레벨이면 이름순) — 묶음 줄('/Beebot/인지'처럼 ': '가 없는 것)은 뒤로"""
        rows = [(n, lv, m) for n, (lv, m) in self.state.diagnostics.items()]
        return sorted(rows, key=lambda r: (': ' not in r[0], -r[1], r[0]))

    def diag_detail_lines(self, name: str, color: bool = True) -> list[str]:
        """진단 항목 하나의 상세 — 레벨 · 메시지 · key-value 전부"""
        level, message = self.state.diagnostics.get(name, (None, ''))
        tag = DIAG_LEVELS.get(level, '없음 (사라짐)')
        if color:
            tag = self.term.paint(tag, GREEN if level == 0 else YELLOW if level == 1 else RED + BOLD if level == 2 else DIM)
        lines = [f'[{self.diag_short(name)}]', f' 레벨: {tag}', f' 메시지: {message or "-"}']
        values = self.state.diag_values.get(name, [])
        if values:
            width = min(28, max(self.term.disp_len(k) for k, _ in values))  # 한글은 두 칸 — 화면 폭으로 맞춤
            lines.append(' 값:')
            lines += [f'  {k}{" " * max(0, width - self.term.disp_len(k))}  {v}' for k, v in values]
        else:
            lines.append(' 값: -')
        return lines

    def show_operation_state(self):
        self._poll(0.5)
        print(f'조작 모드 : {self.state.operation or "-"}')
        print(f'지도 모드 : {self.state.map_mode or "-"}')
        print(f'지도 이름 : {self.state.values.get("map_name") or "-"}')
        print(f'비상정지  : {"ON" if self.state.emergency else "OFF"}')
        maps = [n for n in (self.state.values.get('map_list') or '').splitlines() if n]
        if maps:
            print('지도 목록 : ' + ', '.join(maps))
        else:
            print('지도 목록 : -')

    def show_joints(self):
        self._poll(0.5)
        found = False
        for name in (*WHEEL_JOINTS, *ARM_JOINTS, GRIPPER_JOINT):
            if name in self.state.joints:
                deg = '' if name in WHEEL_JOINTS else f'  ({math.degrees(self.state.joints[name]):+.1f} deg)'
                print(f'  {name:18s} {self.state.joints[name]: .4f} rad{deg}')
                found = True
        if not found:
            print('/joint_states에 팔 관절이 없습니다 — bringup/OpenCR 확인', file=sys.stderr)
