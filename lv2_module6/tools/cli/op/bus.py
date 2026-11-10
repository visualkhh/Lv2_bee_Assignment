"""로봇 동작 전부 — 발행·전송·대기·게이트·CLI 명령 실행.

node=rclpy 노드, state=StateStore, prompt=Ui 입력, term=Terminal.
"""
from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.parameter import Parameter

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Int32MultiArray, String

from .dialog import TeleopDialog
from .pages.ab.core import AbTest
from .consts import (ARM_JOINTS, DIAG_LEVELS, GRIPPER_CLOSE, GRIPPER_JOINT,
                    GRIPPER_OPEN, WHEEL_JOINTS)
from .terminal import BOLD, DIM, GREEN, RED, YELLOW, Terminal

try:
    from nav2_msgs.srv import SaveMap as SaveMapSrv
    from nav2_msgs.srv import Toggle as ToggleSrv
except ImportError:  # nav2_msgs가 없는 환경에서도 나머지 명령은 동작
    SaveMapSrv = None
    ToggleSrv = None


# 수동 주행 속도 상한 (텔레오퍼 -/= · 메뉴 속도 설정) — Waffle Pi 사양 (바퀴 상한 ≈ 8 rad/s × 0.033 m)
class Bus:
    def __init__(self, node, state, prompt, term: Terminal):
        self.node = node
        self.state = state
        self.prompt = prompt
        self.term = term
        self.linear = 0.15
        self.grip_current = 0.6  # 그리퍼 메뉴의 쥐는 힘 한계 [A] (controllers.yaml max_effort와 같게 시작)
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
        self.pub_beep = node.create_publisher(Int32MultiArray, "/cmd/beep", 10)
        self.ab = AbTest(self)

    def _poll(self, seconds: float = 0.2):
        """콜백을 잠깐 돌려 상태를 갱신한다."""
        end = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=min(0.05, max(0.01, end - time.monotonic())))

    def _publish(self, publisher, msg, wait_s: float = 3.0):
        """명령 토픽 publish — discovery가 끝날 때까지 기다린다.

        예전엔 0.2초만 기다리고 구독자가 0이어도 그냥 publish해서, launch 재시작 직후
        `/cmd/map_mode` 같은 한 번짜리 명령이 state_manager에 닿지 않고 사라졌다.
        QoS는 reliable이어도 **매칭 전 sample은 reliable로 재전송되지 않는다** — 먼저 매칭을 기다린다.
        """
        end = time.monotonic() + max(0.0, wait_s)
        count = 0
        while time.monotonic() < end and rclpy.ok():
            count = publisher.get_subscription_count()
            if count > 0:
                break
            self._poll(0.1)
        if count == 0:
            print(f'WARN: {publisher.topic_name} 구독자 없음 — bringup이 떠 있는지 확인', file=sys.stderr)
        publisher.publish(msg)
        self._poll(0.2)

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

    def _send_gripper(self, position: float, current: float | None = None):
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = [GRIPPER_JOINT]
        msg.position = [float(position)]
        if current is not None:  # effort = 쥐는 힘 한계 [A] → manual_control → 그리퍼 액션 goal effort → 펌웨어 C 9번째
            msg.effort = [float(current)]
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

    def _waiting(self, what: str, end: float, total: float):
        """처리 중 창에 지금 기다리는 대상과 경과 (메뉴 밖이면 무시 — base.py 규약)."""
        self.prompt.progress(f'{what} … {total - max(0.0, end - time.monotonic()):.0f}/{total:.0f}초')

    def _wait_value(self, key: str, expected: str, timeout: float = 5.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            self._waiting(f'state_manager가 {key} = {expected}로 바꾸길 기다리는 중', end, timeout)
            self._poll(0.1)
            if self.state.values.get(key, '').upper() == expected.upper():
                return True
        return False

    def _wait_emergency(self, expected: bool, timeout: float = 2.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            self._waiting(f'비상정지 {"ON" if expected else "OFF"} 반영 기다리는 중', end, timeout)
            self._poll(0.1)
            if self.state.emergency == expected:
                return True
        return False

    # -------------------------------------------------------------- 상태/안전
    def _wait_joints(self, timeout: float = 6.0) -> bool:
        """/joint_states가 올 때까지 대기 (Jetson discovery 느림 대응)."""
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            self._waiting('/joint_states 기다리는 중 (bringup·OpenCR)', end, timeout)
            self._poll(0.2)
            if self.state.joints:
                return True
        return bool(self.state.joints)

    def _wait_operation(self) -> None:
        # discovery가 느린 Jetson에서도 state 토픽을 놓치지 않게 최대 8초 대기
        # (status는 5초. 모드 gate는 더 길게 — 못 보면 거부라 재시도만 하면 됨)
        end = time.monotonic() + 8.0
        while time.monotonic() < end and rclpy.ok():
            if not self.state.operation:
                self._waiting('/state/operation_mode 기다리는 중 (state_manager)', end, 8.0)
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
            self._waiting('/state/* 기다리는 중 (state_manager)', end, 5.0)
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
            def amp(name):
                # 모터 전류 [A] — /joint_states effort. 없으면 빈 문자열 (예전 펌웨어).
                a = self.state.joint_effort.get(name)
                return '' if a is None or a != a else f' {a:+.2f}A'
            wheels = [f'{name}={self.state.joints[name]:.3f}{amp(name)}' for name in WHEEL_JOINTS
                      if name in self.state.joints]
            arms = [f'{name}={self.state.joints[name]:+.3f}/{math.degrees(self.state.joints[name]):+.1f}°{amp(name)}'
                    for name in (*ARM_JOINTS, GRIPPER_JOINT) if name in self.state.joints]
            if wheels:
                print('바퀴        : ' + '  '.join(wheels))
            if arms:
                print('관절        : ' + '  '.join(arms))
            if not wheels and not arms:
                print('관절        : (알 수 없는 관절명: ' +
                      ', '.join(sorted(self.state.joints)) + ')')
        print(f'배터리      : {self.state.battery_summary(long=True)}')

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

    # --------------------------------------------------------------- collision_monitor

    def collision_status(self) -> str:
        """manual_control이 마지막으로 반영한 collision_monitor 상태 (진단 값) — 모르면 '-'."""
        for kvs in self.state.diag_values.values():
            for k, v in kvs:
                if k == 'collision_monitor':
                    return v
        return '-'

    def cmd_collision(self, args) -> int:
        """collision_monitor 켜기/끄기 — /collision_monitor/toggle. 모드가 바뀌면(MANUAL·MAPPING = 끔 · 그 밖 = 켬)
        manual_control이 다시 정한다. 그 전까지는 이 명령이 정한 대로 유지된다."""
        if args.state == 'status':
            self._poll(1.0)
            print(f'collision_monitor: {self.collision_status()}')
            return 0
        if ToggleSrv is None:
            print('거부: nav2_msgs가 없어 collision 토글을 쓸 수 없습니다', file=sys.stderr)
            return 2
        client = self.node.create_client(ToggleSrv, '/collision_monitor/toggle')
        try:
            if not client.wait_for_service(timeout_sec=3.0):
                print('거부: /collision_monitor/toggle 없음 — Nav2(collision_monitor)가 떠 있는지 확인 (지도 모드 NONE이면 안 뜸)', file=sys.stderr)
                return 1
            req = ToggleSrv.Request()
            req.enable = args.state == 'on'
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            res = future.result() if future.done() else None
        finally:
            self.node.destroy_client(client)
        if res is None:
            print('요청 실패: collision_monitor 응답 없음', file=sys.stderr)
            return 1
        if not res.success:
            print(f'거절: {res.message}', file=sys.stderr)
            return 1
        print(f'collision_monitor → {"켬" if args.state == "on" else "끔 (근접 정지 없이 통과 — 주의)"}')
        return 0

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

    def latest_calibration_csvs(self, n: int = 2) -> list[Path]:
        root = Path(__file__).resolve().parents[2] / 'artifacts' / 'calibration'
        files = sorted(root.glob('*/raw.csv'), key=lambda p: p.stat().st_mtime, reverse=True)
        return files[:n]

    def calibration_report(self, args) -> int:
        script = Path(__file__).resolve().parent / 'pages' / 'calibration' / 'wheel' / 'report.py'
        if not script.is_file():
            print(f'FAIL: 리포트 스크립트 없음: {script}', file=sys.stderr)
            return 2
        csvs = [Path(p).expanduser() for p in (getattr(args, 'csv', None) or [])]
        if not csvs:
            csvs = self.latest_calibration_csvs(2)
        if not csvs:
            print('FAIL: raw.csv 없음 — 먼저 `run-cli calibrate wheel`', file=sys.stderr)
            return 2
        if len(csvs) > 2:
            print('FAIL: CSV는 1개 또는 2개만', file=sys.stderr)
            return 2
        cmd = [sys.executable, str(script), *map(str, csvs)]
        if getattr(args, 'labels', None):
            cmd += ['--labels', *args.labels]
        if getattr(args, 'out', None):
            cmd += ['--out', str(Path(args.out).expanduser())]
        res = subprocess.run(cmd, capture_output=True, text=True)
        text = ((res.stdout or '') + (res.stderr or '')).rstrip()
        if text:
            print(text)
        return int(res.returncode)

    def cmd_calibrate(self, args) -> int:
        """calibrate <부분> — wheel: 계획 파일을 /cmd/manual/cmd_vel로 재생하며 응답을 raw.csv로 기록 (op/pages/calibration/core.py). arm은 나중."""
        if args.part == 'report':
            return self.calibration_report(args)
        if args.part != 'wheel':
            print(f'거부: calibrate {args.part} — 아직 없음 (wheel만)', file=sys.stderr)
            return 2
        from .pages.calibration.core import ensure_stow, load_plan, run
        try:
            plan = load_plan(args.plan)
        except ValueError as exc:
            print(f'거부: {exc}', file=sys.stderr)
            return 2
        if not self.ensure_manual_or_switch(args.force):
            return 1
        if not ensure_stow(self, move=not args.check_only) and not args.check_only:
            return 1   # 팔이 시작 자세가 아니면 먼저 보낸다 (check-only는 보기만)
        out = Path(args.out) if args.out else Path(__file__).resolve().parents[2] / 'artifacts' / 'calibration' / time.strftime('%Y-%m-%d_%H-%M-%S_wheel') / 'raw.csv'  # tools/artifacts/calibration/<시각>_wheel/raw.csv
        return run(self, plan, out, args.check_only)

    # ------------------------------------------------------------------ teleop

    def cmd_teleop(self, args, on_mark=None) -> int:
        """키보드 수동 주행 — 창·키 처리는 TeleopDialog. 끝나면(어떻게 끝나든) 바퀴를 세운다.
        on_mark: 있으면 g/Enter에 로봇을 세우고 on_mark() 호출 — 반환 문자열을 창에 표시 (A/B 경로 goal 찍기)."""
        if not self.ensure_manual_or_switch(args.force):
            return 1
        if not sys.stdin.isatty():
            print('거부: teleop은 터미널에서 실행해야 합니다', file=sys.stderr)
            return 2
        try:
            TeleopDialog(self, args.linear, args.angular, on_mark).run()
        finally:
            for _ in range(3):
                self._send_velocity(0.0, 0.0)
                self._poll(0.05)
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
            self._waiting('모터 재부팅 뒤 에러가 풀리길 기다리는 중', end, 8.0)
            self._poll(0.5)
            if not self.state.motor_errors():
                print('OK: 모터 에러 풀림')
                return 0
        print('FAIL: 아직 에러: ' + ', '.join(self.state.motor_errors()) +
              ' — manual_control 진단 "마지막 모터 재부팅"·기구 걸림 확인', file=sys.stderr)
        return 1

    PLAN_YAML = Path(__file__).resolve().parents[3] / 'ros2_ws' / 'src' / 'skills' / 'config' / 'skills.yaml'  # skill_pose가 시작 자세를 가진다

    def cmd_capture(self) -> int:
        """지금 팔·그리퍼 자세를 skills.yaml(skill_pose) 형식으로 보여 준다 — 파일은 안 고침, 사람이 붙여 넣는다."""
        if not self._wait_joints():
            print('/joint_states 없음 — bringup/OpenCR 확인', file=sys.stderr)
            return 1
        self._poll(0.3)  # 최신 값
        missing = [j for j in ARM_JOINTS if j not in self.state.joints]
        if missing:
            print('/joint_states에 없는 팔 관절: ' + ', '.join(missing), file=sys.stderr)
            return 1
        values = [self.state.joints[j] for j in ARM_JOINTS]
        print('  '.join(f'{j}={self._rad_deg(v)}' for j, v in zip(ARM_JOINTS, values)))
        gripper = self.state.joints.get(GRIPPER_JOINT)
        print(f'\n{self.PLAN_YAML} 의 skill_pose: ros__parameters: 아래에 붙여 넣으세요 (bringup 재시작 후 적용)\n')
        print('    initial_pose: [' + ', '.join(f'{v:.4f}' for v in values) + ']')
        if gripper is None:
            print('    # initial_gripper: (/joint_states에 gripper_left_joint 없음)')
        else:
            print(f'    initial_gripper: {gripper:.4f}')
        return 0

    def initial_pose(self) -> tuple[dict[str, float], float | None, str] | None:
        """시작 자세 — 떠 있는 skill_pose 파라미터(지금 적용 중인 값)를 먼저, 없으면 skills.yaml.
        반환: ({관절: rad}, 그리퍼 m 또는 None(음수 = 안 건드림), 출처)"""
        cli = self.param_client('skill_pose', '/')
        if cli is not None and cli.wait_for_services(timeout_sec=2.0):
            vals = self.param_get_vals(cli, ['initial_pose_joints', 'initial_pose', 'initial_gripper'])
            if vals and len(vals) == 3 and list(vals[0].string_array_value) and list(vals[1].double_array_value):
                g = vals[2].double_value
                return (dict(zip(vals[0].string_array_value, vals[1].double_array_value)),
                        g if g >= 0.0 else None, 'skill_pose 파라미터')
        try:
            import yaml
            p = yaml.safe_load(self.PLAN_YAML.read_text())['skill_pose']['ros__parameters']
            g = float(p.get('initial_gripper', -1.0))
            return (dict(zip(p['initial_pose_joints'], map(float, p['initial_pose']))),
                    g if g >= 0.0 else None, str(self.PLAN_YAML))
        except Exception as exc:
            print(f'시작 자세를 못 읽음 (skill_pose 없음 · {self.PLAN_YAML}: {exc})', file=sys.stderr)
            return None

    def cmd_arm(self, args) -> int:
        if args.action == 'reboot':
            return self.cmd_reboot(args.force)
        if args.action == 'capture':
            return self.cmd_capture()
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
        elif args.action == 'initial':
            pose = self.initial_pose()
            if pose is None:
                return 1
            targets, gripper, source = pose
            print(f'시작 자세 ({source}): ' + '  '.join(f'{n}={self._rad_deg(v)}' for n, v in targets.items()) +
                  (f'  그리퍼={gripper:.4f}m' if gripper is not None else ''))
            if gripper is not None:
                self._send_gripper(gripper)
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
        print('팔 목표: ' + '  '.join(f'{name}={self._rad_deg(value)}' for name, value in targets.items()))
        self.warn_controller('arm_controller')
        return 0

    def controller_states(self) -> dict[str, str] | None:
        """controller_manager의 컨트롤러 상태 {이름: active|inactive|…} — 응답 없으면 None."""
        try:
            from controller_manager_msgs.srv import ListControllers
        except ImportError:
            return None
        cli = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        try:
            if not cli.wait_for_service(timeout_sec=2.0):
                return None
            res = self.param_wait(cli.call_async(ListControllers.Request()), 3.0)
            items = getattr(res, 'controller', None)
            return {c.name: c.state for c in items} if items is not None else None
        finally:
            self.node.destroy_client(cli)

    # bringup spawner 순서 (ros_system.launch.py) — 체인은 뒤에서부터 (rl_arm_correction → arm_controller)
    CONTROLLERS = ('joint_state_broadcaster', 'imu_sensor_broadcaster', 'diff_drive_controller',
                   'gripper_controller', 'rl_arm_correction', 'arm_controller')

    def cmd_beep(self, args=None) -> int:
        """OpenCR 부저 — 인자 없으면 기본음(2000 Hz 150 ms), `beep <Hz> <ms>`면 높이·길이 (펌웨어가 100~8000 · 10~2000으로 자름).
        /cmd/beep → BeebotSystemHardware → 펌웨어 B 줄. 모드와 무관."""
        vals = list(getattr(args, 'value', None) or [])
        if vals and len(vals) != 2:
            print('사용: beep 또는 beep <Hz 100~8000> <ms 10~2000> (예: beep 4000 300)', file=sys.stderr)
            return 2
        try:
            data = [int(v) for v in vals]
        except ValueError:
            print(f'숫자가 아님: {" ".join(vals)}', file=sys.stderr)
            return 2
        self._publish(self.pub_beep, Int32MultiArray(data=data))
        print('삑 ' + (f'{data[0]} Hz {data[1]} ms (B,{data[0]},{data[1]})' if data else '기본음 (B)'))
        return 0

    def cmd_recover(self, args=None) -> int:
        """조작이 안 될 때 복구 — ① 모터 Hardware Error면 재부팅 ② 꺼진(inactive) 컨트롤러 다시 켜기.
        관절이 URDF 한계 밖이면 ②가 또 실패한다 → 토크 끄고 손으로 옮긴 뒤 다시."""
        self.prompt.step('1/2 모터 에러 확인')
        self._poll(2.5)  # /state/devices 갱신 (2초 주기)
        if self.state.motor_errors():
            self.cmd_reboot()
        else:
            print('모터 에러 없음')
        self.prompt.step('2/2 컨트롤러 확인')
        states = self.controller_states()
        if states is None:
            print('FAIL: controller_manager 응답 없음 — bringup이 떠 있나? (./run-cli start)', file=sys.stderr)
            return 1
        off = [n for n in self.CONTROLLERS if states.get(n) == 'inactive']
        if off:
            try:
                from controller_manager_msgs.srv import SwitchController
            except ImportError:
                print('controller_manager_msgs 없음', file=sys.stderr)
                return 1
            cli = self.node.create_client(SwitchController, '/controller_manager/switch_controller')
            try:
                req = SwitchController.Request()
                req.activate_controllers = off
                req.strictness = SwitchController.Request.BEST_EFFORT  # 되는 것만이라도
                self.prompt.step('2/2 컨트롤러 켜는 중: ' + ', '.join(off))
                res = self.param_wait(cli.call_async(req), 10.0) if cli.wait_for_service(timeout_sec=2.0) else None
                if res is None:
                    print('FAIL: switch_controller 응답 없음', file=sys.stderr)
            finally:
                self.node.destroy_client(cli)
            states = self.controller_states() or states
        bad = [f'{n} {states.get(n, "없음")}' for n in self.CONTROLLERS if states.get(n) != 'active']
        print('컨트롤러: ' + '  '.join(f'{n}={states.get(n, "없음")}' for n in self.CONTROLLERS))
        if bad:
            print('FAIL: 아직 꺼짐 — ' + ', '.join(bad) + '. 관절이 URDF 한계 밖이면(bringup 로그 "out of bounds") '
                  '팔 → 8 토크 끄기 → 손으로 한계 안으로 → 9 토크 켜기 → 다시 복구.', file=sys.stderr)
            if any(n not in states for n in self.CONTROLLERS):
                print('  "없음" = 로드도 안 됨 (bringup 때 앞 컨트롤러가 실패해 spawner가 멈춤) — 다시 켜기로는 안 됨 → ./run-cli restart',
                      file=sys.stderr)
            return 1
        print('OK: 전부 active — 조작 가능')
        return 0

    def warn_controller(self, name: str):
        """명령을 받을 컨트롤러가 active가 아니면 알림 — 아니면 명령이 조용히 사라진다."""
        states = self.controller_states()
        state = (states or {}).get(name)
        if states is None or state == 'active':
            return
        why = ('로드도 안 됨 (bringup 때 spawner가 앞에서 멈춤) → ./run-cli restart' if state is None else
               '꺼짐 → z) 복구. 관절이 URDF 한계 밖이면(bringup 로그 "out of bounds") 팔 → 8 토크 끄기 → 손으로 옮기기 → 9 토크 켜기 → z) 복구')
        print(f'WARN: {name} {state or "없음"} — 이 명령은 실행되지 않습니다. {why}', file=sys.stderr)

    # ----------------------------------------------------------------- gripper

    def gripper_current_text(self) -> str:
        """그리퍼 모터 전류 — /joint_states effort(A). 쥐는 힘 한계는 펌웨어 config.h GRIPPER_GOAL_CURRENT_MA."""
        a = self.state.joint_effort.get(GRIPPER_JOINT)
        return '전류 ? (effort 없음)' if a is None else f'전류 {a * 1000:.0f} mA'

    def cmd_gripper(self, args) -> int:
        if args.action == 'show':
            self._wait_joints()
            if GRIPPER_JOINT not in self.state.joints:
                print('/joint_states에 gripper_left_joint 없음', file=sys.stderr)
                return 1
            print(f'{GRIPPER_JOINT}={self.state.joints[GRIPPER_JOINT]:.4f} m  {self.gripper_current_text()}')
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
        current = getattr(args, 'current', None)
        self._send_gripper(position, current)
        print(f'그리퍼 목표: {position:.4f} m' + (f' · 쥐는 힘 한계 {current:.2f} A' if current is not None else ' · 쥐는 힘 기본 (max_effort)'))
        self.warn_controller('gripper_controller')
        return 0

    # --------------------------------------------------------------------- map
    def map_names(self) -> list[str]:
        self._poll(0.5)
        maps = [n for n in (self.state.values.get('map_list') or '').splitlines() if n]
        if not maps:
            maps = self.local_maps(self.default_map_dir())
        return maps

    @staticmethod
    def map_args(action: str, name: str | None = None, file: str | None = None,
                 localize: bool | None = None):
        return argparse.Namespace(
            action=action, name=name, file=file, map_dir=None,
            format='pgm', topic='/map', localize=localize)

    @staticmethod
    def default_map_dir(explicit: str | None = None) -> Path:
        if explicit:
            return Path(explicit).expanduser()
        env = os.environ.get('BEEBOT_MAP_DIR')
        if env:
            return Path(env).expanduser()
        # tools/cli/operator_tool.py → lv2_module6/ros2_ws/map
        return Path(__file__).resolve().parents[3] / 'ros2_ws' / 'map'

    @staticmethod

    @staticmethod
    def local_maps(map_dir: Path) -> list[str]:
        if not map_dir.is_dir():
            return []
        return sorted(p.name for p in map_dir.iterdir()
                      if p.is_dir() and (p / 'nvblox.nvblx').is_file())

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
            if self.state.map_mode == 'MAPPING' and self.state.values.get('map_name'):
                print('작성 중   : 실제 nvblox는 LOCALIZATION 전환/종료 때 nvblox.nvblx 저장 (테스트베드 fake SLAM은 계속 씀)')
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
            if target not in ('NONE', 'MAPPING', 'LOCALIZATION'):
                print('거부: map mode는 none, mapping 또는 localization', file=sys.stderr)
                return 2
            self._poll(0.5)
            if self.state.map_mode == target:
                print(f'이미 지도 모드 {target}')
                return 0
            if target == 'LOCALIZATION' and not self.state.values.get('map_name'):
                print('거부: 지금 지도 폴더가 없음 — map switch <이름>으로 고르세요', file=sys.stderr)
                return 2
            return self._map_restart_reply(lambda: self._send_map_mode(target), f'지도 모드 → {target}')

        if args.action == 'switch':
            if not args.name:
                print('거부: map switch에는 지도 이름이 필요합니다', file=sys.stderr)
                return 2
            self._poll(0.5)
            if args.name == self.state.values.get('map_name') and self.state.map_mode == 'LOCALIZATION':
                print(f'이미 지도 {args.name}')
                return 0
            maps = [n for n in (self.state.values.get('map_list') or '').splitlines() if n]
            if maps and args.name not in maps:
                print(f'거부: 지도 없음 "{args.name}" (nvblox.nvblx 있는 폴더만)', file=sys.stderr)
                print('있는 지도: ' + ', '.join(maps), file=sys.stderr)
                return 2
            return self._map_restart_reply(lambda: self._send_map_name(args.name), f'지도 → {args.name}')

        if args.action == 'save':
            self._poll(0.6)   # 일회성 프로세스도 /state/map_mode·map_name을 먼저 받는다
            rc = self._map_save(args, map_dir)
            if rc != 0 or self.state.map_mode != 'MAPPING':
                return rc
            localize = getattr(args, 'localize', None)
            if localize is None:
                if not sys.stdin.isatty():
                    localize = False
                else:
                    try:
                        ans = input('지도 미리보기 저장 완료 — LOCALIZATION으로 전환할까요? '
                                    '(다시 뜨기 — 수십 초) [Y/n] ')
                    except EOFError:
                        ans = 'n'
                    localize = ans.strip().lower() not in ('n', 'no')
            if not localize:
                print('LOCALIZATION 전환 생략 — 실제 nvblox는 나중에 전환/종료할 때 지도를 저장합니다.')
                return 0
            print('LOCALIZATION으로 전환 — 지도 저장 + 다시 뜨기 (수십 초)')
            return self.localize_saved()

        print(f'모르는 map action: {args.action}', file=sys.stderr)
        return 2

    def localize_saved(self) -> int:
        """저장 직후 LOCALIZATION — 방금 다른 이름("wow")으로 저장했으면 그 지도로 (map mode는 *현재 작업 폴더*로 가서 nvblx가 없어 NONE이 됨)."""
        saved = getattr(self, '_saved_name', None)
        if not saved or saved == self.state.values.get('map_name'):
            return self.cmd_map(self.map_args('mode', name='localization'))
        end = time.monotonic() + 5.0   # state_manager의 지도 목록이 갱신될 때까지
        while time.monotonic() < end and saved not in (self.state.values.get('map_list') or '').splitlines():
            self._poll(0.3)
        return self.cmd_map(self.map_args('switch', name=saved))

    def _map_restart_reply(self, send, desc: str, timeout: float = 3.0) -> int:
        """지도 모드·지도 바꾸기 = state_manager가 launch 전체를 다시 띄움 (cuVSLAM·nvblox는 실행 중 전환이 없음).
        끝까지 기다리지 않는다 — nvblox 지도 저장 + 다시 뜨기에 수십 초이고, 그동안 state_manager도 꺼진다.
        /state/restart의 첫 응답("all: 재시작 중 …" | "거절: …")만 보여 준다."""
        self.state.values.pop('restart', None)
        send()
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            self._poll(0.1)
            note = self.state.values.get('restart')
            if note:
                if note.startswith('거절'):
                    print(f'요청 실패: {note}', file=sys.stderr)
                    return 1
                print(f'{desc} — {note}')
                print('다시 뜨는 데 수십 초 걸립니다 (빌드 → 지도 모드 시작). 확인: map status')
                return 0
        print('요청 실패: state_manager 응답 없음 — 거절 이유는 /diagnostics의 state_manager '
              '"지도 모드 전환 실패"에 있음', file=sys.stderr)
        return 1

    def _robot_map_dir(self) -> str:
        """state_manager의 map_dir 파라미터 — map_saver가 도는 쪽(로봇·컨테이너)의 지도 폴더. 못 읽으면 ''."""
        cli = self.param_client('state_manager', '/')
        if cli is None or not cli.wait_for_services(timeout_sec=1.0):
            return ''
        vals = self.param_get_vals(cli, ['map_dir'])
        return str(getattr(vals[0], 'string_value', '') or '') if vals else ''

    def _nvblox_exists(self, path: Path) -> bool:
        """nvblox.nvblx가 있는가 — 도커는 CLI(호스트)와 지도(컨테이너 /ws/map)가 다른 곳에 있다."""
        if path.is_file():
            return True
        # /ws/map/<이름>/nvblox.nvblx → 호스트 lv2_module6/ros2_ws/map/<이름>/nvblox.nvblx (compose 바인드)
        if path.parent.parent == Path('/ws/map'):
            return (self.default_map_dir() / path.parent.name / 'nvblox.nvblx').is_file()
        return False

    def _save_nvblox_map(self, path: Path) -> int:
        """nvblox 정본(.nvblx) 저장 — map_saver의 pgm/yaml과 별개다.

        nvblox_node의 after_shutdown_map_save_path는 종료 때 png/yaml만 쓰고 .nvblx는
        쓰지 않는다. 그래서 서비스 ~/save_map을 명시적으로 불러야 위치추정용 지도가 남는다.
        테스트베드 fake SLAM은 fake_nvblox가 자리 파일을 이미 썼다 — 서비스·메시지가 없으면 그 파일로 성공한다.
        """
        try:
            from nvblox_msgs.srv import FilePath
        except ImportError:
            if self._nvblox_exists(path):
                print(f'가짜 SLAM — 저장 지도 유지: {path}')
                return 0
            print('거부: nvblox_msgs 없음 — ~/isaac_ros_ws/install/setup.bash를 source해야 합니다',
                  file=sys.stderr)
            return 1
        cli = self.node.create_client(FilePath, '/nvblox_node/save_map')
        if not cli.wait_for_service(timeout_sec=5.0):
            self.node.destroy_client(cli)
            if self._nvblox_exists(path):
                print(f'가짜 SLAM — 저장 지도 유지: {path}')
                return 0
            print('거부: /nvblox_node/save_map 없음 — nvblox가 MAPPING 중인지 확인', file=sys.stderr)
            return 1
        req = FilePath.Request()
        req.file_path = str(path)
        future = cli.call_async(req)
        self.prompt.progress('nvblox 지도 저장 중 (최대 30초)')
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        self.node.destroy_client(cli)
        if not future.done():
            print('요청 실패: nvblox save_map 응답 없음', file=sys.stderr)
            return 1
        res = future.result()
        if not res or not getattr(res, 'success', False):
            print('요청 실패: nvblox가 지도를 저장하지 못했습니다', file=sys.stderr)
            return 1
        print(f'nvblox 지도 저장 → {path}')
        return 0

    def _map_save(self, args, map_dir: Path) -> int:
        if SaveMapSrv is None:
            print('거부: nav2_msgs가 없어 map save를 사용할 수 없습니다', file=sys.stderr)
            return 2
        if self.state.map_mode == 'NONE':
            print('거부: 지도 없음(NONE) — 먼저 MAPPING 또는 LOCALIZATION으로 전환하세요', file=sys.stderr)
            return 2
        if self.state.state_last_update <= 0:
            print('거부: /state 통신 없음 — 스택 재연결 후 다시 시도하세요', file=sys.stderr)
            return 1
        age = time.monotonic() - self.state.state_last_update
        if age > 3.0:
            print(f'거부: 스택 연결이 끊겨 있음 (마지막 통신 {age:.0f}s 전) — 재연결 후 다시 시도하세요',
                  file=sys.stderr)
            return 1
        name = self.state.values.get('map_name') or ''
        # `run-cli map save`는 새 프로세스라 /state/map_name이 아직 안 왔을 수 있다.
        # map status처럼 잠깐 기다린다 — 안 기다리면 MAPPING 중인데도 "지도 이름 없음"으로 거부된다.
        if not (args.file or name):
            end = time.monotonic() + 5.0
            while time.monotonic() < end and rclpy.ok():
                self._poll(0.1)
                name = self.state.values.get('map_name') or ''
                if name:
                    break
        # 경로는 map_saver가 쓰는 쪽 기준 — CLI가 원격 PC·컨테이너 밖이면 이 컴퓨터 경로가 로봇에 없다.
        #   --file·--map-dir를 주지 않으면 state_manager의 map_dir을 쓴다 (못 읽을 때만 이 컴퓨터의 ros2_ws/map)
        # 이름만 준 경우("wow") = 기본 지도 폴더의 <이름>/map — 안 그러면 현재 작업 폴더(tools)에 떨어진다
        if args.file and not any(c in args.file for c in '/\\~.'):
            name, args.file = args.file, None
        robot_dir = '' if (args.file or args.map_dir) else self._robot_map_dir()
        if args.file:
            path = Path(args.file).expanduser()
        else:
            if not name:
                print('거부: 현재 지도 이름이 없습니다 — --file로 경로를 지정하세요', file=sys.stderr)
                return 2
            path = Path(robot_dir or map_dir) / name / 'map'
        if path.suffix.lower() in ('.yaml', '.pgm'):
            path = path.with_suffix('')
        if not robot_dir:  # 이 컴퓨터 경로일 때만
            path = path.resolve()
        # 새 이름("wow")의 폴더는 아직 없다 — map_saver는 폴더를 안 만든다. 로봇 위에서 돌 때만 만들어진다(원격 CLI는 건너뜀).
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
        except OSError:
            pass

        client = self.node.create_client(SaveMapSrv, '/map_saver/save_map')
        if not client.wait_for_service(timeout_sec=2.0):
            print('거부: /map_saver/save_map 서비스 없음 (지도 저장 서버는 map:=slam·saved일 때만 뜸)', file=sys.stderr)
            self.node.destroy_client(client)
            return 1

        req = SaveMapSrv.Request()
        req.map_topic = args.topic
        # map_saver는 map_url을 ImageMagick 파일 경로로 그대로 쓴다. file:// URI를 주면
        # ImageMagick이 'file'을 이미지 포맷으로 해석해 "No encode delegate ... (FILE)"로 실패한다.
        req.map_url = str(path)
        req.image_format = args.format
        req.map_mode = 'trinary'
        req.free_thresh = 0.25
        req.occupied_thresh = 0.65
        future = client.call_async(req)
        self.prompt.progress('/map_saver/save_map 응답 기다리는 중 (최대 10초)')
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
        self._saved_name = None if args.file else name   # 이름으로 저장한 폴더 — 전환할 때 이 지도로 간다
        if self.state.map_mode == 'MAPPING':
            # 위치추정 정본은 nvblox.nvblx — map_saver의 pgm/yaml과 별개로 반드시 저장한다.
            rc = self._save_nvblox_map(path.parent / 'nvblox.nvblx')
            if rc != 0:
                print('WARN: nvblox.nvblx 없이 LOCALIZATION으로 가면 지도가 비어 보입니다', file=sys.stderr)
                return rc
        return 0

    # ----------------------------------------------------------------- restart

    def cmd_restart(self, args) -> int:
        data = 'all' if args.target == 'all' else f'node {args.target}'
        self._publish(self.pub_restart, String(data=data))
        print(f'재시작 요청: {data}')
        return 0

    # ------------------------------------------------------------------- node

    LIFECYCLE_STATES = {0: 'unknown', 1: 'unconfigured', 2: 'inactive', 3: 'active', 4: 'finalized'}

    def node_rows(self) -> list[dict]:
        """노드 목록 — 그래프(실행 중 ROS 노드) + state_manager 재시작 목록(pid) 합침.
        lifecycle = <노드>/get_state 서비스가 있으면 True (Nav2 수명주기 노드)."""
        self._poll(0.1)   # 메뉴 루프가 이미 poll(0.2)를 돌린다 — 여기서 길게 막지 않는다
        pids: dict[str, str] = {}
        for line in (self.state.values.get('restartable') or '').splitlines():
            parts = line.split()
            if parts:
                pids[parts[0]] = parts[1] if len(parts) > 1 else '?'
        me = self.node.get_name()
        graph: set[str] = set()
        try:
            for n, ns in self.node.get_node_names_and_namespaces():
                if n != me and not n.startswith('_'):
                    graph.add(n if ns in ('/', '') else f'{ns.strip("/")}/{n}')
        except Exception:
            pass
        try:
            services = {name for name, _ in self.node.get_service_names_and_types()}
        except Exception:
            services = set()
        rows = []
        for name in sorted(graph | set(pids)):
            rows.append({'name': name, 'pid': pids.get(name), 'alive': name in graph,
                         'lifecycle': f'/{name}/get_state' in services})
        return rows

    def lifecycle_state(self, name: str, timeout: float = 2.0) -> str | None:
        """수명주기 노드 상태 (active · inactive · unconfigured …), 응답 없으면 None."""
        try:
            from lifecycle_msgs.srv import GetState
        except ImportError:
            return None
        cli = self.node.create_client(GetState, f'/{name}/get_state')
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                return None
            res = self.param_wait(cli.call_async(GetState.Request()), timeout)
            return self.LIFECYCLE_STATES.get(res.current_state.id, res.current_state.label) if res else None
        finally:
            self.node.destroy_client(cli)

    def lifecycle_activate(self, name: str) -> bool:
        """수명주기 노드를 active까지 — unconfigured면 configure → activate, inactive면 activate.
        lifecycle_manager가 기동 중 응답 시간 초과로 멈춘 경우(bond 없음)를 사람이 이어 주는 용도."""
        try:
            from lifecycle_msgs.msg import Transition
            from lifecycle_msgs.srv import ChangeState
        except ImportError:
            print('lifecycle_msgs 없음', file=sys.stderr)
            return False
        steps = {'unconfigured': [(Transition.TRANSITION_CONFIGURE, 'configure'), (Transition.TRANSITION_ACTIVATE, 'activate')],
                 'inactive': [(Transition.TRANSITION_ACTIVATE, 'activate')], 'active': []}
        state = self.lifecycle_state(name)
        if state not in steps:
            print(f'{name}: 상태 {state or "응답 없음"} — 활성화할 수 없음', file=sys.stderr)
            return False
        cli = self.node.create_client(ChangeState, f'/{name}/change_state')
        try:
            for tid, label in steps[state]:
                req = ChangeState.Request()
                req.transition.id = tid
                self.prompt.step(f'{name} {label} (최대 15초)')
                res = self.param_wait(cli.call_async(req), 15.0)
                if not res or not res.success:
                    print(f'{name}: {label} 실패 ({"시간 초과" if not res else "거절"}) — 그 노드 로그 확인', file=sys.stderr)
                    return False
                print(f'{name}: {label} OK')
        finally:
            self.node.destroy_client(cli)
        print(f'{name}: {self.lifecycle_state(name) or "?"}')
        return True

    def node_feature_lines(self, name: str, summary: bool = False) -> list[str]:
        """노드별 추가 상태 — op/pages/nodes/<node>.py의 NodeFeature."""
        from .pages.nodes import feature_for
        feature = feature_for(name)
        return feature.status_lines(self, name, summary=summary) if feature else []

    def node_services(self, name: str) -> list[tuple[str, str, bool]]:
        """노드가 제공하는 서비스 (이름, 타입, 호출 가능) — 실기 rclpy · 도커는 service_action_bag_input 목록."""
        full = name.strip('/')
        short = full.rsplit('/', 1)[-1]
        ns = '/' + full.rsplit('/', 1)[0] if '/' in full else '/'
        items = []
        try:
            items = self.node.get_service_names_and_types_by_node(short, ns)
        except Exception:
            pass
        if not items:
            try:
                all_items = self.node.get_service_names_and_types()
            except Exception:
                all_items = []
            prefix = f'/{full}/'
            items = [(s, t) for s, t in all_items if s == f'/{full}' or s.startswith(prefix)]
        callable_fn = getattr(self.node, 'service_callable', None)
        rows = []
        for s, types in items:
            callable_ = bool(callable_fn(s)) if callable_fn else True
            rows.append((s, types[0] if types else '', callable_))
        return sorted(set(rows))

    def call_service(self, name: str, type_name: str, request: dict):
        """서비스 호출 — (ok, data dict | error str). 도커는 파일 브리지, 실기는 동적 rclpy 클라이언트."""
        if hasattr(self.node, 'call_service'):  # DockerRos
            return self.node.call_service(name, type_name, request or {})
        try:
            from rosidl_runtime_py.convert import message_to_ordereddict
            from rosidl_runtime_py.set_message import set_message_fields
            from rosidl_runtime_py.utilities import get_service
        except ImportError as exc:
            return False, f'rosidl_runtime_py 없음: {exc}'
        try:
            srv = get_service(type_name)
            cli = self.node.create_client(srv, name)
            try:
                if not cli.wait_for_service(timeout_sec=2.0):
                    return False, f'서버 없음: {name}'
                req = srv.Request()
                set_message_fields(req, request or {})
                res = self.param_wait(cli.call_async(req), timeout=10.0)
                if res is None:
                    return False, '응답 없음 (10초)'
                return True, message_to_ordereddict(res)
            finally:
                self.node.destroy_client(cli)
        except Exception as exc:  # noqa: BLE001 — 타입/필드 오류를 화면에 그대로
            return False, f'서비스 호출 실패: {exc}'

    def node_status_lines(self, name: str) -> list[str]:
        row = next((r for r in self.node_rows() if r['name'] == name), None)
        lines = [f'[{name}]']
        if row is None:
            return lines + [' 그래프에도 재시작 목록에도 없음 (안 떠 있음)']
        lines.append(f' 실행: {"그래프에 보임" if row["alive"] else "그래프에 안 보임 (죽었거나 디스커버리 늦음)"}'
                     f' · pid {row["pid"] or "-"}{"" if row["pid"] else " (state_manager 재시작 목록에 없음)"}')
        if row['lifecycle']:
            lines.append(f' 수명주기: {self.lifecycle_state(name) or "응답 없음"} (active가 아니면 3) 활성화)')
        lines += self.node_feature_lines(name)
        try:
            ns = '/'
            pubs = self.node.get_publisher_names_and_types_by_node(name.split('/')[-1], ns)
            subs = self.node.get_subscriber_names_and_types_by_node(name.split('/')[-1], ns)
            lines.append(f' 발행 {len(pubs)}개 · 구독 {len(subs)}개')
        except Exception:
            pass
        diags = [d for d in self.state.diagnostics if name in d]
        for d in diags[:6]:
            level, message = self.state.diagnostics[d]
            lines.append(f' 진단 {self.diag_short(d)}: {DIAG_LEVELS.get(level, "?")} {message}')
        return lines

    def node_restart(self, name: str) -> int:
        """state_manager로 재시작 → 수명주기 노드면 다시 뜬 뒤 active까지."""
        rows = {r['name']: r for r in self.node_rows()}
        row = rows.get(name)
        if row and not row['pid']:
            print(f'거부: {name}은 state_manager 재시작 목록에 없음 (이 launch 밖 노드)', file=sys.stderr)
            return 1
        old_pid = row['pid'] if row else None
        self.cmd_restart(argparse.Namespace(target=name))
        if not (row and row['lifecycle']):
            return 0
        print(f'{name}: 수명주기 노드 — 다시 뜰 때까지 기다린 뒤 활성화')
        self.prompt.step(f'1/2 {name} 다시 뜨기 기다림')
        end = time.monotonic() + 20.0
        while time.monotonic() < end and rclpy.ok():
            self._waiting(f'{name} 새 프로세스·수명주기 서비스 기다리는 중', end, 20.0)
            self._poll(1.0)
            new = {r['name']: r for r in self.node_rows()}.get(name)
            if new and new['pid'] and new['pid'] != old_pid and self.lifecycle_state(name, 1.0):
                return 0 if self.lifecycle_activate(name) else 1
        print(f'FAIL: {name} 20초 안에 다시 안 뜸 — /tmp/restart_{name}.log 확인', file=sys.stderr)
        return 1

    def cmd_ab(self, args) -> int:
        """A/B 테스트 — op/pages/ab/core.py로 위임 (메뉴 13와 같은 구현)."""
        from .pages.ab.core import AbTest
        return AbTest(self).cmd(args)

    def cmd_node(self, args) -> int:
        if args.action in ('list', 'status'):
            self._poll(0.5)   # 한 번 실행 명령 — 디스커버리 시간을 준다
        if args.action == 'list':
            for r in self.node_rows():
                life = f' [{self.lifecycle_state(r["name"], 1.0) or "?"}]' if r['lifecycle'] else ''
                print(f'{r["name"]:<32} {"●" if r["alive"] else "✗"} pid {r["pid"] or "-":<8}{life}')
            return 0
        if not args.name:
            print('노드 이름이 필요합니다', file=sys.stderr)
            return 2
        if args.action == 'status':
            print('\n'.join(self.node_status_lines(args.name)))
            return 0
        if args.action == 'activate':
            return 0 if self.lifecycle_activate(args.name) else 1
        if args.action == 'restart':
            return self.node_restart(args.name)
        return 2

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
        self.prompt.progress(f'노드 응답 기다리는 중 (최대 {timeout:.0f}초)')
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
            self._waiting('/diagnostics_agg 기다리는 중', end, 5.0)
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
        """/diagnostics_agg 이름 '/Beebot/인지/signal_decoder: 상태' → '인지 / signal_decoder: 상태'"""
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
        self._wait_joints()
        found = False
        for name in (*WHEEL_JOINTS, *ARM_JOINTS, GRIPPER_JOINT):
            if name not in self.state.joints:
                continue
            deg = '' if name in WHEEL_JOINTS else f'  ({math.degrees(self.state.joints[name]):+.1f} deg)'
            a = self.state.joint_effort.get(name)
            amp = '' if a is None or a != a else f'  {a:+.2f} A'
            temp = self.state.motor_temp.get(name, -1)
            temp_txt = '' if temp is None or temp < 0 else f'  {temp}°C'
            err = self.state.motor_text.get(name, '')
            err_txt = '' if err in ('', '정상') else f'  [{err}]'
            print(f'  {name:18s} {self.state.joints[name]: .4f} rad{deg}{amp}{temp_txt}{err_txt}')
            found = True
        if not found:
            print('/joint_states에 팔 관절이 없습니다 — bringup/OpenCR 확인', file=sys.stderr)
