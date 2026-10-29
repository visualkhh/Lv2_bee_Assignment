"""수동 주행·팔·그리퍼 메뉴."""
from __future__ import annotations

import argparse
import math
import sys

from ..consts import ARM_JOINTS, GRIPPER_JOINT, GRIPPER_OPEN
from ..defs import BACK, MAIN, QUIT, STAY
from ..terminal import RED, YELLOW
from .base import GoMenu, Menu, MenuItem


def _finite(raw: str) -> str | None:
    """modal_input 검증 — 유한한 숫자만."""
    try:
        value = float(raw)
    except ValueError:
        return f'숫자가 아님: {raw!r}'
    return None if math.isfinite(value) else '유한한 숫자가 아님'


class TeleopItem(MenuItem):
    key = '1'

    def label(self):
        return '텔레오퍼 (w/x/a/d·방향키 · e/c 속도)'

    def run(self):
        self.bus.cmd_teleop(argparse.Namespace(
            linear=self.bus.linear, angular=self.bus.angular, force=False))
        return STAY


class DriveItem(MenuItem):
    """전진·후진·좌회전·우회전·정지."""

    def __init__(self, app, key: str, direction: str, label: str):
        super().__init__(app)
        self.key = key
        self._direction = direction
        self._label = label

    def label(self):
        return self._label

    def run(self):
        self.bus.cmd_drive(argparse.Namespace(
            direction=self._direction, speed=self.bus.linear,
            angular=self.bus.angular, duration=self.bus.duration, force=False))
        return STAY


class ManualMenu(Menu):
    title = '수동 주행 / 텔레오퍼'

    def items(self):
        return [TeleopItem(self.app),
                DriveItem(self.app, '2', 'forward', '전진'),
                DriveItem(self.app, '3', 'back', '후진'),
                DriveItem(self.app, '4', 'left', '좌회전'),
                DriveItem(self.app, '5', 'right', '우회전'),
                DriveItem(self.app, '6', 'stop', '정지'),
                GoMenu(self.app, '7', '속도/시간 설정', DriveSettingsMenu(self.app))]

    def center(self):
        st = self.state
        return [
            '[수동 주행]',
            f' 모드: {st.operation or "-"}',
            f' E-STOP: {"ON" if st.emergency else "OFF"}',
            f' 선속도 {self.bus.linear:.2f} m/s',
            f' 각속도 {self.bus.angular:.2f} rad/s',
            f' 시간 {self.bus.duration:.2f} s',
            ' 2)~6)은 한 번씩 움직임 · 계속 조종은 1) 텔레오퍼',
        ]


class FloatSettingItem(MenuItem):
    """수동 주행 설정값 한 개 — Enter가 모달 값 입력."""

    def __init__(self, app, key: str, name: str, unit: str, get, set, positive=False):
        super().__init__(app)
        self.key = key
        self._name = name
        self._unit = unit
        self._get = get
        self._set = set
        self._positive = positive

    def label(self):
        return f'{self._name} : {self._get():.2f} {self._unit}'

    def run(self):
        value, ok = self.ui.read_float(f'{self._name} [{self._unit}]', self._get(),
                                       positive=self._positive)
        if ok:
            self._set(value)
        return STAY


class DriveSettingsMenu(Menu):
    title = '수동 주행 설정'

    def items(self):
        b = self.bus
        return [FloatSettingItem(self.app, '1', '선속도', 'm/s',
                                 lambda: b.linear, lambda v: setattr(b, 'linear', v)),
                FloatSettingItem(self.app, '2', '각속도', 'rad/s',
                                 lambda: b.angular, lambda v: setattr(b, 'angular', v)),
                FloatSettingItem(self.app, '3', '주행시간', 's',
                                 lambda: b.duration, lambda v: setattr(b, 'duration', v),
                                 positive=True)]

    def center(self):
        b = self.bus
        return ['[현재 설정]', f' 선속도 {b.linear:.2f} m/s',
                f' 각속도 {b.angular:.2f} rad/s',
                f' 주행시간 {b.duration:.2f} s']


class ArmShowItem(MenuItem):
    key = '1'

    def label(self):
        return '관절 상태'

    def run(self):
        self.bus.cmd_arm(argparse.Namespace(action='show', targets=[], force=False))
        return STAY


class ArmMoveItem(MenuItem):
    """관절 목표 설정(2) / 델타 이동(3)."""

    def __init__(self, app, key: str, delta: bool):
        super().__init__(app)
        self.key = key
        self._delta = delta

    def label(self):
        return '관절 델타 이동' if self._delta else '관절 목표 설정'

    def _pick_joint(self):
        st = self.state
        items = [(n, f'{n} = {st.joints[n]:+.3f} rad' if n in st.joints else n)
                 for n in ARM_JOINTS]
        while True:
            name = self.ui.select('관절 선택', items, ['[관절 고르기]', ' Esc = 취소'])
            if name in ARM_JOINTS:
                return name
            if name in (BACK, MAIN, QUIT):
                return None

    def run(self):
        st = self.state
        if not any(n in st.joints for n in ARM_JOINTS):
            print('/joint_states에 팔 관절이 없습니다 — bringup/OpenCR 확인', file=sys.stderr)
            return STAY
        if not self.bus.ensure_arm_or_switch():
            return STAY
        while True:
            name = self._pick_joint()
            if name is None:
                return STAY
            label = '변화량 [rad]' if self._delta else '목표 [rad]'
            cur = f'현재 {st.joints[name]:+.3f} rad' if name in st.joints else '현재 ?'
            raw = self.ui.modal_input('값 입력', [f'{name} {label}', cur], '값>',
                                      validate=_finite)
            if raw is not None:
                value = float(raw)
                break
            # Esc = 값 입력만 닫고 관절 선택으로 복귀
        action = 'delta' if self._delta else 'set'
        self.bus.cmd_arm(argparse.Namespace(
            action=action, targets=[f'{name}={value:g}'], force=False))
        return STAY


class ArmHomeItem(MenuItem):
    key = '4'

    def label(self):
        return '홈 (모든 팔 관절 0)'

    def run(self):
        if not self.bus.ensure_arm_or_switch():
            return STAY
        if not self.ui.confirm('모든 팔 관절을 0으로 이동할까요?', default=False,
                               body=['현재 자세와 주변을 확인하세요.',
                                     'joint2가 URDF 한계 밖이면 홈 이동이 위험합니다.']):
            return STAY
        self.bus.cmd_arm(argparse.Namespace(action='home', targets=[], force=False))
        return STAY


class ArmInitialItem(MenuItem):
    key = '7'

    def label(self):
        return '시작 자세로 (plan.yaml initial_pose · initial_gripper)'

    def run(self):
        if not self.bus.ensure_arm_or_switch():
            return STAY
        pose = self.bus.initial_pose()
        if pose is None:
            return STAY
        targets, gripper, source = pose
        body = [f'출처: {source}', '  '.join(f'{n}={v:+.3f}' for n, v in targets.items())]
        if gripper is not None:
            body.append(f'그리퍼 {gripper:.4f} m')
        body.append('현재 자세와 주변을 확인하세요 (MoveIt 없이 직선 보간 — 충돌 회피 안 함).')
        if not self.ui.confirm('시작 자세로 이동할까요?', default=False, body=body):
            return STAY
        self.bus.cmd_arm(argparse.Namespace(action='initial', targets=[], force=False))
        return STAY


class ArmRebootItem(MenuItem):
    key = '5'

    def label(self):
        return '모터 재부팅 (과부하 등 Hardware Error 풀기)'

    def run(self):
        self.bus.cmd_reboot()
        return STAY


class ArmPoseCaptureItem(MenuItem):
    key = '6'

    def label(self):
        return '손으로 자세 잡기(TEACH) → 시작 자세 값 보기'

    def run(self):
        if self.state.operation != 'TEACH':
            if not self.ui.confirm('TEACH 모드로 바꿀까요? (팔·그리퍼 토크 끔)', default=False,
                                   body=['토크가 꺼지면 팔이 처집니다 — 손으로 잡을 준비를 하세요.',
                                         '바퀴는 그대로입니다.']):
                return STAY
            if self.bus.cmd_mode(argparse.Namespace(mode='teach')) != 0:
                return STAY
        self.ui.confirm('손으로 팔·그리퍼 자세를 잡은 뒤 확인을 누르세요', default=True,
                        body=['지금 각을 읽고 바로 MANUAL로 바꿔 그 자리에서 토크를 켭니다.',
                              '다 잡을 때까지 팔을 놓지 마세요.'])
        self.bus.cmd_capture()
        self.bus.cmd_mode(argparse.Namespace(mode='manual'))  # TEACH를 나가면 manual_control이 지금 자세로 토크 켬
        return STAY


class ArmMenu(Menu):
    title = '팔'

    def items(self):
        return [ArmShowItem(self.app),
                ArmMoveItem(self.app, '2', False),
                ArmMoveItem(self.app, '3', True),
                ArmHomeItem(self.app),
                ArmRebootItem(self.app),
                ArmPoseCaptureItem(self.app),
                ArmInitialItem(self.app)]

    def center(self):
        st = self.state
        values = [f'{n}={st.joints[n]:+.3f} rad ({math.degrees(st.joints[n]):+.1f}°)'
                  for n in ARM_JOINTS if n in st.joints]
        center = ['[팔 관절]']
        center += [f' {v}' for v in values] if values else [' (joint_states 없음)']
        errors = st.motor_errors()
        if errors:
            center.append(f' {self.term.paint("모터 에러: " + ", ".join(errors), RED)} → 5) 재부팅')
        if st.operation != 'MANUAL':
            center.append(f' {self.term.paint("MANUAL 모드가 아닙니다", YELLOW)}')
        if st.map_mode == 'MAPPING':
            center.append(f' {self.term.paint("MAPPING 중 팔 명령 거절", YELLOW)}')
        return center


class GripperShowItem(MenuItem):
    key = '1'

    def label(self):
        return '상태'

    def run(self):
        self.bus.cmd_gripper(argparse.Namespace(action='show', value=None, force=False))
        return STAY


class GripperOpenItem(MenuItem):
    key = '2'

    def label(self):
        return '열기'

    def run(self):
        self.bus.cmd_gripper(argparse.Namespace(action='open', value=None, force=False))
        return STAY


class GripperCloseItem(MenuItem):
    key = '3'

    def label(self):
        return '닫기'

    def run(self):
        self.bus.cmd_gripper(argparse.Namespace(action='close', value=None, force=False))
        return STAY


class GripperPositionItem(MenuItem):
    key = '4'

    def label(self):
        return '위치 지정'

    def run(self):
        raw = self.ui.modal_input(
            '그리퍼 위치', [f'0=닫힘, {GRIPPER_OPEN}=열림 (Esc=뒤로)'], '위치 [m]>')
        if raw is None:
            return STAY
        if raw.strip().lower() in ('b', 'back', ''):
            return STAY
        self.bus.cmd_gripper(argparse.Namespace(action='position', value=raw.strip(), force=False))
        return STAY


class GripperMenu(Menu):
    title = '그리퍼'

    def items(self):
        return [GripperShowItem(self.app), GripperOpenItem(self.app),
                GripperCloseItem(self.app), GripperPositionItem(self.app)]

    def center(self):
        if GRIPPER_JOINT in self.state.joints:
            grip_txt = f'{GRIPPER_JOINT}={self.state.joints[GRIPPER_JOINT]:.4f} m'
        else:
            grip_txt = '(joint_states 없음)'
        return ['[그리퍼]', f' 현재: {grip_txt}']
