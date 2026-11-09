"""수동 주행·팔·그리퍼 메뉴."""
from __future__ import annotations

import argparse
import math
import sys

from ...consts import ARM_JOINTS, GRIPPER_JOINT, GRIPPER_OPEN, WHEEL_JOINTS
from ...defs import BACK, MAIN, QUIT, STAY
from ...terminal import GREEN, RED, YELLOW
from ...menus import GoMenu, Menu, MenuItem


def _finite(raw: str) -> str | None:
    """modal_input 검증 — 유한한 숫자만."""
    try:
        value = float(raw)
    except ValueError:
        return f'숫자가 아님: {raw!r}'
    return None if math.isfinite(value) else '유한한 숫자가 아님'


class TeleopItem(MenuItem):
    shows_busy = False  # 전체 화면 텔레오퍼
    key = '1'

    def label(self):
        return '텔레오퍼 (w/x/a/d·방향키 · q/e/z/c 사선 · -/= 속도 · b 삑)'

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


CURRENT_FULL_A = 1.5  # 전류 막대 끝 (XM430 정지 전류 대략) — 눈금일 뿐 한계 아님
# 좁은 표(6칸)에 맞춘 이름 — wheel_/joint1 구분이 잘리지 않게
MOTOR_SHORT = {'wheel_left_joint': 'wheelL', 'wheel_right_joint': 'wheelR',
               'joint1': 'joint1', 'joint2': 'joint2', 'joint3': 'joint3',
               'joint4': 'joint4', 'joint5': 'joint5', 'gripper_left_joint': 'grip'}


def motor_row(page, name: str, unit: str) -> str:
    """모터 한 줄 (팔·그리퍼·바퀴 공용) — 좁은 중앙 패널(≈50칸)에 맞춘 압축 표기.

    관절  각도(°)  속도   전류(mA, 막대 0~1.5A)  온도  [에러]
    """
    st = page.state
    pos = st.joints.get(name)
    vel = st.joint_vel.get(name)
    cur = st.joint_effort.get(name)
    temp = st.motor_temp.get(name, -1)
    pos_txt = ('      ?' if pos is None else
               f'{math.degrees(pos):+7.1f}' if unit == 'deg' else f'{pos * 1000:7.1f}')
    vel_txt = '     ?' if vel is None or vel != vel else f'{vel:+6.2f}'
    if cur is None or cur != cur:  # 없음·NaN — 예전 펌웨어(바퀴 전류 없음)·토픽 미수신
        cur_txt = '   -  '
        bar = '     '
    else:
        n = min(5, round(abs(cur) / CURRENT_FULL_A * 5))
        bar = (page.term.paint('█' * n, RED if n >= 4 else YELLOW if n >= 3 else GREEN)
               + '·' * (5 - n))
        cur_txt = f'{cur * 1000:+5.0f}'
    temp_txt = '  -' if temp < 0 else page.term.paint(f'{temp:3d}', RED if temp >= 70 else '')
    err = st.motor_text.get(name, '')
    err_txt = '' if err in ('', '정상') else ' ' + page.term.paint(err, RED)
    return f' {MOTOR_SHORT.get(name, name[:6]):<6} {pos_txt} {vel_txt} {cur_txt} {bar} {temp_txt}°{err_txt}'


WHEEL_RADIUS_M = 0.033  # controllers.yaml diff_drive_controller wheel_radius
WHEEL_SEPARATION_M = 0.287  # 같은 곳 wheel_separation

MOTOR_HEADER = ' 관절    각도(°)   속도    mA 0─1.5A  온도'


def wheel_lines(page) -> list[str]:
    """바퀴 실시간 표 (수동 주행·바퀴 화면 공용) — 각도 = 부팅 뒤 누적, 전류 = 모터 Present Current."""
    st = page.state
    vel = [st.joint_vel.get(n) for n in WHEEL_JOINTS]
    speed = (f'{(vel[0] + vel[1]) / 2 * WHEEL_RADIUS_M:+.3f} m/s · 회전 {(vel[1] - vel[0]) * WHEEL_RADIUS_M / WHEEL_SEPARATION_M:+.2f} rad/s'
             if None not in vel else '?')
    return (['[바퀴 상태 — 실시간]  각도 = 부팅 뒤 누적 회전 · 속도 rad/s', MOTOR_HEADER.replace('관절', '바퀴')]
            + [motor_row(page, n, 'deg') for n in WHEEL_JOINTS] + [f' 차체 ≈ {speed}'])


class ManualMenu(Menu):
    title = '수동 주행 / 텔레오퍼'

    def items(self):
        return [TeleopItem(self.app),
                DriveItem(self.app, '2', 'forward', '전진'),
                DriveItem(self.app, '3', 'back', '후진'),
                DriveItem(self.app, '4', 'left', '좌회전'),
                DriveItem(self.app, '5', 'right', '우회전'),
                DriveItem(self.app, '6', 'stop', '정지'),
                GoMenu(self.app, '7', '속도/시간 설정', DriveSettingsMenu(self.app)),
                RecoverItem(self.app)]

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
            '',
        ] + wheel_lines(self)


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


class WheelMenu(Menu):
    """바퀴만 보는 화면 — 실시간 표 + 텔레오퍼·정지·복구 (한 번씩 움직이기·속도 설정은 4) 수동 주행)."""
    title = '바퀴'

    def items(self):
        return [TeleopItem(self.app),
                DriveItem(self.app, '2', 'stop', '정지'),
                RecoverItem(self.app)]

    def center(self):
        st = self.state
        lines = wheel_lines(self)
        lines += ['', f' 모드: {st.operation or "-"} · E-STOP: {"ON" if st.emergency else "OFF"}',
                  ' 한 번씩 움직이기·속도 설정은 메인 4) 수동 주행']
        return lines


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
        items = [(n, f'{n} = {self.bus._rad_deg(st.joints[n])}' if n in st.joints else n)
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
            cur = f'현재 {self.bus._rad_deg(st.joints[name])}' if name in st.joints else '현재 ?'
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
        return '시작 자세로 (skills.yaml skill_pose initial_pose · initial_gripper)'

    def run(self):
        if not self.bus.ensure_arm_or_switch():
            return STAY
        pose = self.bus.initial_pose()
        if pose is None:
            return STAY
        targets, gripper, source = pose
        body = [f'출처: {source}'] + [f'{n} = {self.bus._rad_deg(v)}' for n, v in targets.items()]
        if gripper is not None:
            body.append(f'그리퍼 {gripper:.4f} m')
        body.append('현재 자세와 주변을 확인하세요 (MoveIt 없이 직선 보간 — 충돌 회피 안 함).')
        if not self.ui.confirm('시작 자세로 이동할까요?', default=False, body=body):
            return STAY
        self.bus.cmd_arm(argparse.Namespace(action='initial', targets=[], force=False))
        return STAY


class RecoverItem(MenuItem):
    """조작 메뉴 공통 — 모터 에러 재부팅 + 꺼진 컨트롤러 다시 켜기 (bringup 재시작 없이)."""
    key = 'z'

    def label(self):
        return '복구 (모터 재부팅 + 꺼진 컨트롤러 다시 켜기)'

    def run(self):
        self.bus.cmd_recover()
        return STAY


class ArmRebootItem(MenuItem):
    key = '5'

    def label(self):
        return '모터 재부팅 (과부하 등 Hardware Error 풀기)'

    def run(self):
        self.bus.cmd_reboot()
        return STAY


class ArmControlItem(MenuItem):
    """팔 조작 화면(ArmControlPage) — 숫자 입력 없이 막대와 키로 목표를 고르고, Enter + 확인 뒤에만 움직인다."""
    shows_busy = False
    key = '0'

    def label(self):
        return '팔 조작 화면 (막대 · 키로 조절 · 확인 뒤 이동)'

    def run(self):
        return self.ui.window.navigate('arm')


class ArmMenu(Menu):
    title = '팔'

    def items(self):
        return [ArmControlItem(self.app), ArmShowItem(self.app),
                ArmMoveItem(self.app, '2', False),
                ArmMoveItem(self.app, '3', True),
                ArmHomeItem(self.app),
                ArmRebootItem(self.app),
                ArmInitialItem(self.app),
                RecoverItem(self.app)]

    def center(self):
        st = self.state
        # 한글은 두 칸이라 f-string 정렬 대신 칸을 맞춘 글자 그대로
        center = ['[팔 상태 — 실시간]', MOTOR_HEADER]
        if not any(n in st.joints for n in ARM_JOINTS):
            center.append(' (joint_states 없음)')
        else:
            center += [motor_row(self, n, 'deg') for n in ARM_JOINTS]
            center.append(motor_row(self, GRIPPER_JOINT, 'mm'))
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


def ask_grip_current(item) -> float | None:
    """그리퍼 명령마다 쥐는 힘 한계 [A]를 묻는다 — 빈칸 Enter = 지금 값 · Esc = 명령 취소(None)."""
    value, ok = item.ui.read_float('쥐는 힘 한계 [A] (펌웨어가 0.00269~1.2로 자름)', item.bus.grip_current, positive=True)
    if not ok:
        return None
    item.bus.grip_current = min(1.2, max(0.00269, value))
    return item.bus.grip_current


class GripperOpenItem(MenuItem):
    key = '2'

    def label(self):
        return '열기'

    def run(self):
        current = ask_grip_current(self)
        if current is not None:
            self.bus.cmd_gripper(argparse.Namespace(action='open', value=None, force=False, current=current))
        return STAY


class GripperCloseItem(MenuItem):
    key = '3'

    def label(self):
        return '닫기'

    def run(self):
        current = ask_grip_current(self)
        if current is not None:
            self.bus.cmd_gripper(argparse.Namespace(action='close', value=None, force=False, current=current))
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
        current = ask_grip_current(self)
        if current is not None:
            self.bus.cmd_gripper(argparse.Namespace(action='position', value=raw.strip(), force=False, current=current))
        return STAY


class GripperMenu(Menu):
    title = '그리퍼'

    def items(self):
        return [GripperShowItem(self.app), GripperOpenItem(self.app),
                GripperCloseItem(self.app), GripperPositionItem(self.app),
                FloatSettingItem(self.app, '5', '쥐는 힘 한계', 'A', lambda: self.bus.grip_current,
                                 lambda v: setattr(self.bus, 'grip_current', min(1.2, max(0.00269, v))), positive=True),
                RecoverItem(self.app)]

    def center(self):
        pos = self.state.joints.get(GRIPPER_JOINT)
        gap = f'간격 {20 + 2 * pos * 1000:.1f} mm (= 20 + 2×손가락)' if pos is not None else '(joint_states 없음)'
        return ['[그리퍼 상태 — 실시간]', MOTOR_HEADER.replace('관절      각도(°  rad)', '그리퍼    손가락 이동'),
                motor_row(self, GRIPPER_JOINT, 'mm'), f' {gap}', '',
                f' 쥐는 힘 한계: {self.bus.grip_current:.2f} A (열기·닫기·위치 지정 때마다 물음 — 빈칸 Enter = 이 값)']
