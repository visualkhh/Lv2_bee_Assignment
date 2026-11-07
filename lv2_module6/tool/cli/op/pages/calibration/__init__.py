"""칼리브레이션 메뉴 — 지금은 바퀴. 팔(calibrate arm)은 나중에 이 메뉴에 항목으로 붙인다."""
from __future__ import annotations

import argparse
from pathlib import Path

from ...defs import STAY
from ..base import GoMenu, Menu, MenuItem
from .core import load_plan

DEFAULT_PLAN = Path(__file__).resolve().parent / 'wheel' / 'command.json'   # op/pages/calibration/wheel/ — sim과 같은 네모 54 s


class WheelState:
    """바퀴 칼리브레이션 화면이 공유하는 선택값 — 지금 계획 파일."""
    plan = str(DEFAULT_PLAN)


def _plan_text() -> list[str]:
    try:
        plan = load_plan(WheelState.plan)
    except ValueError as exc:
        return [f' 계획: {WheelState.plan}', f' ⚠ {exc}']
    n = len(plan['steps'])
    return [f' 계획: {Path(WheelState.plan).name} — {n}스텝 · {n * plan["dt"]:.1f} s']


def _args(check_only: bool) -> argparse.Namespace:
    return argparse.Namespace(part='wheel', plan=WheelState.plan, out=None, check_only=check_only, force=False)


class WheelCheckItem(MenuItem):
    key = '1'

    def label(self):
        return '준비 확인 (안 움직임)'

    def run(self):
        self.bus.cmd_calibrate(_args(True))
        return STAY


class WheelRunItem(MenuItem):
    key = '2'

    def label(self):
        return '재생 + 기록 (로봇이 움직임)'

    def run(self):
        try:
            plan = load_plan(WheelState.plan)
        except ValueError as exc:
            print(f'거부: {exc}')
            return STAY
        body = [f'{len(plan["steps"])}스텝 · {len(plan["steps"]) * plan["dt"]:.1f} s · {Path(WheelState.plan).name}',
                '로봇이 움직입니다 — 사방 약 1.5 m가 비어 있어야 합니다',
                '팔이 시작 자세(stow)가 아니면 먼저 자동으로 그 자세로 이동합니다 (충돌 회피 없음 — 팔 주변 확인)',
                '멈추려면: Ctrl+C · 다른 터미널 ./run-cli estop on']
        if not self.ui.confirm('재생할까요?', default=False, body=body):
            return STAY
        self.bus.cmd_calibrate(_args(False))   # MANUAL이 아니면 전환을 먼저 묻는다
        return STAY


class WheelPlanItem(MenuItem):
    key = '3'

    def label(self):
        return '계획 파일 바꾸기'

    def run(self):
        def check(raw):
            try:
                load_plan(str(Path(raw).expanduser()))
            except ValueError as exc:
                return str(exc)
            return None

        raw = self.ui.modal_input('계획 파일', ['{"dt": 0.05, "steps": [[v, w], ...]} JSON — 기본은 sim과 같은 네모'],
                                  '경로', default=WheelState.plan, validate=check)
        if raw is not None:
            WheelState.plan = str(Path(raw).expanduser())
        return STAY


class WheelReportItem(MenuItem):
    key = '4'

    def label(self):
        n = len(self.bus.latest_calibration_csvs(2))
        return f'리포트/비교 (최근 raw.csv {n}개)'

    def run(self):
        csvs = self.bus.latest_calibration_csvs(2)
        if not csvs:
            print('FAIL: raw.csv 없음 — 먼저 2) 재생 + 기록', file=sys.stderr)
            return STAY
        if len(csvs) == 2 and not self.ui.confirm(
                '최근 두 측정을 비교할까요?', default=True,
                body=[str(p) for p in csvs]):
            return STAY
        self.bus.cmd_calibrate(argparse.Namespace(
            part='report', csv=[str(p) for p in csvs], labels=None, out=None))
        return STAY


class WheelCalibrateMenu(Menu):
    title = '칼리브레이션 — 바퀴'

    def items(self):
        return [WheelCheckItem(self.app), WheelRunItem(self.app), WheelPlanItem(self.app),
                WheelReportItem(self.app)]

    def center(self):
        st = self.state
        return [
            '[바퀴 칼리브레이션]',
            f' 모드: {st.operation or "-"} (재생 때 MANUAL이 아니면 전환을 묻습니다)',
            f' E-STOP: {"ON" if st.emergency else "OFF"}',
            *_plan_text(),
            '',
            ' 하는 일: 계획의 (v, w)를 /cmd/manual/cmd_vel로 20 Hz 재생하며',
            '   /odom · /imu · /joint_states · /cmd_vel을 raw.csv로 기록',
            ' 저장: tool/log/calibration/<시각>_wheel/raw.csv',
            ' 순서: 1) 준비 확인 → 공간 비움 → 2) 재생 (팔은 시작 자세로 자동 이동)',
            ' 4) 리포트/비교 — 최근 raw.csv 1~2개 → report.png',
            ' PC로: scp bee@bee.local:<폴더>/raw.csv … 후 isaac report.py (sim과 비교)',
        ]


class CalibrateMenu(Menu):
    title = '칼리브레이션'

    def items(self):
        return [GoMenu(self.app, '1', '바퀴', WheelCalibrateMenu(self.app))]

    def center(self):
        return ['[칼리브레이션]', ' 1) 바퀴 — 모터 응답(gain · 지연)을 재서 sim에 맞춤', ' (팔은 나중에 여기에 추가)']
