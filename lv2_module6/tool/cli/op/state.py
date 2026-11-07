"""StateStore — ROS 상태 보관. 구독 콜백이 채우고, 화면·명령이 읽는다."""
from __future__ import annotations

import math
import time
from collections import deque

# 3S LiPo (TurtleBot3 11.1V) 방전 곡선 — 대략값. 12.6V 만충 · 11.1V 공칭 · 10.5V 방전.
#   아답터를 꽂으면 OpenCR 전원 입력값이라 배터리 잔량이 아니다 (표시에 '충전?'을 붙인다).
BATTERY_CURVE = ((12.6, 100), (12.3, 90), (12.0, 75), (11.8, 60),
                 (11.5, 40), (11.2, 20), (11.0, 10), (10.5, 0))


class Rate:
    """토픽 수신 빈도 — 최근 2초 안의 수신 시각으로 Hz, 마지막 수신 뒤 경과."""

    def __init__(self):
        self.times: deque[float] = deque(maxlen=200)

    def tick(self):
        self.times.append(time.monotonic())

    def age(self) -> float | None:
        return time.monotonic() - self.times[-1] if self.times else None

    def hz(self) -> float:
        now = time.monotonic()
        recent = [t for t in self.times if now - t <= 2.0]
        return (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 1 and recent[-1] > recent[0] else 0.0


class StateStore:
    """ROS 토픽 최신값 보관소."""

    def __init__(self):
        self.values: dict[str, str] = {}
        self.emergency = False
        self.devices: list[str] = []
        self.device_details: dict[str] = {}
        self.diagnostics: dict = {}
        self.diag_values: dict = {}
        self.joints: dict[str, float] = {}
        self.joint_vel: dict[str, float] = {}
        self.joint_effort: dict[str, float] = {}  # BeebotSystemHardware effort = 모터 전류 [A]
        self.motor_temp: dict[str, int] = {}      # /motor_status °C (−1 모름)
        self.motor_text: dict[str, str] = {}      # /motor_status 에러 이름 ("정상" · "과부하" …)
        self.battery_v = float('nan')             # /motor_status OpenCR 전원 입력(배터리) V — NaN 모름
        self.last_joint_time = 0.0
        self.last_update = 0.0  # 마지막 토픽 수신 시각 (monotonic) — 헤더 갱신시각 표시용
        self.state_last_update = 0.0  # 마지막 /state/* 수신 — launch 재시작 뒤 CLI 재연결 판단용
        self.param_clients: dict = {}
        # 상시 상태바용 (없으면 0/빈 값으로 표시, 보드 미동작 아님)
        self.tracked_count = 0
        self.task_status = ''
        self.say_last = ''
        self.rates: dict[str, Rate] = {}  # 상단 생존 표시 — 키는 state_bar/top_state_bar.py TOPIC_RATES
        self.rl_status: list[float] = []  # controller_server FollowPath RL/DWB 상태
        self.rl_status_time = 0.0

    def tick(self, key: str):
        self.rates.setdefault(key, Rate()).tick()

    @property
    def operation(self) -> str:
        return (self.values.get('operation_mode') or '').upper()

    @property
    def map_mode(self) -> str:
        return (self.values.get('map_mode') or '').upper()

    # ------------------------------------------------------------- 구독 콜백
    def _touch(self):
        """토픽 수신 시각 기록 — 헤더 갱신시각 표시용."""
        self.last_update = time.monotonic()

    def on_state_value(self, key: str, data: str):
        self.values[key] = data
        self._touch()
        self.state_last_update = time.monotonic()

    def on_emergency(self, data: bool):
        self._touch()
        self.state_last_update = time.monotonic()
        self.emergency = bool(data)

    def on_devices(self, names: list[str]) -> list[str]:
        """장치 목록 갱신 → 새로 나타난 장치 이름 반환 (상세 구독용)."""
        self._touch()
        self.state_last_update = time.monotonic()
        fresh = [n for n in names if n not in self.devices]
        self.devices = list(names)
        return fresh

    def on_device_detail(self, name: str, text: str):
        self.device_details[name] = text
        self._touch()
        self.state_last_update = time.monotonic()

    def on_joints(self, names, positions, velocities=None, efforts=None):
        self._touch()
        self.tick('joint_states')
        vels = list(velocities or [])
        effs = list(efforts or [])
        for i, name in enumerate(names):
            if i < len(positions):
                self.joints[name] = positions[i]
            if i < len(vels):
                self.joint_vel[name] = vels[i]
            if i < len(effs):
                self.joint_effort[name] = effs[i]
        self.last_joint_time = time.monotonic()

    def on_motor_status(self, names, temps, texts, battery_v=float('nan')):
        self._touch()
        self.battery_v = battery_v
        self.motor_temp.update(zip(names, temps))
        self.motor_text.update(zip(names, texts))

    def on_rl_status(self, data):
        """controller_server FollowPath 상태 — [rl_active, reason, ...]."""
        self.rl_status = list(data)
        self.rl_status_time = time.monotonic()
        self._touch()

    def battery_percent(self) -> int | None:
        """3S LiPo 전압 → 대략 잔량 [%]. 전압이 없거나 12V가 꺼졌으면 None."""
        v = self.battery_v
        if not math.isfinite(v) or v < 5.0:
            return None
        if v >= BATTERY_CURVE[0][0]:
            return 100
        if v <= BATTERY_CURVE[-1][0]:
            return 0
        for (v1, p1), (v2, p2) in zip(BATTERY_CURVE, BATTERY_CURVE[1:]):
            if v2 <= v <= v1:
                return int(round(p2 + (v - v2) / (v1 - v2) * (p1 - p2)))
        return None

    def battery_summary(self, long: bool = False) -> str:
        """상태바·텔레오퍼·status 공용 — '배터리' 접두어는 호출자가 붙인다."""
        v = self.battery_v
        if not math.isfinite(v):
            return '-'
        if v < 5.0:
            return '12V 꺼짐'
        pct = self.battery_percent()
        if pct is None:
            return f'{v:.2f} V'
        charge = ' 충전?' if v >= 12.45 else ''
        return (f'{v:.2f} V ({pct}% 추정{charge})' if long
                else f'{v:.1f}V ({pct}% 추정{charge})')

    def on_diagnostics(self, statuses):
        self._touch()
        for status in statuses:
            level = status.level
            # DiagnosticStatus.level은 byte 타입이라 rclpy에서 bytes로 온다.
            if isinstance(level, (bytes, bytearray)):
                level = level[0] if len(level) else 0
            else:
                level = int(level)
            self.diagnostics[status.name] = (level, status.message)
            self.diag_values[status.name] = [(kv.key, kv.value) for kv in status.values]

    def on_say(self, text: str):
        self._touch()
        self.say_last = text

    def on_objects(self, objects):
        self._touch()
        self.tick('tracked_objects')
        try:
            self.tracked_count = len(objects)
        except Exception:
            self.tracked_count = 0

    def on_task(self, plan_id, step, state, reason):
        self._touch()
        names = {0: 'RUNNING', 1: 'SUCCEEDED', 2: 'FAILED'}
        try:
            label = names.get(int(state), str(state))
            self.task_status = f'plan={plan_id} step={step} {label} {reason}'
        except Exception:
            self.task_status = ''

    # ---------------------------------------------------------------- 파생
    def device_info(self, name: str) -> tuple[str, str, list[str]]:
        """장치의 `종류|active/disconnected|토픽,토픽`을 푼다."""
        raw = (self.device_details.get(name) or '').strip()
        if not raw:
            return '-', '', []
        parts = raw.split('|', 2)
        kind = parts[0].strip() if parts else '-'
        state = parts[1].strip().lower() if len(parts) > 1 else ''
        topics = [t for t in parts[2].split(',') if t] if len(parts) > 2 else []
        return kind or '-', state, topics

    def motor_errors(self) -> list[str]:
        """/state/devices/<wheels|arm|gripper> 세부의 '관절:에러' (state_manager가 모터 Hardware Error를 실어 줌)."""
        errors = []
        for name in ('wheels', 'arm', 'gripper'):
            _, _, details = self.device_info(name)
            errors += [d for d in details if ':' in d]
        return errors
