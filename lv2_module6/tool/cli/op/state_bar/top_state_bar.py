"""TopStateBar — 화면 맨 위 상태바 기본 구성 (StateBar 하위 클래스)."""
from __future__ import annotations

from .connection_state import ConnectionState
from .device_state import DeviceState
from .diag_state import DiagState
from .estop_state import EstopState
from .map_state import MapState
from .no_data_state import NoDataState
from .operation_state import OperationState
from .perception_state import PerceptionState
from .robot_state import RobotState
from .say_state import SayState
from .state_bar import StateBar
from .task_state import TaskState
from .topic_rate_state import TopicRateState

# 토픽 생존 표시: (StateStore.rates 키, 라벨, 이 Hz보다 낮으면 노랑) — 기대 주기의 대략 절반
TOPIC_RATES = (('joint_states', 'joint', 50), ('odom', 'odom', 20), ('imu', 'imu', 50),
               ('front_cam', '앞카메라', 7), ('gripper_cam', '손카메라', 7), ('tracked_objects', '인지', 5))


class TopStateBar(StateBar):
    """줄 1 모드·지도·E-STOP·장치·연결 · 줄 2 인지·임무·진단·발화 · 줄 3 토픽 Hz · 줄 4 로봇 · (상태 없으면 경고)."""

    def __init__(self, ui):
        super().__init__(ui, [
            [OperationState(ui), MapState(ui), EstopState(ui), DeviceState(ui), ConnectionState(ui)],
            [PerceptionState(ui), TaskState(ui), DiagState(ui), SayState(ui)],
            [TopicRateState(ui, key, label, low) for key, label, low in TOPIC_RATES],
            [RobotState(ui)],
            [NoDataState(ui)],
        ])
