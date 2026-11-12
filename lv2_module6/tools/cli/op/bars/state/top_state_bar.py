"""TopStateBar — 화면 맨 위 상태바 기본 구성 (StateBar 하위 클래스)."""
from __future__ import annotations

from .basket_state import BasketState
from .connection_state import ConnectionState
from .device_state import DeviceState
from .diag_state import DiagState
from .estop_state import EstopState
from .map_state import MapState
from .nav_state import CollisionState, ControllerState
from .no_data_state import NoDataState
from .operation_state import OperationState
from .perception_state import PerceptionState
from .pose_state import PoseState
from .robot_state import PowerState, RobotState
from .slam_state import SlamState
from .state_bar import StateBar
from .task_state import TaskState
from .topic_rate_state import TopicRateState

# 토픽 생존 표시: (StateStore.rates 키, 라벨, 이 Hz보다 낮으면 노랑) — 기대 주기의 대략 절반
TOPIC_RATES = (('joint_states', 'joint', 50), ('odom', 'odom', 20), ('imu', 'imu', 50),
               ('front_cam', '앞캠', 7), ('gripper_cam', '손캠', 7), ('points', '점구름', 7),
               ('tracked_objects', '인지', 5))   # cuVSLAM Hz는 줄 2 SLAM 칸


class TopStateBar(StateBar):
    """줄 1 모드·지도·E-STOP·장치·연결 · 줄 2 미션·보이는 물체·주행 제어기·충돌 감시·SLAM 점프·진단 · 줄 3 토픽 Hz · 줄 4 모터(바퀴·팔·그리퍼 그룹) · 줄 5 전원 · (상태 없으면 경고)."""

    def __init__(self, ui):
        super().__init__(ui, [
            [OperationState(ui), MapState(ui), PoseState(ui), EstopState(ui), DeviceState(ui), ConnectionState(ui)],
            [TaskState(ui), PerceptionState(ui), BasketState(ui), ControllerState(ui), CollisionState(ui), SlamState(ui), DiagState(ui)],
            [TopicRateState(ui, key, label, low) for key, label, low in TOPIC_RATES],
            [RobotState(ui)],
            [PowerState(ui)],
            [NoDataState(ui)],
        ])
