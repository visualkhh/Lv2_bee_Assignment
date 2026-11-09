"""두 ROS 대역(fake/docker)이 공유하는 인터페이스와 설치기.

- RosInterface: operator_tool이 쓰는 Node API (발행·구독·서비스·그래프)
- RosPublisher/RosSubscription/RosClient: 백엔드별 구현이 지켜야 하는 최소 API
- RosNode: 이름·로거·시계 등 공통 부분
- install_modules: rclpy/메시지 모듈을 sys.modules에 꽂는 공통 설치기
"""
from __future__ import annotations

import sys
import types
from abc import ABC, abstractmethod

from . import msgs


class _Logger:
    def __init__(self, name):
        self._name = name

    def info(self, msg):
        print(f'[fake:{self._name} INFO] {msg}')

    def warn(self, msg):
        print(f'[fake:{self._name} WARN] {msg}')

    def warning(self, msg):
        print(f'[fake:{self._name} WARN] {msg}')

    def error(self, msg):
        print(f'[fake:{self._name} ERROR] {msg}', file=sys.stderr)

    def debug(self, msg):
        pass


class DoneFuture:
    """이미 끝난 future — fake 서비스/파라미터 응답용."""

    def __init__(self, result=None):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class RosPublisher(ABC):
    @abstractmethod
    def get_subscription_count(self) -> int:
        ...

    @abstractmethod
    def publish(self, msg) -> None:
        ...


class RosSubscription(ABC):
    @abstractmethod
    def destroy(self) -> None:
        ...


class RosClient(ABC):
    @abstractmethod
    def wait_for_service(self, timeout_sec: float = 1.0) -> bool:
        ...

    @abstractmethod
    def call_async(self, req):
        ...

    @abstractmethod
    def destroy(self) -> None:
        ...


class RosInterface(ABC):
    """operator_tool(OperatorTool)이 의존하는 Node API."""

    @abstractmethod
    def create_publisher(self, msg_type, topic, qos) -> RosPublisher:
        ...

    @abstractmethod
    def create_subscription(self, msg_type, topic, callback, qos) -> RosSubscription:
        ...

    @abstractmethod
    def create_client(self, srv_type, topic) -> RosClient:
        ...

    @abstractmethod
    def destroy_subscription(self, sub) -> None:
        ...

    @abstractmethod
    def destroy_client(self, client) -> None:
        ...

    @abstractmethod
    def get_node_names(self) -> list[str]:
        ...

    @abstractmethod
    def get_node_names_and_namespaces(self) -> list[tuple[str, str]]:
        ...

    @abstractmethod
    def get_publishers_info_by_topic(self, topic: str):
        ...


class RosNode(RosInterface):
    """이름·로거·시계만 공통 — 발행/구독/서비스는 백엔드가 구현한다."""

    def __init__(self, name: str, clock):
        self._fake_name = name
        self._logger = _Logger(name)
        self._clock = clock

    def get_name(self):
        return self._fake_name

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    def destroy_node(self):
        pass


# ---------------------------------------------------------------- 모듈 주입
def _mod(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


def _install_custom_msgs() -> None:
    """operator_tool이 import하는 커스텀 msg — 파일/가짜 JSON을 그대로 받는 빈 클래스."""
    for pkg_name, classes in (
            ('perception', ('TrackedObjectArray',)),
            ('control', ('MotorStatus',)),
            ('plan', ('TaskStatus',))):
        pkg = types.ModuleType(pkg_name)
        pkg.__path__ = []  # 패키지처럼
        msg = types.ModuleType(f'{pkg_name}.msg')
        for cname in classes:
            setattr(msg, cname, type(cname, (msgs._Msg,), {}))
        pkg.msg = msg
        sys.modules[pkg_name] = pkg
        sys.modules[f'{pkg_name}.msg'] = msg


def install_modules(node_cls, *, parameter_client_cls, spin_once=None,
                    spin_until_future_complete=None, custom_msgs=False) -> None:
    """rclpy와 메시지 모듈을 sys.modules에 꽂는다. operator_tool import 전에 호출."""
    _mod('rclpy', init=lambda args=None: None, ok=lambda: True, shutdown=lambda: None,
         spin_once=spin_once or (lambda node, timeout_sec=0.0: None),
         spin_until_future_complete=spin_until_future_complete
         or (lambda node, fut, timeout_sec=1.0: None))
    _mod('rclpy.node', Node=node_cls)
    _mod('rclpy.parameter', Parameter=msgs.Parameter)
    _mod('rclpy.parameter_client', AsyncParameterClient=parameter_client_cls)
    qos = _mod('rclpy.qos')
    qos.QoSProfile = msgs.QoSProfile
    qos.QoSReliabilityPolicy = msgs._QosEnum
    qos.QoSDurabilityPolicy = msgs._QosEnum
    qos.QoSHistoryPolicy = msgs._QosEnum
    qos.qos_profile_sensor_data = msgs.QoSProfile(depth=5)
    _mod('std_msgs', msg=_mod('std_msgs.msg', String=msgs.String, Bool=msgs.Bool,
                              Empty=msgs.Empty, Int32MultiArray=msgs.Int32MultiArray,
                              Float32MultiArray=msgs.Float32MultiArray))
    _mod('geometry_msgs', msg=_mod('geometry_msgs.msg', TwistStamped=msgs.TwistStamped,
                                  Twist=msgs.Twist, Vector3=msgs.Vector3,
                                  Point=msgs.Point, Quaternion=msgs.Quaternion,
                                  Pose=msgs.Pose, PoseStamped=msgs.PoseStamped))
    _mod('sensor_msgs', msg=_mod('sensor_msgs.msg', JointState=msgs.JointState,
                                 Header=msgs.Header, Imu=msgs.Imu, CameraInfo=msgs.CameraInfo))
    _mod('nav_msgs', msg=_mod('nav_msgs.msg', Odometry=msgs.Odometry,
                              OccupancyGrid=msgs.OccupancyGrid))
    _mod('diagnostic_msgs', msg=_mod('diagnostic_msgs.msg', DiagnosticArray=msgs.DiagnosticArray,
                                     DiagnosticStatus=msgs.DiagnosticStatus,
                                     KeyValue=msgs.KeyValue))
    _mod('rcl_interfaces',
         msg=_mod('rcl_interfaces.msg', ParameterType=msgs.ParameterType,
                  ParameterValue=msgs._ParamValue))
    _mod('nav2_msgs', srv=_mod('nav2_msgs.srv', SaveMap=msgs.SaveMap))
    if custom_msgs:
        _install_custom_msgs()
