"""ROS 없는 Mac에서 operator_tool UI/UX를 시험하는 가짜 ROS층.

진짜 operator_tool 코드를 그대로 import해 돌린다. 발행은 화면에 찍고,
/cmd/operation_mode·e_stop·map_mode·map_name·restart는 가짜 state_manager가
받아 상태를 바꾼다 (mode 전환·_wait_value까지 실제처럼 동작).
"""
from __future__ import annotations

import sys
import types


# ---------------------------------------------------------------- 메시지
class _Msg:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class Header(_Msg):
    def __init__(self, **kw):
        self.stamp = None
        super().__init__(**kw)


class _Time:
    def to_msg(self):
        return {'sec': 0, 'nanosec': 0}


class String(_Msg):
    def __init__(self, **kw):
        self.data = ''
        super().__init__(**kw)

    def __repr__(self):
        return f"String(data={self.data!r})"


class Bool(_Msg):
    def __init__(self, **kw):
        self.data = False
        super().__init__(**kw)


class Empty(_Msg):
    pass


class Vector3(_Msg):
    def __init__(self, **kw):
        self.x = self.y = self.z = 0.0
        super().__init__(**kw)


class Twist(_Msg):
    def __init__(self, **kw):
        self.linear = Vector3()
        self.angular = Vector3()
        super().__init__(**kw)


class TwistStamped(_Msg):
    def __init__(self, **kw):
        self.header = Header()
        self.twist = Twist()
        super().__init__(**kw)

    def __repr__(self):
        t = self.twist
        return (f'TwistStamped(linear=({t.linear.x:.2f}, {t.linear.y:.2f}, {t.linear.z:.2f}), '
                f'angular=({t.angular.x:.2f}, {t.angular.y:.2f}, {t.angular.z:.2f}))')


class JointState(_Msg):
    def __init__(self, **kw):
        self.header = Header()
        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []
        super().__init__(**kw)

    def __repr__(self):
        pairs = ', '.join(f'{n}={p:.3f}' for n, p in zip(self.name, self.position))
        return f'JointState({pairs})'


class KeyValue(_Msg):
    def __init__(self, **kw):
        self.key = ''
        self.value = ''
        super().__init__(**kw)


class DiagnosticStatus(_Msg):
    def __init__(self, **kw):
        self.level = 0
        self.name = ''
        self.message = ''
        self.values = []
        super().__init__(**kw)


class DiagnosticArray(_Msg):
    def __init__(self, **kw):
        self.header = Header()
        self.status = []
        super().__init__(**kw)


class ParameterType:
    PARAMETER_NOT_SET = 0
    PARAMETER_BOOL = 1
    PARAMETER_INTEGER = 2
    PARAMETER_DOUBLE = 3
    PARAMETER_STRING = 4
    PARAMETER_BYTE_ARRAY = 5
    PARAMETER_BOOL_ARRAY = 6
    PARAMETER_INTEGER_ARRAY = 7
    PARAMETER_DOUBLE_ARRAY = 8
    PARAMETER_STRING_ARRAY = 9


class _ParamValue(_Msg):
    def __init__(self, **kw):
        self.bool_value = False
        self.integer_value = 0
        self.double_value = 0.0
        self.string_value = ''
        self.byte_array_value = []
        self.bool_array_value = []
        self.integer_array_value = []
        self.double_array_value = []
        self.string_array_value = []
        super().__init__(**kw)


class Parameter(_Msg):
    def __init__(self, name='', value=None, **kw):
        self.name = name
        self.value = value
        super().__init__(**kw)


class SaveMapRequest(_Msg):
    def __init__(self, **kw):
        self.map_topic = '/map'
        self.map_url = ''
        self.image_format = 'pgm'
        self.map_mode = 'trinary'
        self.free_thresh = 0.25
        self.occupied_thresh = 0.65
        super().__init__(**kw)


class SaveMapResponse(_Msg):
    def __init__(self, **kw):
        self.result = True
        super().__init__(**kw)


class SaveMap:
    Request = SaveMapRequest
    Response = SaveMapResponse


# ---------------------------------------------------------------- QoS
class _QosEnum:
    RELIABLE = 1
    BEST_EFFORT = 2
    TRANSIENT_LOCAL = 1
    VOLATILE = 2
    KEEP_LAST = 1


class QoSProfile(_Msg):
    def __init__(self, **kw):
        self.depth = 10
        self.reliability = None
        self.durability = None
        self.history = None
        super().__init__(**kw)


# ---------------------------------------------------------------- 가짜 노드/버스
BUS: dict[str, list] = {}  # topic -> [(node, callback)]
PUB_LOG: list[tuple[str, object]] = []


class _Clock:
    def now(self):
        return _Time()


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


class _FakePub:
    def __init__(self, node, topic):
        self._node = node
        self.topic_name = topic

    def get_subscription_count(self):
        return 1  # 가짜 구독자 있음 (WARN 방지)

    def publish(self, msg):
        data = getattr(msg, 'data', msg)
        print(f'[PUB {self.topic_name}] {data!r}')
        PUB_LOG.append((self.topic_name, msg))
        self._node._fake_on_publish(self.topic_name, msg)
        for node, cb in BUS.get(self.topic_name, []):
            try:
                cb(msg)
            except Exception as e:  # noqa: BLE001
                print(f'[fake] 콜백 에러 {self.topic_name}: {e}')


class _DoneFuture:
    def __init__(self, result=None):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _FakeClient:
    def __init__(self, node, srv_type, topic):
        self._node = node
        self.srv_type = srv_type
        self.topic_name = topic

    def wait_for_service(self, timeout_sec=1.0):
        return True

    def call_async(self, req):
        print(f'[SVC {self.topic_name}] 요청 → 가짜 성공')
        return _DoneFuture(SaveMapResponse(result=True))

    def destroy(self):
        pass


class Node:
    """operator_tool.OperatorTool의 부모 — 필요한 API만 가짜로."""

    def __init__(self, name):
        self._fake_name = name
        self._fake_subs: list[tuple[str, object]] = []
        self._clock = _Clock()
        self._logger = _Logger(name)

    def get_name(self):
        return self._fake_name

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    def create_publisher(self, msg_type, topic, qos):
        return _FakePub(self, topic)

    def create_subscription(self, msg_type, topic, callback, qos):
        BUS.setdefault(topic, []).append((self, callback))
        self._fake_subs.append((topic, callback))

        class _Sub:
            def destroy(self):
                pass

        return _Sub()

    def create_client(self, srv_type, topic):
        return _FakeClient(self, srv_type, topic)

    def destroy_client(self, client):
        pass

    def destroy_node(self):
        for topic, cb in self._fake_subs:
            try:
                BUS[topic].remove((self, cb))
            except ValueError:
                pass

    def get_publishers_info_by_topic(self, topic):
        info = _Msg()
        info.node_name = 'joint_state_broadcaster'
        info.node_namespace = '/'
        return [info] if topic in ('/joint_states', '/state/operation_mode') else []

    def get_node_names(self):
        return ['operator_tool', 'state_manager', 'manual_control',
                'vlm_task_planner', 'coordinator', 'perception',
                'pole_front_cam', 'gripper_cam',
                'joint_state_broadcaster', 'controller_manager']

    def get_node_names_and_namespaces(self):
        return [(n, '/') for n in self.get_node_names()]

    def _fake_on_publish(self, topic, msg):
        """가짜 state_manager — /cmd/*를 받아 상태를 바꾼다."""
        state = getattr(self, 'state', None)
        if state is None:
            return
        vals = state.values
        data = getattr(msg, 'data', None)
        if topic == '/cmd/operation_mode' and isinstance(data, str):
            vals['operation_mode'] = data
        elif topic == '/cmd/e_stop' and isinstance(data, bool):
            state.emergency = data
        elif topic == '/cmd/map_mode' and isinstance(data, str):
            vals['map_mode'] = data
        elif topic == '/cmd/map_name' and isinstance(data, str):
            vals['map_name'] = data
        elif topic == '/cmd/restart' and isinstance(data, str):
            vals['restart'] = f'{data} (가짜 수락)'
            # map:= 인자가 있으면 가짜 state_manager가 지도 모드까지 갈아탄 것으로 친다
            import re as _re
            m = _re.search(r'map:=(\S+)', data)
            if m:
                mode = m.group(1)
                if mode == 'slam':
                    vals['map_mode'] = 'MAPPING'
                    vals['map_list'] = 'default\n20261017_223015'
                elif mode == 'saved':
                    vals['map_mode'] = 'LOCALIZATION'
                    nm = _re.search(r'map_name:=(\S+)', data)
                    vals['map_name'] = nm.group(1) if nm else 'default'


class AsyncParameterClient:
    def __init__(self, node, fqn):
        self._node = node

    def wait_for_services(self, timeout_sec=1.0):
        return False  # 파라미터 명령은 거부 경로 UX만 확인


# ---------------------------------------------------------------- 모듈 주입
def _mod(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


def install():
    _mod('rclpy', init=lambda args=None: None, ok=lambda: True, shutdown=lambda: None,
         spin_once=lambda node, timeout_sec=0.0: None,
         spin_until_future_complete=lambda node, fut, timeout_sec=1.0: None)
    _mod('rclpy.node', Node=Node)
    _mod('rclpy.parameter', Parameter=Parameter)
    _mod('rclpy.parameter_client', AsyncParameterClient=AsyncParameterClient)
    qos = _mod('rclpy.qos')
    qos.QoSProfile = QoSProfile
    qos.QoSReliabilityPolicy = _QosEnum
    qos.QoSDurabilityPolicy = _QosEnum
    qos.QoSHistoryPolicy = _QosEnum
    _mod('std_msgs', msg=_mod('std_msgs.msg', String=String, Bool=Bool, Empty=Empty))
    _mod('geometry_msgs', msg=_mod('geometry_msgs.msg', TwistStamped=TwistStamped,
                                  Twist=Twist, Vector3=Vector3))
    _mod('sensor_msgs', msg=_mod('sensor_msgs.msg', JointState=JointState, Header=Header))
    _mod('diagnostic_msgs', msg=_mod('diagnostic_msgs.msg', DiagnosticArray=DiagnosticArray,
                                    DiagnosticStatus=DiagnosticStatus, KeyValue=KeyValue))
    _mod('rcl_interfaces',
         msg=_mod('rcl_interfaces.msg', ParameterType=ParameterType,
                  ParameterValue=_ParamValue))
    _mod('nav2_msgs', srv=_mod('nav2_msgs.srv', SaveMap=SaveMap))
