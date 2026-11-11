"""가짜 ROS 메시지 클래스 — FakeRos와 DockerRos가 같이 쓴다.

rclpy 메시지처럼 속성만 있는 가벼운 객체다. install_modules가 이 클래스들을
sys.modules의 std_msgs/sensor_msgs/... 에 꽂아 operator_tool이 import하게 한다.
"""
from __future__ import annotations


class _Msg:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class OccupancyGrid(_Msg):
    def __init__(self, **kw):
        self.header = None
        self.info = _Msg(width=0, height=0, resolution=0.05, origin=_Msg(position=_Msg(x=0.0, y=0.0, z=0.0)))
        self.data = []
        super().__init__(**kw)


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


class Point(_Msg):
    def __init__(self, **kw):
        self.x = self.y = self.z = 0.0
        super().__init__(**kw)


class Quaternion(_Msg):
    def __init__(self, **kw):
        self.x = self.y = self.z = 0.0
        self.w = 1.0
        super().__init__(**kw)


class Pose(_Msg):
    def __init__(self, **kw):
        self.position = Point()
        self.orientation = Quaternion()
        super().__init__(**kw)


class PoseStamped(_Msg):
    def __init__(self, **kw):
        self.header = Header()
        self.pose = Pose()
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


class ToggleRequest(_Msg):
    def __init__(self, **kw):
        self.enable = True
        super().__init__(**kw)


class ToggleResponse(_Msg):
    def __init__(self, **kw):
        self.success, self.message = True, ''
        super().__init__(**kw)


class Toggle:   # nav2_msgs/srv/Toggle — collision_monitor 켜기·끄기
    Request = ToggleRequest
    Response = ToggleResponse


class Int32MultiArray(_Msg):
    def __init__(self, data=None):
        self.data = list(data or [])


class Float32MultiArray(_Msg):
    def __init__(self, data=None):
        self.data = list(data or [])


class Odometry(_Msg):  # CLI는 수신 시각만 씀
    pass


class Imu(_Msg):
    pass


class CameraInfo(_Msg):
    pass


class PointCloud2(_Msg):  # CLI는 수신 시각만 씀 (raw 구독)
    pass


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


# ---------------------------------------------------------------- 가짜 월드가 쓰는 메시지 (nav_msgs/Path · plan · perception · control)
class Path(_Msg):
    def __init__(self, **kw):
        self.header = Header()
        self.poses = []
        super().__init__(**kw)


class TrackedObjectArray(_Msg):
    def __init__(self, **kw):
        self.objects = []
        super().__init__(**kw)


class MotorStatus(_Msg):
    def __init__(self, **kw):
        self.joints, self.temperature, self.text = [], [], []
        self.battery_voltage = float('nan')
        super().__init__(**kw)


class TaskStatus(_Msg):
    def __init__(self, **kw):
        self.plan_id, self.step, self.state, self.reason = 0, 0, 0, ''
        super().__init__(**kw)


class SubGoal(_Msg):
    def __init__(self, **kw):
        self.step, self.skill, self.target, self.relation, self.offset_m = 0, '', '', '', 0.0
        super().__init__(**kw)


class SubGoalArray(_Msg):
    def __init__(self, **kw):
        self.header = Header()
        self.plan_id = 0
        self.goals = []
        super().__init__(**kw)


class MissionState(_Msg):
    def __init__(self, **kw):
        self.header = Header()
        self.phase, self.note, self.barcode, self.order, self.error = 'IDLE', '', '', '', ''
        self.count_r = self.count_y = self.count_g = 0
        self.tray, self.results = [], []
        self.placed = self.total = 0
        super().__init__(**kw)
