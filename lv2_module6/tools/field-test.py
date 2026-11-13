#!/usr/bin/env python3
"""실물 BEE 로봇 단계별 현장 시험 runner.

One session = one existing or new rosbag + append-only JSONL action log.
Run from a ROS host; the script sources ROS/workspace and inherits bringup DDS env.
It never auto-switches MANUAL/AUTO and requires a typed confirmation per movement.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import shlex
import subprocess
import sys
import time
from collections import defaultdict, deque
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / 'tools'
RUN_CLI = TOOLS / 'run-cli'
SESSION_ROOT = Path.home() / '.cache' / 'beebot' / 'field-tests'
ROS_DISTRO = os.environ.get('ROS_DISTRO_NAME', 'lyrical')

BAG_REGEX = (
    r'^/(tf|tf_static|joint_states|imu|odom|odometry/filtered|cuvslam/odom|cuvslam/path|'
    r'cmd_vel.*|cmd/.*|state/.*|diagnostics_agg|diagnostics_toplevel_state|rosout|plan|local_plan|map|'
    r'tracked_objects|global_costmap/costmap|local_costmap/costmap|controller_server/FollowPath/status|'
    r'mission/.*|coordinator/.*|task/.*|skills/.*/_action/.*|skill_context|rl_arm_correction/debug|hand_relative.*|'
    r'arm_safety_event|motor_status)$'
)
CAMERA_BAG_TOPICS = (
    r'|((pole_front_cam|gripper_cam)/(color/image_raw|aligned_depth_to_color/image_raw|color/camera_info))'
    r'|(perception/(pole_front_cam|gripper_cam)/(yolo_detections|detections|debug_image))'
)


def bag_regex(include_camera_data=False):
    """Base action telemetry plus optional image/depth streams for perception debugging."""
    if not include_camera_data:
        return BAG_REGEX
    return BAG_REGEX[:-2] + CAMERA_BAG_TOPICS + ')$'  # BAG_REGEX 끝 ')$'를 떼고 카메라 토픽을 같은 묶음에 넣음


def _as_int(value):
    """ROS uint8 fields may arrive as one-byte values with some Lyrical typesupports."""
    if isinstance(value, (bytes, bytearray, memoryview)):
        return int.from_bytes(value, byteorder='little')
    return int(value)

SKILLS = ('navigate', 'approach', 'pick', 'place', 'inspect', 'pose',
          'load', 'unload', 'stand', 'stand_pick')
POSES = ('stow', 'carry_left', 'carry_right')  # coordinator pose_names (scan은 #168로 없앰)
POLICIES = ('dock_box', 'park_home', 'drive_arc', 'drive_straight')
PRECONDITIONS = {
    'navigate': ['map/localization 정상, TF map→base_footprint 수신', '경로가 비어 있고 도착 구역이 안전함'],
    'approach': ['상자/받침대가 고정됐고 목표 중심 x,y와 방향을 확인함', '접근 방향과 여유 공간이 확보됨'],
    'pick': ['현재 보이는 puck 또는 puck_lying ID와 자세를 확인함', '손이 비어 있고 팔 경로가 비어 있음'],
    'place': ['손에 퍽이 들려 있음을 확인함', '받침대/tag 또는 목표 좌표·높이를 확인하고 주변이 비어 있음'],
    'inspect': ['대상 ID 또는 상자 중심 좌표를 확인함', '손 카메라 시야와 팔 동작 공간을 확인함'],
    'pose': ['팔 이동 경로와 도달 공간이 비어 있음을 확인함'],
    'load': ['손에 든 퍽과 비어 있는 적재함 칸을 실제로 확인함'],
    'unload': ['꺼낼 적재함 칸/tag와 받을 위치를 확인함', '손가락/팔 경로가 비어 있음'],
    'stand': ['현재 보이는 puck_lying ID와 방향 추정값을 확인함', '팔의 접근 공간이 비어 있음'],
    'stand_pick': ['이번 세션에서 stand 성공을 기록했고 퍽이 세워져 있음', '손이 비어 있고 받을 경로가 비어 있음'],
}


def _ros_shell_command(argv: list[str]) -> str:
    ros_setup = f'/opt/ros/{ROS_DISTRO}/setup.bash'
    workspace_setup = str(ROOT / 'ros2_ws' / 'install' / 'setup.bash')
    venv_activate = TOOLS / '.venv' / 'bin' / 'activate'
    lines = [
        'set -eo pipefail',
        f'test -f {shlex.quote(ros_setup)} || {{ echo "ROS 2 {ROS_DISTRO} setup 없음" >&2; exit 2; }}',
        f'source {shlex.quote(ros_setup)}',
        f'test -f {shlex.quote(workspace_setup)} || {{ echo "ros2_ws install 없음: colcon build 필요" >&2; exit 2; }}',
        f'source {shlex.quote(workspace_setup)}',
    ]
    slam_setup = Path.home() / 'isaac_ros_ws' / 'install' / 'setup.bash'
    lines.append(f'[ ! -f {shlex.quote(str(slam_setup))} ] || source {shlex.quote(str(slam_setup))}')
    if venv_activate.is_file():
        lines.append(f'source {shlex.quote(str(venv_activate))}')
    lines.extend([
        'export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"',
        'export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"',
        f'export FASTDDS_DEFAULT_PROFILES_FILE="${{FASTDDS_DEFAULT_PROFILES_FILE:-{ROOT}/ros2_ws/fastdds_local_only.xml}}"',
        # Match the running bringup process so this node and run-cli join the same ROS graph.
        "pid=$(pgrep -f '[r]os2 launch bringup bringup.launch.py' | head -n 1 || true)",
        'if [ -n "$pid" ] && [ -r "/proc/$pid/environ" ]; then',
        '  for key in ROS_DOMAIN_ID ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS ROS_DISCOVERY_SERVER FASTDDS_DEFAULT_PROFILES_FILE FASTRTPS_DEFAULT_PROFILES_FILE RMW_IMPLEMENTATION; do',
        '    value=$(tr "\\0" "\\n" < "/proc/$pid/environ" | sed -n "s/^${key}=//p" | head -n 1)',
        '    if [ -n "$value" ]; then export "$key=$value"; else unset "$key"; fi',
        '  done',
        'fi',
        'case "$(basename "${FASTDDS_DEFAULT_PROFILES_FILE:-}")" in fastdds_local_only.xml) unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER ROS_LOCALHOST_ONLY;; esac',
        'export BEE_FIELD_TEST_ROS_READY=1',
        'exec python3 ' + shlex.quote(str(Path(__file__).resolve())) + ' ' + ' '.join(shlex.quote(a) for a in argv),
    ])
    return '\n'.join(lines)


def bootstrap_ros(argv: list[str]) -> None:
    if os.environ.get('BEE_FIELD_TEST_ROS_READY') == '1':
        return
    os.execl('/bin/bash', 'bash', '-lc', _ros_shell_command(argv))


class Monitor:
    def __init__(self):
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from nav_msgs.msg import Odometry
        from plan.msg import BasketState, MissionState, TaskStatus, SubGoal, SubGoalArray
        from perception.msg import TrackedObjectArray
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Imu, JointState
        from std_msgs.msg import Bool, String

        self.rclpy = rclpy
        self.String, self.SubGoal, self.SubGoalArray = String, SubGoal, SubGoalArray
        latched = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.plan_qos = latched
        self.node = rclpy.create_node('beebot_field_test')
        self.operation = ''
        self.estop: bool | None = None
        self.mission = None
        self.task = None
        self.basket = None
        self.objects = None
        self.diagnostics = {}
        self.rl_nav_status = None
        self.map_pose = None
        self.rates: dict[str, deque[float]] = defaultdict(lambda: deque(maxlen=100))
        self.active_plan_id: int | None = None
        self.last_skill_success = ''

        self.node.create_subscription(String, '/state/operation_mode',
                                      lambda m: setattr(self, 'operation', m.data.upper()), latched)
        self.node.create_subscription(Bool, '/state/emergency',
                                      lambda m: setattr(self, 'estop', bool(m.data)), latched)
        self.node.create_subscription(MissionState, '/mission/state', self._mission, latched)
        self.node.create_subscription(TaskStatus, '/task/status', self._task, latched)
        self.node.create_subscription(BasketState, '/coordinator/basket',
                                      lambda m: setattr(self, 'basket', m), latched)
        self.node.create_subscription(TrackedObjectArray, '/tracked_objects', self._objects, 10)
        self.node.create_subscription(DiagnosticArray, '/diagnostics_agg', self._diagnostics, 10)
        self.node.create_subscription(Odometry, '/odom', lambda _m: self._rate('odom'), qos_profile_sensor_data)
        self.node.create_subscription(Odometry, '/odometry/filtered', lambda _m: self._rate('ekf'), qos_profile_sensor_data)
        self.node.create_subscription(Odometry, '/cuvslam/odom', lambda _m: self._rate('cuvslam'), qos_profile_sensor_data)
        self.node.create_subscription(Imu, '/imu', lambda _m: self._rate('imu'), qos_profile_sensor_data)
        self.node.create_subscription(CameraInfo, '/pole_front_cam/color/camera_info',
                                      lambda _m: self._rate('front_camera'), qos_profile_sensor_data)
        self.node.create_subscription(CameraInfo, '/gripper_cam/color/camera_info',
                                      lambda _m: self._rate('gripper_camera'), qos_profile_sensor_data)
        self.node.create_subscription(JointState, '/joint_states', lambda _m: self._rate('joints'), 10)
        try:
            from std_msgs.msg import Float32MultiArray
            self.node.create_subscription(Float32MultiArray, '/controller_server/FollowPath/status',
                                          lambda m: setattr(self, 'rl_nav_status', list(m.data)), 10)
        except Exception:
            pass
        try:
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self.node)
        except Exception:
            self.tf_buffer = None
        # Each one-step test uses a short-lived transient-local writer. It closes
        # at terminal status so an old test plan is not kept as the last plan.
        self.mission_pub = self.node.create_publisher(String, '/cmd/mission', 10)
        self.plan_type = SubGoalArray

    def _rate(self, key):
        self.rates[key].append(time.monotonic())

    def rate(self, key):
        now = time.monotonic()
        times = [t for t in self.rates[key] if now - t <= 3.0]
        if len(times) < 2:
            return 0.0
        span = times[-1] - times[0]
        return (len(times) - 1) / span if span > 0 else 0.0

    def _mission(self, msg):
        self.mission = msg

    def _task(self, msg):
        self.task = msg

    def _objects(self, msg):
        self.objects = msg
        self._rate('tracked_objects')

    def _diagnostics(self, msg):
        self.diagnostics = {s.name: (_as_int(s.level), s.message, {v.key: v.value for v in s.values})
                            for s in msg.status}

    def spin(self, seconds=0.2):
        self.rclpy.spin_once(self.node, timeout_sec=seconds)

    def settle(self, seconds=2.0):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.spin(min(0.1, deadline - time.monotonic()))

    def snapshot(self):
        if self.tf_buffer:
            try:
                from rclpy.time import Time
                tr = self.tf_buffer.lookup_transform('map', 'base_footprint', Time()).transform
                t, q = tr.translation, tr.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.map_pose = {'x': round(t.x, 4), 'y': round(t.y, 4), 'yaw_rad': round(yaw, 4)}
            except Exception:
                pass
        return {
            'operation': self.operation or None,
            'estop': self.estop,
            'mission_phase': getattr(self.mission, 'phase', None),
            'mission_note': getattr(self.mission, 'note', None),
            'mission_error': getattr(self.mission, 'error', None),
            'mission_summary': ({'barcode': self.mission.barcode, 'order': self.mission.order,
                                 'count_r': self.mission.count_r, 'count_y': self.mission.count_y,
                                 'count_g': self.mission.count_g, 'tray': list(self.mission.tray),
                                 'placed': self.mission.placed, 'total': self.mission.total,
                                 'results': list(self.mission.results)} if self.mission else None),
            'task': ({'plan_id': self.task.plan_id, 'step': self.task.step,
                      'state': _as_int(self.task.state), 'reason': self.task.reason} if self.task else None),
            'visible_objects': ([{'id': o.id, 'class': o.class_name, 'color': o.color}
                                 for o in self.objects.objects] if self.objects else None),
            'basket_slots': ([{'slot': s.slot, 'filled': s.filled, 'shape': s.shape, 'tag': s.tag}
                              for s in self.basket.slots] if self.basket else None),
            'held': ({'filled': self.basket.held.filled, 'shape': self.basket.held.shape,
                      'tag': self.basket.held.tag} if self.basket else None),
            'map_to_base_footprint': self.map_pose,
            'rates_hz': {k: round(self.rate(k), 2) for k in
                         ('front_camera', 'gripper_camera', 'imu', 'odom', 'ekf', 'cuvslam', 'joints', 'tracked_objects')},
            'controller_status_raw': self.rl_nav_status,
            'diagnostics': self.diagnostics,
        }

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


class Session:
    def __init__(self, monitor, bag_path, baseline_bag=None):
        self.monitor = monitor
        self.bag_path = str(bag_path)
        self.baseline_bag = str(baseline_bag) if baseline_bag else None
        self.owns_bag = True
        self.directory = SESSION_ROOT / datetime.now().strftime('%Y%m%d_%H%M%S')
        self.directory.mkdir(parents=True, exist_ok=False)
        self.log = self.directory / 'events.jsonl'
        self.mission_goal_active = False

    def event(self, action, result, details=None):
        row = {'time': datetime.now(timezone.utc).isoformat(), 'action': action, 'result': result,
               'bag': self.bag_path, 'baseline_bag': self.baseline_bag,
               'snapshot': self.monitor.snapshot(), 'details': details or {}}
        with self.log.open('a', encoding='utf-8') as f:
            f.write(json.dumps(row, ensure_ascii=False, default=str) + '\n')
            f.flush()
            os.fsync(f.fileno())
        return row

    def stop_owned_plan(self, plan_id: int, publisher) -> bool:
        """Timeout/cancel only our step; never replace a newer manager plan."""
        snap = self.monitor.snapshot()
        phase, task = snap['mission_phase'], snap['task']
        if phase not in ('IDLE', 'DONE') or (task and task['state'] == 0 and task['plan_id'] != plan_id):
            self.event('one_step_stop', 'skipped_other_plan_or_mission',
                       {'owned_plan_id': plan_id, 'mission_phase': phase, 'task': task})
            return False
        stop = self.monitor.SubGoalArray()
        stop.header.stamp = self.monitor.node.get_clock().now().to_msg()
        stop.plan_id = 0
        publisher.publish(stop)
        self.event('one_step_stop', 'sent', {'owned_plan_id': plan_id})
        return True

    def safety(self, *, require_auto=False, require_idle=False):
        self.monitor.settle(0.5)
        s = self.monitor.snapshot()
        if s['estop'] is not False:
            raise RuntimeError(f'E-stop 상태를 OFF로 확인하지 못함: {s["estop"]}')
        expected = 'AUTO' if require_auto else 'MANUAL'
        if s['operation'] != expected:
            raise RuntimeError(f'현재 모드 {s["operation"]!r}; 필요한 모드 {expected}. TUI에서 직접 전환한 뒤 다시 시도.')
        if s['mission_phase'] is None:
            raise RuntimeError('MissionManager /mission/state 응답 없음; bringup와 ROS domain을 확인.')
        if require_idle and s['mission_phase'] not in ('IDLE', 'DONE'):
            raise RuntimeError(f'미션 phase={s["mission_phase"]!r}; IDLE/DONE일 때만 개별 액션 실행 가능.')
        if require_idle and s['task'] and s['task']['state'] == 0:
            raise RuntimeError('Coordinator에서 이미 step이 RUNNING; 중첩 실행은 거부.')
        if require_auto:
            if not self.monitor.objects:
                raise RuntimeError('/tracked_objects 응답 없음; 단일 skill 실행을 시작하지 않음.')
            if self.monitor.node.count_publishers('/task/status') < 1:
                raise RuntimeError('Coordinator /task/status publisher 없음.')
            if self.monitor.node.count_subscribers('/task/sub_goals') < 1:
                raise RuntimeError('Coordinator /task/sub_goals subscriber 없음.')
        return s


def ask(question, expected='Y'):
    answer = input(f'{question}\n  계속하려면 {expected!r} 입력: ').strip()
    return answer == expected


def json_objects(monitor):
    msg = monitor.objects
    if msg is None:
        return []
    return [{'id': int(o.id), 'class': o.class_name, 'color': o.color,
             'details': {x.key: x.value for x in o.details}} for o in msg.objects]


def ask_skill_goal(session):
    m = session.monitor
    print('\n실행할 단일 Coordinator step을 입력합니다. 목표 좌표는 map 기준이며 실제 TF/표식으로 읽은 값이어야 합니다.')
    for index, skill in enumerate(SKILLS, 1):
        print(f'  {index}. {skill}: ' + ' · '.join(PRECONDITIONS[skill]))
    try:
        index = int(input('skill 번호: ').strip())
        skill = SKILLS[index - 1]
    except (ValueError, IndexError):
        raise RuntimeError('유효하지 않은 skill 번호.')
    snapshot = session.safety(require_auto=True, require_idle=True)
    objects = json_objects(m)
    print('현재 보이는 객체:', json.dumps(objects, ensure_ascii=False))
    held = snapshot['held']
    hand_full = bool(held and held['filled'])
    if skill in ('pick', 'stand', 'stand_pick') and hand_full:
        raise RuntimeError(f'{skill} 전제 실패: 손에 퍽 {held}이 있음.')
    if skill == 'place' and not hand_full:
        raise RuntimeError('place 전제 실패: Coordinator 장부에서 손에 든 퍽을 확인 못함.')
    goal = m.SubGoal()
    goal.step = 1
    goal.skill = skill
    extra = {}
    coordinate = re.compile(r'^\s*(-?(?:\d+(?:\.\d*)?|\.\d+))\s*,\s*(-?(?:\d+(?:\.\d*)?|\.\d+))(?:\s*,\s*(-?(?:\d+(?:\.\d*)?|\.\d+)))?\s*$')

    if skill in ('navigate', 'approach'):
        raw = input('map target x,y,yaw(rad), current observation/TUI 확인값: ').strip()
        match = coordinate.fullmatch(raw)
        if not match or match.group(3) is None:
            raise RuntimeError('이 skill은 x,y,yaw 세 값이 필요함.')
        vals = tuple(map(float, match.groups()))
        if not all(math.isfinite(x) for x in vals) or any(abs(x) > 15 for x in vals[:2]) or abs(vals[2]) > math.pi:
            raise RuntimeError('좌표가 안전 범위를 벗어남 (x,y ±15m, yaw ±pi).')
        goal.target = ','.join(f'{x:.4f}' for x in vals)
        if skill == 'approach':
            goal.relation = input(f'정책 {POLICIES} 중 하나: ').strip()
            if goal.relation not in POLICIES:
                raise RuntimeError('알 수 없는 approach 정책.')
    elif skill in ('pick', 'stand', 'place', 'inspect'):
        goal.target = input('현재 /tracked_objects ID 또는 (허용된 경우) x,y,z: ').strip()
        is_obj = goal.target.isdigit()
        if is_obj:
            obj = next((o for o in objects if o['id'] == int(goal.target)), None)
            if obj is None:
                raise RuntimeError('해당 ID가 현재 보이는 tracked object에 없음.')
            if skill in ('pick', 'stand') and not obj['class'].startswith('puck'):
                raise RuntimeError(f'{skill} 대상 class가 puck 계열이 아님: {obj["class"]}')
            if skill == 'stand' and obj['class'] != 'puck_lying':
                raise RuntimeError('stand는 puck_lying 대상으로 제한.')
            if skill == 'place' and obj['class'] not in ('location_box', 'start_box'):
                raise RuntimeError('place 목표는 location_box 또는 start_box여야 함.')
            extra['object'] = obj
        else:
            match = coordinate.fullmatch(goal.target)
            if skill not in ('place', 'inspect') or not match or match.group(3) is None:
                raise RuntimeError('이 입력은 해당 skill의 유효한 대상 ID/좌표가 아님.')
            xyz = tuple(map(float, match.groups()))
            if (not all(math.isfinite(x) for x in xyz) or any(abs(x) > 15 for x in xyz[:2])
                    or xyz[2] < 0 or xyz[2] > 1.0):
                raise RuntimeError('map 목표 좌표 범위 초과 (x,y ±15m, z 0–1m).')
        if skill == 'inspect':
            relation = input('inspect relation (상자 윗면이면 box, 아니면 Enter): ').strip()
            if relation not in ('', 'box'):
                raise RuntimeError('inspect relation은 빈 값 또는 box.')
            goal.relation = relation
    elif skill == 'pose':
        goal.target = input(f'pose 이름 {POSES}: ').strip()
        if goal.target not in POSES:
            raise RuntimeError('허용 목록에 없는 pose.')
    elif skill == 'load':
        if not snapshot['held'] or not snapshot['held']['filled']:
            raise RuntimeError('Coordinator 장부에서 손에 퍽이 든 것을 확인 못함.')
        goal.target = input('load 대상 next (빈 칸으로): ').strip() or 'next'
        if goal.target != 'next':
            raise RuntimeError('load 대상은 next만 지원.')
        if snapshot['basket_slots'] is None or all(s['filled'] for s in snapshot['basket_slots']):
            raise RuntimeError('빈 basket slot을 확인 못함.')
    elif skill == 'unload':
        slots = snapshot['basket_slots'] or []
        filled = [str(s['slot']) for s in slots if s['filled']]
        if not filled:
            raise RuntimeError('비우지 않은 적재함 칸이 없음.')
        goal.target = input(f'unload 대상 last 또는 채워진 칸 {filled}: ').strip()
        if goal.target not in ('last', *filled):
            raise RuntimeError('현재 채워진 칸만 선택 가능.')
    elif skill == 'stand_pick':
        if m.last_skill_success != 'stand':
            raise RuntimeError('이번 runner session에서 stand 성공이 기록되지 않음.')
        if snapshot['held'] and snapshot['held']['filled']:
            raise RuntimeError('손이 비어 있지 않음.')
        goal.target = ''

    print('\n실행 조건 카드')
    for condition in PRECONDITIONS[skill]:
        if not ask('확인: ' + condition):
            raise RuntimeError('사전 조건 확인이 취소됨.')
    location_note = input('현재 로봇 위치·방향 (map x,y,yaw 또는 실측 설명, 필수): ').strip()
    fixture_note = input('퍽·받침대·테이블·받을 위치 배치 (없으면 none, 필수): ').strip()
    if not location_note or not fixture_note:
        raise RuntimeError('로봇 위치와 작업물 배치를 기록해야 skill을 보낼 수 있음.')
    confirmed_conditions = []
    for condition in PRECONDITIONS[skill]:
        if not ask('조건 충족: ' + condition, 'CONFIRMED'):
            raise RuntimeError('사전 조건 확인이 취소됨.')
        confirmed_conditions.append(condition)
    description = f'{skill} target={goal.target!r} relation={goal.relation!r}'
    if not ask(f'AUTO·E-stop OFF에서 다음 goal을 Coordinator에 1회 전송합니다: {description}', 'EXECUTE'):
        raise RuntimeError('실행 승인 취소됨.')
    extra['condition_notes'] = {'robot_location_and_heading': location_note,
                                'objects_and_fixtures': fixture_note,
                                'confirmed_preconditions': confirmed_conditions}
    extra['goal'] = {'step': 1, 'skill': skill, 'target': goal.target,
                     'relation': goal.relation, 'offset_m': goal.offset_m}
    return goal, extra


def run_skill(session, timeout):
    goal, extra = ask_skill_goal(session)
    monitor = session.monitor
    # Full SubGoalArray topic is transient-local in the installed Coordinator.
    # A distinct ID correlates this single step to its terminal TaskStatus.
    plan = monitor.SubGoalArray()
    plan.header.stamp = monitor.node.get_clock().now().to_msg()
    plan.plan_id = 3_000_000_000 + (os.getpid() % 1_000_000_000)
    plan.goals = [goal]
    session.event('skill_goal_approved', 'publish', {**extra, 'plan_id': plan.plan_id,
                                                      'precondition_state': monitor.snapshot()})
    session.active_plan_id = plan.plan_id
    publisher = monitor.node.create_publisher(monitor.SubGoalArray, '/task/sub_goals', monitor.plan_qos)
    if monitor.node.count_subscribers('/task/sub_goals') < 1:
        monitor.node.destroy_publisher(publisher)
        raise RuntimeError('Coordinator /task/sub_goals subscriber가 사라짐; goal 미전송.')
    publisher.publish(plan)
    print(f'Coordinator goal 전송 · plan_id={plan.plan_id}; terminal TaskStatus 대기 ({timeout}s).')
    deadline = time.monotonic() + timeout
    next_report = time.monotonic() + 5.0
    terminal = None
    try:
        while time.monotonic() < deadline:
            monitor.spin(0.1)
            s = monitor.task
            if s is not None and s.plan_id == plan.plan_id and _as_int(s.state) in (1, 2):
                terminal = {'plan_id': s.plan_id, 'step': s.step,
                            'state': 'SUCCEEDED' if _as_int(s.state) == 1 else 'FAILED',
                            'reason': s.reason}
                break
            if time.monotonic() >= next_report:
                snap = monitor.snapshot()
                print('진행:', snap['mission_phase'], snap['mission_note'],
                      'TaskStatus=', snap['task'], 'map/base=', snap['map_to_base_footprint'])
                next_report = time.monotonic() + 5.0
        if terminal is None:
            terminal = {'plan_id': plan.plan_id, 'state': 'TIMEOUT', 'reason': f'>{timeout}s'}
        if terminal['state'] in ('TIMEOUT', 'INTERRUPTED'):
            session.stop_owned_plan(plan.plan_id, publisher)
        session.event('skill_goal_result', terminal['state'], {**extra, **terminal,
                                                               'postcondition_state': monitor.snapshot()})
        if terminal['state'] == 'SUCCEEDED':
            monitor.last_skill_success = goal.skill
        print('Coordinator 결과:', json.dumps(terminal, ensure_ascii=False))
        observation = input('실측 결과/실패 원인 (측정하지 않았다면 그 이유를 기록): ').strip()
        terminal['operator_observation'] = observation or '관찰 미기록'
        session.event('operator_measurement', 'recorded', {**extra, **terminal})
    except KeyboardInterrupt:
        terminal = {'plan_id': plan.plan_id, 'state': 'INTERRUPTED'}
        session.event('skill_goal_result', 'INTERRUPTED', {**extra, **terminal})
        print('\n취소 신호를 Coordinator에 전달합니다.')
        session.stop_owned_plan(plan.plan_id, publisher)
    finally:
        # Releasing the transient-local writer clears the completed one-step request.
        monitor.node.destroy_publisher(publisher)
        session.active_plan_id = None
    if terminal['state'] != 'SUCCEEDED':
        print('실패 직전 상태/원인은 events.jsonl + bag /task/status, /diagnostics_agg에서 확인하세요.')


def run_drive(session):
    state = session.safety(require_auto=False)
    if state['operation'] != 'MANUAL':
        raise RuntimeError('수동 휠 시험은 MANUAL mode에서만 실행. TUI로 mode manual을 선택한 뒤 재시도.')
    print('고정 저속 프로브: 전진 0.06 m/s 0.4 s, 회전 0.20 rad/s 0.4 s 이하.')
    direction = input('방향 forward/back/left/right: ').strip()
    if direction not in ('forward', 'back', 'left', 'right'):
        raise RuntimeError('지원하지 않는 방향.')
    start_location = input('출발 로봇 위치·방향 (필수): ').strip()
    clearance = input('주행 방향의 퍽/받침대/장애물 배치 (없으면 none, 필수): ').strip()
    if not start_location or not clearance:
        raise RuntimeError('출발 위치와 주변 작업물 배치를 기록해야 drive를 보낼 수 있음.')
    if not ask('바퀴 앞·뒤 공간이 비었고 손을 E-stop에 뒀음', 'MOVE'):
        raise RuntimeError('주행 승인 취소됨.')
    if direction in ('left', 'right'):
        args = ['drive', direction, '--speed', '0', '--angular', '0.20', '--duration', '0.4']
    else:
        args = ['drive', direction, '--speed', '0.06', '--angular', '0', '--duration', '0.4']
    session.event('manual_drive_approved', 'start', {'direction': direction, 'speed_cap': 0.06,
                                                      'angular_cap': 0.20, 'duration_cap': 0.4,
                                                      'start_location': start_location,
                                                      'clearance_and_fixtures': clearance,
                                                      'precondition_state': state})
    result = subprocess.run([str(RUN_CLI), *args], cwd=TOOLS)
    observation = input('줄자/바닥 마커 기준 이동 거리·회전·미끄럼 또는 오류: ').strip()
    session.event('manual_drive_result', 'exit', {'exit_code': result.returncode, 'args': args,
                                                   'operator_observation': observation or '관찰 미기록',
                                                   'postcondition_state': session.monitor.snapshot()})
    if result.returncode:
        raise RuntimeError('run-cli drive 실패; mode/E-stop/장치 진단을 확인하세요.')


def run_mission(session, timeout):
    state = session.safety(require_auto=True, require_idle=True)
    basket = session.monitor.basket
    if basket is None:
        raise RuntimeError('Coordinator basket 상태가 아직 도착하지 않음; 장부를 확인할 수 없어 full mission을 차단.')
    if basket.held.filled or any(slot.filled for slot in basket.slots):
        raise RuntimeError('손 또는 적재함에 퍽이 기록돼 있음; 물리 적재함을 확인하고 비운 뒤 재시도.')
    start_location = input('출발 위치·방향 및 사용할 map (필수): ').strip()
    puck_setup = input('퍽 ID/위치/모양과 출발 상자 배치 (필수): ').strip()
    platform_setup = input('받침대 위치·태그·주변 장애물 (필수): ').strip()
    if not start_location or not puck_setup or not platform_setup:
        raise RuntimeError('출발 pose, 퍽 배치, 받침대/장애물을 기록해야 full mission을 시작할 수 있음.')
    if not ask('현재 위치에서 전체 주사위 미션이 시작되며 여러 개의 nav/arm 동작이 이어짐', 'FULL-MISSION'):
        raise RuntimeError('전체 미션 실행 승인 취소됨.')
    if not ask('실제 경기장 배치·바코드·신호등이 준비됐고 작업 공간이 통제됨', 'START'):
        raise RuntimeError('미션 환경 확인이 취소됨.')
    next_report = time.monotonic() + 5.0
    msg = session.monitor.String(data='start')
    session.event('mission_start_approved', 'publish', {
        'precondition_state': state, 'start_location': start_location,
        'pucks': puck_setup, 'platforms_obstacles': platform_setup})
    session.monitor.mission_pub.publish(msg)
    session.mission_goal_active = True
    print('MissionManager 시작. 진행 상태:', session.monitor.mission.phase,
          '— 중지하려면 Ctrl+C 또는 runner 메뉴에서 abort를 선택하세요.')
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            session.monitor.spin(0.1)
            phase = getattr(session.monitor.mission, 'phase', '')
            if phase in ('DONE', 'ERROR'):
                observation = input('미션 결과·배치 오차·실패한 구간을 기록: ').strip()
                session.event('mission_terminal', phase, {'mission': session.monitor.snapshot(),
                                                           'operator_observation': observation or '관찰 미기록'})
                session.mission_goal_active = False
                print('미션 종료:', phase, getattr(session.monitor.mission, 'error', ''))
                return
            if time.monotonic() >= next_report:
                snap = session.monitor.snapshot()
                print('미션 진행:', snap['mission_phase'], snap['mission_note'],
                      'TaskStatus=', snap['task'], 'map/base=', snap['map_to_base_footprint'])
                next_report = time.monotonic() + 5.0
        session.event('mission_terminal', 'TIMEOUT', {'mission': session.monitor.snapshot()})
        raise RuntimeError(f'미션 {timeout}s timeout; MissionManager가 ERROR/완료인지 확인.')
    except KeyboardInterrupt:
        if ask('MissionManager에 /cmd/mission abort를 보냄', 'ABORT'):
            session.monitor.mission_pub.publish(session.monitor.String(data='abort'))
            session.event('mission_abort_approved', 'published', {'mission': session.monitor.snapshot()})
            session.mission_goal_active = False
        else:
            print('abort를 보내지 않음. physical E-stop을 사용할 준비를 하세요.')


def start_bag(label, include_camera_data=False):
    cmd = [str(RUN_CLI), 'bag', 'start', '--label', label, '--regex', bag_regex(include_camera_data)]
    result = subprocess.run(cmd, cwd=TOOLS, capture_output=True, text=True)
    print(result.stdout, end='')
    if result.stderr:
        print(result.stderr, file=sys.stderr, end='')
    if result.returncode:
        raise RuntimeError('bag start 실패; 기존 bag 상태 확인 후 재시도.')
    match = re.search(r'녹화 시작:\s*(.+)', result.stdout)
    if not match:
        raise RuntimeError('bag 경로를 출력에서 확인하지 못함; bag status로 실제 상태 점검.')
    return Path(match.group(1).strip())


def active_rosbag_recorders():
    """Linux /proc의 ros2 bag record만 읽는다. 기존 recorder는 자동 중지하지 않는다."""
    found = []
    proc = Path('/proc')
    if not proc.is_dir():
        return found
    for directory in proc.iterdir():
        if not directory.name.isdigit():
            continue
        try:
            args = directory.joinpath('cmdline').read_bytes().decode(errors='replace').split('\0')
            args = [arg for arg in args if arg]
            if 'ros2' not in [Path(arg).name for arg in args] or 'bag' not in args or 'record' not in args:
                continue
            cwd = directory.joinpath('cwd').resolve()
            output = next((args[i + 1] for i, arg in enumerate(args[:-1]) if arg in ('-o', '--output')), '')
            bag = Path(output).expanduser()
            if output and not bag.is_absolute():
                bag = cwd / bag
            found.append({'pid': int(directory.name), 'bag': str(bag.resolve()) if output else None,
                          'args': args})
        except (OSError, ValueError, IndexError):
            continue
    return found


def inspect_baseline_bag(path):
    bag = Path(path).expanduser().resolve()
    if not bag.exists() or not (bag / 'metadata.yaml').is_file():
        raise RuntimeError('비교 기준 bag에 metadata.yaml이 없음.')
    if not list(bag.glob('*.mcap')) and not list(bag.glob('*.db3')):
        raise RuntimeError('비교 기준 bag에 MCAP/DB3 데이터가 없음.')
    info = subprocess.run(['ros2', 'bag', 'info', str(bag)], capture_output=True, text=True)
    if info.returncode:
        raise RuntimeError('비교 기준 ros2 bag info 실패:\n' + info.stderr[-1500:])
    print('기존 bag · 읽기 전용 비교 기준:\n' + info.stdout)
    return bag


def finish_session(session):
    if session.mission_goal_active:
        if not ask('진행 중인 전체 미션을 abort', 'ABORT'):
            session.event('session_detached', 'mission_still_active', {'bag_kept_recording': session.owns_bag})
            print('미션은 계속 실행 중. bag 녹화도 계속 둡니다.')
            print('종료 뒤 새 session에서 기존 bag을 --reuse로 열어 마감하세요:', session.bag_path)
            print('events.jsonl:', session.log)
            return
        session.monitor.mission_pub.publish(session.monitor.String(data='abort'))
        session.event('mission_abort_approved', 'published')
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and getattr(session.monitor.mission, 'phase', '') not in ('IDLE', 'ERROR'):
            session.monitor.spin(0.1)
        if getattr(session.monitor.mission, 'phase', '') not in ('IDLE', 'ERROR'):
            session.event('mission_abort_result', 'no_terminal_state')
            print('abort 상태 확인 실패; bag을 계속 녹화합니다:', session.bag_path)
            return
        session.mission_goal_active = False
    if session.owns_bag:
        stopped = subprocess.run([str(RUN_CLI), 'bag', 'stop'], cwd=TOOLS, check=False)
        if stopped.returncode:
            print('bag stop 반환 오류; run-cli bag status로 녹화 상태 확인하세요.', file=sys.stderr)
    session.event('session_finish', 'closed', {'baseline_bag': session.baseline_bag})
    print('시험 기록:', session.log)
    print('bag:', session.bag_path)
    info = subprocess.run(['ros2', 'bag', 'info', session.bag_path], capture_output=True, text=True)
    if info.returncode == 0:
        print('\n녹화 종료 metadata:')
        print(info.stdout)
    else:
        print('metadata 확인 실패:', info.stderr[-1000:], file=sys.stderr)


def run_session(args):
    global monitor
    monitor = Monitor()
    monitor.settle(2.0)
    initial = monitor.snapshot()
    if initial['estop'] is None or initial['mission_phase'] is None:
        raise RuntimeError('ROS 상태/MissionManager 응답 없음. 실행 전에 bringup와 domain을 확인.')
    if initial['estop'] or initial['mission_phase'] not in ('IDLE', 'DONE'):
        raise RuntimeError(f'기록 시작 전 상태가 안전하지 않음: E-stop={initial["estop"]}, phase={initial["mission_phase"]}')
    baseline = inspect_baseline_bag(args.baseline) if args.baseline else None
    running_recorders = active_rosbag_recorders()
    if len(running_recorders) > 1:
        raise RuntimeError(f'rosbag recorder가 {len(running_recorders)}개 실행 중; 하나씩 정리하고 경로를 지정하세요: {running_recorders}')
    owns_bag = not bool(running_recorders)
    if running_recorders:
        active = running_recorders[0]
        if not args.attach_active or not active['bag']:
            raise RuntimeError(f'이미 ros2 bag record 실행 중 (pid={active["pid"]}, bag={active["bag"]}). '
                               '기존 기록에 붙으려면 --attach-active, 새 bag이 필요하면 기존 recorder부터 정상 종료하세요.')
        bag = Path(active['bag'])
        if not bag.is_dir():
            raise RuntimeError(f'실행 중 recorder의 output 폴더가 아직 없음: {bag}')
        print(f'기존 rosbag에 append 없이 연결 · 종료 시 recorder는 건드리지 않음: {bag}')
    else:
        bag = start_bag(args.label, args.include_camera_data)
    session = Session(monitor, bag, baseline)
    session.owns_bag = owns_bag
    session.event('session_start', 'ready', {'comparison_baseline': str(baseline) if baseline else None,
                                              'attached_to_recorder': not owns_bag,
                                              'include_camera_data': args.include_camera_data})
    print('\nField session: bag 녹화와 별도 events.jsonl을 시작했습니다.')
    print('기준 bag:', str(baseline) if baseline else '(미지정)')
    print('메뉴: [s] 상태/센서 관찰 · [d] 짧은 MANUAL 휠 · [k] 단일 skill · [m] 전체 미션 · [f] 종료')
    print('모든 이동은 단계별 확인을 받고, 전체 미션은 FULL-MISSION 문구를 요구합니다.')
    try:
        while True:
            choice = input('\n[s/d/k/m/f] > ').strip().lower()
            try:
                if choice == 's':
                    session.monitor.settle(2.0)
                    snap = session.monitor.snapshot()
                    print(json.dumps(snap, ensure_ascii=False, indent=2, default=str))
                    session.event('observe', 'recorded', {'snapshot': snap})
                elif choice == 'd':
                    run_drive(session)
                elif choice == 'k':
                    run_skill(session, args.skill_timeout)
                elif choice == 'm':
                    run_mission(session, args.mission_timeout)
                elif choice == 'f':
                    if ask('현재 session 종료', 'END'):
                        break
                elif choice == '':
                    continue
                else:
                    print('s, d, k, m, f 중 선택.')
            except Exception as exc:
                session.event('step_not_completed', 'guard_or_runtime_error',
                              {'menu_choice': choice, 'error': str(exc)})
                print(f'이 단계는 실행 완료로 기록하지 않음: {exc}', file=sys.stderr)
                print('bag을 유지합니다. 조건/위치/작업물 배치를 고치고 같은 단계를 다시 선택할 수 있습니다.')
    except KeyboardInterrupt:
        print('\nCtrl+C 감지.')
    finally:
        finish_session(session)
        session.monitor.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description='ROS bag과 함께 실제 로봇을 단계별 검증하는 인터랙티브 runner')
    sub = parser.add_subparsers(dest='command', required=True)
    run = sub.add_parser('run', help='새 rosbag으로 field session 시작')
    run.add_argument('--label', default='field_test', help='bag 라벨')
    run.add_argument('--baseline', metavar='BAG_DIR', help='기존 bag을 읽기 전용 비교 기준으로 열고 fresh bag에 새 시험을 기록')
    run.add_argument('--attach-active', action='store_true', help='현재 ros2 bag record에 연결; 기존 recorder를 중지하지 않음')
    run.add_argument('--include-camera-data', action='store_true',
                     help='perception 시험용 color/depth/camera_info/YOLO 토픽도 녹화 (bag이 빠르게 커질 수 있음)')
    run.add_argument('--skill-timeout', type=int, default=180)
    run.add_argument('--mission-timeout', type=int, default=1800)
    args = parser.parse_args(argv)
    original_argv = list(sys.argv[1:] if argv is None else argv)
    if os.environ.get('BEE_FIELD_TEST_ROS_READY') != '1':
        bootstrap_ros(original_argv)
    import rclpy
    rclpy.init()
    try:
        run_session(args)
    except Exception as exc:
        print(f'Field test 중단: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
