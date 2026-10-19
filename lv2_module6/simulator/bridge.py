"""컨테이너(ROS)와 파일로 주고받기 — 시뮬레이터 창들이 같이 쓴다 (lv2_module6/debug)
  읽기: debug/output/topic/<토픽>/message  (faker monitor_manager가 쓴 마지막 메시지 JSON)
  쓰기: debug/input/topic/<토픽>/message  (faker input_manager가 그 토픽으로 발행)
  서비스·액션 (faker service_action_input): 요청 debug/input/service/<이름>/message · input/action/<이름>/goal|cancel
         → 응답 debug/output/service/<이름>/message · output/action/<이름>/accept|feedback|result
         · 요청·응답 기록 debug/input/<service|action>/<이름>/history (service_action_input이 쌓음)
"""
import collections
import json
import math
import os
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEBUG = HERE.parent / 'debug'
TOPIC_DIR = DEBUG / 'output' / 'topic'
INPUT_DIR = DEBUG / 'input' / 'topic'
LIVE_DIR = DEBUG / 'input' / 'hardware' / 'camera'
WHEEL_SEPARATION = 0.287


def read_msg(topic, _cache={}):
    """debug/output/topic/<토픽>/message → (data, 몇 초 전) · 없으면 (None, None).
    같은 mtime이면 파싱 결과 재사용 (diagnostics_agg 같은 대용량 매 틱 파싱 방지) — 반환값 변경 금지"""
    path = TOPIC_DIR / topic.strip('/') / 'message'
    try:
        st = path.stat()
        hit = _cache.get(topic)
        if hit is not None and hit[0] == (st.st_mtime_ns, st.st_size):
            return hit[1], time.time() - st.st_mtime
        msg = json.loads(path.read_text())  # NaN(effort 등)도 읽힘
        data = msg.get('data')
        _cache[topic] = ((st.st_mtime_ns, st.st_size), data)
        if len(_cache) > 64:  # 토픽 수십 개 수준 — 무한 증가 방지
            _cache.pop(next(iter(_cache)))
        return data, time.time() - st.st_mtime
    except (OSError, ValueError):
        return None, None


def read_points(topic, _cache={}):
    """debug/output/topic/<토픽>/data (monitor_manager — PointCloud2: 첫 줄 JSON {frame, stamp, count} + float32 xyz) →
    (N×3 float32 배열, 메타, 몇 초 전) · 없으면 (None, None, None). 같은 mtime이면 재사용"""
    path = TOPIC_DIR / topic.strip('/') / 'data'
    try:
        st = path.stat()
        hit = _cache.get(topic)
        if hit is None or hit[0] != st.st_mtime_ns:
            import numpy as np  # bee.py(CLI)는 numpy 없이도 돌게 여기서만
            raw = path.read_bytes()
            nl = raw.index(b'\n')
            meta = json.loads(raw[:nl])
            xyz = np.frombuffer(raw[nl + 1:], '<f4')[:meta['count'] * 3].reshape(-1, 3)
            hit = _cache[topic] = (st.st_mtime_ns, xyz, meta)
        return hit[1], hit[2], time.time() - st.st_mtime
    except (OSError, ValueError, KeyError):
        return None, None, None


def read_grid(topic, _cache={}):
    """debug/output/topic/<토픽>/data (monitor_manager — OccupancyGrid: 첫 줄 JSON {frame, stamp, resolution, width, height,
    origin:[x,y,yaw]} + int8 칸 값) → (height×width int8 배열, 메타, 몇 초 전) · 없으면 (None, None, None)"""
    path = TOPIC_DIR / topic.strip('/') / 'data'
    try:
        st = path.stat()
        hit = _cache.get(topic)
        if hit is None or hit[0] != st.st_mtime_ns:
            import numpy as np
            raw = path.read_bytes()
            nl = raw.index(b'\n')
            meta = json.loads(raw[:nl])
            grid = np.frombuffer(raw[nl + 1:], np.int8)[:meta['width'] * meta['height']].reshape(meta['height'], meta['width'])
            hit = _cache[topic] = (st.st_mtime_ns, grid, meta)
        return hit[1], hit[2], time.time() - st.st_mtime
    except (OSError, ValueError, KeyError):
        return None, None, None


class FileTail:
    """파일 끝에 새로 붙은 줄만 읽는다 — 크기가 바뀌었을 때만 읽음 (가상 시리얼 ↔ 파일: debug/output·input/hardware/serial)
    back: 처음 열 때 끝에서 몇 바이트 앞부터 볼지 (0 = 지금부터 붙는 것만). 파일이 줄면(새 실행으로 비워짐) 처음부터.
    poll() → 새로 읽은 완전한 줄 목록. 최근 keep줄은 lines에 (읽은 시각, 줄)로 남는다 (화면용)"""

    def __init__(self, path, back=0, keep=60):
        self.path, self.back = path, back
        self.offset, self.rest = None, ''
        self.lines = collections.deque(maxlen=keep)

    def poll(self):
        try:
            size = os.path.getsize(self.path)
        except OSError:
            return []
        if self.offset is None or size < self.offset:
            self.offset = max(0, size - self.back) if self.offset is None else 0
            self.rest = ''
        if size == self.offset:
            return []
        with open(self.path, 'rb') as f:
            f.seek(self.offset)
            chunk = f.read(min(size - self.offset, 1 << 20))
        self.offset += len(chunk)
        *new, self.rest = (self.rest + chunk.decode('ascii', 'ignore')).split('\n')
        now = time.time()
        new = [line for line in new if line]
        self.lines.extend((now, line) for line in new)
        return new


def write_atomic(path, write, _made=set()):
    """임시 파일에 쓰고 rename — 읽는 쪽(fake_camera·input_manager)이 반쯤 쓰인 파일을 보지 않게
    폴더는 처음 한 번만 만든다 (Docker Desktop 공유 폴더는 stat 한 번이 수 ms — 카메라 프레임마다 부르면 쌓임)"""
    if path.parent not in _made:
        path.parent.mkdir(parents=True, exist_ok=True)
        _made.add(path.parent)
    tmp = path.with_name(f'.tmp-{os.getpid()}-{path.name}')  # 프로세스마다 다른 임시 이름 — 여러 프로세스가 같은 파일을 써도 안 섞임
    try:
        write(tmp)
        os.replace(tmp, path)
    except FileNotFoundError:  # 그 사이 폴더가 지워짐 (컨테이너 정리 등) — 다시 만들고 한 번 더
        path.parent.mkdir(parents=True, exist_ok=True)
        write(tmp)
        os.replace(tmp, path)


def send(topic, msg_type, data):
    """debug/input/topic/<토픽>/message → input_manager가 발행"""
    write_atomic(INPUT_DIR / topic.strip('/') / 'message',
                 lambda p: p.write_text(json.dumps({'type': msg_type, 'data': data}, ensure_ascii=False)))


HEADER = {'stamp': {'sec': 0, 'nanosec': 0}, 'frame_id': 'base_footprint'}


def send_bool(topic, value):
    send(topic, 'std_msgs/msg/Bool', {'data': bool(value)})


def cmd_manual_mode(manual):
    """조작 모드 요청 → state_manager (결과는 /state/operation_mode)"""
    send('/cmd/operation_mode', 'std_msgs/msg/String', {'data': 'MANUAL' if manual else 'AUTO'})


def send_twist(v, w):
    send('/cmd/manual/cmd_vel', 'geometry_msgs/msg/TwistStamped',
         {'header': HEADER, 'twist': {'linear': {'x': v, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': w}}})


def send_arm(targets):
    send('/cmd/manual/arm', 'sensor_msgs/msg/JointState',
         {'header': HEADER, 'name': list(targets), 'position': list(targets.values()), 'velocity': [], 'effort': []})


def yaw_of(q):
    return math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']), 1 - 2 * (q['y'] ** 2 + q['z'] ** 2))


def current_joints(sim):
    """지금 관절값 dict + 바퀴 속도 [rad/s] — /joint_states (실기 모드면 fake_opencr S → BeebotSystemHardware가 낸 값)"""
    js, _ = read_msg('joint_states')
    if not js:
        return {}, [0.0, 0.0]
    q = {n: p for n, p in zip(js['name'], js['position']) if isinstance(p, float) and math.isfinite(p)}
    vel = dict(zip(js['name'], js['velocity'])) if js.get('velocity') else {}
    return q, [vel.get('wheel_left_joint', 0.0), vel.get('wheel_right_joint', 0.0)]


def read_service_list():
    """debug/service/list.json → (data, 몇 초 전) · 없으면 (None, None)"""
    path = DEBUG / 'service' / 'list.json'
    try:
        return json.loads(path.read_text()), time.time() - path.stat().st_mtime
    except (OSError, ValueError):
        return None, None


def read_action_list():
    """debug/action/list.json → (data, 몇 초 전) · 없으면 (None, None)"""
    path = DEBUG / 'action' / 'list.json'
    try:
        return json.loads(path.read_text()), time.time() - path.stat().st_mtime
    except (OSError, ValueError):
        return None, None


def read_text(path):
    """debug/service|action/<이름>/type|info 같은 텍스트 파일 읽기"""
    try:
        p = path if isinstance(path, Path) else Path(path)
        return p.read_text()
    except OSError:
        return None


def read_interface_list():
    """debug/interface/list → (text, 몇 초 전) · 없으면 (None, None)"""
    path = DEBUG / 'interface' / 'list'
    try:
        return path.read_text(), time.time() - path.stat().st_mtime
    except OSError:
        return None, None


def read_topic_list():
    """debug/output/topic/list.json → (data, 몇 초 전) · 없으면 (None, None)"""
    path = TOPIC_DIR / 'list.json'
    try:
        return json.loads(path.read_text()), time.time() - path.stat().st_mtime
    except (OSError, ValueError):
        return None, None


def _iface_rel(type_str):
    """'ml/srv/ScoreGrasps' → 'srv/ml/ScoreGrasps' (종류별 폴더뎁스)"""
    parts = type_str.strip().strip('/').split('/')
    if len(parts) != 3:
        return None
    pkg, kind, name = parts
    if kind not in ('msg', 'srv', 'action') or not pkg or not name:
        return None
    return f'{kind}/{pkg}/{name}'


def request_interface_info(type_str):
    """debug/interface/<종류>/<패키지>/<이름>/request 쓰기 → service_action_monitor가 옆에 info 생성"""
    rel = _iface_rel(type_str)
    if rel is None:
        raise ValueError(f'타입 형식 오류 (pkg/msg|srv|action/Name): {type_str}')
    write_atomic(DEBUG / 'interface' / rel / 'request',
                 lambda p: p.write_text(json.dumps({'type': type_str.strip()}, ensure_ascii=False)))


def read_interface_info(type_str):
    """debug/interface/<종류>/<패키지>/<이름>/info → (text, 몇 초 전) · 없으면 (None, None)"""
    rel = _iface_rel(type_str)
    paths = []
    if rel is not None:
        paths.append(DEBUG / 'interface' / rel / 'info')
    # 구 레이아웃 호환 (interface/<pkg>/<kind>/<Name>/info)
    paths.append(DEBUG / 'interface' / type_str.strip().strip('/') / 'info')
    for path in paths:
        try:
            return path.read_text(), time.time() - path.stat().st_mtime
        except OSError:
            continue
    return None, None


_svc_seq = 0


def _next_id(prefix):
    global _svc_seq
    _svc_seq += 1
    return f'{prefix}-{time.strftime("%H%M%S")}-{_svc_seq % 1000:03d}'


def send_service_request(service, data, request_id=None):
    """debug/input/service/<이름>/message 쓰기 → service_action_input이 호출. request_id(전문번호) 반환"""
    if request_id is None:
        request_id = _next_id('q')
    payload = {'request_id': request_id, 'data': data}
    write_atomic(DEBUG / 'input' / 'service' / service.strip('/') / 'message',
                 lambda p: p.write_text(json.dumps(payload, ensure_ascii=False)))
    return request_id


def read_service_response(service):
    """debug/output/service/<이름>/message → (dict, 몇 초 전) · 없으면 (None, None)"""
    path = DEBUG / 'output' / 'service' / service.strip('/') / 'message'
    try:
        return json.loads(path.read_text()), time.time() - path.stat().st_mtime
    except (OSError, ValueError):
        return None, None


def send_action_goal(action, data, goal_id=None):
    """debug/input/action/<이름>/goal 쓰기 → service_action_input이 전송. goal_id 반환"""
    if goal_id is None:
        goal_id = _next_id('g')
    payload = {'goal_id': goal_id, 'data': data}
    write_atomic(DEBUG / 'input' / 'action' / action.strip('/') / 'goal',
                 lambda p: p.write_text(json.dumps(payload, ensure_ascii=False)))
    return goal_id


def cancel_action_goal(action, goal_id):
    """debug/input/action/<이름>/cancel 쓰기 → 진행 중 goal 취소"""
    write_atomic(DEBUG / 'input' / 'action' / action.strip('/') / 'cancel',
                 lambda p: p.write_text(json.dumps({'goal_id': goal_id}, ensure_ascii=False)))


def read_action_file(action, name):
    """debug/output/action/<이름>/accept|feedback|result → (dict, 몇 초 전)"""
    path = DEBUG / 'output' / 'action' / action.strip('/') / name
    try:
        return json.loads(path.read_text()), time.time() - path.stat().st_mtime
    except (OSError, ValueError):
        return None, None
