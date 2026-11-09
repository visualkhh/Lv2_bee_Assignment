"""도커 test-faker의 /debug 파일 위에서 도는 가짜 ROS층.

ros/msgs.py의 메시지 스텁을 쓰되 Node를 파일 브리지로 바꾼다.
호스트에서 진짜 operator_tool 코드를 그대로 돌리면서, 컨테이너의 ROS 스택과는
DDS가 아니라 monitor_manager·input_manager·service_action_bag_input이 만드는 파일로 주고받는다.

  구독   output/topic/<토픽>/message  {"type", "data"} JSON   ← monitor_manager
         output/topic/map/data        헤더 JSON + OccupancyGrid int8 (지도는 이쪽이 가볍다)
  발행   input/topic/<토픽>/message   {"type", "data"} JSON   → input_manager
  서비스 input/service/<이름>/message {"request_id", "data"}   → service_action_bag_input
         output/service/<이름>/message {"request_id", "ok", "data"|"error"}
  파라미터 debug/node/<노드>/params   "이름: 값" 텍스트 (읽기 전용 — monitor_manager)
  그래프 debug/node/list · debug/topic/<토픽>/info

서비스는 faker의 service_action_bag_input 화이트리스트만 실제로 호출된다
(map_saver/save_map · score_grasps · clear_octomap · reset · apply_planning_scene 등).
파라미터 변경·lifecycle·컨트롤러 토글은 아직 faker에 입력 통로가 없어 실패로 답한다.
"""
from __future__ import annotations

import json
import os
import re
import sys
import time
import uuid
from pathlib import Path

from ..interface import DoneFuture, RosClient, RosNode, RosPublisher, RosSubscription, install_modules
from ..msgs import OccupancyGrid, _Msg, _ParamValue

# 메시지 클래스 이름 → ROS 타입. 발행할 때 input_manager가 이 타입으로 만든다.
_MSG_TYPES = {
    'String': 'std_msgs/msg/String',
    'Bool': 'std_msgs/msg/Bool',
    'Empty': 'std_msgs/msg/Empty',
    'Int32MultiArray': 'std_msgs/msg/Int32MultiArray',
    'Float32MultiArray': 'std_msgs/msg/Float32MultiArray',
    'TwistStamped': 'geometry_msgs/msg/TwistStamped',
    'PoseStamped': 'geometry_msgs/msg/PoseStamped',
    'Twist': 'geometry_msgs/msg/Twist',
    'JointState': 'sensor_msgs/msg/JointState',
    'Odometry': 'nav_msgs/msg/Odometry',
    'Imu': 'sensor_msgs/msg/Imu',
    'CameraInfo': 'sensor_msgs/msg/CameraInfo',
    'OccupancyGrid': 'nav_msgs/msg/OccupancyGrid',
    'DiagnosticArray': 'diagnostic_msgs/msg/DiagnosticArray',
}

_tmp_seq = 0


# ---------------------------------------------------------------- 변환·파일
def _obj(value):
    """JSON dict/list → 속성으로 읽는 객체 (operator_tool 콜백이 msg.field를 쓴다)."""
    if isinstance(value, dict):
        return _Msg(**{k: _obj(v) for k, v in value.items()})
    if isinstance(value, list):
        return [_obj(v) for v in value]
    return value


def _plain(value):
    """메시지 객체 → JSON dict (input_manager의 serializeFromJson 형식)."""
    if isinstance(value, (str, bool, int, float)) or value is None:
        return value
    if isinstance(value, dict):
        return {k: _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    out = {}
    for k, v in vars(value).items():
        if k.startswith('_'):
            continue
        out[k] = _plain(v)
    return out


def _write_atomic(path: Path, text: str) -> None:
    global _tmp_seq
    path.parent.mkdir(parents=True, exist_ok=True)
    _tmp_seq += 1
    tmp = path.parent / f'.tmp-{os.getpid()}-{_tmp_seq}'
    tmp.write_text(text)
    os.replace(tmp, path)


def _read_json(path: Path):
    return json.loads(path.read_text())


class _Now:
    def to_msg(self):
        t = time.time()
        sec = int(t)
        return {'sec': sec, 'nanosec': int((t - sec) * 1e9)}


class _Clock:
    def now(self):
        return _Now()


# ---------------------------------------------------------------- 구독
class _FileSub(RosSubscription):
    """토픽 파일 하나를 폴링해 콜백으로 넘긴다. 첫 폴링은 현재 파일을 바로 준다."""

    def __init__(self, node: 'DockerRos', topic: str, callback):
        self.node = node
        self.topic = topic
        self.callback = callback
        self.destroyed = False
        self._last = None
        self._warned_at = 0.0

    def destroy(self) -> None:
        self.destroyed = True

    def _message_path(self) -> Path:
        return self.node.debug_dir / 'output' / 'topic' / self.topic.lstrip('/') / 'message'

    def _grid_path(self) -> Path:
        return self.node.debug_dir / 'output' / 'topic' / self.topic.lstrip('/') / 'data'

    def poll(self) -> None:
        if self.destroyed:
            return
        use_grid = (self.topic == '/map' or self.topic.endswith('/costmap')) and self._grid_path().is_file()
        path = self._grid_path() if use_grid else self._message_path()
        try:
            st = path.stat()
        except OSError:
            return
        key = (path, st.st_mtime_ns, st.st_size)
        if key == self._last:
            return
        self._last = key
        try:
            msg = self._load_grid(path) if use_grid else self._load_message(path)
        except Exception as exc:  # noqa: BLE001 — 파일이 쓰이는 중이거나 형식이 깨짐
            now = time.monotonic()
            if now - self._warned_at > 5.0:
                self._warned_at = now
                print(f'[bridge] {self.topic} 파일 읽기 실패: {exc}', file=sys.stderr)
            return
        try:
            self.callback(msg)
        except Exception as exc:  # noqa: BLE001 — 실제 ROS 콜백 예외와 같게 화면만 알린다
            print(f'[bridge] 콜백 에러 {self.topic}: {exc}', file=sys.stderr)

    @staticmethod
    def _load_message(path: Path):
        doc = _read_json(path)
        return _obj(doc.get('data'))

    @staticmethod
    def _load_grid(path: Path):
        raw = path.read_bytes()
        nl = raw.find(b'\n')
        meta = json.loads(raw[:nl] if nl >= 0 else raw)
        payload = raw[nl + 1:] if nl >= 0 else b''
        n = int(meta['width']) * int(meta['height'])
        vals = [b if b < 128 else b - 256 for b in payload[:n]]  # int8
        m = OccupancyGrid()
        origin = list(meta.get('origin') or [0.0, 0.0, 0.0])
        m.info = _Msg(
            width=int(meta['width']), height=int(meta['height']),
            resolution=float(meta['resolution']),
            origin=_Msg(
                position=_Msg(x=float(origin[0]), y=float(origin[1]), z=0.0),
                orientation=_Msg(x=0.0, y=0.0, z=0.0, w=1.0)))
        m.data = vals
        return m


# ---------------------------------------------------------------- 발행
class _FilePub(RosPublisher):
    def __init__(self, node: 'DockerRos', msg_type, topic: str):
        self.node = node
        self.topic_name = topic
        self.type_name = _MSG_TYPES.get(getattr(msg_type, '__name__', ''))

    def get_subscription_count(self) -> int:
        """진짜 구독자 수 — monitor_manager가 남긴 debug/topic/<토픽>/info."""
        info = self.node.debug_dir / 'topic' / self.topic_name.lstrip('/') / 'info'
        try:
            text = info.read_text()
        except OSError:
            return 0
        m = re.search(r'^Subscription count:\s*(\d+)', text, re.M)
        return int(m.group(1)) if m else 0

    def publish(self, msg) -> None:
        if not self.type_name:
            print(f'[bridge] 발행 타입 모름: {type(msg).__name__} ({self.topic_name})',
                  file=sys.stderr)
            return
        data = _plain(msg)
        if isinstance(data, dict) and isinstance(data.get('header'), dict):
            data['header'].setdefault('stamp', {'sec': 0, 'nanosec': 0})
        text = json.dumps({'type': self.type_name, 'data': data}, ensure_ascii=False)
        _write_atomic(self.node.debug_dir / 'input' / 'topic' / self.topic_name.lstrip('/') / 'message', text)
        print(f'[PUB→docker {self.topic_name}] {text}')


# ---------------------------------------------------------------- 서비스
class _ServiceFuture:
    def __init__(self, out_path: Path, request_id: str, name: str):
        self.out_path = out_path
        self.request_id = request_id
        self.name = name
        self._done = False
        self._result = None
        self.ok = True
        self.error = ''

    def _poll(self) -> None:
        if self._done:
            return
        try:
            doc = _read_json(self.out_path)
        except (OSError, ValueError):
            return
        if doc.get('request_id') != self.request_id:
            return  # 다른 요청의 늦은 응답
        if doc.get('ok'):
            self.ok = True
            self._result = _obj(doc.get('data') or {})
        else:
            err = doc.get('error') or '알 수 없음'
            self.ok = False
            self.error = err
            print(f'[bridge] 서비스 {self.name} 실패: {err}', file=sys.stderr)
            self._result = _Msg(result=False, ok=False, error=err)
        self._done = True

    def done(self) -> bool:
        self._poll()
        return self._done

    def result(self):
        self._poll()
        return self._result


class _FileClient(RosClient):
    def __init__(self, node: 'DockerRos', name: str):
        self.node = node
        self.name = name
        self.req_dir = node.debug_dir / 'input' / 'service' / name.lstrip('/')
        self.out_path = node.debug_dir / 'output' / 'service' / name.lstrip('/') / 'message'

    def wait_for_service(self, timeout_sec: float = 1.0) -> bool:
        end = time.monotonic() + max(0.0, timeout_sec)
        while True:
            if self.req_dir.is_dir():
                return True
            if time.monotonic() >= end:
                return False
            time.sleep(0.05)

    def call_async(self, req):
        rid = uuid.uuid4().hex
        _write_atomic(self.req_dir / 'message',
                      json.dumps({'request_id': rid, 'data': _plain(req)}, ensure_ascii=False))
        print(f'[SVC→docker {self.name}] request_id={rid}')
        return _ServiceFuture(self.out_path, rid, self.name)

    def destroy(self) -> None:
        pass


# ---------------------------------------------------------------- 파라미터 (읽기 전용)
_INT_RE = re.compile(r'^[+-]?\d+$')
_FLOAT_RE = re.compile(r'^[+-]?(\d+\.\d*|\.\d+|\d+(\.\d*)?e[+-]?\d+)$', re.I)
_BOOL_RE = re.compile(r'^(true|false)$', re.I)

_BOOL, _INT, _DOUBLE, _STRING = 1, 2, 3, 4
_BOOL_ARRAY, _INT_ARRAY, _DOUBLE_ARRAY, _STRING_ARRAY = 6, 7, 8, 9


def _array_items(text: str):
    inner = text[1:-1].strip()
    return [] if not inner else [p.strip() for p in inner.split(',')]


def _param_type_value(text: str):
    """rclcpp value_to_string 텍스트 → (ParameterType, rclpy ParameterValue 스텁)."""
    text = '' if text is None else text
    pv = _ParamValue()
    if _BOOL_RE.match(text):
        pv.type, pv.bool_value = _BOOL, text.lower() == 'true'
        return pv.type, pv
    if _INT_RE.match(text):
        pv.type, pv.integer_value = _INT, int(text)
        return pv.type, pv
    if _FLOAT_RE.match(text):
        pv.type, pv.double_value = _DOUBLE, float(text)
        return pv.type, pv
    if text.startswith('[') and text.endswith(']'):
        items = _array_items(text)
        if items and all(_BOOL_RE.match(x) for x in items):
            pv.type, pv.bool_array_value = _BOOL_ARRAY, [x.lower() == 'true' for x in items]
        elif items and all(_INT_RE.match(x) for x in items):
            pv.type, pv.integer_array_value = _INT_ARRAY, [int(x) for x in items]
        elif items and all(_FLOAT_RE.match(x) for x in items):
            pv.type, pv.double_array_value = _DOUBLE_ARRAY, [float(x) for x in items]
        else:
            pv.type, pv.string_array_value = _STRING_ARRAY, items
        return pv.type, pv
    pv.type, pv.string_value = _STRING, text
    return pv.type, pv


class AsyncParameterClient:
    """debug/node/<FQN>/params (monitor_manager 기록)를 파라미터 서비스처럼 읽는다.

    변경(set)은 faker에 입력 통로가 없어 실패로 답한다 — 화면에서 이유가 보이게.
    """

    def __init__(self, node: 'DockerRos', fqn: str):
        self.node = node
        self.fqn = fqn
        self.path = node.debug_dir / 'node' / fqn.lstrip('/') / 'params'

    def wait_for_services(self, timeout_sec: float = 1.0) -> bool:
        end = time.monotonic() + max(0.0, timeout_sec)
        while True:
            if self.path.is_file():
                return True
            if time.monotonic() >= end:
                return False
            time.sleep(0.05)

    def _entries(self) -> dict[str, str]:
        try:
            text = self.path.read_text()
        except OSError:
            return {}
        out = {}
        for line in text.splitlines():
            if ': ' in line:
                k, v = line.split(': ', 1)
                out[k] = v
            elif line.endswith(':'):
                out[line[:-1]] = ''
        return out

    def list_parameters(self, *args, **kwargs):
        names = sorted(self._entries())
        return DoneFuture(_Msg(result=_Msg(names=names)))

    def get_parameters(self, names):
        entries = self._entries()
        values = []
        for name in names:
            if name in entries:
                values.append(_param_type_value(entries[name])[1])
            else:
                pv = _ParamValue()
                pv.type = 0
                values.append(pv)
        return DoneFuture(_Msg(values=values))

    def describe_parameters(self, names):
        entries = self._entries()
        descs = []
        for name in names:
            ptype = _param_type_value(entries[name])[0] if name in entries else 0
            descs.append(_Msg(name=name, type=ptype, description='',
                                       additional_constraints='', read_only=False,
                                       dynamic_typing=False))
        return DoneFuture(_Msg(descriptors=descs))

    @staticmethod
    def _json_param_value(value):
        """Python 값 → rcl_interfaces/ParameterValue JSON."""
        if isinstance(value, bool):
            return {'type': 1, 'bool_value': value}
        if isinstance(value, int):
            return {'type': 2, 'integer_value': value}
        if isinstance(value, float):
            return {'type': 3, 'double_value': value}
        if isinstance(value, str):
            return {'type': 4, 'string_value': value}
        if isinstance(value, (list, tuple)):
            vals = list(value)
            if all(isinstance(v, bool) for v in vals):
                return {'type': 6, 'bool_array_value': vals}
            if all(isinstance(v, int) for v in vals):
                return {'type': 7, 'integer_array_value': vals}
            if all(isinstance(v, (int, float)) for v in vals):
                return {'type': 8, 'double_array_value': [float(v) for v in vals]}
            if all(isinstance(v, str) for v in vals):
                return {'type': 9, 'string_array_value': vals}
        return {'type': 0}

    def _set_params(self, params, atomic: bool):
        """debug/input/service/set_parameters → service_action_bag_input이 대상 노드의 set_parameters 호출."""
        debug = self.node.debug_dir
        name = 'set_parameters'
        req_path = debug / 'input' / 'service' / name / 'message'
        out_path = debug / 'output' / 'service' / name / 'message'
        req_id = f'cli-{os.getpid()}-{int(time.time() * 1000)}'
        values = []
        for p in params:
            pname = str(getattr(p, 'name', '') or '')
            pvalue = getattr(p, 'value', None)
            values.append({'name': pname, 'value': self._json_param_value(pvalue)})
        payload = {'request_id': req_id,
                   'data': {'node': self.fqn, 'parameters': values, 'atomic': bool(atomic)}}
        req_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = req_path.with_name(f'.{name}-{os.getpid()}-{_tmp_seq}')
        tmp.write_text(json.dumps(payload, ensure_ascii=False))
        os.replace(tmp, req_path)
        end = time.monotonic() + 5.0
        while time.monotonic() < end:
            try:
                doc = json.loads(out_path.read_text())
            except (OSError, ValueError):
                time.sleep(0.05)
                continue
            if str(doc.get('request_id', '')) != req_id:
                time.sleep(0.05)
                continue
            if not doc.get('ok'):
                reason = str(doc.get('error') or 'set_parameters 실패')
                return [_Msg(successful=False, reason=reason) for _ in params]
            data = doc.get('data') or {}
            results = data.get('results') or []
            out = []
            for i in range(len(params)):
                r = results[i] if i < len(results) and isinstance(results[i], dict) else {}
                out.append(_Msg(successful=bool(r.get('successful')),
                                reason=str(r.get('reason') or '')))
            if any(r.successful for r in out):
                try:  # monitor_manager 스냅샷도 새 값으로 — 다음 발견 때 덮어써도 값은 같다
                    entries = self._entries()
                    for p in params:
                        name = str(getattr(p, 'name', '') or '')
                        if name:
                            entries[name] = str(getattr(p, 'value', None))
                    self.path.parent.mkdir(parents=True, exist_ok=True)
                    self.path.write_text('\n'.join(f'{k}: {v}' for k, v in entries.items()) + '\n')
                except OSError:
                    pass
            return out
        return [_Msg(successful=False, reason='set_parameters 응답 없음 (서비스 브리지 확인)')
                for _ in params]

    def set_parameters(self, params):
        return DoneFuture(_Msg(results=self._set_params(params, atomic=False)))

    def set_parameters_atomically(self, params):
        results = self._set_params(params, atomic=True)
        ok = bool(results) and all(r.successful for r in results)
        reason = '' if ok else next((r.reason for r in results if not r.successful), '실패')
        return DoneFuture(_Msg(successful=ok, reason=reason))


# ---------------------------------------------------------------- 노드
class DockerRos(RosNode):
    """rclpy Node 대역 — 구독은 output/ 폴링, 발행은 input/ 쓰기."""

    debug_dir: Path | None = None

    def __init__(self, name: str):
        super().__init__(name, _Clock())
        assert self.debug_dir is not None, 'DockerRos.install(debug_dir) 먼저'
        self._req_path = self.debug_dir / 'input' / 'monitor' / f'cli-{os.getpid()}.topics'
        self._req_topics: set[str] = set()
        self._req_t = 0.0
        self._file_subs: list[_FileSub] = []
        self._file_clients: list[_FileClient] = []
        self._svc_list: dict = {}
        self._svc_list_key = None

    def _write_requests(self, force: bool = False) -> None:
        """호스트가 지금 구독 중인 토픽 → input/monitor/*.topics (monitor가 동적으로 구독)."""
        now = time.time()
        if not force and now - self._req_t < 2.0:
            return
        try:
            self._req_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._req_path.with_suffix('.tmp')
            tmp.write_text('\n'.join(sorted(self._req_topics)) + '\n')
            os.replace(tmp, self._req_path)
            self._req_t = now
        except OSError:
            pass

    def create_publisher(self, msg_type, topic, qos):
        return _FilePub(self, msg_type, topic)

    def create_subscription(self, msg_type, topic, callback, qos):
        sub = _FileSub(self, topic, callback)
        self._file_subs.append(sub)
        self._req_topics.add(topic)
        self._write_requests(force=True)
        return sub

    def destroy_subscription(self, sub) -> None:
        if isinstance(sub, _FileSub):
            sub.destroyed = True
            if sub in self._file_subs:
                self._file_subs.remove(sub)
            self._req_topics.discard(sub.topic)
            self._write_requests(force=True)

    def create_client(self, srv_type, topic):
        cli = _FileClient(self, topic)
        self._file_clients.append(cli)
        return cli

    def destroy_client(self, client) -> None:
        if client in self._file_clients:
            self._file_clients.remove(client)

    def poll_files(self) -> None:
        self._write_requests()
        for sub in list(self._file_subs):
            sub.poll()

    # ------------------------------------------------ 그래프 (monitor 기록)
    def get_node_names(self):
        return [n for n, _ in self.get_node_names_and_namespaces()]

    def get_node_names_and_namespaces(self):
        try:
            lines = (self.debug_dir / 'node' / 'list').read_text().splitlines()
        except OSError:
            return []
        out = []
        for line in lines:
            full = line.strip()
            if not full:
                continue
            full = '/' + full.lstrip('/')
            ns, _, name = full.rpartition('/')
            out.append((name, ns or '/'))
        return out

    def _topic_info(self, topic: str) -> str:
        try:
            return (self.debug_dir / 'topic' / topic.lstrip('/') / 'info').read_text()
        except OSError:
            return ''

    def count_subscribers(self, topic: str) -> int:
        """debug/topic/<토픽>/info의 Subscription count — rclpy Node API 호환 (docker 그래프에는 실시간 API가 없다)."""
        text = self._topic_info(topic)
        m = re.search(r'^Subscription count:\s*(\d+)', text, re.M)
        return int(m.group(1)) if m else 0

    def count_publishers(self, topic: str) -> int:
        text = self._topic_info(topic)
        m = re.search(r'^Publisher count:\s*(\d+)', text, re.M)
        return int(m.group(1)) if m else 0

    # ------------------------------------------------ 서비스 (service_action_bag_input 목록·파일 브리지)
    def _service_list(self) -> dict:
        path = self.debug_dir / 'output' / 'service' / 'list.json'
        try:
            st = path.stat()
            key = (st.st_mtime_ns, st.st_size)
            if self._svc_list_key == key:
                return self._svc_list
            doc = _read_json(path)
            self._svc_list = doc if isinstance(doc, dict) else {}
            self._svc_list_key = key
            return self._svc_list
        except (OSError, ValueError):
            return self._svc_list or {}

    def get_service_names_and_types(self):
        doc = self._service_list()
        return [(s.get('name', ''), list(s.get('types') or []))
                for s in doc.get('services', [])
                if isinstance(s, dict) and s.get('name')]

    def service_callable(self, name: str) -> bool:
        """service_action_bag_input이 타입을 고정해 실제 호출할 수 있는 서비스인지."""
        for s in self._service_list().get('services', []):
            if isinstance(s, dict) and s.get('name') == name:
                return bool(s.get('callable', True))
        return False

    def get_service_names_and_types_by_node(self, node_name, namespace='/'):
        doc = self._service_list()
        key = f'/{namespace.strip("/")}/{node_name}' if namespace not in ('/', '') else f'/{node_name}'
        names = doc.get('nodes', {}).get(key) or []
        types = {s.get('name'): list(s.get('types') or [])
                 for s in doc.get('services', []) if isinstance(s, dict)}
        return [(n, types.get(n, [])) for n in names]

    def call_service(self, service: str, type_name: str, request: dict):
        """기존 service_action_bag_input 파일 브리지로 서비스 요청 — (ok, data|error).

        타입은 service_action_bag_input이 서비스별로 고정해 둔 것만 실제 호출된다.
        목록에 없는 서비스는 '지원 안 함' 에러로 답한다.
        """
        cli = self.create_client(None, service)
        try:
            if not cli.wait_for_service(timeout_sec=1.0):
                return False, f'서비스 요청 폴더 없음: {service} (faker 서비스 브리지 확인)'
            fut = cli.call_async(request or {})
            end = time.monotonic() + 12.0
            while time.monotonic() < end and not fut.done():
                time.sleep(0.05)
            if not fut.done():
                return False, '응답 시간 초과 (12초)'
            if not getattr(fut, 'ok', True):
                return False, str(getattr(fut, 'error', '실패'))
            res = fut.result()
            return True, _plain(res) if hasattr(res, '__dict__') else (res or {})
        finally:
            self.destroy_client(cli)

    def get_publishers_info_by_topic(self, topic):
        text = self._topic_info(topic)
        if not text:
            return []
        m = re.search(r'^Publisher count:\s*(\d+)', text, re.M)
        if not m or int(m.group(1)) == 0:
            return []
        block = text.split('Publisher count:', 1)[1].split('Subscription count:', 1)[0]
        out = []
        for line in block.splitlines():
            line = line.strip()
            if not line.startswith('- '):
                continue
            endpoint = line[2:].split(' (', 1)[0].strip()
            ns, _, name = endpoint.rpartition('/')
            out.append(_Msg(node_name=name, node_namespace=ns or '/'))
        return out

    def destroy_node(self) -> None:
        self._file_subs.clear()
        self._file_clients.clear()
        try:
            self._req_path.unlink(missing_ok=True)
        except OSError:
            pass

    @classmethod
    def install(cls, debug_dir) -> None:
        cls.debug_dir = Path(debug_dir).resolve()
        install_modules(cls, parameter_client_cls=AsyncParameterClient,
                        spin_once=_spin_once,
                        spin_until_future_complete=_spin_until_future_complete,
                        custom_msgs=True)
        _warn_if_stale(cls.debug_dir)


# ---------------------------------------------------------------- rclpy 대역
def _spin_once(node, timeout_sec: float = 0.0) -> None:
    if isinstance(node, DockerRos):
        node.poll_files()
    time.sleep(min(max(0.0, float(timeout_sec)), 0.02))


def _spin_until_future_complete(node, future, timeout_sec: float = 1.0) -> None:
    end = time.monotonic() + max(0.0, float(timeout_sec))
    while time.monotonic() < end:
        if future.done():
            return
        time.sleep(0.02)


def _warn_if_stale(debug_dir: Path) -> None:
    """지난 실행의 debug 파일을 지금 상태로 오해하지 않게 — 한 줄만 알린다."""
    topic_dir = debug_dir / 'output' / 'topic'
    files = list(topic_dir.glob('*/message')) + list(topic_dir.glob('*/data'))
    if not files:
        print(f'[bridge] {topic_dir} 없음 — 도커에서 test-faker 0이 떠 있는지 확인',
              file=sys.stderr)
        return
    age = time.time() - max(p.stat().st_mtime for p in files)
    if age > 10.0:
        print(f'[bridge] debug 파일이 {age:.0f}초 전 것 — 도커 test-faker가 떠 있는지 확인',
              file=sys.stderr)
