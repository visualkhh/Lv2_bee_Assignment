"""ROS 없는 Mac에서 operator_tool UI/UX를 시험하는 가짜 ROS층.

진짜 operator_tool 코드를 그대로 import해 돌린다. 발행은 화면에 찍고,
/cmd/operation_mode·e_stop·map_mode·map_name·restart는 가짜 state_manager가
받아 상태를 바꾼다 (mode 전환·_wait_value까지 실제처럼 동작).

메시지 클래스는 ros/msgs.py, 공통 설치기는 ros/interface.py를 쓴다.
"""
from __future__ import annotations

import time
from pathlib import Path

from ..interface import DoneFuture, RosClient, RosNode, RosPublisher, RosSubscription, install_modules
from ..msgs import OccupancyGrid, SaveMapResponse, ToggleResponse, _Msg, _Time


# ---------------------------------------------------------------- 가짜 노드/버스
BUS: dict[str, list] = {}  # topic -> [(node, callback)]
PUB_LOG: list[tuple[str, object]] = []
LATEST: dict[str, object] = {}   # 토픽 → 마지막 메시지 (transient_local처럼 늦게 구독해도 바로 받음)
LATCHED = ('/mission/state', '/task/status', '/global_costmap/costmap', '/local_costmap/costmap')


class _Clock:
    def now(self):
        return _Time()


class _FakePub(RosPublisher):
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


class _FakeSub(RosSubscription):
    def destroy(self):
        pass


class _FakeClient(RosClient):
    def __init__(self, node, srv_type, topic):
        self._node = node
        self.srv_type = srv_type
        self.topic_name = topic

    def wait_for_service(self, timeout_sec=1.0):
        return True

    def call_async(self, req):
        print(f'[SVC {self.topic_name}] 요청 → 가짜 성공')
        url = getattr(req, 'map_url', '')
        if url and self.topic_name.endswith('save_map'):
            import shutil
            dst, src = Path(url), MAP_DIR / CURRENT['name']
            dst.parent.mkdir(parents=True, exist_ok=True)
            for ext in ('yaml', 'pgm'):
                shutil.copyfile(src / f'map.{ext}', dst.with_suffix(f'.{ext}'))
            state = getattr(self._node, 'state', None)
            if state is not None:
                state.values['map_list'] = '\n'.join(fake_map_names())
        return DoneFuture(ToggleResponse() if hasattr(req, 'enable') else SaveMapResponse(result=True))

    def destroy(self):
        pass


# ---------------------------------------------------------------- 가짜 /map — 저장 지도 폴더 ros/fake/map/<이름>/map.yaml + map.pgm
MAP_DIR = Path(__file__).resolve().parent / 'map'   # 실기 ros2_ws/map과 같은 모양 — 폴더 하나 = 지도 하나 (목록 · 전환 · 보기 · 저장이 이걸 씀)
CURRENT = {'name': 'default'}                        # 지금 /map으로 내보내는 지도 — 지도 전환·저장하면 바뀜


def fake_map_names() -> list[str]:
    return sorted(p.name for p in MAP_DIR.iterdir() if (p / 'map.yaml').is_file()) if MAP_DIR.is_dir() else []


def fake_map(name: str | None = None):
    """MAP_DIR/<이름>/의 지도 → OccupancyGrid (trinary: 밝음=빈 곳 0 · 어두움=벽 100 · 가운데=미확인 -1) — 지도에서 찍기 시험용."""
    here = MAP_DIR / (name or CURRENT['name'])
    cfg = {}
    for line in (here / 'map.yaml').read_text().splitlines():
        if ':' in line and not line.lstrip().startswith('#'):
            k, v = line.split(':', 1)
            cfg[k.strip()] = v.strip()
    raw = (here / 'map.pgm').read_bytes()
    toks, pos = [], 0
    while len(toks) < 4:   # P5 · 폭 높이 · 최대값 (# 주석 줄 건너뜀)
        while raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b'#':
            pos = raw.index(b'\n', pos) + 1
            continue
        end = pos
        while not raw[end:end + 1].isspace():
            end += 1
        toks.append(raw[pos:end])
        pos = end
    w, h = int(toks[1]), int(toks[2])
    pix = raw[pos + 1:pos + 1 + w * h]
    occ_t, free_t = float(cfg['occupied_thresh']), float(cfg['free_thresh'])
    neg = cfg.get('negate', '0') == '1'
    ox, oy = [float(t) for t in cfg['origin'].strip('[]').split(',')[:2]]
    data = []
    for y in range(h - 1, -1, -1):   # pgm은 위→아래, OccupancyGrid는 아래→위
        for x in range(w):
            v = pix[y * w + x] / 255.0
            p = v if neg else 1.0 - v
            data.append(100 if p > occ_t else (0 if p < free_t else -1))
    m = OccupancyGrid()
    m.info = _Msg(width=w, height=h, resolution=float(cfg['resolution']),
                  origin=_Msg(position=_Msg(x=ox, y=oy, z=0.0)))
    m.data = data
    return m


class FakeRos(RosNode):
    """operator_tool.OperatorTool의 부모 — 프로세스 안 가짜 ROS 구현."""

    def __init__(self, name):
        super().__init__(name, _Clock())
        self._fake_subs: list[tuple[str, object]] = []

    def create_publisher(self, msg_type, topic, qos):
        return _FakePub(self, topic)

    def create_subscription(self, msg_type, topic, callback, qos):
        BUS.setdefault(topic, []).append((self, callback))
        self._fake_subs.append((topic, callback))
        if topic == '/map':   # transient_local — 구독하면 마지막 지도가 바로 옴
            callback(LATEST.get('/map') or fake_map())
        elif topic in LATCHED and topic in LATEST:
            callback(LATEST[topic])
        return _FakeSub()

    def _deliver(self, topic, msg):
        """가짜 월드가 내는 토픽 — 구독자 콜백으로 바로 (발행 로그는 안 찍음)."""
        LATEST[topic] = msg
        for _node, cb in list(BUS.get(topic, [])):
            try:
                cb(msg)
            except Exception as e:  # noqa: BLE001
                print(f'[fake] 콜백 에러 {topic}: {e}')

    def _switch_map(self, name):
        """지도 전환 — /map을 그 폴더 지도로 다시 내보내고, 가짜 월드(벽·경로·costmap)도 그 지도로."""
        CURRENT['name'] = name
        grid = fake_map(name)
        self._deliver('/map', grid)
        world = getattr(self, 'world', None)
        if world is not None:
            world.reload(grid)

    def destroy_subscription(self, sub):
        pass

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
                'coordinator', 'perception',
                'pole_front_cam', 'gripper_cam',
                'joint_state_broadcaster', 'controller_manager']

    def get_node_names_and_namespaces(self):
        return [(n, '/') for n in self.get_node_names()]

    def _fake_on_publish(self, topic, msg):
        """가짜 state_manager — /cmd/*를 받아 상태를 바꾼다."""
        state = getattr(self, 'state', None)
        if state is None:
            return
        world = getattr(self, 'world', None)
        if world is not None:
            world.on_publish(topic, msg)
        vals = state.values
        data = getattr(msg, 'data', None)
        if topic == '/cmd/operation_mode' and isinstance(data, str):
            vals['operation_mode'] = data
        elif topic == '/cmd/e_stop' and isinstance(data, bool):
            state.emergency = data
        # 진짜 state_manager처럼 지도 모드·지도 바꾸기 = launch 재시작 (/state/restart로 응답, 다시 뜬 뒤 새 값).
        #   거절 이유(지도 없음 등)는 진단에만 남고 /state/restart엔 아무것도 안 옴 — 그것도 같게
        elif topic == '/cmd/map_mode' and isinstance(data, str):
            if data == 'NONE':
                arg = 'map:=none'
                vals['map_name'] = ''
            elif data == 'MAPPING':
                arg = 'map:=slam'
            else:
                arg = f'map:=saved map_name:={vals.get("map_name")}'
            vals['restart'] = f'all: 재시작 중 — launch pid 4242 · 바꾼 인자 {arg} (가짜)'
            vals['map_mode'] = data
        elif topic == '/cmd/map_name' and isinstance(data, str):
            if data in fake_map_names():
                self._switch_map(data)
                vals['restart'] = f'all: 재시작 중 — launch pid 4242 · 바꾼 인자 map:=saved map_name:={data} (가짜)'
                vals['map_mode'], vals['map_name'] = 'LOCALIZATION', data
        elif topic == '/cmd/restart' and isinstance(data, str):
            vals['restart'] = f'{data} (가짜 수락)'
            # map:= 인자가 있으면 가짜 state_manager가 지도 모드까지 갈아탄 것으로 친다
            import re as _re
            m = _re.search(r'map:=(\S+)', data)
            if m:
                mode = m.group(1)
                if mode == 'slam':
                    vals['map_mode'] = 'MAPPING'
                    vals['map_list'] = '\n'.join(fake_map_names())
                elif mode == 'saved':
                    vals['map_mode'] = 'LOCALIZATION'
                    nm = _re.search(r'map_name:=(\S+)', data)
                    vals['map_name'] = nm.group(1) if nm else 'default'
                    if vals['map_name'] in fake_map_names():
                        self._switch_map(vals['map_name'])

    @classmethod
    def install(cls):
        install_modules(cls, parameter_client_cls=AsyncParameterClient, spin_once=_spin, custom_msgs=True)


def _spin(node, timeout_sec=0.0):
    """rclpy.spin_once 대용 — 가짜 월드를 한 걸음 굴리고 잠깐 쉰다 (바쁜 대기 방지)."""
    world = getattr(node, 'world', None)
    if world is not None:
        world.step()
    time.sleep(min(max(timeout_sec, 0.0), 0.02))


class AsyncParameterClient:
    def __init__(self, node, fqn):
        self._node = node

    def wait_for_services(self, timeout_sec=1.0):
        return False  # 파라미터 명령은 거부 경로 UX만 확인
