"""OpenCR 연결 — 시뮬레이터는 OpenCR(컨테이너 faker의 fake_opencr)을 읽고 요청만 한다. 시리얼에 쓰지 않는다.

  OpenCR 역할(C 읽기 → 물리 → S 쓰기)은 테스트베드의 fake_opencr 하나 (faker 기본, mock_hardware:=true면 없음) — 실기는 진짜 OpenCR.
  시리얼(debug/output·input/hardware/serial)은 시뮬레이터 화면의 '시리얼' 장비 창이 읽어서 보여 주기만 한다.

  읽기 (monitor_manager 파일):
    /fake_opencr/ground_truth  진짜 위치 (map, 미끄럼까지 반영) → 카메라 렌더 · 지도 '실제 로봇'. map 원점 = 출발 칸
    /fake_opencr/bumped        벽에 막혔나
  요청:
    환경 (미끄럼 · 마찰 · 통신 끊김 · 팔 붙잡기) → 📞 /fake_opencr/set_parameters (바뀔 때만)
    위치 옮기기·초기화                         → ✉️ /cmd/fake_opencr/set_pose (geometry_msgs/PoseStamped, map 기준 — Lyrical엔 Pose2D 없음)
"""
import math

from bridge import read_msg, send, send_service_request, yaw_of

ROBOT_R = 0.18  # 로봇 외곽 반경 [m] — fake_opencr ROBOT_R과 같게 (퍽 밀기·장애물 충돌 판정)
# rcl_interfaces/ParameterType
_PARAM_TYPE = {bool: (1, 'bool_value'), float: (3, 'double_value')}


class OpenCR:
    def __init__(self, spawn):
        self.spawn = spawn
        # 마지막으로 요청한 환경 값 — fake_opencr 기본값(faker.yaml)과 같게 시작 → 같은 값은 안 보냄
        self._env = {'slip': 0.0, 'friction': 1.0, 'comm_down': False, 'arm_grabbed': False}
        self.slip = 0.0
        self.friction = 1.0
        self.comm_down = False
        self.arm_grabbed = False

    def __setattr__(self, name, value):
        # 환경 값은 바뀔 때만 fake_opencr 파라미터로 요청 (env_tick이 매 걸음 slip을 넣어도 한 번만 감)
        object.__setattr__(self, name, value)
        if name in ('slip', 'friction', 'comm_down', 'arm_grabbed'):
            v = bool(value) if isinstance(value, bool) else float(value)
            if self._env.get(name) != v:
                self._env[name] = v
                ptype, field = _PARAM_TYPE[type(v)]
                send_service_request('fake_opencr/set_parameters',
                                     {'parameters': [{'name': name, 'value': {'type': ptype, field: v}}]})

    @property
    def alive(self):
        """fake_opencr가 돌고 있나 (진짜 위치가 1초 안에 왔나) — faker 기본 (mock_hardware:=true면 없음)"""
        _, age = read_msg('fake_opencr/ground_truth')
        return age is not None and age < 1.0

    @property
    def bumped(self):
        b, _ = read_msg('fake_opencr/bumped')
        return bool(b and b.get('data'))

    def pose(self):
        """진짜 위치 (미로 좌표 x, y, yaw) — 없으면 출발 칸"""
        gt, _ = read_msg('fake_opencr/ground_truth')
        if not gt:
            return self.spawn[0], self.spawn[1], 0.0
        p = gt['pose']
        return p['position']['x'] + self.spawn[0], p['position']['y'] + self.spawn[1], yaw_of(p['orientation'])

    def set_pose(self, xy, yaw=0.0):
        """위치 옮기기 (미로 좌표) → fake_opencr"""
        send('/cmd/fake_opencr/set_pose', 'geometry_msgs/msg/PoseStamped', {
            'header': {'stamp': {'sec': 0, 'nanosec': 0}, 'frame_id': 'map'},
            'pose': {'position': {'x': round(xy[0] - self.spawn[0], 3), 'y': round(xy[1] - self.spawn[1], 3), 'z': 0.0},
                     'orientation': {'x': 0.0, 'y': 0.0, 'z': round(math.sin(yaw / 2), 6), 'w': round(math.cos(yaw / 2), 6)}}})


if __name__ == '__main__':  # 자체 검사: 같은 값은 한 번만 요청
    sent = []
    globals()['send_service_request'] = lambda svc, data: sent.append((svc, data))
    o = OpenCR((1.0, 2.0))
    assert not sent  # 기본값은 안 보냄
    o.slip = 0.3
    o.slip = 0.3
    o.comm_down = True
    assert len(sent) == 2, sent
    assert sent[-1][1]['parameters'][0] == {'name': 'comm_down', 'value': {'type': 1, 'bool_value': True}}
    print('OK opencr')
