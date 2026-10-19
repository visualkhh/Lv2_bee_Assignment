"""시뮬레이터 장면 — 미로 벽 · 퍽 · 바닥 · 360° 배경을 로봇 카메라 시점으로 렌더 (컬러 + 깊이)

- 렌더: 광선 추적(torch — CUDA·Apple GPU(MPS) 있으면 GPU, 없으면 CPU. SIM_DEVICE로 강제). 카메라 optical 프레임(z 앞 · x 오른쪽 · y 아래)에서 픽셀마다 광선을 쏴서
    벽(축 정렬 상자) · 퍽(원기둥 · 상자 · 구) · 바닥(z=0) 중 가장 가까운 것, 없으면 배경 파노라마.
    깊이 = optical z [m] → 16비트 mm. MIN_DEPTH보다 가깝거나 MAX_DEPTH보다 멀거나 배경이면 0 (RealSense처럼 측정 안 됨)
- 카메라 자세: 로봇이 내보내는 URDF(/robot_description)로 순기구학 + /joint_states 관절값 + /odom 위치
    world(미로) ← odom(원점 = 미로 spawn) ← base_footprint ← … ← <카메라>_color_optical_frame
- 화각: HFOV_DEG — 프레임 메타(<카메라>.json)에 같이 써서 fake_camera가 camera_info·점구름을 이 값으로 만든다 (한 곳).
- 자기 몸: URDF visual 상자를 광선 추적에 넣어 그린다 (뒤쪽 상판에 실은 퍽 확인용). 렌더 중인 카메라 자체 마운트 상자만 뺀다.
"""
import math
import random
import threading
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np

HFOV_DEG = 69.0
RENDER_W, RENDER_H = 320, 240          # 출력 크기 (fake_camera가 640×480으로 키운다)
SCALE = 0.5                            # 내부 렌더 배율 — 160×120으로 쏘고 320×240으로 확대 (약 4배 빠름)
IW, IH = int(RENDER_W * SCALE), int(RENDER_H * SCALE)
MIN_DEPTH, MAX_DEPTH = 0.2, 6.0         # D435 최소 측정 거리 약 0.2m
VIEW_RANGE = 4.5                        # ponytail: 이보다 먼 벽은 안 그림 (렌더 속도) — 넓은 코스면 늘릴 것
LIGHT = np.array([0.3, 0.5, 0.8]) / np.linalg.norm([0.3, 0.5, 0.8])
WALL_BGR = (204, 184, 166)              # 미로 판넬 (원본 model.sdf 색 0.65 0.72 0.8)
WALL_TILE_M = 0.5                        # 벽 텍스처 가로 반복 [m] (세로는 벽 높이로 늘이기)
PALETTE = {  # 퍽 색 (BGR) — 인지에서 구분하기 쉬운 원색 위주
    'red': (40, 40, 220), 'green': (60, 180, 60), 'blue': (200, 90, 30), 'yellow': (40, 220, 230),
    'orange': (30, 140, 250), 'purple': (170, 60, 140), 'white': (235, 235, 235), 'black': (40, 40, 40)}
SHAPES = ('cylinder', 'box', 'sphere')


# ---------- 좌표 변환
def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, angle):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def transform(R=None, t=(0, 0, 0)):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    T[:3, 3] = t
    return T


def quat_matrix(q):
    x, y, z, w = q['x'], q['y'], q['z'], q['w']
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Kinematics:
    """URDF 문자열 → 링크 자세 (루트 링크 기준). revolute·continuous·prismatic·fixed, mimic 지원"""

    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        self.parent = {}  # child link → joint
        for j in root.findall('joint'):  # 최상위만 (<ros2_control> 안의 joint는 다른 것)
            origin = j.find('origin')
            xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
            rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
            axis = j.find('axis')
            mimic = j.find('mimic')
            self.parent[j.find('child').get('link')] = {
                'name': j.get('name'), 'type': j.get('type'), 'parent': j.find('parent').get('link'),
                'origin': transform(rpy_matrix(*rpy), xyz),
                'axis': [float(v) for v in axis.get('xyz').split()] if axis is not None else [1, 0, 0],
                'mimic': (mimic.get('joint'), float(mimic.get('multiplier', 1)), float(mimic.get('offset', 0)))
                if mimic is not None else None}

    def pose(self, link, q):
        T = np.eye(4)
        while link in self.parent:
            j = self.parent[link]
            value = q.get(j['name'], 0.0)
            if j['mimic']:
                value = q.get(j['mimic'][0], 0.0) * j['mimic'][1] + j['mimic'][2]
            motion = np.eye(4)
            if j['type'] in ('revolute', 'continuous'):
                motion = transform(axis_angle(j['axis'], value))
            elif j['type'] == 'prismatic':
                motion = transform(t=np.asarray(j['axis']) * value)
            T = j['origin'] @ motion @ T
            link = j['parent']
        return T


# ---------- 퍽
def random_pucks(count, cells, spawn, rng=random):
    """cells: 놓을 수 있는 칸 중심 [(x, y)] (미로 좌표). spawn 칸은 피한다. 서로 0.15m 넘게 떨어뜨림"""
    pucks = []
    free = [c for c in cells if math.dist(c, spawn) > 0.3]
    tries = 0
    while len(pucks) < count and tries < 500:
        tries += 1
        cx, cy = rng.choice(free)
        x, y = cx + rng.uniform(-0.2, 0.2), cy + rng.uniform(-0.2, 0.2)
        if any(math.dist((x, y), (p['x'], p['y'])) < 0.15 for p in pucks):
            continue
        shape = rng.choice(SHAPES)
        color = rng.choice(list(PALETTE))
        # 파지 포락선: 닫힘 18mm < 퍽 < 열림 76mm → 지름 40~60mm (박스는 대각선 기준)
        size = {'cylinder': (rng.uniform(0.020, 0.030), rng.uniform(0.03, 0.05)),   # 반지름, 높이
                'box': (rng.uniform(0.040, 0.045), rng.uniform(0.03, 0.05)),        # 한 변, 높이
                'sphere': (rng.uniform(0.020, 0.030), 0.0)}[shape]                    # 반지름
        pucks.append({'id': len(pucks) + 1, 'shape': shape, 'color': color, 'x': round(x, 3), 'y': round(y, 3),
                      'yaw': round(rng.uniform(-math.pi, math.pi), 3), 'size': [round(v, 3) for v in size]})
    return pucks


def random_obstacle(cells, avoid, rng=random):
    """바닥 장애물 하나 — 칸 안 무작위, avoid [(x, y)]에서 0.35m 넘게 떨어진 곳 · 못 찾으면 None"""
    for _ in range(100):
        cx, cy = rng.choice(cells)
        x, y = cx + rng.uniform(-0.2, 0.2), cy + rng.uniform(-0.2, 0.2)
        if all(math.dist((x, y), a) > 0.35 for a in avoid):
            return {'kind': 'obstacle', 'shape': 'box', 'color': (130, 130, 130), 'x': round(x, 3), 'y': round(y, 3),
                    'yaw': round(rng.uniform(-math.pi, math.pi), 3), 'sx': round(rng.uniform(0.08, 0.2), 3),
                    'sy': round(rng.uniform(0.08, 0.2), 3), 'size': [0.1, round(rng.uniform(0.05, 0.2), 3)]}
    return None


def mover(x, y, heading, speed=0.35, life_s=12.0):
    """이동물체 — 원기둥 (반지름 0.2, 키 1.7), heading 방향으로 speed m/s, life_s 뒤 사라짐"""
    return {'kind': 'mover', 'shape': 'cylinder', 'color': (200, 120, 50), 'x': x, 'y': y, 'yaw': 0.0,
            'size': [0.2, 1.7], 'vx': speed * math.cos(heading), 'vy': speed * math.sin(heading), 'until': life_s}


TL_COLORS = {'red': (40, 40, 220), 'yellow': (40, 220, 230), 'green': (60, 180, 60),
               'off': (50, 50, 50)}
TL_CYCLE = [('green', 5.0), ('off', 1.0), ('yellow', 2.0), ('off', 1.0), ('red', 5.0), ('off', 1.0)]
TL_POLE_H = 0.39                           # 폴 높이 [m] — 벽(0.39m)과 같게
TL_LAMPS = (('red', 0.38), ('yellow', 0.30), ('green', 0.22))  # 위부터 빨·노·초
TL_LAMP_R = 0.028
TL_HEAD = (0.14, 0.14, 0.17, 0.43)         # 헤드박스 sx, sy, z0, z1
TL_STATE_KO = {'red': '빨강', 'yellow': '노랑', 'green': '초록', 'off': '꺼짐'}


def _tl_dim(rgb):
    return tuple(int(c * 0.25) for c in rgb)  # 꺼진 램프


def traffic_light(x, y, tid, rng=random):
    """신호등 — 폴 + 헤드박스 + 3개 램프(빨·노·초, 평소 꺼짐). cycle_idx·timer로 순환"""
    return {'kind': 'traffic_light', 'shape': 'box', 'color': TL_COLORS['off'],
            'x': round(x, 3), 'y': round(y, 3), 'yaw': 0.0, 'id': tid,
            'sx': 0.14, 'sy': 0.14, 'size': [0.14, 0.3],
            'state': 'off', 'cycle_idx': 5, 'timer': round(rng.uniform(0, 1.0), 2)}


def maze_cells(course):
    """spawn이 있는 절반의 칸 중심 (미로는 y=0 벽으로 위·아래가 막혀 있음). 칸 = 안쪽 기둥 격자 사이"""
    xs = sorted({round(x, 3) for x, _ in course['posts']})
    inner = [(round(x, 3), round(y, 3)) for x, y in course['posts'] if xs[0] < x < xs[-1]]
    count = {}
    for _, y in inner:
        count[y] = count.get(y, 0) + 1
    # 행 = 안쪽 기둥이 모든 열에 있는 y (세로 벽에만 있는 반 칸 기둥은 뺀다)
    rows = sorted(y for y, n in count.items() if n == max(count.values()))
    side = 1 if course['spawn'][1] > 0 else -1
    return [((x0 + x1) / 2, (y0 + y1) / 2) for x0, x1 in zip(xs, xs[1:]) for y0, y1 in zip(rows, rows[1:])
            if (y0 + y1) / 2 * side > 0]


GOAL_SIZE = 0.30  # 목표 구역 한 변 [m] — 바닥 표시판


def goal_zone(course, cells):
    """목표 구역 (고정) — spawn에서 가장 먼 칸 중심, GOAL_SIZE 정사각 바닥판.
    Renderer._puck 박스 경로로 그려지고, YOLO class 1 · MapView에도 표시된다."""
    gx, gy = max(cells, key=lambda c: math.dist(c, course['spawn']))
    return {'kind': 'goal_zone', 'shape': 'box', 'color': (200, 0, 200),
            'x': round(gx, 3), 'y': round(gy, 3), 'yaw': 0.0,
            'sx': GOAL_SIZE, 'sy': GOAL_SIZE, 'size': [GOAL_SIZE, 0.005]}


# 미션 (documents/MISSION.md §1~3·8) — 한 맵 안에 미션 지역 여러 곳: 지도에서 자리를 찍고 문제를 고른다.
# 채점은 외부(대회 심판)에서 하므로 시뮬레이터는 판(미션 지역 퍽 · QR 판 · B 지역 · 환경)만 깐다. 규정이 나오면 여기만 고친다.
ENV_LAYOUT = {'obstacles': 3, 'movers': 1, 'traffic': 0, 'slip_zones': 2, 'slip': 0.3, 'light_change': True}
QR_DIR = Path(__file__).resolve().parent / 'qr'


def load_problems(qr_dir=QR_DIR):
    """simulator/qr/<코드>/qr.png + info.json → 문제 목록. 폴더 이름 = 문제 코드(지도 표시), qr.png = 판에 입힐 QR 그림,
    info.json = {"text": QR에 담긴 텍스트, "name": 이름, "pucks": 퍽 수, "colors": 색 수, "lying": [최소, 최대]}
    문제 추가 = 폴더 하나 추가 (대회 QR 그림으로 바꿔 끼워도 됨). qr.png를 해독한 값이 text와 다르면 경고.
    → ({text: (이름, 코드, 퍽 수, 색 수, (누운 최소, 최대))}, {text: qr.png 경로})"""
    import json
    problems, images = {}, {}
    for info in sorted(Path(qr_dir).glob('*/info.json')):
        d = json.loads(info.read_text())
        png = info.parent / 'qr.png'
        img = cv2.imread(str(png))
        if img is None:
            print(f'경고: {png} 없음 — 문제 {info.parent.name} 건너뜀')
            continue
        decoded = cv2.QRCodeDetector().detectAndDecode(img)[0]
        if decoded != d['text']:
            print(f'경고: {info.parent.name}/qr.png 해독값 {decoded!r} ≠ info.json text {d["text"]!r}')
        problems[d['text']] = (d['name'], info.parent.name, int(d['pucks']), int(d['colors']), tuple(d['lying']))
        images[d['text']] = png
    return problems, images


PROBLEMS, QR_IMAGES = load_problems()  # QR 텍스트 → (이름, 코드, 퍽 수, 색 수, 누운 퍽 수 범위) · qr.png
STATION_SIZE = 0.45     # 미션 지역 한 변 [m]
START_SIZE = 0.45       # 출발 칸(= 마지막 복귀 자리) 표시 한 변 [m] — 로봇(지름 약 0.36m)이 들어가 멈출 크기
QR_BOARD = {'side': 0.30, 'bottom': 0.10, 'thick': 0.01}   # QR 판 [m] — 아래 끝 바닥 위 0.10
START_COMMAND = '모든 미션 수행 후 출발 칸으로 복귀'   # 시작 신호 (/cmd/user_command)
PUCK_COLORS = ('red', 'green', 'blue', 'yellow')


def station_zone(x, y, name):
    """미션 지역 바닥판 (주황) — 칸 중심 (x, y)"""
    return {'kind': 'station_zone', 'shape': 'box', 'color': (0, 150, 255), 'name': name,
            'x': round(x, 3), 'y': round(y, 3), 'yaw': 0.0,
            'sx': STATION_SIZE, 'sy': STATION_SIZE, 'size': [STATION_SIZE, 0.005]}


def qr_board(zone, normal, text):
    """QR 판 — 미션 지역 가장자리에 서서 normal 방향(지역 안쪽 · 로봇이 다가오는 쪽)을 봄.
    렌더는 흰 상자, QR 무늬(simulator/qr/<코드>/qr.png)는 overlay_qr가 카메라 영상에 입힌다."""
    nx, ny = normal
    off = zone['sx'] / 2 - QR_BOARD['thick']
    return {'kind': 'qr_board', 'shape': 'box', 'color': (245, 245, 245), 'text': text, 'station': zone['name'],
            'x': round(zone['x'] - nx * off, 3), 'y': round(zone['y'] - ny * off, 3), 'yaw': math.atan2(ny, nx),
            'normal': (nx, ny), 'z': QR_BOARD['bottom'],
            'sx': QR_BOARD['thick'], 'sy': QR_BOARD['side'], 'size': [QR_BOARD['side'], QR_BOARD['side']]}


def station_pucks(problem, zone, board, rng, first_id=1):
    """미션 지역 퍽 — 문제에 맞는 수·색·누운 퍽. 격자 칸(0.09m)에 흩뿌림, QR 판 앞 0.08m는 비움. id는 first_id부터"""
    _, _, n, n_colors, (lo, hi) = PROBLEMS[problem]
    colors = rng.sample(PUCK_COLORS, n_colors)
    nx, ny = board['normal']
    slots = []
    for i in range(-2, 3):
        for j in range(-2, 3):
            x, y = zone['x'] + i * 0.09, zone['y'] + j * 0.09
            if (x - board['x']) * nx + (y - board['y']) * ny > 0.08:  # 판 앞 여유
                slots.append((x, y))
    picked = rng.sample(slots, min(n, len(slots)))
    lying = set(rng.sample(range(len(picked)), rng.randint(lo, hi)))
    pucks = []
    for k, (x, y) in enumerate(picked):
        p = {'id': first_id + k, 'station': zone['name'], 'shape': 'cylinder', 'color': colors[k % n_colors] if problem == 'MISSION:SORT_COLOR' else rng.choice(colors),
             'x': round(x + rng.uniform(-0.01, 0.01), 3), 'y': round(y + rng.uniform(-0.01, 0.01), 3),
             'yaw': round(rng.uniform(-math.pi, math.pi), 3),
             'size': [round(rng.uniform(0.022, 0.028), 3), round(rng.uniform(0.03, 0.04), 3)]}
        if k in lying:
            p['lying'] = True  # 옆면이 바닥 — 축이 yaw 방향 수평
        pucks.append(p)
    rng.shuffle(pucks)  # 정렬 문제: 같은 색이 붙어 있지 않게 (id는 그대로)
    return pucks


def puck_height(p):
    """바닥에서 물체 윗면까지 높이 [m] — 구 · 누운 원기둥은 지름"""
    if p['shape'] == 'sphere' or p.get('lying'):
        return p['size'][0] * 2
    return p['size'][1]


def overlay_qr(bgr, depth, T_world_cam, board, _cache={}):
    """QR 판 앞면에 QR 무늬를 원근 투영으로 입힌다 (출력 해상도에서 — 내부 160×120 렌더로는 모듈이 뭉개짐).
    렌더된 깊이가 판 깊이와 맞는 화소만 덮어 앞을 가린 물체는 그대로 둔다."""
    text = board['text']
    if text not in _cache:  # simulator/qr/<코드>/qr.png 그대로 (여백 포함) — 회색조
        _cache[text] = cv2.imread(str(QR_IMAGES[text]), cv2.IMREAD_GRAYSCALE)
    qr = _cache[text]
    nx, ny = board['normal']
    ux, uy = -ny, nx  # 판 가로 방향 (앞에서 봤을 때 왼→오가 되게)
    h = board['sy'] / 2
    fx_, fy_ = board['x'] + nx * board['sx'] / 2, board['y'] + ny * board['sx'] / 2  # 앞면 중심
    z0, z1 = board['z'], board['z'] + board['size'][1]
    # QR 이미지 꼭짓점 (왼위 · 오위 · 오아래 · 왼아래) ↔ 월드
    world = np.array([[fx_ - ux * h, fy_ - uy * h, z1], [fx_ + ux * h, fy_ + uy * h, z1],
                      [fx_ + ux * h, fy_ + uy * h, z0], [fx_ - ux * h, fy_ - uy * h, z0]])
    Tcw = np.linalg.inv(T_world_cam)
    cam = (Tcw[:3, :3] @ world.T).T + Tcw[:3, 3]
    if (cam[:, 2] < 0.15).any():
        return bgr
    H, W = bgr.shape[:2]
    f = W / 2 / math.tan(math.radians(HFOV_DEG) / 2)
    px = np.stack([cam[:, 0] / cam[:, 2] * f + W / 2, cam[:, 1] / cam[:, 2] * f + H / 2], 1).astype(np.float32)
    if px[:, 0].max() < 0 or px[:, 0].min() > W or px[:, 1].max() < 0 or px[:, 1].min() > H:
        return bgr
    # 판을 앞에서 보는지 (뒷면이면 무늬 없음)
    o = T_world_cam[:3, 3]
    if (o[0] - fx_) * nx + (o[1] - fy_) * ny <= 0:
        return bgr
    qh, qw = qr.shape
    M = cv2.getPerspectiveTransform(np.float32([[0, 0], [qw, 0], [qw, qh], [0, qh]]), px)
    warped = cv2.warpPerspective(qr, M, (W, H), flags=cv2.INTER_AREA, borderValue=0)
    mask = cv2.warpPerspective(np.full_like(qr, 255), M, (W, H), flags=cv2.INTER_NEAREST, borderValue=0) > 0
    dmm = depth.astype(np.float32)
    mask &= (dmm >= cam[:, 2].min() * 1000 - 30) & (dmm <= cam[:, 2].max() * 1000 + 30)
    out = bgr.copy()
    out[mask] = warped[mask][:, None]
    return out


def in_zone(zone, x, y):
    """(x, y)가 zone(x, y, yaw, sx, sy 사각형) 안인가"""
    c, s = math.cos(-zone.get('yaw', 0.0)), math.sin(-zone.get('yaw', 0.0))
    lx, ly = (x - zone['x']) * c - (y - zone['y']) * s, (x - zone['x']) * s + (y - zone['y']) * c
    return abs(lx) <= zone['sx'] / 2 and abs(ly) <= zone['sy'] / 2


def stl_bbox(path, scale=(1.0, 1.0, 1.0)):
    """바이너리 STL → (중심, 크기) — 메시 충돌체를 상자로 근사할 때"""
    data = np.fromfile(path, dtype=np.uint8)
    n = int(np.frombuffer(data[80:84].tobytes(), np.uint32)[0])
    rec = np.frombuffer(data[84:84 + n * 50].tobytes(), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    v = rec['v'].reshape(-1, 3) * np.asarray(scale, np.float32)
    lo, hi = v.min(0), v.max(0)
    return (lo + hi) / 2, hi - lo


COLLISION_BGR = (0, 140, 255)  # 충돌 모양 보기 — robot_view와 같은 색


def _origin(el):
    o = el.find('origin')
    xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
    rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
    return transform(rpy_matrix(*rpy), xyz)


class BodyModel:
    """URDF → 로봇 자기 몸 카메라 렌더용 (퍽 싣는 뒤쪽 상판 · 앞으로 뻗은 팔까지 카메라에 보여야 해서).
    collision=False: <visual>, True: <collision> (색 COLLISION_BGR).
      boxes  [(link, T_origin, (sx, sy, sz), BGR)] — box · cylinder · sphere (맞춤 상자로 근사)
      meshes [(link, T_origin, 삼각형 (N,3,3) 링크 좌표, BGR)] — <mesh>는 load_mesh(filename, scale)로 실제 메시
    load_mesh가 없거나 파일을 못 찾으면: 메시 충돌체는 resolve로 STL 경계 상자, visual 메시는 그 링크의 원시 collision 도형"""

    def __init__(self, urdf_xml, collision=False, resolve=None, load_mesh=None, max_tris=192):
        """max_tris: 큰 메시를 이 삼각형 수 이하 조각으로 — CPU는 작게(경계 상자로 많이 거름), GPU는 안 나눔(호출 수가 더 비쌈)"""
        root = ET.fromstring(urdf_xml)
        colors = {m.get('name'): m.find('color').get('rgba') for m in root.findall('material')
                  if m.find('color') is not None}
        self.boxes, self.meshes = [], []
        for link in root.findall('link'):
            name = link.get('name')
            for el in link.findall('collision' if collision else 'visual'):
                geom = el.find('geometry')
                if geom is None or len(geom) == 0:
                    continue
                g = geom[0]
                T = _origin(el)
                if collision:
                    rgb = COLLISION_BGR
                else:
                    mat = el.find('material')
                    rgba = None
                    if mat is not None:
                        c = mat.find('color')
                        rgba = c.get('rgba') if c is not None else colors.get(mat.get('name'))
                    r_, gg, b = ([float(v) for v in rgba.split()[:3]] if rgba else [0.6, 0.6, 0.6])
                    rgb = (int(b * 255), int(gg * 255), int(r_ * 255))
                if g.tag == 'mesh':
                    scale = [float(v) for v in (g.get('scale', '1').split() * 3)[:3]]
                    tris = load_mesh(g.get('filename', ''), scale) if load_mesh else None
                    if tris is not None and len(tris):
                        for part in _split_mesh(np.asarray(tris, np.float32), max_tris):  # 조각마다 경계 상자로 광선을 거름
                            self.meshes.append((name, T, part, rgb))
                    elif collision and resolve is not None and resolve(g.get('filename', '')) is not None:
                        center, size = stl_bbox(resolve(g.get('filename', '')), scale)
                        self.boxes.append((name, T @ transform(np.eye(3), center), tuple(float(v) for v in size), rgb))
                    elif not collision:  # 메시를 못 찾음 — 그 링크의 원시 collision 도형으로 대신
                        for c in link.findall('collision'):
                            cg = c.find('geometry')
                            if cg is not None and len(cg) and cg[0].tag != 'mesh' and (box := _prim_size(cg[0])):
                                self.boxes.append((name, _origin(c), box, rgb))
                    continue
                size = _prim_size(g)
                if size:
                    self.boxes.append((name, T, size, rgb))


def _split_mesh(tris, max_tris=192):
    """큰 메시를 공간별 조각으로 (삼각형 중심을 가장 긴 축으로 반씩 나눔) — 조각 경계 상자가 작아져 광선 검사가 준다"""
    if len(tris) <= max_tris:
        return [tris]
    c = tris.mean(1)
    axis = int(np.argmax(c.max(0) - c.min(0)))
    order = np.argsort(c[:, axis])
    half = len(tris) // 2
    return _split_mesh(tris[order[:half]], max_tris) + _split_mesh(tris[order[half:]], max_tris)


def _prim_size(g):
    """URDF 원시 도형 → 맞춤 상자 크기 (sx, sy, sz) · 아니면 None"""
    if g.tag == 'box':
        return tuple(float(v) for v in g.get('size').split())
    if g.tag == 'cylinder':
        r, h = float(g.get('radius')), float(g.get('length'))
        return (2 * r, 2 * r, h)
    if g.tag == 'sphere':
        r = float(g.get('radius'))
        return (2 * r, 2 * r, 2 * r)
    return None


# ---------- 렌더 (torch — GPU 있으면 GPU)
def pick_device():
    """렌더 장치: 환경변수 SIM_DEVICE(cpu · cuda · mps) > CUDA > Apple GPU(MPS) > CPU"""
    import os

    import torch
    want = os.environ.get('SIM_DEVICE')
    if want:
        return torch.device(want)
    if torch.cuda.is_available():
        return torch.device('cuda')
    if getattr(torch.backends, 'mps', None) is not None and torch.backends.mps.is_available():
        return torch.device('mps')
    return torch.device('cpu')


class Renderer:
    """광선 추적 렌더러 — 수식은 numpy 시절과 같고 배열만 torch 텐서 (float32, MPS는 float64가 없다).
    GPU면 내부 해상도 = 출력 크기(320×240), CPU면 절반(SCALE)으로 쏘고 확대한다.
    WALL_CHUNK_BYTES: 벽 slab 판정 한 묶음의 메모리 상한 — 광선 수 × 벽 수 × 3이 커지면 나눠 계산"""
    WALL_CHUNK_BYTES = 96e6
    GPU_LOCK = threading.Lock()  # MPS는 여러 스레드가 동시에 쓰면 커널 캐시에서 멈춘다(실측) — GPU는 한 번에 한 카메라

    def __init__(self, walls, background=None):
        import torch
        self.t = torch
        self.dev = pick_device()
        self.scale = 1.0 if self.dev.type != 'cpu' else SCALE
        self.iw, self.ih = int(RENDER_W * self.scale), int(RENDER_H * self.scale)
        w = np.asarray(walls, float)
        self.wall_xy = w[:, :2]  # 근처 벽 고르기는 CPU(numpy)에서 — 몇백 개라 빠름
        self.wall_min = self._f(w[:, :3] - w[:, 3:] / 2)
        self.wall_max = self._f(w[:, :3] + w[:, 3:] / 2)
        self.wall_h = float((w[:, 2] + w[:, 5] / 2).max()) if len(w) else 0.39  # 벽 높이 (텍스처 세로 기준)
        self.light = self._f(LIGHT)
        self._body_cache = {}  # body_key → 몸 층 (거리, 바탕색, 카메라 기준 법선) — render() 설명
        self.wall_tex = self.floor_tex = None
        self.set_background(background)
        fx = self.iw / 2 / math.tan(math.radians(HFOV_DEG) / 2)
        u, v = np.meshgrid(np.arange(self.iw) + 0.5, np.arange(self.ih) + 0.5)
        self.rays_opt = self._f(np.stack([(u - self.iw / 2) / fx, (v - self.ih / 2) / fx, np.ones_like(u)], -1).reshape(-1, 3))

    def _f(self, a):
        return self.t.as_tensor(np.asarray(a, np.float32), device=self.dev)

    def _img(self, img):
        return None if img is None else self.t.as_tensor(np.ascontiguousarray(img), device=self.dev)

    def set_background(self, path):
        img = cv2.imread(str(path)) if path else None
        if img is None:  # 격자 방
            img = np.full((512, 1024, 3), 170, np.uint8)
            img[::32, :] = 120
            img[:, ::32] = 120
        self.pano = self._img(cv2.resize(img, (1024, 512)))

    def set_wall(self, path):
        """벽 텍스처 (가로 0.5m 반복 · 세로 벽 높이). 없으면 단색"""
        self.wall_tex = self._img(cv2.imread(str(path)) if path else None)
        self.wall_name = Path(path).name if path else None

    def set_floor(self, path):
        """바닥 텍스처 (1타일 = 1m 반복). 없으면 격자무늬"""
        self.floor_tex = self._img(cv2.imread(str(path)) if path else None)
        self.floor_name = Path(path).name if path else None

    def render(self, T_world_cam, pucks, body=(), meshes=(), body_key=None):
        """meshes: [(T_world 4x4, 삼각형 (N,3,3) 링크 좌표 numpy, BGR)] — 로봇 몸 메시 (BodyModel.meshes에 관절값 반영)
        body_key: 카메라에 보이는 자기 몸이 같은 동안 같은 값 (카메라·관절값) — 주면 몸 광선 추적 결과를 재사용.
          카메라가 로봇에 달려 있어 몸은 카메라 기준으로 관절값에만 달렸다. pole_cam이 몸을 내려다보면 메시 광선 추적이
          프레임당 ~1초(MPS 실측)라 매번 하면 카메라가 ~2Hz로 떨어지고 collision_monitor가 점구름 끊김으로 멈췄다"""
        if self.dev.type == 'cpu':  # CPU는 카메라별 스레드 병렬 그대로
            return self._render(T_world_cam, pucks, body, meshes, body_key)
        with self.GPU_LOCK:
            return self._render(T_world_cam, pucks, body, meshes, body_key)

    def _mesh_tensor(self, tris, _cache={}):
        key = (id(tris), self.dev.type)
        if key not in _cache:
            _cache[key] = self._f(tris.reshape(-1, 3, 3))
        return _cache[key]

    def _meshes(self, o, D, meshes, st):
        """메시 광선 추적 — 메시마다 경계 상자를 지나는 광선만 골라 Möller–Trumbore (삼각형은 묶음으로)
        st: 몸 층 {'best' 거리, 'rgb' 바탕색, 'nrm' 맞은 면 법선(월드)} — 음영은 _render가 합칠 때"""
        t_ = self.t
        inf = float('inf')
        inv = 1.0 / D
        for T, tris, rgb in meshes:
            Tt = self._f(T)
            w = self._mesh_tensor(tris) @ Tt[:3, :3].T + Tt[:3, 3]           # (M,3,3) 월드 좌표
            lo, hi = w.reshape(-1, 3).amin(0), w.reshape(-1, 3).amax(0)
            t1, t2 = (lo - o) * inv, (hi - o) * inv
            tn = t_.nan_to_num(t_.minimum(t1, t2), nan=-inf).amax(1)
            tf = t_.nan_to_num(t_.maximum(t1, t2), nan=inf).amin(1)
            rays = t_.nonzero((tf >= tn) & (tf > 1e-4) & (tn < st['best'])).squeeze(1)
            if rays.numel() == 0:
                continue
            d = D[rays]
            v0, e1, e2 = w[:, 0], w[:, 1] - w[:, 0], w[:, 2] - w[:, 0]
            nrm = t_.linalg.cross(e1, e2)
            nrm = nrm / nrm.norm(dim=1, keepdim=True).clamp(min=1e-12)
            best_t = t_.full((rays.numel(),), inf, device=self.dev)
            best_k = t_.zeros((rays.numel(),), dtype=t_.long, device=self.dev)
            chunk = max(1, int(24e6 // max(1, rays.numel() * 3)))
            for s in range(0, w.shape[0], chunk):
                a1, a2, a0 = e1[s:s + chunk], e2[s:s + chunk], v0[s:s + chunk]
                p = t_.linalg.cross(d[:, None, :].expand(-1, a2.shape[0], -1), a2[None].expand(d.shape[0], -1, -1))
                det = (a1[None] * p).sum(-1)
                idet = 1.0 / det
                tv = o - a0                                                     # (m,3)
                u = (tv[None] * p).sum(-1) * idet
                qv = t_.linalg.cross(tv, a1)                                     # (m,3)
                v = (d @ qv.T) * idet
                tt = (a2 * qv).sum(-1)[None] * idet
                ok = (det.abs() > 1e-12) & (u >= 0) & (v >= 0) & (u + v <= 1) & (tt > 1e-4)
                tt = t_.where(ok, tt, inf)
                tmin, k = tt.min(1)
                better = tmin < best_t
                best_t = t_.where(better, tmin, best_t)
                best_k = t_.where(better, k + s, best_k)
            full = t_.full_like(st['best'], inf)
            full[rays] = best_t
            nf = t_.zeros_like(st['nrm'])
            nf[rays] = nrm[best_k]
            hit = full < st['best']
            st['best'] = t_.where(hit, full, st['best'])
            st['rgb'] = t_.where(hit[:, None], self._f(rgb)[None], st['rgb'])
            st['nrm'] = t_.where(hit[:, None], nf, st['nrm'])

    def _render(self, T_world_cam, pucks, body=(), meshes=(), body_key=None):
        """→ (BGR uint8 H×W×3, depth uint16 mm H×W)  (numpy — 출력 크기 RENDER_W×RENDER_H)
        body: [(T_world 4x4, sx, sy, sz, BGR)] 로봇 자기 몸 (관절값 반영된 월드 자세) — 임의 회전 상자"""
        t_ = self.t
        inf = float('inf')
        Rn = np.asarray(T_world_cam[:3, :3], np.float32)
        on = np.asarray(T_world_cam[:3, 3], np.float32)
        R, o = self._f(Rn), self._f(on)
        D = self.rays_opt @ R.T  # 광선 방향 (optical z = 1 → t가 곧 깊이)
        n = D.shape[0]
        st = {'best': t_.full((n,), inf, device=self.dev), 'color': t_.zeros((n, 3), device=self.dev)}

        def take(t, rgb, shade):  # t·shade: 광선마다
            hit = t < st['best']
            st['best'] = t_.where(hit, t, st['best'])
            col = self._f(rgb)[None] * shade[:, None]
            st['color'] = t_.where(hit[:, None], col, st['color'])

        # 바닥 z=0 — 텍스처 있으면 1m 타일 샘플, 없으면 0.25m 체크무늬
        t = t_.where(D[:, 2] < -1e-6, -o[2] / D[:, 2], inf)
        p = o + D * t_.where(t_.isfinite(t), t, 0)[:, None]
        if self.floor_tex is not None:
            fh, fw = self.floor_tex.shape[:2]
            fu = (t_.remainder(p[:, 0], 1.0) * fw).long().clamp(0, fw - 1)
            fv = (t_.remainder(p[:, 1], 1.0) * fh).long().clamp(0, fh - 1)
            fcol = self.floor_tex[fv, fu].float()
        else:
            checker = t_.remainder(t_.floor(p[:, 0] / 0.25) + t_.floor(p[:, 1] / 0.25), 2)[:, None]
            fcol = (95 + 20 * checker).expand(n, 3)
        hit = t < st['best']
        st['best'] = t_.where(hit, t, st['best'])
        st['color'] = t_.where(hit[:, None], fcol, st['color'])

        # 벽 (축 정렬 상자) — 카메라 앞쪽 VIEW_RANGE 안만, 묶음으로 slab 판정
        rel = self.wall_xy - on[:2]
        fwd = Rn[:2, 2] / max(np.linalg.norm(Rn[:2, 2]), 1e-6)
        idx = np.nonzero((np.linalg.norm(rel, axis=1) < VIEW_RANGE) & (rel @ fwd > -0.6))[0]
        if len(idx):
            it = t_.as_tensor(idx, device=self.dev)
            lo_all, hi_all = self.wall_min[it], self.wall_max[it]
            inv = 1.0 / D
            chunk = max(1, int(self.WALL_CHUNK_BYTES // (n * 3 * 4 * 4)))
            ar = t_.arange(n, device=self.dev)
            for s in range(0, len(idx), chunk):
                lo, hi = lo_all[s:s + chunk], hi_all[s:s + chunk]
                t1 = (lo[None] - o) * inv[:, None]
                t2 = (hi[None] - o) * inv[:, None]
                mn = t_.nan_to_num(t_.minimum(t1, t2), nan=-inf)
                tmin, face = mn.max(2)
                tmax = t_.nan_to_num(t_.maximum(t1, t2), nan=inf).amin(2)
                tmin = t_.where((tmax >= tmin) & (tmin > 1e-4), tmin, inf)
                tk, k = tmin.min(1)
                ax = face[ar, k]
                shade = 0.55 + 0.45 * self.light[ax].abs()  # 축 정렬 면 법선 · 빛 = LIGHT[축]
                if self.wall_tex is not None:
                    ph = o + D * t_.where(t_.isfinite(tk), tk, 0)[:, None]
                    uu = t_.where(ax == 0, ph[:, 1] / WALL_TILE_M, ph[:, 0] / WALL_TILE_M)
                    vv = t_.where(ax == 2, ph[:, 1] / WALL_TILE_M, ph[:, 2] / self.wall_h)
                    th, tw = self.wall_tex.shape[:2]
                    ui = (t_.remainder(uu, 1.0) * tw).long().clamp(0, tw - 1)
                    vi = ((1.0 - t_.remainder(vv, 1.0)) * th).long().clamp(0, th - 1)
                    wcol = self.wall_tex[vi, ui].float() * shade[:, None]
                else:
                    wcol = self._f(WALL_BGR)[None] * shade[:, None]
                hitm = tk < st['best']
                st['best'] = t_.where(hitm, tk, st['best'])
                st['color'] = t_.where(hitm[:, None], wcol, st['color'])

        for puck in pucks:
            if puck.get('kind') == 'traffic_light':
                self._traffic_light(puck, o, D, take)
            else:
                self._puck(puck, o, D, take)

        # 자기 몸 층 (거리 · 바탕색 · 카메라 기준 법선) — body_key가 같으면 지난번 것 재사용, 음영만 지금 자세로
        cached = self._body_cache.get(body_key) if body_key is not None else None
        if cached is None:
            lay = self._body_layer(o, D, Rn, on, body, meshes)
            cached = (lay['best'], lay['rgb'], lay['nrm'] @ R)  # 법선 월드 → 카메라 기준
            if body_key is not None:
                self._body_cache = {k: v for k, v in self._body_cache.items() if k[0] != body_key[0]}  # 카메라당 하나
                self._body_cache[body_key] = cached
        bt, brgb, bn = cached
        hitb = bt < st['best']
        if bool(hitb.any()):
            shade = 0.45 + 0.55 * ((bn @ R.T) @ self.light).abs()
            st['best'] = t_.where(hitb, bt, st['best'])
            st['color'] = t_.where(hitb[:, None], brgb * shade[:, None], st['color'])

        best = st['best']
        depth = t_.where((best >= MIN_DEPTH) & (best <= MAX_DEPTH), best * 1000, 0)
        color = st['color']
        # 배경: 아무것도 안 맞은 광선 → 파노라마 (가운데 = world +x, 위 = +z)
        miss = ~t_.isfinite(best)
        d = D / D.norm(dim=1, keepdim=True)
        yaw = t_.atan2(d[:, 1], d[:, 0])
        pitch = t_.asin(d[:, 2].clamp(-1, 1))
        ph_, pw_ = self.pano.shape[:2]
        pu = t_.remainder(((0.5 - yaw / (2 * math.pi)) * pw_).long(), pw_)
        pv = ((0.5 - pitch / math.pi) * ph_).long().clamp(0, ph_ - 1)
        color = t_.where(miss[:, None], self.pano[pv, pu].float(), color)
        bgr = color.clamp(0, 255).to(t_.uint8).reshape(self.ih, self.iw, 3).cpu().numpy()
        depth = depth.reshape(self.ih, self.iw).cpu().numpy().astype(np.uint16)
        if self.scale != 1.0:  # 출력 크기로 확대 (컬러 선형 · 깊이 최근접 — 값 보존)
            bgr = cv2.resize(bgr, (RENDER_W, RENDER_H), interpolation=cv2.INTER_LINEAR)
            depth = cv2.resize(depth, (RENDER_W, RENDER_H), interpolation=cv2.INTER_NEAREST)
        return bgr, depth

    def _body_layer(self, o, D, Rn, on, body, meshes):
        """자기 몸만 광선 추적 → {'best' 거리 (안 맞으면 inf), 'rgb' 바탕색, 'nrm' 맞은 면 법선 (월드)} — 음영 전"""
        t_ = self.t
        inf = float('inf')
        n = D.shape[0]
        st = {'best': t_.full((n,), inf, device=self.dev), 'rgb': t_.zeros((n, 3), device=self.dev),
              'nrm': t_.zeros((n, 3), device=self.dev)}
        # 회전 상자 여러 개를 한 번에 slab 판정 (상자마다 따로 하면 작은 연산이 수십 번)
        body = [b for b in body if (np.asarray(b[0][:3, 3], np.float32) - on) @ Rn[:, 2] >= -0.2]  # 렌즈 뒤 제외
        if body:
            Rb = self._f(np.stack([np.asarray(T[:3, :3], np.float32) for T, *_ in body]))      # (k,3,3) 열 = 상자 축
            cb = self._f(np.stack([np.asarray(T[:3, 3], np.float32) for T, *_ in body]))       # (k,3)
            half = self._f([[sx / 2, sy / 2, sz / 2] for _, sx, sy, sz, _ in body])           # (k,3)
            rgbs = self._f([rgb for *_, rgb in body])                                          # (k,3)
            ol = t_.einsum('kj,kjm->km', o[None] - cb, Rb)                                     # 상자 좌표 원점
            Dl = t_.einsum('nj,kjm->nkm', D, Rb)                                               # 상자 좌표 방향
            t1, t2 = (-half[None] - ol[None]) / Dl, (half[None] - ol[None]) / Dl
            tmin, face = t_.nan_to_num(t_.minimum(t1, t2), nan=-inf).max(2)
            tmax = t_.nan_to_num(t_.maximum(t1, t2), nan=inf).amin(2)
            tmin = t_.where((tmax >= tmin) & (tmin > 1e-4), tmin, inf)
            tk, k = tmin.min(1)
            ar = t_.arange(n, device=self.dev)
            n_w = Rb[k, :, face[ar, k]]                                                         # 맞은 면 법선 (월드)
            st['best'], st['rgb'], st['nrm'] = tk, rgbs[k], n_w

        if meshes:
            self._meshes(o, D, meshes, st)
        return st

    def _box_local(self, oc, D, Rb, size, rgb, take):
        """회전 상자 slab 판정 — oc = 광선 원점 − 상자 중심 (월드), Rb = 상자 회전 (열 = 상자 축)"""
        t_ = self.t
        inf = float('inf')
        ol = oc @ Rb
        Dl = D @ Rb
        sx, sy, sz = size
        lo = self._f([-sx / 2, -sy / 2, -sz / 2])
        hi = self._f([sx / 2, sy / 2, sz / 2])
        t1, t2 = (lo - ol) / Dl, (hi - ol) / Dl
        tmin, face = t_.nan_to_num(t_.minimum(t1, t2), nan=-inf).max(1)
        tmax = t_.nan_to_num(t_.maximum(t1, t2), nan=inf).amin(1)
        t = t_.where((tmax >= tmin) & (tmin > 1e-4), tmin, inf)
        n_w = Rb.T[face]  # 맞은 면 법선 (월드) = 상자 축
        take(t, rgb, 0.45 + 0.55 * (n_w @ self.light).abs())

    def _cyl_side(self, ox, oy, dx, dy, r):
        """수직(지역 z) 원기둥 옆면 교차 t (z 범위 검사 전)"""
        t_ = self.t
        a = dx * dx + dy * dy
        b = 2 * (ox * dx + oy * dy)
        disc = b * b - 4 * a * (ox * ox + oy * oy - r * r)
        return t_.where(disc >= 0, (-b - t_.sqrt(disc.clamp(min=0))) / (2 * a), float('inf'))

    def _sphere(self, center, r, o, D, take, rgb, emissive=False, shade_lo=0.45):
        t_ = self.t
        oc = o - center
        a = (D * D).sum(1)
        b = 2 * D @ oc
        disc = b * b - 4 * a * (oc @ oc - r * r)
        t = t_.where(disc >= 0, (-b - t_.sqrt(disc.clamp(min=0))) / (2 * a), float('inf'))
        t = t_.where(t > 1e-4, t, float('inf'))
        if emissive:
            take(t, rgb, t_.ones_like(t))  # 켜진 램프는 발광이라 음영 없이 밝게
        else:
            nrm = (o + D * t_.where(t_.isfinite(t), t, 0)[:, None] - center) / r
            take(t, rgb, shade_lo + (1 - shade_lo) * (nrm @ self.light).clamp(0, 1))

    def _puck(self, puck, o, D, take):
        """퍽·장애물·이동물체 공용: shape cylinder(size=[반지름, 높이]) · sphere(size=[반지름]) ·
        box(size=[한 변, 높이], 직사각형이면 sx·sy). color = PALETTE 이름 또는 (B, G, R).
        z = 바닥에서 띄운 높이 (든 퍽) · lying = 누운 원기둥"""
        t_ = self.t
        inf = float('inf')
        rgb = PALETTE.get(puck['color'], (128, 128, 128)) if isinstance(puck['color'], str) else puck['color']
        cx, cy = float(puck['x']), float(puck['y'])
        zo = float(puck.get('z', 0.0))
        r_or_side, height = puck['size']
        if puck['shape'] == 'sphere':
            self._sphere(self._f([cx, cy, r_or_side + zo]), r_or_side, o, D, take, rgb)
        elif puck['shape'] == 'cylinder' and puck.get('lying'):
            # 누운 원기둥: 축이 수평(yaw 방향), 중심 높이 = 반지름. 지역 좌표(축 = z)로 바꿔 판정
            r, half = r_or_side, height / 2
            cyaw, syaw = math.cos(puck['yaw']), math.sin(puck['yaw'])
            Rl = self._f([[-syaw, cyaw, 0], [0, 0, 1], [cyaw, syaw, 0]])  # 월드 → 지역 (행 = 지역 축)
            ol = Rl @ (o - self._f([cx, cy, zo + r]))
            Dl = D @ Rl.T
            t_side = self._cyl_side(ol[0], ol[1], Dl[:, 0], Dl[:, 1], r)
            zl = ol[2] + Dl[:, 2] * t_side
            t_side = t_.where((t_side > 1e-4) & (zl.abs() <= half), t_side, inf)
            ts = t_.where(t_.isfinite(t_side), t_side, 0)[:, None]
            n_l = t_.cat([(ol[:2] + Dl[:, :2] * ts) / r, t_.zeros_like(ts)], 1)
            take(t_side, rgb, 0.45 + 0.55 * ((n_l @ Rl) @ self.light).clamp(0, 1))
            cap_shade = 0.45 + 0.55 * abs(float(np.array([cyaw, syaw, 0]) @ LIGHT))
            for zc in (-half, half):  # 양 끝 원판
                tc = (zc - ol[2]) / Dl[:, 2]
                pc = ol[:2] + Dl[:, :2] * t_.where(t_.isfinite(tc), tc, 0)[:, None]
                tc = t_.where((tc > 1e-4) & ((pc ** 2).sum(1) <= r * r), tc, inf)
                take(tc, rgb, t_.full_like(tc, cap_shade))
        elif puck['shape'] == 'cylinder':
            r = r_or_side
            dx, dy = D[:, 0], D[:, 1]
            ox, oy = o[0] - cx, o[1] - cy
            t_side = self._cyl_side(ox, oy, dx, dy, r)
            z = o[2] + D[:, 2] * t_side
            t_side = t_.where((t_side > 1e-4) & (z >= zo) & (z <= zo + height), t_side, inf)
            t_top = t_.where(D[:, 2] < 0, (zo + height - o[2]) / D[:, 2], inf)
            ptop = o[:2] + D[:, :2] * t_.where(t_.isfinite(t_top), t_top, 0)[:, None]
            t_top = t_.where((t_top > 1e-4) & (((ptop - self._f([cx, cy])) ** 2).sum(1) <= r * r), t_top, inf)
            ts = t_.where(t_.isfinite(t_side), t_side, 0)
            side_n = t_.stack([ox + dx * ts, oy + dy * ts, t_.zeros_like(dx)], 1) / r
            take(t_side, rgb, 0.45 + 0.55 * (side_n @ self.light).clamp(0, 1))
            take(t_top, rgb, t_.full_like(t_top, 0.45 + 0.55 * float(LIGHT[2])))
        else:  # box — 퍽 좌표계로 돌려 slab 판정 (바닥 zo에서 위로 height)
            sx, sy = puck.get('sx', r_or_side), puck.get('sy', r_or_side)
            cyaw, syaw = math.cos(puck['yaw']), math.sin(puck['yaw'])
            Rb = self._f([[cyaw, -syaw, 0], [syaw, cyaw, 0], [0, 0, 1]])
            self._box_local(o - self._f([cx, cy, zo + height / 2]), D, Rb, (sx, sy, height), rgb, take)

    def _traffic_light(self, tl, o, D, take):
        """신호등: 회색 폴 + 어두운 헤드박스 + 3개 램프(켜진 것만 밝게, 앞뒤 양면)"""
        t_ = self.t
        inf = float('inf')
        cx, cy = tl['x'], tl['y']
        state = tl.get('state', 'off')
        r, h = 0.02, TL_POLE_H
        dx, dy = D[:, 0], D[:, 1]
        ox, oy = o[0] - cx, o[1] - cy
        t_side = self._cyl_side(ox, oy, dx, dy, r)
        z = o[2] + D[:, 2] * t_side
        t_side = t_.where((t_side > 1e-4) & (z >= 0) & (z <= h), t_side, inf)
        ts = t_.where(t_.isfinite(t_side), t_side, 0)
        side_n = t_.stack([ox + dx * ts, oy + dy * ts, t_.zeros_like(dx)], 1) / r
        take(t_side, (70, 70, 70), 0.45 + 0.55 * (side_n @ self.light).clamp(0, 1))
        hsx, hsy, hz0, hz1 = TL_HEAD
        lo = self._f([cx - hsx / 2, cy - hsy / 2, hz0])
        hi = self._f([cx + hsx / 2, cy + hsy / 2, hz1])
        t1, t2 = (lo - o) / D, (hi - o) / D
        tmin = t_.nan_to_num(t_.minimum(t1, t2), nan=-inf).amax(1)
        tmax = t_.nan_to_num(t_.maximum(t1, t2), nan=inf).amin(1)
        t = t_.where((tmax >= tmin) & (tmin > 1e-4), tmin, inf)
        take(t, (40, 40, 40), t_.full_like(t, 0.6))
        for name, lz in TL_LAMPS:
            base = np.asarray(TL_COLORS[name], float)
            rgb = tuple(int(v) for v in base) if state == name else _tl_dim(base)
            for side in (hsx / 2 - 0.01, -(hsx / 2 - 0.01)):
                self._sphere(self._f([cx + side, cy, lz]), TL_LAMP_R, o, D, take, rgb, emissive=state == name)


# RealSense D435 깊이 특성 (ponytail: 단순 모델 — 실측 잡음 곡선·IR 그림자는 실기 데이터 나오면 맞출 것)
DEPTH_NOISE_K = 0.005    # 깊이 잡음 표준편차 = K × z² [m] — 1m 5mm · 2m 2cm · 4m 8cm
DEPTH_EDGE_JUMP = 0.05   # 이웃 화소와 깊이가 이 비율 넘게 튀면 경계 — 실기처럼 측정 안 됨(0)
DEPTH_DROPOUT = 0.003    # 무작위 결측 비율


def realsense_depth(depth_mm, rng=np.random):
    """렌더된 완벽한 깊이(uint16 mm, 0 = 측정 안 됨) → 실기 RealSense처럼: 거리 제곱 잡음 + 경계 구멍 + 무작위 결측"""
    z = depth_mm.astype(np.float32) / 1000.0
    valid = depth_mm > 0
    out = z + rng.normal(0.0, 1.0, z.shape).astype(np.float32) * (DEPTH_NOISE_K * z * z)
    # 경계: 오른쪽·아래 이웃과의 상대 차이가 크면 두 화소 모두 구멍 (물체 가장자리에서 실기 깊이가 비는 현상)
    edge = np.zeros_like(valid)
    for a, b in ((z[:, 1:], z[:, :-1]), (z[1:, :], z[:-1, :])):
        jump = np.abs(a - b) > DEPTH_EDGE_JUMP * np.maximum(np.minimum(a, b), 1e-3)
        if a.shape[1] != z.shape[1]:
            edge[:, 1:] |= jump
            edge[:, :-1] |= jump
        else:
            edge[1:, :] |= jump
            edge[:-1, :] |= jump
    drop = rng.random(z.shape) < DEPTH_DROPOUT
    keep = valid & ~edge & ~drop
    return np.where(keep, np.clip(out * 1000.0, 1, 65535), 0).astype(np.uint16)


def depth_colormap(depth_mm):
    """16비트 깊이 → 컬러 (가까움 빨강 · 멂 파랑, 0 = 검정)"""
    d = depth_mm.astype(np.float32)
    norm = np.clip((d - MIN_DEPTH * 1000) / ((MAX_DEPTH - MIN_DEPTH) * 1000), 0, 1)
    img = cv2.applyColorMap((255 - norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[d == 0] = 0
    return img


def cloud_view(depth_mm, size, yaw, pitch, color=None, step=3):
    """깊이 영상 → 점구름을 (yaw, pitch)로 돌려 본 그림. 색은 영상(같은 크기의 BGR)이 있으면 그 색, 없으면 깊이"""
    h, w = depth_mm.shape
    fx = w / 2 / math.tan(math.radians(HFOV_DEG) / 2)
    v, u = np.mgrid[0:h:step, 0:w:step]
    z = depth_mm[::step, ::step].astype(np.float32) / 1000
    ok = z > 0
    pts = np.stack([(u - w / 2) * z / fx, (v - h / 2) * z / fx, z], -1)[ok]
    if len(pts) == 0:
        return points_view(pts, None, size, yaw, pitch)
    cols = (color[::step, ::step][ok] if color is not None else depth_colormap((z[ok] * 1000).astype(np.uint16)[None])[0])
    return points_view(pts, cols, size, yaw, pitch)


def points_view(pts, cols, size, yaw, pitch):
    """점(카메라 축: x 오른쪽 · y 아래 · z 앞)을 (yaw, pitch)로 돌려 본 그림. cols = 점마다 BGR"""
    out = np.zeros((size[1], size[0], 3), np.uint8)
    if len(pts) == 0:
        cv2.putText(out, 'no points', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        return out
    pts = pts - np.median(pts, axis=0)
    rot = rpy_matrix(pitch, 0, 0) @ rpy_matrix(0, yaw, 0)
    p = pts @ rot.T
    scale = 0.45 * min(size) / max(np.percentile(np.abs(p[:, :2]), 95), 1e-3)
    order = np.argsort(-p[:, 2])  # 먼 점부터 → 가까운 점이 덮음
    px = (p[order, 0] * scale + size[0] / 2).astype(int)
    py = (p[order, 1] * scale + size[1] / 2).astype(int)
    keep = (px >= 0) & (px < size[0] - 1) & (py >= 0) & (py < size[1] - 1)
    for dx, dy in ((0, 0), (1, 0), (0, 1), (1, 1)):  # 2×2 점
        out[py[keep] + dy, px[keep] + dx] = cols[order][keep]
    return out


def backgrounds(folder):
    return sorted(Path(folder).glob('*.jpg'))
