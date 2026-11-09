"""주사위 기억 미션 장면 (documents/mission/README.md) — course/dice_maze/walls.json의 'mission'으로 깐다

  퍽 12개 (시작 박스 위 바닥 Z=70, 윗면 AprilTag = 퍽 ID) · 장소 태그 12~14 (박스 수직면, 중심 높이 35mm)
  바코드 판 (Code 128, (80, 650, 450) mm, +X) · 신호등 (80, 950) — [신호 시작]부터 한 색 1초 켜짐 + 1초 소등 × 12
  문제 = dice/<이름>.json: {"signal": "RGRR…", "barcode": "P2-01", "pucks": [{"tag", "color", "shape", "box", "side"}]}
    shape: cylinder (Ø30) | cuboid (30×30) · box: 시작 박스 1~6 · side: -1 = X−40 · +1 = X+40
  태그·바코드 무늬는 렌더 뒤 영상에 원근 투영으로 입힌다 (scene.overlay_qr와 같은 방식 — 내부 렌더로는 칸이 뭉개짐)
"""
import json
import math
from pathlib import Path

import cv2
import numpy as np

import scene

HERE = Path(__file__).resolve().parent
PROBLEM_DIR = HERE / 'dice'
PUCK_TAG_M = 0.016       # 퍽 윗면 태그 검은 사각형 한 변 (발제 AprilTag 배포 표식)
LOC_TAG_M = 0.048        # 장소 태그
TAG_Z = 0.035            # 장소 태그 중심 높이
BOX_TOP = 0.070          # 박스 윗면 = 퍽 바닥
PUCK_H = 0.060
BARCODE = {'width': 0.40, 'height': 0.12, 'thick': 0.01}   # ponytail: 실물 크기 모름 — 시뮬 해상도(320×240)에서 읽히게 크게
SIZE = {'cylinder': [0.015, PUCK_H], 'cuboid': [0.030, PUCK_H]}
SHAPE = {'cylinder': 'cylinder', 'cuboid': 'box'}   # 문제 파일 이름 → 렌더러 모양
LAMP = {'R': 'red', 'Y': 'yellow', 'G': 'green'}
_TAGS = {}


def problems():
    return sorted(p.stem for p in PROBLEM_DIR.glob('*.json'))


def _course_xy(course, x_mm, y_mm):
    cx, cy = course['arena_center_mm']
    return round((x_mm - cx) / 1000, 4), round((y_mm - cy) / 1000, 4)


def setup(course, name):
    """문제 이름 → (pucks, props, faces, problem). faces = 장소 태그 면 (렌더 형상 없음 — 무늬만 입힘)"""
    prob = json.loads((PROBLEM_DIR / f'{name}.json').read_text())
    m = course['mission']
    boxes = {b['name']: b for b in m['boxes']}
    pucks = []
    for p in prob['pucks']:
        b = boxes[f"start_{p['box']}"]
        x, y = _course_xy(course, b['x'] + p['side'] * m['puck_offsets_mm'][1], b['y'])
        pucks.append({'id': p['tag'], 'tag': p['tag'], 'shape': SHAPE[p['shape']], 'color': p['color'],
                      'x': x, 'y': y, 'z': BOX_TOP, 'on_box': True, 'yaw': 0.0, 'size': list(SIZE[p['shape']])})
    sl = m['signal_light']
    lx, ly = _course_xy(course, sl['x'], sl['y'])
    light = {**scene.traffic_light(lx, ly, 'signal'), 'sequence': prob['signal'], 'seq_t': None, 'timer': 1e9,
             'state': 'off', 'color': scene.TL_COLORS['off']}
    bc = m['barcode']
    bx, by = _course_xy(course, bc['x'], bc['y'])
    board = {'kind': 'barcode_board', 'shape': 'box', 'color': (245, 245, 245), 'text': prob['barcode'],
             'x': round(bx + BARCODE['thick'] / 2, 4), 'y': by, 'yaw': 0.0, 'normal': (1.0, 0.0),
             'z': bc['z'] / 1000 - BARCODE['height'] / 2, 'sx': BARCODE['thick'], 'sy': BARCODE['width'],
             'size': [BARCODE['width'], BARCODE['height']], 'img': barcode_image(prob['barcode'])}
    faces = []
    for name_, loc in m['locations'].items():
        b = boxes.get(name_) or boxes['pedestal']  # L2는 중앙 받침대 북쪽 면
        nx, ny = loc['normal']
        half = b['size'] / 2
        fx = (b['x'] + nx * half) if nx else loc['x']
        fy = (b['y'] + ny * half) if ny else loc['y']
        x, y = _course_xy(course, fx, fy)
        faces.append({'tag': loc['tag'], 'x': x + nx * 0.001, 'y': y + ny * 0.001, 'z': TAG_Z, 'normal': (nx, ny)})
    return pucks, [light, board], faces, prob


def barcode_image(text):
    import zxingcpp  # requirements.txt — 바코드 판 무늬 (zxing-cpp가 Code 128을 만들고 인지도 같은 걸로 읽음)
    return np.array(zxingcpp.create_barcode(text, zxingcpp.BarcodeFormat.Code128).to_image(scale=4, add_quiet_zones=True))


def tag_image(tid):
    """AprilTag 36h11 + 흰 여백 (검은 사각형 = 가운데 80%)"""
    if tid not in _TAGS:
        dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        _TAGS[tid] = cv2.copyMakeBorder(cv2.aruco.generateImageMarker(dic, tid, 80), 10, 10, 10, 10,
                                        cv2.BORDER_CONSTANT, value=255)
    return _TAGS[tid]


def signal_step(light, dt):
    """[신호 시작] 뒤 시각으로 램프 상태 — 점멸 i는 [2i, 2i+1) 켜짐, 나머지 꺼짐 (12번째 뒤 계속 꺼짐 = 끝 신호)"""
    if light['seq_t'] is None:
        state = 'off'
    else:
        light['seq_t'] += dt
        i, frac = divmod(light['seq_t'], 2.0)
        seq = light['sequence']
        state = LAMP[seq[int(i)]] if i < len(seq) and frac < 1.0 else 'off'
    light['state'], light['color'] = state, scene.TL_COLORS[state]


def overlay_plane(bgr, depth, T_world_cam, corners, img, tol_m=0.02):
    """월드 사각형 corners (앞에서 본 왼위·오위·오아래·왼아래) 에 img(회색조)를 원근 투영으로 입힘.
    렌더 깊이가 그 면 깊이와 맞는 화소만 (앞을 가린 물체는 그대로) · 뒷면에서 보면 안 입힘"""
    corners = np.asarray(corners, float)
    n = np.cross(corners[3] - corners[0], corners[1] - corners[0])  # 보는 쪽 법선 (아래 × 오른쪽)
    if (T_world_cam[:3, 3] - corners[0]) @ n <= 0:
        return bgr
    Tcw = np.linalg.inv(T_world_cam)
    cam = (Tcw[:3, :3] @ corners.T).T + Tcw[:3, 3]
    if (cam[:, 2] < 0.05).any():
        return bgr
    H, W = bgr.shape[:2]
    f = W / 2 / math.tan(math.radians(scene.HFOV_DEG) / 2)
    px = np.stack([cam[:, 0] / cam[:, 2] * f + W / 2, cam[:, 1] / cam[:, 2] * f + H / 2], 1).astype(np.float32)
    if px[:, 0].max() < 0 or px[:, 0].min() > W or px[:, 1].max() < 0 or px[:, 1].min() > H:
        return bgr
    ih, iw = img.shape[:2]
    M = cv2.getPerspectiveTransform(np.float32([[0, 0], [iw, 0], [iw, ih], [0, ih]]), px)
    warped = cv2.warpPerspective(img, M, (W, H), flags=cv2.INTER_AREA, borderValue=0)
    mask = cv2.warpPerspective(np.full_like(img, 255), M, (W, H), flags=cv2.INTER_NEAREST, borderValue=0) > 0
    dmm = depth.astype(np.float32)
    mask &= (dmm >= (cam[:, 2].min() - tol_m) * 1000) & (dmm <= (cam[:, 2].max() + tol_m) * 1000)
    if not mask.any():
        return bgr
    out = bgr.copy()
    out[mask] = warped[mask][:, None]
    return out


def overlay(bgr, depth, T_world_cam, things, faces):
    """퍽 윗면 태그 · 장소 태그 · 바코드 무늬를 영상에 입힘"""
    for p in things:
        if p.get('tag') is not None and not p.get('lying'):  # 퍽 윗면 — 위에서 볼 때만 (overlay_plane 앞면 판정)
            h = PUCK_TAG_M / 0.8 / 2
            c, s = math.cos(p['yaw']), math.sin(p['yaw'])
            u, v = np.array([c, s, 0]) * h, np.array([-s, c, 0]) * h  # 윗면에서 오른쪽 · 위쪽
            o = np.array([p['x'], p['y'], p.get('z', 0.0) + scene.puck_height(p) + 0.0005])
            bgr = overlay_plane(bgr, depth, T_world_cam, [o - u + v, o + u + v, o + u - v, o - u - v], tag_image(p['tag']))
        elif p.get('kind') == 'barcode_board':
            nx, ny = p['normal']
            ux, uy = -ny, nx  # 판 가로 (앞에서 봐서 왼 → 오)
            fx, fy, h = p['x'] + nx * p['sx'] / 2 + nx * 0.001, p['y'] + ny * p['sx'] / 2 + ny * 0.001, p['sy'] / 2
            z0, z1 = p['z'], p['z'] + p['size'][1]
            bgr = overlay_plane(bgr, depth, T_world_cam, [[fx - ux * h, fy - uy * h, z1], [fx + ux * h, fy + uy * h, z1],
                                                          [fx + ux * h, fy + uy * h, z0], [fx - ux * h, fy - uy * h, z0]],
                                p['img'])
    for t in faces:
        nx, ny = t['normal']
        ux, uy = -ny, nx
        h = LOC_TAG_M / 0.8 / 2
        x, y, z = t['x'], t['y'], t['z']
        bgr = overlay_plane(bgr, depth, T_world_cam, [[x - ux * h, y - uy * h, z + h], [x + ux * h, y + uy * h, z + h],
                                                      [x + ux * h, y + uy * h, z - h], [x - ux * h, y - uy * h, z - h]],
                            tag_image(t['tag']))
    return bgr
