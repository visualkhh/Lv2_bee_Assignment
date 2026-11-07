"""대회 코스 — lv2_module6/course/twin_closed_maze/walls.json (기둥 + 벽판 상자, 세로 벽 포함)

코스 파일은 저장소에 있다 (course/README.md). 원본 메시에서 다시 만들려면 course/tools/build_walls.py.
  좌표: 미로 중심 원점, x 오른쪽 · y 위, m.  크기 4.464 × 6.364, 벽 높이 0.39
  spawn_xy_m: 로봇 시작 위치 — 시뮬레이터는 map/odom 원점을 여기로 본다
  y=0 벽이 미로를 위·아래 두 개로 완전히 나눈다 — spawn이 있는 절반에서만 오갈 수 있다
"""
import json
from pathlib import Path

import cv2
import numpy as np

COURSES = Path(__file__).resolve().parents[2] / 'course'
COURSE = COURSES / 'twin_closed_maze' / 'walls.json'


def load(path=COURSE):
    """→ {'size': (w, h), 'spawn': (x, y), 'wall_h': m, 'boxes': [(cx, cy, cz, sx, sy, sz)], 'posts': [(x, y)]}
    path = walls.json 경로 또는 코스 이름 (course/<이름>/walls.json). 상자마다 'sz'가 있으면 그 높이 (박스 70mm) ·
    'mission'이 있으면 그대로 (주사위 미션 물체 — dice.py), 'arena_center_mm' = 경기장 원점 → 미로 중심"""
    if not str(path).endswith('.json'):
        path = COURSES / str(path) / 'walls.json'
    walls = json.loads(Path(path).read_text())
    h = walls['wall_height_m']
    out = {'name': walls['name'], 'size': tuple(walls['size_m']), 'spawn': tuple(walls['spawn_xy_m']), 'wall_h': h,
           'boxes': [(b['x'], b['y'], b.get('sz', h) / 2, b['sx'], b['sy'], b.get('sz', h)) for b in walls['boxes']],
           'posts': [(b['x'], b['y']) for b in walls['boxes'] if b['name'].startswith('post_')]}
    for k in ('mission', 'arena_center_mm'):
        if k in walls:
            out[k] = walls[k]
    return out


def walls_mask(course, to_px, shape):
    """벽을 위에서 본 흰색 마스크 (한 번만 그려 두고 매 프레임 재사용)"""
    mask = np.zeros(shape, np.uint8)
    for x, y, _, sx, sy, _ in course['boxes']:
        corners = np.array([[x - sx / 2, y - sy / 2], [x + sx / 2, y - sy / 2], [x + sx / 2, y + sy / 2], [x - sx / 2, y + sy / 2]])
        cv2.fillPoly(mask, [to_px(corners)], 255)
    return cv2.dilate(mask, np.ones((3, 3), np.uint8))


def wall_boxes(course):
    return course['boxes']
