"""주사위 맵 (documents/mission/dice-map.png · dice-mission.pdf p8) → dice_maze/walls.json + Nav2 지도

  python3 lv2_module6/course/tools/build_dice.py

좌표: 아래 표는 경기장(arena) mm — 왼쪽 아래 기둥 중심 = 원점, +Y = 지도 위쪽 (발제 그대로).
walls.json은 twin_closed_maze와 같은 규약(미로 중심 원점, m)으로 바꿔 쓴다 → 시뮬레이터·build_map.py가 그대로 읽음.
벽 선분은 dice-map.png에서 축 눈금으로 보정해 읽은 값을 기둥 격자(X 367 · Y 1315+734k)에 맞춘 것 — 실측이 나오면 여기만 고친다.
'mission' 키 = 시뮬레이터가 까는 미션 물체 (arena mm 그대로): 박스 · 장소(태그 면) · 신호등 · 바코드 · 출발 구역.
"""
import json
from pathlib import Path

import build_map

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / 'dice_maze'
SIZE = (4404, 6304)        # 바깥 기둥 중심 사이 [mm] — 기둥 60mm 포함 경기장 4464 × 6364
CENTER = (2202, 3152)      # arena → 미로 중심 원점
WALL_H, PANEL, POST = 0.39, 0.02, 0.06
BOX_H = 0.070

# 벽 선분 (x0, y0, x1, y1) arena mm — 바깥 테두리 + 안쪽 15개
WALLS = [
    (0, 0, 4404, 0), (0, 6304, 4404, 6304), (0, 0, 0, 6304), (4404, 0, 4404, 6304),
    (1838, 4985, 2571, 4985), (367, 4251, 1101, 4251), (3304, 4251, 4037, 4251), (3304, 3517, 4037, 3517),
    (1101, 2049, 1838, 2049), (2571, 2049, 3304, 2049), (1101, 1315, 1838, 1315), (2571, 1315, 4037, 1315),
    (1101, 4985, 1101, 5719), (2571, 4985, 2571, 5719), (3304, 4985, 3304, 5719), (1101, 3517, 1101, 4251),
    (3304, 2049, 3304, 3517), (1101, 2049, 1101, 2783), (1838, 1315, 1838, 2049), (1101, 650, 1101, 1315),
]
# 박스 (이름, x, y, 한 변) arena mm, 높이 70 — 시작 1~6 · 장소 L1 L3 · 중앙 받침대 (410 박스 4개 = 820, L2는 그 북쪽 절반 위)
BOXES = [('start_1', 1468, 950, 205), ('start_2', 3670, 3152, 205), ('start_3', 2936, 5354, 205),
         ('start_4', 734, 5354, 205), ('start_5', 2202, 5354, 205), ('start_6', 3670, 3886, 205),
         ('L1', 3670, 950, 205), ('L3', 3670, 5354, 205), ('pedestal', 2202, 3495, 820)]
MISSION = {
    'start': {'x': 734, 'y': 950, 'yaw_deg': 180, 'zone': [434, 650, 1034, 1250]},   # 출발 구역 [x0, y0, x1, y1] (그림)
    # 장소 — 격자 중심 · 태그 번호 · 태그가 붙은 면의 바깥 방향(접근 방향 쪽) · 접근 점(그림의 주황 원)
    'locations': {'L1': {'x': 3670, 'y': 950, 'tag': 12, 'normal': [-1, 0], 'approach': [3223, 950]},
                  'L2': {'x': 2202, 'y': 3700, 'tag': 13, 'normal': [0, 1], 'approach': [2202, 4265]},
                  'L3': {'x': 3670, 'y': 5354, 'tag': 14, 'normal': [0, -1], 'approach': [3670, 4906]}},
    'signal_light': {'x': 80, 'y': 950, 'facing_deg': 0},
    'barcode': {'x': 80, 'y': 650, 'z': 450, 'facing_deg': 0},
    'puck_offsets_mm': [-40, 40],        # 시작 박스 위 퍽 두 개 — 중심 X±40, Y=0
}


def to_course(x, y):
    return round((x - CENTER[0]) / 1000, 4), round((y - CENTER[1]) / 1000, 4)


def main():
    boxes, posts = [], set()
    for i, (x0, y0, x1, y1) in enumerate(WALLS):
        cx, cy = to_course((x0 + x1) / 2, (y0 + y1) / 2)
        horiz = y0 == y1
        length = abs(x1 - x0 if horiz else y1 - y0) / 1000
        boxes.append({'name': f'panel_{i}', 'x': cx, 'y': cy,
                      'sx': length if horiz else PANEL, 'sy': PANEL if horiz else length})
        posts |= {(x0, y0), (x1, y1)}
    boxes += [{'name': f'post_{i}', 'x': to_course(*p)[0], 'y': to_course(*p)[1], 'sx': POST, 'sy': POST}
              for i, p in enumerate(sorted(posts))]
    boxes += [{'name': f'box_{n}', 'x': to_course(x, y)[0], 'y': to_course(x, y)[1], 'sx': s / 1000, 'sy': s / 1000,
               'sz': BOX_H} for n, x, y, s in BOXES]
    start = MISSION['start']
    walls = {'name': 'dice_maze', 'size_m': [4.464, 6.364], 'wall_height_m': WALL_H,
             'spawn_xy_m': list(to_course(start['x'], start['y'])), 'arena_center_mm': list(CENTER),
             'source': 'documents/mission/dice-map.png via course/tools/build_dice.py',
             'boxes': boxes, 'mission': {**MISSION, 'boxes': [{'name': n, 'x': x, 'y': y, 'size': s} for n, x, y, s in BOXES]}}
    OUT.mkdir(exist_ok=True)
    path = OUT / 'walls.json'
    path.write_text(json.dumps(walls, indent=1, ensure_ascii=False) + '\n')
    # 1 cm 칸: fake_opencr 충돌이 이 지도를 씀 — 5 cm면 박스가 최대 5 cm 커져 박스 앞 1.6 cm 접근을 막음
    pgm, grid, origin = build_map.build(path, res=0.01)
    print(f'{path}: 벽 {len(WALLS)} · 기둥 {len(posts)} · 박스 {len(BOXES)} → {pgm} 원점 {origin}')


if __name__ == '__main__':
    main()
