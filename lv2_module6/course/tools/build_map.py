"""코스 벽(walls.json) → Nav2 map_server 지도 (PGM + YAML)

시뮬레이터·테스트(faker)에서 진짜 Nav2가 쓸 정지 지도. 실기는 RTAB-Map이 지도와 map→odom을 낸다.
  map 원점 = 출발 칸(spawn_xy_m) — 시뮬레이터와 같은 규약 (odom 원점 = spawn, faker는 map→odom 항등)
  벽 = 점유(0, 검정) · 나머지 = 빈 곳(254). 해상도 RES.
  출력: ros2_ws/src/faker/maps/<코스>.pgm · <코스>.yaml  (코스가 바뀔 때만 다시 실행해 커밋)

  python3 lv2_module6/course/tools/build_map.py
"""
import json
import math
from pathlib import Path

RES = 0.05      # [m/칸] — 통로 0.73m = 약 15칸, 벽판 두께도 1칸 이상 잡힘
MARGIN = 0.20   # 미로 바깥 여유 [m]

HERE = Path(__file__).resolve().parent
COURSE = HERE.parent / 'twin_closed_maze' / 'walls.json'
OUT = HERE.parents[1] / 'ros2_ws' / 'src' / 'faker' / 'maps'


def build(course_path=COURSE, out_dir=OUT, res=RES):
    walls = json.loads(Path(course_path).read_text())
    name = walls['name']
    w, h = walls['size_m']
    sx0, sy0 = walls['spawn_xy_m']
    x0, y0 = -w / 2 - MARGIN, -h / 2 - MARGIN          # 미로 좌표의 지도 왼쪽 아래
    cols, rows = math.ceil((w + 2 * MARGIN) / res), math.ceil((h + 2 * MARGIN) / res)
    grid = [[254] * cols for _ in range(rows)]          # 행 0 = 지도 위쪽(y 최대)
    for b in walls['boxes']:
        c0 = max(0, math.floor((b['x'] - b['sx'] / 2 - x0) / res))
        c1 = min(cols - 1, math.ceil((b['x'] + b['sx'] / 2 - x0) / res) - 1)
        r0 = max(0, math.floor((b['y'] - b['sy'] / 2 - y0) / res))
        r1 = min(rows - 1, math.ceil((b['y'] + b['sy'] / 2 - y0) / res) - 1)
        for r in range(r0, r1 + 1):
            row = grid[rows - 1 - r]
            for c in range(c0, c1 + 1):
                row[c] = 0
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    pgm = out_dir / f'{name}.pgm'
    pgm.write_bytes(f'P5\n# {walls["source"]}\n{cols} {rows}\n255\n'.encode() + bytes(v for row in grid for v in row))
    (out_dir / f'{name}.yaml').write_text(
        f'# course/tools/build_map.py 가 {Path(course_path).name}에서 만든 지도 — 원점 = 출발 칸 (map = odom 원점)\n'
        f'image: {name}.pgm\nmode: trinary\nresolution: {res}\n'
        f'origin: [{x0 - sx0:.3f}, {y0 - sy0:.3f}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return pgm, grid, (x0 - sx0, y0 - sy0)


if __name__ == '__main__':
    pgm, grid, origin = build()
    rows, cols = len(grid), len(grid[0])
    occ = sum(v == 0 for row in grid for v in row)
    # 자체 검사: 출발 칸(map 0,0)은 빈 곳, 미로 바깥 테두리 벽은 점유
    c, r = round(-origin[0] / RES), round(-origin[1] / RES)
    assert grid[rows - 1 - r][c] == 254, '출발 칸이 벽으로 찍힘'
    assert occ > 0.03 * rows * cols, '벽이 너무 적음'
    print(f'{pgm} {cols}x{rows} 점유 {occ}칸 원점 {origin}')
