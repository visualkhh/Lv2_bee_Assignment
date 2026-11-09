"""Isaac 미로(USD) → Nav2 map_server 지도 (PGM + YAML) — 기존 코스 지도와 같은 규약

Isaac Lab 장면(isaac/usda/scene_sim.usda)이 쓰는 미로 isaac/usda/maze/maze.usda의 collision 메시를
위에서 본 모양 그대로 칠한다. 대각선 벽이 있어 walls.json(축에 평행한 상자)으로는 옮길 수 없어서 메시를 직접 쓴다.
  map 원점 = 출발 칸 — scene_sim.usda의 beebot 위치를 미로 좌표로 옮긴 곳 (faker: map = odom 원점)
  벽 = 점유(0, 검정) · 나머지 = 빈 곳(254). 해상도 RES. 높이 Z_MIN~Z_MAX에 걸친 collision 삼각형만.
  출력: ros2_ws/src/faker/world/isaac_maze.pgm · isaac_maze.yaml  (미로가 바뀔 때만 다시 실행해 커밋)

  pip install usd-core   # pxr — 이 도구를 돌릴 때만 필요
  python3 lv2_module6/tools/course/build_map_usd.py
"""
import math
from pathlib import Path

from pxr import Gf, Usd, UsdGeom

RES = 0.05            # [m/칸] — 기존 코스 지도와 같게
MARGIN = 0.20         # 미로 바깥 여유 [m]
Z_MIN, Z_MAX = 0.03, 0.30  # 로봇이 부딪히는 높이 [m] (nvblox slice와 같은 범위)
NAME = 'isaac_maze'

HERE = Path(__file__).resolve().parent
ISAAC = HERE.parents[1] / 'isaac' / 'usda'
MAZE = ISAAC / 'maze' / 'maze.usda'
SCENE = ISAAC / 'scene_sim.usda'
OUT = HERE.parents[1] / 'ros2_ws' / 'src' / 'faker' / 'world'


def spawn_in_maze(scene_path=SCENE):
    """scene_sim.usda의 /World/beebot 위치를 /World/maze 좌표로 (x, y) — 출발 칸."""
    st = Usd.Stage.Open(str(scene_path))  # LoadNone이면 payload 프림(beebot·maze)이 빠져 위치를 못 읽음
    t = Usd.TimeCode.Default()
    prims = {n: st.GetPrimAtPath(f'/World/{n}') for n in ('beebot', 'maze')}
    for n, prim in prims.items():
        if not prim.IsValid():
            raise SystemExit(f'{scene_path}에 /World/{n} 없음')
    robot = UsdGeom.Xformable(prims['beebot']).ComputeLocalToWorldTransform(t)
    maze = UsdGeom.Xformable(prims['maze']).ComputeLocalToWorldTransform(t)
    p = maze.GetInverse().Transform(robot.ExtractTranslation())
    return float(p[0]), float(p[1])


def collision_triangles(maze_path=MAZE):
    """collision 메시(purpose guide, …/collision/…) 삼각형 → [(x, y, z) × 3] (미로 좌표)."""
    st = Usd.Stage.Open(str(maze_path))
    t = Usd.TimeCode.Default()
    tris = []
    for prim in Usd.PrimRange(st.GetDefaultPrim()):
        if not prim.IsA(UsdGeom.Mesh) or '/collision/' not in str(prim.GetPath()):
            continue
        mesh = UsdGeom.Mesh(prim)
        xf = mesh.ComputeLocalToWorldTransform(t)
        pts = [xf.Transform(Gf.Vec3d(*p)) for p in mesh.GetPointsAttr().Get()]
        counts, idx = mesh.GetFaceVertexCountsAttr().Get(), mesh.GetFaceVertexIndicesAttr().Get()
        k = 0
        for n in counts:  # 다각형 면 → 부채꼴 삼각형
            face = [pts[i] for i in idx[k:k + n]]
            k += n
            for j in range(1, n - 1):
                tris.append((face[0], face[j], face[j + 1]))
    return tris


def build(maze_path=MAZE, scene_path=SCENE, out_dir=OUT, res=RES):
    tris = [tr for tr in collision_triangles(maze_path)
            if max(p[2] for p in tr) >= Z_MIN and min(p[2] for p in tr) <= Z_MAX]
    if not tris:
        raise SystemExit(f'collision 삼각형 없음: {maze_path}')
    xs = [p[0] for tr in tris for p in tr]
    ys = [p[1] for tr in tris for p in tr]
    x0, y0 = min(xs) - MARGIN, min(ys) - MARGIN               # 미로 좌표의 지도 왼쪽 아래
    cols, rows = math.ceil((max(xs) + MARGIN - x0) / res), math.ceil((max(ys) + MARGIN - y0) / res)
    occ = [[False] * cols for _ in range(rows)]               # occ[r][c], r = 0 아래쪽

    def mark(x, y):
        c, r = int((x - x0) // res), int((y - y0) // res)
        if 0 <= c < cols and 0 <= r < rows:
            occ[r][c] = True

    for a, b, c in tris:
        # 칸 중심이 삼각형(위에서 본) 안이면 점유 + 변을 촘촘히 따라 찍기 — 벽판(2 cm)이 칸(5 cm)보다 얇아도 빠지지 않게
        for p, q in ((a, b), (b, c), (c, a)):
            n = max(1, int(math.hypot(q[0] - p[0], q[1] - p[1]) / (res / 4)))
            for i in range(n + 1):
                u = i / n
                mark(p[0] + u * (q[0] - p[0]), p[1] + u * (q[1] - p[1]))
        area = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])
        if abs(area) < 1e-12:
            continue
        c0, c1 = int((min(a[0], b[0], c[0]) - x0) // res), int((max(a[0], b[0], c[0]) - x0) // res)
        r0, r1 = int((min(a[1], b[1], c[1]) - y0) // res), int((max(a[1], b[1], c[1]) - y0) // res)
        for r in range(max(0, r0), min(rows - 1, r1) + 1):
            for cc in range(max(0, c0), min(cols - 1, c1) + 1):
                px, py = x0 + (cc + 0.5) * res, y0 + (r + 0.5) * res
                w0 = (b[0] - a[0]) * (py - a[1]) - (px - a[0]) * (b[1] - a[1])
                w1 = (c[0] - b[0]) * (py - b[1]) - (px - b[0]) * (c[1] - b[1])
                w2 = (a[0] - c[0]) * (py - c[1]) - (px - c[0]) * (a[1] - c[1])
                if (w0 >= 0 and w1 >= 0 and w2 >= 0) or (w0 <= 0 and w1 <= 0 and w2 <= 0):
                    occ[r][cc] = True

    sx0, sy0 = spawn_in_maze(scene_path)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    pgm = out_dir / f'{NAME}.pgm'
    rows_top_first = [[0 if occ[r][c] else 254 for c in range(cols)] for r in reversed(range(rows))]
    src = f'{maze_path.relative_to(HERE.parents[1])} collision via tools/course/build_map_usd.py'
    pgm.write_bytes(f'P5\n# {src}\n{cols} {rows}\n255\n'.encode() + bytes(v for row in rows_top_first for v in row))
    (out_dir / f'{NAME}.yaml').write_text(
        f'# tools/course/build_map_usd.py 가 isaac/usda/maze/maze.usda에서 만든 지도 — 원점 = 출발 칸 '
        f'(scene_sim.usda beebot = 미로 ({sx0:.3f}, {sy0:.3f}))\n'
        f'image: {NAME}.pgm\nmode: trinary\nresolution: {res}\n'
        f'origin: [{x0 - sx0:.3f}, {y0 - sy0:.3f}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return pgm, occ, (x0 - sx0, y0 - sy0)


if __name__ == '__main__':
    pgm, occ, origin = build()
    rows, cols = len(occ), len(occ[0])
    n = sum(v for row in occ for v in row)
    # 자체 검사: 출발 칸(map 0,0)은 빈 곳, 벽이 충분히 있음
    c, r = int(-origin[0] // RES), int(-origin[1] // RES)
    assert not occ[r][c], '출발 칸이 벽으로 찍힘'
    assert n > 0.03 * rows * cols, '벽이 너무 적음'
    print(f'{pgm} {cols}x{rows} 점유 {n}칸 원점 {origin}')
