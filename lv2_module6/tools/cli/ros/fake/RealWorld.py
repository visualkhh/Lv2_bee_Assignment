"""RealWorld — 테스트베드의 실세계 하나. fake/real/map/<코스>/ 에 둔 원본에서 벽을 읽는다.

  isaac_maze  maze.usda + geo.usd (Isaac 미로 3D 원본 — isaac/usda/maze 사본) · course.json (출발 자세)
  dice_maze   walls.json (주사위 미션 경기장 — 2 cm 벽판 상자)

화면(뷰어 3D 벽) · 충돌(fake_opencr 월드 PGM — tools/course/build_world.py) · --source fake 세계가 모두 이것 하나로 만든다
→ 보이는 벽 = 부딪히는 벽 (2026-11-11: 그림에서 따로 만든 PGM이 뷰어 벽보다 1~2 cm 두꺼워 빈 곳에서 막혔음).

좌표 = map — 실기처럼 켰을 때 로봇 자리가 원점이고 로봇 앞이 +x (원본 좌표를 출발 자세로 옮기고 돌림).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

REAL = Path(__file__).resolve().parent / 'real' / 'map'
DEFAULT = 'isaac_maze'
Z_MIN, Z_MAX = 0.03, 0.30   # 로봇이 부딪히는 높이 [m] — 이 높이에 걸친 벽만 충돌 (nvblox slice와 같은 범위)


def courses() -> list[str]:
    return sorted(p.name for p in REAL.iterdir() if p.is_dir()) if REAL.is_dir() else []


class RealWorld:
    """triangles = [((x,y,z)×3)] map 좌표 · height = 벽 높이 [m] · spawn = 원본 좌표의 출발 (x, y, yaw[rad])."""

    def __init__(self, course: str = DEFAULT):
        self.name = course
        d = REAL / course
        if (d / 'maze.usda').is_file():
            self._load_usd(d)
        elif (d / 'walls.json').is_file():
            self._load_walls(d)
        else:
            raise FileNotFoundError(f'실세계 없음: {d} (maze.usda 또는 walls.json) — 있는 코스: {", ".join(courses())}')

    # ---------------------------------------------------------------- 원본 → map 좌표
    def _to_map(self, x: float, y: float) -> tuple[float, float]:
        sx, sy, syaw = self.spawn
        dx, dy = x - sx, y - sy
        c, s = math.cos(-syaw), math.sin(-syaw)
        return dx * c - dy * s, dx * s + dy * c

    def _load_usd(self, d: Path) -> None:
        from pxr import Gf, Usd, UsdGeom   # usd-core (tools/requirements.txt)
        meta = json.loads((d / 'course.json').read_text())
        sx, sy = meta['spawn_xy_m']
        self.spawn = (float(sx), float(sy), math.radians(float(meta.get('spawn_yaw_deg', 0.0))))
        st = Usd.Stage.Open(str(d / 'maze.usda'))
        t = Usd.TimeCode.Default()
        tris, zmax = [], 0.0
        for prim in Usd.PrimRange(st.GetDefaultPrim()):
            if not prim.IsA(UsdGeom.Mesh) or '/collision/' not in str(prim.GetPath()):
                continue   # 충돌 메시만 (보이는 메시와 모양이 같고 삼각형이 적음)
            mesh = UsdGeom.Mesh(prim)
            xf = mesh.ComputeLocalToWorldTransform(t)
            pts = [xf.Transform(Gf.Vec3d(*p)) for p in mesh.GetPointsAttr().Get()]
            counts, idx = mesh.GetFaceVertexCountsAttr().Get(), mesh.GetFaceVertexIndicesAttr().Get()
            k = 0
            for n in counts:   # 다각형 면 → 부채꼴 삼각형
                face = [pts[i] for i in idx[k:k + n]]
                k += n
                for j in range(1, n - 1):
                    tri = []
                    for p in (face[0], face[j], face[j + 1]):
                        mx, my = self._to_map(p[0], p[1])
                        tri.append((mx, my, float(p[2])))
                        zmax = max(zmax, float(p[2]))
                    tris.append(tuple(tri))
        if not tris:
            raise RuntimeError(f'충돌 메시 없음: {d / "maze.usda"}')
        self.triangles, self.height = tris, zmax

    def _load_walls(self, d: Path) -> None:
        w = json.loads((d / 'walls.json').read_text())
        sx, sy = w['spawn_xy_m']
        self.spawn = (float(sx), float(sy), math.radians(float(w.get('spawn_yaw_deg', 0.0))))
        h = float(w.get('wall_height_m', 0.3))
        tris = []
        for b in w['boxes']:   # 축에 평행한 상자 → 옆면 4장 + 윗면 (삼각형 2개씩)
            x0, x1 = b['x'] - b['sx'] / 2, b['x'] + b['sx'] / 2
            y0, y1 = b['y'] - b['sy'] / 2, b['y'] + b['sy'] / 2
            top = float(b.get('sz', h))
            c = [self._to_map(x, y) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
            for i in range(4):
                a, bb = c[i], c[(i + 1) % 4]
                p = [(a[0], a[1], 0.0), (bb[0], bb[1], 0.0), (bb[0], bb[1], top), (a[0], a[1], top)]
                tris += [(p[0], p[1], p[2]), (p[0], p[2], p[3])]
            q = [(x, y, top) for x, y in c]
            tris += [(q[0], q[1], q[2]), (q[0], q[2], q[3])]
        self.triangles, self.height = tris, h

    # ---------------------------------------------------------------- 2D 충돌 격자
    def occupancy(self, res: float = 0.01, margin: float = 0.2):
        """Z_MIN~Z_MAX에 걸친 삼각형을 위에서 칠한 격자 → (w, h, ox, oy, occ[r][c]) — r 0 = 아래쪽(작은 y).
        얇은 벽판(2 cm)이 칸보다 얇아도 빠지지 않게 변을 촘촘히 따라 찍고, 칸 중심이 삼각형 안이면 칠한다."""
        tris = [t for t in self.triangles if max(p[2] for p in t) >= Z_MIN and min(p[2] for p in t) <= Z_MAX]
        xs = [p[0] for t in tris for p in t]
        ys = [p[1] for t in tris for p in t]
        ox, oy = min(xs) - margin, min(ys) - margin
        w, h = math.ceil((max(xs) + margin - ox) / res), math.ceil((max(ys) + margin - oy) / res)
        occ = [[False] * w for _ in range(h)]

        def mark(x, y):
            c, r = int((x - ox) // res), int((y - oy) // res)
            if 0 <= c < w and 0 <= r < h:
                occ[r][c] = True

        for a, b, c in tris:
            for p, q in ((a, b), (b, c), (c, a)):
                n = max(1, int(math.hypot(q[0] - p[0], q[1] - p[1]) / (res / 4)))
                for i in range(n + 1):
                    u = i / n
                    mark(p[0] + u * (q[0] - p[0]), p[1] + u * (q[1] - p[1]))
            area = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])
            if abs(area) < 1e-12:
                continue
            c0, c1 = int((min(a[0], b[0], c[0]) - ox) // res), int((max(a[0], b[0], c[0]) - ox) // res)
            r0, r1 = int((min(a[1], b[1], c[1]) - oy) // res), int((max(a[1], b[1], c[1]) - oy) // res)
            for r in range(max(0, r0), min(h - 1, r1) + 1):
                py = oy + (r + 0.5) * res
                for cc in range(max(0, c0), min(w - 1, c1) + 1):
                    px = ox + (cc + 0.5) * res
                    w0 = (b[0] - a[0]) * (py - a[1]) - (px - a[0]) * (b[1] - a[1])
                    w1 = (c[0] - b[0]) * (py - b[1]) - (px - b[0]) * (c[1] - b[1])
                    w2 = (a[0] - c[0]) * (py - c[1]) - (px - c[0]) * (a[1] - c[1])
                    if (w0 >= 0 and w1 >= 0 and w2 >= 0) or (w0 <= 0 and w1 <= 0 and w2 <= 0):
                        occ[r][cc] = True
        return w, h, ox, oy, occ

    def write_pgm(self, out_dir: Path, res: float = 0.01) -> Path:
        """map_server 형식 (yaml + pgm, 0 = 벽 · 254 = 빈 곳) — fake_opencr 충돌 월드."""
        w, h, ox, oy, occ = self.occupancy(res)
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        pgm = out_dir / f'{self.name}.pgm'
        rows = bytes(0 if occ[r][c] else 254 for r in reversed(range(h)) for c in range(w))
        src = f'tools/cli/ros/fake/real/map/{self.name} via tools/course/build_world.py'
        pgm.write_bytes(f'P5\n# {src}\n{w} {h}\n255\n'.encode() + rows)
        (out_dir / f'{self.name}.yaml').write_text(
            f'# {src} — 실세계(fake_opencr 충돌 모델 — 로봇 지도 아님) · 원점 = 출발 칸, 로봇 앞 = +x\n'
            f'image: {self.name}.pgm\nmode: trinary\nresolution: {res}\n'
            f'origin: [{ox:.3f}, {oy:.3f}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        return pgm

    # ---------------------------------------------------------------- 뷰어용 (축에 평행한 상자 목록)
    def boxes(self, res: float = 0.01) -> list[tuple[float, float, float, float]]:
        """충돌 격자를 축에 평행한 직사각형으로 묶음 → [(x0, y0, x1, y1)] map 좌표 — 대각선 벽은 res 계단.
        뷰어 3D·카메라가 이걸로 그리면 보이는 벽 = fake_opencr가 부딪히는 벽 (같은 격자)."""
        w, h, ox, oy, occ = self.occupancy(res)
        rects, open_runs = [], {}   # (c0, c1) → 시작 행 — 위아래로 같은 가로 구간이면 이어 붙임
        for r in range(h + 1):
            runs, c = set(), 0
            while r < h and c < w:
                if occ[r][c]:
                    c0 = c
                    while c < w and occ[r][c]:
                        c += 1
                    runs.add((c0, c))
                else:
                    c += 1
            for key in list(open_runs):
                if key not in runs:   # 이 행에서 끊김 → 직사각형 하나 확정
                    r0 = open_runs.pop(key)
                    rects.append((ox + key[0] * res, oy + r0 * res, ox + key[1] * res, oy + r * res))
            for key in runs:
                open_runs.setdefault(key, r)
        return rects

    def viewer_course(self, res: float = 0.01, cell: float = 0.4) -> dict:
        """vazebo 뷰어의 코스 형식 (maze.load와 같은 키) — 뷰어 좌표 = map을 범위 가운데로 옮긴 것, spawn = 출발(=map 원점).
        cells = 퍽·장애물 놓을 빈 곳 (벽에서 0.25 m 넘게 떨어진 cell 간격 격자점)."""
        rects = self.boxes(res)
        x0, y0 = min(r[0] for r in rects), min(r[1] for r in rects)
        x1, y1 = max(r[2] for r in rects), max(r[3] for r in rects)
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        hgt = self.height
        boxes = [((a + c) / 2 - cx, (b + d) / 2 - cy, hgt / 2, c - a, d - b, hgt) for a, b, c, d in rects]
        w, h, ox, oy, occ = self.occupancy(0.05)
        k = 5   # 0.25 m / 0.05

        def clear(x, y):
            c, r = int((x - ox) / 0.05), int((y - oy) / 0.05)
            return all(0 <= r + dr < h and 0 <= c + dc < w and not occ[r + dr][c + dc]
                       for dr in range(-k, k + 1) for dc in range(-k, k + 1))
        cells = [(round(x - cx, 3), round(y - cy, 3))
                 for x in [x0 + cell / 2 + i * cell for i in range(int((x1 - x0) / cell))]
                 for y in [y0 + cell / 2 + j * cell for j in range(int((y1 - y0) / cell))] if clear(x, y)]
        return {'name': self.name, 'size': (x1 - x0, y1 - y0), 'spawn': (-cx, -cy), 'wall_h': hgt,
                'boxes': boxes, 'posts': [], 'cells': cells}

