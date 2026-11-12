"""RealRobot — 테스트베드의 실제 로봇 몸. fake/real/robot/beebot.usda (Isaac 로봇 USD — isaac/usda/beebot 사본).

fake_opencr(가짜 OpenCR)가 벽에 부딪히는 몸 외곽을 여기서 뽑는다 — 손으로 옮긴 상수 대신 (tools/course/build_world.py가
faker/config/robot_footprint.yaml로 씀). 2026-11-11: 상수(뒤 −0.233)는 뒤 배터리 상자(−0.245)를 빠뜨렸었음.
"""
from __future__ import annotations

from pathlib import Path

ROBOT = Path(__file__).resolve().parent / 'real' / 'robot' / 'beebot.usda'
FLOOR_Z = 0.30   # [m] 바닥에서 이 높이 안의 몸만 벽과 부딪힌다 (RealWorld.Z_MAX와 같게)


class RealRobot:
    def __init__(self, path: Path = ROBOT):
        from pxr import Usd   # usd-core (tools/requirements.txt)
        self.stage = Usd.Stage.Open(str(path))

    def footprint(self, z_max: float = FLOOR_Z) -> dict:
        """몸 외곽 (base_footprint 기준 [m], 팔 제외 · 바닥~z_max) → {'front', 'rear', 'side'} — rear는 음수.
        충돌 도형(UsdPhysics.CollisionAPI) 각각의 축 정렬 경계 상자를 모은다. 좌우는 큰 쪽으로 대칭."""
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        t = Usd.TimeCode.Default()
        fp = self.stage.GetPrimAtPath(f'{self.stage.GetDefaultPrim().GetPath()}/Geometry/base_footprint')
        inv = UsdGeom.Xformable(fp).ComputeLocalToWorldTransform(t).GetInverse()
        cache = UsdGeom.BBoxCache(t, [UsdGeom.Tokens.default_, UsdGeom.Tokens.guide], useExtentsHint=False)
        xs, ys = [], []
        for prim in Usd.PrimRange(fp):
            if not prim.HasAPI(UsdPhysics.CollisionAPI) or 'arm_base_link' in str(prim.GetPath()):
                continue   # 팔은 움직여서 몸 외곽에서 뺌 (주행 중 stow)
            box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if box.IsEmpty():
                continue
            pts = [inv.Transform(Gf.Vec3d(x, y, z)) for x in (box.GetMin()[0], box.GetMax()[0])
                   for y in (box.GetMin()[1], box.GetMax()[1]) for z in (box.GetMin()[2], box.GetMax()[2])]
            if max(p[2] for p in pts) < 0.0 or min(p[2] for p in pts) > z_max:
                continue
            xs += [p[0] for p in pts]
            ys += [p[1] for p in pts]
        if not xs:
            raise RuntimeError(f'몸 충돌 도형 없음: {ROBOT}')
        return {'front': round(max(xs), 4), 'rear': round(min(xs), 4), 'side': round(max(abs(v) for v in ys), 4)}
