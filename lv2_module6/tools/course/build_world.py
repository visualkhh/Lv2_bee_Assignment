"""실세계(tools/cli/ros/fake/real/map/<코스>) → fake_opencr 충돌 월드 (faker/world/<코스>.pgm · .yaml)
실제 로봇 몸(tools/cli/ros/fake/real/robot/beebot.usda) → fake_opencr 몸 외곽 (faker/config/robot_footprint.yaml)
기본 코스(isaac_maze) → 완벽한 로봇 지도 tools/cli/ros/fake/map/default/ (map.pgm·map.yaml·nvblox.nvblx FAKESLAM1)
  — 매핑 없이 바로 LOCALIZATION 주행 시험용. 도커 /ws/map · --source fake 지도 폴더가 tools/cli/ros/fake/map

뷰어 3D·--source fake와 같은 RealWorld(cli/ros/fake/RealWorld.py)에서 만든다 — 보이는 벽 = 부딪히는 벽.
실세계 원본(USD·walls.json)이 바뀔 때만 다시 실행해 커밋한다. isaac_maze는 usd-core(pxr) 필요 (tools/requirements.txt).

  tools/.venv/bin/python tools/course/build_world.py              # 전부
  tools/.venv/bin/python tools/course/build_world.py isaac_maze   # 하나
"""
import struct
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'cli'))
from ros.fake.RealRobot import RealRobot  # noqa: E402
from ros.fake.RealWorld import DEFAULT, RealWorld, courses  # noqa: E402

OUT = HERE.parents[1] / 'ros2_ws' / 'src' / 'faker' / 'world'
FOOTPRINT = HERE.parents[1] / 'ros2_ws' / 'src' / 'faker' / 'config' / 'robot_footprint.yaml'
RES = 0.01   # [m/칸] — fake_opencr 충돌 판정 해상도
ROBOT_MAP = HERE.parent / 'cli' / 'ros' / 'fake' / 'map' / 'default'
ROBOT_RES = 0.05   # [m/칸] — fake_nvblox 지도 해상도 (매핑한 지도와 같게)


def check(world: RealWorld, res: float = RES) -> None:
    """자체 검사: 출발 칸(map 0,0) 근처 로봇 몸(반경 0.15)은 빈 곳 · 벽이 충분히 있음."""
    w, h, ox, oy, occ = world.occupancy(res)
    c, r = int(-ox // res), int(-oy // res)
    k = int(0.15 // res)
    hit = [(dr, dc) for dr in range(-k, k + 1) for dc in range(-k, k + 1)
           if dr * dr + dc * dc <= k * k and occ[r + dr][c + dc]]
    assert not hit, f'{world.name}: 출발 자리 몸 안에 벽 {len(hit)}칸'
    n = sum(v for row in occ for v in row)
    assert n > 0.01 * w * h, f'{world.name}: 벽이 너무 적음 ({n})'


def write_footprint(path: Path = FOOTPRINT) -> dict:
    fp = RealRobot().footprint()
    path.write_text('# tools/course/build_world.py가 tools/cli/ros/fake/real/robot/beebot.usda(실제 로봇 몸)에서 만든 몸 외곽 — 손으로 고치지 말 것\n'
                    '# base_footprint 기준 [m] · 팔 제외 · 바닥~0.30 m의 충돌 도형 — fake_opencr가 벽과 부딪히는 몸\n'
                    f'fake_opencr:\n  ros__parameters:\n    foot_front: {fp["front"]}\n    foot_rear: {fp["rear"]}\n    foot_side: {fp["side"]}\n')
    return fp


def robot_map(world: RealWorld, res: float = ROBOT_RES):
    """완벽한 로봇 지도 (OccupancyGrid 값) — 벽 100 · 출발 칸에서 이어진 빈 곳 0 · 미로 밖(못 가는 곳) -1 (다 돌아본 매핑과 같게)."""
    w, h, ox, oy, occ = world.occupancy(res)
    data = [100 if occ[r][c] else -1 for r in range(h) for c in range(w)]
    start = int(-oy // res) * w + int(-ox // res)
    assert data[start] == -1, f'{world.name}: 출발 칸이 벽'
    todo = [start]
    data[start] = 0
    while todo:
        i = todo.pop()
        r, c = divmod(i, w)
        for j in ((i - w) if r > 0 else -1, (i + w) if r < h - 1 else -1, (i - 1) if c > 0 else -1, (i + 1) if c < w - 1 else -1):
            if j >= 0 and data[j] == -1:
                data[j] = 0
                todo.append(j)
    return w, h, ox, oy, data


def write_robot_map(world: RealWorld, out: Path = ROBOT_MAP, res: float = ROBOT_RES) -> Path:
    """map.pgm·map.yaml(map_saver 형식) + nvblox.nvblx(fake_nvblox FAKESLAM1: magic 9B · int32 w·h · double res·ox·oy · int8[])."""
    w, h, ox, oy, data = robot_map(world, res)
    out.mkdir(parents=True, exist_ok=True)
    (out / 'nvblox.nvblx').write_bytes(b'FAKESLAM1' + struct.pack('<iiddd', w, h, res, ox, oy)
                                       + struct.pack(f'<{w * h}b', *data))
    px = {100: 0, 0: 254, -1: 205}
    (out / 'map.pgm').write_bytes(f'P5\n{w} {h}\n255\n'.encode()
                                  + bytes(px[data[r * w + c]] for r in reversed(range(h)) for c in range(w)))
    (out / 'map.yaml').write_text(f'image: map.pgm\nmode: trinary\nresolution: {res}\norigin: [{ox:.3f}, {oy:.3f}, 0.0]\n'
                                  'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    free, wall = data.count(0), data.count(100)
    assert free > 0.1 * w * h and wall > 0, (free, wall)   # 자체 검사: 미로 안이 비었고 벽이 있음
    return out


if __name__ == '__main__':
    print(f'{FOOTPRINT} · {write_footprint()}')
    for name in (sys.argv[1:] or courses()):
        t0 = time.time()
        world = RealWorld(name)
        check(world)
        pgm = world.write_pgm(OUT, RES)
        print(f'{pgm} · 삼각형 {len(world.triangles)} · {time.time() - t0:.1f}s')
        if name == DEFAULT:
            print(f'{write_robot_map(world)} (완벽한 로봇 지도 default)')
