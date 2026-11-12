#!/usr/bin/env python3
"""collision_monitor Approach(velocity_polygon) 시험 — ROS 없이: python3 test/test_collision_monitor.py (pytest 도 됨)

Nav2 approach 계산을 그대로 옮겨 nav2_params.yaml 의 영역으로 돌린다 (nav2_collision_monitor polygon.cpp getCollisionTime ·
velocity_polygon.cpp updatePolygon):
  ① 지금 속도로 영역 고르기 (velocity_polygons 순서대로 첫 번째로 맞는 것 — x·ωz 범위)
  ② 점이 이미 영역 안에 min_points 이상이면 0 (= 정지, 방향 상관없음)
  ③ 아니면 time_before_collision 동안 simulation_time_step 마다 앞으로 굴려 보고 처음 닿는 시간, 없으면 -1
확인: 벽이 몸 앞 0.19 m (예전 원 0.20 안 — 2026-11-11 교착 상황)일 때 전진은 막히고 후진·제자리 회전은 막히지 않음.
"""
import math
from pathlib import Path

import yaml

PARAMS = Path(__file__).resolve().parents[1] / 'config' / 'nav2_params.yaml'


def approach_cfg():
    cm = yaml.safe_load(PARAMS.read_text())['collision_monitor']['ros__parameters']
    return cm['Approach']


def parse_points(text):
    return [tuple(p) for p in yaml.safe_load(text)]


def inside(poly, x, y):
    """점이 다각형 안인가 (짝수-홀수 규칙 — Nav2 Polygon::isPointInside와 같은 방식)."""
    hit = False
    j = len(poly) - 1
    for i in range(len(poly)):
        (xi, yi), (xj, yj) = poly[i], poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            hit = not hit
        j = i
    return hit


def pick(cfg, vx, wz):
    if cfg['type'] == 'circle':   # 예전 설정 비교용 — 원 하나 (방향 상관없이 같은 영역)
        r = cfg['radius']
        return 'circle', [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a in range(0, 360, 5)]
    for name in cfg['velocity_polygons']:
        sub = cfg[name]
        if sub['linear_min'] <= vx <= sub['linear_max'] and sub['theta_min'] <= wz <= sub['theta_max']:
            return name, parse_points(sub['points'])
    return None, None


def collision_time(cfg, points, vx, wz):
    name, poly = pick(cfg, vx, wz)
    if poly is None:
        return name, -1.0
    if sum(inside(poly, x, y) for x, y in points) >= cfg['min_points']:
        return name, 0.0   # 이미 안 — 방향 상관없이 정지
    px = py = th = 0.0
    t, dt = 0.0, cfg['simulation_time_step']
    while t <= cfg['time_before_collision'] + 1e-9:
        px += vx * math.cos(th) * dt
        py += vx * math.sin(th) * dt
        th += wz * dt
        c, s = math.cos(-th), math.sin(-th)
        moved = [((x - px) * c - (y - py) * s, (x - px) * s + (y - py) * c) for x, y in points]
        if sum(inside(poly, x, y) for x, y in moved) >= cfg['min_points']:
            return name, t
        t += dt
    return name, -1.0


def wall_ahead(dist):
    """몸 앞 dist[m]에 가로로 선 벽 — 앞 카메라 깊이 점구름처럼 1 cm 간격 (폭 ±0.16 m)."""
    return [(dist, -0.16 + 0.01 * i) for i in range(33)]


def test_wall_inside_old_circle_blocks_forward_only():
    cfg = approach_cfg()
    assert cfg['type'] == 'velocity_polygon' and cfg['action_type'] == 'approach'
    pts = wall_ahead(0.19)   # 몸(0.18) 바로 앞 — 예전 원(0.20) 안이라 전부 막히던 자리
    name, t = collision_time(cfg, pts, 0.05, 0.0)
    assert name == 'forward' and 0 <= t <= 0.1, f'앞에 붙은 벽 — 전진은 바로 막혀야 함: {name} {t}'
    name, t = collision_time(cfg, pts, -0.05, 0.0)
    assert name == 'backward' and t < 0, f'후진(BackUp)은 막히면 안 됨: {name} {t}'
    name, t = collision_time(cfg, pts, 0.0, 1.0)
    assert name == 'rotate' and t < 0, f'제자리 회전(Spin)은 막히면 안 됨: {name} {t}'


def test_old_circle_blocked_backup_and_spin():
    """예전 설정(원 0.20 하나)이면 같은 자리에서 후진·회전도 0 = 정지 — 교착의 원인을 시험으로 남겨 둔다."""
    old = dict(approach_cfg(), type='circle', radius=0.20)
    pts = wall_ahead(0.19)
    assert collision_time(old, pts, -0.05, 0.0) == ('circle', 0.0)
    assert collision_time(old, pts, 0.0, 1.0) == ('circle', 0.0)


def test_touching_blocks_rotation_too():
    cfg = approach_cfg()
    pts = wall_ahead(0.15)   # 몸 안까지 들어온 점 = 실제로 닿음
    assert collision_time(cfg, pts, 0.0, 1.0) == ('rotate', 0.0)


def test_far_wall_slows_not_stops():
    cfg = approach_cfg()
    name, t = collision_time(cfg, wall_ahead(0.40), 0.26, 0.0)
    assert name == 'forward' and t > 0, f'멀리 있는 벽 — 앞으로 굴려 본 뒤 닿는 시간 (0이면 즉시 정지): {t}'
    assert collision_time(cfg, wall_ahead(1.50), 0.26, 0.0)[1] < 0, '1 s 안에 안 닿으면 그대로'


def test_every_velocity_has_a_polygon():
    cfg = approach_cfg()
    for vx in (-0.26, -0.01, -0.005, 0.0, 0.003, 0.005, 0.01, 0.26):
        for wz in (-1.82, 0.0, 1.82):
            assert pick(cfg, vx, wz)[0] is not None, f'영역 없는 속도: vx={vx} wz={wz}'


if __name__ == '__main__':
    for fn in (test_wall_inside_old_circle_blocks_forward_only, test_old_circle_blocked_backup_and_spin,
               test_touching_blocks_rotation_too,
               test_far_wall_slows_not_stops, test_every_velocity_has_a_polygon):
        fn()
    print('PASS test_collision_monitor')
