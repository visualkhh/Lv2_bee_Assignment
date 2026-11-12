#!/usr/bin/env python3
"""cuvslam_slam MapGate — 큰 점프는 거부·잠금, 작은 보정은 초당 rate까지만, reset으로 풀림 (rclpy·cuvslam 없이 클래스만)
   시나리오는 11-10 시뮬 T0(벽 앞 4.8 m 점프)와 정상 주행"""
import math
import pathlib

SRC = (pathlib.Path(__file__).resolve().parents[1] / 'scripts' / 'cuvslam_slam.py').read_text()
ns = {'math': math}
exec(SRC[SRC.index('class MapGate'):SRC.index('def image(')], ns)  # noqa: S102 — 같은 저장소 소스
G = ns['MapGate']

g = G()
assert g.update((0.0, 0.0, 0.0), 0.0) == ((0.0, 0.0, 0.0), True)
out, ok = g.update((0.001, 0.0, 0.0), 0.1)                   # 0.1 s에 1 mm — 초당 2 cm 안이라 그대로
assert ok and abs(out[0] - 0.001) < 1e-9
out, ok = g.update((0.05, 0.0, 0.0), 0.2)                     # 0.1 s에 5 cm — 2 mm만 따라감
assert not ok and abs(out[0] - 0.003) < 1e-9 and not g.locked
out, ok = g.update((-4.1, -1.0, 0.5), 0.3)                    # T0: 4.8 m 점프 → 거부 · 잠금
assert not ok and g.locked and abs(out[0] - 0.003) < 1e-9
for i in range(50):
    out, ok = g.update((-4.1, -1.0, 0.5), 0.4 + 0.1 * i)       # 잘못된 자리가 계속 안정돼도 안 받음 (예전 MapGate는 10프레임 뒤 받았음)
assert not ok and abs(out[0] - 0.003) < 1e-9
g.reset()                                                     # /initialpose
assert g.update((1.0, 2.0, 0.1), 6.0) == ((1.0, 2.0, 0.1), True) and not g.locked
print('test_map_gate OK')
