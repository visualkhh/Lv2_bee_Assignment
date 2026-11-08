#!/usr/bin/env python3
"""manipulator_env.py --puck, --seed parsing. No Isaac needed.

  python3 isaac/scripts/standalone/test_manipulator_layout.py
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "rl"))
from bee_isaac.constants import BOX_HALF, PUCK_XY, PUCK_Z  # noqa: E402
from bee_isaac.scenes.manipulator_layout import CHASSIS_FRONT, MARGIN, MIN_SEP, layout, random_layout  # noqa: E402

a, b = layout()  # default stays scene_sim start_1
assert (a.name, a.asset, a.rot[2]) == ("puck_box_red_0", "puck/box/puck_box_red_1.usda", 165.7)
assert (b.name, b.asset, b.rot[2]) == ("puck_box_green_6", "puck/box/puck_box_green_0.usda", 352.9)
assert a.pos == (*PUCK_XY[0], PUCK_Z) and b.pos == (*PUCK_XY[1], PUCK_Z)

a, b = layout(["red:cyl:stand:3", "green:box:lie:7"])
assert (a.shape, a.color, a.posture, a.tag) == ("cyl", "red", "stand", 3)
assert (b.shape, b.color, b.posture, b.tag) == ("box", "green", "lie", 7)
r = math.radians(b.rot[2])  # lying: centre stays on the spot, resting on the box top
assert b.rot[1] == 90.0 and abs(b.pos[0] + 0.03 * math.cos(r) - PUCK_XY[1][0]) < 1e-9
assert abs(b.pos[2] - (PUCK_Z + 0.015)) < 1e-9

assert layout(seed=42) == layout(seed=42)  # same seed, same scene
assert {layout(seed=s)[0].tag for s in range(40)} != {layout(seed=0)[0].tag}
for s in range(200):
    x, y = layout(["::lie", ":cyl"], seed=s)
    assert x.posture == "lie" and y.shape == "cyl" and x.tag != y.tag and x.tag in range(12)
assert layout(["red:box:stand:5", "green"], seed=1)[1].tag != 5

for bad in (["red"], ["a:box", "red"], ["red:x", "red"], ["red:box:up", "red"], ["red:box:lie:12", "red"],
            ["red:box:lie:x", "red"], ["red:box:lie:1:2", "red"], ["::lie:3", "::lie:3"], ["a", "b", "c"]):
    try:
        layout(bad)
    except ValueError:
        continue
    raise AssertionError(bad)
for seed in range(500):  # random_layout: box 2-12 cm in front of the chassis, standing pucks on the box top, apart
    t = random_layout(seed)
    assert random_layout(seed) == t and 0.02 <= t.gap <= 0.12
    assert abs(t.box[0] - BOX_HALF - CHASSIS_FRONT - t.gap) < 1e-9
    assert all(p.posture == "stand" for p in t.pucks) and t.pucks[0].tag != t.pucks[1].tag
    assert math.dist(t.pucks[0].pos[:2], t.pucks[1].pos[:2]) >= MIN_SEP - 1e-9
    c, s_ = math.cos(math.radians(-t.box_yaw)), math.sin(math.radians(-t.box_yaw))
    for p in t.pucks:
        dx, dy = p.pos[0] - t.box[0], p.pos[1] - t.box[1]
        assert max(abs(c * dx - s_ * dy), abs(s_ * dx + c * dy)) <= BOX_HALF - MARGIN + 1e-9
    assert t.reachable == [d <= 0.30 for d in t.dist]
print("ok")
