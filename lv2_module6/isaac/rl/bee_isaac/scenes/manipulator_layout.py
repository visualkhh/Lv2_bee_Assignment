"""Manipulator demo pucks from CLI text (manipulator_env.py --puck, --seed). Standard library only: parsed before the Isaac app starts."""

import math
import random
from collections import namedtuple

from bee_isaac.constants import PUCK_L, PUCK_W, PUCK_XY, PUCK_Z

COLORS, SHAPES, POSTURES = ("red", "green"), ("box", "cyl"), ("stand", "lie")
TAGS = range(12)  # source/texture/puck/puck_*_basecolor_id<N>.png
ASSETS = {
    ("box", "green"): "puck/box/puck_box_green_0.usda",
    ("box", "red"): "puck/box/puck_box_red_1.usda",
    ("cyl", "green"): "puck/cyl/puck_cyl_green_2.usda",
    ("cyl", "red"): "puck/cyl/puck_cyl_red_3.usda",
}
DEFAULT = ["red:box:stand:0", "green:box:stand:6"]  # numbers and yaw as scene_sim start_1
DEFAULT_YAW = [165.7, 352.9]

Puck = namedtuple("Puck", "name asset shape color posture tag pos rot")


def parse_puck(text):
    """'color:shape:posture:tag' -> 4 fields, None where left empty or cut short."""
    fields = text.split(":")
    if len(fields) > 4:
        raise ValueError(f"--puck {text}: 색:모양:상태:태그 까지만")
    fields = [f or None for f in fields] + [None] * (4 - len(fields))
    for value, allowed in zip(fields, (COLORS, SHAPES, POSTURES)):
        if value is not None and value not in allowed:
            raise ValueError(f"--puck {text}: {value} 없음 ({'|'.join(allowed)})")
    if fields[3] is not None:
        if not fields[3].isdigit() or int(fields[3]) not in TAGS:
            raise ValueError(f"--puck {text}: 태그는 0~{TAGS[-1]}")
        fields[3] = int(fields[3])
    return fields


def layout(specs=None, seed=None):
    """Two pucks on the fixed spots PUCK_XY (pick_load_play.py and plan.yaml puck_offsets_x assume them)."""
    if specs is None:
        specs = DEFAULT if seed is None else ["", ""]
    if len(specs) != len(PUCK_XY):
        raise ValueError(f"--puck은 {len(PUCK_XY)}번 (지금 {len(specs)}번) — 상자 위 자리가 둘")
    parsed = [parse_puck(s) for s in specs]
    fixed = [p[3] for p in parsed if p[3] is not None]
    if len(fixed) != len(set(fixed)):
        raise ValueError(f"--puck 태그 중복: {fixed}")
    rng = random.Random(seed)
    free_tags = [t for t in TAGS if t not in fixed]
    rng.shuffle(free_tags)
    pucks = []
    for k, (color, shape, posture, tag) in enumerate(parsed):
        color = color or rng.choice(COLORS)
        shape = shape or rng.choice(SHAPES)
        posture = posture or rng.choice(POSTURES)
        tag = free_tags.pop() if tag is None else tag
        yaw = DEFAULT_YAW[k] if seed is None else rng.uniform(0.0, 360.0)
        x, y = PUCK_XY[k]
        if posture == "stand":
            pos, rot = (x, y, PUCK_Z), (0.0, 0.0, yaw)
        else:  # axis along yaw, centre on the spot; the origin is the bottom end, half a length back
            r = math.radians(yaw)
            pos = (x - PUCK_L / 2 * math.cos(r), y - PUCK_L / 2 * math.sin(r), PUCK_Z + PUCK_W / 2)
            rot = (0.0, 90.0, yaw)
        pucks.append(Puck(f"puck_{shape}_{color}_{tag}", ASSETS[shape, color], shape, color, posture, tag, pos, rot))
    return pucks
