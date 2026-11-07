"""Dice memory mission rules (documents/mission/README.md) as pure functions — a Python copy of the C++ calculator
ros2_ws/src/plan/src/mission_solver.cpp (branch feature/mission-flow), plus a random problem for the simulator.

    python3 mission_rules.py     # self-check against the C++ test cases
"""
import random

PIPS = {1: [4], 2: [0, 8], 3: [4, 0, 8], 4: [0, 2, 6, 8], 5: [4, 0, 2, 6, 8], 6: [0, 3, 6, 2, 5, 8]}  # dice_cells (centre first)
BARCODES = {"P2-01": True, "P2-03": True, "P2-02": False, "P2-04": False}  # parse_barcode: True = ascending


def valid_counts(c):
    return all(1 <= n <= 6 for n in c.values()) and sum(c.values()) == 12


def assign_locations(c, ascending):
    """Groups sorted by count -> L1, L2, L3 (stable: ties keep R, Y, G). Returns e.g. 'YRG'."""
    return "".join(sorted("RYG", key=lambda g: c[g] if ascending else -c[g]))


def allocate(pucks, c):
    """pucks: [(tag, color, shape)] -> {tag: 'R'|'Y'|'G'} or None. Y = the leftover reds / greens, all of one shape."""
    n = lambda col, sh=None: sum(1 for _, pc, ps in pucks if pc == col and (sh is None or ps == sh))
    y_red, y_green = n("red") - c["R"], n("green") - c["G"]
    if y_red < 0 or y_green < 0 or y_red + y_green != c["Y"]:
        return None
    for shape in ("cyl", "box"):
        if n("red", shape) < y_red or n("green", shape) < y_green:
            continue
        left = {"red": y_red, "green": y_green}
        out = {}
        for tag, col, sh in pucks:
            if sh == shape and left[col] > 0:
                out[tag] = "Y"
                left[col] -= 1
            else:
                out[tag] = "R" if col == "red" else "G"
        return out
    return None


def targets(pucks, c, ascending):
    """{tag: (location 0..2, cell 0..8)} — the calculator's answer, cells in dice_cells order per location."""
    groups = allocate(pucks, c)
    if groups is None:
        return None
    order = assign_locations(c, ascending)
    out = {}
    for loc, g in enumerate(order):
        tags = [t for t, _, _ in pucks if groups[t] == g]
        for tag, cell in zip(tags, PIPS[c[g]]):
            out[tag] = (loc, cell)
    return out


def random_problem(pucks, rng=random):
    """Random valid signal counts for this puck set and a random barcode -> (counts, barcode, ascending)."""
    while True:
        r, g = rng.randint(1, 6), rng.randint(1, 6)
        c = {"R": r, "Y": 12 - r - g, "G": g}
        if valid_counts(c) and allocate(pucks, c) is not None:
            code = rng.choice(sorted(BARCODES))
            return c, code, BARCODES[code]


def _check():
    c = {"R": 3, "Y": 4, "G": 5}
    assert assign_locations(c, True) == "RYG" and assign_locations(c, False) == "GYR"
    assert assign_locations({"R": 4, "Y": 4, "G": 4}, False) == "RYG"  # ties keep R, Y, G
    pucks = [(i, "red" if i < 7 else "green", "box" if i % 2 else "cyl") for i in range(12)]
    rng = random.Random(0)
    for _ in range(200):
        c, _, asc = random_problem(pucks, rng)
        t = targets(pucks, c, asc)
        assert len(t) == 12 and len(set(t.values())) == 12
        for loc in range(3):
            assert sorted(cell for l, cell in t.values() if l == loc) == sorted(PIPS[c[assign_locations(c, asc)[loc]]])
    print("ok mission_rules")


if __name__ == "__main__":
    _check()
