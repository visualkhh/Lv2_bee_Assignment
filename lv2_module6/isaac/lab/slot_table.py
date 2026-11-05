"""Basket slot table: load / unload joint poses for the 12 slots, solved once offline (no Isaac app).

    ~/pa-course/IsaacLab-6.1/.venv/bin/python slot_table.py      -> slot_table.npz (demo_cycle.py), slot_table.yaml (ROS)

The basket is fixed on the robot, so carry -> slot -> carry is the same every time. Per puck kind (box: tool yaw on the
slot walls; cylinder: any yaw) and slot:
  load    q_over -> q_low (puck bottom 3 mm over the slot floor) -> release -> q_lift -> stow, from the carry pose
  unload  q_over -> q_grasp -> close -> q_over -> carry
Load poses are solved for the nominal puck in the hand (bottom GRASP_Z under the end effector); P_* (5 x 3) turn a
measured offset e of the puck bottom in the end effector frame into the joint step  q = q_nom + P @ (-R @ e),
R = tool rotation (rot_* in the table): the hand moves the other way so the puck still lands on the slot centre.
The tilt of each slot is fixed here (the demo used to pick the first IK-feasible tilt per run, so a few mm of hand
offset could flip it). All poses: joint1..5 [rad], targets in the robot root frame as demo_cycle.py always used.
Check printed per slot: IK error, and the held puck's clearance to the basket walls along the joint-interpolated moves.
"""

import glob
import math
from pathlib import Path

import numpy as np
import pinocchio as pin
import trimesh
import yaml

import bee_common as bc

HERE = Path(__file__).resolve().parent
# demo_cycle.py geometry (not imported: it starts the Isaac app)
SLOTS = [(x, y, 0.09125) for x in (-0.16984, -0.10484, -0.03984) for y in (-0.0975, -0.0325, 0.0325, 0.0975)]
JOINT1_XY = (0.047985, -0.00023)
OVER = {0: (0.05, -20.0), 1: (0.045, 0.0), 2: (0.06, 0.0)}  # per row: over-slot height of the puck bottom, pitch
SET_TILTS = (0.0, -5.0, -10.0, -15.0)
LIFT_UPS = (0.06, 0.05, 0.045, 0.04, 0.035)
WRIST = math.pi / 2
STOW = np.array([0.0, -1.9024, 1.344, 0.9023, 0.0])
CARRY = {1: np.array([math.pi / 2, -0.465027, 0.102271, 1.933412, math.pi / 2]),
         -1: np.array([-math.pi / 2, -0.465027, 0.102271, 1.933412, -math.pi / 2])}
D_NOM = np.array([0.0, 0.0, bc.GRASP_Z])  # puck bottom in the end effector frame, nominal grasp
POLE_SIDE = (1, 2)  # unload grasps away from the pole: box 8 mm, cylinder 5 mm (slot 1 test, 96 runs per kind: 100 %;
# cylinder at 8 mm 85 %, box at 5 mm 80 %)
PUCK_W, PUCK_H = 0.030, bc.PUCK_H
TOL_P, TOL_A = 2e-3, math.radians(2.0)


def rot_tool(heading, pitch_deg, yaw):
    return pin.utils.rotate("z", heading) @ pin.utils.rotate("y", math.radians(pitch_deg)) @ bc.DOWN @ pin.utils.rotate("z", -(yaw - heading))


def wall_yaw(yaw):
    return yaw + math.remainder(-yaw, math.pi / 2)


ik = bc.ArmIK()
parts = []
for f in sorted(glob.glob(str(bc.URDF_PATH.parent / "meshes" / "storage_collision" / "*.stl"))):
    m = trimesh.load(f)
    m.apply_scale(0.001)
    tf = trimesh.transformations.euler_matrix(1.570796, 0, 0)
    tf[:3, 3] = [-0.064, 0.0, -0.0005]  # beebot.urdf storage_collision origin (base_link frame)
    m.apply_transform(tf)
    parts.append(m.convex_hull)
basket = trimesh.util.concatenate(parts)
g = np.linspace(-PUCK_W / 2, PUCK_W / 2, 4)
zs = np.linspace(0, -PUCK_H, 7)  # from the bottom up (tool z points down)
PUCK_PTS = {  # surface points in the end effector frame, around the puck bottom
    True: np.array([[x, y, z] for x in g for y in g for z in zs if abs(x) == PUCK_W / 2 or abs(y) == PUCK_W / 2 or z in (0, -PUCK_H)]),
    False: np.array([[PUCK_W / 2 * math.cos(a), PUCK_W / 2 * math.sin(a), z] for a in np.linspace(0, 2 * math.pi, 16, endpoint=False) for z in zs]),
}


def clearance(moves, is_box, d=D_NOM):
    """Min distance [m] of the held puck to the basket along joint-interpolated moves (negative = in a wall)."""
    worst = np.inf
    for qa, qb in moves:
        for s in np.linspace(0, 1, 30):
            p, r = ik.fk(qa + s * (qb - qa))
            pts = (p + r @ d) + PUCK_PTS[is_box] @ r.T
            dist = trimesh.proximity.closest_point(basket, pts)[1].min()
            worst = min(worst, -dist if any(c.contains(pts).any() for c in parts) else dist)
    return worst


def solve(pos, rot, seeds):
    q, e_p, e_a = ik.solve(pos, rot, seeds)
    return q, e_p < TOL_P and e_a < TOL_A


def slot_rows(slot, is_box):
    sx, sy, sz = SLOTS[slot]
    hd = math.atan2(sy - JOINT1_XY[1], sx - JOINT1_XY[0])
    yaw = wall_yaw(hd + (-WRIST if sy > 0 else WRIST)) if (is_box or slot in POLE_SIDE) else hd + (-WRIST if sy > 0 else WRIST)
    h_over, pitch = OVER[slot // 4]
    seeds = [np.array([hd, j2, j3, j4, hd - yaw]) for j2 in (-0.3, 0.3, 0.8) for j3 in (0.7, -0.5) for j4 in (1.0, 1.9)]
    r_over = rot_tool(hd, pitch, yaw)
    q_over, ok = solve(np.array([sx, sy, sz + h_over]) - r_over @ D_NOM, r_over, seeds)
    assert ok, f"slot {slot}: over pose"
    for tilt in SET_TILTS:
        r_low = rot_tool(hd, tilt, yaw)
        q_low, ok = solve(np.array([sx, sy, sz + 0.003]) - r_low @ D_NOM, r_low, [q_over])
        if ok:
            break
    assert ok, f"slot {slot}: low pose"
    q_lift, lift_up = q_low, 0.0
    for up in LIFT_UPS:
        q_try, ok = solve(np.array([sx, sy, sz + up]) - r_low @ D_NOM, r_low, [q_low])
        if ok:
            q_lift, lift_up = q_try, up
            break
    gy = sy + (math.copysign(0.008 if is_box else 0.005, sy) if slot in POLE_SIDE else 0.0)
    for g_tilt in SET_TILTS:
        r_g = rot_tool(hd, g_tilt, yaw)
        q_grasp, ok = solve(np.array([sx, gy, sz]) - r_g @ D_NOM, r_g, [q_over])
        if ok:
            break
    assert ok, f"slot {slot}: unload grasp pose"
    carry = CARRY[1 if sy >= 0 else -1]
    return {
        "carry": carry, "q_over": q_over, "q_low": q_low, "q_lift": q_lift, "q_grasp": q_grasp,
        "P_over": ik.jac_pos(q_over), "P_low": ik.jac_pos(q_low), "P_lift": ik.jac_pos(q_lift),
        "rot_over": r_over, "rot_low": r_low,
        "tilt_deg": tilt, "lift_up_m": lift_up, "grasp_tilt_deg": g_tilt,
        "clear_load": clearance([(carry, q_over), (q_over, q_low)], is_box),
        "clear_load_deep": clearance([(carry, q_over), (q_over, q_low)], is_box, D_NOM - [0, 0, 0.006]),  # pick holds ~6 mm deeper
        # unload: the puck stays on the slot centre, the fingers close 8 mm off it on the pole side
        "clear_unload": clearance([(q_grasp, q_over), (q_over, carry)], is_box, D_NOM + r_g.T @ np.array([0.0, sy - gy, 0.0])),
    }


table = {kind: [slot_rows(s, kind == "box") for s in range(12)] for kind in ("box", "cyl")}

print("kind slot | load tilt, lift | unload tilt | held puck to basket min [mm]: load, load (6 mm deeper), unload")
for kind, rows in table.items():
    for s, r in enumerate(rows):
        print(f"{kind} {s:2d}   | {r['tilt_deg']:+5.0f} deg, {1000 * r['lift_up_m']:3.0f} mm | {r['grasp_tilt_deg']:+5.0f} deg | "
              f"{1000 * r['clear_load']:+5.1f}, {1000 * r['clear_load_deep']:+5.1f}, {1000 * r['clear_unload']:+5.1f}")

keys = [k for k in table["box"][0] if not k.startswith("clear")]
np.savez(HERE / "slot_table.npz", d_nom=D_NOM, stow=STOW,
         **{f"{kind}_{k}": np.array([r[k] for r in rows]) for kind, rows in table.items() for k in keys})
doc = {
    "description": "BEE basket slot table from isaac/lab/slot_table.py. Joint values joint1..5 [rad]. Load: carry -> "
                   "q_over -> q_low -> open -> q_lift -> stow. Unload: carry -> q_over -> q_grasp -> close -> q_over -> "
                   "carry. Box pucks use 'box', cylinders 'cyl'. Load poses assume the puck bottom at d_nom in the end "
                   "effector frame; for a measured offset e: q = q_nom + P @ (-rot @ e).",
    "d_nom_m": D_NOM.tolist(), "stow": STOW.tolist(), "carry": {"left": CARRY[1].tolist(), "right": CARRY[-1].tolist()},
    "slots": {kind: [{"slot": s, **{k: np.round(np.asarray(r[k]), 6).tolist() for k in keys if k != "carry"},
                      "carry": "left" if SLOTS[s][1] >= 0 else "right"} for s, r in enumerate(rows)]
              for kind, rows in table.items()},
}
(HERE / "slot_table.yaml").write_text(yaml.safe_dump(doc, sort_keys=False, default_flow_style=None, width=160))
print(f"wrote {HERE / 'slot_table.npz'} and {HERE / 'slot_table.yaml'}")
