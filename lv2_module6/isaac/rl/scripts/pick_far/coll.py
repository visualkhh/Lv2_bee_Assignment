"""45 deg tilted entry: collision of the open gripper with the puck / start box along (a) the entry slide, (b) the pitch-down sweep,
(c) the roll that aligns a yawed box puck. hppfcl via pinocchio."""
import math, numpy as np, pinocchio as pin, hppfcl
import tilt_reach as T
from tilt_reach import ik, bc, BOX_Z, PAD, PAD_H, rot_at, OK_POS, OK_ANG

gm = pin.buildGeomFromUrdf(ik.model, str(bc.URDF_PATH), pin.GeometryType.COLLISION,
                           package_dirs=[str(bc.URDF_PATH.parent), str(bc.URDF_PATH.parent / "meshes")])
n_robot = len(gm.geometryObjects)
q0 = ik._q(np.zeros(5)); pin.forwardKinematics(ik.model, ik.data, q0); pin.framesForwardKinematics(ik.model, ik.data, q0)
B = ik.data.oMf[ik.base]   # base_link in universe
JL = ik.model.getJointId("gripper_left_joint"); JR = ik.model.getJointId("gripper_right_joint")
iL, iR = ik.model.joints[JL].idx_q, ik.model.joints[JR].idx_q

def place(pos_b, yaw=0.0):
    R = pin.utils.rotate("z", yaw)
    return pin.SE3(B.rotation @ R, B.translation + B.rotation @ np.asarray(pos_b))

def build(x, y, kind, yaw):
    g = pin.GeometryModel(gm)
    geo = hppfcl.Box(0.03, 0.03, 0.06) if kind == "box" else hppfcl.Cylinder(0.015, 0.06)
    if kind == "cyl":   # hppfcl cylinder axis is z
        pass
    puck = pin.GeometryObject("puck", 0, 0, place([x, y, BOX_Z + 0.03], yaw), geo); puck.parentFrame = 0
    bx = pin.GeometryObject("startbox", 0, 0, place([0.1825, 0.0, BOX_Z - 0.035]), hppfcl.Box(0.205, 0.205, 0.07)); bx.parentFrame = 0
    ip, ib = g.addGeometryObject(puck), g.addGeometryObject(bx)
    arm = [i for i in range(n_robot) if not gm.geometryObjects[i].name.startswith("base_link")]
    hand = [i for i in arm if any(k in gm.geometryObjects[i].name for k in ("gripper", "finger"))]
    for i in arm:
        g.addCollisionPair(pin.CollisionPair(i, ip)); g.addCollisionPair(pin.CollisionPair(i, ib))
    return g, ip, ib, arm

def dists(g, q5, gap_mm):
    q = ik._q(q5)
    j = bc.gap_joint(gap_mm); q[iL] = j; q[iR] = j
    d = pin.GeometryData(g)
    pin.forwardKinematics(ik.model, ik.data, q); pin.updateGeometryPlacements(ik.model, ik.data, g, d, q)
    out = {"puck": 9.0, "box": 9.0}
    for k, cp in enumerate(g.collisionPairs):
        r = d.distanceRequests; 
        res = pin.computeDistance(g, d, k)
        key = "puck" if cp.second == len(gm.geometryObjects) - 2 + 0 and False else None
    return d, q

def min_gap(g, ip, ib, q5, gap_mm):
    q = ik._q(q5); j = bc.gap_joint(gap_mm); q[iL] = j; q[iR] = j
    d = pin.GeometryData(g)
    pin.forwardKinematics(ik.model, ik.data, q); pin.updateGeometryPlacements(ik.model, ik.data, g, d, q)
    mp, mb = 9.0, 9.0
    for k, cp in enumerate(g.collisionPairs):
        dist = pin.computeDistance(g, d, k).min_distance
        if cp.second == ip: mp = min(mp, dist)
        else: mb = min(mb, dist)
    return mp, mb

def pose(x, y, alpha, back=0.0, roll=0.0):
    hd = math.atan2(y + 0.00023, x - 0.047985)
    r = np.array([math.cos(hd), math.sin(hd), 0.0]); a = math.cos(alpha) * r + np.array([0, 0, -math.sin(alpha)])
    pos = np.array([x, y, BOX_Z + PAD_H]) - (PAD + back) * a
    R = rot_at(hd, alpha) @ pin.utils.rotate("z", roll)
    return hd, pos, R

def solve(hd, pos, R, seed):
    seeds = [seed] if seed is not None else [np.array([hd, -0.2, 0.7, 1.0, 0.0]), np.array([hd, -0.8, 1.2, 0.8, 0.0]), np.array([hd, 0.3, 0.3, 0.6, 0.0])]
    q, ep, ea = ik.solve(pos, R, seeds)
    return (q if ep <= OK_POS and ea <= OK_ANG else None)

def run(x, y, kind="box", yaw_deg=0.0, dy=0.0, dx=0.0, gap=60.0, a_in=45.0):
    """Puck at (x+dx, y+dy) in truth; hand aims at (x, y) (perception error). Returns dict of minima."""
    g, ip, ib, arm = build(x + dx, y + dy, kind, math.radians(yaw_deg))
    res = {"ok": True, "min_puck_entry": 9.0, "min_puck_sweep": 9.0, "min_puck_roll": 9.0, "min_box": 9.0}
    seed = None
    # (a) slide in at alpha_in from 80 mm back to the grasp pose
    for back in np.arange(0.08, -0.0001, -0.005):
        hd, pos, R = pose(x, y, math.radians(a_in), back)
        q = solve(hd, pos, R, seed)
        if q is None: res["ok"] = False; res["fail"] = "ik entry back=%.0f" % (back * 1000); return res
        seed = q; mp, mb = min_gap(g, ip, ib, q, gap)
        res["min_puck_entry"] = min(res["min_puck_entry"], mp); res["min_box"] = min(res["min_box"], mb)
    # (b) pitch down to vertical about the pad centre, fingers stay open
    for a in np.deg2rad(np.arange(a_in, 90.1, 3)):
        hd, pos, R = pose(x, y, a)
        q = solve(hd, pos, R, seed)
        if q is None: res["ok"] = False; res["fail"] = "ik sweep a=%.0f" % math.degrees(a); return res
        seed = q; mp, mb = min_gap(g, ip, ib, q, gap)
        res["min_puck_sweep"] = min(res["min_puck_sweep"], mp); res["min_box"] = min(res["min_box"], mb)
    # (c) roll about the tool axis to the puck face (box only): roll = -yaw wrapped to +-45 deg
    if kind == "box":
        target = -math.remainder(math.radians(yaw_deg), math.pi / 2)
        for rho in np.linspace(0.0, target, 10):
            hd, pos, R = pose(x, y, math.pi / 2, roll=rho)
            q = solve(hd, pos, R, seed)
            if q is None: res["ok"] = False; res["fail"] = "ik roll %.0f" % math.degrees(rho); return res
            seed = q; mp, mb = min_gap(g, ip, ib, q, gap)
            res["min_puck_roll"] = min(res["min_puck_roll"], mp); res["min_box"] = min(res["min_box"], mb)
    return res

if __name__ == "__main__":
    for kind, yaw in (("box", 0), ("box", 45), ("cyl", 0)):
        r = run(0.26, -0.074, kind, yaw)
        print(kind, yaw, {k: (round(v * 1000, 1) if isinstance(v, float) else v) for k, v in r.items()})
