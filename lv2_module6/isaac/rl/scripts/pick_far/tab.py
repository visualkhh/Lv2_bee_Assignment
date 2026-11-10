import math, numpy as np
import tilt_reach as T
from tilt_reach import *
import pinocchio as pin
# geometry for min-height check
gm = pin.buildGeomFromUrdf(ik.model, str(bc.URDF_PATH), pin.GeometryType.COLLISION, package_dirs=[str(bc.URDF_PATH.parent), str(bc.URDF_PATH.parent/"meshes")])
gd = pin.GeometryData(gm)
names = [g.name for g in gm.geometryObjects]
hand = [i for i,n in enumerate(names) if any(k in n for k in ("gripper","finger","cam"))]
print("geoms:", len(names), "hand geoms:", [names[i] for i in hand][:12])
verts = {}
for i in hand:
    geo = gm.geometryObjects[i].geometry
    try:
        geo.computeLocalAABB(); bb = geo.aabb_local
        verts[i] = np.array([[x,y,z] for x in (bb.min_[0],bb.max_[0]) for y in (bb.min_[1],bb.max_[1]) for z in (bb.min_[2],bb.max_[2])])
    except Exception as e:
        print("no aabb", names[i], e)
def hand_min_z(q5):
    q = ik._q(q5); pin.forwardKinematics(ik.model, ik.data, q); pin.updateGeometryPlacements(ik.model, ik.data, gm, gd, q)
    b = ik.data.oMf[ik.base]
    zs = []
    for i, v in verts.items():
        M = b.actInv(gd.oMg[i])
        zs.append((M.rotation @ v.T).T[:,2].min() + M.translation[2])
    return min(zs) - BOX_Z   # height of the lowest hand point above the box top

def maxx(y, a_end, xs):
    T.ALPHAS = np.deg2rad(np.arange(90, a_end-1, -3))
    best = None; lowest=None
    for x in xs:
        ok, qs, _ = sweep(x, y)
        if ok:
            best = x; lowest = min(hand_min_z(q) for q in qs)
    return best, lowest
xs = np.arange(0.20, 0.32, 0.0025)
print("max puck x (base_link, mm) where sweep vertical -> entry tilt alpha is feasible; (lowest hand point above box top over the sweep, mm)")
for y in (-0.10,-0.074,0.0,0.091):
    row=[]
    for a in (60,45,30,20):
        b,l = maxx(y,a,xs)
        row.append("a>=%d: %s" % (a, "none" if b is None else "%.0f (low %.0f)" % (b*1000, l*1000)))
    print("y %+.3f | " % y + " | ".join(row))
