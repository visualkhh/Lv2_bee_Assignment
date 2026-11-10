"""Max puck distance for 'enter horizontal, pitch hand down about the pad centre, then grasp vertical'. Offline, pinocchio."""
import math, sys
import numpy as np
import pinocchio as pin
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # isaac/rl
from bee_isaac import common as bc

ik = bc.ArmIK()
base = ik.base_in_root()
BOX_Z = bc.BOX_H - base[2]            # box top in base frame
PAD = 0.0106                          # ee is 10.6 mm behind the pad centre along the tool axis
PAD_H = 0.05                          # pad centre height above the puck bottom when grasped (tips at 40..60 mm)
GRASP_EE = bc.GRASP_Z                 # ee above puck bottom, vertical grasp
ALPHAS = np.deg2rad(np.arange(90, -1, -5))  # 90 = vertical, 0 = horizontal
OK_POS, OK_ANG = 2e-3, math.radians(2.0)


def rot_at(hd, alpha):
    beta = math.pi / 2 + alpha
    return pin.utils.rotate("z", hd) @ pin.utils.rotate("y", beta)


def sweep(x, y, standoff=0.0, verbose=False):
    """Follow alpha 90 -> 0 about the pad centre; returns (ok, qs). standoff: extra pull back along the tool at alpha 0 only."""
    hd = math.atan2(y + 0.00023, x - 0.047985)
    P = np.array([x, y, BOX_Z + PAD_H])
    r_hat = np.array([math.cos(hd), math.sin(hd), 0.0])
    seeds = [np.array([hd, -0.2, 0.7, 1.0, 0.0]), np.array([hd, -0.8, 1.2, 0.8, 0.0]), np.array([hd, 0.3, 0.3, 0.6, 0.0])]
    qs = []
    for a in ALPHAS:
        axis = math.cos(a) * r_hat + np.array([0, 0, -math.sin(a)])
        pos = P - PAD * axis
        q, ep, ea = ik.solve(pos, rot_at(hd, a), seeds)
        if ep > OK_POS or ea > OK_ANG:
            return False, qs, a
        if qs and np.abs(q - qs[-1]).max() > 0.5:   # branch jump
            return False, qs, a
        qs.append(q); seeds = [q]
    return True, qs, None


def vertical_ok(x, y, above):
    hd = math.atan2(y + 0.00023, x - 0.047985)
    pos = np.array([x, y, BOX_Z + GRASP_EE + above])
    seeds = [np.array([hd, -0.2, 0.7, 1.0, 0.0]), np.array([hd, -0.8, 1.2, 0.8, 0.0]), np.array([hd, 0.3, 0.3, 0.6, 0.0])]
    q, ep, ea = ik.solve(pos, pin.utils.rotate("z", hd) @ bc.DOWN, seeds)
    return ep <= OK_POS and ea <= OK_ANG


if __name__ == "__main__":
    xs = np.arange(0.15, 0.40, 0.005)
    print("base_z of box top in base frame: %.3f" % BOX_Z)
    for y in (-0.10, -0.074, 0.0, 0.091):
        mx = {}
        for name, f in (("vert +0", lambda x: vertical_ok(x, y, 0.0)), ("vert +3cm", lambda x: vertical_ok(x, y, 0.03)),
                        ("vert +5cm", lambda x: vertical_ok(x, y, 0.05)), ("tilt sweep", lambda x: sweep(x, y)[0])):
            ok = [x for x in xs if f(x)]
            mx[name] = max(ok) if ok else float("nan")
        print("y %+.3f | " % y + " | ".join("%s %.0f mm" % (k, v * 1000) for k, v in mx.items()))
