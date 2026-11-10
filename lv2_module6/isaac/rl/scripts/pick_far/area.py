from pathlib import Path
import math, numpy as np
import tilt_reach as T
from tilt_reach import *
from stand import standoff_ok
T.ALPHAS = np.deg2rad(np.arange(90, 44, -3))
xs = np.arange(0.10, 0.321, 0.005); ys = np.arange(-0.15, 0.1501, 0.005)
V5 = np.zeros((len(xs), len(ys)), bool); V0 = V5.copy(); TL = V5.copy()
for i, x in enumerate(xs):
    for j, y in enumerate(ys):
        V5[i, j] = vertical_ok(x, y, 0.05)
        V0[i, j] = vertical_ok(x, y, 0.0)
        TL[i, j] = V0[i, j] and sweep(x, y)[0] and standoff_ok(x, y, 45, 0.08)
np.savez(Path(__file__).with_name("area.npz"), xs=xs, ys=ys, V5=V5, V0=V0, TL=TL)
cell = 0.005 ** 2
X, Y = np.meshgrid(xs, ys, indexing="ij")
for name, M in (("vertical +5cm (now)", V5), ("vertical grasp pose only", V0), ("tilt 45 entry + vertical", TL)):
    A = M.sum() * cell * 1e4   # cm^2
    r_b = np.hypot(X[M], Y[M]); r_j = np.hypot(X[M] - 0.047985, Y[M])
    print("%-26s area %6.0f cm2 | radius from base_link  %3.0f..%3.0f mm | from joint1 axis %3.0f..%3.0f mm | max x %3.0f mm" %
          (name, A, 1000 * r_b.min(), 1000 * r_b.max(), 1000 * r_j.min(), 1000 * r_j.max(), 1000 * X[M].max()))
print("added by tilt: %.0f cm2 (%.0f %%)" % ((TL & ~V5).sum() * cell * 1e4, 100 * (TL.sum() / V5.sum() - 1)))
print("in tilt but not vertical grasp pose:", (TL & ~V0).sum(), " in vertical+5 but not tilt:", (V5 & ~TL).sum())
# radial band added per heading
for yy in (-0.10, -0.074, 0.0, 0.091):
    j = int(round((yy + 0.15) / 0.005))
    f = lambda M: [1000 * xs[i] for i in range(len(xs)) if M[i, j]]
    print("y %+.3f  vertical+5: %s..%s  tilt: %s..%s" % (yy, *(("%.0f" % min(f(V5)), "%.0f" % max(f(V5))) if f(V5) else ("-", "-")), *(("%.0f" % min(f(TL)), "%.0f" % max(f(TL))) if f(TL) else ("-", "-"))))
