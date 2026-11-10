import sys
from pathlib import Path
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
d = np.load(Path(__file__).with_name("area.npz")); xs, ys, V5, V0, TL = d["xs"], d["ys"], d["V5"], d["V0"], d["TL"]
fig, ax = plt.subplots(1, 2, figsize=(11, 4.6), gridspec_kw={"width_ratios": [1.15, 1]})
X, Y = np.meshgrid(xs * 1000, ys * 1000, indexing="ij")
a = ax[0]
a.scatter(X[V0 & ~V5 & ~TL], Y[V0 & ~V5 & ~TL], s=9, c="#cccccc", label="vertical grasp pose only")
a.scatter(X[V5], Y[V5], s=9, c="#4c78a8", label="vertical +5 cm (now): 448 cm2")
a.scatter(X[TL & ~V5], Y[TL & ~V5], s=9, c="#e45756", label="added by 45° tilt entry: +85 cm2")
a.add_patch(Rectangle((80, -102.5), 205, 205, fill=False, ec="k", ls="--", lw=1))
a.text(84, 95, "start box (80..285 mm)", fontsize=8)
a.plot(0, 0, "k^"); a.text(4, 4, "base_link", fontsize=8)
a.set_xlabel("x forward from base_link [mm]"); a.set_ylabel("y [mm]"); a.set_aspect("equal"); a.legend(fontsize=7, loc="lower left")
a.set_title("Reachable puck positions (5 mm grid, kinematic)")
b = ax[1]
ysel = (-0.10, -0.074, 0.0, 0.091); lab = ["vert +5cm", "vert +3cm", "tilt 45° + vert", "vert grasp only"]
vals = {"vert +5cm": [235, 250, 260, 240], "vert +3cm": [250, 265, 275, 255], "tilt 45° + vert": [268, 278, 288, 270], "vert grasp only": [270, 280, 290, 270]}
w = 0.2; x = np.arange(4)
for i, k in enumerate(lab):
    b.bar(x + (i - 1.5) * w, vals[k], w, label=k, color=["#4c78a8", "#9ecae9", "#e45756", "#cccccc"][i])
    for xi, v in zip(x + (i - 1.5) * w, vals[k]): b.text(xi, v + 1, str(v), ha="center", fontsize=6.5)
b.set_xticks(x); b.set_xticklabels(["y -100", "y -74", "y 0", "y +91"]); b.set_ylim(200, 300)
b.set_ylabel("max reachable x [mm]"); b.set_title("Max x by lateral position"); b.legend(fontsize=7, loc="upper left")
plt.tight_layout(); plt.savefig(sys.argv[1] if len(sys.argv) > 1 else "pick_far_reach.png", dpi=160)
