"""Per-puck failure breakdown of a YOLO-OBB model on a gen_yolo_dataset.py set: every ground-truth puck is
ok / miss / wrong color / wrong shape / wrong posture, tabled by size, view, frame kind, lighting, noise,
edge cut, posture and class (conf 0.25, nearest predicted center within 0.4 x the puck's long side).
--far-px 30 scores like perception_node's far switch (fine_min_px): a puck shorter than that only needs
its color, so shape/posture mistakes there count as ok.

    env -u LD_LIBRARY_PATH ~/.venv-yolo/bin/python eval_failures.py ~/data/bee_holdout \\
        ../../../../ros2_ws/model/yolo/bee_yolo26n_obb_v8.pt --far-px 30
"""
import argparse
import collections
import json
import os

import numpy as np
from ultralytics import YOLO

p = argparse.ArgumentParser()
p.add_argument("dataset")
p.add_argument("weights")
p.add_argument("--split", default="test")
p.add_argument("--far-px", type=float, default=0, help="node far switch: below this long side only color counts (0 = off)")
a = p.parse_args()
D = os.path.expanduser(a.dataset)
m = json.load(open(D + "/manifest.json"))
N = m["classes"]
W, H = m["resolution"]
model = YOLO(os.path.expanduser(a.weights))
rows = []
for s in m["samples"]:
    if s["split"] != a.split or s["negative"]:
        continue
    g = []
    for line in open("%s/labels/%s/%05d.txt" % (D, a.split, s["index"])):
        v = line.split()
        n = N[int(v[0])]
        if not n.startswith("puck"):
            continue
        q = np.array(v[1:], float).reshape(4, 2) * [W, H]
        L = max(np.linalg.norm(q[1] - q[0]), np.linalg.norm(q[2] - q[1]))
        edge = q.min() <= 2 or q[:, 0].max() >= W - 3 or q[:, 1].max() >= H - 3
        g.append((n, q.mean(0), L, edge))
    if not g:
        continue
    r = model.predict("%s/images/%s/%05d.png" % (D, a.split, s["index"]), imgsz=640, conf=.25, verbose=False)[0]
    pr = [(N[int(c)], np.array(x).reshape(4, 2).mean(0)) for c, x in zip(r.obb.cls.tolist(), r.obb.xyxyxyxy.tolist())]
    used = set()
    for n, c, L, edge in sorted(g, key=lambda t: -t[2]):  # big pucks claim their prediction first
        cand = [(np.linalg.norm(pc - c), k) for k, (pn, pc) in enumerate(pr) if pn.startswith("puck") and k not in used]
        best = min(cand, default=None)
        if best and best[0] < max(6, .4 * L):
            used.add(best[1])
            pn = pr[best[1]][0]
            x, y = n.split("_"), pn.split("_")
            res = "ok" if pn == n else "color" if x[1] != y[1] else "shape" if x[2] != y[2] else "posture"
            if L < a.far_px and res in ("shape", "posture"):
                res = "ok"  # far puck: the node reports color + position only
        else:
            res = "miss"
        size = "<15" if L < 15 else "15-25" if L < 25 else "25-40" if L < 40 else "40-80" if L < 80 else ">80"
        rows.append(dict(res=res, cls=n.replace("puck_", ""), kind=s.get("kind", "normal"), view=s["view"],
                         light=s["lighting"], noise="noisy" if s["noise"] else "clean", size=size,
                         edge="edge-cut" if edge else "inside", posture="lying" if n.endswith("lying") else "stand"))

R = ["ok", "miss", "color", "shape", "posture"]


def table(key, order=None):
    c = collections.defaultdict(collections.Counter)
    for r in rows:
        c[r[key]][r["res"]] += 1
    print("\n## by", key)
    print("%-16s %5s %6s " % ("", "n", "ok%") + " ".join("%7s" % x for x in R[1:]))
    for k in order or sorted(c):
        if k in c:
            t = sum(c[k].values())
            print("%-16s %5d %5.1f%% " % (k, t, 100 * c[k]["ok"] / t) + " ".join("%7d" % c[k][x] for x in R[1:]))


print("total", len(rows), dict(collections.Counter(r["res"] for r in rows)))
for k, o in [("size", ["<15", "15-25", "25-40", "40-80", ">80"]), ("view", None), ("kind", None),
             ("light", ["normal", "dim", "dark"]), ("noise", None), ("edge", None), ("posture", None), ("cls", None)]:
    table(k, o)
