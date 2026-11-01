"""mAP50 of a YOLO-OBB model scored like `yolo obb val` (probiou >= 0.5, greedy one-to-one, ap_per_class) but
with perception_node's far switch: puck boxes whose long side < T px become puck_<color>_far on both the
ground-truth and prediction side (far pucks only need color). Also reports far center-recall: a confident
same-color puck centered near the far ground truth (all the node needs there, plus depth).

    env -u LD_LIBRARY_PATH ~/.venv-yolo/bin/python eval_far_switch_ap.py ~/data/bee_holdout \\
        ../../../../ros2_ws/model/yolo/bee_yolo26n_obb_v8.pt
"""
import argparse
import json
import os

import cv2
import numpy as np
import torch
from ultralytics import YOLO
from ultralytics.utils.metrics import ap_per_class, batch_probiou

p = argparse.ArgumentParser()
p.add_argument("dataset")
p.add_argument("weights")
p.add_argument("--split", default="test")
p.add_argument("--far-px", type=int, nargs="+", default=[0, 20, 25, 30])
a = p.parse_args()
D = os.path.expanduser(a.dataset)
m = json.load(open(D + "/manifest.json"))
N = m["classes"]
W, H = m["resolution"]
model = YOLO(os.path.expanduser(a.weights))


def xywhr(poly):
    (cx, cy), (w, h), ang = cv2.minAreaRect(poly.astype(np.float32))
    return [cx, cy, w, h, np.radians(ang)]


imgs = []
for s in [s for s in m["samples"] if s["split"] == a.split]:
    g = []
    for line in open("%s/labels/%s/%05d.txt" % (D, a.split, s["index"])):
        v = line.split()
        g.append((N[int(v[0])], xywhr(np.array(v[1:], float).reshape(4, 2) * [W, H])))
    r = model.predict("%s/images/%s/%05d.png" % (D, a.split, s["index"]), imgsz=640, conf=0.001, verbose=False)[0]
    imgs.append((g, [(N[int(c)], x.tolist(), float(f)) for c, x, f in zip(r.obb.cls, r.obb.xywhr, r.obb.conf)]))


def rename(n, box, T):
    return "puck_%s_far" % n.split("_")[1] if n.startswith("puck") and max(box[2], box[3]) < T else n


def run(T):
    names = sorted({rename(n, b, T) for g, pr in imgs for n, b in g} | {rename(n, b, T) for g, pr in imgs for n, b, _ in pr})
    idx = {n: i for i, n in enumerate(names)}
    TP, CONF, PC, TC = [], [], [], []
    far_hit = far_n = 0
    for g, pr in imgs:
        gc = np.array([idx[rename(n, b, T)] for n, b in g])
        gb = torch.tensor([b for _, b in g], dtype=torch.float32).reshape(-1, 5)
        pc = np.array([idx[rename(n, b, T)] for n, b, _ in pr])
        pb = torch.tensor([b for _, b, _ in pr], dtype=torch.float32).reshape(-1, 5)
        cf = np.array([c for *_, c in pr])
        tp = np.zeros((len(pr), 1), bool)
        if len(g) and len(pr):
            iou = batch_probiou(gb, pb).numpy() * (gc[:, None] == pc[None, :])  # same class only
            pairs = np.array(np.nonzero(iou >= .5)).T
            if len(pairs):  # like ultralytics match_predictions: best iou first, one-to-one
                pairs = pairs[np.argsort(-iou[pairs[:, 0], pairs[:, 1]])]
                pairs = pairs[np.unique(pairs[:, 1], return_index=True)[1]]
                pairs = pairs[np.unique(pairs[:, 0], return_index=True)[1]]
                tp[pairs[:, 1], 0] = True
        for (n, b), c in zip(g, gc):
            if names[c].endswith("_far"):
                far_n += 1
                col = names[c].split("_")[1]
                far_hit += any(nm.startswith("puck_" + col) and f >= .25 and
                               np.hypot(x[0] - b[0], x[1] - b[1]) < max(6, .5 * max(b[2], b[3])) for nm, x, f in pr)
        TP.append(tp)
        CONF.append(cf)
        PC.append(pc)
        TC.append(gc)
    res = ap_per_class(np.concatenate(TP), np.concatenate(CONF), np.concatenate(PC), np.concatenate(TC),
                       names=dict(enumerate(names)))
    return dict(zip([names[i] for i in res[6]], res[5][:, 0])), far_hit / max(far_n, 1), far_n  # res[6] = classes with GT


for T in a.far_px:
    ap, fr, fn = run(T)
    print("T=%2d mAP50 %.3f  <0.98: %s  | far center-recall %.3f (n=%d)" % (
        T, np.mean(list(ap.values())), {k: round(float(v), 3) for k, v in ap.items() if v < .98}, fr, fn))
