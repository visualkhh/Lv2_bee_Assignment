"""AprilTag tag36h11 read rate on a gen_yolo_dataset.py set with perception_node's detector settings
(OpenCV aruco, minMarkerPerimeterRate 0.01): per puck tag by distance, lighting, noise and view, and for the
L1-L3 top tags (ids 12-14). Expected = the tag face points within 60 deg of the camera and the object
projects into the image; occlusion is not known, so the rates are lower bounds. Ids are unique per frame,
so a read counts by id (manifest pos is the prim origin, tens of px off the tag).

    env -u LD_LIBRARY_PATH ~/.venv-yolo/bin/python eval_tags.py ~/data/bee_holdout
"""
import argparse
import collections
import json
import os

import cv2
import numpy as np

BOX_TOP = 0.07  # start/shelf box height [m] (207 x 207 x 70 mm), sim top tag at its center
SHELF_ID = {"shelf_L1": 12, "shelf_L2": 13, "shelf_L3": 14}

p = argparse.ArgumentParser()
p.add_argument("dataset")
p.add_argument("--split", default="test")
a = p.parse_args()
D = os.path.expanduser(a.dataset)
m = json.load(open(D + "/manifest.json"))
K = np.array(m["K"])
W, H = m["resolution"]
params = cv2.aruco.DetectorParameters()
params.minMarkerPerimeterRate = 0.01
det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), params)
shelves = {b["name"]: np.array(b["center"]) + [0, 0, BOX_TOP] for b in m["boxes"] if b["name"] in SHELF_ID}


def proj(M, pt):
    c = (pt - M[3, :3]) @ M[:3, :3].T  # USD camera (row-vector matrix): x right, y up, looks -z
    if c[2] > -0.05:
        return None
    u, v = K[0, 0] * c[0] / -c[2] + K[0, 2], K[1, 2] - K[1, 1] * c[1] / -c[2]
    return (u, v) if 0 <= u < W and 0 <= v < H else None


stat = collections.defaultdict(lambda: [0, 0])
unexpected = ndet = 0


def add(key, hit):
    stat[key][0] += hit
    stat[key][1] += 1


for s in [s for s in m["samples"] if s["split"] == a.split and not s["negative"]]:
    M = np.array(s["cam_to_world"])
    cam = M[3, :3]
    img = cv2.imread("%s/images/%s/%05d.png" % (D, a.split, s["index"]))
    _, ids, _ = det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
    found = set() if ids is None else {int(i) for i in ids.ravel()}
    ndet += len(found)
    expected = set()
    for pk in s["pucks"]:
        pos, axis = np.array(pk["pos"]), np.array(pk["axis"])
        if proj(M, pos) is None:
            continue
        expected.add(pk["tag"])
        d = cam - pos
        dist = np.linalg.norm(d)
        cos = max(d @ axis, -(d @ axis) if pk["shape"] == "cyl" else -1) / dist  # cyl: tags on both ends, box: top only
        if cos < 0.5:
            continue
        kind = "puck_" + ("basket" if pk["basket"] else "lying" if pk["lying"] else "stand")
        hit = pk["tag"] in found
        for key in [(kind, "dist", "%.1f" % min(np.floor(dist / .3) * .3, 2.1)), (kind, "light", s["lighting"]),
                    (kind, "noise", s["noise"] is not None), (kind, "view", s["view"]), (kind, "all", "")]:
            add(key, hit)
    for name, top in shelves.items():
        d = cam - top
        dist = np.linalg.norm(d)
        if proj(M, top) is None or d[2] / dist < 0.5:  # top tag needs a view from above
            continue
        expected.add(SHELF_ID[name])
        hit = SHELF_ID[name] in found
        for key in [("shelf", "dist", "%.1f" % min(np.floor(dist / .5) * .5, 2.5)), ("shelf", "light", s["lighting"]),
                    ("shelf", "all", "")]:
            add(key, hit)
    unexpected += len(found - expected)
for k in sorted(stat):
    print("%-12s %-6s %-8s %4d/%-4d %.3f" % (*map(str, k), *stat[k], stat[k][0] / stat[k][1]))
print("detections", ndet, "ids not expected in frame", unexpected)
