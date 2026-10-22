#!/usr/bin/env python3
"""beebot.urdf -> robot.json (box-link schema: name/size_m/mass_kg, joints with xyz_m/rpy_rad/axis/limit).

Lossy by design of the schema:
- each link becomes one box: size from the first collision geometry (box as-is,
  cylinder as its bounding box, mesh as its STL bounding box); geometry offsets are dropped.
- inertia tensors, link COM, mimic (gripper_right follows gripper_left) and ros2_control are dropped.
- massless frame links (base_footprint, optical frames, imu, end_effector, basket slots) and their
  fixed joints are dropped, so base_link becomes the root.
"""
import json
import os
import sys
import xml.etree.ElementTree as ET

from stlio import read_stl

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.dirname(HERE)


def vec(s, default="0 0 0"):
    return [float(v) for v in (s or default).split()]


def box_size(link):
    g = link.find("collision/geometry")[0]
    if g.tag == "box":
        return vec(g.get("size"))
    if g.tag == "cylinder":
        r, h = float(g.get("radius")), float(g.get("length"))
        return [2 * r, 2 * r, h]
    if g.tag == "sphere":
        return [2 * float(g.get("radius"))] * 3
    v = read_stl(os.path.join(MODEL, g.get("filename"))).reshape(-1, 3) * vec(g.get("scale"), "1 1 1")
    return [round(float(d), 6) for d in v.max(0) - v.min(0)]


def convert(urdf):
    root = ET.parse(urdf).getroot()
    links = []
    for l in root.findall("link"):
        m = l.find("inertial/mass")
        if m is None or l.find("collision") is None:
            continue
        links.append({"name": l.get("name"), "size_m": box_size(l), "mass_kg": float(m.get("value"))})
    kept = {l["name"] for l in links}

    joints = []
    for j in root.findall("joint"):
        parent, child = j.find("parent").get("link"), j.find("child").get("link")
        if parent not in kept or child not in kept:
            continue
        o = j.find("origin")
        d = {
            "name": j.get("name"),
            "type": j.get("type"),
            "parent": parent,
            "child": child,
            "xyz_m": vec(o.get("xyz") if o is not None else None),
            "rpy_rad": vec(o.get("rpy") if o is not None else None),
        }
        if d["type"] != "fixed":
            d["axis"] = vec(j.find("axis").get("xyz") if j.find("axis") is not None else None, "1 0 0")
            lim = j.find("limit")
            keys = ("effort", "velocity") if d["type"] == "continuous" else ("lower", "upper", "effort", "velocity")
            if lim is not None:
                d["limit"] = {k: float(lim.get(k)) for k in keys if lim.get(k) is not None}
        joints.append(d)

    return {"name": root.get("name"), "units": "SI", "links": links, "joints": joints}


if __name__ == "__main__":
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(MODEL, "beebot.urdf")
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(MODEL, "robot.json")
    data = convert(src)
    names = {l["name"] for l in data["links"]}
    children = [j["child"] for j in data["joints"]]
    assert all(j["parent"] in names for j in data["joints"])
    assert len(set(children)) == len(children) == len(names) - 1, "link tree is not connected"
    with open(out, "w") as f:
        json.dump(data, f, indent=2)
    print(f"{out}: {len(data['links'])} links, {len(data['joints'])} joints, "
          f"{sum(l['mass_kg'] for l in data['links']):.4f} kg")
