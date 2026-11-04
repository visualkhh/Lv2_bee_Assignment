"""Random dice-mission problem for scene_sim.usda (Isaac Sim + full ROS, isaac_sim.sh) — run inside Isaac before Play.

Same draw as the Isaac Lab runner (isaac/lab/mission_run.py, rules in isaac/lab/mission_rules.py):
  - the 12 pucks (colour, shape and tag are fixed per prim, like the real set) shuffled over the 12 start-box spots,
    random yaw; a fraction lie down (BEE_LYING, default 0) with the axis at a random yaw
  - random signal counts R/Y/G valid for this puck set -> a shuffled 12-blink sequence (usda/sign/blink.py plays it)
  - a random barcode P2-01..P2-04 on a board at mission (80, 650, 450) mm facing +X (usda/barcode/<code>.png, Code 128)
All edits go to the session layer (never saved into scene_sim.usda). The answer (counts, barcode, puck -> location/cell)
is written to $BEE_RUN_DIR/problem.json (default ~/bee_twin_runs/<time>/) for scoring the run afterwards.

    BEE_PROBLEM=7 isaac/sim/isaac_sim.sh          # seed 7 (BEE_PROBLEM=random: a new problem each time)
    BEE_LYING=0.2 BEE_PROBLEM=7 isaac/sim/isaac_sim.sh

Comments/strings stay English: Isaac's Script Editor breaks Korean text.
"""
import importlib.util
import json
import math
import os
import random
import time

import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

HERE = os.path.dirname(os.path.abspath(__file__))
USDA = os.path.join(HERE, "..", "usda")
_spec = importlib.util.spec_from_file_location("mission_rules", os.path.join(HERE, "..", "..", "lab", "mission_rules.py"))
mr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mr)

BOX_H = 0.07                   # start box top [m] (bee_common.BOX_H)
PUCK_L, PUCK_W = 0.060, 0.030  # puck length / width (right_env)
# mission (arena, mm, origin at the lower-left wall post) -> scene metres: start_1 (1468, 950) is (-1.253, -2.471)
ARENA_TO_SCENE = (-2.721, -3.421)
BARCODE_ARENA = (80.0, 650.0, 450.0)  # documents/mission/README.md: centre, facing +X
BARCODE_SIZE = (0.30, 0.15)           # [m] assumed A4 landscape print (not in the rules)


def _stage():
    return omni.usd.get_context().get_stage()


def _set_pose(stage, path, pos, quat_xyzw):
    """World pose of a top-level puck prim (its parent /World/pucks has no transform) in the session layer."""
    prim = stage.GetPrimAtPath(path)
    x, y, z, w = quat_xyzw
    rot = Gf.Rotation(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
    ex, ey, ez = rot.Decompose(Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1))
    prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*pos))
    prim.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3f(ex, ey, ez))


def _lying_pose(spot, yaw):
    """isaac/lab/demo_cycle.lying_pose: root (bottom centre) so the puck centre sits on `spot`, axis at world yaw."""
    pos = (spot[0] - PUCK_L / 2 * math.cos(yaw), spot[1] - PUCK_L / 2 * math.sin(yaw), BOX_H + PUCK_W / 2 + 0.0005)
    c, s, h = math.cos(yaw / 2), math.sin(yaw / 2), math.sqrt(0.5)
    return pos, (-s * h, c * h, s * h, c * h)


def _barcode(stage, code):
    """A thin board with the Code 128 texture, centred at the mission barcode point, readable from +X."""
    x = BARCODE_ARENA[0] / 1000 + ARENA_TO_SCENE[0]
    y = BARCODE_ARENA[1] / 1000 + ARENA_TO_SCENE[1]
    z = BARCODE_ARENA[2] / 1000
    w, h = BARCODE_SIZE
    root = "/World/barcode"
    mesh = UsdGeom.Mesh.Define(stage, root + "/board")
    # quad in the YZ plane facing +X; uv so the image reads left->right when seen from +X (looking toward -X)
    mesh.CreatePointsAttr([Gf.Vec3f(x, y + w / 2, z - h / 2), Gf.Vec3f(x, y - w / 2, z - h / 2),
                           Gf.Vec3f(x, y - w / 2, z + h / 2), Gf.Vec3f(x, y + w / 2, z + h / 2)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(1, 0, 0)] * 4)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    mat = UsdShade.Material.Define(stage, root + "/mat")
    pbr = UsdShade.Shader.Define(stage, root + "/mat/pbr")
    pbr.CreateIdAttr("UsdPreviewSurface")
    pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    tex = UsdShade.Shader.Define(stage, root + "/mat/tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(os.path.abspath(os.path.join(USDA, "barcode", code + ".png")))
    reader = UsdShade.Shader.Define(stage, root + "/mat/st")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(pbr.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)


def setup(seed=None, lying=0.0, run_dir=None):
    """Place a random problem; returns the blink sequence (play it with usda/sign/blink.py)."""
    stage = _stage()
    rng = random.Random(seed)
    names = [p.GetName() for p in stage.GetPrimAtPath("/World/pucks").GetChildren()]
    boxes = []
    for b in stage.GetPrimAtPath("/World/start_boxes").GetChildren():
        t = UsdGeom.Xformable(b).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        boxes.append((b.GetName(), t[0], t[1]))
    spots = []
    for n in names:
        t = UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/pucks/{n}")).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation()
        b = min(range(len(boxes)), key=lambda k: math.hypot(t[0] - boxes[k][1], t[1] - boxes[k][2]))
        spots.append((b, t[0], t[1]))
    spots.sort()
    order = names[:]
    rng.shuffle(order)
    placed, lying_names = [], []
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for i, n in enumerate(order):
            b, x, y = spots[i]
            yaw = rng.uniform(-math.pi, math.pi)
            if rng.random() < lying:
                pos, rot = _lying_pose((x, y), yaw)
                lying_names.append(n)
            else:
                pos, rot = (x, y, BOX_H + 0.0005), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
            _set_pose(stage, f"/World/pucks/{n}", pos, rot)
            placed.append((n, boxes[b][0]))
    info = [(int(n.split("_")[-1]), n.split("_")[2], n.split("_")[1]) for n in order]  # (tag, colour, shape)
    counts, code, asc = mr.random_problem(info, rng)
    seq = list("R" * counts["R"] + "Y" * counts["Y"] + "G" * counts["G"])
    rng.shuffle(seq)
    seq = "".join(seq)
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        _barcode(stage, code)
    tgt = mr.targets(info, counts, asc)
    groups = mr.assign_locations(counts, asc)
    answer = {
        "seed": seed, "signal": seq, "counts": counts, "barcode": code, "ascending": asc,
        "locations": {f"L{k + 1}": g for k, g in enumerate(groups)},
        "pucks": [{"prim": n, "tag": t, "colour": c, "shape": s, "start_box": box, "lying": n in lying_names,
                   "location": f"L{tgt[t][0] + 1}", "cell": tgt[t][1]}
                  for (n, box), (t, c, s) in zip(placed, info)],
    }
    run_dir = run_dir or os.path.expanduser(time.strftime("~/bee_twin_runs/%m%d_%H%M%S"))
    os.makedirs(run_dir, exist_ok=True)
    with open(os.path.join(run_dir, "problem.json"), "w") as f:
        json.dump(answer, f, indent=1)
    print(f"[problem] seed {seed}: signal {seq} (R {counts['R']} Y {counts['Y']} G {counts['G']}), barcode {code} "
          f"({'ascending' if asc else 'descending'}) -> L1 {groups[0]} L2 {groups[1]} L3 {groups[2]}; "
          f"lying {lying_names or 'none'} -> {run_dir}/problem.json")
    return seq


def play_signal(seq):
    """Start usda/sign/blink.py with this sequence (it waits for Play and loops)."""
    path = os.path.join(USDA, "sign", "blink.py")
    src = open(path).read().replace("SEQUENCE = None", f'SEQUENCE = "{seq}"', 1)
    exec(compile(src, path, "exec"), {"__name__": "__main__", "__file__": path})
