"""Whole dice memory mission in Isaac on isaac/sim/usda/scene_lab.usda: random problem -> calculator -> planner -> skills.

    python mission_run.py --device cpu --viz newton                 # one random problem, learned planner (plan_next)
    python mission_run.py --device cpu --planner solver --problem_seed 3

Each run draws a random problem the way the mission does: the 12 pucks (colour, shape, tag) shuffled over the 12 start
box spots with random yaw, some lying (--lying), random signal counts R / Y / G (valid for this puck set) and a random
barcode. The calculator (mission_rules.py = mission_solver.cpp) turns that into a target (location, dice cell) per puck.
Then the planner picks the next skill from the current state again and again, with the action mask of
learning/plan/mission_sim.py:
  model  : plan_next (learned, policies/plan_next.pt = TorchScript of the MaskablePPO policy)
  solver : learning/plan/solver.py (shortest order, recomputed every step)
  rule   : learning/plan/baseline.py (the mission_manager order)
Skills are the demo_cycle.py ones (scripted base motion + residual policies where trained): pick, stand up, load into
a basket slot, unload, place on a dice cell, re-pick. The outcome of each skill is judged from the simulation state and
fed back to the planner. Driving: grid path over the maze walls and boxes (A*, inflated), then demo drive_to (P control
on the true base pose), docking on the box face that suits the skill (far dice rows from the opposite face).
Stand-ins: perception = true poses + training noise, the signal / barcode are given, no Nav2, no SLAM.
Not available as skills yet (masked): nudge_cell, stand_puck_in_cell.
"""

import argparse
import heapq
import math
import random
import sys
from pathlib import Path

ap = argparse.ArgumentParser(add_help=False)
ap.add_argument("--problem_seed", type=int, default=-1, help="random problem seed (-1 = random)")
ap.add_argument("--planner", choices=["model", "solver", "rule"], default="model")
ap.add_argument("--plan_model", default=None, help="TorchScript planner (default policies/plan_next.pt)")
ap.add_argument("--lying", type=float, default=0.15, help="probability that a start puck lies on its side")
ap.add_argument("--max_steps", type=int, default=200)
ours, rest = ap.parse_known_args()
sys.argv = [sys.argv[0], *rest]
import demo_cycle as dc  # noqa: E402  (parses the remaining args, starts the Isaac app)

import numpy as np  # noqa: E402
import torch  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
import isaaclab.utils.math as mu  # noqa: E402
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab.visualizers import VisualizerCfg  # noqa: E402
from isaaclab_physx.physics import PhysxCfg  # noqa: E402
import warp as wp  # noqa: E402

import bee_common as bc  # noqa: E402
import mission_rules as mr  # noqa: E402
import place_env as pe  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "learning" / "plan"))
import mission_sim as ms  # noqa: E402
from baseline import baseline_action  # noqa: E402
from solver import solver_action  # noqa: E402

STAGE = dc._STAGE
PUCK_NAMES = [c.GetName() for c in STAGE.GetPrimAtPath("/World/pucks").GetChildren()]
BOX_NAMES = [f"start_boxes/start_{i}" for i in range(1, 7)]
LOC_NAMES = ["shelf_L1", "shelf_L2", "shelf_L3"]
HOME = (-1.987, -2.471)  # start zone centre: mission (734, 950) mm in the scene frame (start_1 = (1468, 950) -> (-1.253, -2.471))
PRE_M = 0.24  # line up this far out on the face normal before docking (demo: an angled approach shoves the box);
# 0.24: the turn-in-place sweep of the rear corners is 0.29 m (root at the wheel axis, body x -0.245..0.069, y +-0.153),
# at 0.15 the robot turning away from a box hit it (run 1: L2 shoved 137 mm / 38 deg)
RES, X0, Y0, X1, Y1 = 0.04, -2.80, -3.50, 1.80, 3.00
R_WALL, R_BOX = 0.26, 0.26  # grid inflation [m]: turning sweep 0.29 minus the waypoints only turning where it is clear


@configclass
class MissionSceneCfg(InteractiveSceneCfg):
    demo = AssetBaseCfg(prim_path="{ENV_REGEX_NS}/demo", spawn=sim_utils.UsdFileCfg(usd_path=dc.DEMO_USD))
    robot = pe.robot_cfg().replace(prim_path="{ENV_REGEX_NS}/robot")
    start = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/demo/start_boxes/start_1", spawn=None)

    def __post_init__(self):
        super().__post_init__()
        self.robot.init_state.pos, self.robot.init_state.rot = dc.ROBOT_POSE
        for name, path in (("start", "/World/start_boxes/start_1"),):
            pos, rot = dc.usd_pose(path)
            getattr(self, name).init_state = RigidObjectCfg.InitialStateCfg(pos=pos, rot=rot)
        for k, n in enumerate(LOC_NAMES):
            pos, rot = dc.usd_pose(f"/World/{n}")
            setattr(self, f"loc{k}", RigidObjectCfg(prim_path="{ENV_REGEX_NS}/demo/" + n, spawn=None,
                                                     init_state=RigidObjectCfg.InitialStateCfg(pos=pos, rot=rot)))
        for n in PUCK_NAMES:
            pos, rot = dc.usd_pose(f"/World/pucks/{n}")
            setattr(self, n, RigidObjectCfg(prim_path="{ENV_REGEX_NS}/demo/pucks/" + n, spawn=None,
                                            init_state=RigidObjectCfg.InitialStateCfg(pos=pos, rot=rot)))


# ------------------------------------------------------------------------------------------------ map and paths
def box_info(path):
    c, _ = dc.usd_pose("/World/" + path)
    return c[0], c[1], math.remainder(dc.usd_yaw("/World/" + path), math.pi / 2)


BOXES = [box_info(p) for p in BOX_NAMES]
LOCS = [box_info(p) for p in LOC_NAMES]
NX, NY = int((X1 - X0) / RES), int((Y1 - Y0) / RES)


def cell_of(x, y):
    return min(max(int((y - Y0) / RES), 0), NY - 1), min(max(int((x - X0) / RES), 0), NX - 1)


def build_grid():
    """Occupancy (walls, boxes) inflated by the robot size: walls from the maze mesh triangles, boxes as squares."""
    gx = X0 + RES * (np.arange(NX) + 0.5)
    gy = Y0 + RES * (np.arange(NY) + 0.5)
    walls = np.zeros((NY, NX), bool)
    for p in Usd.PrimRange(STAGE.GetPrimAtPath("/World/maze")):
        if not p.IsA(UsdGeom.Mesh):
            continue
        m = np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        pts = np.array(UsdGeom.Mesh(p).GetPointsAttr().Get(), dtype=float)
        pts = np.c_[pts, np.ones(len(pts))] @ m
        counts, idx = UsdGeom.Mesh(p).GetFaceVertexCountsAttr().Get(), UsdGeom.Mesh(p).GetFaceVertexIndicesAttr().Get()
        k = 0
        for n in counts:
            f = [idx[k + j] for j in range(n)]
            k += n
            for j in range(1, n - 1):  # fan triangles, rasterized on the cell centres
                tri = pts[[f[0], f[j], f[j + 1]], :2]
                if pts[[f[0], f[j], f[j + 1]], 2].max() < 0.05:
                    continue
                lo, hi = tri.min(0), tri.max(0)
                ix = np.where((gx >= lo[0] - RES / 2) & (gx <= hi[0] + RES / 2))[0]
                iy = np.where((gy >= lo[1] - RES / 2) & (gy <= hi[1] + RES / 2))[0]
                if len(ix) == 0 or len(iy) == 0:
                    continue
                walls[np.ix_(iy, ix)] = True  # ponytail: triangle bbox (maze walls are axis aligned and thin)
    boxes = np.zeros((NY, NX), bool)
    for cx, cy, _ in BOXES + LOCS:
        boxes[np.ix_(np.abs(gy - cy) < bc.BOX_HALF + 0.01, np.abs(gx - cx) < bc.BOX_HALF + 0.01)] = True
    from scipy.ndimage import distance_transform_edt
    dw = distance_transform_edt(~walls) * RES
    db = distance_transform_edt(~boxes) * RES
    return dw, db


DW, DB = build_grid()
print(f"[mission] grid {NX}x{NY}: free {100 * np.mean((DW > R_WALL) & (DB > R_BOX)):.0f} %", flush=True)
try:  # map picture for checking (walls dark, inflation grey)
    from PIL import Image
    img = np.full((NY, NX, 3), 255, np.uint8)
    img[(DW <= R_WALL) | (DB <= R_BOX)] = (190, 190, 190)
    img[(DW < RES) | (DB < RES)] = (40, 40, 40)
    Image.fromarray(img[::-1]).resize((NX * 4, NY * 4), Image.NEAREST).save("/tmp/bee_mission_grid.png")
except ImportError:
    pass


def free(x, y, r_wall=R_WALL, r_box=R_BOX):
    i, j = cell_of(x, y)
    return DW[i, j] > r_wall and DB[i, j] > r_box


def astar(a, b):
    """8-connected A* on the inflated grid, then line-of-sight shortcuts -> [(x, y), ...] from a to b."""
    s, g = cell_of(*a), cell_of(*b)
    ok = (DW > R_WALL) & (DB > R_BOX)
    gy_, gx_ = np.mgrid[0:NY, 0:NX]
    for (ci, cj) in (s, g):  # start / goal next to a box (docked, home corner): let the robot leave or reach it
        ok |= (np.hypot(gy_ - ci, gx_ - cj) * RES < 0.25) & (DW > 0.15) & (DB > 0.04)
    best, prev, pq = {s: 0.0}, {}, [(0.0, s)]
    while pq:
        _, u = heapq.heappop(pq)
        if u == g:
            break
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                v = (u[0] + di, u[1] + dj)
                if (di or dj) and 0 <= v[0] < NY and 0 <= v[1] < NX and ok[v]:
                    c = best[u] + math.hypot(di, dj)
                    if c < best.get(v, 1e18):
                        best[v], prev[v] = c, u
                        heapq.heappush(pq, (c + math.hypot(v[0] - g[0], v[1] - g[1]), v))
    if g not in prev and g != s:
        return None
    cells = [g]
    while cells[-1] != s:
        cells.append(prev[cells[-1]])
    cells.reverse()
    xy = [(X0 + RES * (j + 0.5), Y0 + RES * (i + 0.5)) for i, j in cells]
    xy[0], xy[-1] = a, b
    out, k = [xy[0]], 0
    while k < len(xy) - 1:  # furthest visible point
        nxt = k + 1
        for m in range(len(xy) - 1, k, -1):
            n = int(math.hypot(xy[m][0] - xy[k][0], xy[m][1] - xy[k][1]) / (RES / 2)) + 2
            if all(ok[cell_of(xy[k][0] + (xy[m][0] - xy[k][0]) * t, xy[k][1] + (xy[m][1] - xy[k][1]) * t)] for t in np.linspace(0, 1, n)):
                nxt = m
                break
        out.append(xy[nxt])
        k = nxt
    return out


def face(box, h):
    """Dock and pre-dock poses on the face whose normal points along heading h (robot looks at the box)."""
    cx, cy, _ = box
    dock = (cx - dc.REACH * math.cos(h), cy - dc.REACH * math.sin(h), h)
    pre = (cx - (dc.REACH + PRE_M) * math.cos(h), cy - (dc.REACH + PRE_M) * math.sin(h), h)
    return dock, pre


def face_ok(box, h):
    dock, pre = face(box, h)  # walls do not move: a little less room to them (L3 south face, start_2 sat at 0.265 m)
    return free(*pre[:2], r_wall=0.22) and free(*dock[:2], r_wall=0.17, r_box=0.0)


# ------------------------------------------------------------------------------------------------ the run
class Mission:
    def __init__(self, sim, scene, demo):
        self.sim, self.scene, self.d, self.bot = sim, scene, demo, demo.bot
        self.org = scene.env_origins[0]
        self.docked = None  # (kind, index, heading) of the face the robot stands at
        self.slot_of, self.free_slots = {}, list(range(12))
        self.log = []

    # ---- problem
    def draw(self, rng):
        spots = []  # 12 (box, x, y): the USD puck spots, two per start box
        for n in PUCK_NAMES:
            p, _ = dc.usd_pose(f"/World/pucks/{n}")
            b = min(range(6), key=lambda k: math.hypot(p[0] - BOXES[k][0], p[1] - BOXES[k][1]))
            spots.append((b, p[0], p[1]))
        spots.sort(key=lambda s: (s[0], s[1], s[2]))
        names = PUCK_NAMES[:]
        rng.shuffle(names)
        self.names = names  # planner puck i = names[i], on start box i // 2 (mission_sim convention)
        self.box_h = []
        for b in range(6):
            (_, xa, ya), (_, xb, yb) = spots[2 * b], spots[2 * b + 1]
            across = math.atan2(yb - ya, xb - xa)  # the pair must sit across the gripper: dock on a box face at right
            # angles to it (box faces, not the exact pair normal: that one ended up in a wall's inflation at start_5)
            hs = [BOXES[b][2] + k * math.pi / 2 for k in range(4)]
            hs = [h for h in hs if abs(math.remainder(h - across - math.pi / 2, math.pi)) < math.pi / 4]
            hs.sort(key=lambda h: not face_ok(BOXES[b], h))
            if not face_ok(BOXES[b], hs[0]):  # e.g. start_6 between walls: dock on a free face with the pucks in a line
                inline = [BOXES[b][2] + k * math.pi / 2 for k in range(4) if face_ok(BOXES[b], BOXES[b][2] + k * math.pi / 2)]
                print(f"[mission] start_{b + 1}: no free face across its pucks, docking with them in a line", flush=True)
                hs = inline or hs
            self.box_h.append(hs[0])
        lying = []
        for i, n in enumerate(names):
            b, x, y = spots[i]
            yaw = rng.uniform(-math.pi, math.pi)
            if rng.random() < ours.lying:  # axis away from the robot (right_env trained +-90 deg), +-30 deg
                pos, rot = dc.lying_pose((x, y, 0.0), self.box_h[b] + rng.uniform(-0.5, 0.5))
                lying.append(n)
            else:
                pos, rot = (x, y, bc.BOX_H + 0.001), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
            pose = torch.tensor([[*pos, *rot]], dtype=torch.float32, device=self.d.dev)
            pose[:, :3] += self.org
            self.scene[n].write_root_pose_to_sim_index(root_pose=pose)
            self.scene[n].write_root_velocity_to_sim_index(root_velocity=torch.zeros(1, 6, device=self.d.dev))
        self.scene.write_data_to_sim()
        info = [(int(n.split("_")[-1]), n.split("_")[2], n.split("_")[1]) for n in names]  # (tag, colour, shape)
        self.counts, self.code, asc = mr.random_problem(info, rng)
        tgt = mr.targets(info, self.counts, asc)
        self.targets = [tgt[t] for t, _, _ in info]
        order = mr.assign_locations(self.counts, asc)
        print(f"[mission] problem: signal R {self.counts['R']} Y {self.counts['Y']} G {self.counts['G']}, barcode {self.code} "
              f"({'ascending' if asc else 'descending'}) -> L1 {order[0]}, L2 {order[1]}, L3 {order[2]}; lying: {lying or 'none'}", flush=True)
        for i, n in enumerate(names):
            print(f"[mission]   puck {i:2d} {n:18s} start_{i // 2 + 1} -> L{self.targets[i][0] + 1} cell {self.targets[i][1]}", flush=True)
        return lying

    # ---- driving
    def go(self, kind, idx, h):
        if self.docked == (kind, idx, h):
            return True
        box = BOXES[idx] if kind == "box" else LOCS[idx]
        dock, pre = face(box, h)
        b = self.bot
        x, y, yaw = b.root_pose2d()
        if self.docked is not None:  # back straight out first
            x, y = x - PRE_M * math.cos(yaw), y - PRE_M * math.sin(yaw)
            b.drive_to(x, y, yaw, reverse=True, label="back off")
        # arm in before driving: a puck held out front after a pick swept the boxes on the turns (run 1: L2 shoved
        # 138 mm / 38 deg, L3 73 mm); holding -> carry pose beside the robot (demo load / unload use it), empty -> stow
        held = self.d.in_hand()
        b.move(dc.CARRY[1] if held else dc.STOW, dc.CLOSE if held else dc.OPEN, 0.0, "arm in for driving")
        path = astar((x, y), pre[:2])
        if path is None:
            print(f"[mission] no path to {kind} {idx}", flush=True)
            return False
        for k in range(1, len(path)):
            nx_, ny_ = path[k]
            hd = pre[2] if k == len(path) - 1 else math.atan2(path[k + 1][1] - ny_, path[k + 1][0] - nx_)
            b.drive_to(nx_, ny_, hd)
        b.drive_to(*dock, label=f"dock {kind} {idx + 1} heading {math.degrees(h):.0f}")
        self.docked = (kind, idx, h)
        moved = [round(1000 * math.hypot(self.loc_now(k)[0] - LOCS[k][0], self.loc_now(k)[1] - LOCS[k][1])) for k in range(3)]
        if max(moved) > 5:
            print(f"[mission] location boxes moved {moved} mm", flush=True)
        return True

    def loc_now(self, loc):
        """Live centre and grid yaw of location box `loc` (docking can shove it)."""
        o = self.scene[f"loc{loc}"]
        p = o.data.root_pos_w.torch[0] - self.org
        yaw = mu.euler_xyz_from_quat(o.data.root_quat_w.torch)[2][0].item()
        return p[0].item(), p[1].item(), math.remainder(yaw, math.pi / 2)

    def loc_face(self, loc, cell):
        """Heading at location `loc` for arena cell (row 0 north, col 0 west) and the cell index in demo place() terms."""
        cx, cy, y0 = self.loc_now(loc)
        r, c = divmod(cell, 3)
        ox, oy = (c - 1) * bc.CELL_PITCH, (1 - r) * bc.CELL_PITCH
        ox, oy = ox * math.cos(y0) - oy * math.sin(y0), ox * math.sin(y0) + oy * math.cos(y0)
        best = None
        for k in range(4):
            h = y0 + k * math.pi / 2
            rr = round((ox * math.cos(h) + oy * math.sin(h)) / bc.CELL_PITCH) + 1
            cc = round((-ox * math.sin(h) + oy * math.cos(h)) / bc.CELL_PITCH) + 1
            if rr > 1 or not face_ok(LOCS[loc], h):
                continue
            keep = self.docked == ("loc", loc, h)
            x, y, _ = self.bot.root_pose2d()
            dist = math.hypot(face(LOCS[loc], h)[1][0] - x, face(LOCS[loc], h)[1][1] - y)
            cand = (not keep, dist, h, rr * 3 + cc)
            best = cand if best is None or cand < best else best
        return (best[2], best[3]) if best else (None, None)

    # ---- puck state from the simulation
    def use(self, i):
        self.d.puck = self.scene[self.names[i]]
        self.d.is_box = "_box_" in self.names[i]

    def where(self, i):
        p = self.scene[self.names[i]].data.root_pos_w.torch[0] - self.org
        for kind, lst in (("box", BOXES), ("loc", LOCS)):
            for k, (cx, cy, _) in enumerate(lst):
                if abs(p[0].item() - cx) < 0.12 and abs(p[1].item() - cy) < 0.12 and bc.BOX_H - 0.01 < p[2].item() < bc.BOX_H + 0.06:
                    return kind, k
        return None, None

    def standing(self, i):
        self.use(i)
        return self.d.up_z() > math.cos(math.radians(30.0))

    def cell_err(self, i):
        loc, cell = self.targets[i]
        cx, cy, y0 = self.loc_now(loc)
        r, c = divmod(cell, 3)
        ox, oy = (c - 1) * bc.CELL_PITCH, (1 - r) * bc.CELL_PITCH
        tx, ty = cx + ox * math.cos(y0) - oy * math.sin(y0), cy + ox * math.sin(y0) + oy * math.cos(y0)
        p = self.scene[self.names[i]].data.root_pos_w.torch[0] - self.org
        return math.hypot(p[0].item() - tx, p[1].item() - ty)

    def placed_state(self, i, say=False):
        kind, k = self.where(i)
        st = ms.LOST if kind != "loc" else ms.PLACED_LYING if not self.standing(i) else ms.OK if self.cell_err(i) < 0.010 else ms.OFF
        if say:
            loc = self.targets[i][0]
            cx, cy, y0 = self.loc_now(loc)
            moved = math.hypot(cx - LOCS[loc][0], cy - LOCS[loc][1])
            print(f"[mission]   judge puck {i}: on {kind} {k}, cell error {1000 * self.cell_err(i):.1f} mm, "
                  f"L{loc + 1} box moved {1000 * moved:.0f} mm / {math.degrees(y0 - LOCS[loc][2]):+.1f} deg", flush=True)
        return st

    # ---- one planner action
    def do(self, i, skill):
        d, name = self.d, self.names[i]
        loc, cell = self.targets[i]
        self.use(i)
        if skill in (ms.PICK, ms.STAND):
            if not self.go("box", i // 2, self.box_h[i // 2]):
                return None
            self.use(i)
            if skill == ms.STAND:
                ro = self.scene._rigid_objects
                ro["puck_b"], ro["puck_a"] = self.scene[name], self.scene[self.names[i ^ 1]]  # demo stand_up neighbour lookup
                try:
                    d.stand_up(regrasp=False)
                finally:
                    del ro["puck_a"], ro["puck_b"]
                kind, _ = self.where(i)
                return (ms.BOX_UP if self.standing(i) else ms.BOX_LYING) if kind == "box" else ms.LOST
            if d.pick():
                return ms.HAND
            kind, _ = self.where(i)
            return ms.LOST if kind is None else ms.BOX_UP if self.standing(i) else ms.BOX_LYING
        if skill == ms.LOAD:
            slot = self.free_slots.pop(0)
            if d.load(slot):
                self.slot_of[i] = slot
                return ms.BASKET
            self.free_slots.insert(0, slot)
            return ms.HAND if d.in_hand() else ms.LOST
        h, cell_rel = self.loc_face(loc, cell)
        if h is None:
            print(f"[mission] no dock face for L{loc + 1} cell {cell}", flush=True)
            return None
        if not self.go("loc", loc, h):
            return None
        self.use(i)
        lx, ly, _ = self.loc_now(loc)
        dc.DICE_BOX, dc.DOCK = (lx, ly, h), face((lx, ly, 0.0), h)[0]
        if skill == ms.UNLOAD:
            if d.unload(self.slot_of[i]):
                self.free_slots.append(self.slot_of.pop(i))
                return ms.HAND
            return ms.BASKET if self.where(i)[0] is None and not d.in_hand() else ms.LOST
        if skill == ms.PLACE:
            d.place(cell_rel)
            return ms.HAND if d.in_hand() else self.placed_state(i, say=True)
        if skill == ms.REPICK:
            if d.pick():
                return ms.HAND
            return self.placed_state(i, say=True)
        return None

    def home(self):
        b = self.bot
        x, y, yaw = b.root_pose2d()
        if self.docked is not None:
            x, y = x - PRE_M * math.cos(yaw), y - PRE_M * math.sin(yaw)
            b.drive_to(x, y, yaw, reverse=True, label="back off")
        b.move(dc.STOW, dc.OPEN, 0.0)
        path = astar((x, y), HOME) or [(x, y), HOME]
        for k in range(1, len(path)):
            b.drive_to(*path[k], math.atan2(path[k][1] - path[k - 1][1], path[k][0] - path[k - 1][0]), label="home" if k == len(path) - 1 else "")
        self.docked = None
        x, y, _ = b.root_pose2d()
        return math.hypot(x - HOME[0], y - HOME[1]) < 0.25


class RunSim(ms.MissionSim):
    """Planner state of the real run; skills without an Isaac implementation yet are masked, and an action that failed
    3 times in a row without changing anything is masked too (the abstract mission never repeats a failure forever,
    the planner had no reason to stop re-picking a puck its grasp could not take: 190 times in run 1)."""
    fails = {}

    def action_masks(self):
        m = super().action_masks()
        m[ms.NUDGE::ms.N_PER_PUCK] = False
        m[ms.STAND_PLACED::ms.N_PER_PUCK] = False
        for a, n in self.fails.items():
            if n >= 3:
                m[a] = False
        return m


SKILL = {ms.PICK: "pick_standing_puck", ms.STAND: "stand_lying_puck", ms.LOAD: "load_slot", ms.UNLOAD: "unload_slot", ms.PLACE: "place_cell",
         ms.NUDGE: "nudge_cell", ms.REPICK: "re-pick", ms.STAND_PLACED: "stand_puck_in_cell"}
STATUS = ["BOX_UP", "BOX_LYING", "HAND", "BASKET", "OK", "OFF", "PLACED_LYING", "LOST"]


def main():
    rng = random.Random(None if ours.problem_seed < 0 else ours.problem_seed)
    if dc.args.seed >= 0:
        np.random.seed(dc.args.seed)
        torch.manual_seed(dc.args.seed)
    eye, target = (-0.5, -6.2, 6.0), (-0.5, -0.3, 0.0)
    sim_cfg = sim_utils.SimulationCfg(dt=dc.DT, render_interval=dc.DECIM, physics=PhysxCfg(), device=dc.args.device)
    sim_cfg.default_visualizer_cfg = VisualizerCfg(eye=eye, lookat=target)
    sim = sim_utils.SimulationContext(sim_cfg)
    scene = InteractiveScene(MissionSceneCfg(num_envs=1, env_spacing=8.0))
    sim.stage.GetPrimAtPath(scene.env_prim_paths[0] + "/demo/beebot").SetActive(False)
    sim.stage.GetPrimAtPath(scene.env_prim_paths[0] + "/demo/sign").SetActive(False)
    sim.reset()
    sim.set_camera_view(eye=eye, target=target)
    for name in ("robot", *PUCK_NAMES):  # shape materials through the tensor API, as training (demo_cycle main)
        v = scene[name].root_physx_view
        v.set_material_properties(v.get_material_properties(), wp.from_torch(torch.arange(scene.num_envs, dtype=torch.int32)))
    ro = scene._rigid_objects
    ro["puck_a"], ro["puck_b"] = scene[PUCK_NAMES[0]], scene[PUCK_NAMES[1]]  # Demo.__init__ looks these up
    dc.PUCKS = {"puck_a": PUCK_NAMES[0], "puck_b": PUCK_NAMES[1], **{n: n for n in PUCK_NAMES}}
    demo = dc.Demo(sim, scene)
    del ro["puck_a"], ro["puck_b"]
    demo.right_policy = torch.jit.load(dc.args.right_policy, map_location=demo.dev).eval()
    planner = torch.jit.load(ours.plan_model or str(HERE / "policies" / "plan_next.pt"), map_location="cpu").eval()
    print(f"[mission] skills: place {dc.args.policy}, pick {dc.args.pick_policy}, stand {dc.args.right_policy}; planner {ours.planner}", flush=True)

    robot = scene["robot"]  # robot at its scene pose, arm at stow (demo_cycle.reset_scene without its two pucks)
    root = robot.data.default_root_pose.torch.clone()
    root[:, :3] += scene.env_origins
    robot.write_root_pose_to_sim_index(root_pose=root)
    jp = robot.data.default_joint_pos.torch.clone()
    stow = torch.tensor(dc.STOW, device=root.device)
    jp[:, demo.bot.arm] = stow
    robot.write_joint_state_to_sim_index(position=jp, velocity=torch.zeros_like(jp))
    robot.set_joint_position_target_index(target=jp[:, demo.bot.arm], joint_ids=demo.bot.arm)
    scene.write_data_to_sim()
    demo.bot.reset_state(stow)
    mission = Mission(sim, scene, demo)
    lying = mission.draw(rng)
    for _ in range(int(1.0 / dc.DT)):  # settle
        demo.bot.step(holding=True)
    env = RunSim()
    env.reset(seed=0)
    env.targets = mission.targets
    env.status = [ms.BOX_LYING if n in lying else ms.BOX_UP for n in mission.names]
    env.tries, env.at, env.t, env.returned = [0] * 12, ms.HOME_I, 0.0, False
    k0 = demo.bot.k
    for step in range(ours.max_steps):
        env.t = (demo.bot.k - k0) * dc.DT
        mask = env.action_masks()
        if ours.planner == "model":
            with torch.inference_mode():
                logits = planner(torch.as_tensor(env._obs()[None]))[0].numpy()
            logits[~mask] = -1e9
            a = int(logits.argmax())
        else:
            a = (solver_action if ours.planner == "solver" else baseline_action)(env)
        if a == ms.RETURN:
            print(f"[mission] step {step}: return home", flush=True)
            env.returned = mission.home()
            break
        i, skill = divmod(a, ms.N_PER_PUCK)
        before = env.status[i]
        print(f"[mission] step {step}: {SKILL[skill]} puck {i} ({mission.names[i]}, -> L{env.targets[i][0] + 1} cell {env.targets[i][1]}) "
              f"[{STATUS[before]}] t {env.t:.0f} s", flush=True)
        st = mission.do(i, skill)
        if st is None:
            print("[mission] skill could not start, stop", flush=True)
            break
        env.fails[a] = env.fails.get(a, 0) + 1 if st == before else 0
        env.status[i], env.tries[i] = st, env.tries[i] + 1
        env.at = (i // 2) if skill in (ms.PICK, ms.STAND) else (6 + env.targets[i][0]) if skill not in (ms.LOAD,) else env.at
        print(f"[mission]   -> {STATUS[st]}", flush=True)
    # final judge: every puck on its cell now (a later place can knock an earlier one)
    final = [mission.placed_state(i, say=True) if env.status[i] in (ms.OK, ms.OFF, ms.PLACED_LYING) else env.status[i] for i in range(12)]
    score = 0
    for loc in range(3):
        mine = [i for i in range(12) if env.targets[i][0] == loc]
        ok = all(final[i] == ms.OK for i in mine)
        score += 10 * ok
        print(f"[mission] L{loc + 1}: {sum(final[i] == ms.OK for i in mine)}/{len(mine)} on their cells -> {'+10' if ok else '0'}", flush=True)
    score += 10 * bool(env.returned)
    print(f"[mission] returned {'yes' if env.returned else 'no'}; score {score} / 40 (dice + return), "
          f"sim time {(demo.bot.k - k0) * dc.DT:.0f} s, planner {ours.planner}", flush=True)
    if dc.args.hold:
        while dc.app.is_running():
            demo.bot.step(holding=True)


if __name__ == "__main__":
    main()
    dc.app.close()
