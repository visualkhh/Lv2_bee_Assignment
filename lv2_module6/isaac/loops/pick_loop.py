"""Real-robot pick test: how far off are perception and the grasp? Repeats stow -> inspect box -> pick with the real
perception (YOLO) and logs the error of each round to logs/pick_loop_<ts>.csv.

Bring up ./manipulator-launcher.sh first (terminal 1). Put the box so its centre is --box-x/--box-y from base_footprint
(robot centre, x forward, y left) and one standing puck on it. Per round:
  1. stow -> inspect box (look pose, re-frame if the puck is not seen)
  2. prints where perception put the puck in base_footprint -> you type the ruler-measured centre (mm, Enter = skip)
  3. pick -> logs the last hand-camera residual before the close (/hand_relative rel = puck - fingertip, base_link)
     -> you type the in-hand offset (mm, Enter = skip): puck centre from the gripper centre along the finger pads
        (the parallel fingers centre it in the closing direction, so only this one is left)
One bag per round (./manipulator-record.sh, --bag cam|nocam|off) -> logs/pick_loop_<ts>_bags/round_NN/: starts on Enter,
paused while you type the ruler values, stops ~1 s after the pick.
With --slot S each round clears the box: one inspect, then pick -> /skills/load for every puck it saw, nearest the box
centre first, into slots S, S+1, .. (carried on across rounds, so empty the basket or restart when it is full). Between
pucks the load ends at the carry pose and the next pick goes straight to the remembered puck (no stow, no re-inspect).
You answer whether each puck sits upright in its slot. Stops the round at the first failed pick/load.
Without --slot only the puck nearest the box centre is picked and the hand just opens over the box.
A lying puck (perception puck_lying with a long-axis yaw) is stood up first: /skills/stand then /skills/stand_pick.
Pucks farther than 0.30 m from base_footprint fail with approach_failed (no Nav2 here).
--auto (needs --slot): one round, no stops — inspect -> pick -> load -> pick -> load ... until the box is empty or the first
failure, one bag for the round (no pauses). No ruler / in-hand questions (the pucks are in the basket by then); you answer
"seated upright" per loaded puck once at the end.

    ROS_DOMAIN_ID=88 python3 pick_loop.py --box-x 0.22 --box-y 0.0 --rounds 0      (0 = until q / Ctrl-C)
    ROS_DOMAIN_ID=88 python3 pick_loop.py --pucks 3 --slot 0 --auto
use_sim=true (or --use_sim true): the beebot in Isaac Sim (isaac/standalone/isaac_sim.sh, same topics and skills),
script and bag on sim time — run with the ROS_DOMAIN_ID isaac_sim.sh was started with.
Policy on/off is not set here: models/skills/pick_standing_puck/meta.yaml (use:) + colcon build.
"""

import argparse
import re
import sys
import csv
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from perception.msg import HandRelative, TrackedObjectArray
from plan.action import Skill
from control_msgs.action import ParallelGripperCommand
from rclpy.action import ActionClient
from rosbag2_interfaces.srv import Pause, Resume
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

# launch-style key=value arguments (use_sim=true) → --key value
sys.argv[1:] = [t for a in sys.argv[1:] for t in (["--" + a.split("=", 1)[0], a.split("=", 1)[1]]
                                                  if re.fullmatch(r"[a-z_]+=.*", a) else [a])]
parser = argparse.ArgumentParser()
parser.add_argument("--box-x", type=float, default=0.22, help="box centre ahead of base_footprint [m]")
parser.add_argument("--box-y", type=float, default=0.0, help="box centre left of base_footprint [m]")
parser.add_argument("--support", type=float, default=0.07, help="box top height [m]")
parser.add_argument("--puck-h", type=float, default=0.06, help="standing puck height [m]")
parser.add_argument("--pucks", type=int, default=1, help="pucks expected on the box (inspect stops when it sees them)")
parser.add_argument("--rounds", type=int, default=0, help="0 = until q / Ctrl-C")
parser.add_argument("--slot", type=int, default=-1,
                    help="load every picked puck, first into this basket slot 0-11, then the next ones (/skills/load); -1 = pick one, open the hand")
parser.add_argument("--bag", choices=["cam", "nocam", "off"], default="cam",
                    help="one manipulator-record.sh bag per round (paused while you measure); nocam = no images")
parser.add_argument("--auto", action="store_true", help="--slot only: one round pick -> load ... without stopping, questions at the end")
parser.add_argument("--use_sim", type=lambda v: v.strip().lower() in ("1", "true", "yes"), default=False,
                    help="true: Isaac Sim beebot (isaac/standalone/isaac_sim.sh) — node and bag on sim time (/clock)")
args = parser.parse_args()
if args.auto and args.slot < 0:
    parser.error("--auto needs --slot (pick -> load)")
if args.auto:
    args.rounds = 1

FIELDS = ["round", "puck_n", "slot", "inspect", "id", "tag", "color", "shape", "posture", "perc_x_mm", "perc_y_mm", "true_x_mm", "true_y_mm", "perc_err_mm", "all_err_mm",
          "stand", "stand_reason", "pick", "reason", "hand_rel_x_mm", "hand_rel_y_mm", "hand_rel_z_mm", "hand_age_s", "in_hand_off_mm", "load", "load_reason", "load_seated",
          "seconds", "true_posture", "sim_puck", "sim_inhand_x_mm", "sim_inhand_y_mm", "sim_inhand_z_mm", "sim_seated", "sim_tilt_deg"]
out = Path(__file__).resolve().parent / "logs" / f"pick_loop_{time.strftime('%Y%m%d_%H%M%S')}.csv"
out.parent.mkdir(exist_ok=True)
BAG_SH = Path(__file__).resolve().parent / "manipulator-record.sh"
BAG_NODE = "pick_loop_rec"

# the stack's Fast DDS profile (isaac_sim.sh / manipulator-launcher.sh): without it camera images never arrive
os.environ.setdefault("FASTDDS_DEFAULT_PROFILES_FILE", str(Path(__file__).resolve().parents[2] / "ros2_ws" / "fastdds_local_only.xml"))
rclpy.init()
if args.use_sim:  # same bringup / topics / skills as the robot, every node on /clock
    from rclpy.parameter import Parameter as RclpyParameter  # rcl_interfaces.msg Parameter is used below (load_end_pose)
    node = rclpy.create_node("pick_loop", parameter_overrides=[RclpyParameter("use_sim_time", RclpyParameter.Type.BOOL, True)])
    print("use_sim: Isaac Sim beebot, sim time (/clock)")
else:
    node = rclpy.create_node("pick_loop")
tf_buf = Buffer()
TransformListener(tf_buf, node)
state = {"objects": None, "hand": None}
node.create_subscription(TrackedObjectArray, "/tracked_objects", lambda m: state.update(objects=m), 5)
node.create_subscription(HandRelative, "/hand_relative",
                         lambda m: state.update(hand=m) if m.valid and m.kind == "puck" else None, 10)
clients = {s: ActionClient(node, Skill, f"/skills/{s}") for s in ("pose", "inspect", "pick", "load", "stand", "stand_pick")}
gripper = ActionClient(node, ParallelGripperCommand, "/gripper_controller/gripper_cmd")
GRIPPER_OPEN_M = 0.0265  # skills arm_io.hpp (URDF limit)


def release():
    """open the gripper so the puck drops back on the box — pulling it out of a closed hand trips the gripper overcurrent"""
    if not gripper.wait_for_server(timeout_sec=3.0):
        print("  gripper action not up — open it by hand")
        return
    g = ParallelGripperCommand.Goal()
    g.command.position = [GRIPPER_OPEN_M]
    fut = gripper.send_goal_async(g)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
    spin(1.0)
    print("  gripper opened (puck released)")


load_params = node.create_client(SetParameters, "/skill_load/set_parameters")


def load_end_pose(pose):
    """skill_load ends at carry (next pick goes straight to the puck) or stow (last puck)"""
    v = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=pose)
    if load_params.wait_for_service(timeout_sec=3.0):
        rclpy.spin_until_future_complete(node, load_params.call_async(SetParameters.Request(
            parameters=[Parameter(name="load_end_pose", value=v)])), timeout_sec=3.0)


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def tf2d(src, dst):
    """(x, y, yaw) of frame src in frame dst, None if no TF"""
    try:
        t = tf_buf.lookup_transform(dst, src, Time())
    except Exception:
        return None
    q = t.transform.rotation
    return t.transform.translation.x, t.transform.translation.y, 2.0 * math.atan2(q.z, q.w)


def apply(t, x, y):
    c, s = math.cos(t[2]), math.sin(t[2])
    return t[0] + c * x - s * y, t[1] + s * x + c * y


def run(skill, goal, timeout=180.0):
    cli = clients[skill]
    if not cli.wait_for_server(timeout_sec=10.0):
        return False, f"/skills/{skill} not up"
    last = [""]

    def fb(m):
        if m.feedback.stage != last[0]:
            last[0] = m.feedback.stage
            print(f"    {skill}: {m.feedback.stage}")

    fut = cli.send_goal_async(goal, feedback_callback=fb)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    gh = fut.result()
    if gh is None or not gh.accepted:
        return False, "rejected"
    res = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res, timeout_sec=timeout)
    if not res.done():
        gh.cancel_goal_async()
        return False, "timeout"
    r = res.result().result
    return r.success, r.reason


class RoundBag:
    """manipulator-record.sh for one round: Enter -> start, paused while you measure, stopped after the pick"""

    def __init__(self, n):
        self.proc = None
        if args.bag == "off":
            return
        env = dict(os.environ, BAG_OUT=str(out.with_suffix("")) + f"_bags/round_{n:02d}", BAG_NODE=BAG_NODE, BAG_SIM="1" if args.use_sim else "")
        self.proc = subprocess.Popen([str(BAG_SH)] + (["nocam"] if args.bag == "nocam" else []), env=env,
                                     stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.pause_cli = node.create_client(Pause, f"/{BAG_NODE}/pause")
        self.resume_cli = node.create_client(Resume, f"/{BAG_NODE}/resume")
        if not self.pause_cli.wait_for_service(timeout_sec=15.0):
            print("  bag: recorder did not come up — this round is not recorded")
            self.stop()
            return
        spin(1.0)  # recorder subscribes to the topics after its services are up
        print(f"  bag: recording -> {env['BAG_OUT']}")

    def _call(self, cli, req):
        if self.proc and self.proc.poll() is None:
            rclpy.spin_until_future_complete(node, cli.call_async(req), timeout_sec=3.0)

    def pause(self):
        if args.auto:  # nothing is typed during the round
            return
        self._call(self.pause_cli, Pause.Request())

    def resume(self):
        self._call(self.resume_cli, Resume.Request())

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)  # bash exec'd ros2 bag -> same pid; SIGINT closes the MCAP cleanly
            try:
                self.proc.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None


MATCH_M = 0.04  # same puck if within this of the inspect position [m] (old pick_test.py MATCH_RADIUS, removed)


def dist(a, b):
    return math.hypot(a.pose.pose.position.x - b.pose.pose.position.x, a.pose.pose.position.y - b.pose.pose.position.y)


SIM_TRUTH = "/tmp/beebot_sim_pucks.json"  # isaac_startup.py writes it while playing (base_link = base_footprint x y)


def sim_truth(seen, max_m=0.06):
    """use_sim: Isaac puck centres for the seen pucks [(id, x_mm, y_mm)] -> ([(x_mm, y_mm)], [posture]); nearest Isaac puck
    within max_m, else stops there (same as a skipped ruler entry). Empty if the file is missing or older than 2 s."""
    try:
        with open(SIM_TRUTH) as f:
            d = json.load(f)
    except (OSError, ValueError):
        return [], []
    if time.time() - d["t"] > 2.0:
        return [], []
    xy, post = [], []
    for _, qx, qy in seen:
        p = min(d["pucks"].values(), key=lambda p: math.hypot(p["x"] * 1000 - qx, p["y"] * 1000 - qy), default=None)
        if p is None or math.hypot(p["x"] * 1000 - qx, p["y"] * 1000 - qy) > max_m * 1000:
            break
        xy.append((round(p["x"] * 1000, 1), round(p["y"] * 1000, 1)))
        post.append("lying" if p["tilt"] > 45 else "standing" if p["tilt"] < 10 else f"tilted {p['tilt']:.0f}")
    return xy, post


def sim_pucks():
    """use_sim: Isaac puck truth {name: {x, y, z, tilt, yaw}} in base_link [m, deg], {} if missing or older than 2 s."""
    try:
        with open(SIM_TRUTH) as f:
            d = json.load(f)
    except (OSError, ValueError):
        return {}
    return d["pucks"] if time.time() - d["t"] <= 2.0 else {}


def sim_inhand():
    """use_sim: (puck name, puck centre - end_effector_link in the end effector frame [mm, x y z]) for the puck nearest the
    fingertip, or None. The full TF rotation is used (the tool points down, so a yaw-only frame would be wrong)."""
    pucks = sim_pucks()
    try:
        t = tf_buf.lookup_transform("base_link", "end_effector_link", Time())
    except Exception:
        return None
    if not pucks:
        return None
    p, q = t.transform.translation, t.transform.rotation
    w, x, y, z = q.w, q.x, q.y, q.z
    R = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    name, c = min(pucks.items(), key=lambda kv: math.dist((kv[1]["x"], kv[1]["y"], kv[1]["z"]), (p.x, p.y, p.z)))
    d = (c["x"] - p.x, c["y"] - p.y, c["z"] - p.z)
    return name, [round(1000 * sum(R[r][i] * d[r] for r in range(3)), 1) for i in range(3)]  # R^T d


def ask_mm(prompt, pairs=1):
    """'x y' pairs in mm -> list of (x, y); Enter = [] (skip), 'q' quits. Accepts 1 pair or `pairs` pairs."""
    while True:
        s = input(prompt).strip()
        if s == "q":
            raise KeyboardInterrupt
        if not s:
            return []
        try:
            v = [float(t) for t in s.replace(",", " ").split()]
        except ValueError:
            v = []
        if len(v) in (2, 2 * pairs):
            return list(zip(v[0::2], v[1::2]))
        print(f"  2 numbers{f' or {2 * pairs}' if pairs > 1 else ''}, e.g. 221 -4")


frame, t_wait = None, time.time()
while frame is None and time.time() - t_wait < 15.0:  # TF listener only fills while the node spins
    spin(0.5)
    frame = next((f for f in ("map", "odom") if tf2d("base_footprint", f)), None)
if frame is None:
    seen = len(node.get_node_names())
    raise SystemExit(f"FAIL: no TF map/odom -> base_footprint after 15 s (this process sees {seen} ROS nodes). "
                     "Few nodes = this new process cannot discover the stack (ros2 topic list can still show topics "
                     "from the ros2 daemon's old cache) -> restart the launcher")

with open(out, "w", newline="") as fh:
    w = csv.DictWriter(fh, FIELDS)
    w.writeheader()
    n = 0
    next_slot = args.slot
    auto_rows = []
    try:
        while args.rounds == 0 or n < args.rounds:
            n += 1
            input(f"\n[round {n}] pucks standing on the box, hands clear. Enter = start{' (auto: no stops)' if args.auto else ''}, Ctrl-C = quit ")
            t0 = time.time()
            bag = RoundBag(n)
            ok, why = run("pose", Skill.Goal(target="stow"))
            print(f"  stow: {'ok' if ok else why}")

            # box centre: robot frame -> map (recomputed every round in case odom drifted)
            bx, by = apply(tf2d("base_footprint", frame), args.box_x, args.box_y)
            g = Skill.Goal(target="box", values=[float(args.pucks)])
            g.pose = PoseStamped()
            g.pose.header.frame_id = frame
            g.pose.pose.position.x, g.pose.pose.position.y = bx, by
            g.pose.pose.position.z = args.support + args.puck_h
            ok, why = run("inspect", g)
            inspect = "ok" if ok else why
            print(f"  inspect: {inspect}")

            # remembered pucks (the inspect snapshot), nearest the box centre first = picking order
            spin(0.5)
            pucks = [o for o in (state["objects"].objects if state["objects"] else []) if o.class_name.startswith("puck")]
            if not pucks:
                print("  no puck in /tracked_objects — skip pick")
                bag.stop()
                w.writerow({"round": n, "inspect": inspect, "pick": "skip", "reason": "not_perceived",
                            "seconds": round(time.time() - t0, 1)})
                fh.flush()
                continue
            pucks.sort(key=lambda p: math.hypot(p.pose.pose.position.x - bx, p.pose.pose.position.y - by))
            if args.slot < 0:
                pucks = pucks[:1]
            t_robot = tf2d(state["objects"].header.frame_id or frame, "base_footprint")
            seen = []  # (id, x, y) in base_footprint, picking order
            for p in pucks:
                qx, qy = apply(t_robot, p.pose.pose.position.x, p.pose.pose.position.y)
                seen.append((p.id, qx * 1000, qy * 1000))
                d = {kv.key: kv.value for kv in p.details}
                print(f"  {len(seen)}. puck {p.id} tag {d.get('tag', '?')} | {p.color or '?'} {d.get('shape', '?')}"
                      f" {d.get('posture', '?')} ({p.confidence:.2f}): x {qx * 1000:.0f} mm, y {qy * 1000:.0f} mm")
            bag.pause()
            more = f" or all {len(seen)} in the order above 'x y x y ...'" if len(seen) > 1 else ""
            truth, true_post = sim_truth(seen) if args.use_sim else ([], [])
            if args.use_sim:
                print("  Isaac truth: " + (", ".join(f"x {x:.0f} y {y:.0f} mm {q}" for (x, y), q in zip(truth, true_post)) or "none (no fresh "
                                           f"{SIM_TRUTH} — restart isaac_sim.sh with the new isaac_startup.py)"))
            if not truth and not args.auto:
                truth = ask_mm(f"  ruler puck centre 'x y' mm (y: left +, right -) of puck 1{more} (Enter = skip, q = quit): ", len(seen))
            errs = [(i, math.hypot(qx - tx, qy - ty)) for (i, qx, qy), (tx, ty) in zip(seen, truth)]
            if errs:
                print("  perception error " + ", ".join(f"puck {i} {e:.1f} mm" for i, e in errs))
            bag.resume()

            for k, o in enumerate(pucks):
                last = k == len(pucks) - 1
                details = {kv.key: kv.value for kv in o.details}
                tag = int(details["tag"]) if details.get("tag", "").lstrip("-").isdigit() else -1
                row = {"round": n, "puck_n": k + 1, "inspect": inspect, "id": o.id, "tag": tag, "color": o.color,
                       "shape": details.get("shape", ""), "posture": details.get("posture", ""),
                       "perc_x_mm": round(seen[k][1], 1), "perc_y_mm": round(seen[k][2], 1)}
                if k < len(truth):
                    row.update(true_x_mm=truth[k][0], true_y_mm=truth[k][1], perc_err_mm=round(errs[k][1], 1),
                               all_err_mm=";".join(f"{i}:{e:.1f}" for i, e in errs))
                if k < len(true_post):
                    row["true_posture"] = true_post[k]
                tp = time.time()
                print(f"  -- puck {k + 1}/{len(pucks)}: {o.id} (tag {tag}) x {seen[k][1]:.0f} mm, y {seen[k][2]:.0f} mm")

                state["hand"] = None
                # perception drops a puck it has not seen for 2 s and gives it a new id when it comes back (long_memory false),
                # so the inspect id is often gone by now (11-08: no_grasp in 3 of 4 rounds). Re-match by place, else hand the
                # remembered snapshot to the skill (Skill.action objects = target first, then neighbours).
                # auto: the load action just spun (spin_until_future_complete), so /tracked_objects is fresh — no wait.
                # Otherwise the questions above blocked the node (input) and the last message is old.
                spin(0.02 if args.auto else 0.5)
                now = [p for p in (state["objects"].objects if state["objects"] else []) if p.class_name.startswith("puck")]
                near = min(now, key=lambda p: dist(p, o), default=None)
                goal = Skill.Goal(target=str(o.id), tag=tag, shape=details.get("shape", ""))
                if near is not None and dist(near, o) <= MATCH_M:
                    goal.target = str(near.id)
                    if near.id != o.id:
                        print(f"  target re-matched: puck {o.id} -> {near.id} ({dist(near, o) * 1000:.0f} mm away)")
                else:
                    goal.objects = [o] + pucks[k + 1:]
                    print(f"  puck {o.id} not in /tracked_objects now — picking from the remembered snapshot")
                if o.class_name == "puck_lying":  # lying: /skills/stand (arc) then stand_pick (same wrist, straight down)
                    if not any(kv.key == "yaw_period" for kv in o.details):
                        ok, why = False, "stand_no_yaw"  # long axis unknown (coordinator stand does the same)
                    else:
                        sg = Skill.Goal(shape=details.get("shape", ""), tag=tag)
                        sg.pose.header.frame_id = state["objects"].header.frame_id or frame
                        sg.pose.pose = o.pose.pose  # centre, long axis = orientation
                        ok, why = run("stand", sg)
                        row.update(stand="ok" if ok else "fail", stand_reason=why)
                        print(f"  stand: {'ok' if ok else 'FAIL ' + why}")
                        if ok:
                            spin(1.0)  # coordinator stand_check_s: let the stood puck settle
                            ok, why = run("stand_pick", Skill.Goal(target="last"))
                else:
                    ok, why = run("pick", goal)
                row.update(pick="ok" if ok else "fail", reason=why)
                print(f"  pick: {'ok' if ok else 'FAIL ' + why}")
                if not args.auto:
                    spin(1.0)  # a little after the pick (hand settles) into the bag — auto: straight to the load
                bag.pause()  # measuring in the hand is not recorded
                h = state["hand"]  # last puck residual the hand camera saw during the pick (before / at the close)
                if h is not None:
                    row.update(hand_rel_x_mm=round(h.rel.x * 1000, 1), hand_rel_y_mm=round(h.rel.y * 1000, 1),
                               hand_rel_z_mm=round(h.rel.z * 1000, 1), hand_age_s=round(h.age, 2))
                    print(f"  last hand residual (puck - fingertip): {h.rel.x * 1000:.0f}, {h.rel.y * 1000:.0f},"
                          f" {h.rel.z * 1000:.0f} mm")
                if ok and args.use_sim:  # Isaac truth: puck in the hand (end effector frame)
                    spin(0.5)
                    m = sim_inhand()
                    if m:
                        row.update(sim_puck=m[0], sim_inhand_x_mm=m[1][0], sim_inhand_y_mm=m[1][1], sim_inhand_z_mm=m[1][2])
                        print(f"  sim in-hand (puck - end effector, ee frame): {m[1]} mm ({m[0]})")
                if ok:
                    if not args.auto:  # auto: no questions — straight to the load
                        while True:  # one number: puck centre - gripper centre along the finger pads [mm]
                            v = input("  in-hand offset: puck centre from gripper centre along the pads, mm (Enter = skip, q = quit): ").strip()
                            if v == "q":
                                raise KeyboardInterrupt
                            try:
                                if v:
                                    row["in_hand_off_mm"] = float(v)
                                break
                            except ValueError:
                                print("  one number, e.g. 4 or -3")
                    if args.slot >= 0:
                        if next_slot > 11:
                            print("  basket full (slot 11 used) — not loading; empty it and restart with --slot 0")
                            release()
                            ok = False
                        else:
                            bag.resume()
                            # auto: load ends over the slot, the next pick passes carry without stopping (place_loop unload "over")
                            load_end_pose("stow" if last else "over" if args.auto else "carry")
                            row["slot"] = next_slot
                            lok, lwhy = run("load", Skill.Goal(slot=next_slot, shape=details.get("shape", ""), tag=tag))
                            next_slot += 1  # the puck is in that slot even when the end pose failed
                            row.update(load="ok" if lok else "fail", load_reason=lwhy)
                            print(f"  load slot {row['slot']}: {'ok' if lok else 'FAIL ' + lwhy}")
                            ok = lok
                            if not args.auto or args.use_sim:
                                spin(1.5 if args.use_sim else 1.0)
                            if args.use_sim and row.get("sim_puck"):  # Isaac truth: the loaded puck upright (< 10 deg)?
                                pk = sim_pucks().get(row["sim_puck"])
                                if pk:
                                    row.update(sim_seated="y" if pk["tilt"] < 10 else "n", sim_tilt_deg=round(pk["tilt"], 1))
                                    print(f"  sim slot: tilt {pk['tilt']:.1f} deg -> {'seated' if pk['tilt'] < 10 else 'NOT seated'}")
                            bag.pause()
                            v = "" if args.auto else input("  puck seated upright in the slot? y/n (Enter = skip, q = quit): ").strip().lower()
                            if v == "q":
                                raise KeyboardInterrupt
                            if v in ("y", "n"):
                                row["load_seated"] = v
                    else:
                        release()
                row["seconds"] = round(time.time() - tp, 1)
                if args.auto:
                    auto_rows.append(row)  # written after the seated questions
                else:
                    w.writerow(row)
                    fh.flush()
                if not ok:
                    break  # puck still in hand / on the box -> no next puck this round
                bag.resume()
            bag.stop()
            if not args.auto:
                input("  put the pucks back on the box (leave the basket as is — next round uses the next slots), "
                      "Enter = continue (arm goes to stow) " if args.slot >= 0 else
                      "  put the puck back on the box, Enter = continue (arm goes to stow) ")
            ok, why = run("pose", Skill.Goal(target="stow"))
            print(f"  stow: {'ok' if ok else why}")
    except KeyboardInterrupt:
        pass
    finally:
        if "bag" in globals():
            bag.stop()
    try:
        for row in auto_rows:
            if row.get("load") == "ok":
                v = input(f"  slot {row['slot']} (puck {row['id']}): seated upright? y/n (Enter = skip, q = stop asking): ").strip().lower()
                if v == "q":
                    break
                if v in ("y", "n"):
                    row["load_seated"] = v
    except KeyboardInterrupt:
        pass
    w.writerows(auto_rows)
print(f"\nsaved {out}")
