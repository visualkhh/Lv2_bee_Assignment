"""Real-robot place test: gets a puck into the hand, then /skills/place puts it down at a fixed spot in front of the robot
(no driving, no board perception: the spot is given in base_footprint, x forward, y left). Logs logs/place_loop_<ts>.csv.

Bring up ./manipulator-launcher.sh first (terminal 1). Per round:
  --from slot   /skills/unload from basket slot S (--shape box|cyl), next round slot S+1, ... (ends at carry)
  --from pick   stow -> inspect the box (--box-x/--box-y) -> pick the puck nearest the box centre
  then /skills/place target "cell" at --to x y z (z = surface height, puck bottom lands there) -> stow
  -> you type the ruler-measured centre of the placed puck (mm) and whether it stands.
Arm-motion bag (./manipulator-record.sh motion, --bag off = none) -> logs/place_loop_<ts>_bags/round_NN/: starts on
Enter, stops after the stow (before the ruler questions). --auto: one bag for the whole run -> .../_bags/auto/.
Policies on/off are not set here: ros2_ws/model/skills/<skill>/meta.yaml (use:) + colcon build (place_cell, unload_slot).

    ROS_DOMAIN_ID=88 python3 place_loop.py --from slot --slot 0 --shape box box box cyl cyl --to 0.22 -0.05 0.07
    ROS_DOMAIN_ID=88 python3 place_loop.py --from pick --to 0.22 0.05 0.07
use_sim=true (or --use_sim true): the beebot in Isaac Sim (isaac/scripts/standalone/isaac_sim.sh, same topics and skills),
script and bag on sim time — run with the ROS_DOMAIN_ID isaac_sim.sh was started with.
    ROS_DOMAIN_ID=88 python3 place_loop.py --from slot --shape box box box cyl cyl --dice 5 --to 0.185 0 0.07 --auto   (die face 5, no stops)
"""

import argparse
import re
import sys
import csv
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from perception.msg import TrackedObjectArray
from plan.action import Skill
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rosbag2_interfaces.srv import Pause
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

# launch-style key=value arguments (use_sim=true) → --key value
sys.argv[1:] = [t for a in sys.argv[1:] for t in (["--" + a.split("=", 1)[0], a.split("=", 1)[1]]
                                                  if re.fullmatch(r"[a-z_]+=.*", a) else [a])]
parser = argparse.ArgumentParser()
parser.add_argument("--from", dest="src", choices=["slot", "pick"], required=True, help="where the puck comes from")
parser.add_argument("--slot", type=int, default=0, help="--from slot: first basket slot 0-11 (one more each round)")
parser.add_argument("--shape", nargs="+", choices=["box", "cyl"], default=["box"],
                    help="--from slot: puck shape per slot from --slot on (one value = every slot), e.g. box box box cyl cyl")
parser.add_argument("--to", type=float, nargs=3, metavar=("X", "Y", "Z"), default=[0.22, -0.05, 0.07],
                    help="place spot in base_footprint [m]: puck centre x, y and the surface height z (--dice: the pattern centre)")
parser.add_argument("--dice", type=int, choices=range(1, 7), default=0,
                    help="lay out a die face: round k goes to pip k (far row first, so the arm never reaches over a placed puck)")
parser.add_argument("--pitch", type=float, default=0.045,
                    help="--dice pip spacing [m] (mission board 0.060 does not fit the reach without driving: far row x > 0.25)")
parser.add_argument("--box-x", type=float, default=0.22, help="--from pick: box centre ahead of base_footprint [m]")
parser.add_argument("--box-y", type=float, default=0.0, help="--from pick: box centre left of base_footprint [m]")
parser.add_argument("--support", type=float, default=0.07, help="--from pick: box top height [m]")
parser.add_argument("--puck-h", type=float, default=0.06, help="standing puck height [m]")
parser.add_argument("--rounds", type=int, default=0, help="0 = until q / Ctrl-C")
parser.add_argument("--auto", action="store_true",
                    help="no Enter between rounds: unload -> place -> unload -> place ... in one go (unload ends over the slot, "
                         "place passes carry without stopping, stow only at the end); stops at the first failure; "
                         "ruler values asked once at the end")
parser.add_argument("--bag", choices=["motion", "off"], default="motion",
                    help="manipulator-record.sh motion bag per round (--auto: one for the run)")
parser.add_argument("--use_sim", type=lambda v: v.strip().lower() in ("1", "true", "yes"), default=False,
                    help="true: Isaac Sim beebot (isaac/scripts/standalone/isaac_sim.sh) — node and bag on sim time (/clock)")
args = parser.parse_args()

FIELDS = ["round", "src", "slot", "shape", "get", "get_reason", "place", "place_reason", "to_x_mm", "to_y_mm",
          "true_x_mm", "true_y_mm", "err_mm", "standing", "seconds"]
out = Path(__file__).resolve().parent / "logs" / f"place_loop_{time.strftime('%Y%m%d_%H%M%S')}.csv"
out.parent.mkdir(exist_ok=True)
BAG_SH = Path(__file__).resolve().parent / "manipulator-record.sh"
BAG_NODE = "place_loop_rec"

# the stack's Fast DDS profile (isaac_sim.sh / manipulator-launcher.sh): without it camera images never arrive
os.environ.setdefault("FASTDDS_DEFAULT_PROFILES_FILE", str(Path(__file__).resolve().parents[3] / "ros2_ws" / "fastdds_local_only.xml"))
rclpy.init()
if args.use_sim:  # same bringup / topics / skills as the robot, every node on /clock
    from rclpy.parameter import Parameter
    node = rclpy.create_node("place_loop", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    print("use_sim: Isaac Sim beebot, sim time (/clock)")
else:
    node = rclpy.create_node("place_loop")
tf_buf = Buffer()
TransformListener(tf_buf, node)
state = {"objects": None}
node.create_subscription(TrackedObjectArray, "/tracked_objects", lambda m: state.update(objects=m), 5)
clients = {s: ActionClient(node, Skill, f"/skills/{s}") for s in ("pose", "inspect", "pick", "unload", "place")}


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


class Bag:
    """manipulator-record.sh motion into logs/place_loop_<ts>_bags/<name>, stop() closes the MCAP"""

    def __init__(self, name):
        self.proc = None
        if args.bag == "off" or name is None:
            return
        env = dict(os.environ, BAG_OUT=str(out.with_suffix("")) + f"_bags/{name}", BAG_NODE=BAG_NODE, BAG_SIM="1" if args.use_sim else "")
        self.proc = subprocess.Popen([str(BAG_SH), "motion"], env=env,
                                     stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if not node.create_client(Pause, f"/{BAG_NODE}/pause").wait_for_service(timeout_sec=15.0):
            print("  bag: recorder did not come up — not recorded")
            self.stop()
            return
        spin(1.0)  # recorder subscribes to the topics after its services are up
        print(f"  bag: recording -> {env['BAG_OUT']}")

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)  # bash exec'd ros2 bag -> same pid; SIGINT closes the MCAP cleanly
            try:
                self.proc.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None


unload_params = node.create_client(SetParameters, "/skill_unload/set_parameters")


def unload_end_pose(pose):
    """skill_unload ends at carry (default) or over the slot (the next place passes carry without stopping)"""
    v = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=pose)
    if unload_params.wait_for_service(timeout_sec=3.0):
        rclpy.spin_until_future_complete(node, unload_params.call_async(SetParameters.Request(
            parameters=[Parameter(name="unload_end_pose", value=v)])), timeout_sec=3.0)


def pose_at(frame, x, y, z):
    """base_footprint (x, y) at height z -> PoseStamped in frame (recomputed each call in case odom moved)"""
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x, p.pose.position.y = apply(tf2d("base_footprint", frame), x, y)
    p.pose.position.z = z
    return p


def get_by_pick(frame):
    ok, why = run("pose", Skill.Goal(target="stow"))
    print(f"  stow: {'ok' if ok else why}")
    ok, why = run("inspect", Skill.Goal(target="box", values=[1.0], pose=pose_at(frame, args.box_x, args.box_y, args.support + args.puck_h)))
    print(f"  inspect: {'ok' if ok else why}")
    spin(0.5)
    pucks = [o for o in (state["objects"].objects if state["objects"] else []) if o.class_name.startswith("puck")]
    if not pucks:
        return False, "not_perceived", ""
    bx, by = apply(tf2d("base_footprint", frame), args.box_x, args.box_y)
    o = min(pucks, key=lambda p: math.hypot(p.pose.pose.position.x - bx, p.pose.pose.position.y - by))
    d = {kv.key: kv.value for kv in o.details}
    tag = int(d["tag"]) if d.get("tag", "").lstrip("-").isdigit() else -1
    print(f"  pick puck {o.id} tag {tag} {d.get('shape', '?')}")
    ok, why = run("pick", Skill.Goal(target=str(o.id), tag=tag, shape=d.get("shape", ""), objects=[o]))
    return ok, why, d.get("shape", "")


def ask_mm(prompt):
    while True:
        s = input(prompt).strip()
        if s in ("", "q"):
            return s
        try:
            v = [float(t) for t in s.replace(",", " ").split()]
            if len(v) == 2:
                return v
        except ValueError:
            pass
        print("    two numbers 'x y' in mm, Enter = skip, q = quit")


if not clients["place"].wait_for_server(timeout_sec=10.0):
    raise SystemExit("FAIL: /skills/place not up (launcher running?)")
frame, t_wait = None, time.time()
while frame is None and time.time() - t_wait < 10.0:
    spin(0.2)
    frame = next((f for f in ("map", "odom") if tf2d("base_footprint", f)), None)
if frame is None:
    raise SystemExit("FAIL: no map/odom -> base_footprint TF")
# die faces on a 3 x 3 grid (row +1 = far / +x, col +1 = left / +y), far row first
DICE = {1: [(0, 0)], 2: [(1, 1), (-1, -1)], 3: [(1, 1), (0, 0), (-1, -1)], 4: [(1, 1), (1, -1), (-1, 1), (-1, -1)],
        5: [(1, 1), (1, -1), (0, 0), (-1, 1), (-1, -1)], 6: [(1, 1), (1, -1), (0, 1), (0, -1), (-1, 1), (-1, -1)]}
cx, cy, tz = args.to
spots = [(cx + r * args.pitch, cy + c * args.pitch) for r, c in DICE[args.dice]] if args.dice else [(cx, cy)]
for i, (x, y) in enumerate(spots):
    print(f"place spot {i + 1}: base_footprint x {x * 1000:.0f} mm, y {y * 1000:.0f} mm, surface z {tz * 1000:.0f} mm")
print(f"log {out}")

rows = []
bag = Bag(None)
if args.auto:
    input("\nauto: every round runs without stopping. Basket and place spots ready, hands clear. Enter = start ")
    if args.src == "slot":
        unload_end_pose("over")
    bag = Bag("auto")
with open(out, "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=FIELDS)
    w.writeheader()
    n, slot, pending = 0, args.slot, None
    try:
        while args.rounds == 0 or n < args.rounds:
            n += 1
            k = slot - args.slot
            if args.src == "slot" and len(args.shape) > 1 and k >= len(args.shape):
                print("  every listed slot done")
                break
            shape = args.shape[min(k, len(args.shape) - 1)]
            what = f"slot {slot} ({shape})" if args.src == "slot" else "a puck on the box"
            if args.dice and n > len(spots):
                print("  die face done")
                break
            tx, ty = spots[n - 1] if args.dice else spots[0]
            clear = f"pip {n}/{len(spots)}" if args.dice else "place spot clear (take the last placed puck away)"
            if args.auto:
                print(f"\n[round {n}] {what} -> {clear}")
            elif input(f"\n[round {n}] {what} -> {clear}, hands clear. Enter = start, q = quit ").strip() == "q":
                break
            if not args.auto:
                bag = Bag(f"round_{n:02d}")
            t0 = time.time()
            row = {"round": n, "src": args.src, "to_x_mm": round(tx * 1000), "to_y_mm": round(ty * 1000)}
            if args.src == "slot":
                row.update(slot=slot, shape=shape)
                ok, why = run("unload", Skill.Goal(slot=slot, shape=shape))
                slot += 1
            else:
                ok, why, row["shape"] = get_by_pick(frame)
            row.update(get="ok" if ok else "fail", get_reason=why)
            print(f"  {'unload' if args.src == 'slot' else 'pick'}: {'ok' if ok else why}")
            if ok:
                ok, why = run("place", Skill.Goal(target="cell", pose=pose_at(frame, tx, ty, tz)))
                row.update(place="ok" if ok else "fail", place_reason=why)
                print(f"  place: {'ok' if ok else why}")
                if not args.auto:
                    pok, pwhy = run("pose", Skill.Goal(target="stow"))
                    print(f"  stow: {'ok' if pok else pwhy}")
            if not args.auto:
                bag.stop()
            row["seconds"] = round(time.time() - t0, 1)  # robot time only, not the typing below
            rows.append(row)
            pending = None if args.auto else row  # q / Ctrl-C at the ruler questions still writes this row
            if ok and not args.auto:
                    m = ask_mm("  ruler centre of the placed puck 'x y' mm (y left +; Enter = skip, q = quit): ")
                    if m == "q":
                        raise KeyboardInterrupt
                    if m:
                        e = math.hypot(m[0] - tx * 1000, m[1] - ty * 1000)
                        row.update(true_x_mm=m[0], true_y_mm=m[1], err_mm=round(e, 1))
                        print(f"  place error {e:.1f} mm (x {m[0] - tx * 1000:+.1f}, y {m[1] - ty * 1000:+.1f})")
                    row["standing"] = input("  standing upright? [y/n, Enter = skip] ").strip().lower()[:1]
            if not args.auto:
                w.writerow(row)
                fh.flush()
                pending = None
            elif row.get("place") != "ok":
                print("  auto: stopped at the first failure")
                break
            if args.src == "slot" and slot > 11:
                print("  last slot done")
                break
    except KeyboardInterrupt:
        print("\nstopped")
        if pending:
            w.writerow(pending)
    bag.stop()
    if args.auto:
        if args.src == "slot":
            unload_end_pose("carry")
        ok, why = run("pose", Skill.Goal(target="stow"))
        print(f"stow: {'ok' if ok else why}")
        bag.stop()
        try:
            for row in rows:
                if row.get("place") != "ok":
                    continue
                m = ask_mm(f"  round {row['round']} (spot x {row['to_x_mm']} y {row['to_y_mm']}): ruler centre 'x y' mm (Enter = skip, q = stop asking): ")
                if m == "q":
                    break
                if m:
                    e = math.hypot(m[0] - row["to_x_mm"], m[1] - row["to_y_mm"])
                    row.update(true_x_mm=m[0], true_y_mm=m[1], err_mm=round(e, 1))
                    print(f"    place error {e:.1f} mm (x {m[0] - row['to_x_mm']:+.1f}, y {m[1] - row['to_y_mm']:+.1f})")
                row["standing"] = input("    standing upright? [y/n, Enter = skip] ").strip().lower()[:1]
        except KeyboardInterrupt:
            pass
        w.writerows(rows)
print(f"log: {out}")
