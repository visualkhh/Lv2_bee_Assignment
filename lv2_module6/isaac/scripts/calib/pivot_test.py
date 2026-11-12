"""Real-robot pivot test (stand-up test 3): does a puck pinched at one end turn between the fingers at a given
gripper current? Bringup must be running (gripper_controller); the arm is NOT moved - pose it beforehand (tool down,
finger centre ~10 cm above the table: the hanging 60 mm puck must clear it, closing axis horizontal).

Per trial: open -> operator holds the puck lying (axis horizontal) between the fingers at one end -> close with the
current cap (action goal effort [A] -> firmware C 9th field) -> operator lets go and types the outcome:
    d = dropped (too weak)   h = swung down and hangs (turns: good)   p = partly turned (angle)   n = did not turn
    s = skip this trial   x = skip the rest of this current (next current)   q = quit
Logs commanded current, gripper position/effort from /joint_states, outcome to CSV.

    ROS_DOMAIN_ID=88 python3 pivot_test.py --shape box --trials 5   (default currents: 1-11 Goal Current units)
"""

import argparse
import csv
import datetime
import os
import time

import rclpy
from control_msgs.action import ParallelGripperCommand
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

JOINT = "gripper_left_joint"
OPEN_M = 0.012  # finger gap 44 mm: room to slide the 30 mm puck in
CLOSE_M = 0.0  # closed target; the current cap stops the fingers on the puck (gap 30 mm = 0.005)
FIRMWARE_MIN_A = 0.00269  # GRIPPER_CURRENT_MIN_MA (1 Goal Current unit, PR #50; was 0.05): smaller values are clamped

parser = argparse.ArgumentParser()
parser.add_argument("--shape", choices=["box", "cyl"], required=True)
parser.add_argument("--currents", default="0.00269,0.00538,0.00807,0.01076,0.01614,0.02152,0.02959",
                    help="A, comma separated (multiples of the 2.69 mA Goal Current unit)")
parser.add_argument("--trials", type=int, default=5)
parser.add_argument("--settle", type=float, default=1.0, help="s after closing before the operator lets go")
parser.add_argument("--out", default=None)
args = parser.parse_args()
currents = [float(c) for c in args.currents.split(",")]
if min(currents) < FIRMWARE_MIN_A:
    print(f"WARN: firmware clamps < {FIRMWARE_MIN_A} A to {FIRMWARE_MIN_A} A (config.h GRIPPER_CURRENT_MIN_MA)")
out = args.out or f"pivot_{args.shape}_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv"

rclpy.init()
node = rclpy.create_node("pivot_test")
state = {}


def on_js(m):
    if JOINT in m.name:
        i = m.name.index(JOINT)
        state.update(pos=m.position[i], eff=m.effort[i] if len(m.effort) > i else float("nan"), t=time.time())


node.create_subscription(JointState, "/joint_states", on_js, 10)
client = ActionClient(node, ParallelGripperCommand, "/gripper_controller/gripper_cmd")
if not client.wait_for_server(timeout_sec=10.0):
    raise SystemExit("FAIL: /gripper_controller/gripper_cmd not found (bringup running? ROS_DOMAIN_ID?)")


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def grip(pos, effort):
    goal = ParallelGripperCommand.Goal()
    goal.command.name = [JOINT]
    goal.command.position = [pos]
    goal.command.effort = [effort]
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    h = fut.result()
    if h is None or not h.accepted:
        raise SystemExit("FAIL: gripper goal rejected")
    res = h.get_result_async()
    rclpy.spin_until_future_complete(node, res, timeout_sec=5.0)  # stalled on the puck = expected
    return res.result().result if res.result() else None


def sample(sec):
    """mean gripper pos [m] and effort [A] over sec"""
    ps, es = [], []
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if "pos" in state:
            ps.append(state["pos"])
            es.append(state["eff"])
    n = max(len(ps), 1)
    return sum(ps) / n if ps else float("nan"), sum(es) / n if es else float("nan")


spin(1.0)
if "pos" not in state:
    raise SystemExit("FAIL: no /joint_states with gripper_left_joint")

new = not os.path.exists(out)
with open(out, "a", newline="") as f:
    w = csv.writer(f)
    if new:
        w.writerow(["time", "shape", "cmd_a", "trial", "pos_m", "gap_mm", "eff_a", "outcome", "angle_deg", "note"])
    print(f"log -> {out}\noutcome: d=dropped h=hangs(turned) p=partly(angle) n=no turn  s=skip  x=next current  q=quit")
    for cur in currents:
        print(f"\n===== {args.shape}  current {cur * 1000:.0f} mA =====")
        for k in range(1, args.trials + 1):
            grip(OPEN_M, 0.3)
            c = input(f"[{cur * 1000:.0f} mA #{k}] puck lying between the fingers at one end, hold it. "
                      "Enter = close  x = next current  q = quit ").strip().lower()
            if c == "q":
                grip(OPEN_M, 0.3)
                raise SystemExit(f"stopped, log -> {out}")
            if c == "x":
                break
            grip(CLOSE_M, cur)
            spin(args.settle)
            pos, eff = sample(0.5)
            print(f"  held: gap {20 + 2000 * pos:.1f} mm, current {eff * 1000:.0f} mA -> let go now")
            o = ""
            while o not in ("d", "h", "p", "n", "s", "x", "q"):
                o = input("  outcome [d/h/p/n/s/x/q]: ").strip().lower()
            if o == "q":
                grip(OPEN_M, 0.3)
                raise SystemExit(f"stopped, log -> {out}")
            if o == "s":
                continue
            if o == "x":
                break
            angle = input("  turned angle deg (Enter = skip): ").strip() if o == "p" else ""
            note = input("  note (Enter = none): ").strip()
            w.writerow([f"{time.time():.0f}", args.shape, cur, k, f"{pos:.5f}", f"{20 + 2000 * pos:.1f}",
                        f"{eff:.4f}", o, angle, note])
            f.flush()
    grip(OPEN_M, 0.3)
print(f"done, log -> {out}")
