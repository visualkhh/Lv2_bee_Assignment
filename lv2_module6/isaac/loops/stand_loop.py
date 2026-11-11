"""Real-robot stand-up test 4: can the gripper really pinch a lying puck and turn it? Repeats the coordinator `stand`
skill at a fixed spot so you only have to put a puck there.

No perception: this script publishes a FAKE lying puck on /tracked_objects (box top 0.07 m, puck 0.03 m high ->
puck centre z 0.085) in front of the robot and sends `stand <id>` over and over. Bring up stand_test.launch.py WITHOUT
the perception node (it would publish /tracked_objects too): `ros2 lifecycle`-free way = kill it after launch:
    ros2 node list | grep perception ; pkill -f perception_node

Per round: arm goes above the far end -> opens -> descends -> pinch -> arc -> release -> lift. Between rounds the
script waits --pause seconds with the hand lifted: put the puck lying at the marked spot (long axis along --axis-deg
from the robot's forward x, centre --x/--y from base_footprint) before the next round starts.

    ROS_DOMAIN_ID=88 python3 stand_loop.py --x 0.22 --y 0.0 --rounds 0      (0 = until Ctrl-C)
    add --preview to only hover above the grasp point (check the hand position first)

--perception: keep perception_node running and stand the REAL lying puck it tracks (nearest to --x/--y, within
--match-m) instead of the fake one. Each round: stow -> /skills/inspect the spot (cameras only see the puck from there)
-> pick the tracked lying puck -> stand — needed for the hand-camera re-measure before the descent (/skill_stand stand_cam).
Bring up the stack WITH perception for this.

After each stand: one gripper-cam depth look from where the hand ended (no YOLO) -> height of the highest points
(p95) above the box top around where the stood puck should be: >= 45 mm standing, >= 15 mm lying, else gone
(cam_h_mm / cam_verdict in the CSV, next to your y/n). --pick: if the depth says standing, /skills/pick picks it
(pick_standing_puck policy) at the depth-measured spot; you take it out of the hand, Enter opens the gripper.

--auto: no questions, one lying puck: inspect -> stand -> depth check; lying again -> stand again from the look pose
(--retries); standing -> pick (vertical) -> /skills/load into --slot (default 0; -1 = no load: Enter opens the gripper after
you take the puck out). gone (off the box)
-> stop. A puck that already stands is picked straight away. logs/stand_auto_<ts>.csv, one bag.
    ROS_DOMAIN_ID=88 python3 stand_loop.py --auto --x 0.22 --y 0.0

use_sim=true (or --use_sim true): the beebot in Isaac Sim (isaac/standalone/isaac_sim.sh — same bringup, topics
and skills as the robot, every node on /clock): this script and its bag recorder run on sim time too (the depth check
compares image stamps with "now"). Run it in a shell with the ROS_DOMAIN_ID isaac_sim.sh was started with.
    python3 stand_loop.py use_sim=true --perception --pick --x 0.22 --y 0.0 --rounds 10

One bag per round (./manipulator-record.sh, --bag cam|nocam|off) -> logs/stand_loop_<ts>_bags/round_NN/: starts when the
round starts, stops a little after the result (putting the puck back is not recorded).
logs/stand_loop_<ts>.csv: per round the skill result, whether the puck really stands (you answer y/n after putting it
back lying; --no-ask = old timed --pause, no answer) and the /skill_stand arc parameters of that round — change them
between rounds with `ros2 param set /skill_stand stand_...` (read at every goal) and the CSV keeps track.
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

import numpy as np
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import KeyValue
from plan.msg import SubGoal, SubGoalArray, TaskStatus
from perception.msg import TrackedObject, TrackedObjectArray
from rcl_interfaces.srv import GetParameters
from control_msgs.action import ParallelGripperCommand  # Lyrical: the gripper_controller action (GripperCommand has no server)
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from plan.action import Skill
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rosbag2_interfaces.srv import Pause
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

# launch-style key=value arguments (use_sim=true) → --key value
sys.argv[1:] = [t for a in sys.argv[1:] for t in (["--" + a.split("=", 1)[0], a.split("=", 1)[1]]
                                                  if re.fullmatch(r"[a-z_]+=.*", a) else [a])]
parser = argparse.ArgumentParser()
parser.add_argument("--x", type=float, default=0.22, help="puck centre ahead of base_footprint [m] (arm reach 0.13-0.31)")
parser.add_argument("--y", type=float, default=0.0, help="puck centre left of base_footprint [m]")
parser.add_argument("--axis-deg", type=float, default=0.0, help="puck long axis, deg from robot forward (0 = pointing away)")
parser.add_argument("--support", type=float, default=0.07, help="surface height [m] (box top 0.07)")
parser.add_argument("--shape", choices=["box", "cyl"], default="box")
parser.add_argument("--rounds", type=int, default=0, help="0 = until Ctrl-C")
parser.add_argument("--pause", type=float, default=6.0, help="s between rounds (hand lifted) to put the puck back")
parser.add_argument("--preview", action="store_true", help="only hover above the grasp point")
parser.add_argument("--perception", action="store_true",
                    help="use the real tracked lying puck (perception_node stays up) instead of the fake one")
parser.add_argument("--match-m", type=float, default=0.08, help="--perception: tracked lying puck must be this close to --x/--y [m]")
parser.add_argument("--pick", action="store_true",
                    help="after a stand the depth calls standing: /skills/pick it (demo stand -> pick)")
parser.add_argument("--use_sim", type=lambda v: v.strip().lower() in ("1", "true", "yes"), default=False,
                    help="true: Isaac Sim beebot (isaac_sim.sh) — node and bag on sim time")
parser.add_argument("--auto", action="store_true", help="one lying puck: stand -> (retry) -> pick -> load (--slot), no questions")
parser.add_argument("--slot", type=int, default=0, help="--auto: basket slot 0-11 the picked puck is loaded into (/skills/load); -1 = no load, Enter opens the hand")
parser.add_argument("--retries", type=int, default=3, help="--auto: stand attempts per puck after the first")
parser.add_argument("--ask", action=argparse.BooleanOptionalAction, default=True,
                    help="after each round ask whether the puck stands (the answer starts the next round)")
parser.add_argument("--bag", choices=["cam", "nocam", "off"], default="cam",
                    help="one manipulator-record.sh bag per round; nocam = no images")
args = parser.parse_args()
if args.auto:
    args.perception, args.pick, args.ask = True, True, False

PUCK_ID = 99
PUCK_W = 0.030
PUCK_L = 0.060
STAND_H, LYING_H = 0.045, 0.015  # [m] depth p95 height above the box top: standing puck ~60 mm, lying ~30 mm
BAG_SH = Path(__file__).resolve().parent / "manipulator-record.sh"
BAG_NODE = "stand_loop_rec"
STAMP = time.strftime('%Y%m%d_%H%M%S')
BAG_DIR = BAG_SH.parent / "logs" / f"stand_loop_{STAMP}_bags"
CSV_PATH = BAG_SH.parent / "logs" / f"stand_loop_{STAMP}.csv"
PARAMS = ["stand_arc_k", "stand_cam", "stand_arc_end_deg", "stand_tilt_over_m", "stand_arc_return", "stand_set_dz_m", "stand_above_m",
          "stand_lift_away_m", "stand_pinch_m", "stand_arc_vel_scale", "stand_drag_m"]

# Clean up before starting: other stand_loop.py copies (Ctrl-Z leaves them stopped, and each one is a DDS participant /
# fake-puck publisher) and perception_node (it would publish /tracked_objects too).
for pid in subprocess.run(["pgrep", "-f", "stand_loop.py"], capture_output=True, text=True).stdout.split():
    if int(pid) != os.getpid():
        os.kill(int(pid), signal.SIGCONT)  # a stopped process only handles SIGTERM after it is continued
        os.kill(int(pid), signal.SIGTERM)
if args.perception:
    if not subprocess.run(["pgrep", "-f", "perception_node"], capture_output=True).stdout:
        print("WARN: --perception but no perception_node running — start the stack with perception")
else:
    subprocess.run(["pkill", "-f", "perception_node"])

# depth images only come through with the stack's shared-memory Fast DDS profile (without it: no frames at all, 11/08)
os.environ.setdefault("FASTDDS_DEFAULT_PROFILES_FILE",
                      str(Path(__file__).resolve().parents[2] / "ros2_ws" / "fastdds_local_only.xml"))
rclpy.init()
if args.use_sim:
    from rclpy.parameter import Parameter as RclpyParameter  # not "Parameter": keep the name free for rcl_interfaces.msg
    node = rclpy.create_node("stand_loop", parameter_overrides=[RclpyParameter("use_sim_time", RclpyParameter.Type.BOOL, True)])
    print("use_sim: Isaac Sim beebot, sim time (/clock)")
else:
    node = rclpy.create_node("stand_loop")
tf_buf = Buffer()
TransformListener(tf_buf, node)
latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
status = {}
node.create_subscription(TaskStatus, "/task/status", lambda m: status.update({m.plan_id: m}), latched)
goal_pub = node.create_publisher(SubGoalArray, "/task/sub_goals", latched)
tracked = {"msg": None}
if args.perception:
    node.create_subscription(TrackedObjectArray, "/tracked_objects", lambda m: tracked.update(msg=m),
                             QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
obj_pub = None if args.perception else node.create_publisher(  # --perception: perception_node is the only publisher
    TrackedObjectArray, "/tracked_objects", QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def world_pose():
    """puck centre + yaw in the frame the coordinator uses (odom if there is no map TF), from the robot-frame spot"""
    for frame in ("map", "odom"):
        try:
            t = tf_buf.lookup_transform(frame, "base_footprint", Time())
        except Exception:
            spin(1.0)
            try:
                t = tf_buf.lookup_transform(frame, "base_footprint", Time())
            except Exception:
                continue
        q = t.transform.rotation
        yaw = 2.0 * math.atan2(q.z, q.w)
        c, s = math.cos(yaw), math.sin(yaw)
        x = t.transform.translation.x + c * args.x - s * args.y
        y = t.transform.translation.y + s * args.x + c * args.y
        return frame, x, y, yaw + math.radians(args.axis_deg)
    raise SystemExit("FAIL: no TF map/odom -> base_footprint (bringup running?)")


spin(2.0)  # TF listener only fills while the node spins
frame, px, py, ayaw = world_pose()
if not args.perception:
    print(f"fake puck in {frame}: ({px:.3f}, {py:.3f}, z {args.support + PUCK_W / 2:.3f}) axis {math.degrees(ayaw):.0f} deg")


def fake_objects():
    o = TrackedObject()
    o.id = PUCK_ID
    o.class_name = "puck_lying"
    o.confidence = 1.0
    o.pose.pose.position.x, o.pose.pose.position.y = px, py
    o.pose.pose.position.z = args.support + PUCK_W / 2
    o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(ayaw / 2), math.cos(ayaw / 2)
    o.size.x = o.size.y = PUCK_W
    o.size.z = PUCK_W
    o.details = [KeyValue(key="posture", value="lying"), KeyValue(key="shape", value=args.shape),
                 KeyValue(key="yaw_period", value="180")]
    m = TrackedObjectArray()
    m.header.frame_id = frame
    m.header.stamp = node.get_clock().now().to_msg()
    m.objects = [o]
    return m


class RoundBag:
    """manipulator-record.sh into BAG_DIR/round_NN, stop() closes the MCAP"""

    def __init__(self, n):
        self.proc = None
        if args.bag == "off":
            return
        env = dict(os.environ, BAG_OUT=str(BAG_DIR / f"round_{n:02d}"), BAG_NODE=BAG_NODE, BAG_SIM="1" if args.use_sim else "")
        BAG_DIR.mkdir(parents=True, exist_ok=True)
        self.proc = subprocess.Popen([str(BAG_SH)] + (["nocam"] if args.bag == "nocam" else []), env=env,
                                     stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if not node.create_client(Pause, f"/{BAG_NODE}/pause").wait_for_service(timeout_sec=15.0):
            print("  bag: recorder did not come up — this round is not recorded")
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


param_cli = node.create_client(GetParameters, "/skill_stand/get_parameters")


def stand_params():
    """the /skill_stand values this round runs with (empty if the node does not answer)"""
    if not param_cli.wait_for_service(timeout_sec=2.0):
        return {}
    fut = param_cli.call_async(GetParameters.Request(names=PARAMS))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
    if not fut.result():
        return {}
    vals = [v.double_value if v.type == 3 else v.bool_value if v.type == 1 else v.integer_value if v.type == 2 else ""
            for v in fut.result().values]
    return dict(zip(PARAMS, vals))


skill_cli = {k: ActionClient(node, Skill, f"/skills/{k}") for k in ("pose", "inspect", "load")}


def run_skill(skill, goal, timeout=60.0):
    """/skills/<skill> action → (ok, reason)"""
    cli = skill_cli[skill]
    if not cli.wait_for_server(timeout_sec=10.0):
        return False, f"/skills/{skill} not up"
    fut = cli.send_goal_async(goal)
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


def inspect_spot():
    """stow → inspect the box at the spot (like place_loop --from pick) so the cameras see the lying puck"""
    ok, why = run_skill("pose", Skill.Goal(target="stow"))
    print(f"  stow: {'ok' if ok else why}")
    t = tf_buf.lookup_transform(frame, "base_footprint", Time())
    q = t.transform.rotation
    yaw = 2.0 * math.atan2(q.z, q.w)
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = t.transform.translation.x + math.cos(yaw) * args.x - math.sin(yaw) * args.y
    p.pose.position.y = t.transform.translation.y + math.sin(yaw) * args.x + math.cos(yaw) * args.y
    p.pose.position.z = args.support + PUCK_W
    ok, why = run_skill("inspect", Skill.Goal(target="box", values=[1.0], pose=p))
    print(f"  inspect: {'ok' if ok else why}")
    tracked["msg"] = None  # only detections from the inspect pose
    spin(0.5)


def real_puck(timeout=5.0, posture_want="lying"):
    """--perception: (id, x, y, axis yaw in base_footprint) of the tracked puck in this posture nearest the spot, or None"""
    end = time.time() + timeout
    while time.time() < end:
        spin(0.2)
        m = tracked["msg"]
        if m is None:
            continue
        try:
            t = tf_buf.lookup_transform("base_footprint", m.header.frame_id or "map", Time())
        except Exception:
            continue
        q = t.transform.rotation
        yaw = 2.0 * math.atan2(q.z, q.w)
        c, s = math.cos(yaw), math.sin(yaw)
        best = None
        for o in m.objects:
            posture = next((kv.value for kv in o.details if kv.key == "posture"), "")
            lying = o.class_name == "puck_lying" or posture == "lying"
            if not o.class_name.startswith("puck") or lying != (posture_want == "lying"):
                continue
            ox, oy = o.pose.pose.position.x, o.pose.pose.position.y
            bx = t.transform.translation.x + c * ox - s * oy
            by = t.transform.translation.y + s * ox + c * oy
            oq = o.pose.pose.orientation
            d = math.hypot(bx - args.x, by - args.y)
            if d < args.match_m and (best is None or d < best[0]):
                best = (d, o.id, bx, by, 2.0 * math.atan2(oq.z, oq.w) + yaw)
        if best:
            return best[1:]
    return None


depth = {"img": None, "info": None}
node.create_subscription(Image, "/gripper_cam/aligned_depth_to_color/image_raw", lambda m: depth.update(img=m), qos_profile_sensor_data)
node.create_subscription(CameraInfo, "/gripper_cam/aligned_depth_to_color/camera_info", lambda m: depth.update(info=m),
                         qos_profile_sensor_data)


def depth_points(after_ns, frames=3, timeout=3.0):
    """gripper-cam depth → points (N×3) in base_footprint from up to `frames` images newer than after_ns"""
    pts, last, end = [], None, time.time() + timeout
    while len(pts) < frames and time.time() < end:
        spin(0.05)
        m, info = depth["img"], depth["info"]
        if m is None or info is None:
            continue
        stamp = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        if stamp <= after_ns or stamp == last:
            continue
        last = stamp
        try:
            t = tf_buf.lookup_transform("base_footprint", m.header.frame_id, Time())
        except Exception:
            continue
        if m.encoding == "16UC1":
            z = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(np.float32) * 0.001
        elif m.encoding == "32FC1":
            z = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)
        else:
            print(f"  depth: encoding {m.encoding} not handled")
            return np.zeros((0, 3))
        v, u = np.mgrid[0:m.height:2, 0:m.width:2]
        z = z[::2, ::2]
        ok = (z > 0.09) & (z < 0.6)  # < 9 cm = own fingers · D435 has no depth that close anyway
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        p = np.stack([(u[ok] - cx) * z[ok] / fx, (v[ok] - cy) * z[ok] / fy, z[ok]], 1)
        q = t.transform.rotation
        x_, y_, z_, w_ = q.x, q.y, q.z, q.w
        R = np.array([[1 - 2 * (y_ * y_ + z_ * z_), 2 * (x_ * y_ - z_ * w_), 2 * (x_ * z_ + y_ * w_)],
                      [2 * (x_ * y_ + z_ * w_), 1 - 2 * (x_ * x_ + z_ * z_), 2 * (y_ * z_ - x_ * w_)],
                      [2 * (x_ * z_ - y_ * w_), 2 * (y_ * z_ + x_ * w_), 1 - 2 * (x_ * x_ + y_ * y_)]])
        tr = t.transform.translation
        pts.append(p @ R.T + np.array([tr.x, tr.y, tr.z]))
    return np.concatenate(pts) if pts else np.zeros((0, 3))


def stand_verdict(after_ns, cx, cy, yaw, save=None):
    """depth height above the box top where the stood puck should be (pivot side, see stand_plan waypoints)
    → (verdict, p95 height mm, standing-puck xy or None)"""
    # the stood puck lands 45 mm from the lying puck's centre on either side (skill_stand stand_end_margin_m may pinch the
    # other end), so look at a 65 mm disc around the centre: the height tells standing (~60 mm) from lying (~30 mm)
    p = depth_points(after_ns)
    if len(p) == 0:
        return "no_depth", "", None
    near = p[np.hypot(p[:, 0] - cx, p[:, 1] - cy) < 0.065]
    if len(near) < 20:
        return "unseen", "", None
    if save is not None:
        np.save(save, near)  # points around the stood-puck spot (base_footprint) — the bag has no depth
    # reference = the desk around the puck, measured in the same depth frame: the camera height from the arm FK is off by
    #   1~1.5 cm and changes with the arm pose (desk read 0.095 from stow, 0.111 from another pose, 11/10), so a fixed
    #   --support turned a lying puck into 'standing'. Fall back to --support when the ring has too few points
    r = np.hypot(p[:, 0] - cx, p[:, 1] - cy)
    ring = p[(r > 0.08) & (r < 0.16)]
    desk = float(np.median(ring[:, 2])) if len(ring) >= 200 else args.support
    print(f"  desk z {desk:.3f} ({'ring' if len(ring) >= 200 else '--support'}), --support {args.support:.3f}")
    h = near[:, 2] - desk
    h95 = float(np.percentile(h, 95))
    # pick target = top face only (within 8 mm of the top): every point above STAND_H also takes the side face the camera
    # sees, which pulls the centre toward the camera (11/08 first pick: hand cam said the puck was 18 mm off in y)
    top = near[h > max(STAND_H, h95 - 0.008)]
    xy = (float(np.median(top[:, 0])), float(np.median(top[:, 1]))) if len(top) >= 10 else None
    verdict = "standing" if h95 >= STAND_H else "lying" if h95 >= LYING_H else "gone"
    return verdict, round(h95 * 1000, 1), xy


pick_cli = ActionClient(node, Skill, "/skills/pick")
grip_cli = ActionClient(node, ParallelGripperCommand, "/gripper_controller/gripper_cmd")


def pick_standing(n, bx, by, yaw_b):
    """/skills/pick on a standing box puck at (bx, by) base_footprint — grasp_scorer takes it from the request objects"""
    t = tf_buf.lookup_transform(frame, "base_footprint", Time())
    q = t.transform.rotation
    ryaw = 2.0 * math.atan2(q.z, q.w)
    o = TrackedObject()
    o.id = 9000 + n  # not a perception id → grasp_scorer uses this object, not /tracked_objects
    o.class_name = "puck"
    o.confidence = 1.0
    o.pose.pose.position.x = t.transform.translation.x + math.cos(ryaw) * bx - math.sin(ryaw) * by
    o.pose.pose.position.y = t.transform.translation.y + math.sin(ryaw) * bx + math.cos(ryaw) * by
    o.pose.pose.position.z = args.support + 0.06 / 2  # > 0.09 → skills standing_grasp_z takes it as on the box
    myaw = yaw_b + ryaw  # stood box puck faces keep the lying puck's yaw (mod 90°)
    o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(myaw / 2), math.cos(myaw / 2)
    o.size.x = o.size.y = PUCK_W
    o.size.z = 0.06
    o.details = [KeyValue(key="posture", value="standing"), KeyValue(key="shape", value="box"),
                 KeyValue(key="yaw_period", value="90")]
    if not pick_cli.wait_for_server(timeout_sec=10.0):
        return False, "/skills/pick not up"
    fut = pick_cli.send_goal_async(Skill.Goal(target=str(o.id), tag=-1, shape="box", objects=[o]))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    gh = fut.result()
    if gh is None or not gh.accepted:
        return False, "rejected"
    res = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res, timeout_sec=90.0)
    if not res.done():
        gh.cancel_goal_async()
        return False, "timeout"
    return res.result().result.success, res.result().result.reason


def open_gripper():
    if not grip_cli.wait_for_server(timeout_sec=5.0):
        print("  gripper action not up — open it yourself")
        return
    g = ParallelGripperCommand.Goal()
    g.command.position = [0.0265]  # finger fully open [m] — releases the puck
    fut = grip_cli.send_goal_async(g)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    spin(1.5)


def stand_goal(target, n):
    """stand through the coordinator (/task/sub_goals) → (ok, reason, seconds)"""
    plan_id = int(time.time()) % 1000000 + n
    spin(0.5)  # 코디네이터가 새 퍽 위치를 받은 뒤에 goal
    g = SubGoal(step=0, skill="stand", target=str(target), relation="preview" if args.preview else "", offset_m=0.0)
    goal_pub.publish(SubGoalArray(plan_id=plan_id, goals=[g]))
    t0 = time.time()
    while plan_id not in status or status[plan_id].state == TaskStatus.RUNNING:
        spin(0.1)
        if time.time() - t0 > 120.0:
            print("  timeout waiting for /task/status")
            break
    st = status.get(plan_id)
    return bool(st and st.state == TaskStatus.SUCCEEDED), (st.reason if st else "timeout"), round(time.time() - t0, 1)


LOOK = {"ok": False}  # the last look_verdict reached the look pose (the inspect pose: cameras see the spot from here)


def look_verdict(n, cx, cy, yaw_b, tag):
    """inspect look pose → depth verdict (see stand_verdict)"""
    ok, why = run_skill("pose", Skill.Goal(target="look"))
    LOOK["ok"] = ok
    if not ok:
        print(f"  pose look: {why} — depth from where the hand is")
    spin(0.5)
    BAG_DIR.mkdir(parents=True, exist_ok=True)
    v = stand_verdict(node.get_clock().now().nanoseconds, cx, cy, yaw_b, save=BAG_DIR / f"{tag}_depth.npy")
    print(f"  depth: {v[0]}  (p95 {v[1]} mm above the box)")
    return v


def auto_run():
    """--auto: the lying puck at the spot → stand (retries) → pick → load into --slot (-1: hand it over), then stop"""
    path = BAG_SH.parent / "logs" / f"stand_auto_{STAMP}.csv"
    fields = ["puck", "attempt", "puck_id", "seen_x_m", "seen_y_m", "stand", "stand_reason", "stand_s", "cam_h_mm", "cam_verdict",
              "pick", "pick_reason", "load", "load_reason"] + PARAMS
    fa = open(path, "w", newline="")
    wa = csv.DictWriter(fa, fieldnames=fields)
    wa.writeheader()
    print(f"log {path}")
    input("\nauto: one lying puck on the box at --x/--y, hands clear. Enter = start ")
    k, rb = 0, None
    try:
        while True:  # one puck: every path below ends in a break
            k += 1
            rb = RoundBag(k)
            done = None
            LOOK["ok"] = False
            for attempt in range(1, args.retries + 2):
                row = dict(puck=k, attempt=attempt, **stand_params())
                # retry right after a depth check: the hand is already at the look pose, so skip stow + inspect (about 4 s)
                #   and read the puck that fell over from fresh detections; inspect only when it is not seen from here
                direct = attempt > 1 and LOOK["ok"]
                if direct:
                    tracked["msg"] = None
                    print("  retry from the look pose (no stow/inspect)")
                else:
                    inspect_spot()
                LOOK["ok"] = False
                lying = real_puck(timeout=3.0)
                if lying is None and direct:
                    print("  not seen from the look pose — inspect")
                    inspect_spot()
                    lying = real_puck(timeout=3.0)
                if lying is None:
                    up = real_puck(timeout=2.0, posture_want="standing")
                    if up is None:
                        print(f"[puck {k}] no puck at the spot — done" if attempt == 1 else f"[puck {k}] puck lost")
                        done = "none" if attempt == 1 else "lost"
                        wa.writerow({**row, "stand": "none"})
                        break
                    pid, bx, by, yb = up
                    print(f"[puck {k}] attempt {attempt}: puck {pid} reads as standing at ({bx:.3f}, {by:.3f}) — check with depth")
                    row.update(puck_id=pid, seen_x_m=round(bx, 4), seen_y_m=round(by, 4))
                    verdict, h_mm, top = look_verdict(k, bx, by, yb, f"puck{k:02d}_try{attempt}_pre")
                    row.update(cam_h_mm=h_mm, cam_verdict=f"pre_{verdict}")
                    if verdict == "lying":  # perception said standing (or unknown) but the depth says lying: stand it
                        lying = up
                    elif verdict != "standing" or top is None:
                        print(f"[puck {k}] cannot confirm the posture ({verdict}) — stop")
                        done = "unsure"
                        wa.writerow(row)
                        break
                if lying is not None:
                    pid, cx, cy, yb = lying
                    print(f"[puck {k}] attempt {attempt}: lying puck {pid} at ({cx:.3f}, {cy:.3f}) -> stand")
                    ok, why, secs = stand_goal(pid, k * 10 + attempt)
                    print(f"  stand: {'ok' if ok else 'FAIL ' + why} ({secs} s)")
                    row.update(puck_id=pid, seen_x_m=round(cx, 4), seen_y_m=round(cy, 4), stand="ok" if ok else "fail",
                               stand_reason=why, stand_s=secs)
                    verdict, h_mm, top = look_verdict(k, cx, cy, yb, f"puck{k:02d}_try{attempt}")
                    row.update(cam_h_mm=h_mm, cam_verdict=verdict)
                    if verdict == "gone":
                        print(f"[puck {k}] puck not at the stand spot any more — stop")
                        done = "gone"
                        wa.writerow(row)
                        break
                    if verdict != "standing" or top is None:
                        wa.writerow(row)
                        continue  # lying again: inspect and stand again
                pok, pwhy = pick_standing(k * 10 + attempt, top[0], top[1], yb)
                row.update(pick="ok" if pok else "fail", pick_reason=pwhy)
                print(f"  pick: {'ok' if pok else 'FAIL ' + pwhy}")
                if not pok:
                    open_gripper()
                    wa.writerow(row)
                    LOOK["ok"] = False  # the hand moved during the pick: inspect again
                    continue
                if args.slot >= 0:
                    lok, lwhy = run_skill("load", Skill.Goal(slot=args.slot, shape="box", tag=-1), timeout=90.0)
                    row.update(load="ok" if lok else "fail", load_reason=lwhy)
                    print(f"  load slot {args.slot}: {'ok' if lok else 'FAIL ' + lwhy}")
                wa.writerow(row)
                fa.flush()
                done = "loaded" if args.slot >= 0 and row["load"] == "ok" else "picked"
                break
            fa.flush()
            rb.stop()
            if done == "picked":
                input("  puck is in the hand (no load / load failed) — take it out, then Enter opens the gripper ")
                open_gripper()
                break
            if done is None:
                print(f"[puck {k}] not stood up after {args.retries + 1} attempts — stop")
            break
    except KeyboardInterrupt:
        pass
    finally:
        if rb:
            rb.stop()
        fa.close()
    print("done")


if not args.perception:
    node.create_timer(0.1, lambda: obj_pub.publish(fake_objects()))
spin(1.0)
print("open the gripper (a puck may still be in the hand from the last run)")
open_gripper()
if args.auto:
    auto_run()
    raise SystemExit(0)

n = 0
bag = None
CSV_PATH.parent.mkdir(exist_ok=True)
fh = open(CSV_PATH, "w", newline="")
w = csv.DictWriter(fh, fieldnames=["round", "result", "reason", "seconds", "standing", "x_m", "y_m", "axis_deg",
                                   "puck_id", "seen_x_m", "seen_y_m", "cam_h_mm", "cam_verdict", "pick", "pick_reason"] + PARAMS)
w.writeheader()
print(f"log {CSV_PATH}")
try:
    while args.rounds == 0 or n < args.rounds:
        n += 1
        plan_id = int(time.time()) % 1000000 + n
        if n == 1:
            input(f"\n[round {n}] put the puck lying at the spot, hands clear. Enter = start ")
        elif not args.ask:
            spin(args.pause)
        sp = stand_params()
        # odom 추정이 흔들리면 처음에 odom에 고정한 퍽이 로봇 기준으로 옆으로 밀림 — 라운드마다 지금 로봇 자세에서 다시 계산
        _, px, py, ayaw = world_pose()
        target, seen, yaw_b = PUCK_ID, ("", ""), math.radians(args.axis_deg)
        cx, cy = args.x, args.y
        if args.perception:
            bag = RoundBag(n)  # the inspect is in the bag too
            inspect_spot()
            found = real_puck()
            if found is None:
                bag.stop()
                print(f"[round {n}] no tracked lying puck within {args.match_m * 1000:.0f} mm of the spot — skipped")
                w.writerow(dict(round=n, result="SKIPPED", reason="no_lying_puck", x_m=args.x, y_m=args.y, axis_deg=args.axis_deg))
                fh.flush()
                if args.ask:
                    input(f"[round {n}] fix the puck / perception, Enter = retry ")
                continue
            target, seen, yaw_b = found[0], (round(found[1], 4), round(found[2], 4)), found[3]
            cx, cy = found[1], found[2]
            print(f"  real puck id {target} at base ({seen[0]:.3f}, {seen[1]:.3f})")
        else:
            bag = RoundBag(n)
        spin(0.5)  # 코디네이터가 새 퍽 위치를 받은 뒤에 goal
        g = SubGoal(step=0, skill="stand", target=str(target), relation="preview" if args.preview else "", offset_m=0.0)
        msg = SubGoalArray(plan_id=plan_id, goals=[g])
        goal_pub.publish(msg)
        t0 = time.time()
        while plan_id not in status or status[plan_id].state == TaskStatus.RUNNING:
            spin(0.1)
            if time.time() - t0 > 120.0:
                print("  timeout waiting for /task/status")
                break
        s = status.get(plan_id)
        print(f"[round {n}] {'SUCCEEDED' if s and s.state == TaskStatus.SUCCEEDED else 'FAILED'}"
              f"{' ' + s.reason if s and s.reason else ''}  ({time.time() - t0:.0f} s)")
        spin(1.0)  # hand settles after the release / lift -> into the bag
        # depth look from the inspect look pose: right after the lift the camera is too close to a STANDING puck — its
        # upper half has no depth (D435 min range), so it read as 35 mm = lying (11/08 round, puck was standing)
        ok, why = run_skill("pose", Skill.Goal(target="look"))
        if not ok:
            print(f"  pose look: {why} — depth from where the hand is")
        spin(0.5)
        BAG_DIR.mkdir(parents=True, exist_ok=True)
        verdict, h_mm, top_xy = stand_verdict(node.get_clock().now().nanoseconds, cx, cy, yaw_b,
                                              save=BAG_DIR / f"round_{n:02d}_depth.npy")
        print(f"  depth: {verdict}  (p95 {h_mm} mm above the box)")
        standing = ""
        if args.ask:
            standing = input(f"[round {n}] puck stood up? y/n ").strip()[:1].lower()
        pick, pick_reason = "", ""
        if args.pick and verdict == "standing" and top_xy:
            print(f"  pick: standing puck at base ({top_xy[0]:.3f}, {top_xy[1]:.3f})")
            ok, pick_reason = pick_standing(n, top_xy[0], top_xy[1], yaw_b)
            pick = "ok" if ok else "failed"
            print(f"  pick: {pick} {pick_reason}")
            spin(1.0)
        bag.stop()
        if pick == "ok":
            input(f"[round {n}] take the puck out of the hand, Enter = open the gripper ")
            open_gripper()
        if args.ask:
            input(f"[round {n}] put the puck back lying at the spot, Enter = round {n + 1} ")
        w.writerow(dict(round=n, result="SUCCEEDED" if s and s.state == TaskStatus.SUCCEEDED else "FAILED",
                        reason=s.reason if s else "timeout", seconds=round(time.time() - t0, 1), standing=standing,
                        x_m=args.x, y_m=args.y, axis_deg=args.axis_deg, puck_id=target, seen_x_m=seen[0],
                        seen_y_m=seen[1], cam_h_mm=h_mm, cam_verdict=verdict, pick=pick, pick_reason=pick_reason, **sp))
        fh.flush()
except KeyboardInterrupt:
    pass
finally:
    if bag:
        bag.stop()
    fh.close()
print("done")
