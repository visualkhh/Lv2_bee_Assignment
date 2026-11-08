"""joint1 zero check: is the arm's yaw where the encoder says? Puts the hand straight down 0.20 m out from the joint1 axis
and turns only joint1 through the given angles. At each angle you type the ruler-measured fingertip centre (between the
pads) in base_footprint (x forward from the wheel axle midpoint, y left +); it prints the yaw about the joint1 axis the
encoder/URDF predicts (TF end_effector_link) vs. what you measured. A constant difference = joint1 zero offset.

    ROS_DOMAIN_ID=88 python3 joint1_check.py                    (launcher up, box removed from in front of the robot)
    ROS_DOMAIN_ID=88 python3 joint1_check.py --height box       (tip 6 cm over a 7 cm box top instead of 8 cm over the floor)
Ends back at stow.
"""

import argparse
import math
import statistics as st
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5"]
J1_XY = (0.047985, -0.00023)  # joint1 axis in base_footprint (URDF)
# hand down, fingertip 0.20 m out from the joint1 axis (skills stand::ik_down, offline)
REACH = {"floor": [0.6069, -0.0817, 1.0456, 0.0],   # fingertip z 0.08 m (floor)
         "box": [0.4614, -0.2613, 1.3707, 0.0]}     # fingertip z 0.13 m (6 cm over a 7 cm box)
STOW = [0.0, -1.9024, 1.344, 0.9023, 0.0]

ap = argparse.ArgumentParser()
ap.add_argument("--height", choices=list(REACH), default="floor")
ap.add_argument("--angles", default="0 20 -20 40 -40 0", help="joint1 angles [deg]; keep within +-45 (body, wheels)")
ap.add_argument("--move-s", type=float, default=4.0, help="time per move [s]")
args = ap.parse_args()

rclpy.init()
node = rclpy.create_node("joint1_check")
tf_buf = Buffer()
TransformListener(tf_buf, node)
state = {}
node.create_subscription(JointState, "/joint_states", lambda m: state.update(zip(m.name, m.position)), 10)
arm = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def move(q, sec):
    g = FollowJointTrajectory.Goal()
    g.trajectory.joint_names = JOINTS
    g.trajectory.points = [JointTrajectoryPoint(positions=[float(v) for v in q],
                                                time_from_start=Duration(sec=int(sec), nanosec=int(sec % 1 * 1e9)))]
    fut = arm.send_goal_async(g)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    gh = fut.result()
    if gh is None or not gh.accepted:
        raise SystemExit("FAIL: arm goal rejected")
    res = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res, timeout_sec=sec + 10.0)
    spin(1.5)  # settle (sag integral, gravity feedforward)


def tip():
    t = tf_buf.lookup_transform("base_footprint", "end_effector_link", Time()).transform.translation
    return t.x, t.y


def yaw(x, y):
    return math.degrees(math.atan2(y - J1_XY[1], x - J1_XY[0]))


if not arm.wait_for_server(timeout_sec=10.0):
    raise SystemExit("FAIL: /arm_controller/follow_joint_trajectory not up (launcher running?)")
spin(1.0)
errs = []
try:
    for a in [float(v) for v in args.angles.split()]:
        q = [math.radians(a)] + REACH[args.height]
        print(f"\njoint1 {a:+.0f} deg ...")
        move(q, args.move_s)
        px, py = tip()
        enc = math.degrees(state.get("joint1", float("nan")))
        print(f"  encoder joint1 {enc:+.2f} deg | URDF fingertip x {px * 1000:.1f} y {py * 1000:.1f} mm (yaw {yaw(px, py):+.2f} deg)")
        s = input("  ruler fingertip centre 'x y' mm (y left +; Enter = skip, q = stop): ").strip()
        if s == "q":
            break
        if not s:
            continue
        mx, my = (float(v) for v in s.replace(",", " ").split()[:2])
        d = yaw(mx / 1000, my / 1000) - yaw(px, py)
        r = math.hypot(px - J1_XY[0], py - J1_XY[1]) * 1000
        print(f"  measured yaw {yaw(mx / 1000, my / 1000):+.2f} deg -> error {d:+.2f} deg (+ = arm is more to the left), "
              f"radius URDF {r:.0f} mm / measured {math.hypot(mx - J1_XY[0] * 1000, my - J1_XY[1] * 1000):.0f} mm")
        errs.append(d)
finally:
    if errs:
        print(f"\njoint1 yaw error over {len(errs)} angles: mean {st.mean(errs):+.2f} deg"
              + (f", sd {st.stdev(errs):.2f}" if len(errs) > 1 else "")
              + "  (constant across angles = zero offset; growing with angle = scale / backlash)")
    print("back to stow ...")
    move(STOW, args.move_s)
