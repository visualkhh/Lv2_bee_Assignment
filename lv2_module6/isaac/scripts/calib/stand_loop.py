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
"""

import argparse
import math
import os
import signal
import subprocess
import time

import rclpy
from diagnostic_msgs.msg import KeyValue
from plan.msg import SubGoal, SubGoalArray, TaskStatus
from perception.msg import TrackedObject, TrackedObjectArray
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

parser = argparse.ArgumentParser()
parser.add_argument("--x", type=float, default=0.22, help="puck centre ahead of base_footprint [m] (arm reach 0.13-0.31)")
parser.add_argument("--y", type=float, default=0.0, help="puck centre left of base_footprint [m]")
parser.add_argument("--axis-deg", type=float, default=0.0, help="puck long axis, deg from robot forward (0 = pointing away)")
parser.add_argument("--support", type=float, default=0.07, help="surface height [m] (box top 0.07)")
parser.add_argument("--shape", choices=["box", "cyl"], default="box")
parser.add_argument("--rounds", type=int, default=0, help="0 = until Ctrl-C")
parser.add_argument("--pause", type=float, default=6.0, help="s between rounds (hand lifted) to put the puck back")
parser.add_argument("--preview", action="store_true", help="only hover above the grasp point")
args = parser.parse_args()

PUCK_ID = 99
PUCK_W = 0.030

# Clean up before starting: other stand_loop.py copies (Ctrl-Z leaves them stopped, and each one is a DDS participant /
# fake-puck publisher) and perception_node (it would publish /tracked_objects too).
for pid in subprocess.run(["pgrep", "-f", "stand_loop.py"], capture_output=True, text=True).stdout.split():
    if int(pid) != os.getpid():
        os.kill(int(pid), signal.SIGCONT)  # a stopped process only handles SIGTERM after it is continued
        os.kill(int(pid), signal.SIGTERM)
subprocess.run(["pkill", "-f", "perception_node"])

rclpy.init()
node = rclpy.create_node("stand_loop")
tf_buf = Buffer()
TransformListener(tf_buf, node)
latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
status = {}
node.create_subscription(TaskStatus, "/task/status", lambda m: status.update({m.plan_id: m}), latched)
goal_pub = node.create_publisher(SubGoalArray, "/task/sub_goals", latched)
obj_pub = node.create_publisher(TrackedObjectArray, "/tracked_objects", QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))


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


node.create_timer(0.1, lambda: obj_pub.publish(fake_objects()))
spin(1.0)

n = 0
try:
    while args.rounds == 0 or n < args.rounds:
        n += 1
        plan_id = int(time.time()) % 1000000 + n
        input(f"\n[round {n}] put the puck lying at the spot, hands clear. Enter = start ") if n == 1 else spin(args.pause)
        # odom 추정이 흔들리면 처음에 odom에 고정한 퍽이 로봇 기준으로 옆으로 밀림 — 라운드마다 지금 로봇 자세에서 다시 계산
        _, px, py, ayaw = world_pose()
        spin(0.5)  # 코디네이터가 새 퍽 위치를 받은 뒤에 goal
        g = SubGoal(step=0, skill="stand", target=str(PUCK_ID), relation="preview" if args.preview else "", offset_m=0.0)
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
except KeyboardInterrupt:
    pass
print("done")
