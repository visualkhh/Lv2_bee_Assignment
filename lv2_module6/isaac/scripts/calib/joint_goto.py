#!/usr/bin/env python3
"""Move some arm joints to an angle [deg], the others hold their current angle (arm_controller needs all 5 joints).

  python3 joint_goto.py joint4=0            # joint4 -> 0 deg
  python3 joint_goto.py joint2=0 joint3=0   # several at once
  python3 joint_goto.py                     # nothing to move: prints the current angles
  python3 joint_goto.py tip=0.19,0.07       # hand straight down, finger-tip face centre 0.19 m ahead, 0.07 m above floor
  python3 joint_goto.py tip=0.19,0.07,10    # same, hand pitched 10 deg from straight down

tip= solves joint2..4 with the URDF IK (arm_model.hand_ik, base_footprint: z = floor) and sets joint1 = joint5 = 0.
Encoder angles only: the real tip ends up lower by the gravity sag unless rl_arm_correction compensates it.

Joint speed is limited to 15 deg/s (at least 3 s). Needs arm_cam.launch.py (or the main bringup) running.
"""
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
MAX_SPEED_DEG = 15.0

goal = {}
for a in sys.argv[1:]:
    name, _, val = a.partition('=')
    if name == 'tip':
        from arm_model import Arm
        x, z, *pitch = (float(v) for v in val.split(','))
        v = Arm().hand_ik(x, z, pitch[0] if pitch else 0.0)
        if v is None: sys.exit(f'FAIL: no IK solution for tip x={x} z={z}')
        goal.update(joint1=0.0, joint5=0.0, **{j: float(np.degrees(d)) for j, d in zip(('joint2', 'joint3', 'joint4'), v)})
        print('tip IK [deg]:', {j: round(d, 2) for j, d in goal.items()})
        continue
    if name not in JOINTS or not val: sys.exit(f'bad argument {a!r}: use jointN=<deg>, N = 1..5, or tip=x,z[,pitch]')
    goal[name] = float(val)

rclpy.init()
node = rclpy.create_node('joint_goto')
latest = {}
node.create_subscription(JointState, '/joint_states', lambda m: latest.update(js=m), 10)
end = time.time() + 5.0
while 'js' not in latest and time.time() < end: rclpy.spin_once(node, timeout_sec=0.1)
if 'js' not in latest: sys.exit('FAIL: no /joint_states')
idx = {n: i for i, n in enumerate(latest['js'].name)}
now = np.array([latest['js'].position[idx[j]] for j in JOINTS])
print('now [deg]:', {j: round(float(np.degrees(v)), 2) for j, v in zip(JOINTS, now)})
if not goal: sys.exit()

tgt = now.copy()
for j, d in goal.items(): tgt[JOINTS.index(j)] = np.radians(d)
sec = max(3.0, float(np.max(np.abs(np.degrees(tgt - now)))) / MAX_SPEED_DEG)
client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
if not client.wait_for_server(timeout_sec=10.0): sys.exit('FAIL: /arm_controller/follow_joint_trajectory not up')
g = FollowJointTrajectory.Goal()
g.trajectory = JointTrajectory(joint_names=JOINTS)
g.trajectory.points = [JointTrajectoryPoint(positions=tgt.tolist(), time_from_start=Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9)))]
print(f'moving {goal} in {sec:.1f} s ...')
fut = client.send_goal_async(g)
rclpy.spin_until_future_complete(node, fut)
if not fut.result().accepted: sys.exit('FAIL: goal rejected')
res = fut.result().get_result_async()
rclpy.spin_until_future_complete(node, res)
print('done, error code', res.result().result.error_code, '(0 = ok)')
rclpy.shutdown()
