#!/usr/bin/env python3
"""Gravity-sag measurement: the arm goes to a pose and holds, you measure the fingertip height with a tape and type it in,
the script then logs joint states, the rl_arm_correction state and the TF fingertip height next to your number.

  ros2 launch ./arm_cam.launch.py      # terminal 1 (arm up, nothing else)
  python3 sag_measure.py [--sag on|off|keep] [--poses grasp above ...] [--rounds 2]     # terminal 2

Per pose: confirm move -> arm moves (reset_sag first, like a stand skill) -> settles -> you measure -> type mm + Enter -> logged.
Measure the finger-tip face (the flat bottom of the hand) at two points on the line along the robot's x axis (forward /
backward): `front rear` [mm above the floor the robot stands on], e.g. `131.5 128`. --span is the distance between the two
points [mm]; with --span 0 you type one number instead (centre of the face, no tilt data).
end_effector_link sits FINGER_TIP (20.6 mm) above the face centre (the mean of the two points), so ee_tape = centre + 20.6 and
  sag  = ee_tape - tf_z                  (negative: the hand is lower than the TF/FK says; this is what the encoders cannot see)
  tilt = atan2(rear - front, span)       (positive: the front is lower, nose down) - the hand pitch, sum of the joint2-4 twists
Enter alone skips the measurement (state is still logged), q stops the test. Writes logs/sag_measure/<time>/log.csv.
Poses are reference joint values taken from stand runs (hand pointing down), reach x 0.135 - 0.249 m.

Stiffness test (--grid): poses on an x/z/pitch grid from the URDF IK (arm_model.py), each reached via the same
--via pose so backlash does not depend on the approach path; run with --sag off. Fit the joint stiffness with sag_fit.py.
"""
import argparse
import csv
import math
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
FINGER_TIP = 0.0206  # [m] end_effector_link above the lowest finger-tip point (plan/stand_plan.hpp)
MAX_SPEED_DEG = 15.0  # [deg/s] joint speed of the moves
POSES = {  # [deg] joint1..5, reference values of a stand run (hand down); ee z in base_footprint from the TF
    'above': [0.49, 26.09, -23.96, 88.47, 0.34],   # x 0.247 z 0.147  above the grasp point
    'grasp': [0.06, 29.78, -11.31, 71.52, 0.06],   # x 0.249 z 0.109  fingers at the puck
    'near_mid': [0.15, -22.68, 46.72, 65.96, 0.15],  # x 0.135 z 0.129  short reach, arm folded
    'near_far': [0.11, -11.19, 35.15, 66.05, 0.12],  # x 0.160 z 0.130
}
SEQUENCE = ['above', 'grasp', 'near_mid', 'near_far', 'above', 'near_far', 'grasp']  # visits each pose from above and below
OUT = Path(__file__).resolve().parent / 'logs' / 'sag_measure'
NS = '/rl_arm_correction'

parser = argparse.ArgumentParser()
parser.add_argument('--sag', choices=['on', 'off', 'keep'], default='keep', help='set rl_arm_correction sag_enabled first')
parser.add_argument('--poses', nargs='+', help='only these, in this order (default: the full sequence; with --grid: all grid poses)')
parser.add_argument('--grid', action='store_true', help='pose grid from the URDF IK (for sag_fit.py), hand pitched by --pitches')
parser.add_argument('--pitches', nargs='+', type=float, default=[-30.0, 0.0, 30.0], help='--grid: hand pitch from straight down [deg]; varying it moves the gravity torque of joint 3/4 independently of joint 2 (k_3 / k_4)')
parser.add_argument('--via', help='go through this pose before every target (same approach path every time)')
parser.add_argument('--rounds', type=int, default=1, help='repeat the sequence')
parser.add_argument('--settle', type=float, default=4.0, help='s to wait after arriving before you measure')
parser.add_argument('--sample', type=float, default=1.0, help='s of states averaged at Enter')
parser.add_argument('--span', type=float, help='distance between the front and rear measuring points [mm]; 0 = one number per pose (asked if omitted)')
parser.add_argument('--dry', action='store_true', help='print the plan only')
args = parser.parse_args()
from arm_model import Arm
arm = Arm()
if args.grid:
    for x in (0.15, 0.19, 0.23):    # [m] finger-tip face centre ahead of base_footprint, y = 0
        for z in (0.10, 0.13, 0.16):  # [m] face centre height
            for pitch in args.pitches:
                v = arm.hand_ik(x, z, pitch)
                if v is not None and math.degrees(v[0]) > -60:  # not folded back into the body
                    POSES[f'x{round(x * 100)}_z{round(z * 100)}_p{round(pitch)}'] = [0.0] + [math.degrees(a) for a in v] + [0.0]
    SEQUENCE = [n for n in POSES if n.startswith('x')]
unknown = [n for n in (args.poses or []) + ([args.via] if args.via else []) if n not in POSES]
if unknown: raise SystemExit(f'unknown poses {unknown}; available {list(POSES)}')
seq = (args.poses or SEQUENCE) * args.rounds
if args.dry:
    for i, n in enumerate(seq): print(i + 1, n, POSES[n])
    raise SystemExit

rclpy.init()
node = rclpy.create_node('sag_measure')
tf_buf = Buffer()
TransformListener(tf_buf, node)
best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
latest = {}
node.create_subscription(JointState, '/joint_states', lambda m: latest.update(js=m), 50)
node.create_subscription(Float64MultiArray, f'{NS}/state', lambda m: latest.update(st=m.data), best_effort)
client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
reset_cli = node.create_client(Trigger, f'{NS}/reset_sag')


def spin(sec):
    end = time.time() + sec
    while time.time() < end: rclpy.spin_once(node, timeout_sec=0.02)


def param(name, value=None):
    cmd = ['ros2', 'param', 'get' if value is None else 'set', f'{NS}', name] + ([] if value is None else [str(value)])
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=10).stdout.strip()
    return out.split(':')[-1].strip() if value is None else out


def joint_vec(js, names):
    idx = {n: i for i, n in enumerate(js.name)}
    return [js.position[idx[n]] for n in names], [js.effort[idx[n]] if idx[n] < len(js.effort) else math.nan for n in names]


def move(target_deg):
    js = latest['js']
    now_rad, _ = joint_vec(js, JOINTS)
    tgt = np.radians(target_deg)
    sec = max(3.0, float(np.max(np.abs(np.degrees(tgt - np.array(now_rad))))) / MAX_SPEED_DEG)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = JointTrajectory(joint_names=JOINTS)
    goal.trajectory.points = [JointTrajectoryPoint(positions=tgt.tolist(), time_from_start=Duration(
        sec=int(sec), nanosec=int((sec % 1) * 1e9)))]
    fut = client.send_goal_async(goal)
    while not fut.done(): rclpy.spin_once(node, timeout_sec=0.02)
    handle = fut.result()
    if not handle.accepted: raise SystemExit('FAIL: goal rejected')
    res = handle.get_result_async()
    while not res.done(): rclpy.spin_once(node, timeout_sec=0.02)
    return sec


def sample(sec):
    """average joint states, rl_arm_correction state and the TF fingertip over sec"""
    q, eff, st, ee = [], [], [], []
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
        js = latest.get('js')
        if js:
            v, e = joint_vec(js, JOINTS + ['gripper_left_joint'])
            q.append(v); eff.append(e)
        if latest.get('st') is not None: st.append(list(latest['st']))
        try:
            t = tf_buf.lookup_transform('base_footprint', 'end_effector_link', Time()).transform.translation
            ee.append([t.x, t.y, t.z])
        except Exception:
            pass
    return np.array(q), np.array(eff), np.array(st), np.array(ee)


# --- startup checks
if not client.wait_for_server(timeout_sec=10.0): raise SystemExit('FAIL: /arm_controller/follow_joint_trajectory not up (arm_cam.launch.py running?)')
spin(2.0)
if 'js' not in latest: raise SystemExit('FAIL: no /joint_states')
if args.sag != 'keep': print('sag_enabled ->', args.sag, param('sag_enabled', 'true' if args.sag == 'on' else 'false'))
sag_enabled, sag_gain = param('sag_enabled'), param('sag_gain')
print(f'sag_enabled={sag_enabled} sag_gain={sag_gain}  poses: {" -> ".join(seq)}')
if args.span is None: args.span = float(input('distance between your front and rear measuring points [mm] (0 = measure one centre point) > ') or 0)
print('Clear the arm\'s way. Each move is confirmed. Measure the LOWEST finger-tip point above the floor [mm].')

d = OUT / time.strftime('%Y%m%d_%H%M%S')
d.mkdir(parents=True)
header = (['idx', 'pose', 'from_pose', 't_unix', 'sag_enabled', 'sag_gain', 'move_s', 'arrive_s']
          + [f'ref_{j}_deg' for j in JOINTS] + [f'corr_{j}_deg' for j in JOINTS] + [f'delta_{j}_deg' for j in JOINTS]
          + [f'q_{j}_deg' for j in JOINTS] + ['gripper_pos'] + [f'q_std_{j}_deg' for j in JOINTS]
          + [f'effort_{j}' for j in JOINTS] + ['tf_x_mm', 'tf_y_mm', 'tf_z_mm', 'tf_z_std_mm',
                                                       'mark_z_fk_mm', 'pitch_deg', 'tape_front_mm', 'tape_rear_mm', 'span_mm', 'tilt_deg', 'tape_tip_mm', 'ee_tape_mm', 'sag_mm'])
rows, prev = [], '-'
try:
    for i, name in enumerate(seq):
        ans = input(f'\n[{i + 1}/{len(seq)}] next pose {name}. Enter = move, s = skip, q = quit > ').strip().lower()
        if ans == 'q': break
        if ans == 's': continue
        if reset_cli.service_is_ready(): reset_cli.call_async(Trigger.Request())
        if args.via and name != args.via:
            print(f'  via {args.via} ...')
            move(POSES[args.via])
        t_move = time.time()
        move_s = move(POSES[name])
        arrive_s = time.time() - t_move
        print(f'  arrived ({arrive_s:.1f} s), settling {args.settle:.0f} s ...')
        spin(args.settle)
        _, _, st, ee = sample(0.5)
        qm = np.array(latest['js'].position)[[latest['js'].name.index(n) for n in JOINTS]]
        print(f'  FK face centre z = {arm.mark(qm)[2] * 1000:.1f} mm  (hand pitch {math.degrees(qm[1] + qm[2] + qm[3]) - 90:+.0f} deg from down): put the tape on the centre of the finger-tip face, it should read about this')
        ask = 'front rear [mm]' if args.span > 0 else 'centre [mm]'
        tape = input(f'  tape {ask} of the finger-tip face centre above the floor (Enter = skip, q = quit) > ').strip().lower().replace(',', ' ')
        if tape == 'q': break
        q, eff, st, ee = sample(args.sample)  # states while you measured: hold continues
        row = [i + 1, name, prev, f'{time.time():.3f}', sag_enabled, sag_gain, f'{move_s:.1f}', f'{arrive_s:.1f}']
        if len(st):
            m = st.mean(0)
            row += [math.degrees(x) for x in m[0:5]] + [math.degrees(x) for x in m[5:10]] + [math.degrees(x) for x in m[10:15]]
        else:
            row += [math.nan] * 15
        row += [math.degrees(x) for x in q[:, :5].mean(0)] + [q[:, 5].mean()] + [math.degrees(x) for x in q[:, :5].std(0)]
        row += list(np.nanmean(eff[:, :5], axis=0))
        tfz = ee[:, 2].mean() * 1000 if len(ee) else math.nan
        tfm = ee.mean(0) * 1000 if len(ee) else [math.nan] * 3
        row += [tfm[0], tfm[1], tfz, ee[:, 2].std() * 1000 if len(ee) else math.nan]
        qrad = q[:, :5].mean(0)
        mark_z = arm.mark(qrad, float(q[:, 5].mean()))[2] * 1000
        row += [mark_z, math.degrees(qrad[1] + qrad[2] + qrad[3]) - 90.0]
        try:
            v = [float(x) for x in tape.split()]
            if len(v) != (2 if args.span > 0 else 1): raise ValueError
            front, rear = (v[0], v[1]) if len(v) == 2 else (math.nan, math.nan)
            tip = (front + rear) / 2 if len(v) == 2 else v[0]
            tilt = math.degrees(math.atan2(rear - front, args.span)) if len(v) == 2 else math.nan
            row += [front, rear, args.span, tilt, tip, tip + FINGER_TIP * 1000, tip - mark_z]
            print(f'  logged: centre {tip:.1f} mm, FK {mark_z:.1f} mm -> sag {tip - mark_z:+.1f} mm'
                  + (f', tilt {tilt:+.2f} deg ({"nose down" if tilt > 0 else "nose up"})' if len(v) == 2 else ''))
        except ValueError:
            row += [math.nan] * 7
            print('  logged without tape (need ' + ask + ')')
        rows.append([f'{x:.4f}' if isinstance(x, float) else x for x in row])
        prev = name
except KeyboardInterrupt:
    pass
finally:
    with open(d / 'log.csv', 'w', newline='') as f:
        w = csv.writer(f); w.writerow(header); w.writerows(rows)
    print(f'\n{len(rows)} rows -> {d}/log.csv')
    sags = [(r[1], float(r[-1])) for r in rows if r[-1] != 'nan']
    for n, s in sags: print(f'  {n:9s} sag {s:+6.1f} mm')
    if sags: print(f'  mean {np.mean([s for _, s in sags]):+.1f} mm  (n={len(sags)})')
    rclpy.shutdown()
