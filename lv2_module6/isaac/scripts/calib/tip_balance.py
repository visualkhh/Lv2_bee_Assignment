#!/usr/bin/env python3
"""Find the body centre of mass from the pose where the robot starts to tip: the arm reaches forward slowly, you press
Enter the moment the rear casters lift, the pose is logged and the arm goes back at once.

  ros2 launch sine_test hw.launch.py                # terminal 1 (real robot)
  python3 tip_balance.py [--trials 3] [--speed 2]   # terminal 2
  python3 tip_balance.py --payload 0.3             # weight [kg] held in the gripper when the arm alone can't tip it
  python3 tip_balance.py --self-test                # no ROS, no robot

At the tipping point the whole robot's centre of mass is above the wheel axle, so with the URDF masses of everything
else the base_link centre of mass x follows. Any arm mass error ends up in that number. Joint values are the encoders
(gravity twist not included). Keep a hand near the robot: the arm only comes back after you press Enter.
Writes logs/tip_balance/<time>/log.csv.
"""
import argparse
import csv
import select
import sys
import time
from pathlib import Path

import numpy as np

from arm_model import ARM, TIP, Arm

BACK_SPEED_DEG = 10.0  # [deg/s] after Enter and to the start pose
OUT = Path(__file__).resolve().parent / 'logs' / 'tip_balance'


def total_com_x(arm, q, grip=0.0):
    """whole-robot centre of mass x in base_footprint [m] and total mass [kg]"""
    frames, _ = arm.fk_all(q, grip)
    moment = sum(m * (frames[l][:3, :3] @ arm.com[l] + frames[l][:3, 3])[0] for l, m in arm.mass.items())
    mass = sum(arm.mass.values())
    return moment / mass, mass


def base_com_at_tip(arm, q, grip=0.0):
    """base_link centre of mass x [m, base_link frame] that puts the whole robot's above the wheel axle in pose q"""
    frames, _ = arm.fk_all(q, grip)
    rest = sum(m * (frames[l][:3, :3] @ arm.com[l] + frames[l][:3, 3])[0] for l, m in arm.mass.items() if l != 'base_link')
    axle = frames['wheel_left_link'][0, 3]
    world = (axle * sum(arm.mass.values()) - rest) / arm.mass['base_link']
    return world - frames['base_link'][0, 3]


def add_payload(arm, kg):
    """point mass at the finger-tip face centre (arm_model.mark); end_effector_link has no inertial of its own"""
    if kg > 0:
        arm.mass['end_effector_link'], arm.com['end_effector_link'] = kg, np.array([0.0, 0.0, TIP])


def self_test():
    arm = Arm()
    q = [0.0, -0.4, 0.2, 0.1, 0.0]
    arm.com['base_link'] = arm.com['base_link'].copy()
    arm.com['base_link'][0] = base_com_at_tip(arm, q)
    x, _ = total_com_x(arm, q)
    assert abs(x - arm.fk_all(q)[0]['wheel_left_link'][0, 3]) < 1e-9
    far, _ = total_com_x(arm, [0.0] * 5)
    near, _ = total_com_x(arm, [0.0, -1.9024, 1.344, 0.9023, 0.0])
    assert far > near  # reaching out moves the centre of mass forward
    reach = [0.0, np.radians(80), np.radians(-80), 0.0, 0.0]  # furthest forward centre of mass inside the limits
    bare = base_com_at_tip(arm, reach)
    add_payload(arm, 0.3)
    assert base_com_at_tip(arm, reach) < bare  # a weight in the hand tips a robot whose base mass sits further back
    print('self-test ok')


def main(args):
    import rclpy
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    arm = Arm()
    add_payload(arm, args.payload)
    rclpy.init()
    node = rclpy.create_node('tip_balance')
    latest = {}
    node.create_subscription(JointState, '/joint_states', lambda m: latest.update(js=m), 10)
    client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    def state():
        js = latest['js']
        pos = dict(zip(js.name, js.position))
        return np.array([pos[j] for j in ARM]), pos.get('gripper_left_joint', 0.0)

    def send(target, speed_deg):
        """one goal from the current pose to target; returns the goal's result future"""
        now, grip = state()
        sec = max(1.0, float(np.max(np.abs(np.degrees(np.asarray(target) - now)))) / speed_deg)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(joint_names=[*ARM, 'gripper_left_joint'])  # hw.launch.py arm_controller also has the gripper: holds
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[*(float(v) for v in target), grip], time_from_start=Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9)))]
        fut = client.send_goal_async(goal)
        while not fut.done():
            spin(0.01)
        if not fut.result().accepted:
            sys.exit('FAIL: goal rejected')
        return fut.result().get_result_async()

    def wait(result):
        while not result.done():
            spin(0.02)

    end = time.time() + 5.0
    while 'js' not in latest and time.time() < end:
        spin(0.1)
    if 'js' not in latest:
        sys.exit('FAIL: no /joint_states (is hw.launch.py up?)')
    if not client.wait_for_server(timeout_sec=10.0):
        sys.exit('FAIL: /arm_controller/follow_joint_trajectory not up')

    start, target = np.radians(args.start), np.radians(args.target)
    print(f'now [deg] {np.round(np.degrees(state()[0]), 1).tolist()} -> start {args.start} -> reach {args.target} at {args.speed} deg/s')
    print(f'model base_link com x {arm.com["base_link"][0] * 1000:.1f} mm')
    rows = []
    for k in range(args.trials):
        if input(f'\n[{k + 1}/{args.trials}] Enter = go to the start pose, q = stop: ').strip().lower() == 'q':
            break
        wait(send(start, BACK_SPEED_DEG))
        spin(1.0)  # let it settle so every trial starts the same
        input('  Hand near the robot. Enter = reach out, then Enter again the moment it starts to tip: ')
        result = send(target, args.speed)
        tipped = False
        while not result.done():
            spin(0.01)
            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                tipped = True
                break
        q, grip = state()  # the pose at Enter, before the arm turns back
        send(start, BACK_SPEED_DEG)  # replaces the running goal: the arm starts back right away
        if not tipped:
            print(f'  reached {args.target} without tipping: base_link com x < {base_com_at_tip(arm, q, grip) * 1000:.1f} mm, '
                  f'not a measurement (add/raise --payload)')
            continue
        com = base_com_at_tip(arm, q, grip)
        model, _ = total_com_x(arm, q, grip)
        print(f'  pose [deg] {np.round(np.degrees(q), 2).tolist()}: base_link com x {com * 1000:.1f} mm, '
              f'the model has the whole robot at {model * 1000:.1f} mm here')
        rows.append([k + 1, args.speed, *q.tolist(), grip, com, model])
    wait(send(start, BACK_SPEED_DEG))
    if rows:
        out = OUT / time.strftime('%Y%m%d_%H%M%S')
        out.mkdir(parents=True, exist_ok=True)
        with open(out / 'log.csv', 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow([f'# payload {args.payload} kg'])
            w.writerow(['trial', 'speed_deg_s', *ARM, 'gripper_left_joint', 'base_com_x_m', 'model_total_com_x_m'])
            w.writerows(rows)
        coms = np.array([r[-2] for r in rows]) * 1000
        print(f'\nbase_link com x {coms.mean():.1f} mm (spread {coms.max() - coms.min():.1f} mm, {len(rows)} trials), '
              f'model {arm.com["base_link"][0] * 1000:.1f} mm -> {out / "log.csv"}')
    rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--trials', type=int, default=3)
    parser.add_argument('--speed', type=float, default=2.0, help='fastest joint while reaching out [deg/s]')
    parser.add_argument('--payload', type=float, default=0.0, help='mass held in the gripper [kg]; weigh it first')
    parser.add_argument('--start', type=float, nargs=5, default=[0.0, -109.0, 77.0, 51.7, 0.0], help='joint1..5 [deg], default stow')
    parser.add_argument('--target', type=float, nargs=5, default=[0.0, 0.0, 0.0, 0.0, 0.0], help='joint1..5 [deg], default straight out')
    cli = parser.parse_args()
    if cli.self_test:
        self_test()
    else:
        main(cli)
