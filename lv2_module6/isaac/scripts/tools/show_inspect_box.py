"""Send one 'inspect box' plan for the current start_box (Isaac trial truth) — demo of stow -> look pose -> reframe (skill_inspect target box).
  python3 isaac/scripts/tools/show_inspect_box.py               # current layout
  python3 isaac/scripts/tools/show_inspect_box.py --reset       # reset.sh (same seed) -> wait for ROS -> send
  python3 isaac/scripts/tools/show_inspect_box.py --reset 2017  # reset to that seed -> wait for ROS -> send
"""
import argparse
import json
import subprocess
import time
from pathlib import Path

import rclpy
from plan.msg import SubGoal, SubGoalArray, TaskStatus
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

TRIAL = Path('/tmp/beebot_trial.json')
RESET = Path(__file__).resolve().parents[1] / 'standalone' / 'reset.sh'

ap = argparse.ArgumentParser()
ap.add_argument('--reset', nargs='?', const=-1, type=int, metavar='SEED', help='reset first (no SEED = same layout again)')
a = ap.parse_args()

if a.reset is not None:
    seed = a.reset if a.reset >= 0 else json.loads(TRIAL.read_text())['seed']
    Path('/tmp/beebot_trial_seed').write_text(str(seed))
    t_req = time.time()
    subprocess.run([str(RESET)], check=True)
    while True:  # new layout written by manipulator_env.py
        try:
            d = json.loads(TRIAL.read_text())
            if d['seed'] == seed and d['t'] > t_req:
                break
        except (OSError, ValueError, KeyError):
            pass
        time.sleep(0.5)
    print(f'reset to seed {seed} — waiting for ROS', flush=True)

trial = json.loads(TRIAL.read_text())
x, y = trial['box']
rclpy.init()
n = rclpy.create_node('show_inspect_box')
latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
pub = n.create_publisher(SubGoalArray, '/task/sub_goals', latched)
status = {}
n.create_subscription(TaskStatus, '/task/status', lambda m: status.update(m=m), latched)


def ready():  # coordinator listening and the inspect skill + MoveIt up
    topics = {t for t, _ in n.get_topic_names_and_types()}
    return pub.get_subscription_count() > 0 and '/skills/inspect/_action/status' in topics and \
        '/plan_kinematic_path' in {s for s, _ in n.get_service_names_and_types()}


while not ready():
    rclpy.spin_once(n, timeout_sec=0.5)
time.sleep(3.0 if a.reset is not None else 0.0)  # pose skill moves the arm to stow right after bringup
plan = SubGoalArray(plan_id=int(time.time()) % 100000 * 10 + 7,
                    goals=[SubGoal(step=1, skill='inspect', target=f'{x:.4f},{y:.4f},0.13', relation='box')])
plan.header.stamp = n.get_clock().now().to_msg()
pub.publish(plan)
print(f"seed {trial['seed']} gap {100 * trial['gap']:.1f} cm — inspect box at {x:.3f},{y:.3f}", flush=True)
t0 = time.time()
while time.time() - t0 < 60:  # until the plan finishes
    rclpy.spin_once(n, timeout_sec=0.2)
    m = status.get('m')
    if m and m.plan_id == plan.plan_id and m.state != TaskStatus.RUNNING:
        print(f"done: {'success' if m.state == TaskStatus.SUCCEEDED else 'failed ' + m.reason} ({time.time() - t0:.1f} s)", flush=True)
        break
