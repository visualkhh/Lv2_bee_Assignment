"""Demo: stow -> inspect box (look pose, reframe if needed) -> pick + load each puck one by one (isaac_manipulator_demo.launch.py running).
  Shows whether the arm goes straight from one load to the next pick (skill_load load_end_pose).
  python3 isaac/scripts/tools/demo_inspect_pick_load.py --reset 2000 --end carry   # carry: skip stow between load and next pick
  python3 isaac/scripts/tools/demo_inspect_pick_load.py --end stow                 # current layout, load ends at stow (default)
"""
import argparse
import json
import math
import subprocess
import time
from pathlib import Path

import rclpy
from perception.msg import TrackedObjectArray
from plan.msg import SubGoal, SubGoalArray, TaskStatus
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

TRIAL, SLOTS = Path('/tmp/beebot_trial.json'), Path('/tmp/beebot_slot_pucks.json')
RESET = Path(__file__).resolve().parents[1] / 'standalone' / 'reset.sh'
STOW = (0.0, -1.9024, 1.344, 0.9023, 0.0)
BOX_R = 0.16

ap = argparse.ArgumentParser()
ap.add_argument('--reset', nargs='?', const=-1, type=int, metavar='SEED')
ap.add_argument('--end', choices=('stow', 'carry', 'over'), default='stow', help='skill_load load_end_pose')
ap.add_argument('--no-carry', action='store_true', help='skill_load load_via_carry false — pick 자리에서 칸 위로 바로')
ap.add_argument('--speed', type=float, help='arm_velocity_scale for skill_inspect · skill_pick · skill_load (default: arm.yaml)')
a = ap.parse_args()

if a.reset is not None:
    seed = a.reset if a.reset >= 0 else json.loads(TRIAL.read_text())['seed']
    Path('/tmp/beebot_trial_seed').write_text(str(seed))
    t_req = time.time()
    subprocess.run([str(RESET)], check=True)
    while True:
        try:
            d = json.loads(TRIAL.read_text())
            if d['seed'] == seed and d['t'] > t_req:
                break
        except (OSError, ValueError, KeyError):
            pass
        time.sleep(0.5)

rclpy.init()
n = rclpy.create_node('demo_inspect_pick_load')
latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
pub = n.create_publisher(SubGoalArray, '/task/sub_goals', latched)
st = {'status': None, 'objects': None}
n.create_subscription(TaskStatus, '/task/status', lambda m: st.update(status=m), latched)
n.create_subscription(TrackedObjectArray, '/tracked_objects', lambda m: st.update(objects=m), 5)
log = []  # (t, q5)
n.create_subscription(JointState, '/joint_states',
                      lambda m: log.append((time.time(), [dict(zip(m.name, m.position)).get(f'joint{i}', 0.0) for i in range(1, 6)]))
                      if 'joint1' in m.name else None, 50)


def spin(s):
    t = time.time()
    while time.time() - t < s:
        rclpy.spin_once(n, timeout_sec=0.05)


def ready():
    names = {t for t, _ in n.get_topic_names_and_types()}
    srvs = {s for s, _ in n.get_service_names_and_types()}
    return pub.get_subscription_count() > 0 and {'/skills/inspect/_action/status', '/skills/load/_action/status'} <= names and \
        {'/plan_kinematic_path', '/score_grasps'} <= srvs


seq = [int(time.time()) % 100000 * 10]


def run(goals, timeout=180.0):
    seq[0] += 1
    plan = SubGoalArray(plan_id=seq[0], goals=[SubGoal(step=i + 1, skill=s, target=t, relation=r) for i, (s, t, r) in enumerate(goals)])
    plan.header.stamp = n.get_clock().now().to_msg()
    t0 = time.time()
    pub.publish(plan)
    while time.time() - t0 < timeout:
        rclpy.spin_once(n, timeout_sec=0.05)
        m = st['status']
        if m and m.plan_id == plan.plan_id and m.state != TaskStatus.RUNNING:
            ok = m.state == TaskStatus.SUCCEEDED
            print(f"  {' → '.join(s + ' ' + t for s, t, _ in goals)}: {'성공' if ok else '실패 ' + m.reason} ({time.time() - t0:.1f} s)", flush=True)
            return ok, t0, time.time()
    print('  시간 초과', flush=True)
    return False, t0, time.time()


def tag_of(o):
    return next((kv.value for kv in o.details if kv.key == 'tag'), '')


while not ready():
    spin(0.5)
spin(3.0)  # skill_pose sends the arm to stow right after bringup
trial = json.loads(TRIAL.read_text())  # after ROS is up — manipulator_env.py has written this run's layout
bx, by = trial['box']
subprocess.run(['ros2', 'param', 'set', '/skill_load', 'load_end_pose', a.end], check=False, capture_output=True)
subprocess.run(['ros2', 'param', 'set', '/skill_load', 'load_via_carry', 'false' if a.no_carry else 'true'], check=False, capture_output=True)
if a.speed:
    for node in ('/skill_inspect', '/skill_pick', '/skill_load'):
        subprocess.run(['ros2', 'param', 'set', node, 'arm_velocity_scale', str(a.speed)], check=False, capture_output=True)
clear = n.create_client(Trigger, '/coordinator/basket_clear')  # 리셋하면 적재함은 비지만 coordinator 장부는 이어 감
if clear.wait_for_service(timeout_sec=10.0):
    fut = clear.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
print(f"seed {trial['seed']} · 상자 간격 {100 * trial['gap']:.1f} cm · load 끝 자세 {a.end} · carry {'안 거침' if a.no_carry else '거침'} · 속도 {a.speed or 'arm.yaml'}", flush=True)
run([('inspect', f'{bx:.4f},{by:.4f},0.13', 'box')])
spin(1.0)
pucks = [o for o in (st['objects'].objects if st['objects'] else [])
         if o.class_name.startswith('puck') and math.hypot(o.pose.pose.position.x - bx, o.pose.pose.position.y - by) < BOX_R]
pucks.sort(key=lambda o: math.hypot(o.pose.pose.position.x, o.pose.pose.position.y))  # near one first
print(f"  본 퍽 {[(o.id, tag_of(o)) for o in pucks]}", flush=True)
spans = []
for o in pucks:
    ok, t0, t1 = run([('pick', str(o.id), ''), ('load', 'next', '')])
    spans.append((t0, t1))
    if not ok:
        break
# between one load and the next pick: did the arm pass the stow pose?
for (s0, e0), (s1, _) in zip(spans, spans[1:]):
    seg = [q for t, q in log if e0 - 5.0 <= t <= s1 + 5.0]  # end of the load · start of the next pick
    near = min((max(abs(x - y) for x, y in zip(q, STOW)) for q in seg), default=float('nan'))
    print(f"  load 끝 ↔ 다음 pick 사이 stow와 가장 가까운 관절 차 {math.degrees(near):.1f}° (5° 안이면 stow를 거침)", flush=True)
try:
    rows = json.loads(SLOTS.read_text())['pucks']
    print('  적재 결과:', {k: (v['slot'], round(1000 * v['dxy']), round(1000 * v['dz'])) for k, v in rows.items() if not k.startswith('_')}, '(칸, 수평 mm, 높이 mm)')
except (OSError, ValueError, KeyError):
    pass
