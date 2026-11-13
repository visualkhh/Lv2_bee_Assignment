"""Isaac Sim approach test along the real mission path: drive to a start box with Nav2, then dock with /skills/approach.

Sends the same plan mission_manager go_near() sends (pose stow -> navigate to the side standoff -> approach dock_box to
approach_gap) to /task/sub_goals, so coordinator, Nav2, twist_mux, collision_monitor and the skill nodes all run as on the
robot. Run while isaac/standalone/isaac_sim.sh <scene> is up and playing (coordinator in AUTO).

    python3 approach_loop.py 1                 # start_1, nearest free side
    python3 approach_loop.py 3 --side north    # force a side: west | east | south | north (map frame)
    python3 approach_loop.py 1 --bag full      # also record /map and both costmaps (large)
    python3 approach_loop.py 1 --bag off
    python3 approach_loop.py 1 --plan-error 0.04 -0.02   # give the robot a box hint 4 cm / -2 cm off (wrong floor plan)

skill_approach does not trust the hint: it raises the arm to look slowly, docks on the box it sees (/tracked_objects) and
fails with approach_box_unseen if it sees none. The end error below is against the TRUE box (USD), so --plan-error shows
whether the perception really took over. /tracked_objects boxes near the true box are logged once a second (meas_mm).

Box centre: /World/start_boxes/start_<n> of the running scene (ISAAC_SCENE of the Isaac process, or --scene), in the map
frame (= base_footprint where the robot was at Play). Read with pxr in the Isaac venv. Sides closer than --wall-clearance
to a wall of isaac/mission/map/<scene>/map.pgm are skipped, like mission_manager.
ROS env (domain, Fast DDS profile, discovery range) is copied from the running isaac_system launch.

Ground truth: the pucks sitting on the box (Isaac writes their pose in base_link to /tmp/beebot_sim_pucks.json while
playing) give the TRUE box relative to the robot, independent of cuVSLAM. Logged once a second: loc_mm = map->base_footprint
TF minus the true robot pose (localization error), meas_mm = /tracked_objects box minus the true box, both in the robot
frame. The end error is against the true box when the pucks are found, else against the USD box in the TF map frame.

Output: logs/approach_<ts>/ = result.json (steps, end pose error, per-second track) + bag/ (MCAP, sim time).
Robot ends at the box: run isaac/standalone/runtime/reset.sh before the next try.

Real robot (--real): run in a shell that already talks to the robot's stack (its ROS env is used as is, wall clock).
    python3 approach_loop.py 3 --real --exit 0.165            # start box 3 of plan.yaml
    python3 approach_loop.py 3 --real --start-pose 0.734 0.950 3.1416   # robot was not switched on at start_pose_arena
    python3 approach_loop.py L2 --real                        # location box L1..L3 (plan.yaml location_boxes)
Box centre: plan.yaml start_boxes (arena) -> map with start_pose_arena (map = where the robot was switched on, as
mission_manager). Sides are checked against plan.yaml arena_walls. Ground truth instead of Isaac: the two pucks on the box
as /tracked_objects sees them (class puck, puck_offsets_x apart), turned into the robot frame with the TF of the same
moment - so the map drift cancels and the result is the robot pose relative to the real box (perception accuracy included).
"""
import argparse
import atexit
import json
import math
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC = os.path.join(HERE, '..')
ISAAC_PY = os.path.expanduser(os.environ.get('ISAACSIM_VENV', '~/pa-course/.venv-isaacsim')) + '/bin/python'
TOPICS = ('^/(tf|tf_static|odom|odometry/filtered|cuvslam/odom|imu|cmd_vel|cmd_vel_nav|cmd_vel_mux|cmd_vel_smoothed|'
          'plan|unsmoothed_plan|transformed_global_plan|collision_monitor_state|collision_monitor/triggering_points|'
          'polygon_approach|polygon_slowdown|local_costmap/published_footprint|task/.*|skills/(pose|navigate|approach)/.*|'
          'navigate_to_pose/_action/.*|state/(emergency|operation_mode|collision)|rosout|joint_states|tracked_objects)$')
TOPICS_FULL = TOPICS[:-2] + '|map|local_costmap/costmap|global_costmap/costmap)$'
SIDES = {'west': (-1, 0, 0.0), 'east': (1, 0, math.pi), 'south': (0, -1, math.pi / 2), 'north': (0, 1, -math.pi / 2)}


def proc_env(pattern):
    """Environment of the first process whose command line contains pattern ({} if none)."""
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            if pattern in open(f'/proc/{pid}/cmdline', 'rb').read().decode(errors='ignore'):
                raw = open(f'/proc/{pid}/environ', 'rb').read().decode(errors='ignore')
                return dict(kv.split('=', 1) for kv in raw.split('\0') if '=' in kv)
        except OSError:
            pass
    return {}


TRUTH_FILE = '/tmp/beebot_sim_pucks.json'  # isaac_startup.py


def box_in_map(scene, n):
    """(x, y) of start_<n> in the map frame and {puck name: (x, y)} of the pucks on it, from the USD (pxr: Isaac venv)."""
    code = f'''
from pxr import Usd, UsdGeom
st = Usd.Stage.Open({scene!r})
inv = UsdGeom.Xformable(st.GetPrimAtPath('/World/beebot')).ComputeLocalToWorldTransform(0).GetInverse()
p = st.GetPrimAtPath('/World/start_boxes/start_{n}')
if not p: raise SystemExit('no /World/start_boxes/start_{n} in {scene}')
r = UsdGeom.BBoxCache(0, ['default', 'render']).ComputeWorldBound(p).ComputeAlignedRange()
c = inv.Transform((r.GetMin() + r.GetMax()) / 2)
import json
pk = {{}}
for q in Usd.PrimRange(st.GetPrimAtPath('/World')):
    if q.GetName().startswith('puck_') and q.IsA(UsdGeom.Xformable) and q.GetParent().GetName() != q.GetName():
        t = inv.Transform(UsdGeom.Xformable(q).ComputeLocalToWorldTransform(0).ExtractTranslation())
        if abs(t[0] - c[0]) < 0.12 and abs(t[1] - c[1]) < 0.12: pk[q.GetName()] = (t[0], t[1])
print(json.dumps([c[0], c[1], pk]))
'''
    out = subprocess.run([ISAAC_PY, '-c', code], capture_output=True, text=True)
    if out.returncode:
        sys.exit(out.stderr.strip().splitlines()[-1])
    x, y, pucks = json.loads(out.stdout.strip().splitlines()[-1])
    return x, y, pucks


def truth(pucks_map, box):
    """True robot pose (x, y, yaw) in the USD map frame from the live pucks on the box (None if not readable).
    Needs 2 pucks for the yaw. Assumes the pucks did not move on the box (the box itself may have been pushed)."""
    try:
        live = json.load(open(TRUTH_FILE))
        if time.time() - live['t'] > 2.0:
            return None
        live = {k: v for k, v in live['pucks'].items() if k in pucks_map and v['tilt'] < 20}
    except (OSError, ValueError, KeyError):
        return None
    if len(live) < 2:
        return None
    (n1, a), (n2, b) = list(live.items())[:2]
    m1, m2 = pucks_map[n1], pucks_map[n2]
    yaw = math.atan2(m2[1] - m1[1], m2[0] - m1[0]) - math.atan2(b['y'] - a['y'], b['x'] - a['x'])  # robot yaw in map
    c, s = math.cos(yaw), math.sin(yaw)
    rx = m1[0] - (c * a['x'] - s * a['y'])  # map puck = robot + R(yaw) * robot-frame puck
    ry = m1[1] - (s * a['x'] + c * a['y'])
    return rx, ry, math.remainder(yaw, 2 * math.pi)


PLAN_YAML = os.path.join(HERE, '..', '..', 'ros2_ws', 'src', 'plan', 'config', 'plan.yaml')


def plan_cfg():
    """mission_manager parameters of plan.yaml (arena layout)."""
    import yaml
    d = yaml.safe_load(open(PLAN_YAML))
    return next(v['ros__parameters'] for k, v in d.items() if 'start_boxes' in v.get('ros__parameters', {}))


def arena_to_map(x, y, yaw, start):
    """mission_solver arena_to_map: map = base_footprint where the robot was switched on at start (arena x, y, yaw)."""
    dx, dy = x - start[0], y - start[1]
    c, s = math.cos(-start[2]), math.sin(-start[2])
    return c * dx - s * dy, s * dx + c * dy, math.remainder(yaw - start[2], 2 * math.pi)


def segment_distance(walls):
    """Function (x, y) -> distance [m] to the nearest wall segment [x0, y0, x1, y1, ...] (same frame)."""
    segs = [walls[i:i + 4] for i in range(0, len(walls) - 3, 4)]

    def dist(x, y):
        best = math.inf
        for x0, y0, x1, y1 in segs:
            vx, vy = x1 - x0, y1 - y0
            t = max(0.0, min(1.0, ((x - x0) * vx + (y - y0) * vy) / max(vx * vx + vy * vy, 1e-12)))
            best = min(best, math.hypot(x - x0 - t * vx, y - y0 - t * vy))
        return best
    return dist


def real_truth(objects, p, box, axis_map, sep, now_s, max_age=1.0):
    """Robot pose (x, y, yaw) relative to the real box, written in the plan's map frame, from the 2 pucks on it.
    objects: /tracked_objects (map), p: TF map pose of the robot at the same time. A puck pair is symmetric (180 deg),
    so the yaw closest to the TF yaw is taken. None when no fresh pair about sep apart is seen near the box."""
    if objects is None or p is None:
        return None
    c, s = math.cos(p[2]), math.sin(p[2])
    rf = lambda x, y: (c * (x - p[0]) + s * (y - p[1]), -s * (x - p[0]) + c * (y - p[1]))
    pk = [rf(o.pose.pose.position.x, o.pose.pose.position.y) for o in objects.objects
          if o.class_name == 'puck' and now_s - (o.last_seen.sec + o.last_seen.nanosec * 1e-9) <= max_age]
    eb = rf(*box)
    best = None
    for i in range(len(pk)):
        for j in range(i + 1, len(pk)):
            d = math.hypot(pk[j][0] - pk[i][0], pk[j][1] - pk[i][1])
            mid = ((pk[i][0] + pk[j][0]) / 2, (pk[i][1] + pk[j][1]) / 2)
            off = math.hypot(mid[0] - eb[0], mid[1] - eb[1])
            if abs(d - sep) < 0.025 and off < 0.30 and (best is None or off < best[0]):
                best = (off, mid, math.atan2(pk[j][1] - pk[i][1], pk[j][0] - pk[i][0]))
    if best is None:
        return None
    _, mid, axis_robot = best
    yaw = axis_map - axis_robot
    if abs(math.remainder(yaw - p[2], 2 * math.pi)) > math.pi / 2:
        yaw += math.pi
    yaw = math.remainder(yaw, 2 * math.pi)
    c, s = math.cos(yaw), math.sin(yaw)
    return box[0] - (c * mid[0] - s * mid[1]), box[1] - (s * mid[0] + c * mid[1]), yaw


LIFECYCLE = ('bt_navigator', 'controller_server', 'planner_server', 'behavior_server', 'collision_monitor', 'velocity_smoother')


def wait_ready(node, limit, pose, spin):
    """True once every LIFECYCLE node is active, /skills/{pose,navigate,approach} are up and map->base_footprint resolves.
    reset.sh startups sometimes stall a lifecycle transition (change_state response lost, 11-12) — then this times out."""
    from lifecycle_msgs.srv import GetState
    end, missing = time.time() + limit, ['start']
    clients = {n: node.create_client(GetState, f'/{n}/get_state') for n in LIFECYCLE}
    while time.time() < end:
        missing = []
        for n, c in clients.items():
            if not c.service_is_ready():
                missing.append(n)
                continue
            f = c.call_async(GetState.Request())
            t = time.time() + 2.0
            while not f.done() and time.time() < t:
                spin(0.05)
            if not f.done() or f.result().current_state.id != 3:  # 3 = active
                missing.append(n)
        names = {n for n, _ in node.get_topic_names_and_types()}
        missing += [s for s in ('pose', 'navigate', 'approach') if f'/skills/{s}/_action/status' not in names]
        if pose() is None:
            missing.append('tf')
        if not missing:
            print(f'stack ready ({limit - (end - time.time()):.0f} s)', flush=True)
            return True
        spin(2.0)
    print('not ready:', ', '.join(missing), flush=True)
    return False


def wall_distance(scene_name):
    """Function (x, y) -> distance [m] to the nearest wall cell of the scene map (inf if there is no map)."""
    import numpy as np
    import yaml
    from PIL import Image
    path = os.path.join(ISAAC, 'mission', 'map', scene_name, 'map.yaml')
    if not os.path.exists(path):
        print(f'no wall map {path} — sides are not checked against walls (make it: isaac/mission/make_map.py)')
        return lambda x, y: math.inf
    m = yaml.safe_load(open(path))
    g = np.array(Image.open(os.path.join(os.path.dirname(path), m['image'])))
    res, (ox, oy, _) = m['resolution'], m['origin']
    occ = np.argwhere(g < 100)
    xs = occ[:, 1] * res + ox + res / 2
    ys = (g.shape[0] - 1 - occ[:, 0]) * res + oy + res / 2
    return lambda x, y: float(np.min(np.hypot(xs - x, ys - y))) if len(xs) else math.inf


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('box', help='start box number (start_<n>) · --real also L1..L3 (plan.yaml location_boxes)')
    ap.add_argument('--side', choices=SIDES, help='box side to dock at (default: nearest side clear of walls)')
    ap.add_argument('--scene', help='scene usda (default: ISAAC_SCENE of the running Isaac)')
    ap.add_argument('--policy', default='dock_box', help='approach policy (goal.target)')
    ap.add_argument('--half', type=float, default=0.1025, help='[m] box half side (plan.yaml box_half)')
    ap.add_argument('--real', action='store_true', help='real robot: plan.yaml boxes/walls, pucks seen by perception as truth')
    ap.add_argument('--start-pose', type=float, nargs=3, metavar=('X', 'Y', 'YAW'),
                    help='--real: arena pose the robot was switched on at (default plan.yaml start_pose_arena)')
    ap.add_argument('--exit', type=float, default=0.0,
                    help='[m] after docking, back straight out this far (approach exit, as mission_manager does) - 0 = off')
    ap.add_argument('--standoff', type=float, default=0.25, help='[m] Nav2 stops this far from the side (plan.yaml standoff)')
    ap.add_argument('--gap', type=float, default=0.085, help='[m] approach to this gap base_footprint-side (plan.yaml approach_gap)')
    ap.add_argument('--wall-clearance', type=float, default=0.22, help='[m] skip sides closer to a wall (plan.yaml)')
    ap.add_argument('--timeout', type=float, default=240.0, help='[s] whole plan')
    ap.add_argument('--wait-ready', type=float, default=90.0, metavar='SEC',
                    help='(default 90, 0 = no check) first wait up to SEC for Nav2 + base lifecycle nodes active, skill servers and TF (after reset.sh); '
                         'exit 3 if not (approach_batch.sh then resets again)')
    ap.add_argument('--bag', choices=['on', 'full', 'off'], default='on')
    ap.add_argument('--plan-error', type=float, nargs=2, default=[0.0, 0.0], metavar=('DX', 'DY'),
                    help='[m] shift the box hint sent to the robot (map frame) — the true box stays where the USD has it')
    a = ap.parse_args()

    if a.real:  # this shell's ROS env, plan.yaml arena
        cfg = plan_cfg()
        start_arena = a.start_pose or cfg['start_pose_arena']
        loc = a.box.upper().startswith('L')
        boxes = [cfg['location_boxes'][i:i + 3] for i in range(0, len(cfg['location_boxes']) - 2, 3)] if loc else \
            [cfg['start_boxes'][i:i + 2] + [cfg['box_half']] for i in range(0, len(cfg['start_boxes']) - 1, 2)]
        k = int(a.box[1:] if loc else a.box)
        if not 1 <= k <= len(boxes):
            sys.exit(f'plan.yaml has {"location boxes L" if loc else "start boxes "}1..{len(boxes)}')
        label = f'L{k}' if loc else f'start_{k}'
        a.half = boxes[k - 1][2]
        bx, by, axis_map = arena_to_map(boxes[k - 1][0], boxes[k - 1][1], 0.0, start_arena)
        walls = []
        for i in range(0, len(cfg['arena_walls']) - 3, 4):
            w = cfg['arena_walls'][i:i + 4]
            walls += [*arena_to_map(w[0], w[1], 0.0, start_arena)[:2], *arena_to_map(w[2], w[3], 0.0, start_arena)[:2]]
        near_wall = segment_distance(walls)
        # puck pair spacing on a start box. Location boxes hold pucks on a 6 cm grid (a pair there is not the box axis) -> no truth
        sep = None if loc else abs(cfg['puck_offsets_x'][1] - cfg['puck_offsets_x'][0])
        scene_name, pucks_map = 'real', {}
        # allowed sides, arena axes ("+x -y") - as mission_manager. A location box may hold no pucks -> no ground truth
        sides_cfg = cfg.get('location_box_sides' if loc else 'start_box_sides') or []
        allowed = sides_cfg[k - 1].split() if len(sides_cfg) == len(boxes) else None
        print(f'real robot · start pose (arena) {[round(v, 3) for v in start_arena]} · plan.yaml {PLAN_YAML}')
    else:
        # ROS env of the running stack — must be set before rclpy starts
        ros_env = proc_env('isaac_system.launch.py')
        if not ros_env:
            sys.exit('isaac_system.launch.py is not running — start isaac/standalone/isaac_sim.sh <scene> and press Play')
        for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
            if k in ros_env:
                os.environ[k] = ros_env[k]
        scene = a.scene or ros_env.get('ISAAC_SCENE') or proc_env('isaac_startup.py').get('ISAAC_SCENE') \
            or os.path.join(ISAAC, 'usda', 'scene_sim.usda')
        scene_name = os.path.splitext(os.path.basename(scene))[0]
        if not a.box.isdigit():
            sys.exit('the Isaac scenes only have start boxes - give a number (L1..L3 need --real)')
        a.box = int(a.box)
        label = f'start_{a.box}'

        bx, by, pucks_map = box_in_map(scene, a.box)
        if len(pucks_map) < 2:
            print(f'  only {len(pucks_map)} puck on {label} — no ground truth, errors use the TF map frame')
        near_wall = wall_distance(scene_name)
        allowed = None
    sides = []
    for name, (sx, sy, yaw) in SIDES.items():
        d = a.half + a.standoff
        nav = (bx + sx * d, by + sy * d, yaw)
        # a wall between the box face and the standoff point (large --standoff) -> Nav2 would drive around to the far side
        steps = int((d - a.half) / 0.02)
        crossed = any(near_wall(bx + sx * (a.half + 0.02 * i), by + sy * (a.half + 0.02 * i)) < 0.05 for i in range(1, steps + 1))
        sides.append((name, nav, 0.0 if crossed else near_wall(nav[0], nav[1])))
    sides.sort(key=lambda s: math.hypot(s[1][0], s[1][1]))  # nearest to the robot's start (map origin) first
    if a.side:
        side = next(s for s in sides if s[0] == a.side)
    else:
        free = [s for s in sides if s[2] >= a.wall_clearance]
        if allowed is not None:  # map side -> arena side: the arena axis the box-to-robot direction points along
            def arena_side(n):
                sx, sy = SIDES[n][:2]
                c, s_ = math.cos(start_arena[2]), math.sin(start_arena[2])
                ax, ay = c * sx - s_ * sy, s_ * sx + c * sy
                return ('+x' if ax > 0 else '-x') if abs(ax) > abs(ay) else ('+y' if ay > 0 else '-y')
            free = [s for s in free if arena_side(s[0]) in allowed]
            print(f'  plan.yaml sides: {" ".join(allowed)} -> map sides {[s[0] for s in free]}')
        if not free:
            sys.exit(f'{label}: no allowed side clear of walls ({a.wall_clearance} m)')
        side = free[0]
    name, nav, wall = side
    push = a.standoff - a.gap
    app = (nav[0] + push * math.cos(nav[2]), nav[1] + push * math.sin(nav[2]), nav[2])  # true dock pose (USD box)
    ex, ey = a.plan_error
    nav_hint = (nav[0] + ex, nav[1] + ey, nav[2])
    app_hint = (app[0] + ex, app[1] + ey, app[2])
    if ex or ey:
        print(f'  hint sent to the robot is off by ({ex * 100:+.1f}, {ey * 100:+.1f}) cm')
    print(f'scene {scene_name} · {label} map ({bx:.3f}, {by:.3f}) · side {name} (wall {wall:.2f} m)')
    print(f'  navigate ({nav[0]:.3f}, {nav[1]:.3f}, {math.degrees(nav[2]):.0f}°) -> approach {a.policy} '
          f'({app[0]:.3f}, {app[1]:.3f}) gap {a.gap * 1000:.0f} mm')

    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from perception.msg import TrackedObjectArray
    from plan.msg import SubGoal, SubGoalArray, TaskStatus
    from tf2_ros import Buffer, TransformListener

    out = os.path.join(HERE, 'logs', time.strftime('approach_%m%d_%H%M%S'))
    os.makedirs(out, exist_ok=True)
    bag = None
    if a.bag != 'off':
        bag = subprocess.Popen(
            ['ros2', 'bag', 'record', '--node-name', 'approach_record', '--disable-keyboard-controls',
             *([] if a.real else ['--use-sim-time']), '--regex', TOPICS_FULL if a.bag == 'full' else TOPICS, '--include-hidden-topics',
             '-s', 'mcap', '--storage-preset-profile', 'zstd_fast', '-o', os.path.join(out, 'bag')],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

        def stop_bag():
            if bag.poll() is None:
                os.killpg(bag.pid, signal.SIGINT)
                try:
                    bag.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(bag.pid, signal.SIGKILL)
        atexit.register(stop_bag)

    rclpy.init()
    node = rclpy.create_node('approach_loop', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=not a.real)])
    q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(SubGoalArray, '/task/sub_goals', q)
    tf = Buffer()
    TransformListener(tf, node)
    plan_id = int(time.time()) % 100000 + 1
    steps, state = [], {}
    labels = {1: 'pose stow', 2: 'navigate', 3: f'approach {a.policy}', 4: 'approach exit'}
    last_step = 4 if a.exit > 0 else 3

    def pose():
        try:
            t = tf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
            r = t.rotation
            return t.translation.x, t.translation.y, math.atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y ** 2 + r.z ** 2))
        except Exception:
            return None

    def on_status(m):
        if m.plan_id != plan_id or (m.step, m.state) == state.get('last'):
            return
        state['last'] = (m.step, m.state)
        t = time.time() - state['t0']
        word = ['RUNNING', 'SUCCEEDED', 'FAILED'][m.state]
        print(f'[{t:6.1f}s] {m.step} {labels.get(m.step, "")}: {word} {m.reason}', flush=True)
        if m.state != TaskStatus.RUNNING:
            steps.append({'step': m.step, 'skill': labels.get(m.step, ''), 'ok': m.state == TaskStatus.SUCCEEDED,
                          'reason': m.reason, 'end_s': round(t, 1)})
        # judge the dock when step 3 ends (exit moves the robot afterwards) - with an exit the coordinator may only say "4 RUNNING"
        if 'docked' not in state and ((m.step == 3 and m.state != TaskStatus.RUNNING) or m.step == 4):
            state['docked'] = (pose(), true_pose() or state.get('last_truth'))  # real: pucks may blink this instant
        if m.state == TaskStatus.FAILED or (m.step == last_step and m.state == TaskStatus.SUCCEEDED):
            state['done'] = m
        elif m.state == TaskStatus.RUNNING and m.step > 1:  # previous step finished
            steps.append({'step': m.step - 1, 'skill': labels[m.step - 1], 'ok': True, 'reason': '', 'end_s': round(t, 1)})
    node.create_subscription(TaskStatus, '/task/status', on_status, q)
    seen = {}
    node.create_subscription(TrackedObjectArray, '/tracked_objects', lambda m: seen.__setitem__('m', m), 5)

    def true_pose():  # sim: Isaac pucks · real: pucks seen by perception, with the TF of now
        if a.real:
            if sep is None:
                return None
            return real_truth(seen.get('m'), pose(), (bx, by), axis_map, sep, node.get_clock().now().nanoseconds * 1e-9)
        return truth(pucks_map, (bx, by))

    def to_robot(px, py, rp):  # map point -> robot frame of pose rp
        c, s = math.cos(rp[2]), math.sin(rp[2])
        return c * (px - rp[0]) + s * (py - rp[1]), -s * (px - rp[0]) + c * (py - rp[1])

    def box_meas(p, tp):  # [meas_mm robot frame (dx, dy) vs true box, size_mm (a, b), yaw_deg | None, camera] or None
        m = seen.get('m')
        cand = [o for o in (m.objects if m else []) if o.class_name in ('start_box', 'location_box')]
        if not cand or not p:
            return None
        o = min(cand, key=lambda o: math.hypot(o.pose.pose.position.x - bx, o.pose.pose.position.y - by))
        qz, qw = o.pose.pose.orientation.z, o.pose.pose.orientation.w
        mr = to_robot(o.pose.pose.position.x, o.pose.pose.position.y, p)  # measured box seen from the robot (TF)
        tr = to_robot(bx, by, tp or p)                                      # true box seen from the robot
        return [round((mr[0] - tr[0]) * 1000, 1), round((mr[1] - tr[1]) * 1000, 1),
                [round(o.size.x * 1000), round(o.size.y * 1000)],
                round(math.degrees(2 * math.atan2(qz, qw)), 1) if abs(qz) > 1e-6 else None, '+'.join(o.cameras[-1:])]

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    spin(3.0)  # TF + bag recorder discovery
    if a.wait_ready > 0 and not wait_ready(node, a.wait_ready, pose, spin):
        print(f'stack not ready after {a.wait_ready:.0f} s (lifecycle / skills / TF) — exit 3')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(3)
    start = pose()
    if start is None:
        print('no map -> base_footprint TF — is the stack up?')
    fmt = lambda p, n=3: ','.join(f'{v:.{n}f}' for v in p)
    state['t0'] = time.time()
    pub.publish(SubGoalArray(plan_id=plan_id, goals=[
        SubGoal(step=1, skill='pose', target='stow'),
        SubGoal(step=2, skill='navigate', target=fmt(nav_hint), relation='look'),
        SubGoal(step=3, skill='approach', target=fmt((*app_hint, a.gap, a.half), 4), relation=a.policy)] +
        ([SubGoal(step=4, skill='approach', target=f'{a.exit:.3f}', relation='exit')] if a.exit > 0 else [])))
    track, last = [], 0.0
    try:
        while 'done' not in state and time.time() - state['t0'] < a.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.time() - last > 1.0:
                last = time.time()
                p = pose()
                if p:
                    tp = true_pose()
                    if tp:
                        state['last_truth'] = tp
                    loc = tp and [round((p[0] - tp[0]) * 1000, 1), round((p[1] - tp[1]) * 1000, 1),
                                  round(math.degrees(math.remainder(p[2] - tp[2], 2 * math.pi)), 2)]
                    track.append([round(last - state['t0'], 1), *(round(v, 4) for v in p), loc, box_meas(p, tp)])
    except KeyboardInterrupt:
        print('Ctrl-C — stopping the plan')
    if 'done' not in state:
        pub.publish(SubGoalArray(plan_id=0, goals=[]))  # coordinator: explicit stop
        steps.append({'step': 0, 'skill': '', 'ok': False, 'reason': 'timeout/interrupted',
                      'end_s': round(time.time() - state['t0'], 1)})
    spin(2.0)  # let the robot settle, bag the tail
    end = pose()

    result = {'scene': scene_name, 'box': a.box, 'box_map': [bx, by], 'side': name, 'policy': a.policy,
              'navigate_goal': nav_hint, 'approach_goal_true': app, 'approach_hint': app_hint, 'plan_error': a.plan_error,
              'track_fields': ['t_s', 'x', 'y', 'yaw (TF map)', 'loc_mm [dx, dy, dyaw_deg] TF - truth',
                               'meas [dx_mm, dy_mm] robot frame vs true box, [size_a, size_b] mm, yaw_deg, camera'], 'start_pose': start, 'end_pose': end,
              'ok': bool(state.get('done')) and state['done'].state == TaskStatus.SUCCEEDED, 'steps': steps, 'track': track}
    tend = true_pose()
    result['true_end_pose'] = tend
    if 'docked' in state:  # error = where the dock ended, before any exit
        result['exit_end_pose'], result['exit_true_end_pose'] = end, tend
        end, tend = state['docked']
        result['end_pose'], result['true_end_pose'] = end, tend
    if tend:
        end = tend  # judge against the real box, not the (possibly drifted) map
    if end:
        c, s = math.cos(app[2]), math.sin(app[2])
        dx, dy = end[0] - app[0], end[1] - app[1]
        face = a.half - abs((end[0] - bx) * c + (end[1] - by) * s)  # negative = robot origin outside the box side
        result['error'] = {'along_mm': round((c * dx + s * dy) * 1000, 1), 'lateral_mm': round((-s * dx + c * dy) * 1000, 1),
                           'yaw_deg': round(math.degrees(math.remainder(end[2] - app[2], 2 * math.pi)), 2),
                           'gap_mm': round(-face * 1000, 1)}
        e = result['error']
        print(f'end: along {e["along_mm"]:+.1f} mm · lateral {e["lateral_mm"]:+.1f} mm · yaw {e["yaw_deg"]:+.2f}° · '
              f'gap to box {e["gap_mm"]:.1f} mm (goal {a.gap * 1000:.0f})')
    with open(os.path.join(out, 'result.json'), 'w') as f:
        json.dump(result, f, indent=1, ensure_ascii=False)
    print(f'{"OK" if result["ok"] else "FAILED"} -> {out}')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if result['ok'] else 1)


if __name__ == '__main__':
    main()
