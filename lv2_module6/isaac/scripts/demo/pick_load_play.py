#!/usr/bin/env python3
"""시연: 인지로 상자 위 퍽 2개를 찾아 분류 → 미션 SURVEY와 같은 순서로 싣기 (plan/src/mission_manager.cpp survey · load · grab · go_near)
  pose stow → 변 바깥 standoff → dock_box로 approach_gap까지 → 퍽마다 inspect → pick (누우면 stand → stand_pick) → load next (--storage면 그 칸)
  미션과 다른 점: 지도·Nav2가 없음 · 박스 자리는 경기장 좌표가 아니라 인지 값 · navigate 대신 approach로 standoff까지
  시작은 상자에서 30 cm (manipulator_env.py) — 기둥 카메라가 두 퍽을 보는 거리

  source ros2_ws/install/setup.bash && python3 isaac/scripts/demo/pick_load_play.py   (isaac_manipulator_demo.launch.py가 떠 있을 때)
  python3 pick_load_play.py --self-test                                              (ROS 없이 계산만 확인)
"""
import argparse
import json
import math
import sys
import time


BOX_HALF, DOCK_GAP, STANDOFF = 0.1025, 0.085, 0.25  # plan.yaml box_half · approach_gap · standoff
MATCH_RADIUS, RETRIES, APPROACH_TRIES = 0.04, 2, 3  # plan.yaml match_radius · retries · approach_tries
PLAN_ID_BASE = 2000000              # vlm_task_planner(1~) · mission_manager(1000000~)와 안 겹치게
PAIR_M = 0.15                       # grasp_scorer NEIGHBOR_M — 이 안이면 같은 상자 위 퍽 둘
INSPECT_Z = 0.11                    # 윗면 0.13보다 낮게 줌 — skill_inspect가 이 위 0.15 · 0.10 · 0.08을 시도하는데, 먼 퍽(x 0.24)은 손끝 z 0.19까지만 닿음
STAND_CHECK_S = 1.0                 # plan stand_check_s — 세운 뒤 인지가 다시 볼 시간
TAG_WAIT_S = 3.0                    # inspect 뒤 태그 붙은 퍽을 기다리는 시간 — 팔이 기둥 카메라를 가려 그 트랙은 2초 뒤 지워지고,
                                    # 손 카메라 트랙(태그)은 inspect 1초 대기 끝에 아직 없을 때가 있었음 (Isaac 11-06)
SLOT_FILE = '/tmp/beebot_slot_pucks.json'  # manipulator_env.py가 씀 — 퍽마다 가장 가까운 칸 · 거리 (Isaac 물리 정답 위치)
SLOT_XY_M, SLOT_Z_M = 0.015, 0.02   # 칸 원점에서 이 안이면 적재 (칸 간격 0.065) — skill load 성공 보고는 동작 완료일 뿐이라 이걸로 셈
SLOT_SETTLE_S = 1.5                 # load 뒤 퍽이 가라앉을 시간


def dock_faces(a, b, robot):
    """퍽 두 개 (x, y)가 놓인 상자의 네 변 앞 자세 [(standoff 자세, 붙은 자세)], 로봇에서 가까운 변부터 (mission_manager approach_poses · go_near)"""
    mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
    axis = math.atan2(b[1] - a[1], b[0] - a[0])  # 퍽은 상자 축 위에 놓임
    faces = []
    for k in range(4):
        yaw = math.atan2(math.sin(axis + k * math.pi / 2), math.cos(axis + k * math.pi / 2))  # 상자 쪽을 봄
        at = lambda gap: (mx - (BOX_HALF + gap) * math.cos(yaw), my - (BOX_HALF + gap) * math.sin(yaw), yaw)
        faces.append((at(STANDOFF), at(DOCK_GAP)))
    return sorted(faces, key=lambda f: math.hypot(f[0][0] - robot[0], f[0][1] - robot[1]))


def detail(o, key):
    return next((kv.value for kv in o.details if kv.key == key), '')


def parse_storage(text):
    """'0,1' → [0, 1] — 싣는 순서대로 적재함 칸 2개 (coordinator basket_slots 12)"""
    slots = [int(x) for x in text.split(',') if x.isdigit()]
    if len(slots) != 2 or len(text.split(',')) != 2 or slots[0] == slots[1] or max(slots) > 11:
        raise ValueError(f'--storage {text}: 서로 다른 칸 2개, 0~11 (예: 0,1)')
    return slots


def in_slots(rows):
    """{퍽: {slot, dxy, dz}} → 칸 안에 든 퍽 {퍽: 칸}"""
    return {n: r['slot'] for n, r in rows.items() if r['dxy'] <= SLOT_XY_M and abs(r['dz']) <= SLOT_Z_M}


def read_slots():
    """SLOT_FILE의 퍽 행 — 없거나 5 s보다 오래됐으면 None (Isaac 꺼짐 · manipulator_env 아님)"""
    try:
        with open(SLOT_FILE) as f:
            d = json.load(f)
    except (OSError, ValueError):
        return None
    return d['pucks'] if time.time() - d['t'] < 5.0 else None


def self_test():
    rows = {'a': {'slot': 0, 'dxy': 0.01, 'dz': -0.005}, 'b': {'slot': 3, 'dxy': 0.02, 'dz': 0.0},
            'c': {'slot': 5, 'dxy': 0.0, 'dz': 0.05}}
    assert in_slots(rows) == {'a': 0}
    assert parse_storage('0,1') == [0, 1] and parse_storage('11,4') == [11, 4]
    for bad in ('0', '1,1', '0,12', '0,x', '0,1,2', ''):
        try:
            parse_storage(bad)
        except ValueError:
            continue
        raise AssertionError(bad)
    far, near = dock_faces((1.04, 0.0), (0.96, 0.0), (0.0, 0.0))[0]  # 로봇이 −x 쪽이면 −x 변 · 퍽 순서와 무관
    assert abs(near[0] - 0.8125) < 1e-9 and abs(near[1]) < 1e-9 and abs(near[2]) < 1e-9
    assert abs(far[0] - 0.6475) < 1e-9 and abs(far[2]) < 1e-9
    far, near = dock_faces((1.0, 0.04), (1.0, -0.04), (1.0, 0.8))[0]  # 위쪽에서 오면 위 변, 아래를 봄
    assert abs(near[1] - 0.1875) < 1e-9 and abs(near[2] + math.pi / 2) < 1e-9
    print('self-test ok')


def main(args):
    import rclpy
    import tf2_ros
    from perception.msg import TrackedObjectArray
    from plan.msg import BasketState, SubGoal, SubGoalArray, TaskStatus
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_srvs.srv import Trigger

    rclpy.init()
    node = rclpy.create_node('pick_load_play')
    state = {'objects': None, 'status': None, 'basket': None}
    latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(TrackedObjectArray, '/tracked_objects', lambda m: state.update(objects=m), 5)
    node.create_subscription(TaskStatus, '/task/status', lambda m: state.update(status=m), latched)
    node.create_subscription(BasketState, '/coordinator/basket', lambda m: state.update(basket=m), latched)
    goals_pub = node.create_publisher(SubGoalArray, '/task/sub_goals', latched)
    tf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf, node)
    seq = [PLAN_ID_BASE + int(time.time()) % 100000 * 10]

    def wait(cond, timeout, what):
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            r = cond()
            if r:
                return r
        print(f'시간 초과: {what}')
        return None

    def retry(*steps):  # mission_manager run_retry
        for _ in range(RETRIES + 1):
            if run(*steps):
                return True
        return False

    def run(*steps, timeout=180.0):
        seq[0] += 1
        plan = SubGoalArray(plan_id=seq[0], goals=[SubGoal(step=i + 1, skill=s, target=t, relation=r)
                                                   for i, (s, t, r) in enumerate(steps)])
        plan.header.stamp = node.get_clock().now().to_msg()
        goals_pub.publish(plan)
        done = lambda: (lambda st: st if st and st.plan_id == plan.plan_id and st.state != TaskStatus.RUNNING else None)(state['status'])
        st = wait(done, timeout, f'계획 {[s[0] for s in steps]}')
        ok = bool(st) and st.state == TaskStatus.SUCCEEDED
        print(f"  {' → '.join(s[0] + ' ' + s[1] for s in steps)}: {'성공' if ok else '실패 ' + (st.reason if st else '시간 초과')}")
        return ok

    def robot():
        try:
            t = tf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform.translation
            return t.x, t.y
        except Exception:  # noqa: BLE001 — TF가 아직 없음
            return None

    def pucks():
        return [o for o in state['objects'].objects if o.class_name.startswith('puck')] if state['objects'] else []

    def pair():
        r = robot()
        if not r:
            return None
        ps = sorted(pucks(), key=lambda o: math.hypot(o.pose.pose.position.x - r[0], o.pose.pose.position.y - r[1]))
        for a in ps:
            for b in ps:
                if a.id != b.id and math.hypot(a.pose.pose.position.x - b.pose.pose.position.x,
                                               a.pose.pose.position.y - b.pose.pose.position.y) < PAIR_M:
                    return a, b
        return None

    def puck_near(xy, need_tag=False):  # mission_manager puck_near · survey — match_radius 안 (태그 읽은 것만)
        best, best_d = None, MATCH_RADIUS
        for o in pucks():
            d = math.hypot(o.pose.pose.position.x - xy[0], o.pose.pose.position.y - xy[1])
            if d < best_d and (not need_tag or detail(o, 'tag')):
                best, best_d = o, d
        return best

    def show(o):
        p = o.pose.pose.position
        print(f"  id {o.id}: {o.color or '?'} {detail(o, 'shape') or '?'} {detail(o, 'posture') or o.class_name} "
              f"tag {detail(o, 'tag') or '?'} conf {o.confidence:.2f} @ ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")

    def is_lying(o):  # mission_manager is_lying · grasp_scorer
        return o.class_name == 'puck_lying' or detail(o, 'posture') == 'lying'

    def grab(oid, p):  # mission_manager grab · stand_up — 누운 퍽은 세워서 그 손목으로 집기
        o = puck_near(p)
        if o is None or not is_lying(o):
            return retry(('pick', str(o.id if o else oid), ''))
        for _ in range(RETRIES + 1):
            if not run(('stand', str(o.id), '')):
                return False
            t0 = time.monotonic()
            while time.monotonic() - t0 < STAND_CHECK_S:  # 손은 퍽 위로 비켜 있음
                rclpy.spin_once(node, timeout_sec=0.1)
            again = puck_near(p)
            if again is None or not is_lying(again):  # 섰음 (또는 손 그늘로 못 봄 — stand_pick 잡기 판정이 가림)
                return run(('stand_pick', 'last', ''))
            o = again
        print('  stand_still_lying')
        return False

    print('1) 인지 대기 — 같은 상자 위 퍽 2개')
    found = wait(pair, 300.0, '퍽 2개')
    if not found:
        return 1
    for o in found:
        show(o)
    wait(lambda: state['basket'], 30.0, '적재함 장부')
    if state['basket'] and not args.clear_basket and (state['basket'].held.filled or any(x.filled for x in state['basket'].slots)):
        print('시작 거부: 적재함 장부에 퍽이 있음 — 비었으면 --clear-basket (mission_manager와 같은 규칙)')
        return 1
    if args.clear_basket:
        clear = node.create_client(Trigger, '/coordinator/basket_clear')
        if not clear.wait_for_service(timeout_sec=120.0):
            print('coordinator 없음')
            return 1
        fut = clear.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)

    xy = [(o.pose.pose.position.x, o.pose.pose.position.y) for o in found]
    faces = dock_faces(xy[0], xy[1], robot())[:APPROACH_TRIES]
    print('2) 상자 앞으로')
    for k, (far, near) in enumerate(faces):
        print(f'  변 {k + 1}: standoff ({far[0]:.3f}, {far[1]:.3f}) → ({near[0]:.3f}, {near[1]:.3f}, {math.degrees(near[2]):.0f}°)')
        # standoff는 Nav2 대신 — 끝 방향을 진행 방향으로 둠. 도착 자리에서 크게 돌면 skill_approach가 1 cm 경계에서 맴돌아 approach_blocked (dock_box는 정렬된 자리에서 시작)
        r = robot()
        head = math.atan2(far[1] - r[1], far[0] - r[0])
        if run(('pose', 'stow', ''), ('approach', f'{far[0]:.4f},{far[1]:.4f},{head:.4f}', ''),
               ('approach', f'{near[0]:.4f},{near[1]:.4f},{near[2]:.4f}', 'dock_box')):
            break
    else:
        print('approach_failed')
        return 1
    print('3) 한 퍽씩 — inspect → pick → load')
    loaded = 0
    done_tags = set()
    for k, p in enumerate(xy):
        print(f'  퍽 {k + 1} ({p[0]:.3f}, {p[1]:.3f})')
        if not retry(('inspect', f'{p[0]:.4f},{p[1]:.4f},{INSPECT_Z}', '')):
            print('  inspect 실패 — 본 것만')
        o = wait(lambda: puck_near(p, need_tag=True), TAG_WAIT_S, '태그 붙은 퍽')
        if o is None:
            print('  tag_unread')
            continue
        show(o)
        tag = detail(o, 'tag')
        if tag in done_tags:  # 기둥 카메라가 한 퍽을 둘로 봄
            print(f'  tag {tag}: 이미 처리한 퍽')
            continue
        done_tags.add(tag)
        p = (o.pose.pose.position.x, o.pose.pose.position.y)
        if not grab(o.id, p):
            print(f'  puck {tag}: load_pick_failed')
            continue
        before = in_slots(read_slots() or {})
        if retry(('load', str(args.storage[k]) if args.storage else 'next', '')):
            t0 = time.monotonic()
            while time.monotonic() - t0 < SLOT_SETTLE_S:
                rclpy.spin_once(node, timeout_sec=0.1)
            rows = read_slots()
            new = {n: c for n, c in in_slots(rows or {}).items() if n not in before}
            if rows is None:
                print(f'  puck {tag}: {SLOT_FILE} 없음 — skill 보고만 믿고 적재로 셈')
                loaded += 1
            elif new:
                print(f"  puck {tag}: 칸 {', '.join(map(str, new.values()))} 안 (Isaac 위치)")
                loaded += 1
            else:
                near = ', '.join(f"{n} 칸 {r['slot']} xy {r['dxy'] * 100:.1f} cm z {r['dz'] * 100:+.1f} cm" for n, r in rows.items())
                print(f'  puck {tag}: load_slot_miss — skill은 성공 보고, 칸 안 퍽 없음 ({near})')
        elif run(('place', f'{p[0]:.4f},{p[1]:.4f},0.07', '')):  # mission_manager load — 박스 제자리에 되돌림
            print(f'  puck {tag}: load_tray_failed (박스에 되돌림)')
        else:
            print(f'  puck {tag}: load_tray_failed (되돌리기도 실패)')
    wait(lambda: state['basket'], 5.0, '적재함 장부')
    print(f'결과: {loaded}/2개 적재')
    if state['basket']:
        for s in state['basket'].slots:
            if s.filled:
                print(f'  칸 {s.slot}: {s.color or "?"} {s.shape} tag {s.tag}')
    return 0 if loaded == 2 else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--clear-basket', action='store_true', help='적재함이 정말 비었을 때 장부 비우기 (reset.sh 뒤)')
    parser.add_argument('--storage', help='적재함 칸 2개, 싣는 순서대로 (예: 0,1) — 없으면 빈 칸부터 (load next)')
    cli = parser.parse_args()
    if cli.storage is not None:
        try:
            cli.storage = parse_storage(cli.storage)
        except ValueError as e:
            parser.error(str(e))
    if cli.self_test:
        self_test()
        sys.exit(0)
    sys.exit(main(cli))
