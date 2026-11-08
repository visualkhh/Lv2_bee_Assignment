#!/usr/bin/env python3
"""inspect 자세 비교 시험 (실제 YOLO 인식 필요 — pick_load_demo.sh를 --truth-perception 없이 띄움):
  skill = 지금 방식 (skill_inspect: 퍽마다 그 위 0.11 m에 손을 수직으로)   ·   box = stow에서 팔을 들어 gripper_cam이 상자 윗면 중심을 내려다보는 자세 (tools/inspect_pose.py)
  같은 seed를 두 방식으로 각각 (시행마다 장면과 ROS를 다시 띄움 — 인식이 태그를 기억하므로) 돌려 태그를 몇 개 읽는지 비교.
  isaac/scripts/demo/pick_load_demo.sh --headless --fix-base --random-layout --trial-seed 2000
  source ros2_ws/install/setup.bash && python3 -u isaac/scripts/demo/inspect_test.py --trials 10 --seed 2000
"""
import argparse
import csv
import json
import math
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
TRIAL_FILE, SEED_FILE = '/tmp/beebot_trial.json', '/tmp/beebot_trial_seed'
RESET_SH = Path(__file__).resolve().parents[1] / 'standalone' / 'reset.sh'
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
INSPECT_Z, MATCH_M = 0.11, 0.05
ARM_Q_FILE = '/tmp/beebot_arm_q'  # manipulator_env.py --arm-q가 쓰는 지금 팔 자세 [deg]
FIELDS = ['seed', 'mode', 'gap_cm', 'puck', 'shape', 'tag_truth', 'dist_m', 'seen', 'tag_read', 'tag_ok', 'pos_err_mm', 'dx_mm', 'dy_mm', 'read_s', 'tilt_deg', 'cam_dist_m', 'tag_px']


def read_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def summarize(rows):
    n = len(rows)
    ok = sum(r['tag_ok'] == 'True' for r in rows)
    lines = [f'퍽 {n}개: 태그 맞게 읽음 {ok} ({100 * ok / max(n, 1):.0f} %)']
    for mode in sorted({r['mode'] for r in rows}):
        m = [r for r in rows if r['mode'] == mode]
        seen = sum(r['seen'] == 'True' for r in m)
        good = sum(r['tag_ok'] == 'True' for r in m)
        wrong = sum(r['tag_read'] not in ('', 'None') and r['tag_ok'] != 'True' for r in m)
        lines.append(f'  {mode}: 퍽 {len(m)}, 보임 {seen}, 태그 맞음 {good} ({100 * good / max(len(m), 1):.0f} %), 틀림 {wrong}')
        g = defaultdict(list)
        for r in m:
            g[min(int(float(r['dist_m']) * 20) / 20, 0.30)].append(r['tag_ok'] == 'True')
        lines.append('     거리 [m]: ' + ', '.join(f'{k:.2f} {sum(v)}/{len(v)}' for k, v in sorted(g.items())))
    return '\n'.join(lines)


def self_test():
    rows = [dict(seed=1, mode='skill', dist_m='0.2', seen='True', tag_read='3', tag_ok='True'),
            dict(seed=1, mode='skill', dist_m='0.27', seen='True', tag_read='None', tag_ok='False'),
            dict(seed=1, mode='box', dist_m='0.2', seen='True', tag_read='4', tag_ok='False')]
    out = summarize(rows)
    assert '태그 맞게 읽음 1 (33 %)' in out and 'box: 퍽 1, 보임 1, 태그 맞음 0 (0 %), 틀림 1' in out, out
    print('self-test ok')


def main(cli):
    import rclpy
    from builtin_interfaces.msg import Duration
    from perception.msg import TrackedObjectArray
    from plan.msg import SubGoal, SubGoalArray, TaskStatus
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    import inspect_cover as cover_mod
    from inspect_cover import F as FOCAL_PX, Robot
    from inspect_pose import look_at
    robot = Robot()

    rclpy.init()
    node = rclpy.create_node('inspect_test')
    state = {'objects': None, 'status': None, 'debug': None}
    latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(TrackedObjectArray, '/tracked_objects', lambda m: state.update(objects=m), 5)
    if cli.save_images:
        import cv2
        import numpy as np
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        Path(cli.save_images).mkdir(parents=True, exist_ok=True)
        node.create_subscription(Image, '/perception/gripper_cam/debug_image', lambda m: state.update(debug=m), qos_profile_sensor_data)
    node.create_subscription(TaskStatus, '/task/status', lambda m: state.update(status=m), latched)
    pub = {'goals': node.create_publisher(SubGoalArray, '/task/sub_goals', latched)}
    traj_pub = node.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
    seq = [3000000 + int(time.time()) % 100000 * 10]

    def spin(s):
        t0 = time.monotonic()
        while time.monotonic() - t0 < s:
            rclpy.spin_once(node, timeout_sec=0.05)

    def wait(cond, timeout, what):
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            r = cond()
            if r:
                return r
        print(f'  시간 초과: {what}')
        return None

    def run(skill, target, timeout=90.0, relation=''):
        seq[0] += 1
        plan = SubGoalArray(plan_id=seq[0], goals=[SubGoal(step=1, skill=skill, target=target, relation=relation)])
        plan.header.stamp = node.get_clock().now().to_msg()
        pub['goals'].publish(plan)
        st = wait(lambda: (lambda s: s if s and s.plan_id == plan.plan_id and s.state != TaskStatus.RUNNING else None)(state['status']), timeout, skill)
        ok = bool(st) and st.state == TaskStatus.SUCCEEDED
        print(f"  {skill} {target}: {'성공' if ok else '실패 ' + (st.reason if st else '시간 초과')}")
        return ok

    def track(xy):  # 참값 위치 근처 퍽 트랙
        best, bd = None, MATCH_M
        for o in (state['objects'].objects if state['objects'] else []):
            if o.class_name.startswith('puck'):
                d = math.hypot(o.pose.pose.position.x - xy[0], o.pose.pose.position.y - xy[1])
                if d < bd:
                    best, bd = o, d
        return best

    def tag_of(o):
        return next((kv.value for kv in o.details if kv.key == 'tag'), '') if o else ''

    def new_trial(seed):
        t_req = time.time()
        Path(SEED_FILE).write_text(str(seed))
        state.update(objects=None, status=None, debug=None)
        node.destroy_publisher(pub['goals'])
        pub['goals'] = node.create_publisher(SubGoalArray, '/task/sub_goals', latched)
        subprocess.run([str(RESET_SH)], check=True)
        trial = wait(lambda: (lambda d: d if d and d['seed'] == seed and d['t'] > t_req else None)(read_json(TRIAL_FILE)), 180.0, f'seed {seed} 배치')
        # 리셋 요청과 ROS 종료 사이에 옛 perception이 낸 마지막 /tracked_objects를 버림 — 배치가 나왔으면 옛 ROS는 이미 꺼져 있음
        # (11/07: 이걸 새 결과로 읽어 시행이 바로 끝나고, 다음 리셋이 isaac_sim.sh 리셋 도중에 와서 사라졌음)
        state.update(objects=None, debug=None)
        return trial

    def ready():
        svcs = {n for n, _ in node.get_service_names_and_types()}
        return {'/score_grasps', '/clear_octomap', '/apply_planning_scene'} <= svcs and '/skills/inspect/_action/status' in {n for n, _ in node.get_topic_names_and_types()}

    def move(q):
        traj_pub.publish(JointTrajectory(joint_names=JOINTS, points=[JointTrajectoryPoint(positions=[float(v) for v in q], time_from_start=Duration(sec=4))]))

    def pucks_near(trial):  # 지금 트랙 중 시작 상자 위 퍽 (상자 가운데에서 0.16 m 안 — 상자 반 변 0.1025 + 여유)
        return [o for o in (state['objects'].objects if state['objects'] else []) if o.class_name.startswith('puck')
                and math.hypot(o.pose.pose.position.x - trial['box'][0], o.pose.pose.position.y - trial['box'][1]) < 0.16]

    def save_image(seed, mode):  # 그 순간 gripper_cam 디버그 영상 (회색 = 버린 검출 · drop 이유)
        if cli.save_images and state['debug'] is not None:
            m = state['debug']
            img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)
            cv2.imwrite(str(Path(cli.save_images) / f'{seed}_{mode}.png'), img if m.encoding == 'bgr8' else img[..., ::-1])

    def view_info(q_deg):  # 광축이 상자 윗면(z 0.13)에 닿는 점까지 거리 · 기울기 · 예상 태그 크기 (URDF FK 예측 — inspect_cover.py)
        t = robot.fk('gripper_cam_color_optical_frame', {f'joint{k + 1}': math.radians(v) for k, v in enumerate(q_deg)})
        c, ax = t[:3, 3], t[:3, 2]
        d = (c[2] - 0.13) / -ax[2]
        tilt = math.degrees(math.acos(-ax[2]))
        return {'tilt_deg': tilt, 'dist_m': d, 'tag_px': FOCAL_PX * 0.016 / d * math.cos(math.radians(tilt))}

    rows = []
    out = Path(cli.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, FIELDS)
        w.writeheader()
        for k in range(cli.trials):
            seed = cli.seed + k
            for mode in cli.modes.split(','):
                print(f'== {k + 1}/{cli.trials} seed {seed} {mode}')
                trial = new_trial(seed)
                if not trial:
                    print('  배치를 못 받음 — 중단')
                    return finish(rows, out)
                ps = sorted(trial['pucks'], key=lambda p: p['dist'])
                info = {}
                if mode == 'static':  # 팔은 이미 --arm-q 자세 (pick_load_demo.sh --perception-only) — perception 첫 발행부터 태그를 기다림
                    wait(lambda: state['objects'], 90.0, '인식 시작')
                    t_start = time.monotonic()
                    wait(lambda: all(tag_of(track((p['x'], p['y']))) for p in ps), cli.read_s, '태그 읽기')
                    seen = {p['name']: bool(track((p['x'], p['y']))) for p in ps}
                    info = view_info(cli.q or [float(v) for v in Path(ARM_Q_FILE).read_text().split(',')])
                    save_image(seed, mode)
                else:
                    wait(ready, 90.0, 'ROS 준비')
                    spin(2.0)
                    seen = {p['name']: bool(wait(lambda p=p: track((p['x'], p['y'])), cli.seen_wait, f"퍽 {p['name']} 인식")) for p in ps}
                    t_start = time.monotonic()
                if mode == 'static':
                    pass
                elif mode == 'skillbox':  # skill_inspect 상자 보기 (look 자세 → 모자라면 다시 담기) — 미션 SURVEY와 같은 호출
                    ok = run('inspect', f"{trial['box'][0]:.4f},{trial['box'][1]:.4f},0.13", relation='box')  # 퍽 윗면 높이 (mission box_top_z + puck_height)
                    info = view_info([math.degrees(v) for v in (0.0, -1.682497, 0.324631, 1.926843, 0.0)])
                    save_image(seed, mode)
                elif mode == 'skill':
                    for p in ps:
                        run('inspect', f"{p['x']:.4f},{p['y']:.4f},{INSPECT_Z}")
                        spin(3.0)
                elif mode == 'center':  # --q 자세로 가서 퍽을 본 뒤, 본 퍽 전부를 가장 크게 담는 자세로 다시 (tools/inspect_cover.search)
                    move([math.radians(v) for v in cli.q])
                    print(f"  1차 자세 {cli.q}")
                    spin(cli.settle)
                    seen_xy = [(o.pose.pose.position.x, o.pose.pose.position.y) for o in pucks_near(trial)]
                    q4 = None
                    if seen_xy:
                        cover_mod.V_SHIFT = 0.0  # 전체 스택은 처짐 보정이 있어 FK 그대로 (11/07 실측 1 px)
                        sc, q4, info2 = cover_mod.search(robot, None, cli.search_n, seed, pts=seen_xy)
                    if q4 is None:
                        print(f"  본 퍽 {len(seen_xy)}개 — 2차 자세 없음, 1차 그대로")
                        info = view_info(cli.q)
                    else:
                        qd = [math.degrees(v) for v in q4] + [0.0]
                        info = view_info(qd)
                        print(f"  본 퍽 {len(seen_xy)}개 → 2차 자세 {[round(v, 1) for v in qd]} · 덮음 {sc[0]:.0%} · 최소 태그 {sc[1]:.0f} px")
                        move(list(q4) + [0.0])
                        spin(cli.settle)
                    wait(lambda: all(tag_of(track((p['x'], p['y']))) for p in ps), 8.0, '태그 읽기')
                    save_image(seed, mode)
                else:
                    if mode == 'fixed':
                        q, info = [math.radians(v) for v in cli.q], view_info(cli.q)
                    elif mode == 'cover':  # 간격별 자세 표 (tools/inspect_cover.py --table) — 간격은 Isaac 참값
                        table = json.loads((Path(__file__).resolve().parents[1] / 'tools' / 'inspect_cover_table.json').read_text())
                        qd = table[min(table, key=lambda g: abs(float(g) - trial['gap']))]['q_deg']
                        q, info = [math.radians(v) for v in qd], view_info(qd)
                    else:
                        q, info = look_at((trial['box'][0], trial['box'][1], 0.13))
                    if q is None:
                        print('  자세 없음:', info)
                    else:
                        msg = JointTrajectory(joint_names=JOINTS, points=[JointTrajectoryPoint(positions=[float(v) for v in q], time_from_start=Duration(sec=4))])
                        traj_pub.publish(msg)
                        print(f"  자세 이동: tilt {info['tilt_deg']:.0f}° 거리 {info['dist_m']:.2f} m 태그 {info['tag_px']:.0f} px")
                        spin(5.0)
                        wait(lambda: all(tag_of(track((p['x'], p['y']))) for p in ps), 8.0, '태그 읽기')
                        save_image(seed, mode)
                for p in ps:
                    o = track((p['x'], p['y']))
                    tag = tag_of(o)
                    pos = o.pose.pose.position if o else None
                    row = dict(seed=seed, mode=mode, gap_cm=round(100 * trial['gap'], 1), puck=p['name'], shape=p['shape'], tag_truth=p['tag'], dist_m=round(p['dist'], 3),
                               seen=seen[p['name']], tag_read=tag or 'None', tag_ok=str(tag) == str(p['tag']),
                               pos_err_mm=round(1000 * math.hypot(pos.x - p['x'], pos.y - p['y']), 1) if pos else '', dx_mm=round(1000 * (pos.x - p['x']), 1) if pos else '', dy_mm=round(1000 * (pos.y - p['y']), 1) if pos else '', read_s=round(time.monotonic() - t_start, 1),
                               tilt_deg=round(info.get('tilt_deg', 0), 1) if info else '', cam_dist_m=round(info.get('dist_m', 0), 3) if info else '',
                               tag_px=round(info.get('tag_px', 0), 1) if info else '')
                    print(f"  퍽 {p['name']} (거리 {p['dist']:.2f}): 태그 {'맞음' if row['tag_ok'] else '못 읽음' if tag == '' else '틀림 ' + tag}")
                    w.writerow(row)
                    f.flush()
                    rows.append({k2: str(v) for k2, v in row.items()})
    return finish(rows, out)


def finish(rows, out):
    print('\n' + summarize(rows))
    print(f'CSV: {out}')
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--trials', type=int, default=10, help='seed 개수 (각 seed를 skill · box 두 방식으로)')
    parser.add_argument('--seed', type=int, default=2000)
    parser.add_argument('--modes', default='skill,box', help='skill · box (상자 중앙 look_at) · fixed (--q 자세) · cover (간격별 표 자세) · center (--q로 가서 본 퍽 전부를 담는 자세로 다시) · skillbox (skill_inspect target box) · static (팔이 이미 자세에 있음, perception만 — pick_load_demo.sh --perception-only)')
    parser.add_argument('--settle', type=float, default=6.0, help='center: 자세 명령(4 s 궤적) 뒤 기다리는 시간 [s]')
    parser.add_argument('--search-n', type=int, default=20000, help='center: 2차 자세 무작위 탐색 수')
    parser.add_argument('--seen-wait', type=float, default=45.0, help='skill · box · fixed: 자세 이동 전 퍽마다 인식을 기다리는 시간 [s] (stow에선 거의 안 보임)')
    parser.add_argument('--read-s', type=float, default=15.0, help='static: 인식 시작 뒤 태그를 기다리는 시간 [s]')
    parser.add_argument('--q', type=lambda t: [float(v) for v in t.split(',')], help='fixed 모드 joint1~5 [deg], 예: 0,-96.4,18.6,110.4,0')
    parser.add_argument('--save-images', metavar='DIR', help='static: 시행마다 gripper_cam 디버그 영상 저장')
    parser.add_argument('--out', default=str(Path.home() / 'pick_test_runs' / time.strftime('inspect_%m%d_%H%M.csv')))
    cli = parser.parse_args()
    if cli.self_test:
        self_test()
        sys.exit(0)
    sys.exit(main(cli))
