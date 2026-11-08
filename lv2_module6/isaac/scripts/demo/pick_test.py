#!/usr/bin/env python3
"""pick 성공률 시험 (정책 끄고, inspect에서 시작 — 접근 없이, 인식은 Isaac 참값): 시행마다 상자·퍽 배치를 새로 뽑고(--random-layout) inspect → pick 한 번, Isaac 실제 퍽 높이로 판정.
  시작: isaac/scripts/demo/pick_load_demo.sh --random-layout --trial-seed 0 --no-policy pick_standing_puck --truth-perception
  실행: source ros2_ws/install/setup.bash && python3 -u isaac/scripts/demo/pick_test.py --trials 30 --seed 0   (isaac_sim.sh가 떠 있을 때)
  시행 0은 Isaac이 처음 띄운 배치(--trial-seed와 같은 --seed). 이후는 seed를 적고 reset.sh로 장면과 ROS 노드를 다시 띄움 (시행당 1~2분 예상)
  결과: --out CSV (한 시행 한 줄) + 끝에 요약. 팔이 못 닿는 퍽(base_footprint에서 0.30 m 밖)은 '도달 불가'로 따로 세고 성공률 분모에서 뺌.
  python3 pick_test.py --self-test   (ROS 없이 판정·요약만 확인)
"""
import argparse
import csv
import json
import math
import os
import random
import subprocess
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

TRIAL_FILE, SEED_FILE, SLOT_FILE = '/tmp/beebot_trial.json', '/tmp/beebot_trial_seed', '/tmp/beebot_slot_pucks.json'
SOFT_FILE = '/tmp/beebot_soft_reset'
RESET_SH = Path(__file__).resolve().parents[1] / 'standalone' / 'reset.sh'
PUCK_Z = 0.0705                 # constants.PUCK_Z — 서 있는 퍽 바닥 높이 (상자 윗면)
LIFT_MIN_M = 0.02               # 이만큼 이상 들려야 집기 성공 (첫 실행 실측: skill이 들어 올리는 높이 35~38 mm)
HELD_CLOSE_M, HELD_FINGER_M = 0.025, 0.04  # 손 안: 닫힘 방향 · 손가락 방향으로 이 안
NEIGHBOR_MOVED_M, NEIGHBOR_TILT_DEG = 0.01, 20.0  # 옆 퍽이 이만큼 밀리거나 기울면 '건드림'
MATCH_RADIUS, INSPECT_Z, TAG_WAIT_S = 0.04, 0.11, 3.0  # pick_load_play.py와 같은 값
PERCEIVE_WAIT_S, END_S = 45.0, 1.5  # 참값 인식 더미는 노드가 뜨자마자 냄 — 45 s는 노드 재시작 여유
FIELDS = ['seed', 'gap_cm', 'box_y_cm', 'box_yaw', 'target', 'shape', 'puck_yaw', 'dist_m', 'neighbor_cm', 'mass_scale', 'friction_scale',
          'category', 'reason', 'inspect_ok', 'perceived_err_mm', 'lift_mm', 'lift_end_mm', 'face_above_bottom_mm', 'close_off_mm', 'finger_off_mm',
          'tilt_deg', 'base_x_mm', 'base_y_mm', 'base_yaw_deg', 'neighbor_moved_mm', 'neighbor_tilt_deg', 'seconds']
FACE_DESIGN_MM = 40.0           # stand_plan.hpp: 손가락 끝면이 퍽 바닥 위 40 mm (퍽 높이 60 − 쥠 깊이 20)


def judge(row_truth):
    """Isaac 위치(row_truth = slot 파일의 target 행) -> (성공?, 값). 퍽이 LIFT_MIN_M 넘게 들렸고 손 안(알 수 있을 때)이면 성공."""
    lift = row_truth['z'] - PUCK_Z
    hand = row_truth.get('hand')
    held = hand is None or (abs(hand['close_off']) <= HELD_CLOSE_M and abs(hand['finger_off']) <= HELD_FINGER_M)
    return lift >= LIFT_MIN_M and held, lift


def category(reachable, perceived, tag_read, pick_ok, success):
    if not reachable:
        return 'unreachable'
    if not perceived:
        return 'not_perceived'
    if not tag_read:
        return 'tag_unread'
    if not pick_ok:
        return 'pick_skill_failed'
    return 'success' if success else 'pick_false_ok'  # skill은 성공 보고인데 퍽이 안 들림 (빈손 · 놓침)


def summarize(rows):
    n = len(rows)
    reach = [r for r in rows if r['category'] != 'unreachable']
    ok = [r for r in reach if r['category'] == 'success']
    lines = [f"시행 {n}, 도달 불가 {n - len(reach)}, 대상 {len(reach)}"]
    if reach:
        lines.append(f"성공 {len(ok)}/{len(reach)} = {100 * len(ok) / len(reach):.1f} %")
        for c, k in Counter(r['category'] for r in reach).most_common():
            lines.append(f"  {c}: {k}")
        reasons = Counter(r['reason'] for r in reach if r['reason'] and r['category'] != 'success')
        for c, k in reasons.most_common():
            lines.append(f"  이유 {c}: {k}")

        def by(name, key):
            g = defaultdict(list)
            for r in reach:
                g[key(r)].append(r['category'] == 'success')
            lines.append(f"{name}: " + ', '.join(f"{k} {sum(v)}/{len(v)}" for k, v in sorted(g.items())))
        by('간격 [cm]', lambda r: f"{int(float(r['gap_cm']) // 4) * 4}-{int(float(r['gap_cm']) // 4) * 4 + 4}")
        by('거리 [m]', lambda r: f"{int(float(r['dist_m']) * 20) / 20:.2f}")
        by('모양', lambda r: r['shape'])
        by('옆 퍽 간격 [cm]', lambda r: f"{int(float(r['neighbor_cm']) // 4) * 4}-{int(float(r['neighbor_cm']) // 4) * 4 + 4}")
        face = [float(r['face_above_bottom_mm']) for r in reach if r['face_above_bottom_mm'] not in ('', None)]
        if face:
            m = sum(face) / len(face)
            sd = (sum((x - m) ** 2 for x in face) / len(face)) ** 0.5
            lines.append(f"손가락 끝면 높이(퍽 바닥 위): 평균 {m:.1f} mm ± {sd:.1f} (설계 {FACE_DESIGN_MM:.0f}, 퍽 중앙 30, 윗면 60) — 들린 {len(face)}개")
    return '\n'.join(lines)


def self_test():
    assert judge({'z': PUCK_Z + 0.05, 'hand': {'close_off': 0.002, 'finger_off': 0.0}}) == (True, 0.05)
    assert not judge({'z': PUCK_Z + 0.05, 'hand': {'close_off': 0.06, 'finger_off': 0.0}})[0]  # 들렸지만 손 밖 (튕김)
    assert not judge({'z': PUCK_Z + 0.001})[0] and judge({'z': PUCK_Z + 0.036})[0]  # 안 들림 · 실측 35~38 mm는 성공
    assert judge({'z': PUCK_Z + 0.05})[0]  # 손 값 없음: 높이만
    assert category(False, False, False, False, False) == 'unreachable'
    assert category(True, False, False, False, False) == 'not_perceived'
    assert category(True, True, False, False, False) == 'tag_unread'
    assert category(True, True, True, False, False) == 'pick_skill_failed'
    assert category(True, True, True, True, False) == 'pick_false_ok'
    assert category(True, True, True, True, True) == 'success'
    base = dict(gap_cm='5', dist_m='0.2', shape='box', neighbor_cm='8', reason='', face_above_bottom_mm='41')
    rows = [dict(base, category='success'), dict(base, category='pick_false_ok', reason='grasp_empty', face_above_bottom_mm=''),
            dict(base, category='unreachable')]
    out = summarize(rows)
    assert '성공 1/2 = 50.0 %' in out and '도달 불가 1' in out and 'grasp_empty' in out and '41.0 mm' in out, out
    print('self-test ok')


def save_sample(folder, seed, msg, truth, tgt):
    """gripper_cam 영상 한 장(PNG)과 손 안 퍽 참값을 folder에 — samples.jsonl 한 줄."""
    import cv2
    import numpy as np
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
    if msg.encoding.startswith('rgb'):
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    name = f'seed{seed:04d}.png'
    cv2.imwrite(str(folder / name), img)
    with open(folder / 'samples.jsonl', 'a') as f:
        f.write(json.dumps({'file': name, 'seed': seed, 'shape': tgt['shape'], 'target': tgt['name'], 'lift': truth['z'] - PUCK_Z,
                            'hand': truth.get('hand')}) + '\n')


def read_json(path, max_age=None):
    try:
        with open(path) as f:
            d = json.load(f)
    except (OSError, ValueError):
        return None
    return d if max_age is None or time.time() - d['t'] < max_age else None


def main(cli):
    import rclpy
    from perception.msg import TrackedObjectArray
    from plan.msg import SubGoal, SubGoalArray, TaskStatus
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    from ml.srv import ScoreGrasps
    from std_srvs.srv import Trigger
    from sensor_msgs.msg import Image
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node('pick_test')
    state = {'objects': None, 'status': None, 'img': None}
    if cli.save_images:  # 파지 직후 gripper_cam 영상 + 손 안 퍽 참값 (손 안 퍽 위치 추정 정확도 측정용)
        node.create_subscription(Image, '/gripper_cam/color/image_raw', lambda m: state.update(img=m), qos_profile_sensor_data)
    latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(TrackedObjectArray, '/tracked_objects', lambda m: state.update(objects=m), 5)
    node.create_subscription(TaskStatus, '/task/status', lambda m: state.update(status=m), latched)
    pub = {'goals': node.create_publisher(SubGoalArray, '/task/sub_goals', latched)}
    seq = [2000000 + int(time.time()) % 100000 * 10]

    def wait(cond, timeout, what):
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            r = cond()
            if r:
                return r
        print(f'  시간 초과: {what}')
        return None

    def sleep(s):
        t0 = time.monotonic()
        while time.monotonic() - t0 < s:
            rclpy.spin_once(node, timeout_sec=0.1)

    def run(skill, target, timeout=180.0):
        seq[0] += 1
        plan = SubGoalArray(plan_id=seq[0], goals=[SubGoal(step=1, skill=skill, target=target, relation='')])
        plan.header.stamp = node.get_clock().now().to_msg()
        pub['goals'].publish(plan)
        st = wait(lambda: (lambda s: s if s and s.plan_id == plan.plan_id and s.state != TaskStatus.RUNNING else None)(state['status']),
                  timeout, f'{skill}')
        ok = bool(st) and st.state == TaskStatus.SUCCEEDED
        reason = '' if ok else (st.reason if st else 'timeout')
        print(f"  {skill} {target}: {'성공' if ok else '실패 ' + reason}")
        return ok, reason

    def puck_near(xy, need_tag=False):
        best, best_d = None, MATCH_RADIUS
        for o in (state['objects'].objects if state['objects'] else []):
            if not o.class_name.startswith('puck'):
                continue
            d = math.hypot(o.pose.pose.position.x - xy[0], o.pose.pose.position.y - xy[1])
            if d < best_d and (not need_tag or any(kv.key == 'tag' and kv.value for kv in o.details)):
                best, best_d = o, d
        return best

    score_cli = node.create_client(ScoreGrasps, '/score_grasps')

    def why_no_grasp(oid):
        """pick이 no_grasp로 끝났을 때 스코어러를 직접 불러 이유(message)를 받음 — 진단용."""
        if not score_cli.wait_for_service(timeout_sec=2.0):
            return 'score_grasps 서비스 없음'
        fut = score_cli.call_async(ScoreGrasps.Request(object_id=int(oid), max_candidates=3))
        rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
        r = fut.result()
        return f'{r.message} (후보 {len(r.grasps)})' if r else '응답 없음'

    NEED_SERVICES = {'/score_grasps', '/clear_octomap', '/apply_planning_scene'}

    def ready():  # 리셋 직후 ROS가 막 떴을 때 계획을 보내면 grasp_scorer가 /tracked_objects를 못 받은 채 no_grasp (11-06 실측)
        svcs = {n for n, _ in node.get_service_names_and_types()}
        return NEED_SERVICES <= svcs and '/skills/pick/_action/status' in {n for n, _ in node.get_topic_names_and_types()}

    clear_cli = node.create_client(Trigger, '/coordinator/basket_clear')

    def new_trial(seed, first):
        cur = read_json(TRIAL_FILE) or {}
        if first and cur.get('seed') == seed:
            return cur
        t_req = time.time()
        got = lambda: (lambda d: d if d and d['seed'] == seed and d['t'] > t_req else None)(read_json(TRIAL_FILE))
        if not cli.hard:  # 기본: ROS를 안 죽이고 팔만 stow로 · 손 장부 비움 · 장면만 새 배치 (시행당 수 초)
            if not first:
                run('pose', 'stow')
                if clear_cli.wait_for_service(timeout_sec=2.0):
                    rclpy.spin_until_future_complete(node, clear_cli.call_async(Trigger.Request()), timeout_sec=3.0)
            Path(SEED_FILE).write_text(str(seed))
            Path(SOFT_FILE).touch()
            return wait(got, 60.0, f'seed {seed} 배치 (soft)')
        Path(SEED_FILE).write_text(str(seed))
        state.update(objects=None, status=None)
        node.destroy_publisher(pub['goals'])  # 늦게 뜬 coordinator에 지난 계획이 다시 가지 않게 (latched)
        pub['goals'] = node.create_publisher(SubGoalArray, '/task/sub_goals', latched)
        subprocess.run([str(RESET_SH)], check=True)
        return wait(got, 180.0, f'seed {seed} 배치')

    def truth_after(done_t, name):
        """pick이 끝난 직후 첫 Isaac 위치(손 안 · 들림 판정)와 END_S 뒤 위치(끝까지 들고 있는지) — 영상도 첫 시점 것."""
        first = end = img = None
        t_first = 0.0
        t0 = time.monotonic()
        while time.monotonic() - t0 < END_S + 4.0:
            sleep(0.05)
            d = read_json(SLOT_FILE, 5.0)
            if not d or d['t'] <= done_t or name not in d['pucks']:
                continue
            if first is None:
                first, img, t_first = d['pucks'], state['img'], d['t']
            elif d['t'] >= t_first + END_S:
                end = d['pucks']
                break
        return first, end, img

    rows = []
    out = Path(cli.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, FIELDS)
        w.writeheader()
        for k in range(cli.trials):
            seed = cli.seed + k
            print(f'== 시행 {k + 1}/{cli.trials} seed {seed}')
            t0 = time.monotonic()
            trial = new_trial(seed, k == 0)
            if not trial:
                print('  배치를 못 받음 — 중단 (Isaac이 --random-layout으로 떠 있는지)')
                break
            wait(ready, 90.0, 'pick skill · grasp_scorer · MoveIt 준비')
            sleep(2.0 if cli.hard else 0.7)  # 스코어러가 /tracked_objects 첫 장면을 받을 시간
            ps = trial['pucks']
            reach = [p for p in ps if p['reachable']]
            tgt = random.Random(seed + 1).choice(reach) if reach else ps[0]
            other = next(p for p in ps if p is not tgt)
            row = dict.fromkeys(FIELDS, '')
            row.update(seed=seed, gap_cm=round(100 * trial['gap'], 1), box_y_cm=round(100 * trial['box'][1], 1), box_yaw=round(trial['box_yaw'], 1),
                       target=tgt['name'], shape=tgt['shape'], puck_yaw=round(tgt['yaw'], 1), dist_m=round(tgt['dist'], 3),
                       neighbor_cm=round(100 * math.hypot(tgt['x'] - other['x'], tgt['y'] - other['y']), 1),
                       mass_scale=round(tgt['mass_scale'], 2), friction_scale=round(tgt['friction_scale'], 2))
            perceived = tag_read = pick_ok = success = False
            if tgt['reachable']:
                xy = (tgt['x'], tgt['y'])
                perceived = bool(wait(lambda: puck_near(xy), PERCEIVE_WAIT_S, f'퍽 인식 (대상 {xy[0]:.3f}, {xy[1]:.3f}, 옆 퍽 {other["x"]:.3f}, {other["y"]:.3f})'))
                if perceived:  # inspect에서 시작 (접근은 제외) — 참값 더미는 태그를 처음부터 주지만 팔이 위에서 내려다본 자세에서 pick을 시작
                    if not cli.no_inspect:
                        row['inspect_ok'], _ = run('inspect', f'{xy[0]:.4f},{xy[1]:.4f},{INSPECT_Z}')
                    o = puck_near(xy, need_tag=True)
                    tag_read = o is not None
                    if tag_read:
                        p = o.pose.pose.position
                        row['perceived_err_mm'] = round(1000 * math.hypot(p.x - xy[0], p.y - xy[1]), 1)
                        pick_ok, row['reason'] = run('pick', str(o.id))
                        if not pick_ok and row['reason'] in ('no_grasp', 'grasp_failed_all'):
                            print('  진단:', why_no_grasp(o.id), '| 트랙', [(x.id, x.class_name, round(x.pose.pose.position.x, 3)) for x in state['objects'].objects] if state['objects'] else None)
                        done_t = time.time() - 0.3  # run()이 상태를 받은 시각 근처 — 그 직후 첫 위치부터
                        rows_t, rows_end, img = truth_after(done_t, tgt['name'])
                        if rows_t:
                            success, lift = judge(rows_t[tgt['name']])
                            h = rows_t[tgt['name']].get('hand')
                            row['lift_mm'] = round(1000 * lift, 1)
                            bs = rows_t.get('_base')
                            if bs:
                                row.update(base_x_mm=round(1000 * bs['x'], 1), base_y_mm=round(1000 * bs['y'], 1), base_yaw_deg=round(bs['yaw'], 2))
                            if rows_end:
                                row['lift_end_mm'] = round(1000 * (rows_end[tgt['name']]['z'] - PUCK_Z), 1)
                            if cli.save_images and img is not None:
                                save_sample(cli.save_images, seed, img, rows_t[tgt['name']], tgt)
                            if h and lift >= LIFT_MIN_M:
                                row.update(face_above_bottom_mm=round(1000 * h['face_above_bottom'], 1), close_off_mm=round(1000 * h['close_off'], 1),
                                           finger_off_mm=round(1000 * h['finger_off'], 1), tilt_deg=round(h['tilt_deg'], 1))
                            n = rows_t.get(other['name'])
                            if n:
                                row['neighbor_moved_mm'] = round(1000 * math.hypot(n['xy'][0] - other['x'], n['xy'][1] - other['y']), 1)
                                row['neighbor_tilt_deg'] = round(n['up_tilt'], 1)
                        else:
                            print(f'  {SLOT_FILE} 못 읽음 — 판정 못 함')
            row['category'] = category(tgt['reachable'], perceived, tag_read, pick_ok, success)
            row['seconds'] = round(time.monotonic() - t0)
            print(f"  -> {row['category']} {row['reason']}")
            w.writerow(row)
            f.flush()
            rows.append(row)
    print('\n' + summarize(rows))
    print(f'CSV: {out}')
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--trials', type=int, default=30)
    parser.add_argument('--seed', type=int, default=0, help='첫 시행 seed — Isaac의 --trial-seed와 같게')
    parser.add_argument('--no-inspect', action='store_true', help='inspect 없이 stow 자세에서 바로 pick')
    parser.add_argument('--hard', action='store_true', help='시행마다 reset.sh로 장면과 ROS를 다시 띄움 (기본은 ROS를 두고 장면만 새 배치)')
    parser.add_argument('--save-images', help='파지 직후 gripper_cam 영상과 손 안 퍽 참값을 이 폴더에 저장')
    parser.add_argument('--out', default=time.strftime('/tmp/pick_test_%m%d_%H%M.csv'))
    cli = parser.parse_args()
    if cli.self_test:
        self_test()
        sys.exit(0)
    sys.exit(main(cli))
