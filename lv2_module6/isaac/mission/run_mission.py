#!/usr/bin/env python3
"""Isaac 전체 미션 한 판 — 신호등 · 시작 바코드는 읽었다고 치고 mission_manager에 start를 보내 끝까지 지켜본다.

  python3 isaac/mission/run_mission.py                          # R5 Y3 G4 · P2-01 (scene_sim 퍽으로 되는 조합)
  python3 isaac/mission/run_mission.py --seq RGRRGRGGYGYG --code P2-02
  python3 isaac/mission/run_mission.py --clear-basket           # 적재함 장부가 남아 시작이 거부될 때 먼저 비움

환경: ros2_ws/install/setup.bash · ROS_DOMAIN_ID · FASTDDS_DEFAULT_PROFILES_FILE이 Isaac 쪽(isaac/mission/isaac.sh)과 같아야 한다.
흐름 (mission_manager): GO_OBSERVE → BARCODE → READ → SURVEY → BUILD → VERIFY → RETURN → DONE
  BARCODE 단계 동안 /mission/qr, READ 단계 동안 /mission/signal_status(ok=true)를 1초마다 낸다 — 단계가 시작된 뒤에 받은 값만 쓰기 때문.
  나머지(이동 · 인지 · 집기 · 놓기)는 실기와 같은 노드가 그대로 한다.
신호 조건 (mission_solver allocate): R ≤ 빨간 퍽 수 · G ≤ 초록 퍽 수 · 남는 퍽(Y)이 한 모양 — scene_sim은 빨강 7(box 4 · cyl 3) · 초록 5(box 2 · cyl 3)
  예: R5 Y3 G4 됨 · 공개 연습 Q1(R4 Y2 G6)은 초록이 모자라 no_allocation
Ctrl+C = /cmd/mission abort. 끝나면 DONE 0 · ERROR 1로 나간다. 회차 기록: ~/.ros/mission/<시각>/
"""
import argparse
import sys

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from perception.msg import MissionSignal
from plan.msg import MissionState

NAMES = {'R': 'red', 'Y': 'yellow', 'G': 'green'}
IDLE = ('IDLE', 'DONE', 'ERROR')   # start를 받는 단계 (DONE · ERROR 뒤에도 phase는 그대로 남음)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seq', default='RGYRGRYGRGYR', help='신호등 12회 색 (R/Y/G) — 기본 R5 Y3 G4')
    ap.add_argument('--code', default='P2-01', help='시작 바코드 — P2-01 · P2-03 오름차순 · P2-02 · P2-04 내림차순')
    ap.add_argument('--clear-basket', action='store_true', help='시작 전 /coordinator/basket_clear')
    a = ap.parse_args()
    seq = a.seq.upper()
    if len(seq) != 12 or set(seq) - set(NAMES) or not all(1 <= seq.count(c) <= 6 for c in NAMES):
        sys.exit(f'--seq {a.seq}: R/Y/G 12개 · 색마다 1~6회여야 함')

    rclpy.init()
    n = rclpy.create_node('run_mission', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    log = n.get_logger()
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    cmd = n.create_publisher(String, '/cmd/mission', 10)
    qr = n.create_publisher(String, '/mission/qr', latched)
    sig = n.create_publisher(MissionSignal, '/mission/signal_status', 10)
    st = {'phase': None, 'last': None, 'started': False, 'running': False, 'exit': None}

    if a.clear_basket:
        c = n.create_client(Trigger, '/coordinator/basket_clear')
        if not c.wait_for_service(timeout_sec=20.0):
            sys.exit('/coordinator/basket_clear 안 보임 — coordinator가 떠 있는지 확인')
        f = c.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(n, f, timeout_sec=10.0)
        log.info(f'적재함 장부: {f.result().message if f.result() else "응답 없음"}')

    def on_state(m):
        st['phase'] = m.phase
        line = (m.phase, m.note, m.error, m.placed, tuple(m.results))
        if line != st['last']:
            st['last'] = line
            res = f' · 결과 {list(m.results)[-1]}' if m.results else ''
            log.info(f'[{m.phase}] {m.note} · 놓음 {m.placed}/{m.total}{res}' + (f' · 오류 {m.error}' if m.error else ''))
        if st['started'] and m.phase not in IDLE:
            st['running'] = True
        if st['running'] and m.phase in ('DONE', 'ERROR'):
            st['exit'] = 0 if m.phase == 'DONE' else 1
        if st['started'] and '시작 거부' in m.note:   # 적재함 장부가 남음 — --clear-basket
            st['exit'] = 1

    n.create_subscription(MissionState, '/mission/state', on_state, latched)

    def tick():
        if not st['started']:
            if st['phase'] in IDLE and cmd.get_subscription_count() > 0:
                cmd.publish(String(data='start'))
                st['started'] = True
                log.info(f'start · 신호 {" ".join(seq)} (R{seq.count("R")} Y{seq.count("Y")} G{seq.count("G")}) · 바코드 {a.code}')
            elif st['phase'] not in (None, *IDLE):
                st['exit'] = 1
                log.error(f'미션이 이미 {st["phase"]} — 끝나거나 abort한 뒤에 다시')
            return
        if st['phase'] == 'BARCODE':
            qr.publish(String(data=a.code))
        elif st['phase'] == 'READ':
            m = MissionSignal(sequence=[NAMES[c] for c in seq], red=seq.count('R'), yellow=seq.count('Y'),
                              green=seq.count('G'), ok=True, cycles=2, raw=' '.join(seq))
            m.header.stamp = n.get_clock().now().to_msg()
            sig.publish(m)

    n.create_timer(1.0, tick)
    log.info('/mission/state 기다리는 중 (mission_manager)')
    try:
        while rclpy.ok() and st['exit'] is None:
            rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        if st['started']:
            cmd.publish(String(data='abort'))
            rclpy.spin_once(n, timeout_sec=0.5)
            print('abort 보냄')
        st['exit'] = 1
    sys.exit(st['exit'])


if __name__ == '__main__':
    main()
