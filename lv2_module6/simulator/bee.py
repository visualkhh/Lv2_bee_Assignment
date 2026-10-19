#!/usr/bin/env python3
"""호스트용 명령 CLI — 컨테이너에 들어가지 않고 debug/ 파일로 지시한다.

  .venv/bin/python bee.py user "빨간 퍽을 목표 구역으로 옮겨"
  .venv/bin/python bee.py estop on|off
  .venv/bin/python bee.py manual on|off
  .venv/bin/python bee.py twist 0.1 0.0          # 수동 모드에서만 먹음
  .venv/bin/python bee.py service score_grasps '{"object_id": 3}' [--timeout 15]
  .venv/bin/python bee.py action navigate_to_pose '{"x": 1.0, "y": 0.5}' [--timeout 120]
  .venv/bin/python bee.py cancel navigate_to_pose <goal_id>
  .venv/bin/python bee.py reset   # 로봇 위치 초기화 (odom 0 = spawn)
  .venv/bin/python bee.py interface ml/srv/ScoreGrasps   # 타입 정의 보기
  .venv/bin/python bee.py status
  .venv/bin/python bee.py snap out.png           # main.py --snapshot 위임

service/action은 request_id·goal_id 일치로 자기 응답만 받는다 (stale 무시).
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from bridge import (DEBUG, cancel_action_goal, read_action_file, read_action_list,
                    read_interface_info, read_msg, read_service_list, read_service_response,
                    read_topic_list, request_interface_info, cmd_manual_mode, send, send_action_goal, send_bool,
                    send_service_request, send_twist)


def wait_for(fn, match, timeout, interval=0.2):
    """fn() → (data, age). match(data)가 True인 첫 결과를 기다림. (data, None) 또는 (None, 사유)"""
    end = time.time() + timeout
    while time.time() < end:
        data, _ = fn()
        if data is not None and match(data):
            return data, None
        time.sleep(interval)
    return None, f'timeout {timeout}s'


def cmd_user(args):
    send('/cmd/user_command', 'std_msgs/msg/String', {'data': args.text})
    print(f'보냄 /cmd/user_command: {args.text}')


def cmd_estop(args):
    send_bool('/cmd/e_stop', args.state == 'on')
    print(f'/cmd/e_stop = {args.state}')


def cmd_manual(args):
    cmd_manual_mode(args.state == 'on')
    print(f'/cmd/operation_mode = {"MANUAL" if args.state == "on" else "AUTO"} (결과 /state/operation_mode)')


def cmd_twist(args):
    send_twist(args.v, args.w)
    print(f'/cmd/manual/cmd_vel v={args.v} w={args.w} (수동 모드에서만 동작)')


def cmd_service(args):
    try:
        data = json.loads(args.json)
    except ValueError as e:
        print(f'JSON 오류: {e}');
        return 1
    if not isinstance(data, dict):
        print('data는 JSON 객체여야 함');
        return 1
    qid = send_service_request(args.name, data)
    print(f'요청 {args.name} id={qid} ...')
    resp, err = wait_for(lambda: read_service_response(args.name),
                         lambda d: d.get('request_id') == qid, args.timeout)
    if resp is None:
        print(f'응답 없음: {err} (컨테이너에 test-faker가 떠 있는지 확인)');
        return 1
    print(json.dumps(resp, ensure_ascii=False, indent=1))
    return 0 if resp.get('ok') else 2


def cmd_action(args):
    try:
        data = json.loads(args.json)
    except ValueError as e:
        print(f'JSON 오류: {e}');
        return 1
    if not isinstance(data, dict):
        print('data는 JSON 객체여야 함');
        return 1
    gid = send_action_goal(args.name, data)
    print(f'goal {args.name} id={gid} ...')
    acc, err = wait_for(lambda: read_action_file(args.name, 'accept'),
                        lambda d: d.get('goal_id') == gid, 15)
    if acc is None:
        print(f'accept 없음: {err}');
        return 1
    print(json.dumps(acc, ensure_ascii=False))
    if not acc.get('accepted') or args.no_wait:
        return 0 if acc.get('accepted') else 2
    res, err = wait_for(lambda: read_action_file(args.name, 'result'),
                        lambda d: d.get('goal_id') == gid, args.timeout)
    if res is None:
        print(f'result 없음: {err} (cancel로 중단 가능)');
        return 1
    print(json.dumps(res, ensure_ascii=False, indent=1))
    return 0 if res.get('status') == 'succeeded' else 2


def cmd_cancel(args):
    cancel_action_goal(args.name, args.goal_id)
    print(f'취소 요청 {args.name} id={args.goal_id}')


def cmd_reset(_):
    for svc, data in (('diff_drive_controller/set_odometry', {'x': 0.0, 'y': 0.0, 'yaw': 0.0}),
                      ('reset', {})):
        qid = send_service_request(svc, data)
        print(f'위치 초기화 요청 {svc} id={qid} ...')
        resp, err = wait_for(lambda s=svc: read_service_response(s),
                             lambda d: d.get('request_id') == qid, 10)
        if resp is None:
            print(f'응답 없음: {err}');
            return 1
        print(json.dumps(resp, ensure_ascii=False))
        if not resp.get('ok'):
            return 2
    return 0


def cmd_interface(args):
    request_interface_info(args.type)
    print(f'정의 요청 {args.type} ...')
    info, err = wait_for(lambda: read_interface_info(args.type),
                         lambda d: bool(d and d.strip()), 15)
    if info is None:
        print(f'info 없음: {err} (service_action_monitor가 떠 있는지 확인)');
        return 1
    print(info)


def cmd_status(_):
    data, _ = read_msg('task_status')
    print(f"작업: {data if data else '(없음 — test-faker 실행 중인지 확인)'}")
    data, _ = read_msg('state/operation_mode')
    print(f"운영 모드: {data['data'] if data else '(없음)'}")
    data, _ = read_msg('odom')
    print(f"위치(odom): {data['pose']['pose']['position'] if data else '(없음)'}")
    svc, _ = read_service_list()
    print(f"service: {[s['name'] for s in (svc or {}).get('services', [])]}")
    act, _ = read_action_list()
    print(f"action: {[a['name'] for a in (act or {}).get('actions', [])]}")
    print(f'debug dir: {DEBUG}')


def cmd_snap(args):
    r = subprocess.run([sys.executable, str(HERE / 'main.py'), '--snapshot', args.out])
    return r.returncode


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd')

    s = sub.add_parser('user', help='텍스트 명령 → /cmd/user_command');
    s.add_argument('text');
    s.set_defaults(fn=cmd_user)

    s = sub.add_parser('estop', help='비상정지 on|off');
    s.add_argument('state', choices=('on', 'off'));
    s.set_defaults(fn=cmd_estop)

    s = sub.add_parser('manual', help='수동 모드 on|off');
    s.add_argument('state', choices=('on', 'off'));
    s.set_defaults(fn=cmd_manual)

    s = sub.add_parser('twist', help='수동 바퀴 속도');
    s.add_argument('v', type=float);
    s.add_argument('w', type=float);
    s.set_defaults(fn=cmd_twist)

    s = sub.add_parser('service', help='서비스 호출 후 응답 대기');
    s.add_argument('name');
    s.add_argument('json');
    s.add_argument('--timeout', type=float, default=15);
    s.set_defaults(fn=cmd_service)

    s = sub.add_parser('action', help='액션 goal 전송 후 결과 대기');
    s.add_argument('name');
    s.add_argument('json');
    s.add_argument('--timeout', type=float, default=120);
    s.add_argument('--no-wait', action='store_true');
    s.set_defaults(fn=cmd_action)

    s = sub.add_parser('cancel', help='진행 중 goal 취소');
    s.add_argument('name');
    s.add_argument('goal_id');
    s.set_defaults(fn=cmd_cancel)

    s = sub.add_parser('reset', help='로봇 위치 초기화 (odom 0 = 시작 칸)');
    s.set_defaults(fn=cmd_reset)

    s = sub.add_parser('interface', help='타입 정의 보기 (debug/interface/<타입>/info)');
    s.add_argument('type');
    s.set_defaults(fn=cmd_interface)

    s = sub.add_parser('status', help='작업·모드·위치·목록 요약');
    s.set_defaults(fn=cmd_status)

    s = sub.add_parser('snap', help='지도+카메라 한 장 저장');
    s.add_argument('out');
    s.set_defaults(fn=cmd_snap)

    args = p.parse_args()
    if not args.cmd:
        p.print_help()
        return 0
    sys.exit(args.fn(args) or 0)


if __name__ == '__main__':
    main()
