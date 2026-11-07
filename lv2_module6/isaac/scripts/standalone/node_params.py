"""bringup이 띄운 노드 하나를 고친 파라미터로 다시 띄운다 (isaac_system.launch.py가 노드마다 실행).

  python3 node_params.py <노드 이름> [이름:=값 …] [--after 노드,노드…]

1. -p를 params 파일 뒤로: rcl은 노드 이름 항목('/**'와 '<노드>'는 서로 다른 항목)이 명령줄에 처음 나온
   순서대로 파라미터를 적용한다. 실기에서는 bringup이 넘기는 '/**' params 파일이 패키지 yaml 뒤에 와서 이긴다.
   Isaac에서는 SetParameter(use_sim_time)가 '-p use_sim_time:=True'를 params 파일보다 앞에 붙여 '/**' 항목이
   먼저 생기고, 패키지 yaml의 노드 이름 값이 이긴다. 예: state_manager가 manager.yaml의 LOCALIZATION을 읽어
   map:=slam인데도 EXPLORE를 거절했다 (2026-10-30 실측: 앞쪽 -p가 없으면 MAPPING, 하나라도 있으면 LOCALIZATION).
2. 덧붙일 값: 인자로 받은 '이름:=값'을 명령줄 맨 끝에 -p로 붙인다 (Isaac에서만 바꿀 값 — 예: 스캔 자세).
3. --after: 적은 노드들이 먼저 고쳐져 다시 뜬 뒤에 이 노드를 다시 띄운다. state_manager가 MAPPING을 먼저 내면
   아직 안 고친 coordinator가 옛 스캔 자세로 팔을 보냈다가, 고친 coordinator가 새 자세로 되돌렸다.

ros2_ws 패키지는 실기 그대로 두고 밖에서 고친다. 이 launch가 띄운 그 노드를 기다렸다가 SIGINT로 끄고,
고친 명령줄로 exec한다. 프로세스는 같은 ros2 launch의 자식으로 남으므로 /cmd/restart all과 node 재시작이
그대로 찾아서 끈다 (node 재시작은 고친 명령줄을 그대로 다시 씀).
"""
import os
import signal
import sys
import time


def argv_of(pid):
    with open(f'/proc/{pid}/cmdline', 'rb') as f:
        return [a.decode() for a in f.read().split(b'\0') if a]


def ppid_of(pid):
    with open(f'/proc/{pid}/stat') as f:
        return int(f.read().rsplit(')', 1)[1].split()[1])


def node_name(argv):
    """-r __node:=<이름>이 있으면 그 이름, 없으면 실행 파일 이름 (python 스크립트면 스크립트 이름)."""
    for a in argv:
        if a.startswith('__node:='):
            return a.split(':=', 1)[1]
    exe = argv[1] if os.path.basename(argv[0]).startswith('python') and len(argv) > 1 else argv[0]
    return os.path.basename(exe)


def find_node(launch_pid, name):
    for p in filter(str.isdigit, os.listdir('/proc')):
        try:
            argv = argv_of(int(p))
            if ppid_of(int(p)) == launch_pid and '--ros-args' in argv and node_name(argv) == name:
                return int(p)
        except (OSError, IndexError):
            pass
    return 0


def params_last(argv):
    """모든 '-p 이름:=값'을 나머지 인자 뒤로 옮긴다 (-p끼리의 순서는 유지)."""
    rest, params, i = [], [], 0
    while i < len(argv):
        if argv[i] == '-p' and i + 1 < len(argv):
            params += argv[i:i + 2]
            i += 2
        else:
            rest.append(argv[i])
            i += 1
    return rest + params


def fixed_already(launch_pid, name):
    """그 노드가 고친 명령줄(-p가 params 파일 뒤)로 떠 있나."""
    pid = find_node(launch_pid, name)
    return bool(pid) and params_last(argv_of(pid)) == argv_of(pid)


def wait_for(cond, what):
    deadline = time.time() + 120
    while not (r := cond()):
        if time.time() > deadline:
            sys.exit(f'node_params: 120초 안에 {what}')
        time.sleep(0.5)
    return r


def main():
    args = sys.argv[1:]
    after = []
    if '--after' in args:
        i = args.index('--after')
        after = args[i + 1].split(',')
        del args[i:i + 2]
    name, extra = args[0], args[1:]
    launch_pid = os.getppid()
    for other in after:
        wait_for(lambda: fixed_already(launch_pid, other), f'{other}이(가) 고쳐져 다시 뜨지 않음')
    pid = wait_for(lambda: find_node(launch_pid, name), f'이 launch 아래 {name}이(가) 안 뜸')
    argv = argv_of(pid)
    fixed = params_last(argv)
    for kv in extra:
        fixed += ['-p', kv]
    if fixed == argv:  # 고칠 것 없음
        return
    os.kill(pid, signal.SIGINT)
    for _ in range(40):  # 10초 기다리고 안 꺼지면 SIGKILL
        if not os.path.exists(f'/proc/{pid}'):
            break
        time.sleep(0.25)
    else:
        os.kill(pid, signal.SIGKILL)
    print(f'node_params: {name} 다시 띄움 (-p를 params 파일 뒤로{", 덧붙임 " + " ".join(extra) if extra else ""})',
          flush=True)
    os.execvp(fixed[0], fixed)  # python3 스크립트 노드는 argv[0]이 'python3'라 PATH에서 찾음


if __name__ == '__main__':
    assert params_last(['x', '--ros-args', '-p', 'a:=1', '--params-file', 'f', '-p', 'b:=2']) == \
        ['x', '--ros-args', '--params-file', 'f', '-p', 'a:=1', '-p', 'b:=2']
    assert node_name(['python3', '/a/scan_cam_gate', '--ros-args', '-r', '__node:=pole_depth_gate']) == 'pole_depth_gate'
    assert node_name(['/a/coordinator', '--ros-args']) == 'coordinator'
    main()
