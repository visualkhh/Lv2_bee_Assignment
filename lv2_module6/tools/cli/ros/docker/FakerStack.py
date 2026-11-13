"""도커 모드 — 컨테이너의 test-faker 0이 떠 있는지 보장한다.

`run-cli --source docker`는 파일 브리지(/debug)만 연결한다. 컨테이너 ROS 스택이
꺼져 있으면 SIM은 프레임을 쓰지만 fake_camera·monitor가 없어 ROS 쪽이 전부 stale이 된다.
그래서 docker 모드 진입 시:
  1. 컨테이너가 떠 있는지 확인
  2. `ros2 launch faker`가 없으면 `test-faker 0`을 백그라운드로 시작 — 떠 있으면 붙기만 한다 (실기처럼: 죽이거나 다시 띄우지 않음)
  3. debug/output/topic/{state/devices,joint_states}/message가 fresh해질 때까지 대기
종료할 때 faker를 끄지 않는다 (bringup처럼 다음 실행에서 재사용).
"""
from __future__ import annotations

import json
import os
import subprocess
import time
from pathlib import Path

DEFAULT_CONTAINER = 'lv2_module6'

# 정적 baseline — CLI/뷰어가 뜨기 전에도 필요한 최소 토픽.
# 나머지(진단·카메라·포인트클라우드 등)는 호스트가 input/monitor/*.topics로 동적 요청한다.
TOPICS = (
    '/state/operation_mode', '/state/map_mode', '/state/map_name', '/state/map_list',
    '/state/collision',
    '/state/devices', '/state/restartable', '/state/topics', '/state/restart', '/state/emergency',
    '/state/devices/pole_front_cam', '/state/devices/gripper_cam', '/state/devices/opencr',
    '/state/devices/imu', '/state/devices/wheels', '/state/devices/arm', '/state/devices/gripper',
    '/joint_states', '/odom', '/odometry/filtered', '/imu',
    '/diagnostics_agg', '/tracked_objects', '/motor_status', '/task/status',
    '/pole_front_cam/color/camera_info', '/gripper_cam/color/camera_info',
    '/robot_description', '/fake_opencr/ground_truth',
)


class FakerStack:
    def __init__(self, debug_dir, container=None, timeout: float = 180.0):
        self.debug_dir = Path(debug_dir)
        self.container = container or os.environ.get('BEEBOT_CONTAINER', DEFAULT_CONTAINER)
        self.timeout = timeout

    def _docker(self, *args):
        return subprocess.run(['docker', *args], text=True, capture_output=True)

    def container_running(self) -> bool:
        try:
            r = self._docker('inspect', '-f', '{{.State.Running}}', self.container)
        except FileNotFoundError:
            print('[docker] docker CLI 없음 — 직접 test-faker 0을 띄우거나 --no-faker로 실행',
                  flush=True)
            return False
        return r.returncode == 0 and r.stdout.strip() == 'true'

    def faker_running(self) -> bool:
        # [r]os2 — 이 셸의 명령줄에 있는 문자열과 자기 자신을 매칭하지 않게
        r = self._docker('exec', self.container, 'bash', '-lc',
                         "pgrep -f '[r]os2 launch faker faker.launch' >/dev/null")   # ROS 스택 (fake_hardware.launch = 하드웨어는 제외)
        return r.returncode == 0

    def stack_busy(self) -> bool:
        r = self._docker('exec', self.container, 'bash', '-lc',
                         "pgrep -f '[t]est-faker|[b]uild-ws|[c]olcon build' >/dev/null")
        return r.returncode == 0

    def _fresh(self) -> str | None:
        for rel in ('output/topic/state/devices/message',
                    'output/topic/joint_states/message'):
            try:
                age = time.time() - (self.debug_dir / rel).stat().st_mtime
            except OSError:
                continue
            if age < 3.0:
                return rel
        return None

    def _write_topics(self) -> None:
        path = self.debug_dir / 'input' / 'monitor' / 'topics'
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('\n'.join(TOPICS) + '\n')
            print(f'[docker] monitor 화이트리스트 {len(TOPICS)}개 — {path}', flush=True)
        except OSError as exc:
            print(f'[docker] monitor 화이트리스트 쓰기 실패: {exc}', flush=True)

    def _tail(self) -> str:
        r = self._docker('exec', self.container, 'bash', '-lc', 'tail -n 10 /tmp/faker.log')
        return r.stdout.strip()

    def _state_value(self, key: str) -> str:
        try:
            msg = json.loads((self.debug_dir / 'output' / 'topic' / 'state' / key / 'message').read_text())
            return str(msg.get('data', {}).get('data') or '')
        except (OSError, ValueError, AttributeError):
            return ''

    def _start_args(self) -> list[str]:
        """아무것도 안 떠 있을 때만 — 마지막 /state/map_mode로 띄운다 (MAPPING이면 새 지도, LOCALIZATION이면 그 지도)."""
        mode = self._state_value('map_mode')
        name = self._state_value('map_name')
        if mode == 'LOCALIZATION' and name:
            return ['map:=saved', f'map_name:={name}']
        if mode == 'MAPPING':
            return ['map:=slam']
        return ['map:=none']  # NONE 또는 상태 없음 — 첫 부팅은 지도 없음에서 시작

    def _start_faker(self) -> bool:
        args = self._start_args()
        print(f'[docker] test-faker 0 {" ".join(args)} 시작 (컨테이너 {self.container}) — '
              '빌드 때문에 1분쯤 걸린다', flush=True)
        r = self._docker('exec', '-d', self.container, 'bash', '-lc',
                         'source /opt/ros/lyrical/setup.bash; exec test-faker 0 ' + ' '.join(args))
        if r.returncode != 0:
            print(f'[docker] test-faker 시작 실패: {r.stderr.strip()}', flush=True)
            return False
        return True

    def ensure(self) -> bool:
        if not self.container_running():
            print(f'[docker] 컨테이너 {self.container!r}가 꺼져 있다 — 시작한다', flush=True)
            r = self._docker('start', self.container)
            if r.returncode != 0:
                print(f'[docker] 컨테이너 시작 실패: {r.stderr.strip()}', flush=True)
                return False
            for _ in range(20):
                if self.container_running():
                    break
                time.sleep(0.5)
            if not self.container_running():
                print('[docker] 컨테이너가 10초 안에 뜨지 않았다', flush=True)
                return False

        self._write_topics()
        started = False
        attached = self.faker_running()   # 떠 있던 스택에 붙었으면 이후로도 직접 띄우지 않는다 (state_manager 재시작 틈 포함)
        if attached:
            # 실기처럼 — CLI는 떠 있는 스택에 붙기만 한다. 죽이거나 다시 띄우지 않는다 (재시작은 state_manager 몫).
            #   예전엔 데이터가 늦으면 '좀비'로 보고 죽인 뒤 옛 브리지 값(map_mode NONE)으로 다시 띄워,
            #   지도 모드 전환(state_manager 재시작) 직후 다시 붙은 CLI가 MAPPING을 NONE으로 되돌렸다 (2026-11-11 실측).
            if self._fresh():
                print('[docker] test-faker 이미 정상 — ROS 데이터 fresh', flush=True)
                return True
            print('[docker] test-faker가 떠 있다 — ROS 데이터가 올 때까지 기다린다 (스택은 건드리지 않음)', flush=True)
        elif not self.stack_busy():
            if not self._start_faker():
                return False
            started = True
        else:
            print('[docker] test-faker/build 진행 중 — 끝나기를 기다린다', flush=True)

        start = time.time()
        printed = -10
        while time.time() - start < self.timeout:
            fresh = self._fresh()
            if fresh:
                print(f'[docker] ROS 준비 완료 ({fresh})', flush=True)
                return True
            if not self.container_running():
                print('[docker] 컨테이너가 실행 중에 꺼졌다 (OOM/중지?) — docker ps 확인',
                      flush=True)
                return False
            if not started and not attached and not self.faker_running() and not self.stack_busy():
                print('[docker] 이전 작업이 끝났다 — test-faker 시작', flush=True)
                if not self._start_faker():
                    return False
                started = True
            elapsed = int(time.time() - start)
            if elapsed - printed >= 10:
                printed = elapsed
                print(f'[docker] ROS 준비 기다리는 중 … {elapsed}s', flush=True)
            time.sleep(1.0)

        print(f'[docker] ROS 준비 실패 — {self.timeout:.0f}s 안에 fresh 파일이 안 왔다', flush=True)
        tail = self._tail()
        if tail:
            print(tail, flush=True)
        print('[docker] 로그: docker exec %s tail -f /tmp/faker.log' % self.container, flush=True)
        return False
