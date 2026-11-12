#!/usr/bin/env python3
"""FakerStack 시험 — docker 없이: python3 tests/test_faker_stack.py

실기처럼: CLI(--source docker)는 떠 있는 스택에 붙기만 한다 — 데이터가 늦어도 죽이거나 다시 띄우지 않는다.
(2026-11-11: 지도 모드 전환 직후 다시 붙은 CLI가 스택을 죽이고 map:=none으로 다시 띄워 MAPPING이 풀렸음)
"""
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ros.docker.FakerStack as fs  # noqa: E402

fs.time.sleep = lambda s: None   # 기다림 없이


class Fake(fs.FakerStack):
    """docker를 안 부르는 FakerStack — 상태를 시나리오대로 바꾼다."""

    def __init__(self, running, fresh_after, busy=lambda n: False):
        super().__init__(tempfile.mkdtemp(), timeout=30.0)
        self.running, self.fresh_after, self.busy = running, fresh_after, busy
        self.checks = 0
        self.docker_calls = []
        self.started = 0

    def container_running(self):
        return True

    def faker_running(self):
        return self.running(self.checks)

    def stack_busy(self):
        return self.busy(self.checks)

    def _fresh(self):
        self.checks += 1
        return 'state/devices' if self.fresh_after is not None and self.checks > self.fresh_after else None

    def _docker(self, *args):
        self.docker_calls.append(args)
        raise AssertionError(f'떠 있는 스택에 docker 명령을 보내면 안 됨: {args}')

    def _write_topics(self):
        pass

    def _start_faker(self):
        self.started += 1
        return True


# ① 떠 있는데 데이터가 늦음 (state_manager가 막 다시 띄움) → 기다렸다가 붙음, 안 죽이고 안 띄움
s = Fake(running=lambda n: True, fresh_after=5)
assert s.ensure() and s.started == 0 and not s.docker_calls

# ② 떠 있던 스택이 재시작 틈에 잠깐 사라져도(launch 꺼짐 · 아직 빌드 전) 직접 띄우지 않는다
s = Fake(running=lambda n: n < 2 or n > 4, fresh_after=8)
assert s.ensure() and s.started == 0

# ③ 아무것도 안 떠 있으면 그때만 띄운다 (실기의 bringup 시작)
s = Fake(running=lambda n: False, fresh_after=3)
assert s.ensure() and s.started == 1

# ④ 빌드 중이면 기다렸다가, 끝나도 안 떠 있으면 그때 띄운다
s = Fake(running=lambda n: False, fresh_after=6, busy=lambda n: n < 3)
assert s.ensure() and s.started == 1
print('PASS test_faker_stack')
