#!/usr/bin/env python3
"""TeleopDialog 명령 전송 스레드 시험 — ROS 없이: python3 tools/cli/tests/test_teleop_sender.py

예전 문제(2026-11-10 실기): 그리기가 무거워 on_tick이 0.6초씩 걸리면 명령이 1.6 Hz로 나가 manual_control 만료(0.5초) 정지가 끼었다.
확인: ① 메인 루프가 0.6초씩 막혀도 명령이 SEND_HZ 가까이 나감 ② 루프가 HEARTBEAT_S 넘게 멈추면 정지(0) ③ 끝나면 스레드가 멈추고 정지 명령.
"""
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # tools/cli
from ros.fake.FakeRos import FakeRos  # noqa: E402

FakeRos.install()
from op.dialog.teleop_dialog import TeleopDialog  # noqa: E402


class FakeBus:
    def __init__(self):
        self.sent = []   # (시각, v, w)
        self.prompt = NS()
        self.term = NS()
        self.state = NS()

    def _send_velocity(self, v, w):
        self.sent.append((time.monotonic(), v, w))

    def _poll(self, s):
        time.sleep(s)


bus = FakeBus()
d = TeleopDialog(bus, 0.15, 0.8)
d.move = (1, 0)                       # 전진 키가 눌려 있음 (래치)
d._stop.clear()
d._beat = time.monotonic()
th = threading.Thread(target=d._sender, daemon=True)
th.start()

# ① 메인 루프가 느려도(0.6초 주기로만 도는 것을 흉내) 명령은 20 Hz 가까이
t0 = time.monotonic()
while time.monotonic() - t0 < 2.4:
    time.sleep(0.6)                   # 무거운 그리기
    d._beat = time.monotonic()        # on_tick이 한 번 돌았다
fwd = [t for t, v, w in bus.sent if v > 0]
gaps = [b - a for a, b in zip(fwd, fwd[1:])]
rate = len(fwd) / 2.4
assert rate > 15, f'명령이 너무 드묾: {rate:.1f} Hz'
assert max(gaps) < 0.25, f'명령 간격이 벌어짐: 최대 {max(gaps) * 1000:.0f} ms'   # manual_control 만료 0.5초보다 한참 짧아야

# ② 메인 루프가 HEARTBEAT_S 넘게 멈추면 정지
bus.sent.clear()
time.sleep(d.HEARTBEAT_S + 0.5)       # 루프가 멈춤 (beat 갱신 없음)
tail = [(v, w) for t, v, w in bus.sent[-5:]]
assert tail and all(v == 0.0 and w == 0.0 for v, w in tail), f'멈춘 UI가 계속 명령을 냄: {tail}'

# ③ 루프가 다시 돌면 재개, 끝내면 스레드 종료
d._beat = time.monotonic()
time.sleep(0.2)
assert any(v > 0 for t, v, w in bus.sent[-4:]), '루프가 돌아왔는데 재개 안 함'
d._stop.set()
th.join(timeout=1.0)
assert not th.is_alive(), '스레드가 안 끝남'

# 그리기 주기: draw_period 안이면 건너뜀 · invalidate()하면 바로
assert TeleopDialog.draw_period >= 0.1 and TeleopDialog.SEND_HZ >= 10
print(f'PASS test_teleop_sender (명령 {rate:.0f} Hz · 최대 간격 {max(gaps) * 1000:.0f} ms · 멈춘 UI → 정지)')
