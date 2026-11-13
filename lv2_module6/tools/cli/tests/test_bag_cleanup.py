#!/usr/bin/env python3
"""녹화 뒷정리 시험 — ROS 없이: python3 tools/cli/tests/test_bag_cleanup.py

확인: ① SIGINT를 무시하는 녹화기도 stop()이 SIGTERM으로 끝냄 ② stop_mine()은 이 프로세스가 켠 녹화만 끔
③ max_s가 지나면 timeout이 SIGINT로 끝냄 (coreutils timeout 있을 때 — 로봇·Linux)
"""
import importlib.util
import json
import os
import shutil
import signal
import stat
import sys
import tempfile
import time
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[2]   # tools/
spec = importlib.util.spec_from_file_location('bagrec', TOOLS / 'cli' / 'op' / 'bagrec.py')
bagrec = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bagrec)

# 가짜 ros2 bag record — -o 폴더를 만들고 INT(또는 STUBBORN이면 TERM)에 metadata.yaml에 받은 신호를 쓰고 끝
FAKE_ROS2 = """#!/bin/bash
while [ $# -gt 0 ]; do [ "$1" = "-o" ] && out=$2; shift; done
mkdir -p "$out"
if [ -n "$STUBBORN" ]; then trap '' INT; else trap 'echo INT > "$out/metadata.yaml"; exit 0' INT; fi
trap 'echo TERM > "$out/metadata.yaml"; exit 0' TERM
while :; do sleep 0.1; done
"""


def wait_dead(pid, s=5.0):
    end = time.monotonic() + s
    while time.monotonic() < end and bagrec._alive(pid, 'record'):
        time.sleep(0.05)
    return not bagrec._alive(pid, 'record')


def main(tmp: Path):
    fake = tmp / 'bin' / 'ros2'
    fake.parent.mkdir()
    fake.write_text(FAKE_ROS2)
    fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
    os.environ['PATH'] = f'{fake.parent}:{os.environ["PATH"]}'
    bagrec.BAG_DIR, bagrec.STATE = tmp / 'bags', tmp / 'state.json'
    bagrec.STOP_WAITS = ((signal.SIGINT, 0.5), (signal.SIGTERM, 2.0))

    # ① SIGINT 무시 → SIGTERM
    os.environ['STUBBORN'] = '1'
    rec = bagrec.start('stubborn', sys_on=False, max_s=0)
    os.environ.pop('STUBBORN')
    assert bagrec.stop()['stopped']
    assert wait_dead(rec['pid']) and (Path(rec['bag']) / 'metadata.yaml').read_text().strip() == 'TERM'

    # ② 다른 프로세스가 켠 녹화는 stop_mine이 안 건드림 · 내 것은 끔 (SIGINT로 정상 종료)
    rec = bagrec.start('mine', sys_on=False, max_s=0)
    assert rec['owner'] == os.getpid()
    st = json.loads(bagrec.STATE.read_text())
    bagrec.STATE.write_text(json.dumps({**st, 'owner': 1}))
    assert bagrec.stop_mine() is None and bagrec.current() is not None
    bagrec.STATE.write_text(json.dumps(st))
    assert bagrec.stop_mine() and wait_dead(rec['pid'])
    assert (Path(rec['bag']) / 'metadata.yaml').read_text().strip() == 'INT'
    assert bagrec.stop_mine() is None   # 두 번 불러도 안전

    # ③ 최대 길이
    if shutil.which('timeout'):
        rec = bagrec.start('max', sys_on=False, max_s=1)
        assert wait_dead(rec['pid'], 6.0), 'max_s가 지났는데 녹화가 안 끝남'
        assert (Path(rec['bag']) / 'metadata.yaml').read_text().strip() == 'INT'
        assert bagrec.current() is None
    else:
        print('skip ③ (timeout 없음 — macOS면 brew coreutils)')


if __name__ == '__main__':
    with tempfile.TemporaryDirectory() as d:
        main(Path(d))
    print('PASS')
