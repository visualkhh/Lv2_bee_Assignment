#!/usr/bin/env python3
"""bag 녹화 + 부하 기록 시험 — ROS 없이: python3 tools/cli/tests/test_bagrec.py

확인: ① 명령줄(토픽 선택 / 프리셋) ② sysmon 파서 ③ 가짜 `ros2 bag record` 로 start → sys.csv 가 쌓임 → stop 이 SIGINT 로 정상 종료
(metadata 기록)·상태 정리 ④ 녹화 중 두 번째 start 는 거부 ⑤ --no-sys 면 sys.csv 없음.
"""
import importlib.util
import os
import stat
import sys
import tempfile
import time
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[2]   # tools/


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


bagrec = load('bagrec', TOOLS / 'cli' / 'op' / 'bagrec.py')
sysmon = load('sysmon', TOOLS / 'bag' / 'sysmon.py')

FAKE_ROS2 = """#!/bin/bash
# 가짜 ros2 bag record — -o 폴더를 만들고 SIGINT 를 받으면 metadata.yaml 을 쓰고 끝난다
while [ $# -gt 0 ]; do [ "$1" = "-o" ] && out=$2; shift; done
mkdir -p "$out"; trap 'echo ok > "$out/metadata.yaml"; exit 0' INT
while :; do sleep 0.2; done
"""


def test_command():
    cmd = bagrec.bag_command(Path('/x/b'), topics=['/a', '/b'])
    assert cmd[-3:] == ['--topics', '/a', '/b'] and '--regex' not in cmd, cmd
    cmd = bagrec.bag_command(Path('/x/b'))
    assert cmd[cmd.index('--regex') + 1] == bagrec.DIAG_REGEX and '--include-hidden-topics' in cmd, cmd
    assert 'global_costmap/costmap' in bagrec.DIAG_REGEX and 'cuvslam/odom' in bagrec.DIAG_REGEX


def test_sysmon_parsers():
    assert sysmon.parse_gpu('RAM 1623/7546MB CPU [3%@1510] GR3D_FREQ 25% cv0@41C') == 25
    assert sysmon.parse_gpu('아무 줄') is None
    assert sysmon.parse_meminfo('MemAvailable: 6144000 kB\nSwapTotal: 8388608 kB\nSwapFree: 8000000 kB') == (6000, 379)
    assert sysmon.cpu_pct((100, 200), (160, 300)) == 40.0
    assert sysmon.cpu_pct((5, 5), (5, 5)) == 0.0   # 시간이 안 흐름 → 0 나누기 없음


def test_lifecycle(tmp: Path):
    fake = tmp / 'bin' / 'ros2'
    fake.parent.mkdir()
    fake.write_text(FAKE_ROS2)
    fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
    os.environ['PATH'] = f'{fake.parent}:{os.environ["PATH"]}'
    bagrec.BAG_DIR = tmp / 'bags'
    bagrec.STATE = tmp / 'state.json'

    rec = bagrec.start(label='시험 run', sys_on=True)
    bag = Path(rec['bag'])
    assert bag.is_dir() and bag.name.endswith('_run') or bag.name.endswith('_시험_run'), bag
    assert bagrec.current() is not None
    try:
        bagrec.start()
        raise AssertionError('녹화 중 두 번째 start 가 거부되지 않음')
    except RuntimeError as exc:
        assert '이미 녹화 중' in str(exc), exc
    time.sleep(2.6)   # sys.csv 에 두 줄쯤
    done = bagrec.stop()
    assert done and done['stopped'], done
    assert (bag / 'metadata.yaml').read_text().strip() == 'ok', 'SIGINT 로 정상 종료되지 않음'
    rows = (bag / 'sys.csv').read_text().splitlines()
    assert rows[0].startswith('t,load1,cpu_pct,mem_avail_mb') and len(rows) >= 2, rows
    assert bagrec.current() is None and bagrec.stop() is None   # 두 번 멈춰도 안전
    assert 'MB' in bagrec.describe(done)

    rec = bagrec.start(label='nosys', sys_on=False)
    time.sleep(0.5)
    bagrec.stop()
    assert not (Path(rec['bag']) / 'sys.csv').exists(), '--no-sys 인데 sys.csv 가 생김'


if __name__ == '__main__':
    test_command()
    test_sysmon_parsers()
    with tempfile.TemporaryDirectory() as d:
        test_lifecycle(Path(d))
    print('PASS')
