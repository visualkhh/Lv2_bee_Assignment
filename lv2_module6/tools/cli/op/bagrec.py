"""범용 녹화 — ros2 bag record(MCAP) + 시스템 부하(tools/bag/sysmon.py) 를 한 번에 시작·중지·확인.

`run-cli bag start|stop|status` 와 메뉴 8) 토픽 · bag 이 같이 쓴다 (상태를 파일에 남겨서 다른 프로세스에서도 멈출 수 있다).
폴더 하나 = <bag 폴더>/ (bag 본체 + metadata.yaml) + sys.csv(CPU·메모리·스왑·GPU·큰 프로세스) + tegrastats.log + bag.log.
"""
from __future__ import annotations

import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

BAG_DIR = Path.home() / '.cache' / 'beebot' / 'bags'
STATE = Path.home() / '.cache' / 'beebot' / 'bagrec.json'
SYSMON = Path(__file__).resolve().parents[2] / 'bag' / 'sysmon.py'   # tools/bag/sysmon.py

# 진단 프리셋 — 명령 체인 · odom · TF · 지도 · 상태 (영상·점구름 원본은 뺌, bee_diag/rec_diag.sh 와 같은 목록)
DIAG_REGEX = ('^/(tf|tf_static|joint_states|imu|odom|odometry/filtered|cuvslam/odom|cuvslam/path|cmd_vel.*|cmd/.*|state/.*|'
              'collision_monitor_state|polygon_.*|diagnostics_agg|rosout|plan|local_plan|map|tracked_objects|'
              'global_costmap/costmap|local_costmap/costmap)$')


def bag_command(bag: Path, topics: list[str] | None = None, regex: str | None = None) -> list[str]:
    """ros2 bag record 명령줄. topics 가 있으면 그 토픽만, 아니면 regex(기본 DIAG_REGEX)."""
    cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '--disable-keyboard-controls', '-o', str(bag)]
    if topics:
        return cmd + ['--topics', *topics]
    return cmd + ['--regex', regex or DIAG_REGEX, '--include-hidden-topics']


def _alive(pid, word: str = '') -> bool:
    try:
        pid = int(pid)
        os.kill(pid, 0)
    except (TypeError, ValueError, ProcessLookupError, PermissionError):
        return False
    try:
        cmd = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
    except OSError:
        return True
    return word in cmd


def _load() -> dict:
    try:
        return json.loads(STATE.read_text())
    except (OSError, ValueError):
        return {}


def _save(rec: dict) -> None:
    STATE.parent.mkdir(parents=True, exist_ok=True)
    tmp = STATE.with_suffix('.tmp')
    tmp.write_text(json.dumps(rec, ensure_ascii=False, indent=1))
    tmp.replace(STATE)


def current() -> dict | None:
    """지금 녹화 중인 항목 (프로세스가 죽었으면 None)."""
    rec = _load()
    return rec if rec.get('pid') and not rec.get('stopped') and _alive(rec['pid'], 'record') else None


def start(label: str | None = None, topics: list[str] | None = None, regex: str | None = None,
          sys_on: bool = True) -> dict:
    cur = current()
    if cur:
        raise RuntimeError(f'이미 녹화 중: {cur["bag"]}')
    label = re.sub(r'[^\w.-]+', '_', (label or 'run').strip()) or 'run'
    bag = BAG_DIR / f'{time.strftime("%Y%m%d_%H%M%S")}_{label}'
    BAG_DIR.mkdir(parents=True, exist_ok=True)
    log = BAG_DIR / f'{bag.name}.log'
    cmd = bag_command(bag, topics, regex)
    with log.open('wb') as f:
        f.write((' '.join(cmd) + '\n').encode())
        f.flush()
        proc = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                start_new_session=True)
    deadline = time.monotonic() + 8.0   # discovery 가 느리면 폴더가 늦게 생긴다
    while time.monotonic() < deadline and proc.poll() is None and not bag.is_dir():
        time.sleep(0.1)
    if proc.poll() is not None or not bag.is_dir():
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
        tail = '\n'.join(log.read_text(errors='replace').splitlines()[-6:])
        raise RuntimeError(f'ros2 bag record 시작 실패 (로그 {log})\n{tail}')
    mon = None
    if sys_on:
        mon = subprocess.Popen([sys.executable, str(SYSMON), '--out', str(bag)], stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL, start_new_session=True)
    rec = {'pid': proc.pid, 'mon_pid': mon.pid if mon else None, 'bag': str(bag), 'label': label,
           'started': time.time(), 'log': str(log), 'sys': bool(sys_on)}
    _save(rec)
    return rec


def stop() -> dict | None:
    """녹화 중지 — bag 은 SIGINT 로 정상 종료(metadata 기록)시키고, 부하 기록기는 SIGTERM."""
    rec = _load()
    if not rec.get('pid') or rec.get('stopped'):
        return None
    pid = int(rec['pid'])
    if _alive(pid, 'record'):
        try:
            os.killpg(pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        end = time.monotonic() + 10.0
        while time.monotonic() < end and _alive(pid, 'record'):
            time.sleep(0.2)
        if _alive(pid, 'record'):
            os.killpg(pid, signal.SIGKILL)
    mon = rec.get('mon_pid')
    if mon and _alive(mon, 'sysmon'):
        try:
            os.killpg(int(mon), signal.SIGTERM)
        except ProcessLookupError:
            pass
    rec['stopped'] = time.time()
    _save(rec)
    return rec


def size_mb(path: str | Path) -> float:
    return sum(f.stat().st_size for f in Path(path).rglob('*') if f.is_file()) / 1048576


def describe(rec: dict) -> str:
    """한 줄 설명 — 라벨 · 경과 · 크기 · 부하 기록 여부."""
    end = rec.get('stopped') or time.time()
    return (f'{rec.get("label", "-")} · {end - rec.get("started", end):.0f}s · {size_mb(rec["bag"]):.1f} MB'
            f' · 부하 {"O" if rec.get("sys") else "X"} · {rec["bag"]}')
