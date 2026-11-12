#!/usr/bin/env python3
"""시스템 부하 기록 — CPU · 메모리 · 스왑 · GPU · 메모리를 많이 쓰는 프로세스를 1초마다 sys.csv 에 (표준 라이브러리만).

  python3 sysmon.py --out <폴더> [--interval 1]      # Ctrl+C · SIGTERM 으로 끝낸다
열: t(epoch s) · load1 · cpu_pct(전체 코어 평균) · mem_avail_mb · swap_used_mb · gpu_pct(Jetson tegrastats GR3D_FREQ, 없으면 빈칸) · top(RSS 큰 프로세스 6개 "이름=MB;…")
Jetson 이면 tegrastats 원본도 같은 폴더 tegrastats.log 에 남긴다 (GPU · 온도 · 전력 — 열 값으로는 GR3D_FREQ 만 뽑음).
record.sh 는 ROS 토픽만 담는다 — 부하 분석(메모리 부족 · 루프 지연)에는 이 파일이 같이 있어야 한다. cli 의 `run-cli bag` 이 같이 띄운다.
"""
import argparse
import csv
import os
import re
import shutil
import signal
import subprocess
import threading
import time
from pathlib import Path

GPU_RE = re.compile(r'GR3D_FREQ\s+(\d+)%')


def parse_gpu(line: str):
    """tegrastats 한 줄에서 GPU 사용률 [%] (없으면 None)."""
    m = GPU_RE.search(line)
    return int(m.group(1)) if m else None


def parse_meminfo(text: str):
    """/proc/meminfo 내용 → (MemAvailable MB, 스왑 사용 MB)."""
    kb = {}
    for line in text.splitlines():
        k, _, rest = line.partition(':')
        if rest.split():
            kb[k] = int(rest.split()[0])
    return kb.get('MemAvailable', 0) // 1024, (kb.get('SwapTotal', 0) - kb.get('SwapFree', 0)) // 1024


def cpu_pct(prev, cur):
    """/proc/stat 첫 줄의 (idle, total) 두 번 → 그 사이 CPU 사용률 [%]."""
    di, dt = cur[0] - prev[0], cur[1] - prev[1]
    return 0.0 if dt <= 0 else round(100.0 * (1 - di / dt), 1)


def _read_cpu():
    v = [int(x) for x in Path('/proc/stat').read_text().splitlines()[0].split()[1:]]
    return v[3] + (v[4] if len(v) > 4 else 0), sum(v)


def _top_procs(n=6):
    page_mb = os.sysconf('SC_PAGE_SIZE') / 1048576
    rows = []
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():
            continue
        try:
            rss = int((p / 'statm').read_text().split()[1]) * page_mb
            rows.append((rss, (p / 'comm').read_text().strip()))
        except (OSError, ValueError, IndexError):
            continue
    rows.sort(reverse=True)
    return ';'.join(f'{name}={rss:.0f}' for rss, name in rows[:n])


class Tegra:
    """tegrastats 를 자식으로 띄워 최근 GPU 사용률을 들고 있는다 (Jetson 이 아니면 아무것도 안 함)."""

    def __init__(self, log: Path):
        self.gpu = None
        self.proc = None
        if shutil.which('tegrastats'):
            self.proc = subprocess.Popen(['tegrastats', '--interval', '1000'], stdout=subprocess.PIPE,
                                         stderr=subprocess.DEVNULL, text=True)
            threading.Thread(target=self._read, args=(log,), daemon=True).start()

    def _read(self, log: Path):
        with log.open('w') as f:
            for line in self.proc.stdout:
                f.write(line)
                f.flush()
                g = parse_gpu(line)
                if g is not None:
                    self.gpu = g

    def close(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out', required=True, help='sys.csv 를 쓸 폴더')
    ap.add_argument('--interval', type=float, default=1.0)
    args = ap.parse_args()
    out = Path(args.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    tegra = Tegra(out / 'tegrastats.log')
    prev = _read_cpu()
    with (out / 'sys.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'load1', 'cpu_pct', 'mem_avail_mb', 'swap_used_mb', 'gpu_pct', 'top'])
        while not stop.wait(args.interval):
            cur = _read_cpu()
            avail, swap = parse_meminfo(Path('/proc/meminfo').read_text())
            load1 = Path('/proc/loadavg').read_text().split()[0]
            w.writerow([f'{time.time():.2f}', load1, cpu_pct(prev, cur), avail, swap,
                        '' if tegra.gpu is None else tegra.gpu, _top_procs()])
            f.flush()
            prev = cur
    tegra.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
