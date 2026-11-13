#!/usr/bin/env python3
"""Read-only Linux resource sampler; emits JSONL without ROS or GPU dependencies."""
import argparse
import json
import os
from pathlib import Path
import sys
import time
from datetime import datetime, timezone

MIB = 1024 * 1024


def numeric_file(path):
    """Read /proc key/value files, preserving integer counters in source units."""
    result = {}
    for line in path.read_text().splitlines():
        fields = line.replace(':', ' ', 1).split()
        if len(fields) >= 2:
            try:
                result[fields[0]] = int(fields[1])
            except ValueError:
                pass
    return result


def memory_check(available_mib, minimum):
    if available_mib is None:
        return 'unknown', 3
    return ('ok', 0) if available_mib >= minimum else ('low', 2)


class ProcSampler:
    def __init__(self, proc=Path('/proc'), top=10):
        self.proc, self.top = Path(proc), top
        self.previous_cpu = self.previous_time = None
        self.previous_processes = {}
        self.previous_swap = None
        self.ticks = os.sysconf('SC_CLK_TCK')
        self.page_bytes = os.sysconf('SC_PAGE_SIZE')

    def sample(self, stamp=None):
        now = time.monotonic() if stamp is None else stamp
        elapsed = None if self.previous_time is None else now - self.previous_time
        record = {'time_utc': datetime.now(timezone.utc).isoformat(), 'cpu_percent': None}
        errors = []
        try:
            mem = numeric_file(self.proc / 'meminfo')
            for key, target in [('MemTotal', 'mem_total_mib'), ('MemAvailable', 'mem_available_mib'),
                                ('SwapTotal', 'swap_total_mib')]:
                record[target] = mem[key] / 1024 if key in mem else None
            record['swap_used_mib'] = ((mem['SwapTotal'] - mem['SwapFree']) / 1024
                                       if {'SwapTotal', 'SwapFree'} <= mem.keys() else None)
        except (OSError, ValueError) as exc:
            errors.append('meminfo:' + type(exc).__name__)
            record['mem_available_mib'] = None
        try:
            fields = (self.proc / 'stat').read_text().splitlines()[0].split()
            if fields[0] != 'cpu' or len(fields) < 5:
                raise ValueError('missing aggregate CPU counters')
            # guest/guest_nice are already part of user/nice, so exclude them.
            values = [int(v) for v in fields[1:9]]
            current = (sum(values), values[3] + (values[4] if len(values) > 4 else 0))
            if self.previous_cpu is not None:
                total = current[0] - self.previous_cpu[0]
                idle = current[1] - self.previous_cpu[1]
                if total > 0 and 0 <= idle <= total:
                    record['cpu_percent'] = round(100 * (total - idle) / total, 2)
            self.previous_cpu = current
        except (OSError, ValueError, IndexError) as exc:
            errors.append('stat:' + type(exc).__name__)
        try:
            vm = numeric_file(self.proc / 'vmstat')
            swap = (vm['pswpin'], vm['pswpout'])
            record['swap_in_pages_total'], record['swap_out_pages_total'] = swap
            if self.previous_swap is not None and elapsed and elapsed > 0:
                record['swap_in_pages_per_s'] = max(0, swap[0] - self.previous_swap[0]) / elapsed
                record['swap_out_pages_per_s'] = max(0, swap[1] - self.previous_swap[1]) / elapsed
            self.previous_swap = swap
        except (OSError, ValueError, KeyError) as exc:
            errors.append('vmstat:' + type(exc).__name__)
        processes, next_processes = [], {}
        try:
            entries = list(self.proc.iterdir())
        except OSError as exc:
            entries = []
            errors.append('processes:' + type(exc).__name__)
        for entry in entries:
            if not entry.name.isdecimal():
                continue
            try:
                # A single stat read provides name, CPU, RSS and PID identity. This
                # avoids three file reads per process on a busy Jetson. comm may
                # contain spaces/parentheses; fields after its last ')' start at 3.
                raw = (entry / 'stat').read_text()
                end = raw.rindex(')')
                name = raw[raw.index('(') + 1:end][:64]
                values = raw[end + 2:].split()
                ticks, started = int(values[11]) + int(values[12]), int(values[19])
                key = (int(entry.name), started)
                next_processes[key] = ticks
                cpu = None
                if key in self.previous_processes and elapsed and elapsed > 0:
                    delta = ticks - self.previous_processes[key]
                    if delta >= 0:
                        cpu = round(100 * delta / self.ticks / elapsed, 2)
                processes.append({'pid': key[0], 'name': name, 'rss_mib': max(0, int(values[21])) * self.page_bytes / MIB,
                                  'cpu_percent': cpu})
            except (OSError, ValueError, IndexError):
                # Processes may exit or become unreadable between directory and file reads.
                continue
        processes.sort(key=lambda row: row['rss_mib'], reverse=True)
        record['processes_by_rss'] = processes[:self.top]
        record['process_count_read'] = len(processes)
        if errors:
            record['read_errors'] = errors
        self.previous_processes, self.previous_time = next_processes, now
        return record


class RotatingJSONL:
    def __init__(self, path, max_bytes, backups):
        self.path, self.max_bytes, self.backups = Path(path), max_bytes, backups
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, record):
        line = json.dumps(record, separators=(',', ':'), allow_nan=False) + '\n'
        size = len(line.encode('utf-8'))
        if size > self.max_bytes:
            raise ValueError('one sample exceeds --max-bytes; increase limit or decrease --top')
        current = self.path.stat().st_size if self.path.exists() else 0
        if current + size > self.max_bytes:
            for index in range(self.backups, 0, -1):
                dst = Path(str(self.path) + '.' + str(index))
                src = self.path if index == 1 else Path(str(self.path) + '.' + str(index - 1))
                if src.exists():
                    os.replace(src, dst)
        with self.path.open('a', encoding='utf-8') as stream:
            stream.write(line)
            stream.flush()


def positive(value):
    number = float(value)
    if not 0 < number < float('inf'):
        raise argparse.ArgumentTypeError('must be a finite positive number')
    return number


def main(argv=None, proc=Path('/proc')):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interval', type=positive, default=1.0)
    parser.add_argument('--duration', type=positive, help='seconds; omitted means unlimited')
    parser.add_argument('--samples', type=int, default=0, help='0 means unlimited')
    parser.add_argument('--top', type=int, default=10, help='largest RSS processes (0..100)')
    parser.add_argument('--output', default='-', help='JSONL path or - for stdout/SSH streaming')
    parser.add_argument('--max-bytes', type=int, default=8 * MIB)
    parser.add_argument('--backups', type=int, default=2)
    parser.add_argument('--check', action='store_true', help='one sample; exit 2 if available RAM is below threshold, 3 if unknown')
    parser.add_argument('--min-available-mib', type=positive, default=1536)
    args = parser.parse_args(argv)
    if args.interval < 0.1 or args.samples < 0 or not 0 <= args.top <= 100:
        parser.error('interval must be >= 0.1, samples >= 0, and top must be 0..100')
    if args.max_bytes < 1024 or not 1 <= args.backups <= 10:
        parser.error('max-bytes must be >= 1024 and backups must be 1..10')
    sink = None if args.output == '-' else RotatingJSONL(args.output, args.max_bytes, args.backups)
    sampler, count, started = ProcSampler(proc, args.top), 0, time.monotonic()
    deadline = started
    try:
        while True:
            record = sampler.sample()
            if args.check:
                record['memory_check'], code = memory_check(record.get('mem_available_mib'), args.min_available_mib)
            if sink:
                sink.write(record)
            else:
                print(json.dumps(record, separators=(',', ':'), allow_nan=False), flush=True)
            count += 1
            if args.check:
                return code
            if args.samples and count >= args.samples:
                return 0
            if args.duration and time.monotonic() - started >= args.duration:
                return 0
            deadline += args.interval
            remaining = deadline - time.monotonic()
            if args.duration:
                remaining = min(remaining, args.duration - (time.monotonic() - started))
            if remaining > 0:
                time.sleep(remaining)
            if args.duration and time.monotonic() - started >= args.duration:
                return 0
            # Avoid catch-up bursts after a slow /proc read.
            deadline = max(deadline, time.monotonic())
    except (KeyboardInterrupt, BrokenPipeError):
        return 0
    except (OSError, ValueError) as exc:
        print('resource_watch: ' + str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
