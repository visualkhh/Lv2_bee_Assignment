#!/usr/bin/env python3
import contextlib
import io
import json
import pathlib
import tempfile
import unittest
from unittest import mock
import resource_watch as rw


class ResourceWatchTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.tmp.name)
        self.addCleanup(self.tmp.cleanup)
        (self.root / 'meminfo').write_text('MemTotal: 8192000 kB\nMemAvailable: 262144 kB\nSwapTotal: 4096000 kB\nSwapFree: 3072000 kB\nName: ignore\n')
        (self.root / 'stat').write_text('cpu 100 0 50 850 0 0 0 0 10 0\n')
        (self.root / 'vmstat').write_text('pswpin 5\npswpout 10\nother_text nope\n')
        self.process(12, 'robot node', 204800, 50, 100)
        self.process(13, 'another', 102400, 20, 120)
        (self.root / '14').mkdir()  # exited/unreadable process
        (self.root / 'not-a-pid').mkdir()

    def process(self, pid, name, rss, ticks, started):
        p = self.root / str(pid)
        p.mkdir(exist_ok=True)
        (p / 'comm').write_text(name + '\n')
        (p / 'status').write_text(f'Name:\t{name}\nVmRSS:\t{rss} kB\n')
        tail = ['0'] * 22
        tail[0], tail[11], tail[12], tail[19] = 'R', str(ticks), '0', str(started)
        tail[21] = str(rss * 1024 // rw.os.sysconf('SC_PAGE_SIZE'))
        (p / 'stat').write_text(f'{pid} ({name}) ' + ' '.join(tail))
        (p / 'cmdline').write_text('SECRET_DO_NOT_READ')

    def test_numeric_cpu_swap_and_safe_processes(self):
        sampler = rw.ProcSampler(self.root, top=1)
        sampler.ticks = 100
        first = sampler.sample(10)
        self.assertEqual(first['mem_available_mib'], 256)
        self.assertEqual(first['swap_used_mib'], 1000)
        self.assertIsNone(first['cpu_percent'])
        self.assertEqual(first['process_count_read'], 2)
        self.assertEqual(first['processes_by_rss'][0]['rss_mib'], 200)
        self.assertNotIn('SECRET', json.dumps(first))
        (self.root / 'stat').write_text('cpu 150 0 50 900 0 0 0 0 20 0\n')
        (self.root / 'vmstat').write_text('pswpin 7\npswpout 14\n')
        self.process(12, 'robot node', 204800, 100, 100)
        second = sampler.sample(12)
        self.assertEqual(second['cpu_percent'], 50)
        self.assertEqual(second['swap_in_pages_per_s'], 1)
        self.assertEqual(second['swap_out_pages_per_s'], 2)
        self.assertEqual(second['processes_by_rss'][0]['cpu_percent'], 25)
        self.process(12, 'reused pid', 204800, 1, 200)
        self.assertIsNone(sampler.sample(13)['processes_by_rss'][0]['cpu_percent'])

    def test_read_failure_is_reported(self):
        (self.root / 'meminfo').unlink()
        sample = rw.ProcSampler(self.root).sample(1)
        self.assertIsNone(sample['mem_available_mib'])
        self.assertIn('meminfo:FileNotFoundError', sample['read_errors'])
        self.assertEqual(rw.memory_check(sample['mem_available_mib'], 1536), ('unknown', 3))

    def test_check_exits_and_writes_one_sample(self):
        for minimum, status, code in [('1536', 'low', 2), ('256', 'ok', 0)]:
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                result = rw.main(['--check', '--min-available-mib', minimum], proc=self.root)
            self.assertEqual(result, code)
            self.assertEqual(len(out.getvalue().splitlines()), 1)
            self.assertEqual(json.loads(out.getvalue())['memory_check'], status)

    def test_rotation_is_bounded_and_json_lines_are_intact(self):
        path = self.root / 'nested' / 'resources.jsonl'
        sink = rw.RotatingJSONL(path, max_bytes=1024, backups=2)
        for index in range(20):
            sink.write({'index': index, 'payload': 'x' * 400})
        files = list(path.parent.glob('resources.jsonl*'))
        self.assertEqual(len(files), 3)
        for file in files:
            self.assertLessEqual(file.stat().st_size, 1024)
            for line in file.read_text().splitlines():
                self.assertIsInstance(json.loads(line)['index'], int)
        self.assertEqual(json.loads(path.read_text().splitlines()[-1])['index'], 19)
        with self.assertRaises(ValueError):
            sink.write({'payload': 'x' * 2000})

    def test_sample_limit_and_bad_arguments(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out), mock.patch.object(rw.time, 'sleep'):
            self.assertEqual(rw.main(['--samples', '2', '--top', '0'], proc=self.root), 0)
        self.assertEqual(len(out.getvalue().splitlines()), 2)
        for option in [['--interval', '0'], ['--interval', 'nan'], ['--interval', '0.01'],
                       ['--samples', '-1'], ['--top', '101'], ['--backups', '0']]:
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exc:
                rw.main(option, proc=self.root)
            self.assertEqual(exc.exception.code, 2)


if __name__ == '__main__':
    unittest.main()
