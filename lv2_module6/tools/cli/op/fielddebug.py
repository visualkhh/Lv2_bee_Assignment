"""현장 디버그 도우미 — 미션 화면 줄(신호등 지금 · 부하 · 빌드 · Nav2 · 인지)과 로그 창 · 스냅샷.

ROS 없이 돌아가는 부분만 여기 (StateStore·mission_lines가 부름). 실기(젯슨)에서 TUI가 도니 /proc·git은 로봇 것.
스크립트로 실행하면 최근 로그 WARN/ERROR 따라 보기 (미션 메뉴 l이 새 창으로 띄움):
  python3 tools/cli/op/fielddebug.py logs
"""
from __future__ import annotations

import os
import re
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path

# ---------------------------------------------------------------- 신호등 지금
# signal_decoder (perception/src/signal_decoder.cpp)의 점멸 세기를 TUI에서 가볍게 따라 함 — 지금 몇 번째인지 보려고.
#   같은 규칙: 켜진 구간(짧은 꺼짐 min_off_s 포함) = 점멸 1번 · 색은 구간 다수결(min_share 미만·all이면 ?) ·
#   min_on_s보다 짧은 켜짐은 잡음 · 꺼짐이 gap_s 이어지면 사이클 끝. 끊긴 조각 이어 붙이기(tail)·확정 검사는 안 함 —
#   판정은 decoder(/mission/signal_status)가 하고 이건 '지금 어디까지'만 보여 줌.
LETTER = {'red': 'R', 'yellow': 'Y', 'green': 'G'}
LIGHT_MAX_AGE_S = 0.6   # signal_decoder on_objects와 같음 — 이보다 오래 못 본 신호등은 안 씀


def light_state(lights, now_s: float, max_age_s: float = LIGHT_MAX_AGE_S) -> str:
    """signal_decoder light_state와 같음 — (마지막 본 시각, 색) 중 켜진 것 우선, 없으면 가장 최근 것, 없으면 ''."""
    lit = any_ = None
    for t, c in lights:
        if now_s - t >= max_age_s:
            continue
        if any_ is None or t > any_[0]:
            any_ = (t, c)
        if (c in LETTER or c == 'all') and (lit is None or t > lit[0]):
            lit = (t, c)
    return lit[1] if lit else any_[1] if any_ else ''


class SignalTracker:
    def __init__(self, min_on_s=0.3, min_off_s=0.3, min_share=0.5, gap_s=2.5, blinks=12):
        self.min_on_s, self.min_off_s, self.min_share, self.gap_s, self.blinks = min_on_s, min_off_s, min_share, gap_s, blinks
        self.state = ''            # 지금 신호등 (red | yellow | green | off | all | '' 안 보임)
        self.seen = False          # 신호등을 한 번이라도 봄 (아니면 화면 줄 안 냄)
        self.in_on = False
        self.on_start = self.last_on = 0.0
        self.votes: Counter = Counter()
        self.off_since: float | None = None
        self.off_saved: float | None = None
        self.marked = False
        self.seg: list[str] = []   # 지금 사이클 점멸 (R | Y | G | ?)
        self.last_cycle: list[str] = []
        self.t = 0.0

    @classmethod
    def from_config(cls):
        """perception.yaml signal_decoder 값 — 못 읽으면 기본값 (cpp 기본과 같음)."""
        try:
            from .configref import node_defaults
            p = node_defaults('signal_decoder')[1]
            keys = ('min_on_s', 'min_off_s', 'min_share', 'gap_s', 'blinks')
            return cls(**{k: p[k] for k in keys if isinstance(p.get(k), (int, float))})
        except Exception:
            return cls()

    def _color(self) -> str:
        total = sum(self.votes.values())
        if not total:
            return '?'
        best, n = self.votes.most_common(1)[0]
        return LETTER.get(best, '?') if n >= self.min_share * total else '?'

    def feed(self, t: float, state: str):
        self.t, self.state = t, state
        if self.off_since is None:
            self.off_since = t
        if state not in ('off', ''):
            self.seen = True
            if not self.in_on:
                self.in_on, self.on_start, self.off_saved = True, t, self.off_since
                self.votes = Counter()
            self.votes[state] += 1
            self.last_on = t
            return
        if self.in_on:
            if t - self.last_on < self.min_off_s - 1e-6:
                return
            self.in_on = False
            if self.last_on - self.on_start >= self.min_on_s - 1e-9:
                self.seg.append(self._color())
                self.off_since, self.marked = self.last_on, False
            else:
                self.off_since = self.off_saved   # 잡음 — 그 전 꺼짐을 이어 잼
        if not self.marked and t - self.off_since >= self.gap_s - 1e-6:
            self.marked = True
            if self.seg:
                self.last_cycle, self.seg = self.seg, []

    def summary(self) -> str:
        """'R 켜짐 · 5/12 (R2 Y1 G2) · 꺼짐 0.8/2.5s' — 화면 한 줄."""
        now = LETTER.get(self.state, '켜짐(all)' if self.state == 'all' else '꺼짐' if self.state == 'off' else '안 보임')
        if self.state in LETTER:
            now += ' 켜짐'
        c = Counter(self.seg)
        unk = f' ?{c["?"]}' if c['?'] else ''
        cur = f'{len(self.seg)}/{self.blinks} (R{c["R"]} Y{c["Y"]} G{c["G"]}{unk})'
        if self.in_on:
            cur += f' +{self._color()} 켜짐 중'
        gap = '' if self.in_on or self.off_since is None else f' · 꺼짐 {min(self.t - self.off_since, 99):.1f}/{self.gap_s:.1f}s'
        return f'{now} · {cur}{gap}'


def parse_answer(text: str) -> list[str]:
    """'R G R R …' · 'RGRR…' · 'r,g,…' → ['R', 'G', …] (R·Y·G 외 글자는 버림)."""
    return [ch for ch in (text or '').upper() if ch in 'RYG']


def answer_match(answer: list[str], got: list[str]) -> str:
    """정답과 읽은 사이클 비교 — 개수(순서 무관)와 순서(자리마다) 둘 다."""
    a, g = Counter(answer), Counter(got)
    counts = '개수 맞음' if a == g else f'개수 다름 (읽음 R{g["R"]} Y{g["Y"]} G{g["G"]} · 정답 R{a["R"]} Y{a["Y"]} G{a["G"]})'
    same = sum(x == y for x, y in zip(answer, got))
    wrong = [str(i + 1) for i, (x, y) in enumerate(zip(answer, got)) if x != y]
    order = f'순서 {same}/{len(answer)}' + (f' (틀린 자리 {",".join(wrong)})' if wrong else '') + \
        (f' · 길이 {len(got)}' if len(got) != len(answer) else '')
    return f'{counts} · {order}'


# ---------------------------------------------------------------- 인지 진단
PERCEPTION_DIAG = 'perception: 상태'   # perception_node.cpp on_diag — /diagnostics_agg에선 '/…/perception: 상태'


def perception_diag(diag_values: dict) -> dict[str, str]:
    """perception 진단 키/값 (없으면 {})."""
    for name, kvs in diag_values.items():
        if name.endswith(PERCEPTION_DIAG):
            return dict(kvs)
    return {}


_pparams: dict = {}


def perception_param(name: str, default):
    """perception.yaml perception 파라미터 (한 번 읽고 기억) — 없으면 default (perception_node.cpp 기본과 같게)."""
    if not _pparams:
        try:
            from .configref import node_defaults
            _pparams.update(node_defaults('perception')[1] or {'-': None})
        except Exception:
            _pparams['-'] = None
    v = _pparams.get(name)
    return v if isinstance(v, (int, float)) else default


# ---------------------------------------------------------------- 부하
LOAD_RED = 1.5   # load1 / 코어 수가 이보다 크면 빨강
_procs = {'t': 0.0, 'prev': {}, 'top': []}


def load_ratio() -> tuple[float, int] | None:
    """(1분 load, 코어 수) — 없으면 None."""
    try:
        return os.getloadavg()[0], os.cpu_count() or 1
    except OSError:
        return None


def top_procs(n: int = 3, min_interval_s: float = 3.0) -> list[tuple[float, str]]:
    """CPU 많이 쓰는 프로세스 [(%, 이름)] — /proc/<pid>/stat 지난 호출과의 차이 (3초보다 자주 부르면 지난 값).
    첫 호출은 비교할 값이 없어 [] · /proc 없는 곳(Mac)도 []."""
    now = time.monotonic()
    if now - _procs['t'] < min_interval_s:
        return _procs['top']
    ticks = {}
    try:
        pids = [d for d in os.listdir('/proc') if d.isdigit()]
    except OSError:
        return []
    for pid in pids:
        try:
            with open(f'/proc/{pid}/stat') as f:
                s = f.read()
        except OSError:
            continue
        fields = s[s.rindex(')') + 2:].split()
        ticks[pid] = (s[s.index('(') + 1:s.rindex(')')], int(fields[11]) + int(fields[12]))   # utime + stime
    dt, prev = now - _procs['t'], _procs['prev']
    hz = os.sysconf('SC_CLK_TCK')
    use = sorted(((t - prev[p][1]) / hz / dt * 100, p, name) for p, (name, t) in ticks.items()
                 if p in prev and prev[p][0] == name and (t - prev[p][1]) / hz / dt >= 0.01) if prev else []
    _procs.update(t=now, prev=ticks, top=[(pct, _proc_name(pid, name)) for pct, pid, name in reversed(use[-n:])])
    return _procs['top']


def _proc_name(pid: str, comm: str) -> str:
    """python3이면 스크립트 이름 (ROS 파이썬 노드는 argv[1]이 lib/<pkg>/<노드>)."""
    if not comm.startswith('python'):
        return comm
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            args = [a for a in f.read().split(b'\0') if a and not a.startswith(b'-')]
        return Path(args[1].decode(errors='replace')).name if len(args) > 1 else comm
    except OSError:
        return comm


# ---------------------------------------------------------------- 빌드 정보
_build = {'t': -1e9, 'v': None}


def _git(repo: Path, *args) -> str:
    return subprocess.run(['git', '-C', str(repo), *args], capture_output=True, text=True, timeout=3).stdout.strip()


def build_info(src: Path | None = None, refresh_s: float = 30.0) -> dict | None:
    """{branch, commit, dirty, build_type} — ws repo의 git + build/perception/CMakeCache.txt. 30초 캐시."""
    if time.monotonic() - _build['t'] < refresh_s:
        return _build['v']
    if src is None:
        from .configref import src_dir
        src = src_dir()
    info = None
    if src is not None:
        info = {'branch': '', 'commit': '', 'dirty': None, 'build_type': None}
        try:
            info['branch'] = _git(src, 'rev-parse', '--abbrev-ref', 'HEAD')
            info['commit'] = _git(src, 'rev-parse', '--short', 'HEAD')
            info['dirty'] = len(_git(src, 'status', '--porcelain').splitlines())
        except (OSError, subprocess.SubprocessError):
            pass
        cache = src.parent / 'build' / 'perception' / 'CMakeCache.txt'
        try:
            m = re.search(r'^CMAKE_BUILD_TYPE:\w+=(.*)$', cache.read_text(errors='replace'), re.M)
            info['build_type'] = m.group(1).strip() if m else ''
        except OSError:
            pass   # 빌드 안 함 (None = 모름)
    _build.update(t=time.monotonic(), v=info)
    return info


# ---------------------------------------------------------------- Nav2 수명주기
NAV2_NODES = (('planner_server', 'planner'), ('controller_server', 'controller'), ('bt_navigator', 'bt'),
              ('behavior_server', 'behavior'), ('velocity_smoother', 'smoother'), ('collision_monitor', 'collision'))
NAV2_POLL_S = 5.0


# ---------------------------------------------------------------- 스냅샷 영상
def save_jpeg(msg, path: Path) -> bool:
    """sensor_msgs/Image (rgb8 · bgr8 · mono8) → JPEG. cv_bridge 없이 numpy·cv2 (젯슨엔 cv_bridge 파이썬이 없음). 못 하면 False."""
    try:
        import cv2
        import numpy as np
    except ImportError:
        return False
    ch = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(msg.encoding)
    if ch is None:
        return False
    img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :msg.width * ch]
    img = img.reshape(msg.height, msg.width, ch)
    if msg.encoding == 'rgb8':
        img = img[:, :, ::-1]
    return bool(cv2.imwrite(str(path), img))


# ---------------------------------------------------------------- 로그
LOG_DIR = Path(os.environ.get('ROS_LOG_DIR') or Path.home() / '.ros' / 'log')
LOG_NODES = ('mission_manager', 'coordinator', 'perception', 'signal_decoder')   # perception = perception_node_* 포함
BAD = re.compile(r'\[(WARN|ERROR|FATAL)\]')


def newest_logs(log_dir: Path = LOG_DIR) -> list[Path]:
    """노드마다 가장 최근 ~/.ros/log/<노드>_<pid>_<시각>.log (rclcpp 파일 로그)."""
    out = []
    for node in LOG_NODES:
        files = [p for p in log_dir.glob(f'{node}*_*.log') if re.fullmatch(rf'{node}\w*_\d+_\d+\.log', p.name)]
        if files:
            out.append(max(files, key=lambda p: p.stat().st_mtime))
    return out


def tail_lines(path: Path, n: int, bad_only: bool = False) -> list[str]:
    try:
        lines = path.read_text(errors='replace').splitlines()
    except OSError:
        return []
    return [ln for ln in lines if BAD.search(ln)][-n:] if bad_only else lines[-n:]


def recent_bad(n: int = 40, log_dir: Path = LOG_DIR) -> list[str]:
    """노드별 최근 WARN/ERROR를 시각 순으로 합쳐 마지막 n줄 ('[WARN] [1794…] [노드]: …' 의 시각으로 정렬)."""
    rows = [ln for p in newest_logs(log_dir) for ln in tail_lines(p, n, bad_only=True)]
    return sorted(rows, key=_stamp)[-n:]


def _stamp(line: str) -> float:
    m = re.search(r'\] \[(\d+\.\d+)\]', line)
    return float(m.group(1)) if m else 0.0


def follow_logs(n: int = 40, period_s: float = 1.0):
    """recent_bad를 찍고, 파일에 새로 붙는 WARN/ERROR를 계속 (tail -F처럼 — 노드가 다시 뜨면 새 파일로 바꿈). Ctrl+C로 끝."""
    print(f'최근 WARN/ERROR — {LOG_DIR}/{{{",".join(LOG_NODES)}}}_*.log (Ctrl+C 끝)')
    for ln in recent_bad(n):
        print(_paint(ln))
    pos = {p: p.stat().st_size for p in newest_logs()}
    try:
        while True:
            time.sleep(period_s)
            for p in newest_logs():
                start = pos.get(p, 0)
                try:
                    size = p.stat().st_size
                    if size <= start:
                        continue
                    with open(p, errors='replace') as f:
                        f.seek(start)
                        chunk = f.read()
                except OSError:
                    continue
                pos[p] = size
                for ln in chunk.splitlines():
                    if BAD.search(ln):
                        print(_paint(ln), flush=True)
    except KeyboardInterrupt:
        pass


def _paint(line: str) -> str:
    if not sys.stdout.isatty():
        return line
    return ('\033[31m' if '[ERROR]' in line or '[FATAL]' in line else '\033[33m') + line + '\033[0m'


if __name__ == '__main__':
    if sys.argv[1:2] == ['logs']:
        follow_logs()
    else:
        print('사용: fielddebug.py logs')
