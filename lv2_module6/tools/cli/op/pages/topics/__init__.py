"""토픽 · bag — /state/topics 목록에서 토픽을 골라 rosbag2(MCAP)로 녹화/재생.

rosbag2는 노드 단위가 아니라 **토픽 단위**다. 노드별로 묶고 싶으면 그 노드가 발행하는 토픽을
골라서 녹화하면 된다. 여기서는 /state/topics의 `이름|타입|publisher 수`를 그대로 보여 준다.

  s = 선택 토픽 녹화 시작/중지      d = 진단 프리셋 녹화 시작/중지 (명령 체인 · odom · TF · 지도 · 코스트맵)
  u = 시스템 부하(CPU · 메모리 · GPU) 같이 기록 켜기/끄기 — bag 폴더에 sys.csv · tegrastats.log
  p = 최근 bag 재생/중지
  x = 선택 해제
  숫자/↑↓ = 토픽 선택 토글
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import time
from pathlib import Path

from ... import bagrec
from ...defs import STAY
from ...terminal import DIM
from ...menus import ActionItem, Menu


class TopicBagMenu(Menu):
    title = '토픽 · bag'
    REFRESH_S = 2.0

    def __init__(self, app):
        super().__init__(app)
        self.selected: set[str] = set()
        self._topics: list[tuple[str, str]] = []
        self._at = 0.0
        self.sys_on = True   # 녹화할 때 부하(sys.csv)도 같이
        self._play = None
        self._docker_topics: set[str] = set()
        self._bag_dir = Path.home() / '.cache' / 'beebot' / 'bags'

    def _refresh(self) -> None:
        now = time.monotonic()
        if self._topics and now - self._at < self.REFRESH_S:
            return
        rows = []
        for line in (self.bus.state.values.get('topics') or '').splitlines():
            parts = line.split('|')
            if parts and parts[0].startswith('/'):
                rows.append((parts[0], parts[1] if len(parts) > 1 else ''))
        self._topics = rows
        self._at = now

    def _toggle(self, name: str) -> None:
        if name in self.selected:
            self.selected.discard(name)
        else:
            self.selected.add(name)

    def _toggle_sys(self) -> None:
        self.sys_on = not self.sys_on

    def _recording(self) -> bool:
        return bool(self._docker_topics) or bagrec.current() is not None

    def _playing(self) -> bool:
        return self._play is not None and self._play.poll() is None

    def _record_label(self) -> str:
        return '녹화 중지' if self._recording() else f'선택 토픽 녹화 시작 ({len(self.selected)}개)'

    def _diag_label(self) -> str:
        return '녹화 중지' if self._recording() else '진단 녹화 시작 (프리셋 토픽)'

    def _sys_label(self) -> str:
        return f'부하 기록 (CPU · 메모리 · GPU): {"켬" if self.sys_on else "끔"}'

    def _play_label(self) -> str:
        return 'bag 재생 중지' if self._playing() else '최근 bag 재생'

    def _docker_write(self, topic: str, action: str) -> bool:
        debug = getattr(self.bus.node, 'debug_dir', None)
        if debug is None:
            print('docker debug 경로를 모름')
            return False
        path = Path(debug) / 'input' / 'topic' / topic.lstrip('/') / 'bag'
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {'request_id': f'bag-{os.getpid()}-{int(time.time() * 1000)}', 'action': action}
        tmp = path.with_name('.bag.tmp')
        tmp.write_text(json.dumps(payload, ensure_ascii=False))
        os.replace(tmp, path)
        return True

    def toggle_record(self) -> None:
        if self._recording():
            if self._docker_topics:
                for topic in sorted(self._docker_topics):
                    self._docker_write(topic, 'stop')
                print(f'docker bag 중지: {len(self._docker_topics)} topics')
                self._docker_topics.clear()
            rec = bagrec.stop()
            if rec is not None:
                print(f'녹화 중지: {bagrec.describe(rec)}')
            return
        topics = sorted(self.selected)
        if not topics:
            print('토픽을 하나 이상 골라라 (숫자/↑↓ Enter) — 토픽 없이 하려면 d (진단 프리셋)')
            return
        if os.environ.get('CLI_SOURCE') == 'docker':
            # 토픽별 파일 브리지 — 각 토픽 폴더에 같은 명령을 쓴다.
            for topic in topics:
                if not self._docker_write(topic, 'start'):
                    return
                self._docker_topics.add(topic)
            print(f'docker bag 녹화: {len(topics)} topics')
            print('저장: debug/output/topic/<토픽>/bags/<시각>/')
            return
        self._start(topics=topics, label=f'{len(topics)}topics')

    def toggle_diag(self) -> None:
        """진단 프리셋 녹화 시작/중지 — 토픽을 안 골라도 된다."""
        if self._recording():
            self.toggle_record()
        elif os.environ.get('CLI_SOURCE') == 'docker':
            print('docker 소스에서는 진단 녹화를 쓸 수 없음 — 실기(real)에서 실행')
        else:
            self._start(topics=None, label='diag')

    def _start(self, topics, label: str) -> None:
        try:
            rec = bagrec.start(label=label, topics=topics, sys_on=self.sys_on)
        except RuntimeError as exc:
            print(f'녹화 시작 실패: {exc}')
            return
        print(f'녹화 시작: {rec["bag"]}')
        print(f'  {"선택 " + str(len(topics)) + "개" if topics else "진단 프리셋"} · 부하 기록 {"켬" if rec["sys"] else "끔"}')

    def toggle_play(self) -> None:
        if self._playing():
            try:
                os.killpg(self._play.pid, signal.SIGINT)
            except Exception:  # noqa: BLE001
                self._play.send_signal(signal.SIGINT)
            print('bag 재생 중지')
            self._play = None
            return
        if os.environ.get('CLI_SOURCE') == 'docker':
            print('docker 소스에서는 호스트 ros2 bag을 쓸 수 없음 — 실기(real)에서 실행')
            return
        bags = sorted((p for p in self._bag_dir.glob('*') if p.is_dir()), key=lambda p: p.stat().st_mtime)
        if not bags:
            print(f'bag 없음: {self._bag_dir}')
            return
        bag = bags[-1]
        if not self.ui.confirm(f'최근 bag을 재생할까요?', default=False,
                               body=[str(bag), '재생 중 로봇이 움직일 수 있음 — 비상정지 확인']):
            return
        self._play = subprocess.Popen(
            ['ros2', 'bag', 'play', '-s', 'mcap', str(bag)],
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL, start_new_session=True)
        print(f'bag 재생: {bag}')

    def items(self):
        self._refresh()
        items = [
            ActionItem(self.app, 's', self._record_label, self.toggle_record),
            ActionItem(self.app, 'd', self._diag_label, self.toggle_diag),
            ActionItem(self.app, 'u', self._sys_label, self._toggle_sys),
            ActionItem(self.app, 'p', self._play_label, self.toggle_play),
            ActionItem(self.app, 'x', '선택 모두 해제', self.selected.clear),
        ]
        for i, (name, typ) in enumerate(self._topics, 1):
            mark = '[x]' if name in self.selected else '[ ]'
            items.append(ActionItem(self.app, str(i), f'{mark} {name}  [{typ}]',
                                    lambda n=name: self._toggle(n)))
        return items

    def center(self):
        self._refresh()
        lines = ['[토픽 · bag]', f' 토픽 {len(self._topics)}개 · 선택 {len(self.selected)}개']
        if self._recording():
            cur = bagrec.current()
            lines.append(f' 녹화 중: {bagrec.describe(cur) if cur else str(len(self._docker_topics)) + " docker topics"}')
            debug = getattr(self.bus.node, 'debug_dir', None)
            if self._docker_topics and debug:
                for topic in sorted(self._docker_topics)[:3]:
                    try:
                        d = json.loads((Path(debug) / 'output' / 'topic' / topic.lstrip('/') / 'bag_status').read_text())
                        lines.append(f'  {topic}: {d.get("state")} {d.get("bag")}')
                    except (OSError, ValueError):
                        pass
        else:
            lines.append(f' 저장 폴더: {self._bag_dir} · 부하 기록 {"켬" if self.sys_on else "끔"}')
        if self._playing():
            lines.append(' 재생 중')
        lines.append(self.term.paint(' s 선택 녹화 · d 진단 녹화 · u 부하 기록 · p 재생 · x 선택해제 · 숫자/Enter 토픽 선택', DIM))
        return lines + self.ui.center_summary()


__all__ = ['TopicBagMenu']
