"""RlNavController 정책 ON/OFF A/B — 상태·ON/OFF·rosbag 기록·리포트.

사용자 진입점은 `run-cli ab <노드> <동작>` / 메뉴 14 하나다. 이 파일은 UI 없이 동작만 담당한다.
  - ON/OFF: /controller_server/FollowPath/toggle (std_srvs/SetBool), 안 되면 파라미터 FollowPath.enable로 대체
  - 기록: ros2 bag record를 백그라운드로 띄우고 ~/.cache/beebot/ab.json에 pid·bag 경로를 남긴다
  - 리포트: tool/node/rl_nav_controller/report.py (rosbag2 → report.png)

bee는 ROS_DOMAIN_ID=88 + fastdds_local_only.xml(SHM 전용)이라 PC에서 원격 기록할 수 없다.
그래서 bag은 기본적으로 bee의 ~/bags/ab에 쌓는다.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy

from .consts import _param_value_to_py

# A/B 메뉴에 노출할 노드 — 새 노드가 생기면 여기와 pages/ab.py의 화면 매핑에 추가한다.
AB_NODES = (
    ('rl_nav_controller', 'Nav2 주행 정책 (정책 vs DWB)'),
)

NODE = 'rl_nav_controller'
PARAM_NODE = 'controller_server'  # 플러그인이라 파라미터는 controller_server가 들고 있음 (FollowPath.*)
TOGGLE = '/controller_server/FollowPath/toggle'
BAG_DIR = Path.home() / 'bags' / 'ab'
STATE = Path.home() / '.cache' / 'beebot' / 'ab.json'
RECORD_LOG = Path.home() / '.cache' / 'beebot' / 'ab-record.log'
REPORT = Path(__file__).resolve().parents[1] / 'node' / 'rl_nav_controller' / 'report.py'
PLAN = Path(__file__).resolve().parents[1] / 'ab' / NODE / 'command.json'

# controller 출력 + 실제 움직임 + 정책 디버그를 같이 본다.
TOPICS = ('/cmd_vel_nav',
    '/odometry/filtered', '/odom', '/imu', '/plan', '/tf', '/tf_static',
    '/controller_server/FollowPath/debug',
)


class AbTest:
    """run-cli ab의 실제 동작. Bus가 node·param_client·diag를 제공한다."""

    def __init__(self, bus):
        self.bus = bus
        self.node = bus.node
        self.prompt = bus.prompt
        self.term = bus.term
        self._toggle_cli = None
        self._nav_cli = None
        self._param_cache = (0.0, None, None)  # (읽은 시각, enabled, model_path)

    # --------------------------------------------------------------- 상태 파일
    @staticmethod
    def _load() -> dict:
        try:
            data = json.loads(STATE.read_text())
            if isinstance(data, dict):
                data.setdefault('recording', None)
                data.setdefault('history', [])
                return data
        except Exception:
            pass
        return {'recording': None, 'history': []}

    @staticmethod
    def _save(data: dict) -> None:
        STATE.parent.mkdir(parents=True, exist_ok=True)
        tmp = STATE.with_suffix('.tmp')
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=1))
        tmp.replace(STATE)

    @staticmethod
    def _alive(pid) -> bool:
        try:
            pid = int(pid)
            os.kill(pid, 0)
        except (TypeError, ValueError, ProcessLookupError, PermissionError):
            return False
        try:
            cmd = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
        except OSError:
            return True
        return 'bag' in cmd and 'record' in cmd

    def _finalize(self, data: dict, rec: dict, stopped: float | None = None) -> dict:
        rec = dict(rec)
        rec.setdefault('stopped', stopped or time.time())
        hist = [h for h in data.get('history', []) if h.get('bag') != rec.get('bag')]
        hist.append(rec)
        data['history'] = hist[-50:]
        if data.get('recording') and data['recording'].get('bag') == rec.get('bag'):
            data['recording'] = None
        self._save(data)
        return rec

    def current(self) -> dict | None:
        """지금 녹화 중인 항목. 프로세스가 죽었으면 history로 옮긴다."""
        data = self._load()
        rec = data.get('recording')
        if not rec:
            return None
        if self._alive(rec.get('pid')):
            return rec
        self._finalize(data, rec, stopped=rec.get('stopped') or time.time())
        return None

    def history(self) -> list[dict]:
        data = self._load()
        raw = data.get('history', [])
        hist = [h for h in raw if h.get('bag') and Path(h['bag']).exists()]
        if len(hist) != len(raw):  # 지워진 bag은 상태 파일에서도 정리
            data['history'] = hist
            self._save(data)
        return hist

    # ------------------------------------------------------------------ 계획
    @staticmethod
    def _pose(value, where: str) -> dict:
        if not isinstance(value, dict):
            raise RuntimeError(f'{where}: {{x, y, yaw_deg}} 객체가 아님')
        try:
            x, y = float(value['x']), float(value['y'])
            yaw = float(value.get('yaw_deg', 0.0))
        except (KeyError, TypeError, ValueError):
            raise RuntimeError(f'{where}: x·y(숫자) 필요') from None
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise RuntimeError(f'{where}: 유한한 숫자가 아님')
        return {'x': x, 'y': y, 'yaw_deg': yaw}

    def load_plan(self, path: str | None = None) -> dict:
        plan_path = Path(path).expanduser() if path else PLAN
        if not plan_path.is_file():
            raise RuntimeError(f'A/B 계획 없음: {plan_path}')
        try:
            raw = json.loads(plan_path.read_text())
        except Exception as exc:
            raise RuntimeError(f'A/B 계획 JSON 읽기 실패: {plan_path} ({exc})') from None
        if not isinstance(raw, dict):
            raise RuntimeError(f'A/B 계획이 객체가 아님: {plan_path}')
        start = None if raw.get('start') is None else self._pose(raw['start'], 'start')
        route_raw = raw.get('route')
        if not isinstance(route_raw, list) or not route_raw:
            raise RuntimeError('A/B 계획 route가 비어 있음')
        if len(route_raw) > 50:
            raise RuntimeError('A/B 계획 route가 너무 많음 (최대 50)')
        route = [self._pose(p, f'route[{i}]') for i, p in enumerate(route_raw)]
        try:
            repeats = int(raw.get('repeats', 1))
            timeout = float(raw.get('goal_timeout_s', 120.0))
            settle = float(raw.get('settle_s', 1.0))
        except (TypeError, ValueError):
            raise RuntimeError('repeats·goal_timeout_s·settle_s는 숫자') from None
        if not 1 <= repeats <= 20:
            raise RuntimeError('repeats는 1~20')
        if not 1.0 <= timeout <= 600.0:
            raise RuntimeError('goal_timeout_s는 1~600초')
        if not 0.0 <= settle <= 30.0:
            raise RuntimeError('settle_s는 0~30초')
        return {
            'path': str(plan_path),
            'frame_id': str(raw.get('frame_id') or 'map'),
            'start': start,
            'route': route,
            'repeats': repeats,
            'goal_timeout_s': timeout,
            'settle_s': settle,
            'auto_mode': bool(raw.get('auto_mode', True)),
            'move_to_start': bool(raw.get('move_to_start', True)),
            'return_to_start': bool(raw.get('return_to_start', True)),
        }

    def plan_summary(self, path: str | None = None) -> str:
        try:
            plan = self.load_plan(path)
        except RuntimeError as exc:
            return f'계획: 없음/오류 — {exc}'
        start = (f'({plan["start"]["x"]:.2f}, {plan["start"]["y"]:.2f})'
                 if plan['start'] else '현재 자세')
        return (f'계획: {Path(plan["path"]).name} · goal {len(plan["route"])}개 · '
                f'{plan["repeats"]}회 · 시작 {start}')

    # ------------------------------------------------------------------ 상태
    def params(self, force: bool = False) -> tuple[bool | None, str | None]:
        now = time.monotonic()
        at, enabled, model = self._param_cache
        if not force and now - at < 1.0:
            return enabled, model
        enabled = model = None
        cli = self.bus.param_client(PARAM_NODE, '/')
        if cli is not None and cli.wait_for_services(timeout_sec=1.5):
            descs = self.bus.param_describe(cli, ['FollowPath.enable', 'FollowPath.model_path']) or []
            vals = self.bus.param_get_vals(cli, ['FollowPath.enable', 'FollowPath.model_path']) or []
            for i, name in enumerate(('enabled', 'model_path')):
                if i >= len(descs) or i >= len(vals):
                    continue
                try:
                    value = _param_value_to_py(descs[i].type, vals[i])
                except Exception:
                    value = None
                if name == 'enabled' and isinstance(value, bool):
                    enabled = value
                elif name == 'model_path' and isinstance(value, str):
                    model = value
        self._param_cache = (time.monotonic(), enabled, model)
        return enabled, model

    def status(self) -> dict:
        enabled, model = self.params()
        return {
            'alive': enabled is not None,
            'enabled': enabled,
            'model': model,
            'recording': self.current(),
            'history': self.history(),
        }

    def status_lines(self) -> list[str]:
        st = self.status()
        on = 'ON' if st['enabled'] else 'OFF' if st['enabled'] is False else '? (노드 응답 없음)'
        lines = [f'[{NODE}] 정책 {on}']
        if st['model']:
            lines.append(f' 모델: {st["model"]}')
        diags = [(n, lv, msg) for n, lv, msg in self.bus.diag_entries() if 'controller_server' in n]
        for n, lv, msg in diags[:4]:
            mark = 'OK' if lv == 0 else 'WARN' if lv == 1 else 'ERR' if lv == 2 else '?'
            lines.append(f' 진단 {self.bus.diag_short(n)}: {mark} {msg}')
        rec = st['recording']
        if rec:
            elapsed = max(0.0, time.time() - float(rec.get('started', time.time())))
            lines.append(f' 녹화 중: {rec.get("label", "-")} · {elapsed:.0f}s · {rec.get("bag")}')
        else:
            lines.append(' 녹화 중: 없음')
        hist = st['history'][-2:]
        if hist:
            lines.append(' 최근: ' + ' / '.join(
                f'{h.get("label", "?")} {Path(h["bag"]).name}' for h in hist))
        lines.append(' ' + self.plan_summary())
        return lines

    # ------------------------------------------------------------------ 토글
    def toggle(self, on: bool) -> tuple[bool, str]:
        """서비스 우선, 실패하면 파라미터 enabled로 바꾼다."""
        try:
            from std_srvs.srv import SetBool
        except Exception:
            SetBool = None
        if SetBool is not None:
            if self._toggle_cli is None:
                self._toggle_cli = self.node.create_client(SetBool, TOGGLE)
            cli = self._toggle_cli
            if cli.wait_for_service(timeout_sec=2.0):
                req = SetBool.Request()
                req.data = bool(on)
                fut = cli.call_async(req)
                try:
                    rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
                except Exception:
                    pass
                if fut.done():
                    try:
                        res = fut.result()
                    except Exception:
                        res = None
                    if res is not None and getattr(res, 'success', False):
                        self._param_cache = (0.0, None, None)
                        return True, str(getattr(res, 'message', '') or f'enabled={bool(on)}')

        from rclpy.parameter import Parameter
        cli = self.bus.param_client(PARAM_NODE, '/')
        if cli is not None and cli.wait_for_services(timeout_sec=2.0):
            try:
                res = self.bus.param_wait(
                    cli.set_parameters([Parameter('FollowPath.enable', value=bool(on))]), timeout=3.0)
            except Exception as exc:
                return False, f'파라미터 설정 실패: {exc}'
            if res is not None and getattr(res, 'results', None) and res.results[0].successful:
                self._param_cache = (0.0, None, None)
                return True, f'enabled={bool(on)}'
        return False, '토글 서비스·파라미터 모두 응답 없음'

    # ------------------------------------------------------------------ 기록
    @staticmethod
    def _log_tail(n: int = 8) -> str:
        try:
            lines = RECORD_LOG.read_text(errors='replace').splitlines()
            return '\n'.join(lines[-n:])
        except Exception:
            return ''

    def start(self, label: str | None = None, out: str | None = None) -> dict:
        cur = self.current()
        if cur:
            raise RuntimeError(f'이미 녹화 중: {cur.get("bag")}')
        label = re.sub(r'[^\w.-]+', '_', (label or 'run').strip()) or 'run'
        if out:
            bag = Path(out).expanduser()
        else:
            stamp = time.strftime('%Y%m%d_%H%M%S')
            bag = BAG_DIR / f'{stamp}_{label}'
            n = 1
            while bag.exists():
                n += 1
                bag = BAG_DIR / f'{stamp}_{label}_{n}'
        bag = bag.resolve()
        bag.parent.mkdir(parents=True, exist_ok=True)
        RECORD_LOG.parent.mkdir(parents=True, exist_ok=True)
        cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '--disable-keyboard-controls',
               '-o', str(bag), '--topics', *TOPICS]
        started = time.time()
        with open(RECORD_LOG, 'wb') as log:
            log.write((' '.join(cmd) + '\n').encode())
            log.flush()
            proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, start_new_session=True)
        # bag 폴더가 실제로 생길 때까지 기다린다 — discovery가 느리면 1~2초 걸리고,
        # 그 전에 stop하면 rosbag2가 폴더를 만들지 못한 채 끝날 수 있다.
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                raise RuntimeError(
                    f'ros2 bag record 시작 실패 (exit {proc.returncode})\n{self._log_tail()}')
            if bag.is_dir():
                break
            time.sleep(0.1)
        else:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except Exception:
                pass
            tail = self._log_tail()
            hint = (' — Fast DDS SHM 잔재 가능: `run-cli stop` → `run-cli start`로 정리하세요'
                    if 'RTPS_TRANSPORT_SHM' in tail or 'open_and_lock_file' in tail else '')
            raise RuntimeError(f'ros2 bag record가 8초 안에 bag 폴더를 만들지 못함{hint}\n{tail}')
        time.sleep(0.2)
        if proc.poll() is not None:
            raise RuntimeError(f'ros2 bag record 시작 실패 (exit {proc.returncode})\n{self._log_tail()}')
        rec = {'pid': proc.pid, 'bag': str(bag), 'label': label,
               'started': started, 'log': str(RECORD_LOG)}
        data = self._load()
        data['recording'] = rec
        self._save(data)
        return rec

    def stop(self) -> dict | None:
        data = self._load()
        rec = data.get('recording')
        if not rec:
            return None
        pid = int(rec.get('pid') or 0)
        if pid and self._alive(pid):
            try:
                os.killpg(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            except Exception:
                try:
                    os.kill(pid, signal.SIGINT)
                except Exception:
                    pass
            end = time.monotonic() + 10.0
            while time.monotonic() < end and self._alive(pid):
                time.sleep(0.2)
            if self._alive(pid):
                try:
                    os.killpg(pid, signal.SIGKILL)
                except Exception:
                    pass
                time.sleep(0.5)
        return self._finalize(data, rec)

    # ------------------------------------------------------------------ 리포트
    def report(self, bags: list[str], labels: list[str] | None = None,
               out: str | None = None) -> int:
        if not REPORT.is_file():
            raise RuntimeError(f'리포트 스크립트 없음: {REPORT}')
        bags = [str(Path(b).expanduser()) for b in bags]
        for bag in bags:
            if not Path(bag).exists():
                raise RuntimeError(f'bag 없음: {bag}')
        if out is None:
            out = str(Path(bags[0]).parent / f'report_{time.strftime("%Y%m%d_%H%M%S")}.png')
        out = str(Path(out).expanduser())
        Path(out).parent.mkdir(parents=True, exist_ok=True)
        cmd = [sys.executable, str(REPORT), *bags]
        if labels:
            cmd += ['--labels', *labels]
        cmd += ['--out', out]
        print(f'리포트 생성: {" ".join(cmd)}')
        # subprocess 출력은 OS fd로 나가 TUI의 캡처를 피한다 — 직접 받아 print로 넘긴다.
        res = subprocess.run(cmd, capture_output=True, text=True)
        text = ((res.stdout or '') + (res.stderr or '')).rstrip()
        if text:
            print(text)
        if res.returncode == 0:
            print(f'저장: {out}')
        return int(res.returncode)

    def latest(self, n: int = 2) -> list[dict]:
        return self.history()[-n:]

    # ------------------------------------------------------------------ 자동 주행
    def _progress(self, text: str) -> None:
        self.prompt.progress(text)   # TUI '처리 중' 창 — 메뉴 밖이면 무시
        print(text)

    def _sleep(self, seconds: float) -> None:
        end = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=min(0.1, max(0.01, end - time.monotonic())))

    def _current_map_pose(self, frame_id: str) -> dict:
        """map→base_footprint TF로 지금 자세를 잡는다 (start=null일 때)."""
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformListener

        buf = Buffer()
        listener = TransformListener(buf, self.node)
        end = time.monotonic() + 5.0
        last_err = ''
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                tr = buf.lookup_transform(frame_id, 'base_footprint', Time())
                t = tr.transform.translation
                q = tr.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                return {'x': float(t.x), 'y': float(t.y), 'yaw_deg': math.degrees(yaw)}
            except Exception as exc:
                last_err = str(exc)
        del listener
        raise RuntimeError(f'{frame_id}→base_footprint TF 없음 — 지도/로컬라이제이션 확인 ({last_err})')

    def _nav_client(self):
        if self._nav_cli is None:
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient
            self._nav_cli = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        return self._nav_cli

    def nav_goal(self, pose: dict, frame_id: str, timeout: float, what: str) -> None:
        """Nav2 NavigateToPose 한 점 — 도착/성공까지 기다린다. 실패는 RuntimeError."""
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import PoseStamped

        cli = self._nav_client()
        self._progress(f'{what}: /navigate_to_pose 대기')
        if not cli.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('/navigate_to_pose 액션 서버 없음 — 지도/로컬라이제이션/Nav2 확인')
        from nav2_msgs.action import NavigateToPose
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pose['x'])
        goal.pose.pose.position.y = float(pose['y'])
        yaw = math.radians(float(pose.get('yaw_deg', 0.0)))
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send = cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=10.0)
        if not send.done():
            raise RuntimeError(f'{what}: goal 전송 timeout')
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f'{what}: goal 거부됨')
        result_fut = handle.get_result_async()
        end = time.monotonic() + timeout
        next_tick = 0.0
        while not result_fut.done() and time.monotonic() < end and rclpy.ok():
            rclpy.spin_until_future_complete(self.node, result_fut, timeout_sec=0.5)
            now = time.monotonic()
            if now >= next_tick:
                self._progress(f'{what}: 주행 중 … {timeout - max(0.0, end - now):.0f}/{timeout:.0f}s')
                next_tick = now + 2.0
        if not result_fut.done():
            handle.cancel_goal_async()
            raise RuntimeError(f'{what}: timeout ({timeout:.0f}s)')
        result = result_fut.result()
        status = getattr(result, 'status', None)
        if status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'{what}: Nav2 실패 (status={status})')
        self._progress(f'{what}: 도착')

    def _ensure_auto(self, plan: dict) -> None:
        self.bus._poll(0.8)
        if self.bus.state.emergency:
            raise RuntimeError('비상정지 ON — 먼저 해제하세요')
        if self.bus.state.operation == 'AUTO':
            return
        if not plan['auto_mode']:
            raise RuntimeError(f'조작 모드가 {self.bus.state.operation or "?"} — AUTO 필요')
        self._progress(f'조작 모드 {self.bus.state.operation or "?"} → AUTO')
        rc = self.bus.cmd_mode(argparse.Namespace(mode='auto'))
        if rc != 0:
            raise RuntimeError('AUTO 전환 실패')

    def _run_route(self, plan: dict, label: str) -> None:
        route = plan['route']
        for i, pose in enumerate(route, 1):
            self.nav_goal(pose, plan['frame_id'], plan['goal_timeout_s'],
                          f'{label} goal {i}/{len(route)}')
            if plan['settle_s']:
                self._sleep(plan['settle_s'])

    # ------------------------------------------------------------------ 명령
    LEGACY_ACTIONS = ('status', 'on', 'off', 'record', 'stop', 'report', 'guide')

    def cmd(self, args) -> int:
        target = getattr(args, 'node', None) or 'nodes'
        action = getattr(args, 'action', None)
        # 예전 `ab status`도 한동안 받아 준다 — 새 문법은 `ab rl_nav_controller status`.
        if target in self.LEGACY_ACTIONS and action is None:
            print(f'WARN: `ab {target}`는 예전 문법 — `ab {NODE} {target}`로 바뀌었습니다',
                  file=sys.stderr)
            action = target
            target = NODE
        try:
            if target == 'nodes':
                for name, desc in AB_NODES:
                    print(f'  {name:<24} {desc}')
                print('\n사용: ab <노드> <동작> — 예: ab rl_nav_controller status')
                return 0
            if target != NODE:
                print(f'FAIL: A/B 미구현 노드: {target} (가능: {", ".join(n for n, _ in AB_NODES)})',
                      file=sys.stderr)
                return 2
            action = action or 'status'
            if action == 'status':
                self.bus._poll(0.8)   # /diagnostics_agg가 올 시간을 준다
                print('\n'.join(self.status_lines()))
                return 0
            if action in ('on', 'off'):
                ok, msg = self.toggle(action == 'on')
                print(('ok: ' if ok else 'FAIL: ') + msg)
                return 0 if ok else 1
            if action == 'record':
                rec = self.start(getattr(args, 'label', None), getattr(args, 'out', None))
                print(f'녹화 시작: {rec["bag"]} (pid {rec["pid"]})')
                print(f'  로그: {rec["log"]}')
                print('  주행 후: ./run-cli ab rl_nav_controller stop')
                return 0
            if action == 'stop':
                rec = self.stop()
                if rec is None:
                    print('녹화 중 아님')
                    return 0
                dur = max(0.0, float(rec.get('stopped', time.time())) - float(rec.get('started', time.time())))
                print(f'녹화 종료: {rec["bag"]} ({dur:.1f}s)')
                return 0
            if action == 'report':
                bags = list(getattr(args, 'bags', []) or [])
                if getattr(args, 'last', False) or not bags:
                    pair = self.latest(2)
                    if len(pair) < 2:
                        print('FAIL: 최근 녹화가 2개 미만 — 먼저 A/B를 기록하세요', file=sys.stderr)
                        return 2
                    bags = [h['bag'] for h in pair]
                labels = getattr(args, 'labels', None)
                if not labels and len(bags) == 2:
                    by_bag = {h['bag']: h.get('label') for h in self.history()}
                    labels = [by_bag.get(bags[0], 'A'), by_bag.get(bags[1], 'B')]
                return self.report(bags, labels, getattr(args, 'out', None))
            if action == 'guide':
                return self.guide(getattr(args, 'label', None), getattr(args, 'config', None))
            print(f'FAIL: 모르는 ab 동작: {action}', file=sys.stderr)
            return 2
        except (RuntimeError, OSError) as exc:
            print(f'FAIL: {exc}', file=sys.stderr)
            return 1

    def guide(self, label: str | None = None, config: str | None = None) -> int:
        """command.json 계획대로 A(ON) → 복귀 → B(OFF)를 Nav2로 자동 주행하고 리포트."""
        label = (label or time.strftime('%Y%m%d_%H%M%S')).strip()
        try:
            plan = self.load_plan(config)
        except RuntimeError as exc:
            print(f'FAIL: {exc}', file=sys.stderr)
            return 2
        # DDS 참여자가 굳은 상태에서 bag record를 띄우면 Fast DDS SHM 포트 lock으로 실패한다.
        # 상단 연결이 정상으로 돌아온 뒤에만 시작한다 (자동 재연결은 12초 뒤).
        age = (time.monotonic() - self.bus.state.state_last_update
               if self.bus.state.state_last_update > 0 else float('inf'))
        if age > 5.0:
            print(f'FAIL: /state 갱신이 {age:.0f}s 끊겼습니다 — 상단 연결이 정상이 된 뒤 다시 실행하세요',
                  file=sys.stderr)
            return 1
        try:
            self._ensure_auto(plan)
            if plan['start'] is None:
                start = self._current_map_pose(plan['frame_id'])
                self._progress(f'시작 자세: ({start["x"]:.2f}, {start["y"]:.2f}, {start["yaw_deg"]:.0f}°)')
            else:
                start = plan['start']
        except RuntimeError as exc:
            print(f'FAIL: {exc}', file=sys.stderr)
            return 1

        # 파라미터 읽기(status) 대신 토글 서비스로 확실히 ON을 보장한다.
        ok, msg = self.toggle(True)
        if not ok:
            print(f'FAIL: {msg}', file=sys.stderr)
            return 1

        bags: list[str] = []
        labels: list[str] = []
        ok_all = True
        try:
            for r in range(1, plan['repeats'] + 1):
                if plan['start'] is not None and plan['move_to_start']:
                    self.nav_goal(start, plan['frame_id'], plan['goal_timeout_s'], f'{r}회차 시작 자세')
                # A — 정책 ON
                rec = self.start(f'{label}_r{r}_A')
                bags.append(rec['bag'])
                labels.append(f'A{r}')
                self._progress(f'A{r} 녹화: {rec["bag"]}')
                try:
                    self._run_route(plan, f'A{r}')
                except RuntimeError as exc:
                    print(f'FAIL: {exc}', file=sys.stderr)
                    ok_all = False
                finally:
                    self.stop()
                if plan['return_to_start']:
                    try:
                        self.nav_goal(start, plan['frame_id'], plan['goal_timeout_s'], f'A{r} → 시작 자세')
                    except RuntimeError as exc:
                        print(f'FAIL: {exc}', file=sys.stderr)
                        ok_all = False
                        break

                ok, msg = self.toggle(False)
                if not ok:
                    raise RuntimeError(msg)
                # B — 정책 OFF (inner DWB)
                rec = self.start(f'{label}_r{r}_B')
                bags.append(rec['bag'])
                labels.append(f'B{r}')
                self._progress(f'B{r} 녹화: {rec["bag"]}')
                try:
                    self._run_route(plan, f'B{r}')
                except RuntimeError as exc:
                    print(f'FAIL: {exc}', file=sys.stderr)
                    ok_all = False
                finally:
                    self.stop()

                ok, msg = self.toggle(True)
                if not ok:
                    raise RuntimeError(msg)
                if not ok_all:
                    break
        except (RuntimeError, OSError) as exc:
            print(f'FAIL: {exc}', file=sys.stderr)
            ok_all = False
        finally:
            if self.current():
                self.stop()
            st2 = self.status()
            if st2['enabled'] is not True:
                self.toggle(True)

        if len(bags) >= 2:
            out = str(Path(bags[0]).parent / f'report_{time.strftime("%Y%m%d_%H%M%S")}.png')
            if self.report(bags, labels, out) != 0:
                ok_all = False
        return 0 if ok_all else 1
