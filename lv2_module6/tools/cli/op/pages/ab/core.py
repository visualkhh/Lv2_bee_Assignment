"""RlNavController 정책 ON/OFF A/B — 상태·ON/OFF·rosbag 기록·리포트.

사용자 진입점은 `run-cli ab <노드> <동작>` / 메뉴 13 하나다. 이 파일은 UI 없이 동작만 담당한다.
  - ON/OFF: /controller_server/FollowPath/toggle (std_srvs/SetBool), 안 되면 파라미터 FollowPath.enable로 대체
  - 기록: ros2 bag record를 백그라운드로 띄우고 ~/.cache/beebot/ab.json에 pid·bag 경로를 남긴다
  - 리포트: pages/ab/rl_nav_controller/report.py (rosbag2 → report.png)

bee는 ROS_DOMAIN_ID=88 + fastdds_local_only.xml(SHM 전용)이라 PC에서 원격 기록할 수 없다.
산출물(bag·리포트)은 tools/artifacts/ab/<노드> 아래에 쌓는다 (컨테이너: /debug/tools/artifacts).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy

from ...consts import _param_value_to_py

# A/B 메뉴에 노출할 노드 — 새 노드가 생기면 여기와 pages/ab/__init__.py의 화면 매핑에 추가한다.
AB_NODES = (
    ('rl_nav_controller', 'Nav2 주행 정책 (정책 vs DWB)'),
)

NODE = 'rl_nav_controller'
PARAM_NODE = 'controller_server'  # 플러그인이라 파라미터는 controller_server가 들고 있음 (FollowPath.*)
TOGGLE = '/controller_server/FollowPath/toggle'
STATUS = '/controller_server/FollowPath/status'  # [rl_active, reason, …] — 컨트롤러가 한 번 돌 때마다 (rl_nav_controller.hpp)
REASONS = {0: '정책', 1: '모델 없음', 2: 'enable=false(DWB)', 3: '센서 없음/stale', 4: '추론 실패'}
# guide 전에 active여야 하는 Nav2 수명주기 노드 (map_server는 저장 지도용 — 테스트베드 fake SLAM은 fake_nvblox가 /map)
LIFECYCLE_NODES = ('planner_server', 'controller_server', 'bt_navigator', 'behavior_server',
                   'velocity_smoother', 'collision_monitor')
MIN_GOAL_GAP_M = 0.3  # 출발·goal 사이 최소 거리 — goal_checker 허용(0.10)보다 넉넉히
RL_MIN_RATIO = 0.9  # A leg에서 컨트롤러 주기 중 정책이 돈 비율이 이보다 작으면 무효


def _artifact_root() -> Path:
    # run-cli 산출물 루트: 컨테이너(/debug 공유 볼륨 있음)면 debug 아래,
    # 호스트 직접 실행이면 레포 tools/artifacts 아래. 둘 다 git 무시 대상.
    if Path('/debug').is_dir():
        return Path('/debug/tools/artifacts')
    return Path(__file__).resolve().parents[4] / 'artifacts'


ARTIFACT_ROOT = _artifact_root()
BAG_DIR = ARTIFACT_ROOT / 'ab' / NODE  # 노드별 폴더: ab/rl_nav_controller/<bag> · report_<그룹>.png
STATE = Path.home() / '.cache' / 'beebot' / 'ab.json'
RECORD_LOG = Path.home() / '.cache' / 'beebot' / 'ab-record.log'
REPORT = Path(__file__).resolve().parent / NODE / 'report.py'  # command.json 옆
PLAN = Path(__file__).resolve().parent / NODE / 'command.json'  # 기본 설정 (저장소 — 읽기만)
ROUTE = BAG_DIR / 'route.json'  # TUI·RViz로 지정한 start·route (산출물 폴더 — 실기·도커 모두 쓰기 가능). 있으면 command.json보다 우선


class LegStopped(Exception):
    """주행 중 s — 지금 leg만 멈추고 다음 leg로."""


class AbortAll(Exception):
    """주행 중 q — A/B 전체 중단."""

# controller 출력 + 실제 움직임 + 정책 디버그를 같이 본다.
TOPICS = ('/cmd_vel_nav', '/cmd_vel', '/cmd_vel_mux',  # nav 출력 · 모니터 통과 후 실제 · mux 출력
    '/odometry/filtered', '/odom', '/imu', '/plan', '/tf', '/tf_static',
    '/controller_server/FollowPath/debug', STATUS,
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
        self._pub_reset = None
        self._param_cache = (0.0, None, None)  # (읽은 시각, enabled, model_path)
        self._tf_buf = None  # map → base_footprint 조회용 (leg 자세 기록 · 시작 자세)
        self._tf_listener = None
        self._map = None     # 사전 점검에서 받은 /map — 그룹 폴더에 저장해 리포트 배경으로

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
        """설정 = command.json(또는 --config), start·route = route.json이 있으면 그것 (TUI·RViz로 지정한 경로)."""
        plan = self._load_plan_file(path)
        if path is None and ROUTE.is_file():
            try:
                raw = json.loads(ROUTE.read_text())
                start = None if raw.get('start') is None else self._pose(raw['start'], 'route.json start')
                route = [self._pose(p, f'route.json route[{i}]') for i, p in enumerate(raw.get('route') or [])]
            except (ValueError, RuntimeError) as exc:
                raise RuntimeError(f'route.json 읽기 실패: {ROUTE} ({exc}) — TUI 경로 편집에서 지우거나 다시 지정') from None
            if not route:
                raise RuntimeError(f'route.json에 goal이 없음: {ROUTE}')
            plan.update(start=start, route=route, route_path=str(ROUTE))
        return plan

    def save_route(self, start: dict | None, route: list[dict]) -> Path:
        """TUI·RViz로 지정한 경로 저장 → 이후 guide가 이걸 쓴다."""
        ROUTE.parent.mkdir(parents=True, exist_ok=True)
        tmp = ROUTE.with_suffix('.tmp')
        tmp.write_text(json.dumps({'frame_id': 'map', 'start': start, 'route': route,
                                   'saved': time.strftime('%Y-%m-%d %H:%M:%S')}, ensure_ascii=False, indent=1))
        tmp.replace(ROUTE)
        return ROUTE

    def clear_route(self) -> bool:
        """route.json 지우기 → command.json 기본 경로로."""
        if ROUTE.is_file():
            ROUTE.unlink()
            return True
        return False

    def current_pose(self, frame_id: str = 'map') -> dict:
        """지금 로봇 자세 (TUI '지금 위치를 goal로' 등) — TF가 없으면 RuntimeError."""
        return self._current_map_pose(frame_id)

    def set_robot_pose(self, x: float, y: float, yaw_deg: float, frame_id: str = 'map') -> None:
        """로봇 위치를 지도 위 (x, y, yaw)로.
        faker는 /cmd/fake_opencr/set_pose 텔레포트, 실기는 /initialpose (cuVSLAM 등 위치추정 노드가 받는다)."""
        if self._is_sim():
            self._reset_sim_pose({'x': x, 'y': y, 'yaw_deg': yaw_deg}, frame_id)
        else:
            self._publish_initialpose(x, y, yaw_deg, frame_id)

    def _publish_initialpose(self, x: float, y: float, yaw_deg: float, frame_id: str) -> None:
        from geometry_msgs.msg import PoseWithCovarianceStamped
        yaw = math.radians(float(yaw_deg))
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        pub = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        try:
            for _ in range(3):
                pub.publish(msg)
                self.bus._poll(0.2)
            if pub.get_subscription_count() == 0:
                raise RuntimeError('/initialpose 구독자 없음 — cuVSLAM initialpose 지원(재시작) 확인')
        finally:
            self.node.destroy_publisher(pub)
        print(f'[initialpose] map ({x:+.2f}, {y:+.2f}, {yaw_deg:+.1f}°)')

    def robot_pose(self, frame_id: str = 'map') -> dict | None:
        """지금 로봇 자세 — 기다리지 않음 (지도 화면이 0.3초마다 부른다).
        docker는 호스트에 tf2_ros가 없으므로 fake_opencr의 map 프레임 ground_truth를 쓴다 (odom 아님)."""
        if os.environ.get('CLI_SOURCE') == 'docker' and frame_id == 'map':
            truth = self._docker_ground_truth()
            if truth is not None:
                return truth
        try:
            x, y, yaw = self._lookup_pose(frame_id)
        except Exception:
            return None
        return {'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(math.degrees(yaw), 1)}

    def _docker_ground_truth(self) -> dict | None:
        debug = getattr(self.node, 'debug_dir', None)
        if not debug:
            return None
        try:
            doc = json.loads((Path(debug) / 'output' / 'topic' / 'fake_opencr' / 'ground_truth' / 'message').read_text())
            pose = (doc.get('data') or {}).get('pose') or {}
            pos, q = pose.get('position') or {}, pose.get('orientation') or {}
            x, y = float(pos['x']), float(pos['y'])
            qx, qy, qz, qw = (float(q.get(k, 0.0)) for k in ('x', 'y', 'z', 'w'))
            if qw == 0.0 and qz == 0.0:
                qw = 1.0
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            return {'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(math.degrees(yaw), 1)}
        except (OSError, ValueError, KeyError, TypeError):
            return None

    def _load_plan_file(self, path: str | None = None) -> dict:
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
            'route_path': str(plan_path),
        }

    def plan_summary(self, path: str | None = None) -> str:
        try:
            plan = self.load_plan(path)
        except RuntimeError as exc:
            return f'계획: 없음/오류 — {exc}'
        start = (f'({plan["start"]["x"]:.2f}, {plan["start"]["y"]:.2f})'
                 if plan['start'] else '현재 자세')
        src = 'route.json (TUI·RViz 지정)' if plan['route_path'] == str(ROUTE) else f'{Path(plan["route_path"]).name} (기본)'
        return f'경로: {src} · goal {len(plan["route"])}개 · {plan["repeats"]}회 · 시작 {start}'

    # ------------------------------------------------------------------ 상태
    def params(self, force: bool = False) -> tuple[bool | None, str | None]:
        now = time.monotonic()
        at, enabled, model = self._param_cache
        if not force and now - at < 1.0:
            return enabled, model
        # 실제 적용 상태는 플러그인 진단이 정확하다 (toggle 서비스는 ROS 파라미터를 안 바꾼다).
        diag = next((dict(v) for n, v in self.bus.state.diag_values.items()
                     if 'controller_server' in n), None)
        if diag and 'enable' in diag:
            enabled = diag.get('enable') == 'true'
            model = diag.get('model') or model
            self._param_cache = (now, enabled, model)
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
        for diag_name, values in self.bus.state.diag_values.items():
            if 'controller_server' in diag_name:
                d = dict(values)
                lines.append(
                    f' device {d.get("device", "?")} · {d.get("provider", "?")}'
                    f' · enable {d.get("enable", "?")} · active {d.get("active", "?")}'
                    f' · mode {d.get("mode", "?")} · fallback {d.get("inner", "?")}')
                break
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
        # 토픽을 다 구독할 때까지 기다린다 — 그 전에 주행하면 출발 순간의 /plan(Nav2 전역 경로, latched 아님)이
        # bag에 빠져 리포트가 밀려난 뒤 다시 짠 경로를 기준으로 삼는다 (실측: /plan 구독까지 약 3초).
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and proc.poll() is None:
            if 'All requested topics are subscribed' in self._log_tail(40):
                break
            time.sleep(0.2)
        else:
            if proc.poll() is None:
                print('WARN: 10초 안에 모든 토픽을 구독하지 못함 — 그대로 진행 (빠진 토픽은 리포트에 없음)', file=sys.stderr)
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
        end = time.monotonic() + 5.0
        last_err = ''
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                x, y, yaw = self._lookup_pose(frame_id)
                return {'x': x, 'y': y, 'yaw_deg': math.degrees(yaw)}
            except Exception as exc:
                last_err = str(exc)
        raise RuntimeError(f'{frame_id}→base_footprint TF 없음 — 지도/로컬라이제이션 확인 ({last_err})')

    def _lookup_pose(self, frame_id: str) -> tuple[float, float, float]:
        """지금 map(frame_id) 기준 base_footprint (x, y, yaw[rad]) — TF가 없으면 예외."""
        from rclpy.time import Time
        if self._tf_buf is None:
            from tf2_ros import Buffer, TransformListener
            self._tf_buf = Buffer()
            self._tf_listener = TransformListener(self._tf_buf, self.node)
        tr = self._tf_buf.lookup_transform(frame_id, 'base_footprint', Time())
        t, q = tr.transform.translation, tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return float(t.x), float(t.y), yaw

    def _nav_client(self):
        if self._nav_cli is None:
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient
            self._nav_cli = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        return self._nav_cli

    def _reset_sim_pose(self, pose: dict, frame_id: str) -> None:
        """시뮬 로봇을 leg 시작점으로 텔레포트 — A가 맵 밖으로 나가도 B는 기획 가능.
        fake_opencr 있으면 /cmd/fake_opencr/set_pose(진짜 위치), mock이면
        /diff_drive_controller/set_odometry(odom). 둘 다 쏜다 — 각 모드에서 하나씩만 먹는다."""
        import math as _math
        from geometry_msgs.msg import PoseStamped
        yaw = _math.radians(float(pose.get('yaw_deg', 0.0)))
        if self._pub_reset is None:
            from std_msgs.msg import String as _S  # noqa — 임포트 순서 고정용 더미
            self._pub_reset = self.node.create_publisher(PoseStamped, '/cmd/fake_opencr/set_pose', 10)
        m = PoseStamped()
        m.header.frame_id = frame_id
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.pose.position.x = float(pose['x'])
        m.pose.position.y = float(pose['y'])
        m.pose.orientation.z = _math.sin(yaw / 2.0)
        m.pose.orientation.w = _math.cos(yaw / 2.0)
        for _ in range(3):
            self._pub_reset.publish(m)
            self.bus._poll(0.2)
        try:
            from control_msgs.srv import SetOdometry
            cli = self.node.create_client(SetOdometry, '/diff_drive_controller/set_odometry')
            if cli.wait_for_service(timeout_sec=2.0):
                req = SetOdometry.Request()
                req.x = float(pose['x'])
                req.y = float(pose['y'])
                req.yaw = yaw
                fut = cli.call_async(req)
                rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        except Exception:
            pass
        self.bus._poll(1.0)  # TF·EKF가 점프를 따라잡을 시간

    @staticmethod
    def _write_plan_csv(bag: str, start: dict, plan: dict) -> None:
        """leg 폴더에 plan.csv (route 웨이포인트, map 좌표).
        bag의 /plan은 latched라 rosbag record가 놓치기 일쑤 — guide가 아는 route를 직접 쓴다."""
        try:
            import csv
            pts = [(float(start['x']), float(start['y']))]
            pts += [(float(p['x']), float(p['y'])) for p in plan.get('route', [])]
            with open(Path(bag) / 'plan.csv', 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['x', 'y'])
                w.writerows(pts)
        except Exception as exc:
            print(f'WARN: plan.csv 기록 실패: {exc}', file=sys.stderr)

    def send_nav_goal(self, pose: dict, frame_id: str = 'map') -> None:
        """Nav2 NavigateToPose 한 점을 보내고 바로 돌아온다 — 지도 화면 goal 찍기용 (진행은 로봇이 한다).
        docker 소스는 가짜 rclpy에 nav2_msgs.action이 없으므로 컨테이너의 ros2 CLI로 보낸다."""
        if os.environ.get('CLI_SOURCE') == 'docker':
            self._send_nav_goal_docker(pose, frame_id)
            return
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateToPose

        cli = self._nav_client()
        if not cli.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('/navigate_to_pose 액션 서버 없음 — 지도/로컬라이제이션/Nav2 확인')
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
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=5.0)
        if not send.done():
            raise RuntimeError('이동 목표: goal 전송 timeout')
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('이동 목표: goal 거부됨')

    def _send_nav_goal_docker(self, pose: dict, frame_id: str) -> None:
        """docker: debug/input/action/navigate_to_pose/goal 파일 브리지 → 컨테이너 service_action_bag_input → 진짜 Nav2.
        (rclpy action을 호스트에 흉내내지 않는다 — 컨테이너가 이미 액션을 받아 준다.)"""
        debug = getattr(self.node, 'debug_dir', None) or os.environ.get('CLI_DEBUG_DIR')
        if not debug:
            raise RuntimeError('docker debug 경로를 모름 — --debug-dir 확인')
        debug = Path(debug)
        name = 'navigate_to_pose'
        folder = debug / 'input' / 'action' / name
        folder.mkdir(parents=True, exist_ok=True)
        goal_id = f'cli-{os.getpid()}-{int(time.time() * 1000)}'
        yaw = math.radians(float(pose.get('yaw_deg', 0.0)))
        payload = {'goal_id': goal_id, 'data': {'pose': {
            'position': {'x': float(pose['x']), 'y': float(pose['y']), 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0,
                            'z': math.sin(yaw / 2.0), 'w': math.cos(yaw / 2.0)},
        }}}
        goal = folder / 'goal'
        tmp = folder / f'.goal-{os.getpid()}-{int(time.time() * 1000)}'
        tmp.write_text(json.dumps(payload, ensure_ascii=False))
        os.replace(tmp, goal)
        accept_path = debug / 'output' / 'action' / name / 'accept'
        end = time.monotonic() + 5.0
        while time.monotonic() < end and rclpy.ok():
            try:
                doc = json.loads(accept_path.read_text())
            except (OSError, ValueError):
                time.sleep(0.1)
                continue
            if str(doc.get('goal_id', '')) == goal_id:
                if not doc.get('accepted'):
                    raise RuntimeError(f'goal 거부: {doc.get("error") or "이유 없음"}')
                print(f'[docker] Nav2 goal 전송 → ({pose["x"]:.2f}, {pose["y"]:.2f}, {pose.get("yaw_deg", 0):.0f}°)')
                return
            time.sleep(0.1)
        raise RuntimeError('action 브리지 응답 없음 — service_action_bag_input/Nav2 확인')

    def nav_goal(self, pose: dict, frame_id: str, timeout: float, what: str) -> None:
        """Nav2 NavigateToPose 한 점 — 도착/성공까지 기다린다. 실패는 RuntimeError.
        TUI에서 기다리는 동안 s = LegStopped(이 leg만) · q = AbortAll(전체) — goal을 취소하고 올린다."""
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
        stop = None
        while not result_fut.done() and time.monotonic() < end and rclpy.ok():
            rclpy.spin_until_future_complete(self.node, result_fut, timeout_sec=0.5)
            key = self.prompt.poll_key() if hasattr(self.prompt, 'poll_key') else None
            if key in ('s', 'S'):
                stop = LegStopped(f'{what}: 사용자가 중지 (s)')
                break
            if key in ('q', 'Q'):
                stop = AbortAll(f'{what}: 사용자가 전체 중단 (q)')
                break
            now = time.monotonic()
            if now >= next_tick:
                self._progress(f'{what}: 주행 중 … {timeout - max(0.0, end - now):.0f}/{timeout:.0f}s  ·  s 이 leg 중지 · q 전체 중단')
                next_tick = now + 2.0
        if stop is not None:
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel, timeout_sec=5.0)
            end2 = time.monotonic() + 5.0
            while not result_fut.done() and time.monotonic() < end2 and rclpy.ok():
                rclpy.spin_until_future_complete(self.node, result_fut, timeout_sec=0.5)
            raise stop
        if not result_fut.done():
            # 취소가 처리되기 전에 다음 goal을 보내면 BT가 거부한다 — 결과 올 때까지 대기.
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel, timeout_sec=5.0)
            end2 = time.monotonic() + 5.0
            while not result_fut.done() and time.monotonic() < end2 and rclpy.ok():
                rclpy.spin_until_future_complete(self.node, result_fut, timeout_sec=0.5)
            raise RuntimeError(f'{what}: timeout ({timeout:.0f}s)')
        result = result_fut.result()
        status = getattr(result, 'status', None)
        if status != GoalStatus.STATUS_SUCCEEDED:
            res = getattr(result, 'result', None)
            code, msg = getattr(res, 'error_code', None), getattr(res, 'error_msg', '')
            raise RuntimeError(f'{what}: Nav2 실패 (status={status} · error {code}: {msg})')
        self._progress(f'{what}: 도착')

    def _wait_map(self, timeout: float) -> bool:
        """/map(transient_local)을 한 번이라도 받는지 — 없으면 global costmap이 'no map received'로 멈춘다."""
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        got = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = self.node.create_subscription(OccupancyGrid, '/map', got.append, qos)
        try:
            end = time.monotonic() + timeout
            while not got and time.monotonic() < end and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
        finally:
            self.node.destroy_subscription(sub)
        if got:
            self._map = got[-1]
        return bool(got)

    def _save_map(self, folder: Path) -> None:
        """받은 /map → folder/map.pgm + map.yaml (map_server 형식) — 리포트가 지도 위에 궤적을 그린다."""
        m = self._map
        if m is None:
            return
        w, h = m.info.width, m.info.height
        rows = [bytes((254 if v == 0 else 205 if v < 0 else 0 if v >= 65 else 254) for v in m.data[r * w:(r + 1) * w])
                for r in reversed(range(h))]  # PGM은 위쪽 행부터
        (folder / 'map.pgm').write_bytes(f'P5\n{w} {h}\n255\n'.encode() + b''.join(rows))
        o = m.info.origin.position
        (folder / 'map.yaml').write_text(
            f'image: map.pgm\nmode: trinary\nresolution: {m.info.resolution}\n'
            f'origin: [{o.x:.4f}, {o.y:.4f}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')

    def _plan_path(self, start: dict, goal: dict, frame_id: str) -> tuple[bool, str]:
        """planner_server에 start→goal 경로를 한 번 물어본다 (주행 없음)."""
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import ComputePathToPose
        from rclpy.action import ActionClient

        def pose(p):
            m = PoseStamped()
            m.header.frame_id = frame_id
            yaw = math.radians(float(p.get('yaw_deg', 0.0)))
            m.pose.position.x, m.pose.position.y = float(p['x']), float(p['y'])
            m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
            return m
        cli = ActionClient(self.node, ComputePathToPose, '/compute_path_to_pose')
        try:
            if not cli.wait_for_server(timeout_sec=5.0):
                return False, '/compute_path_to_pose 없음'
            g = ComputePathToPose.Goal()
            g.start, g.goal, g.use_start = pose(start), pose(goal), True
            send = cli.send_goal_async(g)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=5.0)
            handle = send.result() if send.done() else None
            if handle is None or not handle.accepted:
                return False, 'goal 거부/응답 없음'
            fut = handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=15.0)
            if not fut.done():
                return False, '15초 안에 응답 없음'
            res = fut.result()
            if res.status != GoalStatus.STATUS_SUCCEEDED or not res.result.path.poses:
                return False, f'error {getattr(res.result, "error_code", "?")}: {getattr(res.result, "error_msg", "")}'
            return True, f'{len(res.result.path.poses)}점'
        finally:
            cli.destroy()

    def _preflight(self, plan: dict, start: dict) -> None:
        """녹화 전에 주행이 될 상태인지 — 안 되면 복구 동작만 녹화돼 A/B가 무의미해진다 (2026-11-07)."""
        self._progress('사전 점검: Nav2 수명주기 · /map · 경로 계획')
        names = {n for n, _ in self.node.get_node_names_and_namespaces()}
        nodes = LIFECYCLE_NODES + (('map_server',) if 'map_server' in names else ())
        bad = [f'{n}={self.bus.lifecycle_state(n, 2.0) or "응답 없음"}' for n in nodes
               if self.bus.lifecycle_state(n, 2.0) != 'active']
        if bad:
            raise RuntimeError('active가 아닌 Nav2 노드: ' + ', '.join(bad) +
                               ' — `run-cli node activate <이름>` 후 다시 (기동 중 응답 시간 초과로 멈춘 경우)')
        if not self._wait_map(5.0):
            raise RuntimeError('/map 없음 — fake_nvblox(SLAM) 또는 저장 지도 확인')
        prev = start
        for i, goal in enumerate(plan['route'], 1):
            gap = math.hypot(goal['x'] - prev['x'], goal['y'] - prev['y'])
            if gap < MIN_GOAL_GAP_M:
                raise RuntimeError(f'goal {i}가 앞 지점과 {gap:.2f} m — 바로 도착 처리돼 컨트롤러가 안 돎 '
                                   f'(command.json start·route 확인, {MIN_GOAL_GAP_M} m 이상)')
            ok, why = self._plan_path(prev, goal, plan['frame_id'])
            if not ok:
                raise RuntimeError(f'경로 계획 실패 (goal {i}: ({goal["x"]:.2f}, {goal["y"]:.2f})) — {why}')
            prev = goal
        self._progress('사전 점검: 통과')

    @staticmethod
    def _fmt(p: dict) -> str:
        return f'({p["x"]:.2f}, {p["y"]:.2f}, {p["yaw_deg"]:.0f}°)'

    def _is_sim(self) -> bool:
        """테스트베드(faker)인가 — fake_opencr가 /cmd/fake_opencr/set_pose를 듣고 있으면 텔레포트가 된다."""
        try:
            if self.node.count_subscribers('/cmd/fake_opencr/set_pose') > 0:
                return True
        except Exception:
            pass
        try:
            return any(n == 'fake_opencr' for n, _ in self.node.get_node_names_and_namespaces())
        except Exception:
            return False

    def _ask_place(self, start: dict, what: str) -> None:
        """실기에서 자동 복귀가 안 됐을 때 — 사람이 출발점에 놓고 Enter. TUI가 아니면(명령줄) 멈추지 않고 실패."""
        if not hasattr(self.prompt, 'modal_key') or not sys.stdin.isatty():
            raise RuntimeError(f'{what}: 시작점 자동 복귀 실패 — 로봇을 {self._fmt(start)}에 놓고 다시 실행하세요')
        key = self.prompt.modal_key(f'{what} — 로봇을 출발점으로', [
            f'Nav2(DWB) 자동 복귀가 안 됐습니다. 로봇을 출발점 {self._fmt(start)}에 놓아 주세요.',
            '(텔레오퍼로 몰거나 손으로 옮긴 뒤 — 방향도 맞춤)', '',
            'Enter = 놓았음, 이 leg 시작 · s = 이 leg 건너뛰기 · q = 전체 중단'])
        if key in ('s', 'S'):
            raise LegStopped(f'{what}: 출발점에 못 놓아 건너뜀 (s)')
        if key in ('q', 'Q') or key is None:
            raise AbortAll(f'{what}: 출발점에 못 놓아 전체 중단')

    def _clear_costmaps(self) -> None:
        """leg 사이 costmap 지우기 — 앞 leg가 남긴 표시가 다음 leg 판단에 섞이지 않게."""
        try:
            from nav2_msgs.srv import ClearEntireCostmap
        except ImportError:
            return
        for name in ('/global_costmap/clear_entirely_global_costmap', '/local_costmap/clear_entirely_local_costmap'):
            cli = self.node.create_client(ClearEntireCostmap, name)
            try:
                if cli.wait_for_service(timeout_sec=2.0):
                    fut = cli.call_async(ClearEntireCostmap.Request())
                    rclpy.spin_until_future_complete(self.node, fut, timeout_sec=3.0)
            finally:
                self.node.destroy_client(cli)

    def _watch_status(self):
        """leg 동안 FollowPath/status를 센다 — 그 leg가 정말 정책(A)·DWB(B)로 돌았는지 판정용."""
        from std_msgs.msg import Float32MultiArray
        counts = {'n': 0, 'rl': 0, 'reasons': {}}

        def cb(m):
            if len(m.data) < 2:
                return
            counts['n'] += 1
            counts['rl'] += int(m.data[0] > 0.5)
            r = int(m.data[1])
            counts['reasons'][r] = counts['reasons'].get(r, 0) + 1
        return counts, self.node.create_subscription(Float32MultiArray, STATUS, cb, 50)

    @staticmethod
    def _judge(kind: str, counts: dict, error: str, duration: float) -> dict:
        """leg 판정 — valid = 비교에 쓸 수 있나(모드가 맞게 돌았나), arrived = 성능 결과(도착했나)."""
        n, rl = counts['n'], counts['rl']
        ratio = rl / n if n else 0.0
        reasons = {REASONS.get(k, str(k)): v for k, v in sorted(counts['reasons'].items())}
        why = ''
        if n == 0:
            why = '컨트롤러가 한 번도 안 돎 (경로 없음 — Nav2 로그 확인)'
        elif kind == 'A' and ratio < RL_MIN_RATIO:
            why = f'정책 실행 {ratio:.0%} (바이패스: ' + ', '.join(f'{k} {v}' for k, v in reasons.items() if k != '정책') + ')'
        elif kind == 'B' and rl:
            why = f'B인데 정책이 {rl}회 돎'
        arrived = not error
        return {
            'leg': kind, 'arrived': arrived, 'error': error, 'duration_s': round(duration, 2),
            'controller_ticks': n, 'rl_ticks': rl, 'rl_ratio': round(ratio, 3), 'reasons': reasons,
            'valid': not why, 'invalid_reason': why,
            'verdict': ('도착' if arrived else '실패') + (' · 유효' if not why else f' · 무효 — {why}'),
        }

    def _leg(self, kind: str, r: int, plan: dict, start: dict, group_dir: Path,
             label: str, first: bool) -> tuple[str, dict]:
        """leg 하나: 시작 자세로 → (시뮬) 텔레포트로 같은 자세 → costmap 지우기 → 모드 → 녹화·주행 → 판정."""
        drive = (plan['move_to_start'] and plan['start'] is not None) if first else plan['return_to_start']
        sim = self._is_sim()
        placed = not drive
        if drive:
            self.toggle(False)  # 위치 맞추기는 측정이 아님 — 늘 inner DWB로 (정책이 못 가면 측정 전부터 막힘)
            self._progress(f'{kind}{r} 전 시작점으로 복귀: Nav2(DWB)가 자동 주행 — {self._fmt(start)}')
            try:
                self.nav_goal(start, plan['frame_id'], plan['goal_timeout_s'], f'{kind}{r} 전 시작점 복귀(DWB)')
                placed = True
            except (RuntimeError, LegStopped) as exc:
                print(f'WARN: 시작점 복귀 실패 — {exc}', file=sys.stderr)
        if sim:
            self._reset_sim_pose(start, plan['frame_id'])  # 시뮬: 텔레포트로 A·B 출발점을 정확히 같게
            self._progress(f'{kind}{r} 출발점 맞춤 (시뮬 텔레포트): {self._fmt(start)}')
        elif not placed:
            self._ask_place(start, f'{kind}{r}')  # 실기: 자동 복귀 실패 → 사람이 옮김 (s 건너뛰기 · q 전체 중단)
        self._clear_costmaps()
        ok, msg = self.toggle(kind == 'A')
        if not ok:
            raise RuntimeError(f'{kind}: 정책 {"ON" if kind == "A" else "OFF"} 실패 — {msg}')
        counts, sub = self._watch_status()
        poses: list[tuple[float, float, float, float]] = []  # (t, x, y, yaw) map 기준 — odom은 텔레포트로 어긋남
        t_rec = time.monotonic()

        def sample():
            try:
                poses.append((time.monotonic() - t_rec, *self._lookup_pose(plan['frame_id'])))
            except Exception:
                pass
        timer = self.node.create_timer(0.1, sample)
        rec = self.start(f'{label}_r{r}_{kind}', out=str(group_dir / f'{time.strftime("%Y%m%d_%H%M%S")}_{kind}'))
        self._write_plan_csv(rec['bag'], start, plan)
        self._progress(f'{kind}{r} 녹화 ({"정책" if kind == "A" else "DWB"}): {rec["bag"]}')
        t0, err, stopped, abort = time.monotonic(), '', False, None
        try:
            self._run_route(plan, f'{kind}{r}')
        except LegStopped as exc:
            err, stopped = str(exc), True
            print(f'중지: {exc} — 다음 leg로', file=sys.stderr)
        except AbortAll as exc:
            err, stopped, abort = str(exc), True, exc
            print(f'중단: {exc}', file=sys.stderr)
        except RuntimeError as exc:
            err = str(exc)
            print(f'FAIL: {exc}', file=sys.stderr)
        finally:
            self.stop()
            self.node.destroy_subscription(sub)
            self.node.destroy_timer(timer)
        if poses:
            with open(Path(rec['bag']) / 'pose.csv', 'w') as f:
                f.write('t,x,y,yaw\n' + ''.join(f'{t:.3f},{x:.4f},{y:.4f},{yaw:.4f}\n' for t, x, y, yaw in poses))
        result = self._judge(kind, counts, err, time.monotonic() - t0)
        result['goals_reached'], result['goals_total'] = getattr(self, '_reached', 0), len(plan['route'])
        if stopped:  # 사람이 멈춘 leg — 성능 결과가 아니므로 비교에서 뺀다
            result.update(stopped=True, valid=False, invalid_reason='사용자가 중지',
                          verdict=f'중지됨 · 목표 {result["goals_reached"]}/{result["goals_total"]}')
        (Path(rec['bag']) / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=1))
        self._progress(f'{kind}{r}: {result["verdict"]}')
        if abort is not None:
            raise AbortAll(str(abort), rec['bag'], result)
        return rec['bag'], result

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
        self._reached = 0  # 도착한 goal 수 — leg 판정에 남긴다
        for i, pose in enumerate(route, 1):
            self.nav_goal(pose, plan['frame_id'], plan['goal_timeout_s'],
                          f'{label} goal {i}/{len(route)}')
            self._reached = i
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
        # 단발 실행에선 discovery 전에 체크하면 첫 수신 없이 inf로 실패한다 — 첫 /state까지 최대 10초 대기.
        end = time.monotonic() + 10.0
        while self.bus.state.state_last_update <= 0 and time.monotonic() < end and rclpy.ok():
            self.bus._poll(0.2)
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

        bags: list[str] = []
        labels: list[str] = []
        results: list[dict] = []
        skipped: list[tuple[str, str]] = []
        ok_all = True
        try:
            if self._is_sim():
                self._reset_sim_pose(start, plan['frame_id'])
            self._preflight(plan, start)
            # 그룹 폴더: ab/<노드>/<guide 시작시각>/ — leg bag + result.json + report.png를 한데 묶음.
            group_dir = BAG_DIR / time.strftime('%Y%m%d_%H%M%S')
            n = 1
            while group_dir.exists():
                n += 1
                group_dir = BAG_DIR / f'{time.strftime("%Y%m%d_%H%M%S")}_{n}'
            group_dir.mkdir(parents=True, exist_ok=True)
            self._save_map(group_dir)
            for r in range(1, plan['repeats'] + 1):
                pair_ok = True
                for kind in ('A', 'B'):
                    try:
                        bag, res = self._leg(kind, r, plan, start, group_dir, label, first=(r == 1 and kind == 'A'))
                    except LegStopped as exc:  # 녹화 전에 건너뜀 (실기: 출발점에 못 놓음) — 요약에만, 리포트엔 없음
                        skipped.append((f'{kind}{r}', str(exc)))
                        continue
                    bags.append(bag)
                    labels.append(f'{kind}{r}')
                    results.append(res)
                    pair_ok &= res['valid'] or res.get('stopped', False)
                if not pair_ok:  # 모드가 틀린 leg가 있으면 반복해도 같은 문제 — 멈추고 원인부터
                    ok_all = False
                    break
        except AbortAll as exc:  # q — 그때까지 녹화한 leg는 리포트로 남긴다
            if len(exc.args) >= 3:
                bags.append(exc.args[1])
                labels.append(f'{exc.args[2]["leg"]}{(len(bags) + 1) // 2}')  # A·B 순서라 bag 수로 회차가 나옴
                results.append(exc.args[2])
            print(f'중단: {exc.args[0]}', file=sys.stderr)
            ok_all = False
        except (RuntimeError, OSError) as exc:
            print(f'FAIL: {exc}', file=sys.stderr)
            ok_all = False
        finally:
            if self.current():
                self.stop()
            ok, msg = self.toggle(True)  # 기본 = 정책 ON 복원
            if not ok:
                print(f'WARN: 정책 ON 복원 실패 — {msg}', file=sys.stderr)

        if results or skipped:
            print('\nA/B 결과')
            for lab, res in zip(labels, results):
                print(f'  {lab}: {res["verdict"]} · {res["duration_s"]:.1f}s · 정책 {res["rl_ratio"]:.0%}'
                      + (f' · {res["error"]}' if res['error'] else ''))
            for lab, why in skipped:
                print(f'  {lab}: 건너뜀 — {why}')
        ok_all &= all(res['valid'] for res in results) and bool(results) and not skipped
        for bag, label in zip(bags, labels):
            if self.report([bag], [label], str(Path(bag) / 'report.png')) != 0:
                ok_all = False
        if len(bags) >= 2:
            out = str(group_dir / 'compare.png')
            if self.report(bags, labels, out) != 0:
                ok_all = False
        return 0 if ok_all else 1
