"""controller_server — Nav2 FollowPath(ml::RlNavController)의 RL/DWB 상태.

플러그인이 매 제어 주기 `/controller_server/FollowPath/status`를 발행한다.
  [rl_active, reason, policy_loaded, enable, cloud_ok, cloud_age, scan_min, v, w, inner_kind]
  reason: 0=RL, 1=모델 없음/로드 실패, 2=enable=false, 3=점구름/스캔 없음·stale, 4=추론 실패
  inner_kind: 1=DWB, 2=MPPI, 3=RPP, 4=RotationShim

status 토픽이 없는 구버전 플러그인이면 debug/cmd_vel_nav Hz로 추정한다.
"""
from __future__ import annotations

import json
import time

from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32MultiArray

from ...defs import STAY
from ...terminal import DIM
from ...menus import ActionItem, GoMenu, InfoItem, Menu, MenuItem
from .feature import NodeFeature

LIVE = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
REASONS = {
    0: '',
    1: '모델 없음/로드 실패',
    2: 'enable=false',
    3: '점구름/스캔 없음·stale',
    4: '추론 실패',
}
INNER = {1: 'DWB', 2: 'MPPI', 3: 'RPP', 4: 'RotationShim'}

# controller_server(FollowPath)가 제공하는 서비스 중 CLI에서 자주 쓰는 것 — 라벨·기본 요청.
# 여기 없는 서비스도 discovery되면 generic JSON 입력으로 호출할 수 있다.
SERVICE_HINTS = {
    '/controller_server/FollowPath/toggle': {
        'label': 'FollowPath 토글 (RL↔DWB)',
        'default': '{"data": true}',
    },
    '/controller_server/FollowPath/set_fallback': {
        'label': '안쪽 controller 바꾸기 (dwb|mppi|rpp|rotation_shim)',
        'default': '{"controller": "mppi"}',
    },
    '/controller_server/set_parameters': {
        'label': '파라미터 설정 (SetParameters)',
        'default': '{"parameters": [{"name": "FollowPath.enable", "value": {"type": 1, "bool_value": false}}]}',
    },
    '/controller_server/get_state': {
        'label': '수명주기 get_state',
        'default': '{}',
    },
}


def _parse_request(text: str) -> dict:
    text = (text or '').strip()
    if not text:
        return {}
    value = json.loads(text)
    if not isinstance(value, dict):
        raise ValueError('JSON 객체({...}) 형태여야 합니다')
    return value


def _validate_request(text: str):
    try:
        _parse_request(text)
        return None
    except ValueError as exc:
        return f'JSON 오류: {exc}'


class ServiceCallItem(MenuItem):
    """서비스 하나 — Enter로 Request JSON 입력 후 호출."""

    def __init__(self, app, name: str, type_name: str, callable_: bool = True,
                 label: str = '', default: str = '{}'):
        super().__init__(app)
        self._name = name
        self._type = type_name
        self._callable = callable_
        self._label = label
        self._default = default

    def label(self):
        tail = '' if self._callable else '  (docker 미지원)'
        shown = self._label or self._name
        return f'{shown}  [{self._type or "타입 ?"}]{tail}'

    def run(self):
        if not self._type:
            print('서비스 타입 정보 없음 — discovery를 기다린 뒤 새로고침하세요')
            return STAY
        body = [f'서비스: {self._name}', f'타입: {self._type}']
        if self._type.endswith('/Trigger') or self._type.endswith('/Empty'):
            body.append('요청 필드 없음')
            request = {}
        else:
            if self._type.endswith('/SetBool'):
                body.append('요청: {"data": true} 또는 {"data": false}')
                default = '{"data": true}'
            else:
                body.append('Request 메시지 필드 JSON — 빈 값 = {}')
                default = self._default or '{}'
            text = self.ui.modal_input('서비스 호출', body, 'request>',
                                       default=default, validate=_validate_request)
            if text is None:
                return STAY
            try:
                request = _parse_request(text)
            except ValueError as exc:
                print(f'JSON 오류: {exc}')
                return STAY
        # 마지막 확인 — Enter만 눌러 바로 날아가지 않게 (기본 아니오)
        body += ['', '요청: ' + json.dumps(request, ensure_ascii=False)]
        if not self._callable:
            body.append('도커 service_action_bag_input 미지원 — 실패할 수 있음')
        if not self.ui.confirm(f'{self._name} 호출할까요?', default=False, body=body):
            return STAY
        ok, result = self.bus.call_service(self._name, self._type, request)
        if ok:
            print(f'[{self._name}] 응답')
            print(json.dumps(result, ensure_ascii=False, indent=2, default=str))
        else:
            print(f'[{self._name}] 실패: {result}')
        return STAY


class ServiceMenu(Menu):
    """노드 서비스 목록 — entries는 그 노드 파일(서비스 카탈로그)이 만들어 넘긴다."""
    title = '서비스'

    def __init__(self, app, node_name: str, entries: list[dict] | None = None):
        super().__init__(app)
        self.node_name = node_name
        self.title = f'서비스 · {node_name}'
        if entries is None:
            entries = [{'name': n, 'type': t, 'callable': c, 'label': '', 'default': '{}'}
                       for n, t, c in self.bus.node_services(node_name)]
        self._entries = entries

    def items(self):
        if not self._entries:
            return [InfoItem(self.app, '서비스 없음 (노드가 안 떠 있거나 discovery 대기)')]
        return [ServiceCallItem(self.app, e['name'], e.get('type', ''), e.get('callable', True),
                                e.get('label', ''), e.get('default', '{}'))
                for e in self._entries]

    def center(self):
        lines = [f'[{self.node_name}]', f' 서비스 {len(self._entries)}개', '',
                 ' Enter: Request 입력 → 호출 · Esc/◀ 뒤로']
        lines.append(self.term.paint(' 도커: service_action_bag_input이 타입이 고정된 서비스만 실제 호출', DIM))
        return lines + self.ui.center_summary()


class ControllerServerFeature(NodeFeature):
    node = 'controller_server'

    def wire(self, tool) -> None:
        tool.create_subscription(
            Float32MultiArray, '/controller_server/FollowPath/status',
            lambda m: tool.state.on_rl_status(m.data), LIVE)
        # status가 없는 구버전에서도 RL/DWB를 추정할 수 있게
        tool.create_subscription(
            Float32MultiArray, '/controller_server/FollowPath/debug',
            lambda _m: tool.state.tick('rl_debug'), LIVE)
        tool.create_subscription(
            TwistStamped, '/cmd_vel_nav',
            lambda _m: tool.state.tick('cmd_vel_nav'), LIVE)

    def services(self, bus, name: str) -> list[dict]:
        """controller_server가 제공하는 서비스 카탈로그 — discovery + 이 노드 전용 힌트."""
        rows = []
        for svc_name, type_name, callable_ in bus.node_services(name):
            hint = SERVICE_HINTS.get(svc_name, {})
            rows.append({
                'name': svc_name,
                'type': type_name,
                'callable': callable_,
                'label': hint.get('label', ''),
                'default': hint.get('default', '{}'),
            })
        return rows

    def actions(self, app, bus, name: str) -> list:
        """노드 메뉴에 붙는 서비스 테스트 항목 — 여기서 콜백(요청·결과)을 정의한다."""
        if name.rstrip('/').split('/')[-1] != self.node:
            return []

        def toggle():
            enabled = None
            for diag_name, values in bus.state.diag_values.items():
                if 'controller_server' in diag_name:
                    enabled = dict(values).get('enable') == 'true'
                    break
            target = (not enabled) if enabled is not None else True
            now = '?' if enabled is None else ('ON' if enabled else 'OFF')
            body = [f'현재: {now}', f'바꿀 상태: {"ON (정책)" if target else "OFF (DWB)"}',
                    '서비스: /controller_server/FollowPath/toggle']
            if not app.ui.confirm('FollowPath.toggle을 보낼까요?', default=False, body=body):
                return
            ok, result = bus.call_service(
                '/controller_server/FollowPath/toggle', 'std_srvs/srv/SetBool', {'data': target})
            if ok:
                print(f'FollowPath.toggle(data={str(target).lower()})')
                print(json.dumps(result, ensure_ascii=False, indent=2, default=str))
            else:
                print(f'실패: {result}')

        def set_fallback():
            now = '?'
            for diag_name, values in bus.state.diag_values.items():
                if 'controller_server' in diag_name:
                    now = dict(values).get('inner', '?')
                    break
            names = {'1': 'dwb', '2': 'mppi', '3': 'rpp', '4': 'rotation_shim'}
            lines = [f' 지금 안쪽 controller: {now}',
                     ' 서비스: /controller_server/FollowPath/set_fallback (ml/srv/SetFallback)',
                     ' 정책 ON 이면 정책이 돌고, OFF(바이패스)일 때 이 controller 가 돈다 — 바꾸는 데 몇 초']
            choice = app.ui.choose('안쪽 controller 고르기', list(names.items()), lines)
            if choice not in names:
                return
            ok, result = bus.call_service(
                '/controller_server/FollowPath/set_fallback', 'ml/srv/SetFallback', {'controller': names[choice]})
            if ok:
                print(f'FollowPath.set_fallback(controller={names[choice]})')
                print(json.dumps(result, ensure_ascii=False, indent=2, default=str))
            else:
                print(f'실패: {result}')

        return [
            GoMenu(app, 'a', '서비스 목록 · 호출 (테스트)',
                   ServiceMenu(app, name, self.services(bus, name))),
            ActionItem(app, 'b', '서비스: FollowPath.toggle (RL↔DWB)', toggle),
            ActionItem(app, 'c', '서비스: FollowPath.set_fallback (안쪽 controller 고르기)', set_fallback),
        ]

    @staticmethod
    def _rates(bus) -> tuple[float, float]:
        dbg = bus.state.rates.get('rl_debug')
        cmd = bus.state.rates.get('cmd_vel_nav')
        return (dbg.hz() if dbg else 0.0), (cmd.hz() if cmd else 0.0)

    def status_lines(self, bus, name: str, summary: bool = False) -> list[str]:
        st = bus.state.rl_status
        age = time.monotonic() - bus.state.rl_status_time if st else None
        dbg_hz, cmd_hz = self._rates(bus)
        fresh = bool(st) and age is not None and age <= 1.0
        try:
            has_status = bus.node.count_publishers('/controller_server/FollowPath/status') > 0
        except Exception:  # FakeRos/ROS 대역처럼 count_publishers가 없는 환경
            has_status = bool(st)

        if summary:
            if fresh:
                inner = INNER.get(int(st[9]) if len(st) > 9 else 0, '기본 제어기')
                mode = 'RL 적용' if st[0] >= 0.5 else f'{inner} 바이패스'
                return [f' RL: {mode} (controller_server)']
            if cmd_hz > 0:
                mode = 'RL 적용' if dbg_hz > 0 else '기본 제어기 바이패스'
                return [f' RL: {mode} (controller_server{"" if has_status else " · status 없음"})']
            return [f' RL: 대기 (controller_server{"" if has_status else " · status 없음"})']

        lines = ['[RL 주행]']
        for diag_name, values in bus.state.diag_values.items():
            if 'controller_server' in diag_name:
                d = dict(values)
                lines.append(
                    f' device {d.get("device", "?")} · {d.get("provider", "?")}'
                    f' · {d.get("runtime", "?")} · enable {d.get("enable", "?")}'
                    f' · active {d.get("active", "?")} · mode {d.get("mode", "?")}'
                    f' · fallback {d.get("inner", "?")}'
                    + (f' · {d.get("device_note")}' if d.get('device_note') else ''))
                if d.get('model'):
                    lines.append(f' model {d["model"]}')
                break
        if not fresh:
            if cmd_hz > 0 and dbg_hz > 0:
                lines.append(f' RL 적용 (status 없음 · debug {dbg_hz:.1f}Hz)')
            elif cmd_hz > 0:
                lines.append(f' 기본 제어기 바이패스 (status 없음 · debug 0Hz / cmd_vel_nav {cmd_hz:.1f}Hz)')
            else:
                lines.append(' 대기 — 주행 중 아님'
                             + (' (status 발행 대기)' if has_status else ' (status 토픽 없음)'))
            if not has_status:
                lines.append(' status 토픽 없음: ml 재빌드 후 controller_server 재시작 필요')
            return lines

        v = list(st) + [0.0] * (10 - len(st))
        rl_active, reason, loaded, enable, cloud_ok, cloud_age, scan_min, v_out, w_out, inner_kind = v[:10]
        inner = INNER.get(int(inner_kind), '기본 제어기')
        mode = 'RL 적용' if rl_active >= 0.5 else f'{inner} 바이패스'
        why = REASONS.get(int(reason), f'reason {int(reason)}')
        lines.append(f' {mode}' + (f' — {why}' if why else ''))
        lines.append(
            f' 모델 {"loaded" if loaded else "none"} · enable {"on" if enable else "off"}'
            f' · cloud {"ok" if cloud_ok else "none"}'
            + (f' ({cloud_age:.2f}s)' if cloud_age >= 0 else ''))
        lines.append(f' debug {dbg_hz:.1f}Hz · cmd_vel_nav {cmd_hz:.1f}Hz · v {v_out:.2f} w {w_out:.2f}')
        if 0.0 < scan_min < 3.49:
            lines.append(f' 전방 최소거리 {scan_min:.2f} m')
        return lines
