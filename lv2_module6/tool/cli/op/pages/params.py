"""노드별 파라미터 — 노드 선택 → 파라미터 목록(현재값 표시)에서 골라 바로 값 설정."""
from __future__ import annotations

import sys

from ..defs import STAY
from .base import InfoItem, Menu, MenuItem


class NodeItem(MenuItem):
    """노드 하나 — Enter가 그 노드의 파라미터 화면."""

    def __init__(self, app, key: str, name: str, ns: str):
        super().__init__(app)
        self.key = key
        self._name = name
        self._ns = ns

    def label(self):
        ns = self._ns if self._ns.endswith('/') else self._ns + '/'
        return f'{ns}{self._name}' if self._ns not in ('/', '') else self._name

    def run(self):
        return NodeParamMenu(self.app, self._name, self._ns).show()


class ParamMenu(Menu):
    title = '파라미터 (노드별)'

    def items(self):
        nodes = self.bus.param_nodes()
        if not nodes:
            return [InfoItem(self.app, '노드 없음 (bringup 확인)')]
        return [NodeItem(self.app, str(i + 1), n, ns) for i, (n, ns) in enumerate(nodes)]

    def center(self):
        return ['[노드를 고르면 파라미터 목록 → 골라서 바로 값 설정]']


class ParamSetItem(MenuItem):
    """파라미터 하나 — 라벨에 현재값 표시, Enter면 바로 값 입력."""

    def __init__(self, app, page, key: str, name: str, current):
        super().__init__(app)
        self.key = key
        self._page = page
        self._name = name
        self._current = current

    def label(self):
        cur = '?' if self._current is None else repr(self._current)
        return f'{self._name} = {cur}'

    def run(self):
        cli = self.bus.param_client(self._page.nodename, self._page.nodens)
        if cli is None or not cli.wait_for_services(timeout_sec=5.0):
            print('거부: 파라미터 서비스 무응답', file=sys.stderr)
            return STAY
        descs = self.bus.param_describe(cli, [self._name])
        if not descs:
            print('거부: 조회 실패 (없는 이름?)', file=sys.stderr)
            return STAY
        if descs[0].read_only:
            print(f'거부: 읽기전용 파라미터입니다: {self._name}', file=sys.stderr)
            return STAY
        from ..consts import _parse_param_text

        def check(text: str) -> str | None:
            try:
                _parse_param_text(text.strip(), descs[0].type)
            except (ValueError, TypeError) as exc:
                return str(exc)
            return None

        raw = self.ui.modal_input(
            '값 설정',
            [self._name, f'현재: {self._current!r}', 'bool/int/double/문자열, 배열은 쉼표 구분'],
            '새 값>', validate=check)
        if raw is None:
            return STAY
        new_val = _parse_param_text(raw.strip(), descs[0].type)
        if not self.ui.confirm(f'{self._name} 값을 바꾸겠습니까?', default=False,
                               body=[f'{self._current!r} → {new_val!r}']):
            return STAY
        print(f'  변경: {self._current!r} → {new_val!r}')
        from rclpy.parameter import Parameter
        try:
            res = self.bus.param_wait(cli.set_parameters([Parameter(self._name, value=new_val)]))
        except Exception as exc:
            print(f'거부: 설정 호출 실패: {exc}', file=sys.stderr)
            return STAY
        if res is None or not getattr(res, 'results', None):
            print('거부: 설정 응답 없음', file=sys.stderr)
            return STAY
        ok = bool(res.results[0].successful)
        print(f'  {"성공" if ok else "실패: " + (res.results[0].reason or "?")}')
        self._page.refresh()
        return STAY


class ConfigViewItem(MenuItem):
    key = 'c'

    def __init__(self, app, page):
        super().__init__(app)
        self._page = page

    def label(self):
        return 'config 기본값 보기'

    def run(self):
        from .. import configref as _cfg
        name = self._bus_nodename()
        if not _cfg.has_yaml():
            path, text = _cfg.raw_text(name)
            if path is None:
                print('거부: config yaml을 못 찾음 (ROS 환경에서 실행하면 파싱 표시)', file=sys.stderr)
                return STAY
            print(f'  파일: {path} (원문 — PyYAML 없음)')
            print(text)
            return STAY
        path, defaults = _cfg.node_defaults(name)
        if path is None:
            print(f'거부: {name}의 config yaml을 못 찾음', file=sys.stderr)
            return STAY
        live = self._live_map()
        print(f'  파일: {path}')
        for k in sorted(defaults):
            lv = live.get(k, '(없음)')
            mark = '' if repr(lv) == repr(defaults[k]) else '  ← 다름'
            print(f'  {k} = {lv!r}  (yaml: {defaults[k]!r}){mark}')
        for k in sorted(set(live) - set(defaults)):
            print(f'  {k} = {live[k]!r}  (yaml 없음 — launch 인자?)')
        return STAY

    def _bus_nodename(self):
        return self._page.nodename

    def _live_map(self):
        cli = self.bus.param_client(self._page.nodename, self._page.nodens)
        if cli is None or not cli.wait_for_services(timeout_sec=5.0):
            return {}
        names = self.bus.param_list_names(cli) or []
        if not names:
            return {}
        from ..consts import _param_value_to_py
        descs = self.bus.param_describe(cli, names) or []
        vals = self.bus.param_get_vals(cli, names) or []
        out = {}
        for i, n in enumerate(names):
            try:
                out[n] = _param_value_to_py(descs[i].type, vals[i]) if i < len(descs) and i < len(vals) else None
            except Exception:
                out[n] = None
        return out


class ConfigResetItem(MenuItem):
    key = 'd'  # r은 새로고침

    def __init__(self, app, page):
        super().__init__(app)
        self._page = page

    def label(self):
        return 'yaml 기본값으로 되돌리기'

    def run(self):
        self.bus.param_act_reset(self._page.nodename, self._page.nodens)
        self._page.refresh()
        return STAY


class NodeParamMenu(Menu):
    title = '파라미터'

    def __init__(self, app, name: str, ns: str):
        super().__init__(app)
        self.nodename = name
        self.nodens = ns
        disp = f'{ns if ns.endswith("/") else ns + "/"}{name}' if ns not in ('/', '') else name
        self.title = f'파라미터 · {disp}'
        self._rows: list | None = None  # [(이름, 현재값)] 캐시 — 진입·설정 때 갱신

    def refresh(self):
        self._rows = None

    def _fetch(self) -> list:
        if self._rows is not None:
            return self._rows
        rows = []
        cli = self.bus.param_client(self.nodename, self.nodens)
        if cli is not None and cli.wait_for_services(timeout_sec=5.0):
            names = self.bus.param_list_names(cli) or []
            if names:
                from ..consts import _param_value_to_py
                descs = self.bus.param_describe(cli, names) or []
                vals = self.bus.param_get_vals(cli, names) or []
                for i, n in enumerate(names):
                    try:
                        v = _param_value_to_py(descs[i].type, vals[i]) if i < len(descs) and i < len(vals) else None
                    except Exception:
                        v = None
                    rows.append((n, v))
        self._rows = rows
        return rows

    def items(self):
        items: list = [ParamSetItem(self.app, self, str(i + 1), n, v)
                       for i, (n, v) in enumerate(self._fetch())]
        if not items:
            items = [InfoItem(self.app, '파라미터 없음 (서비스 무응답?)')]
        return items + [ConfigViewItem(self.app, self), ConfigResetItem(self.app, self)]

    def center(self):
        return [f'[노드] {self.nodename} (ns: {self.nodens or "/"})',
                ' 항목 선택 → 바로 값 설정 (현재값 표시됨)']
