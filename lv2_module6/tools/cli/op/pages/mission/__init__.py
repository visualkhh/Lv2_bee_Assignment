"""미션 메뉴 — mission_manager에 시작·중지(/cmd/mission)를 보내고, 중앙에 실기 시험 디버그 한 화면.

중앙(mission_lines — `run-cli mission status`와 공용)이 읽는 것 (operator_tool._wire_mission · StateStore):
  /mission/state · /task/status · /task/sub_goals · /coordinator/basket · /mission/signal_status · /skill_context
  · /controller_server/FollowPath/status · /collision_monitor_state · Nav2 feedback · TF map→odom · /diagnostics_agg
"""
from __future__ import annotations

import argparse
import time

from ...defs import STAY
from ...menus import ActionItem, GoMenu, Menu, MenuItem
from ...terminal import BOLD, GREEN, RED, YELLOW
from ...bars.state.nav_state import collision_text, controller_text

ARM_PHASES = {0: '정지(위)', 1: '내려가기', 2: '안정', 3: '쥐기/놓기', 4: '올리기', 5: '대기'}  # ml/SkillContext phase
FRESH_S = 3.0   # 이보다 오래된 live 값은 '(N s 전)'을 붙임


def _age(t: float) -> str:
    a = time.monotonic() - t
    return '' if a < FRESH_S else f' ({a:.0f}s 전)'


def mission_lines(st, term) -> list[str]:
    """미션 화면 중앙 · `run-cli mission status` 공용."""
    p, now = term.paint, time.monotonic()
    out = []
    m = st.mission
    if m is None:
        out.append('[미션] - (/mission/state 없음 — mission_manager 확인)')
    else:
        color = RED + BOLD if m.phase == 'ERROR' else GREEN if m.phase == 'DONE' else YELLOW
        out.append(f'[미션] {p(m.phase, color)} {now - st.mission_since:.0f}s — {m.note or "-"}')
        sig = f'R{m.count_r} Y{m.count_y} G{m.count_g}' if (m.count_r or m.count_y or m.count_g) else '-'
        out.append(f' 바코드 {m.barcode or "-"} · 신호 {sig} · 배정 {m.order or "-"} · 놓음 {m.placed}/{m.total}')
        for r in list(m.results)[-6:]:
            out.append(f'  결과 {r}')
        if m.error:
            out.append(' ' + p(f'오류: {m.error}', RED + BOLD))
    task = st.task
    if task is None:
        out.append('[skill] - (/task/status 없음)')
    else:
        plan_id, step, state, reason = task
        skill = st.current_skill()
        name = f'{skill[0]} → {skill[1] or "-"}' if skill else '(sub_goals 모름)'
        total = len(st.sub_goals[1]) if st.sub_goals[0] == plan_id else 0
        label = {0: p('RUNNING', YELLOW), 1: p('SUCCEEDED', GREEN), 2: p('FAILED', RED + BOLD)}.get(state, str(state))
        out.append(f'[skill] step {step}{f"/{total}" if total else ""} {name} · {label} '
                   f'{now - st.task_since:.0f}s{" · " + p(reason, RED) if reason else ""}  (plan {plan_id})')
    out.append(f'[실패 이력 {len(st.failures)}/10]' + ('' if st.failures else ' 없음'))
    for at, _, src, reason in reversed(st.failures):
        out.append(' ' + p(f'{at} {src}: {reason}', RED))
    b = st.basket
    if b is None and m is not None and m.tray:   # coordinator 장부를 못 받으면 /mission/state의 사본
        out.append(f'[적재함] {" ".join(m.tray)}')
    if b is not None:
        cells = ' '.join(f'{s.slot}:{s.tag if s.tag >= 0 else "?"}{"!" if s.check else ""}' for s in b.slots if s.filled)
        held = f'tag {b.held.tag if b.held.tag >= 0 else "?"} {b.held.color}'.strip() if b.held.filled else '빈손'
        out.append(f'[적재함] {cells or "비었음"} · 손 {held} · 순서 {",".join(map(str, b.order)) or "-"}'
                   + (' · ' + p('! = 확인 필요', YELLOW) if any(s.check for s in b.slots) else ''))
    nav = []
    if st.nav is not None:
        (dist, rec, el), t = st.nav
        nav.append(f'남은 {dist:.2f}m · 회복 {rec} · {el:.0f}s{_age(t)}')
    ctl = controller_text(st)
    if ctl:
        s = st.rl_status
        extra = f' · cloud {s[5]:.1f}s · scan_min {s[6]:.2f}m · v {s[7]:+.2f} w {s[8]:+.2f}' if len(s) > 8 else ''
        nav.append(f'제어기 {p(ctl, GREEN if ctl == "RL" else YELLOW)}{extra}')
    on, action = collision_text(st)
    nav.append(f'충돌감시 {on}' + (' · 지금 ' + p(action, RED + BOLD) if action else ''))
    out.append('[주행] ' + nav[0])
    out += [' ' + n for n in nav[1:]]
    rate = st.rates.get('cuvslam')
    hz = f'{rate.hz():.0f}Hz' if rate and (rate.age() or 99) < 3 else p('끊김/없음', RED)
    if st.map_odom is None:
        jump = 'map→odom 없음'
    elif st.slam_last_jump:
        d, deg, at = st.slam_last_jump
        jump = p(f'map→odom 점프 {st.slam_jumps}회 (최근 {d:.2f}m {deg:.0f}° {at})', YELLOW)
    else:
        jump = p('map→odom 점프 0', GREEN)
    out.append(f'[SLAM] cuVSLAM {hz} · {jump}')
    seen = ' · '.join(f'{k} {v}' for k, v in sorted(st.class_counts.items())) or '없음'
    out.append(f'[보이는 물체] {seen}')
    if st.skill_ctx is not None:
        c, t = st.skill_ctx
        if c.skill:
            out.append(f'[팔 skill] {c.skill} · {ARM_PHASES.get(c.phase, c.phase)} · '
                       f'{"붙듦" if c.holding else "-"} · 보정 {"live" if c.residual_live else "고정"} '
                       f'(run {c.run_id}){_age(t)}')
        else:
            out.append(f'[팔 skill] 없음 (보정 끔){_age(t)}')
    if st.signal is not None:
        s, t = st.signal
        res = p('확정', GREEN) if s.ok else p(s.error or '?', YELLOW)
        out.append(f'[신호 해독] {res} · R{s.red} Y{s.yellow} G{s.green} · 연속 {s.cycles} · {s.raw or "-"}{_age(t)}')
    pct = st.battery_percent()
    if pct is not None and pct < 20:
        out.append(p(f'[전원] 배터리 낮음 {st.battery_summary()}', RED + BOLD))
    bad = sorted(((lv, n, msg) for n, (lv, msg) in st.diagnostics.items() if lv >= 1), reverse=True)
    out.append(f'[진단 WARN/ERROR {len(bad)}]' + ('' if bad else ' 없음'))
    for lv, n, msg in bad[:6]:
        out.append(' ' + p(f'{n.split(": ", 1)[-1]}: {msg}', RED if lv >= 2 else YELLOW))
    return out


class MissionCmdItem(MenuItem):
    def __init__(self, app, key: str, cmd: str, label: str, ask: str):
        super().__init__(app)
        self.key, self._cmd, self._label, self._ask = key, cmd, label, ask

    def label(self):
        return self._label

    def run(self):
        if not self.ui.confirm(self._ask, default=False, body=[f'현재 단계: {self._phase()}']):
            return STAY
        self.bus.cmd_mission(self._cmd)
        return STAY

    def _phase(self):
        m = self.state.mission
        return m.phase if m else '-'


# 세부 명령 — skill마다 (키, 이름, target 설명, 예). 정의: documents/project/ros/05-arm.md 3절
SKILLS = [
    ('1', 'navigate', '이동', ['물체 id 또는 "x,y[,yaw]" (map 좌표)'], '1.5,0.8,0'),
    ('2', 'approach', '박스 앞 직선 접근', ['"x,y,yaw" (map) — 박스 앞 마지막 몇 cm를 저속 직선'], '1.5,0.8,0'),
    ('3', 'pose', '팔 자세', ['stow · carry_left · carry_right · scan · look'], 'stow'),
    ('4', 'inspect', '퍽 보기 (태그 읽기)', ['퍽 id 또는 "x,y,z"'], '5'),
    ('5', 'pick', '집기', ['퍽 id (/tracked_objects)'], '5'),
    ('6', 'place', '놓기', ['장소 상자 id 또는 "x,y,z"'], '14'),
    ('7', 'load', '적재함에 싣기', ['칸 0~11 또는 next'], 'next'),
    ('8', 'unload', '적재함에서 꺼내기', ['칸 0~11 또는 last'], 'last'),
    ('9', 'stand', '누운 퍽 세우기', ['누운 퍽 id'], '5'),
    ('0', 'stand_pick', '세운 퍽 다시 집기', ['직전 stand의 퍽 id (기록용)'], '5'),
]


class SkillItem(MenuItem):
    def __init__(self, app, key: str, skill: str, desc: str, hint: list[str], example: str):
        super().__init__(app)
        self.key, self._skill, self._desc, self._hint, self._example = key, skill, desc, hint, example

    def label(self):
        return f'{self._skill} — {self._desc}'

    def run(self):
        m = self.state.mission
        busy = m is not None and m.phase not in ('IDLE', 'DONE', 'ERROR')
        target = self.ui.modal_input(
            f'{self._skill} — {self._desc}',
            self._hint + [f'예: {self._example}'] +
            ([f'주의: 미션이 {m.phase} 단계 — 같은 coordinator를 쓰니 먼저 미션 중지'] if busy else []),
            'target>', validate=lambda t: None if t.strip() else 'target이 비었습니다')
        if target is None:
            return STAY
        target = target.strip()
        if not self.ui.confirm(f'{self._skill} "{target}" 을(를) 실행할까요?', default=False,
                               body=['로봇이 움직입니다.', f'조작 모드: {self.state.operation or "-"}']):
            return STAY
        if self.state.operation != 'AUTO':
            if not self.ui.confirm('AUTO로 전환할까요?', default=True,
                                   body=['coordinator는 AUTO에서만 계획을 받습니다.']):
                return STAY
            self.bus.cmd_mode(argparse.Namespace(mode='auto'))
        self.bus.cmd_skill(self._skill, target)
        return STAY


class SkillStopItem(MenuItem):
    key = 's'

    def label(self):
        return '진행 중 전부 취소 (정지)'

    def run(self):
        self.bus.cmd_skill('stop', '')
        return STAY


class SkillMenu(Menu):
    title = '세부 명령 (skill 직접)'

    def items(self):
        return [SkillItem(self.app, *s) for s in SKILLS] + [SkillStopItem(self.app)]

    def center(self):
        return ['[세부 명령]', f' 조작: {self.state.operation or "-"}',
                ' mission_manager가 쓰는 길(/task/sub_goals → coordinator → /skills/<이름>)로 한 단계만 보냄',
                ' 미션이 돌고 있으면 먼저 중지 · AUTO에서만 실행',
                f' task: {self.state.task_status or "-"}']


class BasketClearItem(MenuItem):
    key = '4'

    def label(self):
        return '적재함 장부 비우기 (/coordinator/basket_clear)'

    def run(self):
        if not self.ui.confirm('적재함 장부를 비울까요?', default=False,
                               body=['적재함을 손으로 정말 비웠을 때만 — 장부만 지웁니다.']):
            return STAY
        ok, res = self.bus.call_service('/coordinator/basket_clear', 'std_srvs/srv/Trigger', {})
        print(f'basket_clear: {res if ok else "실패 — " + str(res)}')
        return STAY


class MissionMenu(Menu):
    title = '미션'

    def items(self):
        return [MissionCmdItem(self.app, '1', 'start', '미션 시작', '미션을 시작할까요?'),
                MissionCmdItem(self.app, '2', 'abort', '미션 중지 (abort)', '미션을 중지할까요? (coordinator도 정지)'),
                GoMenu(self.app, '3', '세부 명령 (navigate · pick · place … skill 직접)', SkillMenu(self.app)),
                BasketClearItem(self.app),
                ActionItem(self.app, '5', '실패 이력 지우기', self.state.failures.clear)]

    def center(self):
        return mission_lines(self.state, self.term)
