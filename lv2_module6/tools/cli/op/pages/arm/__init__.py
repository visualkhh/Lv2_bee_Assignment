"""ArmControlPage — 팔 조작 화면 (Menu 하위). 중앙 = ArmView(관절 막대, 포커스 가능), 왼쪽 = 동작 메뉴.

숫자를 치지 않는다: ↑↓ 관절 · ←→ 목표 ∓2° · , . ∓1° 로 **목표만** 바꾸고, Enter(또는 왼쪽 '적용')를 눌러야
움직인다 — 그때 바꾼 관절의 이동량을 보여 주고 확인(기본 아니오)을 받는다. 한계 범위 밖으로는 목표가 못 간다.
실제 전송은 `Bus.cmd_arm`(set)을 그대로 쓴다 — MANUAL 전환 확인·MAPPING/E-STOP 거절·manual_control 속도 제한이 그대로 적용.
이름 있는 자세(stow · carry_left · carry_right)는 왼쪽 5~7 — skill_pose를 coordinator로 보냄(MoveIt 경로, AUTO에서만 실행 · 확인 거침).
window.navigate('arm') 로 연다.
"""
from __future__ import annotations

import argparse
import math
import sys

from ...views.arm import ArmView
from ...consts import ARM_JOINTS, ARM_LIMITS
from ...defs import MAIN, QUIT, STAY
from ...terminal import RED, YELLOW
from ...menus import ActionItem, Menu

# 이름 있는 자세 — coordinator pose_names(plan.yaml)가 받는 것만. scan은 미션에서 빼는 중이라 안 넣음
POSES = [('5', 'stow', '접기 (주행·시작 자세)'), ('6', 'carry_left', '왼쪽으로 들기 (운반)'), ('7', 'carry_right', '오른쪽으로 들기 (운반)')]
BIG_MOVE = math.radians(30.0)   # 한 관절이 이만큼 넘게 움직이면 확인창에 경고


class ArmControlPage(Menu):
    title = '팔 조작'

    def __init__(self, app):
        super().__init__(app)
        self.view = ArmView(ARM_JOINTS, ARM_LIMITS, self.state.joints.get, self.term.color, extra_key=self._extra_key)
        self.msg = ''

    # ---------------------------------------------------------------- 동작
    def apply(self) -> None:
        """바꾼 관절을 확인 뒤 보낸다 — 확인창에서 아니오(기본)면 아무것도 안 보냄."""
        changes = self.view.changes()
        if not changes:
            self.msg = '바꾼 관절 없음 — ←→로 목표를 먼저 바꾸세요'
            return
        body, big = [], False
        for n, (cur, tgt) in changes.items():
            warn = abs(tgt - cur) > BIG_MOVE
            big = big or warn
            body.append(f'{n}: {math.degrees(cur):+.1f}° → {math.degrees(tgt):+.1f}°  (Δ {math.degrees(tgt - cur):+.1f}°)'
                        + ('  ⚠ 크게 움직임' if warn else ''))
        if big:
            body += ['', self.term.paint('⚠ 30° 넘게 움직이는 관절이 있습니다 — 주변에 걸리는 게 없는지 확인', YELLOW)]
        if not self.ui.confirm('팔을 움직일까요?', default=False, body=body):
            self.msg = '취소 — 보내지 않음'
            return
        with self.ui.capture() as (out, err):
            rc = self.bus.cmd_arm(argparse.Namespace(
                action='set', targets=[f'{n}={tgt:.6f}' for n, (_cur, tgt) in changes.items()], force=False))
        text = (out.getvalue() + err.getvalue()).strip().splitlines()
        if rc == 0:
            self.msg = '보냈음 — 팔이 움직이는 중 (막대의 ●가 ◆로 가는 걸 보세요)'
        else:
            self.msg = self.term.paint('거부/실패: ' + (text[-1] if text else f'rc={rc}'), RED)

    def go_pose(self, name: str) -> None:
        """이름 있는 자세로 — /task/sub_goals에 pose 한 단계 (미션 화면 세부 명령과 같은 길). coordinator는 AUTO에서만 실행."""
        m = self.state.mission
        busy = m is not None and getattr(m, 'phase', 'IDLE') not in ('IDLE', 'DONE', 'ERROR')
        body = ['팔이 MoveIt 경로로 움직입니다 — 주변에 걸리는 게 없는지 확인', f'조작 모드: {self.state.operation or "-"}']
        if busy:
            body.append(self.term.paint(f'⚠ 미션이 {m.phase} 단계 — 같은 coordinator를 쓰니 먼저 미션 중지', YELLOW))
        if not self.ui.confirm(f'팔을 {name} 자세로 움직일까요?', default=False, body=body):
            self.msg = '취소 — 보내지 않음'
            return
        if self.state.operation != 'AUTO':
            if not self.ui.confirm('AUTO로 전환할까요?', default=True, body=['coordinator는 AUTO에서만 계획을 받습니다.']):
                self.msg = '취소 — AUTO가 아니면 자세 명령은 실행되지 않음'
                return
            with self.ui.capture():
                self.bus.cmd_mode(argparse.Namespace(mode='auto'))
        with self.ui.capture() as (out, err):
            rc = self.bus.cmd_skill('pose', name)
        text = (out.getvalue() + err.getvalue()).strip().splitlines()
        self.msg = (f'{name} 보냄 — 결과는 상단 미션 칸 · 미션 화면(9) skill 줄' if rc == 0
                    else self.term.paint('실패: ' + (text[-1] if text else f'rc={rc}'), RED))

    def reset(self) -> None:
        self.view.reset()
        self.msg = '목표를 현재값으로 되돌림'

    def _extra_key(self, k) -> bool:
        if k == 'enter':
            self.apply()
            return True
        return False

    # ---------------------------------------------------------------- 화면
    def items(self):
        return [ActionItem(self.app, '1', '적용 — 바꾼 관절을 움직임 (Enter · 확인 거침)', self.apply),
                ActionItem(self.app, '2', '목표를 현재값으로 되돌리기 (z)', self.reset),
                ActionItem(self.app, '3', '이 관절 목표 +2° (→)', lambda: self.view.on_key('right')),
                ActionItem(self.app, '4', '이 관절 목표 -2° (←)', lambda: self.view.on_key('left'))] + \
               [ActionItem(self.app, k, f'자세: {n} — {d}', lambda n=n: self.go_pose(n)) for k, n, d in POSES]

    def center(self) -> list[str]:
        """메시지·경고를 맨 위에 — 낮은 터미널(24줄)에서 아래가 잘려도 안 가려진다. 관절 5개 = 10줄."""
        cw, _ = self.ui.window.center_size()
        st = self.state
        head = [self.msg] if self.msg else []
        head.append(f'바꾼 관절 {len(self.view.changes())}개 (●현재 ◆목표)' + (' — Enter로 적용' if self.view.changes() else ' — ←→로 목표를'))
        if st.operation != 'MANUAL':
            head.append(self.term.paint(f'조작 모드 {st.operation or "?"} — 적용 때 MANUAL 전환을 물음', YELLOW))
        errors = st.motor_errors()
        if errors:
            head.append(self.term.paint('모터 에러: ' + ', '.join(errors), RED))
        return head + self.view.render(cw)

    def show(self) -> str:
        if not any(n in self.state.joints for n in ARM_JOINTS):
            print('/joint_states에 팔 관절이 없습니다 — bringup/OpenCR 확인', file=sys.stderr)
            return STAY
        result = self.ui.window.run(self.title, self.items, self.center, back=True, main=True,
                                    center_comp=self.view, focus_center=True)
        return result if result in (MAIN, QUIT) else STAY
