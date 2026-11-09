"""TeleopDialog — 텔레오퍼 창 (Dialog 하위 클래스). bus.cmd_teleop이 띄운다."""
from __future__ import annotations

import time

from std_msgs.msg import Int32MultiArray

from ..terminal import CYAN, DIM, RED
from .dialog import CONTINUE, Dialog

MAX_LIN, MAX_ANG = 0.26, 1.82
# 텔레오퍼 키 → (전후, 회전) 방향. 사선(둘 다 0 아님) = 설정 선속도 그대로 + 반경 DIAG_RADIUS 원호.
# 후진 사선은 회전 부호 반대 (teleop_twist_keyboard와 같음: z = 뒤로 가며 왼쪽으로 휘어짐)
TELEOP_MOVES = {'w': (1, 0), 'x': (-1, 0), 'a': (0, 1), 'd': (0, -1), 's': (0, 0),
                'q': (1, 1), 'e': (1, -1), 'z': (-1, -1), 'c': (-1, 1)}
DIAG_RADIUS = 0.5  # [m] 사선 원호 반경
TELEOP_ALIAS = {'up': 'w', 'down': 'x', 'left': 'a', 'right': 'd', ' ': 's'}  # 방향키·스페이스 = 같은 이동


class TeleopDialog(Dialog):
    """텔레오퍼 창 — w/x/a/d·방향키 이동 · q/e/z/c 사선(원호) · s/space 정지 · -/= 속도± · b 삑 · Esc 종료.
    속도는 방향 × 설정값이라 달리는 중에 바꿔도 바로 반영. 0.1초마다 다시 보냄 (manual_control cmd_timeout 0.5초).
    on_mark가 있으면 g/Enter에 로봇을 세우고 on_mark() — 반환 문자열을 창 맨 아래에 (A/B 경로 goal 찍기)."""

    def __init__(self, bus, linear: float, angular: float, on_mark=None):
        super().__init__(bus.prompt, '텔레오퍼 → /cmd/manual/cmd_vel')
        self.bus, self.term, self.state = bus, bus.term, bus.state
        self.on_mark = on_mark
        self.move = (0, 0)  # (전후, 회전) 방향
        self.lin_speed, self.ang_speed = min(linear, MAX_LIN), min(angular, MAX_ANG)
        self.last_send = self.last_bar = 0.0
        self.last_key, self.key_count = '-', 0
        self.mark_txt = ''

    def _command(self) -> tuple[float, float]:
        linear = self.move[0] * self.lin_speed
        # 사선 = 설정 선속도 그대로 + 반경 DIAG_RADIUS 원호 (ω = v / R → 속도를 바꿔도 휘는 모양이 같음)
        angular = self.move[1] * (self.lin_speed / DIAG_RADIUS if self.move[0] else self.ang_speed)
        return linear, angular

    def content(self) -> list[str]:
        st, term = self.state, self.term
        linear, angular = self._command()
        wl, wr = st.joint_vel.get('wheel_left_joint'), st.joint_vel.get('wheel_right_joint')
        actual = (f'실제 바퀴 좌{wl:+.2f} 우{wr:+.2f} rad/s'
                  if wl is not None and wr is not None else '실제 ? (joint_states 무응답)')
        # 창이 떠 있는 동안 뒤 프레임은 안 그리므로, 전류·온도·배터리는 여기 넣는다
        al, ar = st.joint_effort.get('wheel_left_joint'), st.joint_effort.get('wheel_right_joint')
        cur = (f'전류 L {al * 1000:+.0f} R {ar * 1000:+.0f} mA'
               if al is not None and ar is not None else '전류 ? (effort 없음)')
        tl, tr = st.motor_temp.get('wheel_left_joint', -1), st.motor_temp.get('wheel_right_joint', -1)
        temp = f'온도 L {tl} R {tr}°C' if tl >= 0 and tr >= 0 else '온도 ?'
        err = [n for n in ('wheel_left_joint', 'wheel_right_joint') if st.motor_text.get(n, '정상') not in ('', '정상')]
        err_txt = (' · ' + term.paint('에러 ' + ', '.join(err), RED)) if err else ''
        key_txt = f'키 {self.last_key} ×{self.key_count}' if self.key_count else '키 입력 대기 중'
        lines = [self.title,
                 f'모드: {st.operation or "-"}' + (' · E-STOP' if st.emergency else ''),
                 f'명령 v={linear:+.2f} m/s ω={angular:+.2f} rad/s',
                 actual,
                 f'{cur} · {temp} · 배터리 {st.battery_summary()}{err_txt}',
                 f'{key_txt} · 설정 v {self.lin_speed:.2f} m/s · ω {self.ang_speed:.2f} rad/s',
                 term.paint('w/x/a/d·방향키 이동 · q/e/z/c 사선 · s/space 정지', DIM),
                 term.paint('-/= 속도± · b 삑 · Esc 종료', DIM)]
        if self.on_mark is not None:
            lines += [term.paint('g/Enter = 지금 위치를 goal로 추가', CYAN)] + ([self.mark_txt] if self.mark_txt else [])
        return lines

    def on_tick(self) -> None:
        now = time.monotonic()
        if now - self.last_bar >= 0.5:  # 창 뒤 상단 상태바·중앙(가리지 않은 줄) 갱신
            self.ui.refresh_bar()
            self.ui.refresh_center(self._geom)
            self.last_bar = now
        if now - self.last_send >= 0.1:
            self.bus._send_velocity(*self._command())
            self.last_send = now
        self.bus._poll(0.01)

    def on_key(self, k):
        if k in ('esc', 'eof', '\x03'):
            return 0
        if self.on_mark is not None and k in ('g', 'G', 'enter'):
            self.move = (0, 0)  # 세우고 묻는다 — 확인창 동안 굴러가지 않게
            for _ in range(3):
                self.bus._send_velocity(0.0, 0.0)
                self.bus._poll(0.05)
            self.mark_txt = self.on_mark() or ''
            return CONTINUE
        k = TELEOP_ALIAS.get(k, k)
        self.key_count += 1
        self.last_key = 'space' if k == 's' else k
        if k in TELEOP_MOVES:
            self.move = TELEOP_MOVES[k]
        elif k == 'b':
            self.bus.pub_beep.publish(Int32MultiArray())
        elif k in ('=', '+'):  # 상한 = Waffle Pi 사양
            self.lin_speed, self.ang_speed = min(self.lin_speed + 0.05, MAX_LIN), min(self.ang_speed + 0.3, MAX_ANG)
        elif k in ('-', '_'):
            self.lin_speed, self.ang_speed = max(self.lin_speed - 0.05, 0.05), max(self.ang_speed - 0.3, 0.2)
        return CONTINUE
