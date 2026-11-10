"""TeleopDialog — 텔레오퍼 창 (Dialog 하위 클래스). bus.cmd_teleop이 띄운다."""
from __future__ import annotations

import threading
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
    속도는 방향 × 설정값이라 달리는 중에 바꿔도 바로 반영. 명령은 *그리기 루프와 따로* 별도 스레드가 SEND_HZ(20 Hz)로 다시 보낸다
    (manual_control cmd_timeout 0.5초). 예전엔 on_tick이 상단바·지도를 다시 그린 뒤에 보내서, SLAM으로 CPU가 포화되면 그리기가 0.5초 가까이
    걸려 명령이 0.6초마다(1.6 Hz) 나가 만료 정지(watchdog)가 끼며 가다 섰다 (실기 2026-11-10 녹화 613 ms 간격).
    안전: 메인 루프가 HEARTBEAT_S 넘게 멈추면(UI 정지·확인창 대기) 스레드는 정지(0)를 보낸다 — 멈춘 UI가 명령을 계속 내지 않게.
    on_mark가 있으면 g/Enter에 로봇을 세우고 on_mark() — 반환 문자열을 창 맨 아래에 (A/B 경로 goal 찍기)."""

    SEND_HZ = 20.0       # 명령 — 별도 스레드 (그리기와 무관)
    draw_period = 0.2    # 박스(속도·전류) 5 Hz — 사람이 읽는 값. 키를 누르면 invalidate()로 즉시
    BAR_PERIOD = 1.0     # 창 뒤 상단바·중앙 1 Hz
    HEARTBEAT_S = 1.0   # 메인 루프(on_tick)가 이보다 오래 안 돌면 정지 명령

    def __init__(self, bus, linear: float, angular: float, on_mark=None):
        super().__init__(bus.prompt, '텔레오퍼 → /cmd/manual/cmd_vel')
        self.bus, self.term, self.state = bus, bus.term, bus.state
        self.on_mark = on_mark
        self.move = (0, 0)  # (전후, 회전) 방향
        self.lin_speed, self.ang_speed = min(linear, MAX_LIN), min(angular, MAX_ANG)
        self.last_bar = 0.0
        self._beat = time.monotonic()      # 메인 루프가 마지막으로 돈 시각 (스레드가 본다)
        self._stop = threading.Event()
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

    def run(self):
        """명령 전송 스레드를 띄우고 대화상자를 돌린다 — 어떻게 끝나든 스레드를 멈추고 정지 명령을 보낸다."""
        self._stop.clear()
        self._beat = time.monotonic()
        sender = threading.Thread(target=self._sender, name='teleop-send', daemon=True)
        sender.start()
        try:
            return super().run()
        finally:
            self._stop.set()
            sender.join(timeout=1.0)
            self.move = (0, 0)
            self.bus._send_velocity(0.0, 0.0)

    def _sender(self) -> None:
        period = 1.0 / self.SEND_HZ
        while not self._stop.wait(period):
            alive = time.monotonic() - self._beat < self.HEARTBEAT_S
            self.bus._send_velocity(*(self._command() if alive else (0.0, 0.0)))

    def on_tick(self) -> None:
        now = time.monotonic()
        self._beat = now
        if now - self.last_bar >= self.BAR_PERIOD:  # 창 뒤 상단 상태바·중앙(가리지 않은 줄) 갱신 — 그리기가 무거우니 느리게 (명령은 스레드가 따로 보냄)
            self.ui.refresh_bar()
            self.ui.refresh_center(self._geom)
            self.last_bar = now
        self._beat = time.monotonic()      # 그리기가 끝난 뒤에도 한 번 — 그리기에 걸린 시간은 '멈춤'이 아니다
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
            self.invalidate()  # 확인창이 지나간 자리를 다시
            return CONTINUE
        k = TELEOP_ALIAS.get(k, k)
        self.invalidate()  # 키 반응은 바로 화면에
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
