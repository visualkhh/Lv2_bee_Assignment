"""수동 조작 패널 — [수동 조작] 버튼이 띄우는 별도 창에 넣는다. 닫아도(hide) 로봇엔 아무것도 안 보낸다.
모드는 prompt를 보내면 자동, 직접 만지면(바퀴·팔) 수동 — 마지막 행동이 모드다.

  [바퀴 조작판]                │ [팔 3D — 관절 마커를 잡아 위아래로 끌기] │ [팔 조작 — 세로 스크롤]
  앞 · 뒤 · 멈춤                │  빈 곳 드래그 = 회전 · Shift+드래그 = 이동 · 휠 = 줌
  좌측 바퀴 앞/뒤 · 우측 바퀴 앞/뒤 │  놓으면 /cmd/manual/arm 으로 보냄 · 충돌 모양 보기 체크
  전체 속도 ▲ · ▼               │  [지금 자세로] · 관절 목표/실제 값
  비상정지 · 상태               │
                                                                       버튼 · 관절 슬라이더(joint1~5 °, gripper mm) · 값

로봇 전체 3D(지금 상태)는 메인 창에 있다.

바퀴: 좌·우 바퀴 방향(-1·0·+1) × 전체 속도[m/s] → 차동 구동 (v, ω)로 바꿔 /cmd/manual/cmd_vel (0.2초마다, 멈추면 0 한 번)
      → control/manual_control → twist_mux(자동보다 우선) → 안전 체인 → diff_drive_controller → 바퀴
팔:   목표(밝게) · 지금(어둡게)을 겹쳐 그린다. 관절 한계(URDF)로 자른다.
지금 상태: 플랜트(fake_opencr)가 돌면 플랜트 값, 아니면 /joint_states.
"""
import math
import sys
import time
import tkinter as tk

import cv2
import numpy as np

import robot_view
from bridge import WHEEL_SEPARATION, current_joints, read_msg, cmd_manual_mode, send, send_arm, send_bool, send_twist

# 마지막 행동이 모드다: 수동 조작을 하면 수동 모드 (명령을 내리면 main.send_command가 자동으로 바꿈).
# state_manager가 같은 값은 무시하므로 중복 발행 무해.
_send_twist, _send_arm = send_twist, send_arm


def send_twist(v, w):
    cmd_manual_mode(True)
    _send_twist(v, w)


def send_arm(target):
    cmd_manual_mode(True)
    _send_arm(target)

VIEW = (420, 360)
SPEED_STEP, SPEED_MAX, SPEED_MIN = 0.02, 0.26, 0.02
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_left_joint']


_shift_keys = set()  # 지금 눌린 Shift 키 — 마우스 이벤트 state만으론 macOS에서 Shift가 안 잡힐 때가 있어 키로도 추적


def track_shift(root):
    """Shift 눌림을 키 이벤트로 추적 (창이 포커스를 잃으면 풂) — 창 하나에 한 번"""
    for k in ('Shift_L', 'Shift_R'):
        root.bind_all(f'<KeyPress-{k}>', lambda e, k=k: _shift_keys.add(k), add='+')
        root.bind_all(f'<KeyRelease-{k}>', lambda e, k=k: _shift_keys.discard(k), add='+')
    root.bind('<FocusOut>', lambda e: _shift_keys.clear(), add='+')


def shift_held(e):
    """마우스 이벤트 때 Shift가 눌려 있나 (state 비트 0x1 또는 추적한 키)"""
    return bool(e.state & 0x1) or bool(_shift_keys)


def to_photo(img):
    import base64
    ok, buf = cv2.imencode('.png', img)
    return tk.PhotoImage(data=base64.b64encode(buf.tobytes())) if ok else None


class ManualPanel(tk.Frame):
    """수동 조작 패널 — 별도 창에 넣는다. show()면 수동 모드, hide()면 바퀴 정지 + 자동 모드."""

    def __init__(self, master, sim, on_close=None, frame_var=None):
        super().__init__(master)
        self.sim, self.on_close = sim, on_close
        self.frame_var = frame_var  # 본창 '관절축 보기'와 공유 (없으면 조작창에서만 씀)
        self.shown = False
        self.model = None
        self.model_col = None         # 충돌 모양 (지도 탭 '충돌 모양 보기'를 켜면 — sim.show_collision)
        self.wheel = [0, 0]           # 좌·우 바퀴 방향
        self.speed = 0.10             # 전체 바퀴 속도 [m/s]
        self.sent_zero = True
        self.target = {}              # 팔 목표 (관절 → 값)
        self.selected = None
        self.drag_from = None
        self.arm_cam = robot_view.OrbitView(target=(0.05, 0, 0.35), dist=0.75, yaw=-2.0, pitch=0.35)

        # --- 명령 (자연어 → 자동 모드)
        top = tk.Frame(self)
        top.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(8, 0))
        tk.Label(top, justify=tk.LEFT, anchor=tk.W, fg='#555',
                 text='말로 시키면 자동 모드 (예: 빨간 퍽을 목표 구역으로 옮겨)\n'
                      '직접 만지면 수동 모드 — 마지막 행동이 모드다').pack(anchor=tk.W)
        cmdrow = tk.Frame(top)
        cmdrow.pack(fill=tk.X, pady=4)
        self.cmd_entry = tk.Entry(cmdrow)
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.cmd_entry.bind('<Return>', lambda _: self.send_command())
        tk.Button(cmdrow, text='보내기', command=self.send_command).pack(side=tk.LEFT, padx=(4, 0))
        self.cmd_status = tk.Label(top, justify=tk.LEFT, anchor=tk.W, fg='#555')
        self.cmd_status.pack(anchor=tk.W)
        self.resp_label = tk.Label(top, justify=tk.LEFT, anchor=tk.W, fg='#1b5e20', font=('Menlo', 10), wraplength=520)
        self.resp_label.pack(anchor=tk.W)
        self.last_sent = None  # (시각, 텍스트) — 보내면 표시, 틱에서 보이는 물체 수와 함께 갱신
        self.last_sent_t = 0.0
        self.mode_label = tk.Label(top, font=('', 13, 'bold'), anchor=tk.W)
        self.mode_label.pack(anchor=tk.W)

        # --- 바퀴 조작판
        left = tk.Frame(self)
        left.pack(side=tk.LEFT, padx=8, pady=8, anchor=tk.N)
        tk.Label(left, text='바퀴', font=('', 13, 'bold')).pack(anchor=tk.W)
        pad = tk.Frame(left)
        pad.pack(pady=4)
        buttons = [
            ('좌측 바퀴 앞', (1, None), 0, 0), ('앞', (1, 1), 0, 1), ('우측 바퀴 앞', (None, 1), 0, 2),
            ('', None, 1, 0), ('멈춤', (0, 0), 1, 1), ('', None, 1, 2),
            ('좌측 바퀴 뒤', (-1, None), 2, 0), ('뒤', (-1, -1), 2, 1), ('우측 바퀴 뒤', (None, -1), 2, 2)]
        for text, d, r, c in buttons:
            if d is None:
                continue
            tk.Button(pad, text=text, width=11, height=2, command=lambda d=d: self.set_wheels(*d)).grid(row=r, column=c, padx=2, pady=2)
        sp = tk.Frame(left)
        sp.pack(pady=4)
        tk.Button(sp, text='전체 속도 ▼', command=lambda: self.change_speed(-SPEED_STEP)).pack(side=tk.LEFT)
        self.speed_label = tk.Label(sp, width=10)
        self.speed_label.pack(side=tk.LEFT)
        tk.Button(sp, text='전체 속도 ▲', command=lambda: self.change_speed(SPEED_STEP)).pack(side=tk.LEFT)
        self.estop = tk.BooleanVar(value=False)
        tk.Checkbutton(left, text='비상정지 (/cmd/e_stop)', variable=self.estop, fg='#c62828',
                       command=lambda: send_bool('/cmd/e_stop', self.estop.get())).pack(anchor=tk.W, pady=4)
        self.info = tk.Label(left, justify=tk.LEFT, font=('Menlo', 10), anchor=tk.W)
        self.info.pack(anchor=tk.W, pady=6)
        tk.Label(left, text='단축키: W/X 앞뒤 · S 멈춤 · Q/Z 좌바퀴앞뒤\nE/C 우바퀴앞뒤 · A/D 회전 · [/] 속도', fg='#666',
                 justify=tk.LEFT).pack(anchor=tk.W, pady=4)

        # --- 팔 3D
        mid = tk.Frame(self)
        mid.pack(side=tk.LEFT, padx=4, pady=8, anchor=tk.N)
        tk.Label(mid, text='매니퓰레이터 — 마커를 위아래로 끌기 · 빈 곳 드래그 회전 · Shift+드래그 이동').pack(anchor=tk.W)
        viewrow = tk.Frame(mid)
        viewrow.pack(anchor=tk.W)
        self.col_var = tk.BooleanVar(value=bool(self.sim.show_collision))
        tk.Checkbutton(viewrow, text='충돌모양', variable=self.col_var,
                       command=self.on_toggle_collision).pack(side=tk.LEFT)
        if self.frame_var is None:
            self.frame_var = tk.BooleanVar(value=False)
        tk.Checkbutton(viewrow, text='관절축', variable=self.frame_var,
                       command=self.on_toggle_frames).pack(side=tk.LEFT)
        self.arm_label = tk.Label(mid, bg='black', cursor='hand2')
        self.arm_label.pack()
        self.arm_label.bind('<ButtonPress-1>', self.on_arm_press)
        self.arm_label.bind('<B1-Motion>', self.on_arm_drag)
        self.arm_label.bind('<ButtonRelease-1>', self.on_arm_release)
        self.arm_label.bind('<MouseWheel>', lambda e: self.on_arm_zoom(1 if e.delta > 0 else -1))
        self.arm_label.bind('<Button-4>', lambda e: self.on_arm_zoom(1))
        self.arm_label.bind('<Button-5>', lambda e: self.on_arm_zoom(-1))

        # --- 팔 조작 (오른쪽, 위아래 스크롤): 버튼 · 관절별 슬라이더 · 목표/실제 값
        right = self.scroll_column(self)
        tk.Label(right, text='팔 조작', font=('', 13, 'bold')).pack(anchor=tk.W)
        row = tk.Frame(right)
        row.pack(anchor=tk.W, pady=2)
        for i, (text, cmd) in enumerate([('지금 자세로', self.reset_target), ('팔 초기화', self.init_arm),
                                         ('그리퍼 열기', lambda: self.set_gripper(0.026)),
                                         ('그리퍼 닫기', lambda: self.set_gripper(0.0)),
                                         ('그리퍼 초기화', self.init_gripper)]):
            tk.Button(row, text=text, width=10, command=cmd).grid(row=i // 2, column=i % 2, padx=2, pady=2, sticky=tk.W)
        # 관절별 슬라이더 — 3D 화면 마커 끌기와 같은 목표(self.target)를 바꾼다. 놓으면 /cmd/manual/arm
        tk.Label(right, text='관절 (놓으면 /cmd/manual/arm)', fg='#666').pack(anchor=tk.W, pady=(6, 0))
        self.slider_box = tk.Frame(right)
        self.slider_box.pack(anchor=tk.W, fill=tk.X)
        self.sliders = {}             # 관절 → (Scale, 단위 배율) — URDF가 오면 만든다 (한계·종류가 거기 있음)
        self.slider_held = None       # 끌고 있는 슬라이더 관절 (그동안 틱이 값을 덮지 않게)
        self.joint_text = tk.Label(right, justify=tk.LEFT, font=('Menlo', 10), anchor=tk.W)
        self.joint_text.pack(anchor=tk.W, pady=6)
        self.photos = []
        self.bind_all('<KeyPress>', self.on_key, add='+')
        # bind_all은 앱 전체('all')에 걸려 창을 닫아도 남는다 → 닫힐 때 떼야 죽은 패널로 키가 가지 않음 (TclError: bad window path name)
        # '<KeyPress>' 통째 바인딩은 이 패널만 씀 (track_shift는 <KeyPress-Shift_L> 등 따로)
        self.bind('<Destroy>', lambda e: self.unbind_all('<KeyPress>') if e.widget is self else None, add='+')
        track_shift(self.winfo_toplevel())
        self.ticks = 0
        self.tick()

    @staticmethod
    def scroll_column(master, width=340):
        """세로 스크롤 열 → 안쪽 Frame (높이보다 길어지면 스크롤 · 휠). 시뮬레이터 본창 카메라 칸도 이걸 씀"""
        outer = tk.Frame(master)
        outer.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4, pady=8)
        canvas = tk.Canvas(outer, highlightthickness=0, width=width)
        bar = tk.Scrollbar(outer, orient=tk.VERTICAL, command=canvas.yview)
        canvas.configure(yscrollcommand=bar.set)
        bar.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        inner = tk.Frame(canvas)
        canvas.create_window((0, 0), window=inner, anchor=tk.NW)
        inner.bind('<Configure>', lambda _: canvas.configure(scrollregion=canvas.bbox('all')))
        wheel = lambda e: canvas.yview_scroll(-1 if (e.delta > 0 or e.num == 4) else 1, 'units')
        # 열 위에 마우스가 있을 때만 휠을 받음 (지도·3D 휠 줌과 안 섞이게)
        outer.bind('<Enter>', lambda _: [canvas.bind_all(s, wheel) for s in ('<MouseWheel>', '<Button-4>', '<Button-5>')])
        outer.bind('<Leave>', lambda _: [canvas.unbind_all(s) for s in ('<MouseWheel>', '<Button-4>', '<Button-5>')])
        return inner

    # ---------- 상태
    def actual(self):
        """지금 관절값 · 바퀴 속도 [rad/s]"""
        return current_joints(self.sim)

    # ---------- 바퀴
    def set_wheels(self, left, right):
        if left is not None:
            self.wheel[0] = left
        if right is not None:
            self.wheel[1] = right

    def change_speed(self, delta):
        self.speed = round(min(SPEED_MAX, max(SPEED_MIN, self.speed + delta)), 2)

    def twist(self):
        vl, vr = self.wheel[0] * self.speed, self.wheel[1] * self.speed
        return (vl + vr) / 2, (vr - vl) / WHEEL_SEPARATION

    # ---------- 팔
    def build_sliders(self):
        """URDF 관절 한계로 슬라이더 — revolute는 도(°), prismatic은 mm로 보여 줌.
        이름 옆 화살표 = 움직이는 방향 (영점 자세의 관절축, 차체 기준): 축 위(z) ↔ 좌우 · 축 옆(y) ↕ 상하 · 축 앞(x) ⟲ 비틀기 · 그리퍼 ↔ 벌림"""
        axes = self.model.joint_frames({})
        for j in ARM_JOINTS:
            if j not in self.model.joints:
                continue
            _, kind, _, lo, hi = self.model.joints[j]
            k = 1000.0 if kind == 'prismatic' else 180.0 / math.pi
            row = tk.Frame(self.slider_box)
            row.pack(fill=tk.X)
            ax = np.abs(axes[j][1])
            arrow = '↔' if kind == 'prismatic' else '↔↕⟲'[[2, 1, 0][int(np.argmax(ax))]]
            tk.Label(row, text=f"{j.replace('_left_joint', '')} ({'mm' if k == 1000.0 else '°'}) {arrow}", width=13,
                     anchor=tk.W).pack(side=tk.LEFT)
            sc = tk.Scale(row, from_=lo * k, to=hi * k, resolution=0.1 if k == 1000.0 else 1, orient=tk.HORIZONTAL,
                          length=220, command=lambda v, j=j, k=k: self.on_slider(j, float(v) / k))
            sc.pack(side=tk.LEFT)
            sc.bind('<ButtonPress-1>', lambda e, j=j: setattr(self, 'slider_held', j))
            sc.bind('<ButtonRelease-1>', self.on_slider_release)
            self.sliders[j] = (sc, k)

    def on_slider(self, joint, value):
        if self.slider_held != joint:  # 틱이 값을 맞춰 넣은 것 (사람이 안 움직임)
            return
        if not self.target:
            self.reset_target()
        self.target[joint] = value
        q, _ = self.actual()
        if q:
            self.render_arm(q)

    def on_slider_release(self, _):
        self.slider_held = None
        send_arm(self.target)

    def sync_sliders(self):
        """슬라이더를 지금 목표에 맞춤 (마커 끌기 · 버튼으로 바뀐 것)"""
        for j, (sc, k) in self.sliders.items():
            if j != self.slider_held and j in self.target and abs(sc.get() - self.target[j] * k) > 1e-6:
                sc.set(self.target[j] * k)

    def reset_target(self):
        q, _ = self.actual()
        self.target = {j: q.get(j, 0.0) for j in ARM_JOINTS}
        send_arm(self.target)

    def init_arm(self):
        """팔 초기화 — joint1~5를 URDF 원점(0.0)으로 (그리퍼는 그대로)"""
        if not self.target:
            self.reset_target()
        for j in ARM_JOINTS:
            if j != 'gripper_left_joint':
                self.target[j] = 0.0
        send_arm(self.target)

    def set_gripper(self, value):
        if not self.target:
            self.reset_target()
        self.target['gripper_left_joint'] = value
        send_arm(self.target)

    def init_gripper(self):
        """그리퍼 초기화 — URDF 원점(관절값 0.0, 그린 그대로)으로"""
        if not self.target:
            self.reset_target()
        self.target['gripper_left_joint'] = 0.0
        send_arm(self.target)

    def on_arm_press(self, e):
        self.drag_from = (e.x, e.y)
        self.selected = None
        if shift_held(e) or not self.model or not self.target:  # Shift = 시점 이동 (관절 안 잡음)
            return
        frames = self.model.joint_frames(self.target)
        best, best_d = None, 16
        for name in ARM_JOINTS:
            if name in frames:
                uv, z = self.arm_cam.project([frames[name][0]], VIEW)
                d = math.dist(uv[0], (e.x, e.y))
                if z[0] > 0 and d < best_d:
                    best, best_d = name, d
        self.selected = best
        self.drag_value = self.target.get(best, 0.0) if best else None

    def on_arm_drag(self, e):
        dx, dy = e.x - self.drag_from[0], e.y - self.drag_from[1]
        if self.selected:
            _, kind, _, lo, hi = self.model.joints[self.selected]
            scale = 0.0002 if kind == 'prismatic' else 0.01  # [m/px] · [rad/px]
            self.target[self.selected] = min(hi, max(lo, self.drag_value - dy * scale))
        elif shift_held(e):  # Shift+드래그 = 시점 이동
            self.arm_cam.pan(dx, dy)
            self.drag_from = (e.x, e.y)
        else:
            self.arm_cam.orbit(dx, dy)
            self.drag_from = (e.x, e.y)
        q, _ = self.actual()
        if self.model and q:
            if not self.target:
                self.target = {j: q.get(j, 0.0) for j in ARM_JOINTS}
            now = time.monotonic()  # 팔 렌더 2장분 — 드래그 중엔 70ms로 솎음 (값은 계속 누적)
            if now - getattr(self, '_adrag_t', 0) < 0.07:
                return
            self._adrag_t = now
            self.render_arm(q)  # 드래그는 틱을 기다리지 않고 즉시

    def on_toggle_collision(self):
        """조작창 충돌 모양 보기 — 본창 체크박스와 같은 sim.show_collision을 켜고 끈다"""
        self.sim.show_collision = bool(self.col_var.get())
        self.sim._render_t = 0  # 카메라도 바로 다시 그림
        q, _ = self.actual()
        if self.model and q:
            self._arm_key = None  # 다음 tick에서 강제 다시 그림
            self.render_arm(q)

    def on_toggle_frames(self):
        """조작창 관절축 보기 — 본창과 같은 BooleanVar라 양쪽 체크박스가 함께 바뀐다"""
        q, _ = self.actual()
        if self.model and q:
            self._arm_key = None  # 다음 tick에서 강제 다시 그림
            self.render_arm(q)

    def on_arm_zoom(self, direction):
        self.arm_cam.zoom(direction)
        q, _ = self.actual()
        if self.model and q:
            self.render_arm(q)  # 틱은 자세가 바뀔 때만 다시 그리므로 여기서 즉시

    def on_arm_release(self, _):
        if self.selected:
            send_arm(self.target)
        self.selected = None

    def render_arm(self, q):
        """팔 3D 한 장 — 목표·지금이 바뀌었을 때만 호출 (틱에서 매번 그리지 않음)"""
        model = self.model_col if self.sim.show_collision and self.model_col else self.model
        arm = self.arm_cam.render(model.faces(q, self.arm_links, tint=(70, 70, 80)), VIEW)  # 지금 (어둡게)
        goal = self.arm_cam.render(model.faces(self.target, self.arm_links), VIEW, grid=False)  # 목표
        mask = goal.any(axis=2) & (goal != (40, 40, 46)).any(axis=2)
        arm[mask] = cv2.addWeighted(arm, 0.3, goal, 0.7, 0)[mask]
        for name, (origin, _) in self.model.joint_frames(self.target).items():
            if name not in ARM_JOINTS:
                continue
            uv, z = self.arm_cam.project([origin], VIEW)
            if z[0] > 0:
                c = tuple(uv[0].astype(int))
                sel = name == self.selected
                cv2.circle(arm, c, 9 if sel else 7, (0, 80, 255) if sel else (0, 220, 255), 2, cv2.LINE_AA)
                cv2.putText(arm, name.replace('_joint', ''), (c[0] + 9, c[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                            (220, 220, 220), 1, cv2.LINE_AA)
        if self.frame_var is not None and self.frame_var.get():  # 관절축 트라이어드 (목표 자세, X빨·Y초·Z파)
            cols = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]
            for j in ARM_JOINTS:
                if j not in self.model.joints:
                    continue
                T = self.model.kin.pose(self.model.joints[j][0], self.target)
                o, R = T[:3, 3], T[:3, :3]
                pts = [o] + [o + R[:, i] * 0.04 for i in range(3)]
                uv, z = self.arm_cam.project(pts, VIEW)
                if (z < 0.02).any():
                    continue
                p = uv.astype(int)
                for i in range(3):
                    cv2.line(arm, tuple(p[0]), tuple(p[i + 1]), cols[i], 2, cv2.LINE_AA)
        self.photos = [to_photo(arm)]
        self.arm_label.config(image=self.photos[-1])

    # 물리 키코드 → KEYS 키 (한글 2벌식 등 입력기에서 keysym이 어긋날 때 폴백 — 자판 위치 기준이라 배열 무관)
    # darwin = macOS virtual keycode · linux = X11 keycode
    _PHYSICAL_KEYS = {
        'darwin': {13: 'w', 7: 'x', 1: 's', 12: 'q', 6: 'z', 14: 'e', 8: 'c', 0: 'a', 2: 'd',
                   30: 'bracketleft', 33: 'bracketright'},
        'linux': {25: 'w', 53: 'x', 39: 's', 24: 'q', 52: 'z', 26: 'e', 54: 'c', 38: 'a', 40: 'd',
                  34: 'bracketleft', 35: 'bracketright'},
    }

    # ---------- 바퀴 단축키 (W/X 앞뒤 · S 멈춤 · Q/Z 좌바퀴 앞뒤 · E/C 우바퀴 앞뒤 · A/D 회전 · [/] 속도)
    KEYS = {
        'w': (1, 1), 'x': (-1, -1), 's': (0, 0),
        'q': (1, 0), 'z': (-1, 0), 'e': (0, 1), 'c': (0, -1),
        'a': (-1, 1), 'd': (1, -1),
        'bracketright': 'faster', 'bracketleft': 'slower',
    }

    def on_key(self, e):
        if not self.winfo_viewable():  # 수동 탭이 아닐 땐 무시
            return
        focus = self.focus_get()
        if focus is not None and focus.winfo_class() in ('Entry', 'Text', 'Spinbox', 'TSpinbox'):
            return  # 명령 입력 중엔 무시
        key = (e.keysym or '').lower()
        d = self.KEYS.get(key)
        if d is None:
            # 한글 입력기 등 keysym이 어긋나면 물리 키코드로 (자판 배열 무관)
            d = self.KEYS.get(_PHYSICAL_KEYS.get(sys.platform, {}).get(getattr(e, 'keycode', None)))
        if d is None:
            return
        if isinstance(d, str):  # 속도 조절 — 바퀴 명령 없이 속도만
            self.change_speed(SPEED_STEP if d == 'faster' else -SPEED_STEP)
            return
        self.set_wheels(*d)
        self._key_serial = getattr(self, '_key_serial', 0) + 1
        self.info.after(1500, lambda s=self._key_serial: self._key_stop(s))

    def _key_stop(self, serial):
        if getattr(self, '_key_serial', 0) == serial:  # 그 사이 다른 키가 안 눌렸을 때만 정지
            self.set_wheels(0, 0)

    # ---------- 그리기 · 보내기
    def send_command(self):
        text = self.cmd_entry.get().strip()
        if not text:
            return
        cmd_manual_mode(False)  # 명령을 내리면 자동 모드
        send('/cmd/user_command', 'std_msgs/msg/String', {'data': text})
        self.cmd_entry.delete(0, tk.END)
        self.last_sent = (time.strftime('%H:%M:%S'), text)
        self.last_sent_t = time.time()  # 이 뒤에 바뀐 /sub_goals · /task_status만 응답으로
        self.refresh_cmd_status()

    def refresh_response(self):
        """planner 응답 — 내가 [보내기] 한 뒤에 온 것만 (파일엔 이전 실행 계획이 남아 있으므로 수정 시각으로 거름).
        단계마다 한 줄: 계획 /sub_goals → 1. skill target … · 진행 /task_status → step · 상태"""
        if not self.last_sent:
            self.resp_label.config(text='응답: 아직 보낸 명령 없음')
            return
        sent_t = self.last_sent_t
        plan, p_age = read_msg('sub_goals')
        st, s_age = read_msg('task_status')
        now = time.time()
        lines = []
        if plan and p_age is not None and now - p_age >= sent_t:
            goals = plan.get('goals', [])
            lines.append(f"계획  /sub_goals  plan {plan.get('plan_id', '?')} · {len(goals)}단계")
            for g in goals:
                rel = f" ({g.get('relation')})" if g.get('relation') else ''
                lines.append(f" {g.get('step', '?')}. {g.get('skill', '?')}  {g.get('target', '')}{rel}")
        if st and s_age is not None and now - s_age >= sent_t:
            state = {0: '진행 중', 1: '성공', 2: '실패'}.get(st.get('state', -1), '?')
            reason = st.get('reason', '')
            lines.append(f"진행  /task_status")
            lines.append(f" - plan {st.get('plan_id', '?')} step {st.get('step', '?')} {state}" + (f' — {reason}' if reason else ''))
        self.resp_label.config(text='\n'.join(lines) if lines else '응답 대기 중… (planner가 계획을 만들면 여기에)')

    def refresh_cmd_status(self):
        """명령줄 상태 — 보낸 명령 + 지금 카메라에 보이는 물체 (퍽이 안 보이면 planner가 계획 보류)"""
        tobjs, tage = read_msg('tracked_objects')
        objs = (tobjs or {}).get('objects', []) if tobjs else []
        kinds = {}
        for o in objs:
            k = o.get('label', o.get('class', '?'))
            kinds[k] = kinds.get(k, 0) + 1
        seen = ', '.join(f'{k} {v}' for k, v in sorted(kinds.items())) or '없음'
        if tage is not None and tage > 3:
            seen += ' (끊김?)'
        sent = f'보냄 {self.last_sent[0]} "{self.last_sent[1]}" · ' if self.last_sent else ''
        self.cmd_status.config(text=f'{sent}보이는 물체: {seen}')

    def show(self):
        # 창을 열기만으론 모드 안 바꿈 — 직접 만져야 수동 (send_twist/send_arm이 /cmd/operation_mode MANUAL)
        if not self.shown:
            self.shown = True

    def hide(self):
        # 창을 닫아도 로봇엔 아무것도 안 보낸다 — 모드도, 바퀴 정지도 그대로 둔다.
        # 바퀴 명령이 끊기면 체인(diff_drive_controller timeout)에서 알아서 브레이크.
        if not self.shown:
            return
        self.shown = False
        self.wheel = [0, 0]
        if self.on_close:
            self.on_close()

    def tick(self):
        if not self.winfo_exists():
            return
        if not self.winfo_viewable():  # 다른 탭이면 가볍게 폴링만
            self.after(300, self.tick)
            return
        desc, _ = read_msg('robot_description')
        if desc and desc['data'] != getattr(self, '_desc', None):  # 처음 · URDF가 바뀜 (urdf:= 바꿔 ROS 재시작)
            first = self.model is None
            self._desc = desc['data']
            self.model = robot_view.RobotModel(desc['data'])
            self.model_col = None  # 충돌 모양은 다시
            self.arm_links = self.model.descendants('arm_base_link')
            if first:
                self.build_sliders()
        q, wheel_vel = self.actual()
        if q and not self.target:
            self.target = {j: q.get(j, 0.0) for j in ARM_JOINTS}

        # 바퀴: 움직이는 동안 0.2초마다, 멈추면 0 한 번
        # mock 모드는 물리가 없어 벽을 뚫으므로 여기서 0.2초 앞을 예측해 막음 (실기 모드는 fake_opencr가 막음)
        self.ticks += 1
        v, w = self.twist()
        self.wall_stop = False
        if (v, w) != (0.0, 0.0):
            x, y, yaw, _ = self.sim.robot_xy_yaw()
            if self.sim.blocked(x + v * math.cos(yaw) * 0.2, y + v * math.sin(yaw) * 0.2, 0.18):
                v, w = 0.0, 0.0
                self.wall_stop = True
        if self.ticks % 2 == 0 and ((v, w) != (0.0, 0.0) or not self.sent_zero):
            send_twist(v, w)
            self.sent_zero = (v, w) == (0.0, 0.0)

        if self.model and self.sim.show_collision and self.model_col is None:
            desc, _ = read_msg('robot_description')
            if desc:
                self.model_col = robot_view.RobotModel(desc['data'], collision=True)
        self.sync_sliders()
        if self.model:
            key = (self.sim.show_collision, self.frame_var.get(),
                     tuple(round(self.target.get(j, 0.0), 3) for j in ARM_JOINTS),
                   tuple(round(q.get(j, 0.0), 3) for j in ARM_JOINTS))
            if key != getattr(self, '_arm_key', None) or self.selected is not None:
                self._arm_key = key
                self.render_arm(q)
        else:
            blank = np.full((VIEW[1], VIEW[0], 3), 40, np.uint8)
            cv2.putText(blank, 'waiting /robot_description (test-faker 0?)', (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (200, 200, 200), 1)
            self.photos = [to_photo(blank)]
            self.arm_label.config(image=self.photos[-1])

        if bool(self.col_var.get()) != bool(self.sim.show_collision):  # 본창에서 바꾼 것도 따라감
            self.col_var.set(bool(self.sim.show_collision))
        if self.ticks % 10 == 0:  # 1초마다 — 보낸 명령 + 보이는 물체 + planner 응답 갱신
            self.refresh_cmd_status()
            self.refresh_response()
        mode, _ = read_msg('state/operation_mode')
        mode = (mode or {}).get('data') if mode else None
        if mode is None:
            self.mode_label.config(text='○ 연결 안 됨', fg='#888888')
        else:
            manual = mode == 'MANUAL'
            self.mode_label.config(text='● 수동 모드' if manual else '○ 자동 모드',
                                   fg='#c62828' if manual else '#2e7d32')
        self.speed_label.config(text=f'{self.speed:.2f} m/s')
        wl, wr = self.wheel
        sign = lambda d: {1: '앞', -1: '뒤', 0: '정지'}[d]
        self.info.config(text=(
            f'모드     {mode if mode else "?"}\n'
            f'명령     좌 {sign(wl)} · 우 {sign(wr)}\n'
            f'         v {v:+.2f} m/s · ω {w:+.2f} rad/s\n'
            f'실제     바퀴 좌 {wheel_vel[0]:+.2f} · 우 {wheel_vel[1]:+.2f} rad/s\n'
            f'상태     {"실기 모드 (fake_opencr)" if self.sim.opencr.alive else "mock 하드웨어"} · 관절 /joint_states'
            + ('\n벽 앞 정지 (mock guard)' if getattr(self, 'wall_stop', False) else '')))
        self.joint_text.config(text='\n'.join(
            f'{j.replace("_joint", ""):8} 목표 {self.target.get(j, 0):+.3f}  지금 {q.get(j, 0):+.3f}' for j in ARM_JOINTS))
        self.after(100, self.tick)

    def close(self):
        self.hide()


ManualWindow = ManualPanel  # 하위 호환 별칭
