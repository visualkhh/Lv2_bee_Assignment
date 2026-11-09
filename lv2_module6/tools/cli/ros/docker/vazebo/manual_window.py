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
import threading
import time
import tkinter as tk

import cv2
import numpy as np

import robot_view
import scene
from perception_view import detected_key, draw_detected_3d
from bridge import (WHEEL_SEPARATION, cancel_action_goal, cmd_manual_mode, current_joints, read_action_file, read_msg, send,
                    send_action_goal, send_arm, send_bool, send_twist)

# MoveIt 결과 코드 (moveit_msgs/msg/MoveItErrorCodes) — [end_effector 이동] 결과 표시
MOVEIT_CODES = {1: '성공', -1: '계획 실패 (길 없음)', -2: '계획이 무효 (그새 바뀜)', -3: '환경 바뀜', -4: '실행 실패 (컨트롤러)',
                -6: '시간 초과', -7: '선점됨', -10: '시작 자세가 충돌', -12: '목표가 충돌', -13: '제약 위반',
                -15: '그룹 이름 틀림', -16: '목표 제약 틀림', -21: '프레임 틀림', -31: '닿을 수 없음 (IK 해 없음)',
                99999: '실패 — 그 자리에 손끝을 둘 자세가 없음 (닿지 않음 · 몸과 겹침)'}

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

    def __init__(self, master, sim, on_close=None, frame_var=None, object_faces=None, detected_var=None, base_map=None):
        super().__init__(master)
        self.sim, self.on_close = sim, on_close
        self.object_faces = object_faces  # T_wb → 주변 물체 면 목록 (본창 상태 3D와 같은 것 — main.App.world_object_faces)
        self.frame_var = frame_var  # 본창 '관절축 보기'와 공유 (없으면 조작창에서만 씀)
        self.detected_var = detected_var  # 본창 로봇 3D '감지 물체'와 공유 (없으면 조작창에서만 씀)
        self.base_map = base_map          # () → 로봇이 아는 자기 위치 (map x, y, yaw) — 감지 물체를 로봇 기준으로 옮길 때
        self.wheel_vel = [0.0, 0.0]       # 지금 바퀴 속도 [rad/s] (/joint_states) — 3D 바퀴 회전 표시 색
        self.shown = False
        self.model = None
        self.model_col = None         # 충돌 모양 (지도 탭 '충돌 모양 보기'를 켜면 — sim.show_collision)
        self.wheel = [0, 0]           # 좌·우 바퀴 방향
        self.speed = 0.10             # 전체 바퀴 속도 [m/s]
        self.sent_zero = True
        self.target = {}              # 팔 목표 (관절 → 값)
        self.selected = None
        self.drag_from = None
        self.arm_cam = robot_view.OrbitView(target=(0.05, 0, 0.18), dist=1.0, yaw=-2.0, pitch=0.35)  # 몸체·팔 전체 + 앞 바닥 물체

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
        tk.Label(mid, text='손끝 목표 — 손끝(하늘색)을 잡고 끌기 = 수평 · Shift+끌기 = 높이 → [end_effector 이동]',
                 fg='#6a1b9a').pack(anchor=tk.W)
        viewrow = tk.Frame(mid)
        viewrow.pack(anchor=tk.W)
        self.col_var = tk.BooleanVar(value=bool(self.sim.show_collision))
        tk.Checkbutton(viewrow, text='충돌모양', variable=self.col_var,
                       command=self.on_toggle_collision).pack(side=tk.LEFT)
        if self.frame_var is None:
            self.frame_var = tk.BooleanVar(value=False)
        tk.Checkbutton(viewrow, text='관절축', variable=self.frame_var,
                       command=self.on_toggle_frames).pack(side=tk.LEFT)
        if self.detected_var is None:
            self.detected_var = tk.BooleanVar(value=False)
        tk.Checkbutton(viewrow, text='감지 물체', variable=self.detected_var,
                       command=lambda: self.model and self.render_arm(self.actual()[0])).pack(side=tk.LEFT)
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
                                         ('그리퍼 초기화', self.init_gripper), ('모터 재부팅', self.reboot_motors)]):
            tk.Button(row, text=text, width=10, command=cmd).grid(row=i // 2, column=i % 2, padx=2, pady=2, sticky=tk.W)
        # 모터 Hardware Error (과부하·과열 등) — /state/devices/<wheels|arm|gripper> 세부의 '관절:에러' (state_manager)
        self.motor_label = tk.Label(right, text='', anchor=tk.W, justify=tk.LEFT, wraplength=260)
        self.motor_label.pack(anchor=tk.W)
        # 관절별 슬라이더 — 3D 화면 마커 끌기와 같은 목표(self.target)를 바꾼다. 놓으면 /cmd/manual/arm
        tk.Label(right, text='관절 (놓으면 MoveIt으로 — 몸체와 안 부딪히는 경로)', fg='#666').pack(anchor=tk.W, pady=(6, 0))
        self.slider_box = tk.Frame(right)
        self.slider_box.pack(anchor=tk.W, fill=tk.X)
        self.sliders = {}             # 관절 → (Scale, 단위 배율) — URDF가 오면 만든다 (한계·종류가 거기 있음)
        self.slider_held = None       # 끌고 있는 슬라이더 관절 (그동안 틱이 값을 덮지 않게)
        self.joint_text = tk.Label(right, justify=tk.LEFT, font=('Menlo', 10), anchor=tk.W)
        self.joint_text.pack(anchor=tk.W, pady=6)
        self.photos = []
        self.goal = None              # 손끝(end_effector_link) 목표 [x, y, z] — base_footprint 기준 (팔 3D 화면 좌표계 그대로)
        self.goal_drag = None         # 손끝을 끄는 중: 'xy' (수평면) · 'z' (Shift, 높이)
        self.reach = None             # 손끝이 닿는 범위 (build_reach) — 끌 때 이 밖으로 못 나감
        self.body = None              # 몸체 높이 지도 (build_body) — 손끝은 이 위로만
        tk.Label(right, text='손끝 목표 (base_footprint)', font=('', 11, 'bold'), fg='#6a1b9a').pack(anchor=tk.W, pady=(8, 0))
        self.goal_text = tk.Label(right, text='', justify=tk.LEFT, font=('Menlo', 10), fg='#6a1b9a')
        self.goal_text.pack(anchor=tk.W)
        # 손끝 방향 — 기본 위→아래(집기 자세: MoveIt 손끝 목표 + 방향 제약) · 자유(팔 든 자세 그대로 손끝 위치만: 시뮬레이터 IK → MoveIt 관절 목표)
        self.free_orient = tk.BooleanVar(value=False)
        tk.Checkbutton(right, text='손끝 방향 자유 (위치만 — 팔 든 자세 그대로)', variable=self.free_orient,
                       command=self.on_toggle_free).pack(anchor=tk.W)
        self.orient_text = tk.Label(right, text='', font=('Menlo', 9))
        self.orient_text.pack(anchor=tk.W)
        # 손가락 방향 (손목 회전 joint5) — 끄면 자유(MoveIt이 고름), 켜면 손가락이 벌어지는 선의 수평 방위로 고정 (누운 퍽 등)
        frow = tk.Frame(right)
        frow.pack(anchor=tk.W)
        self.finger_fix = tk.BooleanVar(value=False)
        tk.Checkbutton(frow, text='손가락 방향', variable=self.finger_fix, command=self.redraw_goal).pack(side=tk.LEFT)
        self.finger_deg = tk.Scale(frow, from_=0, to=179, orient=tk.HORIZONTAL, length=150, showvalue=True,
                                   command=lambda _: self.redraw_goal())
        self.finger_deg.pack(side=tk.LEFT)
        tk.Label(frow, text='° (0 = 앞뒤로 벌림)', font=('Menlo', 9)).pack(side=tk.LEFT)
        brow = tk.Frame(right)
        brow.pack(anchor=tk.W)
        tk.Button(brow, text='end_effector 이동', command=self.move_ee).pack(side=tk.LEFT)
        tk.Button(brow, text='이동 취소', command=self.cancel_ee).pack(side=tk.LEFT)
        tk.Button(brow, text='목표 지우기', command=self.clear_goal).pack(side=tk.LEFT)
        # 손끝 목표에서 하는 동작 (MoveIt 경유) — coordinator pick·place와 같은 순서를 수동으로
        grow = tk.Frame(right)
        grow.pack(anchor=tk.W, pady=(2, 0))
        tk.Button(grow, text='퍽 집기', command=self.seq_pick).pack(side=tk.LEFT)
        tk.Button(grow, text='내려놓기', command=self.seq_place).pack(side=tk.LEFT)
        trow = tk.Frame(right)
        trow.pack(anchor=tk.W)
        tk.Label(trow, text='적재함 칸').pack(side=tk.LEFT)
        self.tray_slot = tk.Spinbox(trow, from_=1, to=4, width=3)
        self.tray_slot.pack(side=tk.LEFT)
        tk.Button(trow, text='싣기', command=self.seq_tray_put).pack(side=tk.LEFT)
        tk.Button(trow, text='꺼내기', command=self.seq_tray_take).pack(side=tk.LEFT)
        self.direct = tk.BooleanVar(value=False)
        tk.Checkbutton(right, text='관절 직접 보내기 (MoveIt 없이 — 충돌 검사 없음)', variable=self.direct).pack(anchor=tk.W)
        self.ee_text = tk.Label(right, text='', justify=tk.LEFT, font=('Menlo', 10))
        self.ee_text.pack(anchor=tk.W)
        self.ee_goal_id = None  # 진행 중 /move_action goal
        self.ee_kind = 'pose'   # 'pose' (손끝) · 'joints' (관절 — 실패하면 목표를 지금 자세로 되돌림)
        self.seq = []           # 진행 중 동작 순서 [('move', xyz) | ('grip', 폭 m), …] — 하나씩 끝나면 다음
        self.seq_wait = None    # 그리퍼 대기 (목표 폭, 시작 시각)
        self.seq_name = None    # 진행 중 동작 이름 (없으면 None)
        self.redraw_goal()
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
        # 놓는 이벤트(ButtonRelease)가 안 오는 경우가 있어(macOS — 놓았는데 안 보내짐, 실측) 0.4초 멈추면 보냄
        if getattr(self, '_slider_after', None):
            self.after_cancel(self._slider_after)
        self._slider_after = self.after(400, self.slider_settled)

    def slider_settled(self):
        self._slider_after = None
        self.slider_held = None
        self.send_joints()

    def on_slider_release(self, _):
        if getattr(self, '_slider_after', None):  # 놓음이 먼저 오면 바로 (대기 취소)
            self.after_cancel(self._slider_after)
            self._slider_after = None
        if self.slider_held is not None:
            self.slider_held = None
            self.send_joints()

    def send_joints(self):
        """팔 관절 목표 보내기 — 기본은 MoveIt(/move_action 관절 목표: 몸체·폴과 안 부딪히는 경로, 불가능하면 거절),
        '관절 직접 보내기'면 예전처럼 /cmd/manual/arm (JTC로 바로 — 충돌 검사 없음). 그리퍼는 늘 직접"""
        if self.direct.get():
            send_arm(self.target)
            return
        arm = {j: round(float(v), 4) for j, v in self.target.items() if j != 'gripper_left_joint'}
        cmd_manual_mode(True)
        self.ee_kind = 'joints'
        self.ee_goal_id = send_action_goal('move_action', {'joints': arm})
        self.ee_text.config(text='요청: 관절 목표 → /move_action (MoveIt)', fg='#1565c0')

    def send_gripper(self, width):
        if not self.target:
            q, _ = self.actual()
            self.target = {j: q.get(j, 0.0) for j in ARM_JOINTS}
        self.target['gripper_left_joint'] = width
        send_arm({'gripper_left_joint': width})  # 팔 관절은 manual_control이 지금 각으로 채움 (안 움직임)

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
        self.send_joints()

    def set_gripper(self, value):
        self.send_gripper(value)

    def reboot_motors(self):
        """Hardware Error 난 DYNAMIXEL만 재부팅 — /cmd/manual/reboot_motors (수동 모드에서만 manual_control이 받음)"""
        cmd_manual_mode(True)
        send('/cmd/manual/reboot_motors', 'std_msgs/msg/Empty', {})
        self.motor_note = time.time()

    def refresh_motor_errors(self):
        errors = []
        for dev in ('wheels', 'arm', 'gripper'):
            msg, _ = read_msg(f'state/devices/{dev}')
            parts = (msg or {}).get('data', '').split('|', 2)
            if len(parts) == 3:
                errors += [d for d in parts[2].split(',') if ':' in d]
        if errors:
            text, color = '모터 에러: ' + ', '.join(errors) + ' → [모터 재부팅]', '#d33'
        elif time.time() - getattr(self, 'motor_note', 0) < 5:
            text, color = '재부팅 요청 보냄 — 팔은 천천히 명령 자세로', '#666'
        else:
            text, color = '', '#666'
        if self.motor_label.cget('text') != text:
            self.motor_label.config(text=text, fg=color)

    def init_gripper(self):
        """그리퍼 초기화 — URDF 원점(관절값 0.0, 그린 그대로)으로"""
        self.send_gripper(0.0)

    # ---------- 손끝 목표 (3D 화면 좌표 = base_footprint)
    def view_ray(self, u, v):
        """팔 3D 화면 픽셀 → 광선 (시작점 eye, 단위 방향)"""
        eye, right, up, fwd = self.arm_cam.basis()
        f = VIEW[1] / 2 / math.tan(self.arm_cam.fov / 2)
        d = fwd + right * (u - VIEW[0] / 2) / f - up * (v - VIEW[1] / 2) / f
        return eye, d / np.linalg.norm(d)

    def ee_target(self):
        return self.goal

    def move_ee(self):
        """[end_effector 이동] → MoveIt /move_action (계획+실행): 손끝(end_effector_link)을 끌어 둔 자리로, 위→아래 방향"""
        t = self.ee_target()
        if t is None:
            self.ee_text.config(text='먼저 팔 3D에서 손끝(하늘색)을 잡고 끄세요', fg='#b26a00')
            return
        if (not self.free_orient.get() and getattr(self, 'reach_full', None) is not None and not self.in_reach(t, self.reach)
                and self.in_reach(t, self.reach_full)):
            # 집기 자세로는 못 가는데 방향 자유로는 가는 곳 (예: 팔을 든 채 높이) — 보내 봐야 MoveIt이 실패하니 먼저 물어봄
            from tkinter import messagebox
            if messagebox.askyesno('손끝 방향', '집기 자세(손가락 아래)로는 못 가는 곳입니다.\n' + self.reach_hint() +
                                   '\n\n손끝 방향 자유(위치만 — 팔 든 자세 그대로)로 보낼까요?'):
                self.free_orient.set(True)
                self.on_toggle_free()
            else:
                self.ee_text.config(text='취소 — 집기 자세로 보내려면 청록(집기 자세) 범위 안으로 끄세요', fg='#b26a00')
                return
        if self.free_orient.get():  # 방향 자유: 위치만 맞는 관절값을 여기서 풀어 MoveIt 관절 목표로
            q = self.solve_free_ik(t)
            if q is None:
                self.ee_text.config(text=f'IK 해 없음 — 손끝이 ({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f})에 닿는 관절값이 없음\n'
                                    + self.reach_hint(), fg='#c62828')
                return
            self.target.update(q)
            self.sync_sliders()
            arm = {j: round(v, 4) for j, v in q.items()}
            cmd_manual_mode(True)
            self.ee_kind = 'free'
            self.ee_goal_id = send_action_goal('move_action', {'joints': arm})
            self.ee_text.config(text=f'요청: 손끝 → ({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f}) 방향 자유 — IK 관절 목표 /move_action',
                                fg='#1565c0')
            return
        self.send_pose(t)

    def finger_yaw(self):
        """손가락 방향을 지정했으면 /move_action의 yaw (손끝 x축 방위 = 손가락 선 + 90° — 손가락은 손끝 y축으로 벌어짐), 아니면 None"""
        if not self.finger_fix.get():
            return None
        return math.radians(float(self.finger_deg.get())) + math.pi / 2

    def send_pose(self, t):
        cmd_manual_mode(True)  # 수동 — 자동 작업(coordinator)이 팔을 같이 움직이지 않게
        self.ee_kind = 'pose'
        goal = {'x': round(float(t[0]), 4), 'y': round(float(t[1]), 4), 'z': round(float(t[2]), 4)}
        yaw = self.finger_yaw()
        if yaw is not None:
            goal['yaw'] = round(yaw, 4)
        self.ee_goal_id = send_action_goal('move_action', goal)
        turn = '' if yaw is None else f' · 손가락 {float(self.finger_deg.get()):.0f}°'
        self.ee_text.config(text=f'요청: 손끝 → ({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f}){turn} /move_action', fg='#1565c0')

    # ---------- 동작 순서 (퍽 집기 · 내려놓기 · 적재함) — MoveIt 이동과 그리퍼를 차례로, 하나라도 실패하면 멈춤
    GRIP_OPEN, GRIP_CLOSED = 0.0265, 0.0   # URDF 한계 (coordinator와 같음)
    APPROACH = 0.05                        # 집기·내려놓기 접근 높이 [m]
    TRAY_RELEASE_Z, TRAY_GRASP_Z = 0.22, 0.17  # coordinator tray_release_z · tray_grasp_z와 같게

    def tray_slots(self):
        """적재함 칸 — coordinator 파라미터 tray_slots (debug/node/coordinator/params, monitor_manager 기록) · 없으면 기본값"""
        try:
            from bridge import DEBUG
            for line in (DEBUG / 'node' / 'coordinator' / 'params').read_text().splitlines():
                if line.startswith('tray_slots:'):
                    v = [float(x) for x in line.split('[', 1)[1].rstrip(']').split(',')]
                    return [(v[i], v[i + 1]) for i in range(0, len(v) - 1, 2)]
        except (OSError, ValueError, IndexError):
            pass
        return [(-0.135, 0.05), (-0.135, -0.05), (-0.095, 0.05), (-0.095, -0.05)]

    def run_seq(self, name, steps):
        self.seq = list(steps)
        self.seq_name = name
        self.ee_text.config(text=f'{name}: 시작 ({len(steps)}단계)', fg='#1565c0')
        self.next_seq()

    def next_seq(self):
        if not self.seq:
            self.ee_text.config(text=f'{self.seq_name}: 끝', fg='#2e7d32')
            self.seq_name = None
            return
        kind, arg = self.seq.pop(0)
        if kind == 'move':
            self.send_pose(arg)
        else:  # grip — 폭에 닿거나(닫힘은 퍽에 막혀 멈춤) 2.5초 지나면 다음
            self.send_gripper(arg)
            self.seq_wait = (arg, time.time())

    def poll_seq(self):
        if self.seq_wait is None:
            return
        width, t0 = self.seq_wait
        q, _ = self.actual()
        if abs((q or {}).get('gripper_left_joint', width) - width) < 0.002 or time.time() - t0 > 2.5:
            self.seq_wait = None
            self.next_seq()

    def seq_pick(self):
        """[퍽 집기] — 손끝 목표 = 집을 자리 (퍽 가운데 높이): 벌리기 → 위로 접근 → 내려가기 → 닫기 → 들어올리기"""
        t = self.ee_target()
        if t is None:
            self.ee_text.config(text='먼저 손끝(하늘색)을 퍽 위치로 끌어다 놓으세요', fg='#b26a00')
            return
        up = t + [0, 0, self.APPROACH]
        self.run_seq('퍽 집기', [('grip', self.GRIP_OPEN), ('move', up), ('move', t), ('grip', self.GRIP_CLOSED), ('move', up)])

    def seq_place(self):
        """[내려놓기] — 손끝 목표에서: 위로 접근 → 내려가기 → 벌리기 → 들어올리기"""
        t = self.ee_target()
        if t is None:
            self.ee_text.config(text='먼저 손끝(하늘색)을 내려놓을 자리로 끌어다 놓으세요', fg='#b26a00')
            return
        up = t + [0, 0, self.APPROACH]
        self.run_seq('내려놓기', [('move', up), ('move', t), ('grip', self.GRIP_OPEN), ('move', up)])

    def slot_xy(self):
        slots = self.tray_slots()
        try:
            return slots[max(0, min(len(slots) - 1, int(self.tray_slot.get()) - 1))]
        except ValueError:
            return slots[0]

    def seq_tray_put(self):
        """[적재함 싣기] — 고른 칸 위 TRAY_RELEASE_Z에서 벌림 (coordinator place tray와 같음)"""
        x, y = self.slot_xy()
        self.run_seq('적재함 싣기', [('move', np.array([x, y, self.TRAY_RELEASE_Z])), ('grip', self.GRIP_OPEN)])

    def seq_tray_take(self):
        """[적재함 꺼내기] — 고른 칸: 벌리기 → 칸 위 → 파지 높이 → 닫기 → 들어올리기 (coordinator pick tray와 같음)"""
        x, y = self.slot_xy()
        up, at = np.array([x, y, self.TRAY_RELEASE_Z]), np.array([x, y, self.TRAY_GRASP_Z])
        self.run_seq('적재함 꺼내기', [('grip', self.GRIP_OPEN), ('move', up), ('move', at), ('grip', self.GRIP_CLOSED), ('move', up)])

    def reach_hint(self):
        """실패했을 때 — 닿는 범위 지도(build_reach)로 이유: 그 거리에서 집기 자세(손가락 아래)·방향 자유로 갈 수 있는 높이"""
        t = self.ee_target()
        if t is None or not self.model or self.reach is None or getattr(self, 'reach_full', None) is None:
            return ''
        axis = self.reach[0]
        r = math.hypot(t[0] - axis[0], t[1] - axis[1])

        def z_range(reach):  # 그 거리(±1cm)에서 닿는 높이 범위
            rz = reach[1]
            row = rz[np.abs(rz[:, 0] - r) <= 0.01]
            return (float(row[:, 1].min()), float(row[:, 1].max())) if len(row) else None

        down, full = z_range(self.reach), z_range(self.reach_full)
        where = f'팔 회전축에서 수평 {r:.2f}m · 높이 {t[2]:+.2f}m — '
        if full is None:
            return where + '이 거리는 방향 상관없이도 안 닿음 (팔이 짧거나 몸에 너무 가까움)'
        if down is None:
            return where + f'이 거리에선 집기 자세가 안 됨 · 방향 자유로는 높이 {full[0]:.2f}~{full[1]:.2f}m → [손끝 방향 자유]'
        if not (down[0] - 0.01 <= t[2] <= down[1] + 0.01):
            return (where + f'이 거리에서 집기 자세(손가락 아래)는 높이 {down[0]:.2f}~{down[1]:.2f}m까지' +
                    (f' · 방향 자유로는 {full[0]:.2f}~{full[1]:.2f}m → [손끝 방향 자유]' if full[0] - 0.01 <= t[2] <= full[1] + 0.01 else ''))
        return where + '범위 안 — 몸체·폴과 부딪히는 경로이거나 IK가 못 찾음 (조금 옮겨서 다시)'

    def cancel_ee(self):
        if self.ee_goal_id:
            cancel_action_goal('move_action', self.ee_goal_id)

    def poll_ee(self):
        """진행 중 /move_action 수락·진행·결과 → 글로"""
        if not self.ee_goal_id:
            return
        acc, _ = read_action_file('move_action', 'accept')
        if acc and acc.get('goal_id') == self.ee_goal_id and not acc.get('accepted'):
            self.ee_text.config(text=f"거절: {acc.get('error', '?')}", fg='#c62828')
            self.ee_goal_id = None
            self.seq = []
            return
        res, _ = read_action_file('move_action', 'result')
        if res and res.get('goal_id') == self.ee_goal_id:
            code = (res.get('data') or {}).get('error_code')
            ok = code == 1
            hint = '' if ok or self.ee_kind == 'joints' else '\n' + self.reach_hint()
            self.ee_text.config(text=f"결과: {MOVEIT_CODES.get(code, code)} ({res.get('status')}){hint}", fg='#2e7d32' if ok else '#c62828')
            self.ee_goal_id = None
            if ok or self.ee_kind in ('joints', 'free'):
                # 성공: MoveIt이 옮긴 실제 자세로 슬라이더·밝은 팔(목표)을 맞춤 — 안 맞추면 예전 목표가 남아 엉뚱한 자세로 보였음.
                # 관절 목표 실패: 거절됐으니 지금 자세로 되돌림. /joint_states가 결과보다 늦게 올 수 있어 1.5초 동안 계속 맞춤
                self.follow_actual_until = time.time() + 1.5
            if getattr(self, 'seq_name', None):  # 동작 순서 진행 중 — 성공이면 다음 단계, 실패면 멈춤
                if ok:
                    self.next_seq()
                else:
                    self.ee_text.config(text=f"{self.seq_name}: 실패 — {MOVEIT_CODES.get(code, code)}{hint}", fg='#c62828')
                    self.seq, self.seq_name = [], None
            return
        fb, _ = read_action_file('move_action', 'feedback')
        if fb and fb.get('goal_id') == self.ee_goal_id:
            self.ee_text.config(text=f"진행: {(fb.get('data') or {}).get('state', '')}", fg='#1565c0')

    def on_toggle_free(self):
        self.goal_entered = self.in_reach(self.goal)
        self.redraw_goal()

    def clear_goal(self):
        self.goal = None
        self.redraw_goal()

    def redraw_goal(self):
        g = self.goal
        self.goal_text.config(text='없음 — 팔 3D에서 손끝(하늘색)을 잡고 끄세요' if g is None else
                              f'x {g[0]:+.3f}  y {g[1]:+.3f}  z {g[2]:+.3f} m')
        free = self.free_orient.get()
        msg, fg = (('방향: 자유 — 시뮬레이터 IK → MoveIt 관절 목표 (몸체와 안 부딪히는 경로)', '#1565c0') if free else
                   ('방향: 위→아래 (집기 자세 — MoveIt 손끝 목표)', '#000000'))
        if g is not None and self.active_reach() is not None and not self.in_reach(g):
            msg, fg = ((f'⚠ 손끝이 닿지 않는 곳 (전체 범위 밖)', '#c62828') if free else
                       ('⚠ 집기 자세로는 못 가는 곳 — 청록 범위 안으로 끌거나 [손끝 방향 자유]', '#c62828'))
        self.orient_text.config(text=msg, fg=fg)
        q, _ = self.actual()
        if self.model and q:
            self.render_arm(q)

    def ee_now(self):
        """지금 손끝(end_effector_link) 위치 — 실제 관절값 기준 (손잡이)"""
        q, _ = self.actual()
        if not self.model or not q or 'end_effector_link' not in self.model.kin.parent:
            return None
        return self.model.kin.pose('end_effector_link', q)[:3, 3]

    def near_point(self, p, u, v):
        if p is None:
            return False
        uv, z = self.arm_cam.project([p], VIEW)
        return z[0] > 0 and math.dist(uv[0], (u, v)) < 14

    # 닿는 범위 보정 (MoveIt /move_action으로 실측, beebot_mesh): 관절 계산 범위의 바깥 경계는 약 2cm 넘쳐 실패 →
    # REACH_MARGIN칸 깎음 · 팔 회전축에서 REACH_R_MIN 안쪽은 팔이 자기 몸과 부딪혀 실패 (높이 0.10에서 r 0.05 X · 0.08 O)
    # 몸체: joint1은 360° 돌아 방향 제한은 없고, 손끝이 로봇 몸체(차체·적재함·폴·바퀴 — URDF 충돌 모양) 위에 있어야 함.
    #   몸체 높이 지도(x·y 칸마다 몸체 꼭대기 높이)를 손가락·그리퍼 반폭 BODY_CLEAR_R만큼 넓혀, 손끝이 그보다 BODY_CLEAR_Z 위여야 닿는 곳
    REACH_MARGIN = 2
    REACH_R_MIN = 0.08
    BODY_CLEAR_R = 0.04
    BODY_CLEAR_Z = 0.01
    BODY_HAND_H = 0.10  # 손끝 위로 그리퍼·손목이 차지하는 높이 — 그보다 높은 몸체(적재함 위 pole 카메라 0.42m)는 손 위라 상관없음

    def build_body(self, desc, cell=0.01):
        """몸체 칸 지도 — URDF 충돌 모양(팔 제외) 면을 반 칸 간격으로 찍어 3D 칸(x·y·z)을 채우고 x·y로 BODY_CLEAR_R 넓힘.
        z마다 '그 높이 이하에서 가장 높은 몸체 칸'을 미리 계산 → self.body = (x0, y0, cell, top_le[x, y, z])"""
        m = robot_view.RobotModel(desc, collision=True)
        arm = m.descendants('arm_base_link')
        tris = []
        for link, T_o, faces, _ in m.visuals:
            if link in arm:
                continue
            T = m.kin.pose(link, {}) @ T_o
            polys = faces if isinstance(faces, np.ndarray) else [np.asarray(f) for f in faces]
            for f in polys:
                f = np.asarray(f, float) @ T[:3, :3].T + T[:3, 3]
                for k in range(1, len(f) - 1):  # 다각형 → 부채꼴 삼각형
                    tris.append(np.stack([f[0], f[k], f[k + 1]]))
        if not tris:
            return None
        allp = np.concatenate(tris)
        x0, y0 = allp[:, 0].min() - 0.1, allp[:, 1].min() - 0.1
        nx, ny = int((allp[:, 0].max() + 0.1 - x0) / cell) + 1, int((allp[:, 1].max() + 0.1 - y0) / cell) + 1
        nz = int(allp[:, 2].max() / cell) + 2
        occ = np.zeros((nx, ny, nz), bool)
        for t in tris:  # 면 위를 반 칸 간격으로 찍음 (삼각형 상자로 잡으면 큰 면이 넓게 번졌음)
            n = max(1, int(math.ceil(max(np.linalg.norm(t[1] - t[0]), np.linalg.norm(t[2] - t[0]),
                                         np.linalg.norm(t[2] - t[1])) / (cell / 2))))
            u, v = np.meshgrid(np.arange(n + 1), np.arange(n + 1))
            keep = (u + v) <= n
            u, v = u[keep] / n, v[keep] / n
            pts = t[0] + u[:, None] * (t[1] - t[0]) + v[:, None] * (t[2] - t[0])
            occ[np.clip(((pts[:, 0] - x0) / cell).astype(int), 0, nx - 1),
                np.clip(((pts[:, 1] - y0) / cell).astype(int), 0, ny - 1),
                np.clip((pts[:, 2] / cell).astype(int), 0, nz - 1)] = True
        k = int(math.ceil(self.BODY_CLEAR_R / cell))
        grown = occ.copy()  # 그리퍼 반폭만큼 x·y 넓히기
        for di in range(-k, k + 1):
            for dj in range(-k, k + 1):
                if di * di + dj * dj <= k * k:
                    grown[max(di, 0):nx + min(di, 0), max(dj, 0):ny + min(dj, 0)] |= \
                        occ[max(-di, 0):nx + min(-di, 0), max(-dj, 0):ny + min(-dj, 0)]
        idx = np.where(grown, np.arange(nz)[None, None, :], -1)
        return (x0, y0, cell, np.maximum.accumulate(idx, axis=2))  # z 이하에서 가장 높은 몸체 칸 번호 (없으면 −1)

    def body_floor(self, x, y, z):
        """손끝이 (x, y, z)에 있으려면 넘어야 할 몸체 높이 — 손끝~손끝+BODY_HAND_H 높이까지의 몸체만 봄 (배열 가능).
        손끝 z가 이 값 이상이면 몸체와 안 부딪힘"""
        x, y, z = np.broadcast_arrays(np.asarray(x, float), np.asarray(y, float), np.asarray(z, float))
        if getattr(self, 'body', None) is None:
            return np.zeros_like(x)
        x0, y0, cell, top = self.body
        nx, ny, nz = top.shape
        i = np.clip(((x - x0) / cell).astype(int), 0, nx - 1)
        j = np.clip(((y - y0) / cell).astype(int), 0, ny - 1)
        k = np.clip(((z + self.BODY_HAND_H) / cell).astype(int), 0, nz - 1)
        inside = (x >= x0) & (y >= y0) & (x < x0 + nx * cell) & (y < y0 + ny * cell)
        t = top[i, j, k]
        return np.where(inside & (t >= 0), (t + 1) * cell + self.BODY_CLEAR_Z, 0.0)

    def build_reach(self, model, steps=32, tilt=0.15, cell=0.01, desc=None):
        """손끝이 닿는 곳 두 가지 — (팔 회전축에서 수평 거리 r, 높이 z) 칸 [cell m]. joint1은 수직축 회전이라 r·z만 보면 되고
        joint5는 손끝 위치에 영향 없음 → joint2~4를 한계 안에서 steps³개 격자로 FK.
          self.reach      집기 자세(손가락이 아래, 기울기 ≤ tilt — [end_effector 이동] 기본 MoveIt 조건과 같음)로 닿는 곳
          self.reach_full 손끝 방향 상관없이 닿는 곳 ('손끝 방향 자유' — 팔을 든 자세 그대로 옮길 때)
          self.ik_table   FK 표 [[r, z, joint2, joint3, joint4], …] — 방향 자유 IK의 시작점
        각각 (회전축 x·y, [[r, z], …] 칸 중심 배열)"""
        kin = model.kin
        axis = kin.pose('shoulder_link', {})[:3, 3]  # joint1 회전축이 지나는 곳 (팔 받침 위)
        grids = [np.linspace(model.joints[j][3], model.joints[j][4], steps) for j in ('joint2', 'joint3', 'joint4')]
        cells, cells_all, table = set(), set(), []
        down = -math.cos(tilt)
        for a in grids[0]:
            for b in grids[1]:
                for c in grids[2]:
                    T = kin.pose('end_effector_link', {'joint1': 0.0, 'joint2': a, 'joint3': b, 'joint4': c, 'joint5': 0.0})
                    if T[2, 3] < 0.0:  # 바닥 아래는 뺌
                        continue
                    r = math.hypot(T[0, 3] - axis[0], T[1, 3] - axis[1])
                    key = (round(r / cell), round(T[2, 3] / cell))
                    cells_all.add(key)
                    table.append((r, T[2, 3], a, b, c))
                    if T[2, 2] <= down:  # 손끝 z축(손가락 방향)이 충분히 아래
                        cells.add(key)
        rz_down, rz_full = self.reach_grid(cells, cell), self.reach_grid(cells_all, cell)
        body = self.build_body(desc) if desc else None
        if model is self.model:  # 그새 모델이 바뀌었으면 버림
            self.body = body
            self.reach = (axis[:2].copy(), rz_down)
            self.reach_full = (axis[:2].copy(), rz_full)
            self.ik_table = np.array(table, float)
            self._arm_key = None  # 다음 틱에 다시 그림 (범위 표시)

    def reach_grid(self, cells, cell):
        """FK로 찍힌 (r, z) 칸 → 닿는 칸 배열 [[r, z], …]: 성긴 격자 샘플의 빈틈 메우기(닫기 = 넓혔다 좁힘) → 바깥 경계
        REACH_MARGIN칸 깎기(MoveIt 실측 보정) → 몸 쪽 REACH_R_MIN 안쪽 빼기.
        바닥(z=0)은 경계가 아니라 깎지 않게, 바닥 아래로 pad칸을 바닥 줄과 같은 값으로 채워 두고 계산"""
        if not cells:
            return np.zeros((0, 2))
        pad = self.REACH_MARGIN + 3
        nr, nz = max(r for r, _ in cells) + 2 * pad, max(z for _, z in cells) + 2 * pad
        g = np.zeros((nr, nz), bool)
        for ri, zi in cells:
            g[ri, zi + pad] = True

        def dilate(a, k):  # 원판(반지름 k칸) 넓히기 — 격자 밖은 빈 칸
            out = a.copy()
            for dr in range(-k, k + 1):
                for dz in range(-k, k + 1):
                    if dr * dr + dz * dz <= k * k:
                        out[max(dr, 0):nr + min(dr, 0), max(dz, 0):nz + min(dz, 0)] |= \
                            a[max(-dr, 0):nr + min(-dr, 0), max(-dz, 0):nz + min(-dz, 0)]
            return out

        def floor_fill(a):  # 바닥 아래 = 바닥 줄 그대로
            a[:, :pad] = a[:, [pad]]
            return a

        g = floor_fill(~dilate(~floor_fill(dilate(floor_fill(g), 2)), 2))  # 닫기
        g = ~dilate(~g, self.REACH_MARGIN)                                 # 깎기
        g[: int(math.ceil(self.REACH_R_MIN / cell)), :] = False
        keep = [(ri, zi - pad) for ri, zi in zip(*np.nonzero(g)) if zi >= pad]
        return np.array(sorted(keep), float).reshape(-1, 2) * cell

    def active_reach(self):
        """지금 모드의 닿는 범위 — '손끝 방향 자유'면 전체, 아니면 집기 자세"""
        return getattr(self, 'reach_full', None) if self.free_orient.get() else self.reach

    def in_reach(self, p, reach=None):
        """손끝 목표 p가 닿는 칸 안이고 몸체 위인가"""
        reach = reach if reach is not None else self.active_reach()
        if reach is None or not len(reach[1]) or p is None:
            return False
        axis, rz = reach
        r = math.hypot(p[0] - axis[0], p[1] - axis[1])
        near = np.min((rz[:, 0] - r) ** 2 + (rz[:, 1] - p[2]) ** 2) <= 0.01 ** 2
        return bool(near and p[2] >= float(self.body_floor(p[0], p[1], p[2])))

    def solve_free_ik(self, t, tol=0.005):
        """'손끝 방향 자유' IK — 손끝(end_effector_link)을 t에 두는 관절값 (방향은 상관없음) 또는 None.
        joint1 = 회전축에서 본 방위, (r, z)는 FK 표에서 가까운 후보 몇 개를 감쇠 최소제곱으로 다듬어
        오차 tol 안인 것 중 지금 자세에서 가장 덜 움직이는 해. joint5(손목 회전)는 지금 값 그대로"""
        table = getattr(self, 'ik_table', None)
        if table is None or not len(table) or not self.model:
            return None
        kin, joints = self.model.kin, self.model.joints
        axis = self.reach_full[0]
        dx, dy = t[0] - axis[0], t[1] - axis[1]
        q_now, _ = self.actual()
        r_t = math.hypot(dx, dy)
        j1 = math.atan2(dy, dx) if r_t > 1e-4 else float(q_now.get('joint1', 0.0))
        lo = np.array([joints[j][3] for j in ('joint2', 'joint3', 'joint4')])
        hi = np.array([joints[j][4] for j in ('joint2', 'joint3', 'joint4')])
        cur = np.array([float(q_now.get(j, 0.0)) for j in ('joint2', 'joint3', 'joint4')])
        target = np.array([r_t, float(t[2])])

        def rz_of(q):
            T = kin.pose('end_effector_link', {'joint1': 0.0, 'joint2': q[0], 'joint3': q[1], 'joint4': q[2], 'joint5': 0.0})
            return np.array([math.hypot(T[0, 3] - axis[0], T[1, 3] - axis[1]), T[2, 3]])

        d = np.hypot(table[:, 0] - target[0], table[:, 1] - target[1])
        best = None
        for i in np.argsort(d)[:8]:
            q = table[i, 2:].copy()
            for _ in range(15):  # 감쇠 최소제곱 (수치 야코비 2×3)
                e = target - rz_of(q)
                if np.linalg.norm(e) < tol / 5:
                    break
                J = np.stack([(rz_of(q + dq) - rz_of(q)) / 1e-4 for dq in np.eye(3) * 1e-4], 1)
                q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(2), e), lo, hi)
            err = float(np.linalg.norm(target - rz_of(q)))
            if err < tol:
                cost = float(np.abs(q - cur).sum())
                if best is None or cost < best[0]:
                    best = (cost, q, err)
        if best is None:
            return None
        q = best[1]
        return {'joint1': j1, 'joint2': float(q[0]), 'joint3': float(q[1]), 'joint4': float(q[2]),
                'joint5': float(q_now.get('joint5', 0.0))}

    def clamp_reach(self, p):
        """손끝 목표를 닿는 곳으로 — (r, z)가 닿는 칸이고 몸체 위여야 함. 아니면 같은 방향(회전축 기준)에서 그 조건을 만족하는
        가장 가까운 칸으로 (방향은 그대로 — joint1은 360°)"""
        reach = self.active_reach()
        if reach is None or not len(reach[1]):
            return p
        axis, rz = reach
        dx, dy = p[0] - axis[0], p[1] - axis[1]
        r = math.hypot(dx, dy)
        az = math.atan2(dy, dx) if r > 1e-6 else 0.0
        near = np.min((rz[:, 0] - r) ** 2 + (rz[:, 1] - p[2]) ** 2) <= 0.01 ** 2
        if near and p[2] >= float(self.body_floor(p[0], p[1], p[2])):
            return p
        cx, cy = axis[0] + rz[:, 0] * math.cos(az), axis[1] + rz[:, 0] * math.sin(az)
        ok = rz[:, 1] >= self.body_floor(cx, cy, rz[:, 1])
        if not ok.any():
            return p
        d = np.where(ok, (rz[:, 0] - r) ** 2 + (rz[:, 1] - p[2]) ** 2, np.inf)
        i = int(np.argmin(d))
        return np.array([cx[i], cy[i], rz[i, 1]])

    def draw_reach(self, img):
        """손끝이 닿는 범위 — **지금 팔이 향한 방향(joint1 실측)의 수직 단면**: 그 방향에서 손끝(아래 방향)이 갈 수 있는 (거리, 높이)
        영역을 옅은 청록으로 채우고 테두리. 팔이 돌면 같이 돈다. 몸체(차체·적재함·폴)와 겹치는 칸은 뺌.
        끄는 중엔 지금 손끝 목표 높이의 닿는 고리(360°)를 진하게 — 그 높이에서 팔을 돌리면 어디까지 가는지"""
        reach = self.active_reach()
        if reach is None or not len(reach[1]):
            return
        axis, rz = reach
        q, _ = self.actual()
        az = float((q or {}).get('joint1', 0.0))  # joint1 = 0이면 +x(앞) — 팔 회전축 기준 방위
        overlay = img.copy()
        # 두 범위를 같이: 지금 모드 범위 = 채움 · 다른 모드 범위 = 외곽선만 (집기 자세 ⊂ 전체)
        free = self.free_orient.get()
        other = self.reach if free else getattr(self, 'reach_full', None)
        labels = ('전체 (방향 자유)', '집기 자세 (손가락 아래)') if free else ('집기 자세 (손가락 아래)', '전체 (방향 자유)')
        for rr, filled, label in ((other, False, labels[1]), (reach, True, labels[0])):
            pts = self.reach_section(rr, az)
            if pts is None:
                continue
            if filled:
                cv2.fillPoly(overlay, [pts], (90, 110, 60), cv2.LINE_AA)
                cv2.polylines(overlay, [pts], True, (170, 200, 110), 1, cv2.LINE_AA)
            else:
                cv2.polylines(overlay, [pts], True, (150, 170, 120), 1, cv2.LINE_AA)
            # 라벨: 채운 범위는 가운데 · 외곽선은 가장 오른쪽 끝 옆 (화면 안으로)
            at = pts.mean(0) if filled else pts[np.argmax(pts[:, 0])]
            ax_, ay_ = int(min(max(at[0] - 40, 4), VIEW[0] - 150)), int(min(max(at[1], 14), VIEW[1] - 6))
            cv2.putText(overlay, label, (ax_, ay_), cv2.FONT_HERSHEY_SIMPLEX, 0.36,
                        (190, 220, 140) if filled else (150, 170, 120), 1, cv2.LINE_AA)
        if self.goal is not None and self.goal_drag:  # 끄는 높이의 고리 — 팔을 돌리면 갈 수 있는 곳
            z = round(float(self.goal[2]) / 0.01) * 0.01
            row = rz[np.abs(rz[:, 1] - z) < 0.005]
            if len(row):
                ring = np.linspace(-math.pi, math.pi, 73)
                for r in (row[:, 0].min(), row[:, 0].max()):
                    pts3 = np.stack([axis[0] + r * np.cos(ring), axis[1] + r * np.sin(ring), np.full_like(ring, z)], -1)
                    good = z >= self.body_floor(pts3[:, 0], pts3[:, 1], pts3[:, 2])
                    uv, zc = self.arm_cam.project(pts3, VIEW)
                    good &= zc > 0.02
                    run = []
                    for pt, g in zip(uv.astype(np.int32), good):
                        if g:
                            run.append(pt)
                            continue
                        if len(run) > 1:
                            cv2.polylines(overlay, [np.array(run)], False, (200, 230, 120), 2, cv2.LINE_AA)
                        run = []
                    if len(run) > 1:
                        cv2.polylines(overlay, [np.array(run)], False, (200, 230, 120), 2, cv2.LINE_AA)
        cv2.addWeighted(overlay, 0.55, img, 0.45, 0, img)

    def reach_section(self, reach, az):
        """닿는 범위의 방위 az 수직 단면 → 화면 다각형 (몸체와 겹치는 칸은 뺌) 또는 None"""
        if reach is None or not len(reach[1]):
            return None
        axis, rz = reach
        cx, cy = axis[0] + rz[:, 0] * math.cos(az), axis[1] + rz[:, 0] * math.sin(az)
        cells = rz[rz[:, 1] >= self.body_floor(cx, cy, rz[:, 1])]
        if not len(cells):
            return None
        zs = np.unique(np.round(cells[:, 1], 3))
        outer = [(cells[np.isclose(cells[:, 1], z), 0].max(), z) for z in zs]
        inner = [(cells[np.isclose(cells[:, 1], z), 0].min(), z) for z in zs[::-1]]
        poly = np.array([[axis[0] + r * math.cos(az), axis[1] + r * math.sin(az), z] for r, z in outer + inner])
        uv, zc = self.arm_cam.project(poly, VIEW)
        return uv.astype(np.int32) if (zc > 0.02).all() else None

    def drag_goal(self, e):
        """손끝 목표 끌기 — 커서를 따라감 (수평: 목표 높이의 수평면 · Shift: 높이). Shift는 끄는 도중에 눌러도 바로 바뀜"""
        self.goal_drag = 'z' if shift_held(e) else 'xy'
        if self.goal_drag == 'z':  # 높이: 위로 끌면 올라감 (0.002 m/px)
            self.goal[2] = max(0.0, self.goal[2] - (e.y - self.drag_from[1]) * 0.002)
        else:
            eye, d = self.view_ray(e.x, e.y)
            if abs(d[2]) > 1e-6:
                t = (self.goal[2] - eye[2]) / d[2]
                if t > 0:
                    self.goal[:2] = (eye + d * t)[:2]
        self.drag_from = (e.x, e.y)
        # 닿는 곳까지만 — 단 범위 밖(예: 팔을 든 자세, 집기 범위 밖)에서 잡았으면 범위에 들어올 때까지는 안 묶음
        # (예전엔 잡자마자 목표가 범위 안 가장 가까운 칸으로 튀었음)
        if self.in_reach(self.goal):
            self.goal_entered = True
        if getattr(self, 'goal_entered', True):
            self.goal = self.clamp_reach(self.goal)

    def on_arm_press(self, e):
        self.drag_from = (e.x, e.y)
        self.selected = None
        self.goal_drag = None
        if self.near_point(self.goal, e.x, e.y) or self.near_point(self.ee_now(), e.x, e.y):
            # 손끝 잡기 — 끌어 둔 목표가 아니라 지금 손끝을 잡았으면 목표를 지금 손끝 자리에서 시작
            if not self.near_point(self.goal, e.x, e.y):
                self.goal = self.ee_now().copy()
            self.goal_entered = self.in_reach(self.goal)
            self.goal_drag = 'z' if shift_held(e) else 'xy'
            return
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
        if self.goal_drag:
            self.drag_goal(e)
            now = time.monotonic()
            if now - getattr(self, '_adrag_t', 0) >= 0.07:
                self._adrag_t = now
                self.redraw_goal()
            return
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

    def on_arm_release(self, e):
        if self.selected:
            self.send_joints()
        if self.goal_drag:
            self.goal_drag = None
            self.redraw_goal()
        self.selected = None

    def render_arm(self, q):
        """팔 3D 한 장 — 목표·지금이 바뀌었을 때만 호출 (틱에서 매번 그리지 않음)"""
        model = self.model_col if self.sim.show_collision and self.model_col else self.model
        # 지금 — 몸체(차체·적재함·폴·바퀴)는 팔보다 더 어둡게(갈색 기운) 구분, 팔은 회색 + 주변 물체 (본창 상태 3D처럼, 로봇 기준으로)
        body = {link for link, *_ in model.visuals} - self.arm_links
        faces = model.faces(q, body, tint=(48, 52, 62)) + model.faces(q, self.arm_links, tint=(80, 80, 90))
        x, y, yaw, _ = self.sim.robot_xy_yaw()
        T_wb = scene.transform(scene.rpy_matrix(0, 0, yaw), (x, y, 0))
        if self.object_faces is not None:
            faces += self.object_faces(T_wb)
        arm = self.arm_cam.render(faces, VIEW)
        robot_view.draw_wheel_spin(arm, model, self.arm_cam, VIEW, q, self.wheel_vel)  # 바퀴 굴러가는 것 (스포크)
        if self.detected_var is not None and self.detected_var.get():  # 감지 물체 (보라 상자 · 카메라별 점)
            draw_detected_3d(arm, self.arm_cam, VIEW, self.base_map() if self.base_map else
                             (x - self.sim.spawn[0], y - self.sim.spawn[1], yaw))
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
        self.draw_reach(arm)
        ee = self.ee_now()
        if ee is not None:  # 지금 손끝 손잡이 (잡고 끌면 손끝 목표가 여기서 시작)
            uv, z = self.arm_cam.project([ee], VIEW)
            if z[0] > 0.02:
                c = tuple(uv[0].astype(int))
                cv2.circle(arm, c, 9, (255, 220, 120), 1, cv2.LINE_AA)
                cv2.circle(arm, c, 3, (255, 220, 120), -1, cv2.LINE_AA)
                if self.goal is None:
                    cv2.putText(arm, 'drag ee', (c[0] + 10, c[1] + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 220, 120), 1, cv2.LINE_AA)
        if self.goal is not None:  # 손끝 목표 — 하늘색 원 + 좌표축 + 바닥까지 세로선(높이 감)
            g = self.goal
            pts = [g, [g[0], g[1], 0.0], g + [0.04, 0, 0], g + [0, 0.04, 0], g + [0, 0, 0.04]]
            uv, z = self.arm_cam.project(pts, VIEW)
            if (z > 0.02).all():
                p = uv.astype(int)
                cv2.line(arm, tuple(p[0]), tuple(p[1]), (150, 130, 90), 1, cv2.LINE_AA)
                cv2.circle(arm, tuple(p[1]), 3, (150, 130, 90), -1, cv2.LINE_AA)
                for i, col in enumerate([(0, 0, 255), (0, 255, 0), (255, 0, 0)]):
                    cv2.line(arm, tuple(p[0]), tuple(p[2 + i]), col, 2, cv2.LINE_AA)
                cv2.circle(arm, tuple(p[0]), 10 if self.goal_drag else 8, (255, 220, 120), 2, cv2.LINE_AA)
                if self.finger_fix.get():  # 손가락이 놓일 선 (벌렸을 때 폭) — 퍽 방향에 맞추기
                    a = math.radians(float(self.finger_deg.get()))
                    d = np.array([math.cos(a), math.sin(a), 0.0]) * 0.065
                    fu, fz = self.arm_cam.project([g - d, g + d], VIEW)
                    if (fz > 0.02).all():
                        fp = fu.astype(int)
                        cv2.line(arm, tuple(fp[0]), tuple(fp[1]), (120, 180, 255), 2, cv2.LINE_AA)
                        for c in fp:
                            cv2.circle(arm, tuple(c), 4, (120, 180, 255), -1, cv2.LINE_AA)
                cv2.putText(arm, f'ee z{g[2]:.2f}', (p[0][0] + 11, p[0][1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 220, 120), 1, cv2.LINE_AA)
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
        char = getattr(e, 'char', '') or ''
        if d is None and char and not char.isascii():
            # 한글 입력기 등 keysym이 어긋나면 물리 키코드로 (자판 배열 무관). 글자가 들어온 키만 —
            # Alt·Cmd·Tab 같은 키는 글자가 없고 macOS에선 키코드 0(= A 자리)으로 와서 회전 명령이 될 뻔했음
            d = self.KEYS.get(self._PHYSICAL_KEYS.get(sys.platform, {}).get(getattr(e, 'keycode', None)))
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
            self.reach = self.reach_full = self.ik_table = None  # 닿는 범위·IK 표는 모델마다 — 백그라운드로 다시 계산 (약 2초)
            threading.Thread(target=self.build_reach, args=(self.model,), kwargs={'desc': desc['data']}, daemon=True).start()
            if first:
                self.build_sliders()
        q, wheel_vel = self.actual()
        self.wheel_vel = wheel_vel
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
        if time.time() < getattr(self, 'follow_actual_until', 0) and q and self.slider_held is None and not self.selected:
            self.target.update({j: q[j] for j in ARM_JOINTS if j in q and j != 'gripper_left_joint'})
        self.sync_sliders()
        self.refresh_motor_errors()
        self.poll_ee()
        self.poll_seq()
        if self.model:
            with self.sim.lock:  # 주변 물체·로봇이 움직여도 다시 그림
                things = tuple((round(p['x'], 2), round(p['y'], 2), round(float(p.get('z', 0.0)), 2))
                               for p in self.sim.pucks + self.sim.props)
            rx, ry, ryaw, _ = self.sim.robot_xy_yaw()
            key = (self.sim.show_collision, self.frame_var.get(), things, (round(rx, 2), round(ry, 2), round(ryaw, 2)),
                     tuple(round(self.target.get(j, 0.0), 3) for j in ARM_JOINTS),
                   tuple(round(q.get(j, 0.0), 3) for j in ARM_JOINTS),
                   # 바퀴가 돌면(관절각 · 속도) · 감지 물체가 바뀌면 다시 그림
                   tuple(round(q.get(j, 0.0), 1) for j in ('wheel_left_joint', 'wheel_right_joint')),
                   tuple(abs(w) > 0.05 for w in wheel_vel),
                   detected_key() if self.detected_var.get() else None)
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
