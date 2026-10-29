"""SIL 시뮬레이터 (호스트에서 실행) — 컨테이너(ROS)와는 공유 폴더 lv2_module6/debug 의 파일로만 주고받는다 (호스트에 ROS 불필요)

  [지도: 미로 · 실제 로봇 · /odom · 경로 · 인지 물체 · 점구름 · 퍽 · 장애물]   │ [pole_front_cam:    SIM 영상 │ SIM 깊이 ‖ ROS 영상+인지 │ ROS 점구름]
  [퍽 · 배경 · 카메라 송출]                                                  │ [gripper_cam: SIM 영상 │ SIM 깊이 ‖ ROS 영상+인지 │ ROS 점구름]
  [환경: 미끄럼 · 마찰 · 이동물체 추가 · 바닥 장애물 · 신호등 · 통신 끊기 · 팔 붙잡기]   │ [상태 · 진단(/diagnostics_agg)]
  [수동 조작 탭 · 비상정지]                                             │ [명령 입력 → /cmd/user_command]

SIL(Software-in-the-Loop): 실기 대신 이 시뮬레이터가 세상(코스·퍽·장애물·이동물체·카메라 장면·로봇 몸)을 만들고,
ROS 쪽은 실제 노드·컨트롤러가 그대로 돈다.
  ① 카메라: 로봇 자세로 두 카메라 시점 렌더(scene.py) → debug/input/hardware/camera/ → fake_camera → RealSense 토픽
  ② 하드웨어(opencr.py): OpenCR 역할은 컨테이너 fake_opencr 하나 (faker 기본) — 시뮬레이터는 시리얼에 쓰지 않는다.
     시리얼은 '시리얼' 장비 창이 읽어서 보여 주고, 진짜 위치(/fake_opencr/ground_truth)로 카메라를 그리며,
     환경(미끄럼·마찰·통신 끊김·팔 붙잡기)은 파라미터, 위치 옮기기는 /cmd/fake_opencr/set_pose 토픽으로 요청한다.
     → 지도의 '실제 로봇'(채움)과 /odom(테두리)이 미끄럼만큼 어긋난다.  faker mock_hardware:=true면 /odom을 따라 그린다.
  ③ ROS → 화면: monitor_manager가 debug/output/topic/<토픽>/ 에 저장한 것을 표시
  ④ 화면 → ROS: input_manager가 debug/input/topic/<토픽>/message 를 발행 (명령 · 수동 조작 · 비상정지)
- 지도: 퍽(정답, 채움 — 왼쪽 드래그로 이동) · 바닥 장애물(회색 상자) · 이동물체(파랑, 움직임) · 인지 물체(/tracked_objects, 보라 테두리)
- 감지 물체: 하단 [감지 물체 보기] 버튼 → 별도 창 (objects_window.py) — 두 카메라 융합 트랙 · 카메라별 검출 · 2σ 확대도
- 수동 조작: 하단 [수동 조작] 버튼 → 별도 창 (manual_window.py) — 창을 열면 수동 모드, 닫으면 바퀴 정지 + 자동 모드
             바퀴 버튼(앞·뒤·멈춤·좌/우 바퀴 앞·뒤·전체 속도 ▲▼) · 로봇 3D · 팔 3D(관절 마커 드래그)
             [비상정지] → /cmd/e_stop → state_manager /state/emergency (twist_mux 잠금: 모든 바퀴 명령 차단)
- 배경: images/background/*.jpg (360° 파노라마, Poly Haven CC0 — PANORAMA_CREDITS.txt)

  컨테이너:  test-faker 0                        (fake_opencr가 OpenCR 역할 — 미끄럼·충돌·통신 끊김까지)
             test-faker 0 mock_hardware:=true    (OpenCR 없이 mock — 물리 없음)
  호스트:    cd lv2_module6/simulator
             python3 -m venv .venv && .venv/bin/pip install -r requirements.txt   # 최초 1회
             .venv/bin/python main.py
             .venv/bin/python main.py --snapshot out.png              # 창 없이 화면 한 장 (확인용)
"""
import argparse
import base64
import json
import math
import os
import random
import threading
import time
from concurrent.futures import ThreadPoolExecutor

from pathlib import Path

# OpenBLAS 스레드풀 폭주 방지 (import numpy보다 먼저 — 실측: 8+ 스레드 busy-spin으로 700%)
os.environ.setdefault('OPENBLAS_NUM_THREADS', '2')
os.environ.setdefault('OMP_NUM_THREADS', '2')
os.environ.setdefault('VECLIB_MAXIMUM_THREADS', '2')
import cv2
import numpy as np

import dice
import maze
import opencr
import robot_view
import scene
from perception_view import CAM_COLORS, camera_detections, draw_detected_3d, fused_ids
from bridge import (DEBUG, FileTail, LIVE_DIR, TOPIC_DIR, cancel_action_goal, cmd_manual_mode, current_joints,
                    read_action_file, read_grid, read_msg, read_points, read_service_response, read_topic_list, send, send_action_goal,
                    send_service_request,
                    write_atomic, yaw_of)
try:  # 창(GUI) — 컨테이너(--headless)엔 tkinter가 없다
    import tkinter as tk
    from tkinter import ttk

    from manual_window import ManualPanel, shift_held
except ImportError:
    tk = ttk = ManualPanel = None

HERE = Path(__file__).resolve().parent
BG_DIR = HERE / 'images' / 'background'
FLOOR_DIR = HERE / 'images' / 'floor'
WALL_DIR = HERE / 'images' / 'wall'
CAMERAS = ('pole_front_cam', 'gripper_cam')
ROS_FRESH_S = 2.0   # ROS 칸: 파일 수정 시각이 이 안이면 '지금 받은 것', 넘으면 흐리게 + STALE (monitor_manager 저장 간격 0.2초)
STALE_HIDE_S = 60.0  # 이보다 오래된 파일은 이전 실행 잔재로 보고 안 그림
TILE = (208, 156)  # 카메라 타일 (2열 × 4줄 × 2구역 — 창 가로를 줄이려고 작게)
ROBOT_VIEW = (320, 260)
MAP_SCALE = 0.7              # 지도 축소율 (낮은 해상도에서 잘림 방지)
STALE_S = 2.0
PX_PER_M = 110
MARGIN = 30
STATE_NAMES = {0: 'RUNNING', 1: 'SUCCEEDED', 2: 'FAILED'}
LEVEL = {0: ('OK', ''), 1: ('WARN', 'warn'), 2: ('ERROR', 'error'), 3: ('STALE', 'warn')}

FLIP_PITCH = math.radians(60)  # 집을 때와 놓을 때 손 기울기가 이만큼 다르면 퍽 자세가 바뀜 (누움 ↔ 섬)
MAP_DIR = '/ws/map'  # 지도 폴더 (컨테이너 안 — 호스트 ros2_ws/map). 지도 한 벌 = <이름>/ rtabmap.db · map.pgm · map.yaml
NO_BG = '(격자)'
NO_FLOOR = '(격자)'
NO_WALL = '(단색)'
GRIP_R = 0.03            # 그리퍼 집기 반경 [m] — 손끝 이 안에 퍽이 있고 닫히면 attach
GRIP_CLOSED = 0.003      # 이보다 오므라들면 집기 (URDF: 0=닫힘 · 0.0265=열림 — 닫기 버튼은 0.0)
GRIP_OPEN = 0.015        # 이보다 벌어지면 놓음 [m] (열기 버튼은 0.026)
GRIP_Z = 0.15            # (사용 안 함 — 아래 수직 창으로 대체, 호환용 유지)
VERT_ABOVE = 0.06        # 손끝이 물체 윗면보다 이만큼 위까지는 접촉으로 침
VERT_BELOW = 0.03        # 손끝이 물체 아랫면보다 이만큼 아래까지는 접촉으로 침
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
# 적재함 (URDF tray_link과 같은 치수 — base_footprint 기준 [m], x 앞 · z 바닥)
TRAY = {'x0': -0.19, 'x1': -0.07, 'y': 0.10, 'surface_z': 0.150, 'wall_t': 0.005}
# 몸통 받침 (base_footprint 기준 — 트레이·데크가 우선, 나머지는 몸통 윗면)
BODY_TOP = {'x0': -0.197, 'x1': 0.069, 'y': 0.133, 'z': 0.145}
DECK_TOP = {'x0': -0.064, 'x1': 0.069, 'y': 0.133, 'z': 0.200}


def _support_at(lx, ly):
    """base 기준 받침 높이 — 트레이 > 데크 > 몸통, 없으면 None(바닥)"""
    if TRAY['x0'] <= lx <= TRAY['x1'] and abs(ly) <= TRAY['y']:
        return TRAY['surface_z']
    if DECK_TOP['x0'] <= lx <= DECK_TOP['x1'] and abs(ly) <= DECK_TOP['y']:
        return DECK_TOP['z']
    if BODY_TOP['x0'] <= lx <= BODY_TOP['x1'] and abs(ly) <= BODY_TOP['y']:
        return BODY_TOP['z']
    return None


def _to_base(px, py, bx, by, byaw):
    """world → base_footprint 2D"""
    dx, dy = px - bx, py - by
    c, s = math.cos(byaw), math.sin(byaw)
    return (c * dx + s * dy, -s * dx + c * dy)


def _to_world(lx, ly, bx, by, byaw):
    """base_footprint → world 2D"""
    c, s = math.cos(byaw), math.sin(byaw)
    return (bx + c * lx - s * ly, by + s * lx + c * ly)


def _tray_reach(p):
    if p['shape'] == 'cylinder':
        return p['size'][0]
    if p['shape'] == 'sphere':
        return p['size'][0]
    return max(p.get('sx', p['size'][0]), p.get('sy', p['size'][0])) / 2
PUSH_R = 0.04            # 팔이 물체를 미는 반경 [m] — 손끝이 닿고 팔이 움직이면 밀림 (베이스 이동분 제외)
PUSH_K = 1.0             # 미는 속도 = 손끝 속도 × K
SHOVE_MARGIN = 0.005     # 바퀴가 퍽을 밀 때 남기는 여유 [m]


# ---------- 시뮬레이션 상태
class Sim:
    def __init__(self, course, spawn=None):
        self.course = course
        self.spawn = spawn or course['spawn']
        self.walls = np.asarray(maze.wall_boxes(course), float)
        self.renderer = scene.Renderer(self.walls)
        self.cells = scene.maze_cells(course)
        self.goal = scene.goal_zone(course, self.cells)  # B 지역 = 고정 목표 구역 (카메라 렌더·지도·데이터셋 공용)
        if 'mission' in course:  # 주사위 맵엔 B 지역이 없음 — ponytail: 경기장 밖으로 치움 (goal을 쓰는 곳이 많아 지우진 않음)
            self.goal.update(x=-50.0, y=-50.0)
        self.faces = []           # 장소 태그 면 (dice.py — 무늬만 입힘)
        self.dice_problem = None  # 주사위 문제 (dice/<이름>.json)
        self.stations = []        # 미션 지역 [station_zone dict + 'problem'] — 지도에서 찍어 놓음
        self.kin = None
        self.body = None
        self.render_ts = {}          # 카메라 → 마지막 렌더 시각 (화면이 SIM 직결인지 판단)
        self.pool = ThreadPoolExecutor(max_workers=len(CAMERAS))
        self.lock = threading.RLock()  # 물체 목록·렌더 공유 (재진입 가능)
        self.pucks = []
        self.props = []           # 바닥 장애물 · 이동물체
        self.renders = {}         # 카메라 → (bgr, depth, T_world_cam)
        self.streaming = True
        self.running = True
        self.opencr = opencr.OpenCR(self.spawn)  # 컨테이너 fake_opencr 연결 (읽기·요청만 — 시리얼에 안 씀)
        # 미션 (documents/mission/README.md) — 배치 · 지정 퍽 · 미끄러운 구간 · 조명 (채점은 외부)
        self.env = None           # {'seed', 't0'} — [환경 배치] 후 (장애물·이동물체·미끄러운 구간·조명 변화)
        self.slip_zones = []      # [{'x','y','yaw','sx','sy','slip'}] — 로봇이 안에 있으면 그 미끄럼
        self.base_slip = 0.0      # 구간 밖 미끄럼 (창의 슬라이더)
        self.light = 1.0          # 카메라 밝기 배율 (조명 변화)
        self.cam_noise = 0.0      # 카메라 잡음 표준편차 [밝기 0~255]
        self.light_change = False  # 환경 배치 중 조명이 천천히 오르내림
        self.show_collision = False  # 충돌 모양 보기 — 카메라 렌더·로봇 3D를 <collision>으로
        # 처음엔 빈 맵 — 미션 지역을 찍거나 [퍽 랜덤 배치]로 채운다

    # --- 주사위 미션 (dice.py)
    def load_dice(self, name):
        """문제 깔기 — 퍽 12개 · 신호등 · 바코드 판 · 장소 태그 (다른 퍽·물체는 지움)"""
        pucks, props, faces, prob = dice.setup(self.course, name)
        with self.lock:
            self.pucks, self.props, self.faces, self.dice_problem = pucks, props, faces, {**prob, 'name': name}

    def start_signal(self):
        """운영자 [신호 시작] — 12회 점멸 시퀀스를 처음부터"""
        with self.lock:
            for p in self.props:
                if p.get('sequence') is not None:
                    p['seq_t'] = 0.0

    def signal_text(self):
        light = next((p for p in self.props if p.get('sequence') is not None), None)
        if light is None:
            return '신호등 없음'
        if light['seq_t'] is None:
            return '신호 대기'
        n = min(int(light['seq_t'] // 2) + 1, len(light['sequence']))
        return f"신호 {n}/{len(light['sequence'])} {light['state']}" if light['seq_t'] < 2 * len(light['sequence']) else '신호 끝 (소등)'

    def box_top(self, x, y):
        """(x, y) 아래 낮은 박스(시작·장소 박스 70mm) 윗면 높이 · 없으면 None"""
        for cx, cy, _, sx, sy, sz in self.walls:
            if sz < 0.1 and abs(x - cx) <= sx / 2 and abs(y - cy) <= sy / 2:
                return float(sz)
        return None

    # --- 물체
    def randomize(self, count):
        """미션 지역 밖 퍽을 count개 무작위로 다시 깐다 (미션 지역 퍽은 그대로)"""
        with self.lock:
            kept = [p for p in self.pucks if p.get('station')]
            loose = scene.random_pucks(count, self.cells, self.spawn, random.Random())
            first = max([p['id'] for p in kept] + [0]) + 1
            for k, p in enumerate(loose):
                p['id'] = first + k
            self.pucks = kept + loose

    def add_obstacle(self):
        x, y, _, _ = self.robot_xy_yaw()
        with self.lock:
            avoid = [(x, y), tuple(self.spawn)] + [(p['x'], p['y']) for p in self.pucks + self.props]
            ob = scene.random_obstacle(self.cells, avoid)
            if ob:
                self.props.append(ob)

    def clear_obstacles(self):
        with self.lock:
            self.props = []

    def add_puck(self):
        """퍽 하나 추가 — 빈 칸, 로봇·기존 물체와 0.35m 이상"""
        x, y, _, _ = self.robot_xy_yaw()
        with self.lock:
            avoid = [(x, y), tuple(self.spawn)] + [(p['x'], p['y']) for p in self.pucks + self.props]
            for _ in range(100):
                cand = scene.random_pucks(1, self.cells, self.spawn, random.Random())
                if not cand:
                    return
                p = cand[0]
                if all(math.dist((p['x'], p['y']), a) > 0.35 for a in avoid):
                    p['id'] = max([q.get('id', 0) for q in self.pucks] + [0]) + 1
                    self.pucks.append(p)
                    break

    def pop_puck(self):
        """퍽 하나 삭제 — 가장 나중에 추가된 것부터"""
        with self.lock:
            if self.pucks:
                self.pucks.pop()

    def pop_prop(self, kind):
        """스택식 삭제 — 같은 종류 중 가장 나중에 추가된 것부터"""
        with self.lock:
            for i in range(len(self.props) - 1, -1, -1):
                if self.props[i].get('kind') == kind:
                    del self.props[i]
                    break

    def spawn_mover(self):
        """미로 안을 돌아다니는 이동물체 — 빈 칸에서 시작해 칸 사이를 계속 이동 (장애물 지우기까지)"""
        with self.lock:
            cx, cy = random.choice(self.cells)
            p = scene.mover(cx, cy, 0.0)
            p['wander'] = True
            p['until'] = float('inf')
            self.props.append(p)
            self._retarget(p)

    def reset_robot(self):
        # 로봇 위치 초기화: fake_opencr 위치 + mock odom 리셋 + EKF 리셋 (map 원점 = spawn)
        self.opencr.set_pose(self.spawn)
        self._base_prev = None
        send_service_request('diff_drive_controller/set_odometry', {'x': 0.0, 'y': 0.0, 'yaw': 0.0})
        send_service_request('reset', {})

    def teleport_robot(self, wx, wy):
        # 지도 드래그 이동: fake_opencr가 돌면 그 위치를, mock이면 odom 리셋 서비스로
        _, _, yaw, _ = self.robot_xy_yaw()
        if self.opencr.alive:
            self.opencr.set_pose((wx, wy), yaw)
            self._base_prev = None
            # EKF는 속도만 적분해서 순간이동을 모른다 → 같이 옮기지 않으면 로봇이 아는 위치가 옛 자리에 남음
            # (perception이 그 위치로 map에 옮기므로 감지 물체가 통째로 어긋났음)
            send('/set_pose', 'geometry_msgs/msg/PoseWithCovarianceStamped', {
                'header': {'stamp': {'sec': 0, 'nanosec': 0}, 'frame_id': 'odom'},
                'pose': {'pose': {'position': {'x': round(wx - self.spawn[0], 3), 'y': round(wy - self.spawn[1], 3), 'z': 0.0},
                                  'orientation': {'x': 0.0, 'y': 0.0, 'z': round(math.sin(yaw / 2), 6),
                                                  'w': round(math.cos(yaw / 2), 6)}},
                         'covariance': [1e-4 if i in (0, 7, 35) else 0.0 for i in range(36)]}})
        else:
            send_service_request('diff_drive_controller/set_odometry',
                                 {'x': round(wx - self.spawn[0], 3),
                                  'y': round(wy - self.spawn[1], 3), 'yaw': round(yaw, 3)})

    def spawn_traffic(self):
        """신호등 랜덤 배치 — 빈 칸, 로봇·퍽·기존 물체와 0.4m 이상"""
        x, y, _, _ = self.robot_xy_yaw()
        with self.lock:
            avoid = [(x, y), tuple(self.spawn)] + [(p['x'], p['y']) for p in self.pucks + self.props]
            for _ in range(100):
                cx, cy = random.choice(self.cells)
                tx, ty = cx + random.uniform(-0.2, 0.2), cy + random.uniform(-0.2, 0.2)
                if all(math.dist((tx, ty), a) > 0.4 for a in avoid):
                    self._tl_n = getattr(self, '_tl_n', 0) + 1
                    self.props.append(scene.traffic_light(tx, ty, f'T{self._tl_n}'))
                    break

    def update_traffic(self, dt):
        """신호등 순환 green → (꺼짐) → yellow → (꺼짐) → red → (꺼짐) …"""
        with self.lock:
            changed = False
            for p in self.props:
                if p.get('kind') != 'traffic_light':
                    continue
                if p.get('sequence') is not None:  # 신호등 — [신호 시작]부터 정해진 시퀀스만
                    dice.signal_step(p, dt)
                    continue
                p['timer'] -= dt
                if p['timer'] <= 0:
                    idx = (p.get('cycle_idx', 5) + 1) % len(scene.TL_CYCLE)
                    nxt, dur = scene.TL_CYCLE[idx]
                    p['state'] = nxt
                    p['cycle_idx'] = idx
                    p['timer'] = dur
                    p['color'] = scene.TL_COLORS[nxt]
                    changed = True

    def _retarget(self, p):
        """0.5m 이상 떨어진 칸을 새 목표로 (벽에 막히면 move_people이 다시 고름)"""
        for _ in range(20):
            tx, ty = random.choice(self.cells)
            if math.dist((p['x'], p['y']), (tx, ty)) > 0.5:
                p['tx'], p['ty'] = tx, ty
                return
        p['tx'], p['ty'] = p['x'], p['y']

    def wall_hit(self, x, y, r):
        lo, hi = self.walls[:, :2] - self.walls[:, 3:5] / 2, self.walls[:, :2] + self.walls[:, 3:5] / 2
        nearest = np.clip([x, y], lo, hi)
        return bool((np.sum((nearest - [x, y]) ** 2, axis=1) < r * r).any())

    def move_people(self, dt):
        turn_max, speed = 2.0, 0.35  # 최대 회전 [rad/s] · 이동 속도 [m/s] — 급회전 없이 부드럽게
        with self.lock:
            for p in self.props:
                if p['kind'] != 'mover':
                    continue
                if p.get('wander'):
                    if math.dist((p['x'], p['y']), (p.get('tx', p['x']), p.get('ty', p['y']))) < 0.15:
                        self._retarget(p)
                    want = math.atan2(p['ty'] - p['y'], p['tx'] - p['x'])
                    got = p.get('heading', want)
                    diff = (want - got + math.pi) % (2 * math.pi) - math.pi
                    got += max(-turn_max * dt, min(turn_max * dt, diff))
                    p['heading'] = got
                    p['vx'], p['vy'] = math.cos(got) * speed, math.sin(got) * speed
                    nx, ny = p['x'] + p['vx'] * dt, p['y'] + p['vy'] * dt
                    if self.wall_hit(nx, ny, 0.2):
                        self._retarget(p)
                    else:
                        p['x'], p['y'] = nx, ny
                else:
                    p['x'] += p['vx'] * dt
                    p['y'] += p['vy'] * dt
                    p['until'] -= dt
            self.props = [p for p in self.props if p['kind'] != 'mover' or p.get('wander') or p['until'] > 0]

    # --- 충돌 (플랜트가 부름): 벽·바닥장애물·이동물체는 막힘, 퍽은 바퀴에 밀려남
    def blocked(self, x, y, r):
        # 이동 차단 여부만 (퍽 밀기 없음) - 수동 조종 예측용
        if self.wall_hit(x, y, r):
            return True
        with self.lock:
            for p in self.props:
                reach = p['size'][0] if p['shape'] == 'cylinder' else max(p.get('sx', 0.1), p.get('sy', 0.1)) / 2
                if math.dist((x, y), (p['x'], p['y'])) < r + reach:
                    return True
        return False

    def collide(self, x, y, r):
        if self.blocked(x, y, r):
            return True
        with self.lock:
            for p in self.pucks:
                if p.get('held') or 'z' in p:
                    continue  # 든 것·로봇 위 적재는 바퀴 shove 대상 아님
                reach = p['size'][0]
                d = math.dist((x, y), (p['x'], p['y']))
                if d < r + reach:  # 퍽은 막지 않고 로봇 바깥으로 밀어냄
                    if d < 1e-9:
                        dx, dy = 1.0, 0.0
                    else:
                        dx, dy = (p['x'] - x) / d, (p['y'] - y) / d
                    nx, ny = x + dx * (r + reach + SHOVE_MARGIN), y + dy * (r + reach + SHOVE_MARGIN)
                    if self.wall_hit(nx, ny, reach):
                        return True
                    p['x'], p['y'] = round(nx, 3), round(ny, 3)
        return False

    # --- 팔·물체 상호작용: 집기(attach) · 놓기(detach) · 밀기(push)
    def grip_interact(self, T_wb, q, dt):
        try:
            tip = (T_wb @ self.kin.pose('end_effector_link', q))[:3, 3]
        except KeyError:
            return
        tip = (float(tip[0]), float(tip[1]), float(tip[2]))
        # 밀기용 속도는 팔 관절 움직임분만 — 베이스가 움직여 손끝이 이동한 건 제외
        qprev = getattr(self, '_q_prev', None)
        self._q_prev = dict(q)
        if qprev and dt > 0:
            try:
                ptip = (T_wb @ self.kin.pose('end_effector_link', {**q, **{k: qprev.get(k, v) for k, v in q.items() if k in ARM_JOINTS}}))[:3, 3]
            except KeyError:
                ptip = None
            if ptip is not None:
                vx, vy, vz = (tip[0] - float(ptip[0])) / dt, (tip[1] - float(ptip[1])) / dt, (tip[2] - float(ptip[2])) / dt
            else:
                vx, vy, vz = 0.0, 0.0, 0.0
        else:
            vx, vy, vz = 0.0, 0.0, 0.0
        if math.sqrt(vx * vx + vy * vy + vz * vz) > 5.0:  # 텔레포트성 점프(리셋 등)는 무시
            vx, vy, vz = 0.0, 0.0, 0.0
        grip = q.get('gripper_left_joint', 0.0)
        with self.lock:
            held = next((p for p in self.pucks if p.get('held')), None)
            if held is not None:
                held['x'], held['y'] = round(tip[0], 3), round(tip[1], 3)
                h = scene.puck_height(held)
                held['z'] = round(max(tip[2] - h / 2, 0.0), 3)  # 손끝 높이로 들어올림
                if grip > GRIP_OPEN:  # 벌리면 놓음 — z는 settle_cargo가 판단 (트레이 위면 적재, 아니면 바닥)
                    held.pop('held', None)
                    # 자세 바꾸기 (문제 ② 누운 퍽 세우기): 집을 때보다 손을 FLIP_PITCH 넘게 기울여 놓으면 누움 ↔ 섬
                    # ponytail: 손 기울기 차이만 봄 — 실제 접촉·굴러감은 없음. 필요하면 손 축과 퍽 축 각도로 판정
                    if held['shape'] == 'cylinder' and abs(self._ee_pitch(T_wb, q) - held.pop('grip_pitch', 0.0)) > FLIP_PITCH:
                        if held.pop('lying', False) is False:
                            held['lying'] = True
                return
            near = [p for p in self.pucks
                    if math.dist((tip[0], tip[1]), (p['x'], p['y'])) < GRIP_R + p['size'][0]
                    and tip[2] <= p.get('z', 0.0) + scene.puck_height(p) + VERT_ABOVE
                    and tip[2] >= p.get('z', 0.0) - VERT_BELOW]
            if near and grip < GRIP_CLOSED:  # 닫히면 가장 가까운 퍽을 집음 (손끝 높이가 맞아야)
                pick = min(near, key=lambda p: math.dist((tip[0], tip[1]), (p['x'], p['y'])))
                pick['held'] = True
                pick.pop('on_box', None)
                pick['x'], pick['y'] = round(tip[0], 3), round(tip[1], 3)
                pick['z'] = round(max(tip[2] - scene.puck_height(pick) / 2, 0.0), 3)
                pick['grip_pitch'] = self._ee_pitch(T_wb, q)
                return
            if math.sqrt(vx * vx + vy * vy + vz * vz) > 0.05:  # 손끝이 물체를 밀면 밀려남 (퍽·바닥장애물)
                moved = False
                for p in self.pucks + [o for o in self.props if o.get('kind') == 'obstacle']:
                    reach = p['size'][0] if p['shape'] == 'cylinder' else max(p.get('sx', 0.1), p.get('sy', 0.1)) / 2
                    ptop = p.get('z', 0.0) + scene.puck_height(p)
                    if math.dist((tip[0], tip[1]), (p['x'], p['y'])) >= PUSH_R + reach:
                        continue
                    if not (p.get('z', 0.0) - VERT_BELOW <= tip[2] <= ptop + VERT_ABOVE):
                        continue  # 손끝이 물체 높이와 안 맞으면 통과 (위에서 지나감)
                    nx, ny = p['x'] + vx * PUSH_K * dt, p['y'] + vy * PUSH_K * dt
                    if not self.wall_hit(nx, ny, reach):
                        p['x'], p['y'] = round(nx, 3), round(ny, 3)
                        moved = True


    def carry_cargo(self, x, y, yaw):
        """로봇 위 퍽(z 있음, 든 것 제외)을 베이스와 함께 운반"""
        prev = getattr(self, '_base_prev', None)
        if prev is None:
            return
        px, py, pyaw = prev
        with self.lock:
            for p in self.pucks:
                if p.get('held') or 'z' not in p or p.get('on_box'):
                    continue
                lx, ly = _to_base(p['x'], p['y'], px, py, pyaw)
                wx, wy = _to_world(lx, ly, x, y, yaw)
                p['x'], p['y'] = round(wx, 3), round(wy, 3)

    def settle_cargo(self, x, y, yaw):
        """로봇 위 퍽은 받침 높이로 · 트레이 안에선 가이드벽 안쪽으로 가둠, 받침 밖은 바닥으로"""
        with self.lock:
            for p in self.pucks:
                if p.get('held') or 'z' not in p or p.get('on_box'):
                    continue
                lx, ly = _to_base(p['x'], p['y'], x, y, yaw)
                reach = _tray_reach(p)
                lo_x, hi_x = TRAY['x0'] + TRAY['wall_t'] + reach, TRAY['x1'] - TRAY['wall_t'] - reach
                lo_y, hi_y = -TRAY['y'] + TRAY['wall_t'] + reach, TRAY['y'] - TRAY['wall_t'] - reach
                if TRAY['x0'] <= lx <= TRAY['x1'] and abs(ly) <= TRAY['y']:
                    cx = min(max(lx, lo_x), hi_x)  # 가이드벽 가둠
                    cy = min(max(ly, lo_y), hi_y)
                    wx, wy = _to_world(cx, cy, x, y, yaw)
                    p['x'], p['y'], p['z'] = round(wx, 3), round(wy, 3), TRAY['surface_z']
                    continue
                s = _support_at(lx, ly)
                top = self.box_top(p['x'], p['y']) if s is None else None
                if s is not None:
                    p['z'] = s
                elif top is not None:  # 로봇 밖이고 박스 위 — 박스에 안착 (장소·시작 박스)
                    p['z'], p['on_box'] = top, True
                else:
                    del p['z']  # 받침 밖 — 바닥으로

    # --- 미션 (documents/mission/README.md): 한 맵에 미션 지역 여러 곳 — 지도에서 찍고 문제를 고른다 (채점은 외부)
    def snap_cell(self, x, y):
        """찍은 점 → 가장 가까운 칸 중심 (출발 칸에서 0.3m 안이면 None — 출발·귀환 자리)"""
        c = min(self.cells, key=lambda c: math.dist(c, (x, y)))
        return None if math.dist(c, self.spawn) < 0.3 else c

    def _board_normal(self, c):
        """QR 판 방향 — 벽이 있는 쪽에 판을 붙이고 반대(열린 쪽, 로봇이 오는 쪽)를 보게.
        맞은편도 열린 벽 쪽을 우선, 벽이 없으면 출발 칸 쪽을 봄"""
        dirs = [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)]
        walled = [d for d in dirs if self.wall_hit(c[0] + d[0] * 0.40, c[1] + d[1] * 0.40, 0.03)]
        good = [d for d in walled if (-d[0], -d[1]) not in walled] or walled
        if good:
            d = good[0]
            return (-d[0], -d[1])
        dx, dy = self.spawn[0] - c[0], self.spawn[1] - c[1]
        return (math.copysign(1, dx), 0.0) if abs(dx) >= abs(dy) else (0.0, math.copysign(1, dy))

    def _fill_station(self, st, normal):
        """미션 지역에 QR 판 + 문제에 맞는 퍽 (이 지역 것만 새로)"""
        board = scene.qr_board(st, normal, st['problem'])
        with self.lock:
            self.pucks = [p for p in self.pucks if p.get('station') != st['name']]
            self.props = [p for p in self.props if p.get('station') != st['name']]
            first = max([p.get('id', 0) for p in self.pucks] + [0]) + 1
            self.pucks += scene.station_pucks(st['problem'], st, board, random.Random(), first)
            self.props.append(board)
        st['normal'] = normal

    def add_station(self, x, y, problem):
        """(x, y) 칸에 미션 지역 — 이미 있으면 문제만 바꿔 다시 깐다. → 이름 또는 None(출발 칸 · B 지역)"""
        c = self.snap_cell(x, y)
        if c is None or math.dist(c, (self.goal['x'], self.goal['y'])) < 0.3:
            return None
        st = next((z for z in self.stations if math.dist(c, (z['x'], z['y'])) < 0.3), None)
        if st is None:
            used = {z['name'] for z in self.stations}
            name = next(f'M{i}' for i in range(1, 100) if f'M{i}' not in used)
            st = scene.station_zone(c[0], c[1], name)
            self.stations.append(st)
        st['problem'] = problem
        self._fill_station(st, self._board_normal(c))
        return st['name']

    def rotate_station(self, name):
        """QR 판을 90° 돌림 (자동 방향이 마음에 안 들 때) — 퍽도 판 앞을 비워 다시 깐다"""
        st = next((z for z in self.stations if z['name'] == name), None)
        if st:
            nx, ny = st['normal']
            self._fill_station(st, (-ny, nx))

    def remove_station(self, name):
        with self.lock:
            self.stations = [z for z in self.stations if z['name'] != name]
            self.pucks = [p for p in self.pucks if p.get('station') != name]
            self.props = [p for p in self.props if p.get('station') != name]

    def station_at(self, x, y):
        """(x, y)가 들어 있는 미션 지역 (없으면 None)"""
        return next((z for z in self.stations if scene.in_zone(z, x, y)), None)

    def zone_cell_ok(self, st, c):
        """미션 지역 st(None이면 B 지역)를 칸 c에 놓을 수 있나 — 출발 칸(c None) · B · 다른 미션 지역 칸은 안 됨"""
        if c is None:
            return False
        if st is not None and math.dist(c, (self.goal['x'], self.goal['y'])) < 0.3:
            return False
        return not any(z is not st and math.dist(c, (z['x'], z['y'])) < 0.3 for z in self.stations)

    def move_station(self, name, x, y):
        """미션 지역을 (x, y) 칸으로 — 판·QR 판·퍽을 통째로 옮긴다. 옮긴 칸에서 벽 방향이 달라지면 QR 판 방향·퍽을 다시 깐다.
        출발 칸 · B 지역 · 다른 미션 지역 칸이면 False (그대로)"""
        st = next((z for z in self.stations if z['name'] == name), None)
        c = self.snap_cell(x, y)
        if st is None or not self.zone_cell_ok(st, c):
            return False
        normal = self._board_normal(c)
        dx, dy = c[0] - st['x'], c[1] - st['y']
        with self.lock:
            st['x'], st['y'] = round(c[0], 3), round(c[1], 3)
            if tuple(normal) == tuple(st['normal']):  # 같은 방향 — 배치 그대로 평행이동
                for p in self.pucks + self.props:
                    if p.get('station') == name:
                        p['x'], p['y'] = round(p['x'] + dx, 3), round(p['y'] + dy, 3)
        if tuple(normal) != tuple(st['normal']):
            self._fill_station(st, normal)
        return True

    def move_goal(self, x, y):
        """B 지역을 (x, y) 칸으로 (출발 칸 · 미션 지역 칸은 안 됨)"""
        c = self.snap_cell(x, y)
        if not self.zone_cell_ok(None, c):
            return False
        self.goal['x'], self.goal['y'] = round(c[0], 3), round(c[1], 3)
        return True

    def setup_env(self, seed=None):
        """환경 (MISSION §2·8): 바닥 장애물 · 이동물체 · 미끄러운 구간 · 조명 변화. 출발 칸 · B · 미션 지역은 피함.
        같은 seed = 같은 배치 (미션 지역이 같을 때)"""
        lay = scene.ENV_LAYOUT
        seed = random.randrange(1 << 30) if seed is None else seed
        random.seed(seed)  # spawn_mover·spawn_traffic 이 전역 random을 씀
        rng = random.Random(seed)
        keep = [(z['x'], z['y']) for z in self.stations] + [tuple(self.spawn), (self.goal['x'], self.goal['y'])]
        with self.lock:
            self.props = [p for p in self.props if p.get('kind') == 'qr_board']
            avoid = list(keep)
            for _ in range(lay['obstacles']):
                ob = scene.random_obstacle(self.cells, avoid, rng)
                if ob:
                    self.props.append(ob)
                    avoid.append((ob['x'], ob['y']))
            # 미끄러운 구간 (0.6m 정사각, 바닥엔 안 보임 — 실제 미끄러운 바닥처럼)
            free = [c for c in self.cells if all(math.dist(c, a) > 0.5 for a in keep)]
            self.slip_zones = [{'x': cx, 'y': cy, 'yaw': 0.0, 'sx': 0.6, 'sy': 0.6, 'slip': lay['slip']}
                               for cx, cy in rng.sample(free, min(lay['slip_zones'], len(free)))]
        for _ in range(lay['movers']):
            self.spawn_mover()
        for _ in range(lay['traffic']):
            self.spawn_traffic()
        self.light_change = lay['light_change']
        self.env = {'seed': seed, 't0': time.time()}
        return seed

    def clear_env(self):
        """환경 해제 — 장애물·이동물체·신호등 지우고 미끄러운 구간·조명 변화 끔 (미션 지역은 그대로)"""
        with self.lock:
            self.props = [p for p in self.props if p.get('kind') == 'qr_board']
        self.env = None
        self.slip_zones = []
        self.light_change = False
        self.light = 1.0

    def _ee_pitch(self, T_wb, q):
        """손(end_effector_link) 앞 방향(x축)이 수평에서 기운 각 [rad] — 아래를 보면 −90°"""
        try:
            x_axis = (T_wb @ self.kin.pose('end_effector_link', q))[:3, 0]
        except KeyError:
            return 0.0
        return math.asin(max(-1.0, min(1.0, float(x_axis[2]))))

    def env_tick(self, x, y, now):
        """미끄러운 구간 안이면 그 미끄럼 · 미션 중 조명 40초 주기로 0.55 ~ 1.45배"""
        zone = next((z for z in self.slip_zones if scene.in_zone(z, x, y)), None)
        self.opencr.slip = zone['slip'] if zone else self.base_slip  # 바뀔 때만 fake_opencr에 요청
        if self.env and self.light_change:
            self.light = 1.0 + 0.45 * math.sin(2 * math.pi * (now - self.env['t0']) / 40)

    # --- 로봇 자세: fake_opencr가 돌면 진짜 위치, 아니면 /odom (mock 하드웨어). 관절은 /joint_states
    def robot_xy_yaw(self):
        js, _ = read_msg('joint_states')
        q = {n: v for n, v in zip(js['name'], js['position']) if isinstance(v, float) and math.isfinite(v)} if js else {}
        if self.opencr.alive:
            return (*self.opencr.pose(), q)
        odom, _ = read_msg('odom')
        if not odom:
            return self.spawn[0], self.spawn[1], 0.0, q
        pose = odom['pose']['pose']
        return pose['position']['x'] + self.spawn[0], pose['position']['y'] + self.spawn[1], yaw_of(pose['orientation']), q

    def step(self):
        desc, _ = read_msg('robot_description')
        if desc and desc['data'] != getattr(self, '_desc', None):  # 처음 · URDF가 바뀜 (urdf:= 바꿔 ROS 재시작)
            self._desc = desc['data']
            self.renderer._body_cache = {}  # 몸 층은 모델이 바뀌면 무효
            self.kin = scene.Kinematics(desc['data'])
            # 로봇 몸 — visual은 STL 메시 그대로, 충돌 모양 보기는 <collision> (메시 충돌체도 메시)
            split = 192 if self.renderer.dev.type == 'cpu' else 10 ** 6  # 메시 조각 (실측: CPU는 나눌수록, GPU는 안 나눌수록 빠름)
            self.body = scene.BodyModel(desc['data'], resolve=robot_view.resolve_mesh, load_mesh=robot_view.camera_mesh,
                                        max_tris=split)
            self.body_col = scene.BodyModel(desc['data'], collision=True, resolve=robot_view.resolve_mesh,
                                            load_mesh=robot_view.camera_mesh, max_tris=split)
        if self.kin is None:
            return False
        x, y, yaw, q = self.robot_xy_yaw()
        T_wb = scene.transform(scene.rpy_matrix(0, 0, yaw), (x, y, 0))
        now = time.time()
        if self.opencr.alive:  # 바퀴가 퍽을 밀어냄 — 퍽은 시뮬레이터가 갖고 있으니 진짜 위치로 여기서 (반환값 무시)
            self.collide(x, y, opencr.ROBOT_R)
        self.carry_cargo(x, y, yaw)
        self.grip_interact(T_wb, q, min(now - getattr(self, '_step_t', now), 0.5) or 1e-3)
        self.settle_cargo(x, y, yaw)
        self.env_tick(x, y, now)
        self._base_prev = (x, y, yaw)
        self._step_t = now
        # 렌더 주기: GPU 10Hz · CPU 5Hz (CPU는 렌더가 무거워 700%까지 먹던 걸 제한)
        period = 0.1 if self.renderer.dev.type != 'cpu' else 0.2
        if now - getattr(self, '_render_t', 0) < period and self.renders:
            return True  # 물리는 매번, 무거운 카메라 렌더는 5Hz로 (실측 700% → 절감)
        self._render_t = now
        with self.lock:
            things = [dict(p) for p in self.pucks + self.props] + [dict(self.goal)] + [dict(z) for z in self.stations]
        boards = [p for p in things if p.get('kind') == 'qr_board']
        faces = list(self.faces)
        kin, renderer, twb = self.kin, self.renderer, T_wb
        body = self.body_col if self.show_collision else self.body  # 충돌 모양 보기 토글
        light, noise = self.light, self.cam_noise
        # 몸 광선 추적 재사용 키 — 카메라에 보이는 몸은 관절값에만 달림 (scene.Renderer.render 설명).
        # ponytail: 바퀴는 키에서 뺌 (계속 돌아 매번 다시 하게 됨, 둥글어 돌아도 거의 같게 보임) · 0.01rad 단위
        joints = tuple(sorted((n, round(v, 2)) for n, v in q.items() if 'wheel' not in n))

        def render_one(cam):
            T = twb @ kin.pose(f'{cam}_color_optical_frame', q)
            own = f'{cam}_link'
            boxes = [(twb @ kin.pose(link, q) @ To, sx, sy, sz, rgb)
                     for link, To, (sx, sy, sz), rgb in body.boxes if link != own]
            meshes = [(twb @ kin.pose(link, q) @ To, tris, rgb)  # 렌더 중인 카메라 자체는 뺀다 (렌즈 앞 가림)
                      for link, To, tris, rgb in body.meshes if link != own]
            bgr, depth = renderer.render(T, things, boxes, meshes, body_key=(cam, body is self.body_col, joints))
            if faces:  # 표식은 640×480에서 입힘 (320 렌더에 입히면 16mm 태그·바코드 칸이 1px 아래로 뭉개져 못 읽음)
                bgr = cv2.resize(bgr, (640, 480), interpolation=cv2.INTER_LINEAR)
                depth = cv2.resize(depth, (640, 480), interpolation=cv2.INTER_NEAREST)
            for b in boards:  # QR 무늬 (조명·잡음보다 먼저 — 같이 어두워지고 지글거림)
                bgr = scene.overlay_qr(bgr, depth, T, b)
            bgr = dice.overlay(bgr, depth, T, things, faces)  # 퍽·장소 태그 · 바코드 무늬
            depth = scene.realsense_depth(depth)  # 실기 RealSense처럼 — 거리 제곱 잡음 · 경계 구멍 · 결측
            if light != 1.0 or noise > 0:  # 조명 변화 · 카메라 잡음 (depth는 그대로 — RealSense IR은 조명 영향 적음)
                f = bgr.astype(np.float32) * light
                if noise > 0:
                    f += np.random.normal(0, noise, f.shape).astype(np.float32)
                bgr = np.clip(f, 0, 255).astype(np.uint8)
            return cam, T, bgr, depth

        stamp = now  # 촬영 시각 = 이 렌더에 쓴 로봇 자세를 읽은 시각
        self.frame_seq = getattr(self, 'frame_seq', 0) + 1
        for cam, T, bgr, depth in self.pool.map(render_one, CAMERAS):
            with self.lock:
                self.renders[cam] = (bgr, depth, T)
                self.render_ts[cam] = time.time()
            if self.streaming:
                try:
                    write_frame(cam, self.frame_seq, stamp, bgr, depth)
                except OSError as e:  # 한 프레임 못 써도 시뮬레이터는 계속 (다음 프레임에서 다시)
                    print(f'카메라 프레임 쓰기 실패 ({cam}): {e}', flush=True)
        return True

    def stop_streaming(self):
        for cam in CAMERAS:  # 메타가 없으면 fake_camera는 아무것도 안 낸다 (카메라 끊김)
            for f in LIVE_DIR.glob(f'{cam}*'):
                f.unlink(missing_ok=True)

    def render_loop(self):
        t = time.time()
        while self.running:
            now = time.time()
            self.move_people(now - t)
            self.update_traffic(now - t)
            t = now
            if not self.step():
                time.sleep(0.5)  # 로봇 URDF(/robot_description) 대기 — test-faker 0 실행 중인지
            else:
                time.sleep(0.01)  # GUI 스레드에 양보 (렌더율 영향 거의 없음)


def write_frame(cam, seq, stamp, bgr, depth):
    """카메라 한 프레임 → fake_camera (debug/input/hardware/camera/)
    컬러·깊이를 번갈아 쓰는 두 벌(<cam>_<0|1>.png · <cam>_<0|1>_depth.png)에 쓰고, 마지막에 메타(<cam>.json)가 그 벌을 가리킨다.
    fake_camera는 메타가 바뀔 때만 그 벌을 읽어 한 번 발행 — 컬러와 깊이가 늘 같은 순간 · 같은 프레임 반복 없음.
    (다음 프레임은 다른 벌에 쓰므로 읽는 동안 덮어쓰지 않는다)"""
    buf = seq % 2
    color, dep = f'{cam}_{buf}.png', f'{cam}_{buf}_depth.png'
    # 메모리에서 PNG(무압축)로 만든 뒤 한 번에 쓴다 — cv2.imwrite는 조각조각 여러 번 써서
    # Docker Desktop 공유 폴더에선 장당 ~125ms, 한 번 쓰기는 ~15ms (실측)
    for name, img in ((color, bgr), (dep, depth)):
        buf = cv2.imencode('.png', img, [cv2.IMWRITE_PNG_COMPRESSION, 0])[1].tobytes()
        write_atomic(LIVE_DIR / name, lambda p, b=buf: p.write_bytes(b))
    meta = {'seq': seq, 'stamp': stamp, 'hfov_deg': scene.HFOV_DEG, 'width': int(bgr.shape[1]), 'height': int(bgr.shape[0]),
            'color': color, 'depth': dep}
    write_atomic(LIVE_DIR / f'{cam}.json', lambda p: p.write_text(json.dumps(meta)))


# ---------- 지도
class MapView:
    def __init__(self, sim):
        self.sim = sim
        self._base = None  # (base 미로좌표, 수신시각) — /tf 캐시
        self._map_odom = (0.0, 0.0, 0.0)  # 마지막으로 본 map→odom (x, y, yaw) — RTAB-Map 보정, 없으면 항등
        self.ppm = PX_PER_M * MAP_SCALE
        self.drop_hint = None  # 판 드래그 중 놓일 칸 (x, y, sx, sy, 놓을 수 있나) — 점선 사각형
        self.zoom = 1.0    # 보기 확대 (로봇 위치가 아니라 지도 보기만)
        self.cx, self.cy = 0, 0  # 확대 중심 (전체 지도 px) — viewport()에서 화면 크기로 초기화
        w, h = sim.course['size']
        self.half = (w / 2 + 0.2, h / 2 + 0.2)
        self.size = (int(2 * self.half[0] * self.ppm) + 2 * MARGIN, int(2 * self.half[1] * self.ppm) + 2 * MARGIN)
        self.walls = maze.walls_mask(sim.course, self.px_array, (self.size[1], self.size[0]))
        self.show_cloud = False  # 점구름 (로봇이 보는 장애물 점) 오버레이 — 체크박스
        self.nav_goal = None  # 오른쪽 클릭 [여기로 이동] 목표 (미로 좌표) — 진행 중일 때만 깃발
        self.show_costmap = False  # Nav2 코스트맵 (전역 + 지역) 오버레이 — 체크박스
        self.show_navmap = False  # Nav2가 보는 지도 /map 오버레이 — 체크박스

    def scan_choices(self):
        """가진 지도 폴더 = state_manager /state/map_list (로봇에 있는 지도 — 실기에 붙여도 같음). 호스트 폴더는 안 봄"""
        data, _ = read_msg('state/map_list')
        return [n for n in (data or {}).get('data', '').split('\n') if n]

    def map_to_world(self):
        """RTAB-Map map 프레임 → 미로(시뮬레이터 world) 변환 (tx, ty, th): world = R(th)·map + t.
        같은 로봇의 두 자세로 구함 — 실제 위치(fake_opencr, 없으면 /odom+출발 칸)와 로봇이 아는 map 위치(map→odom ∘ /odom).
        저장 지도를 불러오면 map 원점이 출발 칸과 다르므로 '원점 = 출발 칸' 가정을 쓰면 안 됨. /odom 없으면 출발 칸 기준"""
        # map→odom(RTAB-Map)은 /tf 파일이 마지막 메시지 하나뿐이라(EKF odom→base_footprint가 대부분) 보일 때마다 캐시
        tf, _ = read_msg('tf')
        mo = next((t for t in (tf or {}).get('transforms', []) if t.get('child_frame_id') == 'odom'), None)
        if mo:
            tr = mo['transform']['translation']
            self._map_odom = (tr['x'], tr['y'], yaw_of(mo['transform']['rotation']))
        odom, _ = read_msg('odom')
        if not odom:
            return self.sim.spawn[0], self.sim.spawn[1], 0.0
        p = odom['pose']['pose']
        ox, oy, oyaw = p['position']['x'], p['position']['y'], yaw_of(p['orientation'])
        tx, ty, th = self._map_odom
        mx, my, myaw = tx + math.cos(th) * ox - math.sin(th) * oy, ty + math.sin(th) * ox + math.cos(th) * oy, th + oyaw
        wx, wy, wyaw = self.sim.robot_xy_yaw()[:3]
        d = wyaw - myaw
        c, s_ = math.cos(d), math.sin(d)
        return wx - (c * mx - s_ * my), wy - (s_ * mx + c * my), d

    def draw_grid(self, img, topic, occupancy=False, max_age=None):
        """OccupancyGrid(monitor_manager data)를 지도에 반투명으로 — 칸 → 화면 px 아핀 변환 한 번 (칸마다 그리지 않음).
        프레임: map이면 원점 = 출발 칸, odom이면 map→odom(RTAB-Map, 없으면 항등)을 거쳐서. 2초 넘은 것은 안 그림"""
        grid, meta, age = read_grid(topic)
        if grid is None or not grid.size or (max_age is not None and age > max_age):
            return
        g = grid.astype(np.int16)
        h, w = g.shape
        color = np.zeros((h, w, 3), np.uint8)
        if occupancy:  # 지도(/map): 벽(≥65) 진회색 · 빈 곳(0~24) 연파랑 · 모름(−1)은 투명 — map_saver trinary와 같은 기준
            color[g >= 65] = (60, 60, 60)
            color[(g >= 0) & (g < 25)] = (250, 225, 200)
            mask = ((g >= 65) | ((g >= 0) & (g < 25))).astype(np.uint8)
        else:  # 코스트맵 RViz 색: 1~98 파랑→빨강 · 99 하늘 · 100 자홍
            t = np.clip((g - 1) / 97.0, 0, 1)[..., None]
            color[:] = ((1 - t) * np.array([255, 0, 0]) + t * np.array([0, 0, 255])).astype(np.uint8)
            color[g == 99] = (255, 255, 0)
            color[g == 100] = (255, 0, 255)
            mask = ((g > 0) & (g <= 100)).astype(np.uint8)
        res, (ox, oy, oyaw) = meta['resolution'], meta['origin']
        tx, ty, th = self._map_odom if meta.get('frame') == 'odom' else (0.0, 0.0, 0.0)  # odom 격자는 map으로 먼저
        wx0, wy0, wth = self.map_to_world()

        def to_px(c, r):  # 칸 (열, 행) 모서리 → 화면 px (격자 → map → 미로)
            gx, gy = ox + c * res * math.cos(oyaw) - r * res * math.sin(oyaw), oy + c * res * math.sin(oyaw) + r * res * math.cos(oyaw)
            mx, my = tx + gx * math.cos(th) - gy * math.sin(th), ty + gx * math.sin(th) + gy * math.cos(th)
            return self.px_array(np.array([wx0 + mx * math.cos(wth) - my * math.sin(wth),
                                           wy0 + mx * math.sin(wth) + my * math.cos(wth)])).astype(np.float32)

        A = cv2.getAffineTransform(np.float32([[0, 0], [1, 0], [0, 1]]), np.stack([to_px(0, 0), to_px(1, 0), to_px(0, 1)]))
        size = (img.shape[1], img.shape[0])
        warped = cv2.warpAffine(color, A, size, flags=cv2.INTER_NEAREST)
        m = cv2.warpAffine(mask, A, size, flags=cv2.INTER_NEAREST).astype(bool)
        img[m] = (img[m] * 0.45 + warped[m] * 0.55).astype(np.uint8)

    def px_array(self, xy):
        return np.stack([MARGIN + (xy[..., 0] + self.half[0]) * self.ppm,
                         MARGIN + (self.half[1] - xy[..., 1]) * self.ppm], -1).astype(np.int32)

    def px(self, x, y):
        return (int(MARGIN + (x + self.half[0]) * self.ppm), int(MARGIN + (self.half[1] - y) * self.ppm))

    def world(self, u, v):
        u, v = self._to_full(u, v)
        return ((u - MARGIN) / self.ppm - self.half[0], self.half[1] - (v - MARGIN) / self.ppm)

    def _roi(self):
        """확대 표시 영역 (전체 지도 px: x0, y0, w, h) — zoom >= 1 전용"""
        h, w = self.size[1], self.size[0]
        if self.cx == 0 and self.cy == 0:
            self.cx, self.cy = w / 2, h / 2
        rw, rh = w / self.zoom, h / self.zoom
        return (min(max(self.cx - rw / 2, 0), w - rw), min(max(self.cy - rh / 2, 0), h - rh), rw, rh)

    def _to_full(self, u, v):
        """라벨 px → 전체 지도 px (확대·축소 역변환)"""
        if self.zoom >= 1.0:
            if self.zoom <= 1.0:
                return u, v
            x0, y0, _, _ = self._roi()
            return x0 + u / self.zoom, y0 + v / self.zoom
        h, w = self.size[1], self.size[0]
        sw, sh = w * self.zoom, h * self.zoom
        return (u - (w - sw) / 2) / self.zoom, (v - (h - sh) / 2) / self.zoom

    def viewport(self, img):
        """전체 지도 → 화면 표시 (확대 잘라내기 · 축소 여백)"""
        h, w = img.shape[:2]
        if self.zoom >= 1.0:
            if self.zoom <= 1.0:
                return img
            x0, y0, rw, rh = self._roi()
            roi = img[int(y0):int(y0 + rh), int(x0):int(x0 + rw)]
            return cv2.resize(roi, (w, h), interpolation=cv2.INTER_LINEAR)
        sw, sh = max(1, int(w * self.zoom)), max(1, int(h * self.zoom))
        small = cv2.resize(img, (sw, sh), interpolation=cv2.INTER_AREA)
        canvas = np.full_like(img, 230)
        x0, y0 = (w - sw) // 2, (h - sh) // 2
        canvas[y0:y0 + sh, x0:x0 + sw] = small
        return canvas

    def _to_screen(self, fx, fy):
        """전체 지도 px → 라벨 px (viewport 정변환)"""
        h, w = self.size[1], self.size[0]
        if self.zoom >= 1.0:
            if self.zoom <= 1.0:
                return fx, fy
            x0, y0, _, _ = self._roi()
            return (fx - x0) * self.zoom, (fy - y0) * self.zoom
        sw, sh = w * self.zoom, h * self.zoom
        return (w - sw) / 2 + fx * self.zoom, (h - sh) / 2 + fy * self.zoom

    def rulers(self, img):
        """화면 가장자리 눈금 (0.5m 틱 · 1m 라벨) — 화면 기준이라 확대·축소해도 읽힘"""
        h, w = img.shape[:2]
        for u in range(0, w, 3):
            x, _ = self.world(u, h - 1)
            m = round(x * 2) / 2  # 0.5m 단위
            su, _ = self._to_screen(*self.px(m, 0))
            if abs(su - u) > 2:
                continue
            big = abs(m - round(m)) < 1e-9
            cv2.line(img, (u, h - 1), (u, h - (12 if big else 7)), (60, 60, 60), 1)
            if big:
                cv2.putText(img, f'{m:g}', (u + 2, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (60, 60, 60), 1)
        for v in range(0, h, 3):
            _, y = self.world(1, v)
            m = round(y * 2) / 2
            _, sv = self._to_screen(*self.px(0, m))
            if abs(sv - v) > 2:
                continue
            big = abs(m - round(m)) < 1e-9
            cv2.line(img, (0, v), ((12 if big else 7), v), (60, 60, 60), 1)
            if big:
                cv2.putText(img, f'{m:g}', (14, v - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (60, 60, 60), 1)
        return img

    def reset_view(self):
        h, w = self.size[1], self.size[0]
        self.zoom, self.cx, self.cy = 1.0, w / 2, h / 2

    def from_map(self, x, y):
        return x + self.sim.spawn[0], y + self.sim.spawn[1]

    def cov_ellipse(self, img, x, y, o, color, k=2.0):
        """pose.covariance 위치 x·y 블록 → k·σ 타원 (map→미로는 평행이동뿐이라 공분산은 그대로)"""
        try:
            c = o['pose']['covariance']
            P = np.array([[c[0], c[1]], [c[6], c[7]]], float)
        except (KeyError, IndexError, TypeError, ValueError):
            return
        if not np.all(np.isfinite(P)) or P[0, 0] <= 0 or P[1, 1] <= 0:
            return
        w, v = np.linalg.eigh(P)
        r = np.sqrt(np.clip(w, 0, None)) * k * self.ppm
        ang = -math.degrees(math.atan2(v[1, 1], v[0, 1]))  # 긴 축(고유값 큰 쪽) — 화면 y는 위아래 반대
        cv2.ellipse(img, self.px(x, y), (max(1, int(r[1])), max(1, int(r[0]))), ang, 0, 360, color, 1)

    def base_from_tf(self):
        """/tf에서 odom→base_footprint를 찾아 미로 좌표 base 위치. 없으면 None.
        /odom 테두리와 같은 규약 (map 원점 = spawn, map→odom 보정은 무시).
        /tf 메시지는 건당 1개 transform이라 마지막을 캐시해 깜빡임을 막는다 (2초 만료)."""
        tf, _ = read_msg('tf')
        if tf:
            ob = next((t for t in tf.get('transforms', [])
                       if t.get('child_frame_id') == 'base_footprint'), None)
            if ob:
                tr = ob['transform']['translation']
                self._base = ((tr['x'] + self.sim.spawn[0], tr['y'] + self.sim.spawn[1],
                               yaw_of(ob['transform']['rotation'])), time.time())
        if self._base and time.time() - self._base[1] < STALE_S:
            return self._base[0]
        return None

    def base_map_estimate(self):
        """로봇이 아는 자기 위치 (map x, y, yaw) = map→odom(RTAB-Map) ∘ odom→base_footprint(EKF TF) — perception이 쓰는 것.
        TF가 없으면 실제 위치(미로 − 출발 칸)로 대신"""
        base = self.base_from_tf()
        if base is None:
            x, y, yaw, _ = self.sim.robot_xy_yaw()
            return x - self.sim.spawn[0], y - self.sim.spawn[1], yaw
        ox, oy, oyaw = base[0] - self.sim.spawn[0], base[1] - self.sim.spawn[1], base[2]
        tx, ty, th = self._map_odom
        return (tx + math.cos(th) * ox - math.sin(th) * oy, ty + math.sin(th) * ox + math.cos(th) * oy, th + oyaw)

    def localization_error(self):
        """(로봇이 아는 위치 − 실제 위치) [m, rad] — 실제 위치(fake_opencr)가 없으면 None"""
        if not self.sim.opencr.alive:
            return None
        ex, ey, eyaw = self.base_map_estimate()
        x, y, yaw = self.sim.opencr.pose()
        return ex - (x - self.sim.spawn[0]), ey - (y - self.sim.spawn[1]), math.atan2(math.sin(eyaw - yaw), math.cos(eyaw - yaw))

    def dashed(self, img, a, b, color, dash=8, gap=6):
        ax, ay = self.px(*a)
        bx, by = self.px(*b)
        length = math.hypot(bx - ax, by - ay)
        if length < 4:
            return
        ux, uy = (bx - ax) / length, (by - ay) / length
        s = 0.0
        while s < length:
            e = min(s + dash, length)
            cv2.line(img, (int(ax + ux * s), int(ay + uy * s)), (int(ax + ux * e), int(ay + uy * e)), color, 1)
            s = e + gap

    def robot(self, img, x, y, yaw, color, filled):
        c = self.px(x, y)
        r = int(opencr.ROBOT_R * self.ppm)
        if filled:
            cv2.circle(img, c, r, color, -1)
            cv2.circle(img, c, r, (30, 30, 30), 1)
        else:
            cv2.circle(img, c, r, color, 2)
        cv2.line(img, c, self.px(x + 0.25 * math.cos(yaw), y + 0.25 * math.sin(yaw)), (30, 30, 30) if filled else color, 2)

    def render(self):
        img = np.full((self.size[1], self.size[0], 3), 245, np.uint8)
        for g in np.arange(-math.floor(self.half[0]), self.half[0], 0.5):  # 0.5m 격자
            cv2.line(img, self.px(g, -self.half[1]), self.px(g, self.half[1]), (225, 225, 225), 1)
        for g in np.arange(-math.floor(self.half[1]), self.half[1], 0.5):
            cv2.line(img, self.px(-self.half[0], g), self.px(self.half[0], g), (225, 225, 225), 1)
        img[self.walls > 0] = (90, 80, 70)
        sx, sy = self.px(*self.sim.spawn)
        cv2.drawMarker(img, (sx, sy), (0, 160, 0), cv2.MARKER_CROSS, 18, 2)
        cv2.putText(img, 'spawn (map 0,0)', (sx + 8, sy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 140, 0), 1)
        # 적재함 외곽 (베이스 기준 → 월드)
        if self.sim.opencr.alive:
            bx, by, byaw = self.sim.opencr.pose()
        else:
            odom0, _ = read_msg('odom')
            if odom0:
                op = odom0['pose']['pose']
                bx, by = op['position']['x'] + self.sim.spawn[0], op['position']['y'] + self.sim.spawn[1]
                byaw = yaw_of(op['orientation'])
            else:
                bx, by, byaw = None, None, None
        if bx is not None:
            corners = [(TRAY['x0'], -TRAY['y']), (TRAY['x1'], -TRAY['y']),
                       (TRAY['x1'], TRAY['y']), (TRAY['x0'], TRAY['y'])]
            tpts = np.array([self.px(*_to_world(lx, ly, bx, by, byaw)) for lx, ly in corners], np.int32)
            cv2.polylines(img, [tpts], True, (60, 60, 60), 1)

        with self.sim.lock:
            things = [dict(p) for p in self.sim.pucks + self.sim.props]
        start = {'x': self.sim.spawn[0], 'y': self.sim.spawn[1], 'yaw': 0.0, 'sx': scene.START_SIZE, 'sy': scene.START_SIZE}
        zones = [(start, 'START', (0, 160, 0)), (self.sim.goal, 'B', (200, 0, 200))] + [
            (z, f"{z['name']} {scene.PROBLEMS[z['problem']][1]}", (0, 150, 255)) for z in self.sim.stations]
        for g, label, col in zones:  # 출발·복귀(초록) · B 지역(보라) · 미션 지역(주황, 이름 + 문제)
            gpts = np.array([self.px(*q) for q in _rect(g['x'], g['y'], g['yaw'], g['sx'], g['sy'])], np.int32)
            cv2.polylines(img, [gpts], True, col, 2)
            cv2.putText(img, label, tuple(gpts[0] + [4, -4]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
        if self.drop_hint:  # 판 드래그 중: 놓일 칸 (초록 = 놓을 수 있음 · 빨강 = 안 됨, 놓으면 원위치)
            hx, hy, hsx, hsy, ok = self.drop_hint
            hpts = [self.px(*q) for q in _rect(hx, hy, 0.0, hsx, hsy)]
            col = (0, 170, 0) if ok else (0, 0, 220)
            for i in range(4):  # 점선
                a, b = np.array(hpts[i], float), np.array(hpts[(i + 1) % 4], float)
                n = max(1, int(np.linalg.norm(b - a) / 6))
                for k in range(0, n, 2):
                    cv2.line(img, tuple((a + (b - a) * k / n).astype(int)), tuple((a + (b - a) * min(k + 1, n) / n).astype(int)), col, 2)
        for z in self.sim.slip_zones:  # 미끄러운 구간 (하늘색 — 카메라엔 안 보임)
            zpts = np.array([self.px(*q) for q in _rect(z['x'], z['y'], z['yaw'], z['sx'], z['sy'])], np.int32)
            over = img.copy()
            cv2.fillPoly(over, [zpts], (240, 220, 160))
            cv2.addWeighted(over, 0.5, img, 0.5, 0, img)
            cv2.putText(img, f"slip {z['slip']:.1f}", tuple(zpts[0] + [4, -4]), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (160, 120, 40), 1)
        for p in things:
            color = scene.PALETTE.get(p['color'], (128, 128, 128)) if isinstance(p['color'], str) else tuple(p['color'])
            c = self.px(p['x'], p['y'])
            if p['shape'] == 'box':
                pts = np.array([self.px(*q) for q in _rect(p['x'], p['y'], p['yaw'], p.get('sx', p['size'][0]),
                                                           p.get('sy', p['size'][0]))], np.int32)
                cv2.fillPoly(img, [pts], color)
                cv2.polylines(img, [pts], True, (30, 30, 30), 1)
            else:
                if p.get('lying'):  # 누운 원기둥 = 위에서 보면 직사각형 (길이 = 높이, 폭 = 지름)
                    pts = np.array([self.px(*q) for q in _rect(p['x'], p['y'], p['yaw'], p['size'][1], p['size'][0] * 2)], np.int32)
                    cv2.fillPoly(img, [pts], color)
                    cv2.polylines(img, [pts], True, (30, 30, 30), 1)
                    continue
                r = max(4, int(p['size'][0] * self.ppm))
                cv2.circle(img, c, r, color, -1)
                cv2.circle(img, c, r, (30, 30, 30), 1)
                if p['shape'] == 'sphere':
                    cv2.circle(img, (c[0] - r // 3, c[1] - r // 3), max(1, r // 3), (255, 255, 255), -1)
            if 'id' in p:
                cv2.putText(img, f"#{p['id']}", (c[0] + 8, c[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (60, 60, 60), 1)
            if p.get('kind') == 'traffic_light':  # 신호등 — 상태색 링 + 상태 글자
                cv2.circle(img, c, max(8, int(0.09 * self.ppm)), (255, 255, 255), 2)
                cv2.circle(img, c, max(8, int(0.09 * self.ppm)), color, 1)
                cv2.putText(img, scene.TL_STATE_KO.get(p.get('state', ''), ''), (c[0] + 8, c[1] + 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (60, 60, 60), 1)
            if p.get('kind') == 'qr_board':  # QR 판 — 앞면 방향 화살표
                n = p['normal']
                cv2.arrowedLine(img, c, self.px(p['x'] + n[0] * 0.15, p['y'] + n[1] * 0.15), (40, 40, 40), 1, tipLength=0.3)
                cv2.putText(img, 'QR', (c[0] - 8, c[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (40, 40, 40), 1)
            if p.get('lying'):  # 누운 퍽 — 표시
                cv2.putText(img, 'lying', (c[0] + 8, c[1] + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 200), 1)
            if p.get('held'):  # 든 퍽 — 흰 테두리 + 표시
                cv2.circle(img, c, max(6, int(p['size'][0] * self.ppm) + 3), (255, 255, 255), 2)
                cv2.putText(img, 'held', (c[0] + 8, c[1] + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (60, 60, 60), 1)

        # 인지: 융합 결과(/tracked_objects, 보라 테두리 + 2σ 타원) · 융합 전 카메라별 검출(/perception/<cam>/detections,
        # 카메라 색 ✕ + 2σ 타원 → 붙은 융합 트랙까지 선). 두 카메라가 같은 트랙에 붙었으면 라벨에 F
        cams_det = camera_detections()
        both = fused_ids(cams_det)
        objects, _ = read_msg('tracked_objects')
        fused_px = {}
        for o in (objects or {}).get('objects', []):
            pos = o['pose']['pose']['position']
            x, y = self.from_map(pos['x'], pos['y'])
            hx, hy = max(o['size']['x'], 0.06) / 2, max(o['size']['y'], 0.06) / 2
            cv2.rectangle(img, self.px(x - hx, y + hy), self.px(x + hx, y - hy), (150, 0, 150), 2)
            self.cov_ellipse(img, x, y, o, (150, 0, 150))
            fused_px[o['id']] = self.px(x, y)
            cv2.putText(img, f"{o['class_name']}#{o['id']}" + (' F' if o['id'] in both else ''),
                        self.px(x + hx + 0.03, y + hy), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 0, 150), 1)
        for cam, objs in cams_det.items():
            for o in objs:
                pos = o['pose']['pose']['position']
                x, y = self.from_map(pos['x'], pos['y'])
                c = self.px(x, y)
                cv2.drawMarker(img, c, CAM_COLORS[cam], cv2.MARKER_TILTED_CROSS, 9, 1)
                self.cov_ellipse(img, x, y, o, CAM_COLORS[cam])
                if o['id'] in fused_px:
                    cv2.line(img, c, fused_px[o['id']], CAM_COLORS[cam], 1)

        # /plan = Nav2 planner_server의 전역 경로 — 주행 중엔 BT가 1초마다 다시 계산해 새로 옴. 끝나도 빈 경로를 안 내서
        # 마지막 경로 파일이 남음(이전 실행 것까지) → 3초 넘게 안 바뀌면 주행 중이 아님 → 안 그림
        path, path_age = read_msg('plan')
        if path_age is None or path_age > 3.0:
            path = None
        pts = [self.px(*self.from_map(ps['pose']['position']['x'], ps['pose']['position']['y']))
               for ps in (path or {}).get('poses', [])]
        if len(pts) > 1:
            cv2.polylines(img, [np.array(pts, np.int32)], False, (220, 120, 0), 2)
            cv2.circle(img, pts[-1], 6, (220, 120, 0), 2)

        for cam, (_, _, T) in list(self.sim.renders.items()):  # 카메라 시야
            o, f = T[:3, 3], T[:3, 2]
            for side in (-1, 1):
                a = math.atan2(f[1], f[0]) + side * math.radians(scene.HFOV_DEG) / 2
                cv2.line(img, self.px(o[0], o[1]), self.px(o[0] + 0.6 * math.cos(a), o[1] + 0.6 * math.sin(a)),
                         CAM_COLORS.get(cam, (255, 0, 170)), 1)

        odom, age = read_msg('odom')
        if self.sim.opencr.alive:  # 실제 로봇(채움, fake_opencr 진짜 위치) + /odom(테두리) — 미끄럼만큼 어긋남
            x, y, yaw = self.sim.opencr.pose()
            self.robot(img, x, y, yaw, (60, 180, 240) if not self.sim.opencr.bumped else (40, 40, 220), True)
            c = self.px(x, y)
            cv2.putText(img, 'real', (c[0] + 22, c[1] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (60, 60, 60), 1)
        ox, oy = None, None
        if odom:
            pose = odom['pose']['pose']
            ox, oy = self.from_map(pose['position']['x'], pose['position']['y'])
            self.robot(img, ox, oy, yaw_of(pose['orientation']), (0, 0, 200) if age > STALE_S else (200, 60, 0), False)
            c = self.px(ox, oy)
            cv2.putText(img, 'odom', (c[0] + 22, c[1] + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (60, 60, 60), 1)
        base = self.base_from_tf()
        if base is not None and ox is not None:  # base_footprint(초록점) + /odom까지 점선·거리
            bx, by, byaw = base
            self.dashed(img, (ox, oy), (bx, by), (60, 60, 60))
            c = self.px(bx, by)
            cv2.circle(img, c, 6, (0, 200, 0), -1)
            cv2.circle(img, c, 6, (30, 30, 30), 1)
            cv2.line(img, c, self.px(bx + 0.15 * math.cos(byaw), by + 0.15 * math.sin(byaw)), (0, 200, 0), 2)
            d = math.dist((ox, oy), (bx, by))
            cv2.putText(img, f'base {d:.2f}m', self.px((ox + bx) / 2 + 0.03, (oy + by) / 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 130, 0), 1)
        if self.show_navmap:  # Nav2가 보는 지도 /map (바뀔 때만 발행 — 오래돼도 지금 값, 단 이번 실행에서 온 것만)
            _, _, mage = read_grid('map')
            if this_run(mage):
                self.draw_grid(img, 'map', occupancy=True)
            else:
                cv2.putText(img, 'Nav2 /map: not received in this run', (MARGIN, MARGIN + 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 200), 1)
        if self.show_costmap:  # Nav2 코스트맵 — 전역(map) 위에 지역(odom), RViz 색: 100 자홍 · 99 하늘 · 1~98 파랑→빨강
            for topic in ('global_costmap/costmap', 'local_costmap/costmap'):
                self.draw_grid(img, topic, max_age=ROS_FRESH_S)
        if self.nav_goal is not None:  # Nav2 이동 목표 — 초록 깃발 + 로봇에서 점선
            gx, gy = self.nav_goal
            c = self.px(gx, gy)
            cv2.circle(img, c, 7, (0, 160, 0), 2)
            cv2.line(img, c, (c[0], c[1] - 18), (0, 160, 0), 2)
            cv2.putText(img, 'goal', (c[0] + 6, c[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 130, 0), 1)
        if self.show_cloud:  # 점구름 — LiDAR 대신 깊이 카메라: 몸을 뺀 점(body_filter)을 바닥에 눌러 편 것 = 로봇이 보는 장애물
            # pole_front_cam = 빨강 (body_filter가 몸 뺀 것 — Nav2가 보는 것) · gripper_cam = 분홍 (원본 — 손가락·차체가 섞여 로봇 중심
            # 0.22m 안은 뺌. 지도 만들 땐 스캔 자세로 왼쪽 뒤를 봄). 예전엔 pole_front_cam만 그려 gripper_cam 점이 안 보였음
            sources = [(('pole_front_cam/depth/color/points_filtered', 'pole_front_cam/depth/color/points'), (0, 0, 230), 0.0),
                       (('gripper_cam/depth/color/points',), CAM_COLORS['gripper_cam'], 0.22)]
            rx, ry, ryaw = self.sim.robot_xy_yaw()[:3]  # base_footprint 기준 점 → 지금 그린 로봇 자리에 (실기면 /odom)
            c, s_ = math.cos(ryaw), math.sin(ryaw)
            h, w = img.shape[:2]
            for topics, color, body_r in sources:
                xyz, age = None, None
                for t in topics:
                    xyz, _, age = read_points(t)
                    if xyz is not None:
                        break
                if xyz is None or not len(xyz) or age >= ROS_FRESH_S:  # 지금 받은 것만
                    continue
                keep = (xyz[:, 2] > 0.02) & (xyz[:, 2] < 0.6)  # 바닥은 빼고 부딪힐 높이만 (Nav2 costmap과 같은 생각)
                if body_r > 0:
                    keep &= np.hypot(xyz[:, 0], xyz[:, 1]) > body_r
                pts = xyz[keep]
                world = np.stack([rx + c * pts[:, 0] - s_ * pts[:, 1], ry + s_ * pts[:, 0] + c * pts[:, 1]], -1)
                pp = self.px_array(world)
                ok = (pp[:, 0] >= 0) & (pp[:, 0] < w) & (pp[:, 1] >= 0) & (pp[:, 1] < h)
                img[pp[ok, 1], pp[ok, 0]] = color
        cv2.putText(img, 'robot: fill=real(fake_opencr) ring=/odom green=base_footprint(/tf) path=/plan  purple=/tracked_objects(F=both cams)'
                    '  orange/pink x=pole/gripper det  grey=obstacle  blue=mover',
                    (MARGIN - 25, self.size[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (80, 80, 80), 1)
        return img


def _rect(x, y, yaw, sx, sy):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in
            ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2))]


# ---------- 카메라 타일 (SIM 렌더 우선 — ROS 왕복분 빠름, 없으면 ROS가 낸 것)
def run_age():
    """이번 ROS 실행이 시작된 지 몇 초 — monitor_manager가 시작 때 debug/node/를 비우고 노드마다 params를 한 번 씀 →
    state_manager의 params 파일 시각 = 이번 실행 시작. 없으면 None (ROS 꺼짐)"""
    try:
        return time.time() - (DEBUG / 'node' / 'state_manager' / 'params').stat().st_mtime
    except OSError:
        return None


def this_run(age, slack=5.0):
    """토픽 파일 나이(age)가 이번 실행에서 온 것인가 — 바뀔 때만 발행되는 토픽(/map · managed_nodes_activated)은
    이전 실행 파일이 남아 있으면 지금 값처럼 보이므로 시작 시각과 비교"""
    started = run_age()
    return age is not None and started is not None and age <= started + slack


def camera_tiles(sim, cam, view):
    """카메라 한 대 = 8장: HARDWARE(시뮬레이터 렌더 = 카메라 하드웨어가 찍은 원본) RGB · RGBD · RGB-SEG · POINT
    + ROS(받아 온 것) RGB · RGBD · RGB-SEG · POINT. 두 구역을 맞대 보면 렌더 → fake_camera → 인지·점구름 중 어디서 틀어졌는지 보인다
    (실기에 붙이면 HARDWARE는 비고 ROS만)"""
    with sim.lock:
        fresh = time.time() - sim.render_ts.get(cam, 0) < 3.0
        cached = sim.renders.get(cam)
    t_world_cam = cached[2] if cached is not None else None  # 인지 물체 투영용 (ROS 영상에도 마지막 SIM 자세로 근사)

    def blank(text):
        img = np.zeros((TILE[1], TILE[0], 3), np.uint8)
        cv2.putText(img, text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        return img

    def label(img, text):
        cv2.putText(img, text, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        return img

    def depth_tile(depth, text):
        return label(cv2.resize(scene.depth_colormap(depth), TILE, interpolation=cv2.INTER_NEAREST), text)

    def seg_tile(color, text):
        img = cv2.resize(color, TILE)
        overlay_tracked(img, sim, cam, t_world_cam)
        return label(img, text)

    # HARDWARE: 시뮬레이터 렌더 원본
    if fresh and cached is not None:
        color, depth, _ = cached
        hw = [label(cv2.resize(color, TILE), 'HW RGB'), depth_tile(depth, 'HW RGBD'), seg_tile(color, 'HW RGB-SEG'),
              label(scene.cloud_view(depth, TILE, view[0], view[1], color=color,
                                     step=4 if depth.shape[1] > 400 else 2), 'HW POINT')]
    else:
        hw = [blank('no HW render')] * 4
    # ROS: monitor_manager 파일 (실기에서도 같음). 없으면 imread 안 부름 (OpenCV가 매 틱 WARN)
    # 받은 시각을 숨기지 않는다 — 오래된 파일(이전 실행·전달 지연)도 지우지 않고 흐리게 + 빨간 'STALE N s'
    #   (지우면 몇 초씩 끊겨 올 때 검정으로 깜빡임). STALE_HIDE_S 넘으면 이전 실행 잔재로 보고 안 그림
    def age_of(path):
        try:
            return time.time() - path.stat().st_mtime
        except OSError:
            return None

    def mark(img, age, text):
        if age is not None and age >= ROS_FRESH_S:
            img = (img * 0.4).astype(np.uint8)
            cv2.putText(img, f'STALE {age:.0f}s', (6, TILE[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 60, 255), 2)
        return label(img, text)

    def usable(age):
        return age is not None and age < STALE_HIDE_S

    cpath = TOPIC_DIR / cam / 'color/image_raw/image.jpg'
    dpath = TOPIC_DIR / cam / 'aligned_depth_to_color/image_raw/depth.png'
    c_age, d_age = age_of(cpath), age_of(dpath)
    color = cv2.imread(str(cpath)) if usable(c_age) else None
    depth = cv2.imread(str(dpath), cv2.IMREAD_ANYDEPTH) if usable(d_age) else None
    why = lambda age: 'no file' if age is None else f'old {age:.0f}s (이전 실행?)'
    if color is not None:
        rgb = cv2.resize(color, TILE)
        seg = rgb.copy()
        overlay_tracked(seg, sim, cam, t_world_cam)
        ros = [mark(rgb, c_age, 'ROS RGB')]
    else:
        ros = [blank(f'ROS RGB: {why(c_age)}')]
    ros.append(mark(cv2.resize(scene.depth_colormap(depth), TILE, interpolation=cv2.INTER_NEAREST), d_age, 'ROS RGBD')
               if depth is not None else blank(f'ROS RGBD: {why(d_age)}'))
    ros.append(mark(seg, c_age, 'ROS RGB-SEG') if color is not None else blank(f'ROS RGB-SEG: {why(c_age)}'))
    xyz, _, p_age = read_points(f'{cam}/depth/color/points')
    if xyz is not None and len(xyz) and usable(p_age):
        # 점구름(base_footprint) → 그 카메라 자신의 시점(광학 프레임: x 오른 · y 아래 · z 앞 = points_view 축) — RGB 칸과 같은 방향.
        # 예전엔 늘 로봇 앞을 보는 시점이라, 스캔 자세에서 뒤를 보는 gripper_cam 점은 시점 뒤로 가 안 보였음. 색 = 높이
        if t_world_cam is not None:
            rx, ry, ryaw, _ = sim.robot_xy_yaw()
            T_cb = np.linalg.inv(np.asarray(t_world_cam, float)) @ scene.transform(scene.rpy_matrix(0, 0, ryaw), (rx, ry, 0))
            pts = xyz @ T_cb[:3, :3].T + T_cb[:3, 3]
        else:  # 카메라 자세를 모르면 로봇 앞 시점
            pts = np.stack([-xyz[:, 1], -xyz[:, 2], xyz[:, 0]], -1)
        cols = cv2.applyColorMap((np.clip(xyz[:, 2] / 0.6, 0, 1) * 255).astype(np.uint8)[None], cv2.COLORMAP_TURBO)[0]
        ros.append(mark(scene.points_view(pts, cols, TILE, view[0], view[1]), p_age, f'ROS POINT {len(xyz)}'))
    else:
        ros.append(blank(f'ROS POINT: {why(p_age)}'))
    return (*hw, *ros)


def overlay_tracked(rgb, sim, cam, t_world_cam):
    """카메라 타일 위에 인지 네모+라벨 (지도 보라테두리와 같은 색). 라벨 F = 다른 카메라도 같은 트랙에 붙음(융합).
    이 카메라의 융합 전 검출(/perception/<cam>/detections)을 우선 — 그 영상의 px라 카메라마다 늘 맞는다.
    없으면(이전 perception) /tracked_objects 중 마지막에 이 카메라가 본 것만.
    중심은 인지 px(640×480 기준)를 타일에 스케일 — 회전·프레임 가정 없음.
    네모 크기는 핀홀 근사(카메라 위치~물체 3D 거리, 회전 불필요)."""
    cams_det = camera_detections()
    both = fused_ids(cams_det)
    if cam in cams_det:
        objs = cams_det[cam]
    else:
        objects, age = read_msg('tracked_objects')
        if age is None or age > ROS_FRESH_S:  # 지금 받은 인지 결과만 (이전 실행 파일은 안 그림)
            return
        objs = (objects or {}).get('objects', [])
    if not objs:
        return
    h, w = rgb.shape[:2]
    try:
        twc = np.asarray(t_world_cam[:3, 3], float) if t_world_cam is not None else None
    except (TypeError, ValueError, IndexError):
        twc = None
    fx = (640.0 / 2) / math.tan(math.radians(scene.HFOV_DEG) / 2)
    sx0, sy0 = sim.spawn[0], sim.spawn[1]
    for o in objs:
        if o.get('camera') and o['camera'] != cam:
            continue
        try:
            px = o.get('px') or (0, 0)
            u, v = px[0] * w / 640.0, px[1] * h / 480.0
            dist = 1.0
            if twc is not None:
                p = o['pose']['pose']['position']
                dist = max(float(np.linalg.norm(
                    np.array([p['x'] + sx0, p['y'] + sy0, p['z']], float) - twc)), 0.05)
        except (KeyError, TypeError, ValueError, IndexError):
            continue
        hw = max(o.get('size', {}).get('x', 0.06), o.get('size', {}).get('y', 0.06), 0.02) * fx / dist / 2 * w / 640.0
        hh = max(o.get('size', {}).get('z', 0.06), 0.02) * fx / dist / 2 * h / 480.0
        cv2.rectangle(rgb, (int(u - hw), int(v - hh)), (int(u + hw), int(v + hh)), (150, 0, 150), 1)
        cv2.putText(rgb, f"{o.get('class_name', '?')}#{o.get('id', '?')}" + (' F' if o.get('id') in both else ''),
                    (int(u + hw + 2), int(v)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 0, 150), 1)


def status_lines(sim=None):
    """[(글, 태그)] — 작업 상태 + 플랜트"""
    lines = []

    def add(label, topic, fmt, latched=False):
        lines.append((f'{label}  /{topic}', 'title'))  # 제목 + 어느 토픽에서 받은 값인지 (로컬 값이 아님), 값은 다음 줄
        data, age = read_msg(topic)
        if data is None:
            lines.append((' - (없음)', 'warn'))
            return
        try:
            text = fmt(data)
        except (KeyError, IndexError, TypeError) as e:
            text = f'(형식 다름: {e})'
        if latched:  # 상태(/state/*)는 바뀔 때만 발행 (transient_local) — 오래돼도 지금 값. 바뀐 시각만 보여 줌
            lines.append((f' - {text}  [{age:.0f}s 전 바뀜]', ''))
        else:
            lines.append((f' - {text}  [{age:.1f}s 전]', 'warn' if age > STALE_S else ''))

    add('작업', 'task_status', lambda d: f"plan {d['plan_id']} step {d['step']} {STATE_NAMES.get(d['state'], d['state'])} {d['reason']}")
    add('계획', 'sub_goals', lambda d: f"plan {d['plan_id']}: " + ' → '.join(f"{g['skill']}({g['target']})" for g in d['goals']))
    add('로봇 말', 'robot_say', lambda d: d['data'])
    add('재시작 (state_manager)', 'state/restart', lambda d: d['data'], latched=True)
    # Nav2 관리자 — 기동 때 가끔(약 4번에 1번) 서버 응답을 못 받고 멈춤 → 꺼짐이면 빨간불, 사람이 [ROS 재시작]
    for label, mgr in (('Nav2 바퀴 체인 (velocity_smoother · collision_monitor)', 'lifecycle_manager_base'),
                       ('Nav2 주행 계획 (planner · controller · bt_navigator)', 'lifecycle_manager_navigation')):
        topic = f'{mgr}/managed_nodes_activated'
        lines.append((f'{label}  /{topic}', 'title'))
        data, age = read_msg(topic)
        if data and data.get('data') and this_run(age):  # 이번 실행에서 켜진 것만 (이전 실행 true 파일 무시)
            lines.append((' - 켜짐', ''))
        else:
            lines.append((' - 꺼짐 (기동 중이 아니면 멈춘 것) → 여기를 눌러 [ROS 재시작]', 'restart'))
    add('조작 모드', 'state/operation_mode', lambda d: d['data'], True)  # state_manager 발행
    add('지도 모드', 'state/map_mode', lambda d: d['data'], True)        # state_manager 발행
    add('지도 폴더', 'state/map_name', lambda d: d['data'] or '-', True)
    add('탐사 결과', 'explore/result', lambda d: d['data'], True)  # explorer: DONE · FAILED
    add('지도 목록', 'state/map_list', lambda d: (lambda n: f"{len(n)}개 {', '.join(n)}" if n else '없음')([x for x in d['data'].split('\n') if x]), True)
    add('비상정지', 'state/emergency', lambda d: 'ON' if d['data'] else '-', True)
    try:  # /state/* 전체 원문 (state_manager가 늘리면 자동 표시)
        import json as _j
        for f in sorted((TOPIC_DIR / 'state').glob('*')):
            if f.name in ('operation_mode', 'map_mode', 'map_name', 'map_list', 'emergency') or not (f / 'message').exists():
                continue
            try:
                raw = _j.loads((f / 'message').read_text()) or {}
            except ValueError:
                continue
            lines.append((f"/state/{f.name}", 'title'))
            lines.append((f" - {raw.get('data', raw)}", ''))
    except OSError:
        pass
    add('속도 상한', 'speed_limit', lambda d: f"{d['speed_limit']:.0f}{'%' if d['percentage'] else ' m/s'}")
    add('위치(odom)', 'odom', lambda d: '({x:.2f}, {y:.2f})'.format(**d['pose']['pose']['position'])
        + f" yaw {math.degrees(yaw_of(d['pose']['pose']['orientation'])):.0f}°")
    add('팔', 'joint_states', lambda d: ' '.join(
        f'{n.replace("_joint", "")}={p:.2f}' for n, p in zip(d['name'], d['position']) if not n.startswith('wheel')))
    if sim:
        p = sim.opencr
        if p.alive:
            x, y, yaw = p.pose()
        lines.append(('OpenCR 실제 위치  /fake_opencr/ground_truth · /fake_opencr/bumped', 'title'))
        if p.alive:
            lines.append((f' - 연결 · ({x - sim.spawn[0]:.2f}, {y - sim.spawn[1]:.2f}) '
                          f'yaw {math.degrees(yaw):.0f}°{" · 충돌!" if p.bumped else ""}', 'error' if p.bumped else ''))
        else:
            lines.append((' - 없음 — faker가 mock_hardware:=true로 떠 있음 (물리 없음)', 'warn'))
    return lines


# 그래프의 내부 노드 — 사람이 볼 일 없음 (TF 리스너 · 액션 내부 · rclcpp 보조 노드)
INTERNAL_NODE = ('transform_listener_impl_', '_private_', '_rclcpp_node')


def graph_nodes():
    """지금 ROS 그래프의 노드 이름들 (monitor_manager debug/node/list — 중복·내부 노드 뺌, 앞 '/' 뺌)"""
    try:
        lines = (DEBUG / 'node' / 'list').read_text().split()
    except OSError:
        return []
    return sorted({n.strip('/') for n in lines if n.strip('/') and not any(k in n for k in INTERNAL_NODE)})


def restartable_nodes():
    """state_manager /state/restartable → {이름: pid} (재시작할 수 있는 노드 = 프로세스) · 없으면 None (예전 state_manager)"""
    data, _ = read_msg('state/restartable')
    if data is None:
        return None
    out = {}
    for line in (data.get('data') or '').splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[1].isdigit():
            out[parts[0]] = int(parts[1])
    return out


def diagnostics_lines():
    """진단 칸 → [(글, 태그 — 여러 개면 공백 구분)]: ① 노드 전부 — 그래프 노드마다 진단(가장 나쁜 수준 · 메시지), 재시작할 수 있는 노드는 태그 'rs:<이름>'
    (누르면 확인 → /cmd/restart node <이름>) ② 노드가 아닌 진단 (하드웨어 · Nav2 관리자 상태 등)"""
    data, age = read_msg('diagnostics_agg')
    by_node, other = {}, []
    for s in sorted((data or {}).get('status', []), key=lambda s: s['name']):
        if ': ' not in s['name']:
            continue
        node, item = s['name'].split('/')[-1].split(': ', 1)
        # Nav2 관리자는 노드가 안 켜져도(멈춤) 수준을 OK로 냄 → active가 아니면 오류로 + 누르면 전체 재시작
        nav2_down = item == 'Nav2 Health' and 'map_saver' not in node and s['message'] != 'Managed nodes are active'
        by_node.setdefault(node, []).append((2 if nav2_down else s['level'], f"{item}: {s['message']}", nav2_down))
    nodes = graph_nodes()
    rs = restartable_nodes()
    out = []
    head = f'노드 {len(nodes)}개'
    if rs is None:
        head += ' — 재시작 목록 없음 (/state/restartable: state_manager 새 버전이 떠야 함)'
    else:
        head += f' · 재시작 가능 {len(rs)}개 — 이름을 누르면 그 노드만 재시작 (같은 파라미터)'
    out.append((head, 'title'))
    if not data:
        out.append(('(진단 없음 — bringup의 diagnostic_aggregator)', 'warn'))
    elif age > STALE_S:
        out.append((f'(진단이 {age:.0f}s 전 값)', 'warn'))
    for n in sorted(set(nodes) | set(by_node) | set(rs or {})):
        items = by_node.pop(n, [])
        level, tag = LEVEL.get(max((i[0] for i in items), default=-1), ('-', ''))
        msg = ' · '.join(i[1] for i in items) if items else '(진단 없음)'
        if rs is not None and n in rs:  # 태그 여러 개는 공백으로 — 수준 색 + 누를 수 있음(rs:)
            out.append((f'{level:5} {n:24} {msg}   [재시작 pid {rs[n]}]', f'{tag} rs:{n}'.strip()))
        else:
            why = '' if rs is None else ('  (프로세스 아님 — 컨트롤러 · 하위 노드)' if n in nodes else '  (그래프에 없음)')
            out.append((f'{level:5} {n:24} {msg}{why}', tag))
        if any(i[2] for i in items):
            out.append(('      → 기동 중이 아니면 멈춘 것: 여기를 눌러 [ROS 재시작] (전체)', 'restart'))
    return out


def to_photo(img):
    ok, buf = cv2.imencode('.png', img)
    return tk.PhotoImage(data=base64.b64encode(buf.tobytes())) if ok else None


# ---------- 창
class App:
    def __init__(self, root, sim, view):
        self.root, self.sim, self.view = root, sim, view
        self.cloud_view = {cam: [0.6, -0.4] for cam in CAMERAS}
        self.drag = None
        root.title('lv2_module6 SIL 시뮬레이터')
        left = tk.Frame(root)
        left.pack(side=tk.LEFT, padx=6, pady=6, anchor=tk.N)
        right = tk.Frame(root)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=6, pady=6)

        top = tk.Frame(left)  # 지도 + 물체 조작(세로) 나란히
        top.pack(fill=tk.X)
        mapbox = tk.Frame(top)
        mapbox.pack(side=tk.LEFT, anchor=tk.N)
        self.map_label = tk.Label(mapbox, cursor='hand2', takefocus=1)
        self.map_label.pack()
        self.map_label.bind('<ButtonPress-1>', self.on_map_press)
        self.map_label.bind('<B1-Motion>', self.on_map_drag)
        self.map_label.bind('<ButtonRelease-1>', self.on_map_release)
        # 오른쪽 버튼 (macOS Tk는 Button-2): 끌면 지도 이동, 그냥 떼면 그 자리 미션 메뉴
        for b in ('2', '3'):
            self.map_label.bind(f'<ButtonPress-{b}>', self.on_map_pan_press)
            self.map_label.bind(f'<B{b}-Motion>', self.on_map_pan_drag)
            self.map_label.bind(f'<ButtonRelease-{b}>', self.on_map_context)
        self.map_label.bind('<MouseWheel>', self.on_map_wheel)
        self.map_label.bind('<Button-4>', lambda e: self.on_map_wheel_step(1))
        self.map_label.bind('<Button-5>', lambda e: self.on_map_wheel_step(-1))
        mapctl = tk.Frame(mapbox)
        mapctl.pack(fill=tk.X, pady=2)
        tk.Button(mapctl, text='보기 리셋', command=self.view.reset_view).pack(side=tk.LEFT)  # 확대·이동만 처음으로 (지도 데이터는 그대로)
        self.coords_label = tk.Label(mapctl, justify=tk.LEFT, font=('Menlo', 9), anchor=tk.W)
        self.coords_label.pack(side=tk.LEFT, padx=(6, 0))
        self.scan_choice = tk.StringVar(value='선택없음')
        self.scan_menu = tk.OptionMenu(mapctl, self.scan_choice, '선택없음', *self.view.scan_choices())
        self.scan_menu.pack(side=tk.LEFT)
        self.cloud_on = tk.BooleanVar(value=False)
        tk.Checkbutton(mapctl, text='점구름', variable=self.cloud_on,
                       command=lambda: (setattr(self.view, 'show_cloud', self.cloud_on.get()), self.redraw_map())).pack(side=tk.LEFT)
        self.navmap_on = tk.BooleanVar(value=False)
        tk.Checkbutton(mapctl, text='Nav2 지도', variable=self.navmap_on,
                       command=lambda: (setattr(self.view, 'show_navmap', self.navmap_on.get()), self.redraw_map())).pack(side=tk.LEFT)
        self.costmap_on = tk.BooleanVar(value=False)
        tk.Checkbutton(mapctl, text='코스트맵', variable=self.costmap_on,
                       command=lambda: (setattr(self.view, 'show_costmap', self.costmap_on.get()), self.redraw_map())).pack(side=tk.LEFT)

        def _pick_scan(*_):
            name = self.scan_choice.get()
            if not getattr(self, '_syncing_map', False):  # 사람이 고른 것 — '선택없음'이면 그 선택을 지킴 (로봇 지도 이름을 안 따라감)
                self._map_none = name == '선택없음'
            if name == '선택없음' and self._map_none:
                self.navmap_on.set(False)
                self.view.show_navmap = False
                cur = (read_msg('state/map_name')[0] or {}).get('data')
                self.sent.config(text='지도 선택없음 — Nav2 지도를 겹치지 않고, 로봇 지도 이름을 따라가지 않음'
                                      + (f' (로봇은 지금 지도 {cur} 를 계속 씀 — ROS는 그대로)' if cur else ''))
            if name != '선택없음':
                # 그 지도로 갈아끼움 → state_manager가 RTAB-Map load_database + 위치추정 (결과 /state/map_name · /state/map_mode)
                # 화면엔 토픽 /map([Nav2 지도])으로만 — 호스트의 map.pgm 파일은 안 씀 (실기에 붙이면 의미 없음)
                self.navmap_on.set(True)
                self.view.show_navmap = True
                if (read_msg('state/map_mode')[0] or {}).get('data') == 'COURSE':  # RTAB-Map 없음 → 그 지도로 다시 띄울지
                    self.restart_ros(f'map:=saved map_name:={name}', f'지금은 코스 지도(COURSE)입니다.\n지도 {name} 로 위치추정 모드로 ROS를 다시 띄울까요?')
                elif name != (read_msg('state/map_name')[0] or {}).get('data'):
                    send('/cmd/map_name', 'std_msgs/msg/String', {'data': name})
                    self.sent.config(text=f'요청: 지도 {name} 로 갈아끼움 (/cmd/map_name → state_manager → RTAB-Map 위치추정)')
            self.redraw_map()
        self.scan_choice.trace_add('write', _pick_scan)
        robotbox = tk.LabelFrame(mapbox, text='로봇')
        robotbox.pack(fill=tk.X, pady=(2, 0))
        ext = tk.LabelFrame(robotbox, text='외력 (fake_opencr — mock이면 효과 없음)')
        ext.pack(fill=tk.X)
        row1 = tk.Frame(ext)
        row1.pack(fill=tk.X)
        self.slip = tk.Scale(row1, label='미끄럼', from_=0.0, to=0.6, resolution=0.05, orient=tk.HORIZONTAL,
                             length=120, command=lambda v: setattr(self.sim, 'base_slip', float(v)))
        self.slip.pack(side=tk.LEFT)
        self.friction = tk.Scale(row1, label='마찰(가속)', from_=0.2, to=1.5, resolution=0.1, orient=tk.HORIZONTAL,
                                 length=120, command=lambda v: setattr(self.sim.opencr, 'friction', float(v)))
        self.friction.set(1.0)
        self.friction.pack(side=tk.LEFT)
        row3 = tk.Frame(ext)
        row3.pack(fill=tk.X)
        self.light = tk.Scale(row3, label='조명(밝기 배율)', from_=0.3, to=1.6, resolution=0.05, orient=tk.HORIZONTAL,
                              length=120, command=lambda v: setattr(self.sim, 'light', float(v)))
        self.light.set(1.0)
        self.light.pack(side=tk.LEFT)
        self.noise = tk.Scale(row3, label='카메라 잡음', from_=0, to=40, resolution=1, orient=tk.HORIZONTAL,
                              length=120, command=lambda v: setattr(self.sim, 'cam_noise', float(v)))
        self.noise.pack(side=tk.LEFT)
        row2 = tk.Frame(robotbox)
        row2.pack(fill=tk.X, anchor=tk.W)
        tk.Button(row2, text='위치 초기화', command=self.sim.reset_robot).pack(side=tk.LEFT)
        self.comm = tk.BooleanVar(value=False)
        tk.Checkbutton(row2, text='OpenCR 통신 끊기', variable=self.comm,
                       command=lambda: setattr(self.sim.opencr, 'comm_down', self.comm.get())).pack(side=tk.LEFT)
        self.grab = tk.BooleanVar(value=False)
        tk.Checkbutton(row2, text='팔 붙잡기(과전류)', variable=self.grab,
                       command=lambda: setattr(self.sim.opencr, 'arm_grabbed', self.grab.get())).pack(side=tk.LEFT)
        mapside = tk.Frame(top)
        mapside.pack(side=tk.LEFT, anchor=tk.N, padx=(6, 0))
        SIDE_WRAP = 170  # 이 열의 글자 줄바꿈 폭 [px] — 긴 상태 글자·파일 이름이 열을 넓히지 않게
        envbox = tk.LabelFrame(mapside, text='물체·환경')
        envbox.pack(fill=tk.X)
        tk.Button(envbox, text='퍽 랜덤 배치', command=lambda: self.sim.randomize(max(sum(1 for p in self.sim.pucks if not p.get('station')), 6))).pack(fill=tk.X)
        tk.Button(envbox, text='퍽 추가', command=self.sim.add_puck).pack(fill=tk.X)
        tk.Button(envbox, text='퍽 삭제', command=self.sim.pop_puck).pack(fill=tk.X)
        tk.Button(envbox, text='이동물체 추가', command=self.sim.spawn_mover).pack(fill=tk.X)
        tk.Button(envbox, text='이동물체 삭제', command=lambda: self.sim.pop_prop('mover')).pack(fill=tk.X)
        tk.Button(envbox, text='바닥 장애물 추가', command=self.sim.add_obstacle).pack(fill=tk.X)
        tk.Button(envbox, text='바닥 장애물 삭제', command=lambda: self.sim.pop_prop('obstacle')).pack(fill=tk.X)
        tk.Button(envbox, text='신호등 추가', command=self.sim.spawn_traffic).pack(fill=tk.X)
        tk.Button(envbox, text='신호등 삭제', command=lambda: self.sim.pop_prop('traffic_light')).pack(fill=tk.X)
        # 미션 (documents/mission/README.md) — 한 맵에 미션 지역 여러 곳. 지도에서 오른쪽 클릭 메뉴로 놓기·바꾸기·지우기,
        # 왼쪽 드래그로 미션 지역·B 지역 옮기기 (채점은 외부)
        missbox = tk.LabelFrame(mapside, text='미션 (MISSION.md)')
        missbox.pack(fill=tk.X, pady=(6, 0))
        tk.Label(missbox, text='지도 오른쪽 클릭 = 미션 메뉴\nShift+드래그 = 미션·B 지역 통째로\n드래그 = 퍽·로봇 하나', justify=tk.LEFT, anchor=tk.W,
                 fg='#555').pack(fill=tk.X)
        row = tk.Frame(missbox)
        row.pack(fill=tk.X)
        tk.Label(row, text='환경 seed').pack(side=tk.LEFT)
        self.seed = tk.Entry(row, width=8)
        self.seed.pack(side=tk.LEFT)
        row = tk.Frame(missbox)
        row.pack(fill=tk.X)
        tk.Button(row, text='환경 배치', command=self.on_env).pack(side=tk.LEFT, fill=tk.X, expand=True)
        tk.Button(row, text='환경 해제', command=self.sim.clear_env).pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.mission_label = tk.Label(missbox, justify=tk.LEFT, anchor=tk.W, font=('Menlo', 9), text='대기', wraplength=SIDE_WRAP)
        self.mission_label.pack(fill=tk.X)
        # 주사위 미션 (course dice_maze) — 문제 깔기 · 운영자 신호 시작 · /cmd/mission start|abort → mission_manager
        self.dice_label = None
        if 'mission' in self.sim.course:
            dicebox = tk.LabelFrame(mapside, text='주사위 미션')
            dicebox.pack(fill=tk.X, pady=(6, 0))
            self.dice_name = tk.StringVar(value=(self.sim.dice_problem or {}).get('name', (dice.problems() or [''])[0]))
            ttk.Combobox(dicebox, textvariable=self.dice_name, values=dice.problems(), width=10, state='readonly').pack(fill=tk.X)
            tk.Button(dicebox, text='문제 깔기', command=lambda: self.sim.load_dice(self.dice_name.get())).pack(fill=tk.X)
            tk.Button(dicebox, text='신호 시작', command=self.sim.start_signal).pack(fill=tk.X)
            self.auto_signal = tk.BooleanVar(value=True)
            tk.Checkbutton(dicebox, text='READ가 되면 신호 자동 시작', variable=self.auto_signal).pack(anchor=tk.W)
            row = tk.Frame(dicebox)
            row.pack(fill=tk.X)
            tk.Button(row, text='미션 시작', command=lambda: send('/cmd/mission', 'std_msgs/msg/String', {'data': 'start'})).pack(
                side=tk.LEFT, fill=tk.X, expand=True)
            tk.Button(row, text='중지', command=lambda: send('/cmd/mission', 'std_msgs/msg/String', {'data': 'abort'})).pack(
                side=tk.LEFT, fill=tk.X, expand=True)
            self.dice_label = tk.Label(dicebox, justify=tk.LEFT, anchor=tk.W, font=('Menlo', 9), text='대기', wraplength=SIDE_WRAP)
            self.dice_label.pack(fill=tk.X)

        # 지도 (SLAM) — 컨테이너 faker map:=slam·saved 일 때 RTAB-Map. 요청만 보내고 상태는 state_manager 발행값(/state/map_mode)을 보여 줌
        #   스캔 시작 / 위치추정으로 → /cmd/map_mode → state_manager가 RTAB-Map 모드 전환 (재시작 없이, DB는 rtabmap.db 그대로)
        #   지도 저장 → /map_saver/save_map → 지금 지도 폴더(/state/map_name) ros2_ws/map/<이름>/map.pgm · yaml (rtabmap.db는 RTAB-Map이 같은 폴더에 씀)
        #   지도 한 벌 = 폴더 하나 — 위 스캔 지도 목록에서 폴더를 골라 겹쳐 봄. 새 지도는 컨테이너 map:=slam 이 날짜_시각 폴더를 새로 만듦
        slambox = tk.LabelFrame(mapside, text='지도 (SLAM)')
        slambox.pack(fill=tk.X, pady=(6, 0))
        self.slam_label = tk.Label(slambox, justify=tk.LEFT, anchor=tk.W, font=('Menlo', 9), text='대기', wraplength=SIDE_WRAP)
        self.slam_label.pack(fill=tk.X)
        row = tk.Frame(slambox)
        row.pack(fill=tk.X)
        for text, mode in (('스캔 시작', 'MAPPING'), ('위치추정으로', 'LOCALIZATION')):
            tk.Button(row, text=text, command=lambda m=mode: self.map_mode_request(m)).pack(side=tk.LEFT, fill=tk.X, expand=True)
        row = tk.Frame(slambox)
        row.pack(fill=tk.X)
        tk.Button(row, text='자동 탐사', command=self.start_explore).pack(side=tk.LEFT, fill=tk.X, expand=True)
        tk.Button(row, text='탐사 멈춤', command=lambda: self.stop_explore()).pack(side=tk.LEFT, fill=tk.X, expand=True)
        tk.Button(slambox, text='지도 저장', command=self.save_map).pack(fill=tk.X)  # → ros2_ws/map/<지금 지도 폴더>/map.pgm·yaml
        # 위치추정이 어긋났을 때 — 실제 위치를 /initialpose로 (위치추정 모드에서만. 지도 오른쪽 클릭 [여기가 로봇 위치]는 찍은 곳으로)
        tk.Button(slambox, text='실제 위치로 맞추기 (/initialpose)', command=self.snap_to_real_pose).pack(fill=tk.X)
        self.save_pending = None  # 지도 저장 request_id — 응답 기다리는 중
        self.nav_goal_id = None  # [여기로 이동] goal_id — 진행 중

        self.robot_model = None
        self.robot_cam = robot_view.OrbitView(target=(0, 0, 0.2), dist=1.3)

        viewbox = tk.LabelFrame(mapside, text='보기')
        viewbox.pack(fill=tk.X, pady=(6, 0))
        names = [NO_BG] + [p.name for p in scene.backgrounds(BG_DIR)]
        self.bg = tk.StringVar(value=names[1] if len(names) > 1 else NO_BG)
        tk.Label(viewbox, text='배경').pack(anchor=tk.W)
        menu = tk.OptionMenu(viewbox, self.bg, *names, command=self.on_background)
        menu.config(width=16)  # 긴 파일 이름이 열을 넓히지 않게
        menu.pack(fill=tk.X)
        self.on_background(self.bg.get())
        fnames = [NO_FLOOR] + [p.name for p in scene.backgrounds(FLOOR_DIR) if p.name != 'FLOOR_CREDITS.txt']
        self.floor = tk.StringVar(value=fnames[1] if len(fnames) > 1 else NO_FLOOR)
        tk.Label(viewbox, text='바닥').pack(anchor=tk.W)
        menu = tk.OptionMenu(viewbox, self.floor, *fnames, command=self.on_floor)
        menu.config(width=16)  # 긴 파일 이름이 열을 넓히지 않게
        menu.pack(fill=tk.X)
        self.on_floor(self.floor.get())
        wnames = [NO_WALL] + [p.name for p in scene.backgrounds(WALL_DIR) if p.suffix == '.jpg']
        self.wall = tk.StringVar(value=wnames[1] if len(wnames) > 1 else NO_WALL)
        tk.Label(viewbox, text='벽').pack(anchor=tk.W)
        menu = tk.OptionMenu(viewbox, self.wall, *wnames, command=self.on_wall)
        menu.config(width=16)  # 긴 파일 이름이 열을 넓히지 않게
        menu.pack(fill=tk.X)
        self.on_wall(self.wall.get())
        self.stream = tk.BooleanVar(value=True)
        tk.Checkbutton(viewbox, text='카메라 송출', variable=self.stream, command=self.on_stream).pack(anchor=tk.W)

        # 로봇 3D (지금 상태) — 카메라와 한 덩이
        # 장비 줄: 로봇 3D (지금 상태) │ 시리얼 (OpenCR 송수신 — 플랜트가 OpenCR 대신)
        equip = tk.Frame(right)
        equip.pack(fill=tk.X, anchor=tk.W)
        robot_box = tk.Frame(equip)
        robot_box.pack(side=tk.LEFT, anchor=tk.N)
        tk.Label(robot_box, text='로봇 (지금 상태) — 드래그 회전 · Shift+드래그 이동 · 휠 줌', anchor=tk.W).pack(fill=tk.X)
        self.robot_label = tk.Label(robot_box, bg='black', width=ROBOT_VIEW[0], height=ROBOT_VIEW[1])
        self.robot_label.pack(anchor=tk.W)
        serial_box = tk.Frame(equip)
        serial_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(6, 0), anchor=tk.N)
        self.serial_head = tk.Label(serial_box, text='시리얼 (OpenCR)', anchor=tk.W, justify=tk.LEFT, font=('Menlo', 9), wraplength=330)
        self.serial_head.pack(fill=tk.X)
        self.serial_text = {}
        # 실제 가상 시리얼 파일을 그대로 (플랜트가 꺼져 있어도 컨테이너가 보낸 C는 보임) — 크기가 바뀔 때만 읽음
        self.serial_tail = {'cmd': FileTail(DEBUG / 'output' / 'hardware' / 'serial', back=4096),
                            'state': FileTail(DEBUG / 'input' / 'hardware' / 'serial', back=4096)}
        for key, title in (('cmd', 'SBC → OpenCR  C 명령 (8값: 바퀴L·R rad/s · joint1~5 rad · 그리퍼 m)'),
                           ('state', 'OpenCR → SBC  S 상태 (40값: 바퀴 · 관절 pos·vel·A · 그리퍼 · IMU · 모터 온도)')):
            tk.Label(serial_box, text=title, anchor=tk.W, justify=tk.LEFT, font=('Menlo', 9), fg='#555', wraplength=330).pack(fill=tk.X)
            t = tk.Text(serial_box, height=7, width=46, wrap=tk.NONE, font=('Menlo', 9), bg='#111', fg='#9f9')
            t.pack(fill=tk.BOTH, expand=True)
            self.serial_text[key] = t
        self.robot_label.bind('<ButtonPress-1>', self.on_robot_press)
        self.robot_label.bind('<B1-Motion>', self.on_robot_drag)
        self.robot_label.bind('<MouseWheel>', lambda e: self.robot_cam.zoom(1 if e.delta > 0 else -1))
        self.robot_label.bind('<Button-4>', lambda e: self.robot_cam.zoom(1))
        self.robot_label.bind('<Button-5>', lambda e: self.robot_cam.zoom(-1))
        # 보기 체크 — 로봇 3D와 pole_front_cam·gripper_cam 렌더를 URDF <collision>으로 (메시 충돌체: 3D는 STL, 카메라는 경계 상자)
        viewrow = tk.Frame(robot_box)
        viewrow.pack(anchor=tk.W)
        self.show_frames = tk.BooleanVar(value=False)
        tk.Checkbutton(viewrow, text='관절축', variable=self.show_frames).pack(side=tk.LEFT)
        self.show_collision = tk.BooleanVar(value=False)
        tk.Checkbutton(viewrow, text='충돌모양', variable=self.show_collision,
                       command=self.on_toggle_collision).pack(side=tk.LEFT)
        # 감지 물체 — /tracked_objects(보라 와이어 상자, F = 두 카메라 융합) + 카메라별 검출 점. 조작 창 3D도 같은 체크
        self.show_detected = tk.BooleanVar(value=False)
        tk.Checkbutton(viewrow, text='감지 물체', variable=self.show_detected,
                       command=self.refresh_robot).pack(side=tk.LEFT)
        self.robot_model_col = None

        # 상태 · 진단 — 맨 오른쪽 열 (세로로 길게)
        side = tk.Frame(root)
        side.pack(side=tk.LEFT, fill=tk.BOTH, padx=(0, 6), pady=6)
        texts = ttk.Notebook(side)
        texts.pack(fill=tk.BOTH, expand=True)
        status_tab = tk.Frame(texts)
        texts.add(status_tab, text='상태 · 진단')
        def scrolled_text(height):  # 넘치면 세로·가로 스크롤 (줄바꿈 안 함)
            box = tk.Frame(status_tab)
            box.pack(fill=tk.BOTH, expand=True)
            t = tk.Text(box, width=64, height=height, font=("Menlo", 9), wrap=tk.NONE)
            ys, xs = tk.Scrollbar(box, command=t.yview), tk.Scrollbar(box, orient=tk.HORIZONTAL, command=t.xview)
            t.configure(yscrollcommand=ys.set, xscrollcommand=xs.set)
            ys.pack(side=tk.RIGHT, fill=tk.Y)
            xs.pack(side=tk.BOTTOM, fill=tk.X)
            t.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
            return t
        self.status = scrolled_text(20)
        self.diag = scrolled_text(30)
        self.status.tag_config('title', font=('Menlo', 9, 'bold'))
        self.diag.tag_config('title', font=('Menlo', 9, 'bold'))
        self._rs_tags = set()  # 진단 칸 노드 줄 태그 'rs:<이름>' — 처음 나올 때 한 번만 묶음
        for t in (self.status, self.diag):
            t.tag_config('warn', foreground='#b26a00')
            t.tag_config('error', foreground='#c62828')
            # 'restart' 줄 = 누르면 [ROS 재시작] (확인 창) — Nav2 관리자 멈춤처럼 재시작이 답인 진단에
            t.tag_config('restart', foreground='#c62828', underline=True)
            t.tag_bind('restart', '<Button-1>', lambda e: self.restart_ros(why='Nav2 관리자가 노드를 못 켰습니다 (기동 때 가끔 멈춤).'))
            t.tag_bind('restart', '<Enter>', lambda e, w=t: w.config(cursor='hand2'))
            t.tag_bind('restart', '<Leave>', lambda e, w=t: w.config(cursor=''))

        # 카메라 — HARDWARE(시뮬레이터 렌더) 구역 · ROS(받아 온 것) 구역. 구역마다 열 = 카메라, 줄 = RGB · RGBD · RGB-SEG · POINT. 세로 스크롤
        camwrap = tk.Frame(right)  # 스크롤 열은 side=LEFT로 붙으니 한 겹 감싸서 아래 버튼 줄과 안 섞이게
        camwrap.pack(fill=tk.BOTH, expand=True)
        cams = ManualPanel.scroll_column(camwrap, width=TILE[0] * len(CAMERAS) + 12)
        self.tiles = {cam: [None] * 8 for cam in CAMERAS}
        for sec, (title, base) in enumerate((('HARDWARE (시뮬레이터 렌더 — 카메라 원본)', 0), ('ROS (받아 온 토픽 — 실기도 같음)', 4))):
            tk.Label(cams, text=title, anchor=tk.W, font=('TkDefaultFont', 11, 'bold')).pack(fill=tk.X, pady=(6 if sec else 0, 0))
            grid = tk.Frame(cams)
            grid.pack(anchor=tk.W)
            for c, cam in enumerate(CAMERAS):
                tk.Label(grid, text=cam).grid(row=0, column=c)
                for r in range(4):  # RGB · RGBD · RGB-SEG · POINT
                    lab = tk.Label(grid, bg='black', width=TILE[0], height=TILE[1])
                    lab.grid(row=r + 1, column=c, padx=1, pady=1)
                    self.tiles[cam][base + r] = lab
                    if r == 3:  # POINT — 드래그로 회전 (두 구역이 같은 시점)
                        lab.bind('<B1-Motion>', lambda e, cm=cam: self.on_cloud_drag(e, cm))
                        lab.bind('<ButtonPress-1>', lambda e, cm=cam: self.on_cloud_press(e, cm))

        self.manual_win = None
        self.manual_panel = None
        self.tabs = texts
        cmd = tk.Frame(mapside)  # 왼쪽 버튼 열(물체·환경 · 미션 · 지도 · 보기) 맨 아래
        cmd.pack(fill=tk.X, pady=(6, 0))
        tk.Button(cmd, text='토픽보기', command=self.open_topic_viewer).pack(fill=tk.X)
        tk.Button(cmd, text='감지 물체 보기', command=self.open_objects_window).pack(fill=tk.X)
        tk.Button(cmd, text='조작 (명령·수동)', command=self.open_manual_window).pack(fill=tk.X)
        # 로봇 모델 — lv2_module6/urdf/<폴더>/urdf.xacro 중 고르면 확인 후 urdf:=<폴더>로 ROS 재시작 (모델은 기동 때만 읽힘)
        # 목록 = 호스트의 urdf/ (컨테이너 /ws/urdf와 같은 폴더) · 지금 모델 = /robot_description 메시 경로의 폴더 이름
        tk.Label(cmd, text='로봇 모델 (urdf/)').pack(anchor=tk.W, pady=(6, 0))
        models = sorted(p.parent.name for p in robot_view.URDF_DIR.glob('*/urdf.xacro'))
        self.urdf_choice = tk.StringVar(value='?')
        tk.OptionMenu(cmd, self.urdf_choice, *models, command=self.pick_urdf).pack(fill=tk.X)
        tk.Button(cmd, text='ROS 재시작', fg='#c62828', command=lambda: self.restart_ros()).pack(fill=tk.X, pady=(6, 0))
        self.sent = tk.Label(right, text='', fg='#2e7d32')
        self.sent.pack(anchor=tk.W)
        self.photos = []
        self.tile_photos = {}  # 타일 사진 별도 보관 — 타일을 격틱 갱신해도 홀틱에 GC로 지워지지 않게
        self.tick()

    # --- 미션
    def on_env(self):
        text = self.seed.get().strip()
        seed = self.sim.setup_env(int(text) if text.isdigit() else None)
        self.seed.delete(0, tk.END)
        self.seed.insert(0, str(seed))  # 같은 배치를 다시 하려면 이 seed 그대로

    def refresh_serial(self):
        """시리얼 장비 창 — debug/output/hardware/serial(SBC → OpenCR C) · input/hardware/serial(OpenCR → SBC S)
        파일에 새 줄이 붙었을 때만 다시 그린다 (맨 아래가 최신). 머리줄: 연결 상태 · 초당 줄 수"""
        pl = self.sim.opencr
        now = time.time()
        changed = [bool(self.serial_tail[k].poll()) for k in ('cmd', 'state')]
        rate = lambda log: sum(1 for t, _ in log if now - t < 1.0)
        if pl.comm_down:
            state = '통신 끊김 (C 안 읽음 · S 안 보냄)'
        elif pl.alive:
            state = '연결 — controller_manager가 실기 모드(mock_hardware:=false)로 붙어 있음'
        else:
            state = '대기 — C 명령 없음 (faker가 mock_hardware:=true면 시리얼을 안 씀)'
        cmd, st = self.serial_tail['cmd'].lines, self.serial_tail['state'].lines
        self.serial_head.config(text=f'시리얼 (OpenCR ↔ BeebotSystemHardware) — {state} · C {rate(cmd)}/s · S {rate(st)}/s')
        for (key, log), ch in zip((('cmd', cmd), ('state', st)), changed):
            if not ch:
                continue
            lines = [time.strftime('%H:%M:%S', time.localtime(t)) + f'.{int(t % 1 * 1000):03d}  {line}' for t, line in list(log)[-7:]]
            text = '\n'.join(lines)
            w = self.serial_text[key]
            if w.get('1.0', 'end-1c') != text:
                w.delete('1.0', tk.END)
                w.insert(tk.END, text)

    def start_explore(self):
        """[자동 탐사] → /cmd/operation_mode EXPLORE — state_manager가 지도 모드 MAPPING일 때만 허락,
        plan/explorer가 /map 프론티어로 Nav2 목표를 차례로 보냄 (끝나면 /explore/result → AUTO). 수동 조작을 만지면 바로 멈춤"""
        mode = (read_msg('state/map_mode')[0] or {}).get('data')
        if mode == 'COURSE':
            self.restart_ros('map:=slam', '지금은 코스 지도(COURSE)라 탐사할 수 없습니다 (매핑 중에만).\n'
                                          'RTAB-Map 매핑 모드(map:=slam)로 다시 띄운 뒤 [자동 탐사]를 다시 누르세요. 다시 띄울까요?')
            return
        if mode != 'MAPPING':
            self.sent.config(text=f'자동 탐사는 매핑 중에만 — 지금 지도 모드 {mode}. 먼저 [스캔 시작]')
            return
        send('/cmd/operation_mode', 'std_msgs/msg/String', {'data': 'EXPLORE'})
        self.sent.config(text='요청: 자동 탐사 (/cmd/operation_mode EXPLORE → state_manager → explorer)')

    def stop_explore(self):
        send('/cmd/operation_mode', 'std_msgs/msg/String', {'data': 'AUTO'})
        self.sent.config(text='요청: 탐사 멈춤 (/cmd/operation_mode AUTO)')

    def map_mode_request(self, mode):
        # COURSE(코스 지도, RTAB-Map 없음)면 모드를 바꿀 대상이 없음 → RTAB-Map 모드로 ROS 재시작을 물어봄
        if (read_msg('state/map_mode')[0] or {}).get('data') == 'COURSE':
            if mode == 'MAPPING':
                self.restart_ros('map:=slam', '지금은 코스 지도(COURSE)라 스캔할 수 없습니다.\n'
                                              'RTAB-Map 매핑 모드(map:=slam, 새 지도 폴더)로 ROS를 다시 띄울까요?')
            elif self.view.scan_choices():
                self.restart_ros('map:=saved', '지금은 코스 지도(COURSE)입니다.\n'
                                               '기본 지도(default 폴더, 없으면 가장 최근)로 위치추정 모드(map:=saved)로 ROS를 다시 띄울까요?')
            else:
                self.sent.config(text='위치추정 불가: 저장된 지도 없음 — 먼저 [스캔 시작]으로 매핑')
            return
        if mode == 'MAPPING':
            self.ask_scan_start()
            return
        send('/cmd/map_mode', 'std_msgs/msg/String', {'data': mode})
        self.sent.config(text=f'요청: 지도 모드 {mode} (/cmd/map_mode → state_manager)')

    def ask_scan_start(self):
        """[스캔 시작] — 지금 지도를 어떻게 할지 고름:
          이어서 스캔      지금 지도(같은 폴더 rtabmap.db)에 덧붙임 — /cmd/map_mode MAPPING (재시작 없음)
          지우고 처음부터  같은 폴더를 비우고 다시 — 📞 /rtabmap/rtabmap/reset → /cmd/map_mode MAPPING (재시작 없음)
          새 지도로        지금 지도는 그대로 두고 새 폴더 <날짜_시각>/ — /cmd/restart all map:=slam (ROS 재시작)
        [지도 저장]은 그때 쓰는 지도 폴더(/state/map_name)에 씀"""
        name = (read_msg('state/map_name')[0] or {}).get('data') or '(없음)'
        mode = (read_msg('state/map_mode')[0] or {}).get('data') or '?'
        win = tk.Toplevel(self.root)
        win.title('스캔 시작')
        win.transient(self.root)
        tk.Label(win, justify=tk.LEFT, padx=12, pady=8,
                 text=f'지금 지도: {name} (모드 {mode})\n스캔을 어떻게 시작할까요?').pack(anchor=tk.W)

        def pick(choice):
            win.destroy()
            if choice == 'continue':
                send('/cmd/map_mode', 'std_msgs/msg/String', {'data': 'MAPPING'})
                self.sent.config(text=f'요청: 지도 {name} 에 이어서 스캔 (/cmd/map_mode MAPPING)')
            elif choice == 'clear':
                from tkinter import messagebox
                if not messagebox.askyesno('지우고 처음부터', f'지도 {name} 의 내용(rtabmap.db)을 비우고 처음부터 스캔합니다.\n'
                                                            '되돌릴 수 없습니다. 진행할까요?'):
                    return
                send_service_request('rtabmap/rtabmap/reset', {})
                # 비운 뒤 매핑 모드로 (위치추정 중이었으면 바꿈) — 순서 보장 위해 조금 뒤에
                self.root.after(1500, lambda: send('/cmd/map_mode', 'std_msgs/msg/String', {'data': 'MAPPING'}))
                self.sent.config(text=f'요청: 지도 {name} 비우고 처음부터 스캔 (/rtabmap/rtabmap/reset → /cmd/map_mode MAPPING)')
            elif choice == 'new':
                self.restart_ros('map:=slam', f'지금 지도 {name} 는 그대로 두고, 새 지도 폴더(날짜_시각)로 ROS를 다시 띄워 스캔합니다.')
        for text, choice in (('이어서 스캔 — 지금 지도에 덧붙임', 'continue'),
                             ('지우고 처음부터 — 같은 지도 폴더를 비움', 'clear'),
                             ('새 지도로 — 지금 지도는 그대로, 새 폴더 (ROS 재시작)', 'new'),
                             ('취소', None)):
            tk.Button(win, text=text, anchor=tk.W, command=(lambda c=choice: pick(c)) if choice else win.destroy).pack(
                fill=tk.X, padx=12, pady=2)

    def save_map(self):
        name, _ = read_msg('state/map_name')  # state_manager 발행 — launch가 정한 RTAB-Map DB 폴더
        name = (name or {}).get('data')
        if not name:
            self.sent.config(text='지도 저장 불가: 지도 폴더 없음 (컨테이너를 map:=slam · saved 로 띄울 것)')
            return
        self.save_url = f'{MAP_DIR}/{name}/map'
        self.save_pending = send_service_request('map_saver/save_map', {'map_url': self.save_url})
        self.sent.config(text=f'요청: 지도 저장 (/map_saver/save_map → {self.save_url})')

    def sync_map_choice(self):
        """지도 목록·선택을 컨테이너 상태에 맞춤 — 목록 = /state/map_list, 선택 = /state/map_name (바뀌었을 때만 따라감,
        사람이 '선택없음'으로 겹침을 끈 건 그대로 둠). 따라갈 때는 /cmd/map_name을 다시 보내지 않음 (_pick_scan이 같은 이름은 안 보냄)"""
        names = tuple(self.view.scan_choices())
        if names != getattr(self, '_map_names', None):
            self._map_names = names
            self.refresh_scan_menu()
        cur = (read_msg('state/map_name')[0] or {}).get('data')
        if cur and cur != getattr(self, '_map_name_seen', None):
            self._map_name_seen = cur
            # 사람이 '선택없음'을 고른 동안은 따라가지 않음 (명시적으로 안 고른 것 — 위치추정 모드에서도)
            if cur in names and self.scan_choice.get() != cur and not getattr(self, '_map_none', False):
                self._syncing_map = True
                try:
                    self.scan_choice.set(cur)
                finally:
                    self._syncing_map = False

    def refresh_scan_menu(self):
        """지도 목록을 다시 읽음 (/state/map_list) — 새 지도 폴더가 생기면 보이게"""
        menu = self.scan_menu['menu']
        menu.delete(0, tk.END)
        for name in ['선택없음', *self.view.scan_choices()]:
            menu.add_command(label=name, command=tk._setit(self.scan_choice, name))

    def conn_text(self, value, names=None):
        if value is None:
            return '연결 안 됨'
        return (names or {}).get(value, value)

    def slam_text(self):
        """지도 모드 = state_manager가 발행한 /state/map_mode 그대로 (짐작 안 함) · 지도 저장 응답 확인"""
        if self.save_pending:
            res, _ = read_service_response('map_saver/save_map')
            if res and res.get('request_id') == self.save_pending:
                self.save_pending = None
                ok = res.get('ok') and res.get('data', {}).get('result')
                self.sent.config(text=f"지도 저장 → {self.save_url}.pgm·yaml: "
                                      f"{'완료' if ok else '실패 — ' + str(res.get('error', res.get('data')))}")
                if ok:
                    self.refresh_scan_menu()
        mode, _ = read_msg('state/map_mode')
        mode = (mode or {}).get('data') if mode else None
        names = {'MAPPING': '스캔 중 (지도 작성)', 'LOCALIZATION': '위치추정 (지도 고정)',
                 'COURSE': '코스 지도 (RTAB-Map 없음 — 바꿀 수 없음)'}
        line = '지도 모드: ' + (names.get(mode, mode) if mode is not None else '연결 안 됨')
        _, meta, age = read_grid('map')
        line += (f"\nNav2 지도 /map {meta['width']}×{meta['height']}칸 · {age:.0f}s 전 바뀜" if meta and this_run(age)
                 else "\nNav2 지도 /map: 이번 실행에서 아직 안 옴 (RTAB-Map은 카메라가 들어와야 지도를 냄)")
        return line + (' · 저장 응답 대기…' if self.save_pending else '')

    def mission_text(self):
        lines = [f"{z['name']} {scene.PROBLEMS[z['problem']][0]}" for z in self.sim.stations] or ['미션 지역 없음 — 지도 오른쪽 클릭']
        env = self.sim.env
        lines.append(f"환경 seed {env['seed']} · {int(time.time() - env['t0'])}s" if env else '환경 없음')
        return '\n'.join(lines)

    # --- 조작
    def on_background(self, name):
        self.sim.renderer.set_background(None if name == NO_BG else BG_DIR / name)

    def on_floor(self, name):
        self.sim.renderer.set_floor(None if name == NO_FLOOR else FLOOR_DIR / name)

    def on_wall(self, name):
        self.sim.renderer.set_wall(None if name == NO_WALL else WALL_DIR / name)

    def on_stream(self):
        self.sim.streaming = self.stream.get()
        if not self.sim.streaming:
            self.sim.stop_streaming()

    def open_manual_window(self):
        """조작 창 — 직접 만지면 수동 모드, 명령을 내리면 자동 모드, 닫으면 바퀴 정지 + 자동 모드"""
        if self.manual_win is not None and self.manual_win.winfo_exists():
            self.manual_win.lift()
            return
        self.manual_win = tk.Toplevel(self.root)
        self.manual_win.title('조작')
        self.manual_panel = ManualPanel(self.manual_win, self.sim, on_close=self.manual_win.destroy,
                                        frame_var=self.show_frames, object_faces=self.world_object_faces,
                                        detected_var=self.show_detected, base_map=self.view.base_map_estimate)
        self.manual_panel.pack(fill=tk.BOTH, expand=True)
        self.manual_panel.show()
        self.manual_win.protocol('WM_DELETE_WINDOW', self.close_manual_window)

    def close_manual_window(self):
        if self.manual_panel is not None:
            self.manual_panel.hide()
        if self.manual_win is not None:
            self.manual_win.destroy()
        self.manual_win, self.manual_panel = None, None

    def on_map_press(self, e):
        x, y = self.view.world(e.x, e.y)
        whole = shift_held(e)  # Shift = 미션 지역·B 지역 판 통째로 (그냥 드래그는 퍽·로봇 하나)
        self.drag_zone = None
        with self.sim.lock:
            # Shift + 판 안 → 판 위 퍽·QR 판·로봇이 있어도 판 통째로 (퍽에 가려 못 잡던 문제)
            st = self.sim.station_at(x, y)
            if whole and (st or scene.in_zone(self.sim.goal, x, y)):
                zone = st or self.sim.goal
                members = [zone] + ([p for p in self.sim.pucks + self.sim.props if p.get('station') == st['name']] if st else [])
                self.drag_zone = (st, x, y, [(o, o['x'], o['y']) for o in members])
                self.drag_robot = False
                self.drag = None
                self.map_pan = None
                return
        rx, ry, _, _ = self.sim.robot_xy_yaw()
        self.drag_robot = math.dist((x, y), (rx, ry)) < 0.25
        if self.drag_robot:
            self.drag = None
            if self.sim.opencr.alive:
                self.sim.teleport_robot(x, y)
            self.map_pan = None
            return
        with self.sim.lock:
            near = [p for p in self.sim.pucks + self.sim.props if math.dist((x, y), (p['x'], p['y'])) < 0.2]
        self.drag = min(near, key=lambda p: math.dist((x, y), (p['x'], p['y']))) if near else None
        # 빈 곳 왼쪽 드래그 = 지도 이동 (확대 상태에서만)
        self.map_pan = (e.x, e.y, self.view.cx, self.view.cy)

    def on_map_drag(self, e):
        if getattr(self, 'drag_robot', False):
            if self.sim.opencr.alive:  # fake_opencr는 실시간, mock은 릴리스 때 1회
                x, y = self.view.world(e.x, e.y)
                self.sim.teleport_robot(x, y)
            return
        if getattr(self, 'drag_zone', None):  # 미리보기 — 판은 마우스를 따라오고, 놓일 칸은 점선으로
            st, x0, y0, members = self.drag_zone
            x, y = self.view.world(e.x, e.y)
            zone = members[0][0]
            with self.sim.lock:
                for o, ox, oy in members:
                    o['x'], o['y'] = round(ox + x - x0, 3), round(oy + y - y0, 3)
            c = self.sim.snap_cell(zone['x'], zone['y'])
            ok = self.sim.zone_cell_ok(st, c)
            c = c or (members[0][1], members[0][2])
            self.view.drop_hint = (c[0], c[1], zone['sx'], zone['sy'], ok)
            self.redraw_map()
            return
        if self.drag:
            with self.sim.lock:
                self.drag['x'], self.drag['y'] = (round(v, 3) for v in self.view.world(e.x, e.y))
        elif self.view.zoom > 1.0:
            x0, y0, cx0, cy0 = self.map_pan
            self.view.cx, self.view.cy = cx0 - (e.x - x0) / self.view.zoom, cy0 - (e.y - y0) / self.view.zoom
        self.redraw_map()

    def on_map_release(self, _):
        if getattr(self, 'drag_robot', False):
            self.drag_robot = False
            if not self.sim.opencr.alive:
                x, y = self.view.world(_.x, _.y)
                self.sim.teleport_robot(x, y)
            return
        if getattr(self, 'drag_zone', None):
            st, x0, y0, members = self.drag_zone
            self.drag_zone = None
            self.view.drop_hint = None
            x, y = self.view.world(_.x, _.y)
            with self.sim.lock:  # 원위치로 되돌린 뒤 칸에 맞춰 옮김 (안 되는 칸이면 그대로)
                for o, ox, oy in members:
                    o['x'], o['y'] = ox, oy
            cx, cy = members[0][1] + x - x0, members[0][2] + y - y0
            ok = self.sim.move_station(st['name'], cx, cy) if st else self.sim.move_goal(cx, cy)
            what = st['name'] if st else 'B 지역'
            self.sent.config(text=f'{what} 옮김' if ok else f'{what}: 출발 칸 · B · 다른 미션 지역 칸에는 못 놓음 — 그대로')
        self.drag = None
        self.redraw_map()

    def on_map_pan_press(self, e):
        self.map_pan = (e.x, e.y, self.view.cx, self.view.cy)
        self.ctx_press = (e.x, e.y)

    def on_map_context(self, e):
        """오른쪽 클릭(끌지 않고 뗌) → 그 자리 메뉴: 미션 지역 위면 문제 바꾸기·QR 돌리기·지우기, 빈 칸이면 놓기·B 지역·로봇"""
        px, py = getattr(self, 'ctx_press', (e.x, e.y))
        if abs(e.x - px) + abs(e.y - py) > 4:
            return  # 지도 이동이었음
        x, y = self.view.world(e.x, e.y)
        menu = tk.Menu(self.root, tearoff=0)
        problems = sorted(scene.PROBLEMS.items(), key=lambda kv: kv[1][0])
        st = self.sim.station_at(x, y)
        if st:
            menu.add_command(label=f"{st['name']} · {scene.PROBLEMS[st['problem']][0]}", state=tk.DISABLED)
            sub = tk.Menu(menu, tearoff=0)
            for code, v in problems:
                sub.add_command(label=v[0], command=lambda c=code: self.sim.add_station(st['x'], st['y'], c))
            menu.add_cascade(label='문제 바꾸기', menu=sub)
            menu.add_command(label='QR 판 돌리기 (90°)', command=lambda: self.sim.rotate_station(st['name']))
            menu.add_command(label='미션 지역 지우기', command=lambda: self.sim.remove_station(st['name']))
        else:
            sub = tk.Menu(menu, tearoff=0)
            for code, v in problems:
                sub.add_command(label=v[0], command=lambda c=code: self.place_station(x, y, c))
            menu.add_cascade(label='여기에 미션 지역', menu=sub)
            menu.add_command(label='B 지역을 여기로', command=lambda: self.sent.config(
                text='B 지역 옮김' if self.sim.move_goal(x, y) else 'B 지역: 출발 칸 · 미션 지역 칸에는 못 놓음'))
            menu.add_command(label='로봇을 여기로', command=lambda: self.sim.teleport_robot(x, y))
        # Nav2 계획 주행 — 지도가 고정된 상태(위치추정 · 코스 지도)에서만. 매핑 중엔 지도가 바뀌어 계획이 흔들림
        mode = (read_msg('state/map_mode')[0] or {}).get('data')
        menu.add_separator()
        menu.add_command(label='여기로 이동 (Nav2 계획 주행)' + ('' if mode in ('LOCALIZATION', 'COURSE') else f' — 지도 모드 {mode} (위치추정에서)'),
                         state=tk.NORMAL if mode in ('LOCALIZATION', 'COURSE') else tk.DISABLED,
                         command=lambda: self.navigate_to(x, y))
        if self.nav_goal_id:
            menu.add_command(label='이동 취소', command=self.cancel_navigation)
        # 초기 위치 — 위치추정(RTAB-Map)이 위치를 잃었을 때 "로봇은 여기"라고 알려 줌 (/initialpose → RTAB-Map, RViz 2D Pose Estimate와 같음)
        menu.add_command(label='여기가 로봇 위치 (초기 위치 지정 /initialpose)' + ('' if mode == 'LOCALIZATION' else f' — 지도 모드 {mode} (위치추정에서)'),
                         state=tk.NORMAL if mode == 'LOCALIZATION' else tk.DISABLED,
                         command=lambda: self.set_initial_pose(x, y))
        menu.tk_popup(e.x_root, e.y_root)

    def to_map(self, x, y):
        """미로(시뮬레이터 지도) 좌표 → map 좌표 + 회전각 (RTAB-Map map→odom 보정 반영 — map_to_world의 역변환)"""
        tx, ty, th = self.view.map_to_world()
        dx, dy = x - tx, y - ty
        return math.cos(th) * dx + math.sin(th) * dy, -math.sin(th) * dx + math.cos(th) * dy, th

    def set_initial_pose(self, x, y, yaw=None):
        """[초기 위치 지정] → /initialpose (geometry_msgs/PoseWithCovarianceStamped, map) — RTAB-Map이 위치추정 모드일 때만 받음
        (매핑 중엔 'Initial pose can only be set in localization mode … ignoring it' — librtabmap_core).
        map 좌표는 **고정 기준**으로: 지도는 출발 칸에서 시작해 만드므로 map 원점 = 출발 칸, 방향 0°.
        예전엔 map_to_world(= 로봇이 믿는 자기 위치와 실제 위치를 맞댄 변환)로 바꿔서, 실제 자리를 찍어도 로봇이 믿던 위치가 그대로
        나와 보정이 0이었음 (어긋난 만큼을 스스로 지움). 방향을 안 주면 지금 실제 로봇 방향. 공분산 = 위치 0.05m² · 방향 0.07rad²"""
        mx, my = x - self.sim.spawn[0], y - self.sim.spawn[1]
        if yaw is None:
            yaw = self.sim.robot_xy_yaw()[2]
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05
        cov[35] = 0.07
        send('/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped', {
            'header': {'stamp': {'sec': 0, 'nanosec': 0}, 'frame_id': 'map'},
            'pose': {'pose': {'position': {'x': round(mx, 3), 'y': round(my, 3), 'z': 0.0},
                              'orientation': {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw / 2), 'w': math.cos(yaw / 2)}},
                     'covariance': cov}})
        self.sent.config(text=f'요청: 초기 위치 (map {mx:+.2f}, {my:+.2f}, {math.degrees(yaw):+.0f}°) — /initialpose → RTAB-Map')

    def snap_to_real_pose(self):
        """[실제 위치로 맞추기] — 시뮬레이터는 진짜 위치를 앎(fake_opencr) → 그대로 /initialpose. 위치추정 모드에서만 (RTAB-Map이 그때만 받음)"""
        mode = (read_msg('state/map_mode')[0] or {}).get('data')
        if mode != 'LOCALIZATION':
            self.sent.config(text=f'실제 위치로 맞추기: 위치추정 모드에서만 (지금 {mode}) — RTAB-Map이 매핑 중엔 초기 위치를 무시함. '
                                  '[위치추정으로] 뒤에 다시')
            return
        x, y, yaw, _ = self.sim.robot_xy_yaw()
        self.set_initial_pose(x, y, yaw)

    def current_urdf(self):
        """지금 돌고 있는 로봇 모델 폴더 — /robot_description 메시 경로 package://ros_system/urdf/<폴더>/meshes 에서.
        ponytail: 메시 없는 모델(beebot_normal 등)은 URDF에 폴더 이름이 안 남아 마지막으로 고른 것 · 모르면 '?'
        (정확히 하려면 ROS가 모델 이름을 토픽으로 내야 함)"""
        import re
        m = re.search(r'package://ros_system/urdf/([^/"]+)/meshes/', getattr(self, '_model_desc', None) or '')
        return m.group(1) if m else getattr(self, '_picked_urdf', '?')

    def pick_urdf(self, name):
        """[로봇 모델] 고름 → 확인 → /cmd/restart urdf:=<폴더> (다른 인자 map:= 등은 test-faker가 그대로 둠). 취소면 원래대로"""
        now = self.current_urdf()
        if name == now:
            return
        if not self.restart_ros(f'urdf:={name}', f'로봇 모델을 {name} 로 바꿉니다 (lv2_module6/urdf/{name}/urdf.xacro, 지금 {now}).\n'
                                                  '모델(URDF)은 ROS 기동 때만 읽혀서 다시 띄워야 적용됩니다.'):
            self.urdf_choice.set(now)
        else:
            self._picked_urdf = name

    def restart_ros(self, args='', why=''):
        """[ROS 재시작] → /cmd/restart "all [args]" (args 비움 = 같은 설정, 있으면 그 launch 인자만 바꿈 — 예 map:=slam · urdf:=beebot_normal, 나머지 유지).
        확인을 누르면 True
        state_manager가 받아 이 launch를 정상 종료 → (테스트베드는 빌드) → 같은 명령줄로 다시 띄움 — 실기 bringup도 같은 길.
        결과는 /state/restart (상태판). 한 노드만은 "node <이름>" (ros2 topic pub --once /cmd/restart …)"""
        from tkinter import messagebox
        if not messagebox.askyesno('ROS 재시작', (why + '\n\n' if why else '') +
                                   f'컨테이너의 ROS를 전부 껐다가 다시 띄웁니다 (1~2분){" — " + args if args else ""}.\n'
                                   '진행 중 작업·매핑은 끊깁니다 (RTAB-Map 지도는 정상 종료로 저장). 진행할까요?'):
            return False
        send('/cmd/restart', 'std_msgs/msg/String', {'data': ('all ' + args).strip()})
        self.sent.config(text=f'요청: ROS 재시작{" " + args if args else ""} (/cmd/restart all → state_manager) — 1~2분 뒤 다시 연결')
        return True

    def restart_node(self, name):
        """진단 칸 노드 줄 → 확인 → /cmd/restart "node <이름>" (state_manager가 그 프로세스만 같은 명령줄·파라미터로 다시 — 결과 /state/restart)"""
        from tkinter import messagebox
        lifecycle = name in ('planner_server', 'controller_server', 'behavior_server', 'bt_navigator', 'velocity_smoother',
                             'collision_monitor', 'map_server', 'map_saver')
        if not messagebox.askyesno('노드 재시작', f'{name} 노드만 껐다가 같은 파라미터로 다시 띄웁니다.\n' +
                                   ('Nav2 수명주기 노드라 다시 떠도 unconfigured — lifecycle_manager가 다시 켜야 합니다 '
                                    '(안 되면 [ROS 재시작]).\n' if lifecycle else '') + '진행할까요?'):
            return
        send('/cmd/restart', 'std_msgs/msg/String', {'data': f'node {name}'})
        self.sent.config(text=f'요청: 노드 재시작 {name} (/cmd/restart → state_manager) — 결과는 상태판 /state/restart')

    def navigate_to(self, x, y):
        """[여기로 이동] → 자동 모드(/cmd/operation_mode AUTO — 수동 명령이 twist_mux에서 이기지 않게) +
        Nav2 /navigate_to_pose 목표 (map 좌표 = 미로 좌표 − 출발 칸). 도착 방향 = 로봇에서 목표 쪽"""
        rx, ry = self.sim.robot_xy_yaw()[:2]
        tx, ty, th = self.view.map_to_world()  # 미로 → map: 역변환 (저장 지도면 map 원점 ≠ 출발 칸)
        dx, dy = x - tx, y - ty
        mx, my = math.cos(th) * dx + math.sin(th) * dy, -math.sin(th) * dx + math.cos(th) * dy
        yaw = math.atan2(y - ry, x - rx) - th  # 도착 방향 = 로봇에서 목표 쪽 (map 기준)
        cmd_manual_mode(False)
        self.nav_goal_id = send_action_goal('navigate_to_pose', {'x': round(mx, 3), 'y': round(my, 3), 'yaw': round(yaw, 3)})
        self.view.nav_goal = (x, y)
        self.sent.config(text=f'요청: 여기로 이동 (map {mx:+.2f}, {my:+.2f}) — /navigate_to_pose')
        self.redraw_map()

    def cancel_navigation(self):
        if self.nav_goal_id:
            cancel_action_goal('navigate_to_pose', self.nav_goal_id)
            self.sent.config(text='요청: 이동 취소')

    def nav_text(self):
        """이동 목표 진행 — accept · feedback(남은 거리) · result. 끝나면 깃발 지움"""
        if not self.nav_goal_id:
            return None
        res, _ = read_action_file('navigate_to_pose', 'result')
        if res and res.get('goal_id') == self.nav_goal_id:
            self.nav_goal_id = None
            self.view.nav_goal = None
            return f"이동 {res.get('status')}"
        acc, _ = read_action_file('navigate_to_pose', 'accept')
        if acc and acc.get('goal_id') == self.nav_goal_id and not acc.get('accepted', True):
            self.nav_goal_id = None
            self.view.nav_goal = None
            return f"이동 거절: {acc.get('error', '')}"
        fb, _ = read_action_file('navigate_to_pose', 'feedback')
        if fb and fb.get('goal_id') == self.nav_goal_id:
            d = (fb.get('data') or {}).get('distance_remaining')
            return f'이동 중 — 남은 거리 {d:.2f} m' if isinstance(d, (int, float)) else '이동 중'
        return '이동 목표 보냄 — 수락 대기'

    def place_station(self, x, y, problem):
        name = self.sim.add_station(x, y, problem)
        self.sent.config(text=f'미션 지역 {name}: {scene.PROBLEMS[problem][0]}' if name else '출발 칸 · B 지역 칸에는 못 놓음')

    def on_map_pan_drag(self, e):
        if self.view.zoom <= 1.0:
            return
        x0, y0, cx0, cy0 = self.map_pan
        self.view.cx, self.view.cy = cx0 - (e.x - x0) / self.view.zoom, cy0 - (e.y - y0) / self.view.zoom
        self.redraw_map()

    def on_map_wheel(self, e):
        self.on_map_wheel_step(1 if e.delta > 0 else -1, e.x, e.y)

    def on_map_wheel_step(self, direction, lx=None, ly=None):
        old = self.view.zoom
        new = min(4.0, max(0.25, old * (1.15 if direction > 0 else 1 / 1.15)))
        if new == old:
            return
        if lx is not None:
            # 커서 아래 점 고정 — 중심 이동 (zoom 1에서도 커서 기준 확대)
            x0, y0, _, _ = self.view._roi()
            fx, fy = x0 + lx / old, y0 + ly / old
            h, w = self.view.size[1], self.view.size[0]
            self.view.cx = fx - (lx - w / 2) / new
            self.view.cy = fy - (ly - h / 2) / new
        self.view.zoom = new
        self.redraw_map()

    def on_cloud_press(self, e, cam):
        self.cloud_press = (e.x, e.y, *self.cloud_view[cam])

    def on_cloud_drag(self, e, cam):
        x0, y0, yaw, pitch = self.cloud_press
        self.cloud_view[cam] = [yaw + (e.x - x0) * 0.01, max(-1.5, min(1.5, pitch - (e.y - y0) * 0.01))]
        self.refresh_cam(cam)  # 틱을 기다리지 않고 즉시

    def on_robot_press(self, e):
        self.robot_drag = (e.x, e.y)

    def on_robot_drag(self, e):
        x0, y0 = self.robot_drag
        if shift_held(e):  # Shift+드래그 = 시점 이동
            self.robot_cam.pan(e.x - x0, e.y - y0)
        else:
            self.robot_cam.orbit(e.x - x0, e.y - y0)
        self.robot_drag = (e.x, e.y)
        now = time.monotonic()  # 메시 렌더 0.13초 — 드래그 중엔 70ms로 솎음 (각도는 계속 누적)
        if now - getattr(self, '_rdrag_t', 0) < 0.07:
            return
        self._rdrag_t = now
        self.refresh_robot()  # 틱을 기다리지 않고 즉시 (드래그가 부드럽게)

    def on_toggle_collision(self):
        on = self.show_collision.get()
        if on and self.robot_model_col is None:
            desc, _ = read_msg('robot_description')
            if desc:
                self.robot_model_col = robot_view.RobotModel(desc['data'], collision=True)
        self.sim.show_collision = on
        self.sim._render_t = 0  # 카메라도 바로 다시 그림
        self.refresh_robot()

    def refresh_robot(self):
        q = getattr(self, '_last_q', None)
        model = self.robot_model_col if self.sim.show_collision and self.robot_model_col else self.robot_model
        if model is None or not q:
            return
        rx, ry, ryaw, _ = self.sim.robot_xy_yaw()
        T_wb = scene.transform(scene.rpy_matrix(0, 0, ryaw), (rx, ry, 0))
        faces = model.faces(q) + self.world_object_faces(T_wb)
        img = self.robot_cam.render(faces, ROBOT_VIEW)
        self.draw_wheels(img, q, getattr(self, '_last_wheel', [0.0, 0.0]))
        if self.show_detected.get():
            draw_detected_3d(img, self.robot_cam, ROBOT_VIEW, self.view.base_map_estimate())
        if self.show_frames.get():
            self.draw_frames(img, q)
        self._robot_ph = to_photo(img)
        self.robot_label.config(image=self._robot_ph, width=ROBOT_VIEW[0], height=ROBOT_VIEW[1])

    def refresh_cam(self, cam):
        for i, (lab, img) in enumerate(zip(self.tiles[cam], camera_tiles(self.sim, cam, self.cloud_view[cam]))):
            ph = to_photo(img)
            self.tile_photos[(cam, i)] = ph
            lab.config(image=ph, width=TILE[0], height=TILE[1])

    def draw_wheels(self, img, q, wheel_vel):
        robot_view.draw_wheel_spin(img, self.robot_model, self.robot_cam, ROBOT_VIEW, q, wheel_vel)

    def draw_frames(self, img, q):
        """관절축 트라이어드 (자식 링크 프레임: X빨·Y초·Z파). 바퀴는 자식 프레임이 같이 돌아 회전도 보임"""
        import numpy as _np
        cols = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]
        for _, (child, _, _, _, _) in self.robot_model.joints.items():
            T = self.robot_model.kin.pose(child, q)
            o, R = T[:3, 3], T[:3, :3]
            pts = [o] + [o + R[:, i] * 0.05 for i in range(3)]
            uv, z = self.robot_cam.project(pts, ROBOT_VIEW)
            if (z < 0.02).any():
                continue
            p = uv.astype(int)
            for i in range(3):
                cv2.line(img, tuple(p[0]), tuple(p[i + 1]), cols[i], 2, cv2.LINE_AA)

    def world_object_faces(self, T_wb):
        """퍽·장애물·이동물체·신호등을 로봇 루트 프레임 면 목록으로 — 상태 3D에 같이 그림"""
        import numpy as _np
        R, t = _np.asarray(T_wb[:3, :3], float), _np.asarray(T_wb[:3, 3], float)
        out = []
        with self.sim.lock:
            things = [dict(p) for p in self.sim.pucks + self.sim.props]
        for p in things:
            color = scene.PALETTE.get(p['color'], (128, 128, 128)) if isinstance(p['color'], str) else tuple(p['color'])
            color = tuple(int(v) for v in color)
            shape = p['shape']
            r_or_sx, h = p['size'][0], p['size'][1]
            zo = float(p.get('z', 0.0))
            if p.get('kind') == 'traffic_light':  # 폴 + 헤드 + 3램프(켜진 것만 상태색)
                from scene import _tl_dim as _dim
                parts = [(robot_view.primitive_faces('box', 0.04, scene.TL_POLE_H, 0.04), scene.TL_POLE_H / 2,
                          (70, 70, 70)),
                         (robot_view.primitive_faces('box', *scene.TL_HEAD[:2], scene.TL_HEAD[3] - scene.TL_HEAD[2]),
                          (scene.TL_HEAD[2] + scene.TL_HEAD[3]) / 2, (40, 40, 40))]
                for _name, _lz in scene.TL_LAMPS:
                    _on = p.get('state') == _name
                    _rgb = tuple(int(v) for v in scene.TL_COLORS[_name]) if _on else _dim(scene.TL_COLORS[_name])
                    _sph = robot_view.primitive_faces('sphere', scene.TL_LAMP_R, 0)
                    for _sx in (0.06, -0.06):
                        parts.append(([(vv + _np.array([_sx, 0.0, 0.0])) for vv in _sph], _lz, _rgb))
            elif shape == 'sphere':
                parts = [(robot_view.primitive_faces(shape, r_or_sx, 0), r_or_sx + zo, color)]
            elif shape == 'cylinder' and p.get('lying'):  # 누운 원기둥 — 상자로 근사 (길이 = 높이, 폭·높이 = 지름)
                parts = [(robot_view.primitive_faces('box', h, r_or_sx * 2, r_or_sx * 2), r_or_sx + zo, color)]
            elif shape == 'cylinder':
                parts = [(robot_view.primitive_faces(shape, r_or_sx, h), h / 2 + zo, color)]
            else:
                parts = [(robot_view.primitive_faces(shape, p.get('sx', r_or_sx), h, p.get('sy', r_or_sx)),
                          h / 2 + zo, color)]
            cy, sy = math.cos(p.get('yaw', 0.0)), math.sin(p.get('yaw', 0.0))
            Rz = _np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            c = _np.array([p['x'], p['y'], 0.0])
            for faces, z0, col in parts:
                cc = c + _np.array([0.0, 0.0, z0])
                for f in faces:
                    out.append(((_np.asarray(f, float) @ Rz.T + cc - t) @ R, col))
        return out

    def coords_text(self):
        """map · odom · base_footprint 좌표 (map = odom + spawn)"""
        odom, _ = read_msg('odom')
        if not odom:
            return '좌표: (데이터 없음 — test-faker 실행 중인지 확인)'
        pose = odom['pose']['pose']
        ox, oy = pose['position']['x'], pose['position']['y']
        oyaw = math.degrees(yaw_of(pose['orientation']))
        mx, my = self.view.from_map(ox, oy)
        base = self.view.base_from_tf()
        lines = [f'map   ({mx:+.2f}, {my:+.2f}) yaw {oyaw:+.0f}°',
                 f'odom  ({ox:+.2f}, {oy:+.2f}) yaw {oyaw:+.0f}°']
        if base is not None:
            lines.append(f'base  ({base[0]:+.2f}, {base[1]:+.2f}) yaw {math.degrees(base[2]):+.0f}°')
        else:
            lines.append('base  (없음 — /tf 대기)')
        return '\n'.join(lines)

    def open_objects_window(self):
        """감지 물체 보기 새 창 (objects_window.py) — 융합 트랙 · 카메라별 검출 표 + 확대도 + 두 카메라 영상. 누를 때마다 독립 창"""
        from objects_window import ObjectsWindow
        ObjectsWindow(self.root, self.sim, overlay_tracked, camera_detections, CAM_COLORS, ROS_FRESH_S,
                      localization_error=self.view.localization_error)

    def open_topic_viewer(self):
        """토픽 보기 새 창 — 콤보박스로 토픽 선택, 0.5초마다 갱신. 버튼을 누를 때마다 독립 창."""
        win = tk.Toplevel(self.root)
        win.title('토픽 보기')
        win.geometry('640x480')
        topics, _ = read_topic_list()
        names = [t['name'] for t in (topics or {}).get('topics', []) if t.get('name')] if topics else []
        if not names:  # list.json 없으면 파일에서 직접 수집
            try:
                names = sorted('/' + p.name for p in TOPIC_DIR.iterdir()
                               if p.is_dir() and (p / 'message').exists())
            except OSError:
                names = []
        box = ttk.Combobox(win, values=names, width=50)
        box.pack(fill=tk.X, padx=4, pady=4)
        if names:
            box.set(names[0])
        meta = tk.Label(win, text='', anchor=tk.W)
        meta.pack(fill=tk.X, padx=4)
        text = tk.Text(win, font=('Menlo', 10))
        text.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        def refresh():
            if not win.winfo_exists():
                return
            name = box.get().strip()
            if name:
                data, age = read_msg(name)
                meta.config(text=f'{name}  ({age:.1f}초 전)' if age is not None
                            else f'{name} (없음 — test-faker 실행 중인지 확인)')
                text.delete('1.0', tk.END)
                if data is not None:
                    try:
                        text.insert(tk.END, json.dumps(data, ensure_ascii=False, indent=1)[:20000])
                    except (TypeError, ValueError):
                        text.insert(tk.END, str(data)[:20000])
            win.after(500, refresh)
        refresh()

    def redraw_map(self):
        """지도만 다시 그림 (~1ms) — 틱(0.25초) 말고 드래그·이동·확대 때도 바로 불러서 끊김 없게"""
        self._map_ph = to_photo(self.view.rulers(self.view.viewport(self.view.render())))
        self.map_label.config(image=self._map_ph)

    def tick(self):
        self.tick_n = getattr(self, 'tick_n', 0) + 1
        self.redraw_map()
        photos = [self._map_ph]  # Tk가 이미지를 붙잡도록 보관
        desc, _ = read_msg('robot_description')
        if desc and desc['data'] != getattr(self, '_model_desc', None):  # 처음 · URDF가 바뀜
            self._model_desc = desc['data']
            if hasattr(self, 'urdf_choice'):  # 창을 다 만들기 전에 불릴 수 있음
                self.urdf_choice.set(self.current_urdf())
            self.robot_model = robot_view.RobotModel(desc['data'])
            self.robot_model_col = None  # 충돌 모양은 켤 때 다시
        q, wheel_vel = current_joints(self.sim)
        self._last_q, self._last_wheel = q, wheel_vel
        if self.robot_model is not None and q:
            if self.tick_n % 4 == 0 or not hasattr(self, '_robot_ph'):
                self.refresh_robot()  # 메시 렌더 0.12초 — 1초마다만 (드래그는 즉시)
            photos.append(self._robot_ph)
        else:
            blank = np.zeros((ROBOT_VIEW[1], ROBOT_VIEW[0], 3), np.uint8)
            cv2.putText(blank, 'waiting /robot_description', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (200, 200, 200), 1)
            photos.append(to_photo(blank))
        self.robot_label.config(image=photos[-1], width=ROBOT_VIEW[0], height=ROBOT_VIEW[1])
        self.coords_label.config(text=self.coords_text())
        self.mission_label.config(text=self.mission_text())
        if self.dice_label is not None:  # mission_manager 단계 · 신호등 진행 — READ가 되면 신호 자동 시작
            st, _ = read_msg('mission/state')
            state = (st or {}).get('data', '-') if isinstance(st, dict) else (st or '-')
            sig = self.sim.signal_text()
            if self.auto_signal.get() and str(state).startswith('READ') and sig == '신호 대기':
                self.sim.start_signal()
            prob = self.sim.dice_problem or {}
            self.dice_label.config(text=f"{state}\n{sig}\n문제 {prob.get('name', '-')} · {prob.get('signal', '')} · {prob.get('barcode', '')}")
        if self.tick_n % 2 == 0:
            self.sync_map_choice()
            nav = self.nav_text()
            self.slam_label.config(text=self.slam_text() + (f'\n{nav}' if nav else ''))
        if self.sim.env and self.sim.light_change:
            self.light.set(round(self.sim.light, 2))
        if self.tick_n % 2 == 0:  # 카메라 타일은 0.5초마다 (Tk 디코딩 부하 분산)
            for cam in self.tiles:
                self.refresh_cam(cam)
                for i, lab in enumerate(self.tiles[cam]):
                    photos.append(self.tile_photos[(cam, i)])
        self.refresh_serial()  # 시리얼 장비 창 — 파일에 새 줄이 붙었을 때만 다시 그림
        if self.tick_n % 2 == 0:  # 상태 텍스트도 0.5초마다
            for widget, lines in ((self.status, status_lines(self.sim)),):
                y, x = widget.yview()[0], widget.xview()[0]  # 다시 써도 스크롤 위치 유지
                widget.delete('1.0', tk.END)
                for text, tag in lines:
                    widget.insert(tk.END, text + '\n', (tag,) if tag else ())
                widget.yview_moveto(y)
                widget.xview_moveto(x)
        if self.tick_n % 4 == 0:  # 진단(diagnostics_agg 대용량)은 1초마다
            y, x = self.diag.yview()[0], self.diag.xview()[0]
            self.diag.delete('1.0', tk.END)
            for text, tag in diagnostics_lines():
                tags = tuple(tag.split())
                for t in tags:
                    if t.startswith('rs:') and t not in self._rs_tags:  # 재시작할 수 있는 노드 줄 — 누르면 확인 → node 재시작
                        self._rs_tags.add(t)
                        self.diag.tag_config(t, underline=True)  # 색은 진단 수준 그대로
                        self.diag.tag_bind(t, '<Button-1>', lambda e, n=t[3:]: self.restart_node(n))
                        self.diag.tag_bind(t, '<Enter>', lambda e: self.diag.config(cursor='hand2'))
                        self.diag.tag_bind(t, '<Leave>', lambda e: self.diag.config(cursor=''))
                self.diag.insert(tk.END, text + '\n', tags)
            self.diag.yview_moveto(y)
            self.diag.xview_moveto(x)
        self.photos = photos
        self.root.after(250, self.tick)


def snapshot(sim, view, path):
    """창 없이 한 장: 지도 | 카메라 타일 2줄"""
    sim.step()
    cols = [camera_tiles(sim, cam, (0.6, -0.4)) for cam in CAMERAS]
    tiles = np.vstack([np.hstack([c[k] for c in cols]) for k in range(8)])  # 열 = 카메라, 줄 = HW 4장 · ROS 4장
    m = view.render()
    h = max(m.shape[0], tiles.shape[0])
    pad = lambda a: np.vstack([a, np.full((h - a.shape[0], a.shape[1], 3), 255, np.uint8)])
    cv2.imwrite(path, np.hstack([pad(m), pad(tiles)]))
    for text, tag in status_lines(sim) + diagnostics_lines():
        print(('! ' if {'warn', 'error'} & set(tag.split()) else '  ') + text)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--snapshot', help='창 없이 화면 한 장을 이 PNG로 저장하고 상태를 출력한 뒤 끝냄')
    parser.add_argument('--spawn', help='map 원점의 미로 좌표 "x,y" (기본: 코스 spawn)')
    parser.add_argument('--course', default='twin_closed_maze', help='코스 = lv2_module6/course/<이름> (주사위 미션: dice_maze)')
    parser.add_argument('--dice', default='q1', help='dice_maze에서 처음 깔 문제 = dice/<이름>.json (빈 값이면 안 깜)')
    parser.add_argument('--pucks', type=int, default=0, help='시작할 때 무작위로 깔 퍽 수 (기본 0 = 빈 맵 — 테스트·CI는 6)')
    parser.add_argument('--headless', action='store_true',
                        help='창 없이 플랜트·카메라 렌더만 계속 (호스트에서 — test-faker·CI용, Ctrl+C/TERM으로 끝)')
    args = parser.parse_args()
    spawn = tuple(float(v) for v in args.spawn.split(',')) if args.spawn else None
    sim = Sim(maze.load(args.course), spawn)
    if 'mission' in sim.course and args.dice:
        sim.load_dice(args.dice)
    if args.pucks:
        sim.randomize(args.pucks)
    view = MapView(sim)
    bgs = scene.backgrounds(BG_DIR)
    sim.renderer.set_background(bgs[0] if bgs else None)
    fls = [p for p in scene.backgrounds(FLOOR_DIR) if p.name != 'FLOOR_CREDITS.txt']
    sim.renderer.set_floor(fls[0] if fls else None)
    wls = [p for p in scene.backgrounds(WALL_DIR) if p.suffix == '.jpg']
    sim.renderer.set_wall(wls[0] if wls else None)
    if args.snapshot:
        snapshot(sim, view, args.snapshot)
        return
    if args.headless:  # 창 없이: 렌더 루프를 이 스레드에서 (TERM → 송출 파일 정리 후 종료)
        import signal
        signal.signal(signal.SIGTERM, lambda *_: setattr(sim, 'running', False))
        print('SIL 시뮬레이터 headless — 카메라 프레임 →', LIVE_DIR, flush=True)
        try:
            sim.render_loop()
        except KeyboardInterrupt:
            pass
        finally:
            sim.running = False
            sim.stop_streaming()
        return
    threading.Thread(target=sim.render_loop, daemon=True).start()
    root = tk.Tk()
    App(root, sim, view)
    try:
        root.mainloop()
    finally:
        sim.running = False
        sim.stop_streaming()  # 창을 닫으면 송출도 끝 (fake_camera는 회색으로)


if __name__ == '__main__':
    main()
