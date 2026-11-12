"""fake 모드의 움직이는 세계 — 더미 센서 · 로봇 이동 · 미션 · coordinator를 한 프로세스 안에서 흉내 낸다.

FakeRos._spin이 step()을 불러 약 20 Hz로 굴린다 (스레드 없음). 입력은 FakeRos.\\_fake_on_publish가 on_publish로 넘긴다.
  입력   /cmd/manual/cmd_vel(바퀴) · /cmd/manual/arm(팔) · /cmd/mission(start|abort) · /task/sub_goals(skill 계획)
         + 지도 화면의 goal 찍기(send_nav_goal) · initialpose(set_robot_pose) — attach()가 bus.nav에 끼워 넣는다
  출력   /joint_states · /odom · /imu · 카메라 · /tracked_objects · /motor_status · /diagnostics_agg(생존 표시)
         /mission/state · /task/status · /plan · /local_plan · /global_costmap/costmap · /local_costmap/costmap
실세계 = RealWorld(fake/real/map/<코스> — Isaac 미로 USD 등): 로봇이 부딪히는 벽 · 앞 카메라가 보는 벽.
로봇 지도 = /map (실기처럼): NONE이면 없음 · MAPPING이면 빈 지도에서 앞 카메라 시야로 채움 · LOCALIZATION이면 저장 지도.
  경로 계획(가짜 Nav2)·costmap은 로봇 지도로 (모르는 곳은 지나갈 수 있다고 봄) — 실세계와 다르면 가다가 막힌다 (실기처럼).
진짜와 다른 점(알아 둘 것): 경로는 지도 격자 위 최단 경로 · 위치추정 오차 없음 · 미션·skill은 시간표대로 성공 · 값은 전부 더미.
"""
from __future__ import annotations

import heapq
import math
import time

from .. import msgs
from ..msgs import (DiagnosticArray, DiagnosticStatus, JointState, KeyValue, MissionState, MotorStatus,
                    OccupancyGrid, Path, PoseStamped, TaskStatus, TrackedObjectArray, _Msg)

DT = 0.05
RES = 0.05                      # 가짜 세계 격자 [m] (Nav2 costmap과 같게)
SCAN_FOV, SCAN_RANGE = 87.0, 3.0   # 앞 카메라(pole_front_cam) 시야 [°] · 깊이 최대 [m] — 매핑이 이만큼 지도를 채움
WHEEL_R, WHEEL_L = 0.033, 0.287
ARM = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_left_joint']
POSES = {'stow': [0.0, -1.0, 1.3, 0.7, 0.0], 'carry_left': [1.57, -0.5, 0.8, 0.4, 0.0],
         'carry_right': [-1.57, -0.5, 0.8, 0.4, 0.0],
         'look': [0.0, 0.3, 0.3, 0.9, 0.0]}
SKILL_S = {'inspect': 2.0, 'pose': 1.5, 'pick': 3.0, 'place': 3.0, 'load': 2.5, 'unload': 2.5, 'stand': 3.0,
           'stand_pick': 2.5, 'approach': 2.0}
# 미션 시간표 — (phase, 초, 설명)
MISSION = [('GO_OBSERVE', 3, '관측 자세로 이동'), ('BARCODE', 2, '바코드 읽는 중'), ('READ', 3, '신호 12회 점멸 읽는 중'),
           ('SURVEY', 6, '시작 박스 6곳 조사'), ('BUILD', 0, '퍽 놓는 중'), ('VERIFY', 3, '장소별 확인'),
           ('RETURN', 3, '출발 구역으로 복귀')]


class FakeWorld:
    def __init__(self, node, real):
        self.node, self.real = node, real
        w, h, ox, oy, occ = real.occupancy(RES)
        self.w, self.h, self.res, self.ox, self.oy = w, h, RES, ox, oy
        self.occ = [occ[r][c] for r in range(h) for c in range(w)]   # 실세계 벽 (0행 = 아래)
        self.cost = self._inflate(self.occ)                          # 실세계 — 몸이 벽에 닿는지 (충돌)
        self.known = [-1] * (w * h)                                  # 로봇 지도 (/map) — -1 모름 · 0 빈 곳 · 100 벽
        self.kcost = self._inflate([False] * (w * h))                # 로봇 지도 costmap — 경로 계획 · costmap 토픽
        self.map_on = self.mapping = False                           # /map을 내는 중 · 매핑 중 (NONE이면 둘 다 꺼짐)
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0                     # 켜면 출발 자리 = map 원점, 앞 = +x (실기처럼)
        self.odom = (0.05, -0.03, 2.0)             # map→odom (고정) — 지도 화면의 O 표시용
        self.vx = self.wz = 0.0
        self.vel_at = 0.0
        self.joints = dict(zip(ARM, POSES['stow'] + [0.026]))
        self.targets = dict(self.joints)
        self.wheel = [0.0, 0.0]
        self.batt = 12.4
        self.t0 = self.last = time.monotonic()
        self.slow = {}                              # 주기가 긴 출력의 마지막 시각
        self.path: list[tuple[float, float]] = []   # 지금 따라가는 경로
        self.nav_done = None                        # 경로 끝나면 부를 함수(ok)
        self.nav_active = False                     # 이동 중(경로 따라가기 → 목표 방향 맞추기)
        self.goal_yaw = None                        # 도착 뒤 제자리에서 맞출 방향 [rad] (None이면 안 맞춤)
        self.wp_queue: list[dict] = []               # 지도 화면에서 찍은 웨이포인트 — 앞 goal 성공 뒤 다음을 보낸다
        self.wp_i = 0
        self.wp_state = 'idle'                      # idle | running | done | failed | canceled
        self.wp_error = ''
        self.mission = None                         # {'i', 'since', 'placed', ...}
        self.m = MissionState()
        self.plan = None                            # coordinator 계획 {'id','goals','i','since'}
        self.task = TaskStatus()

    # ------------------------------------------------------------------ 로봇 지도 (/map) — 지도 모드에 따라
    def start_mapping(self):
        """MAPPING — 빈 지도에서 시작해 앞 카메라 시야로 채운다 (fake_nvblox·nvblox 흉내)."""
        self._cancel_nav()
        self.known = [-1] * (self.w * self.h)
        self.map_on = self.mapping = True
        self._scan()
        self._pub_map()

    def set_known(self, grid):
        """LOCALIZATION — 저장 지도(OccupancyGrid)를 로봇 지도로 (실세계 격자에 맞춰 옮겨 담음)."""
        self._cancel_nav()
        gw, gh, gres = int(grid.info.width), int(grid.info.height), float(grid.info.resolution)
        gx, gy = float(grid.info.origin.position.x), float(grid.info.origin.position.y)
        known = []
        for r in range(self.h):
            for c in range(self.w):
                x, y = self._world(c, r)
                gc, gr = int(math.floor((x - gx) / gres)), int(math.floor((y - gy) / gres))
                known.append(grid.data[gr * gw + gc] if 0 <= gc < gw and 0 <= gr < gh else -1)
        self.known = known
        self.map_on, self.mapping = True, False
        self._pub_map()

    def clear_map(self):
        """NONE — 지도 없음 (/map·costmap 안 냄, 경로 계획 불가)."""
        self._cancel_nav()
        self.map_on = self.mapping = False

    def _scan(self):
        """앞 카메라 시야(±SCAN_FOV/2 · SCAN_RANGE)로 실세계를 비춰 로봇 지도를 채움 — 벽까지 빈 곳, 닿은 칸은 벽."""
        step = self.res / 2
        for deg in range(-int(SCAN_FOV // 2), int(SCAN_FOV // 2) + 1, 2):
            a = self.yaw + math.radians(deg)
            ca, sa = math.cos(a), math.sin(a)
            d = 0.0
            while d <= SCAN_RANGE:
                k = self._cell(self.x + d * ca, self.y + d * sa)
                if k is None:
                    break
                i = k[1] * self.w + k[0]
                if self.occ[i]:
                    self.known[i] = 100
                    break
                if self.known[i] != 100:
                    self.known[i] = 0
                d += step

    def _pub_map(self):
        if not self.map_on:
            return
        self.kcost = self._inflate([v >= 50 for v in self.known])
        m = OccupancyGrid()
        m.info = _Msg(width=self.w, height=self.h, resolution=self.res,
                      origin=_Msg(position=_Msg(x=self.ox, y=self.oy, z=0.0)))
        m.data = list(self.known)
        self.node._deliver('/map', m)

    # ------------------------------------------------------------------ 지도
    def _cell(self, x, y):
        c, r = int(math.floor((x - self.ox) / self.res)), int(math.floor((y - self.oy) / self.res))
        return (c, r) if 0 <= c < self.w and 0 <= r < self.h else None

    def _world(self, c, r):
        return self.ox + (c + 0.5) * self.res, self.oy + (r + 0.5) * self.res

    def _inflate(self, occ):
        """벽 0칸=100 · 2칸 안=99(내접) · 4칸 안=50(팽창) — Nav2 costmap 흉내."""
        w, h, out = self.w, self.h, [0] * (self.w * self.h)
        for i, o in enumerate(occ):
            if not o:
                continue
            c0, r0 = i % w, i // w
            for dr in range(-4, 5):
                for dc in range(-4, 5):
                    c, r = c0 + dc, r0 + dr
                    if 0 <= c < w and 0 <= r < h:
                        d = max(abs(dc), abs(dr))
                        out[r * w + c] = max(out[r * w + c], 100 if d == 0 else 99 if d <= 2 else 50)
        return out

    def blocked(self, x, y):
        k = self._cell(x, y)
        return k is None or self.cost[k[1] * self.w + k[0]] >= 99

    def find_path(self, gx, gy):
        """로봇 → (gx, gy) 격자 최단 경로(8방향) — 못 가면 None."""
        s, e = self._cell(self.x, self.y), self._cell(gx, gy)
        if s is None or e is None or self.kcost[e[1] * self.w + e[0]] >= 99:   # 로봇 지도로 계획 (모르는 곳 = 지나감)
            return None
        dist, prev, heap = {s: 0.0}, {}, [(0.0, s)]
        while heap:
            d, cur = heapq.heappop(heap)
            if cur == e:
                break
            if d > dist.get(cur, 1e9):
                continue
            for dc in (-1, 0, 1):
                for dr in (-1, 0, 1):
                    nb = (cur[0] + dc, cur[1] + dr)
                    if (dc or dr) and 0 <= nb[0] < self.w and 0 <= nb[1] < self.h \
                            and self.kcost[nb[1] * self.w + nb[0]] < 99:
                        nd = d + (1.414 if dc and dr else 1.0)
                        if nd < dist.get(nb, 1e9):
                            dist[nb], prev[nb] = nd, cur
                            heapq.heappush(heap, (nd, nb))
        if e not in prev and e != s:
            return None
        cells, cur = [e], e
        while cur != s:
            cur = prev[cur]
            cells.append(cur)
        return [self._world(*c) for c in reversed(cells)]

    # ------------------------------------------------------------------ 입력
    def on_publish(self, topic, msg):
        st = self.node.state
        if topic == '/cmd/manual/cmd_vel':
            if st.operation == 'MANUAL' and not st.emergency:
                self.vx, self.wz, self.vel_at = msg.twist.linear.x, msg.twist.angular.z, time.monotonic()
        elif topic == '/cmd/manual/arm':
            if st.operation in ('MANUAL', 'TEACH') and not st.emergency:
                for n, p in zip(msg.name, msg.position):
                    if n in self.targets:
                        self.targets[n] = float(p)
        elif topic == '/cmd/mission' and isinstance(getattr(msg, 'data', None), str):
            self._mission_cmd(msg.data)
        elif topic == '/task/sub_goals':
            self._on_goals(msg)
        elif topic == '/cmd/e_stop' and getattr(msg, 'data', False):
            self.vx = self.wz = 0.0
            self._cancel_nav()

    def send_nav_goal(self, pose, frame_id='map'):
        """지도 화면 goal — Nav2 NavigateToPose 흉내."""
        self._send_nav_goal_now(pose)

    def _send_nav_goal_now(self, pose):
        if self.node.state.operation != 'AUTO':
            raise RuntimeError('AUTO가 아니라 거부 (가짜 Nav2)')
        if not self.map_on:
            raise RuntimeError('지도 없음 (지도 모드 NONE) — MAPPING 또는 LOCALIZATION에서 (가짜 Nav2)')
        path = self.find_path(float(pose['x']), float(pose['y']))
        if path is None:
            raise RuntimeError('경로를 못 찾음 — 벽·팽창 영역이거나 지도 밖 (가짜 Nav2)')
        self.path, self.nav_done, self.nav_active = path, None, True
        yaw = pose.get('yaw_deg')
        self.goal_yaw = None if yaw is None else math.radians(float(yaw))   # 도착하면 이 방향으로 제자리 회전

    def send_nav_waypoints(self, poses, frame_id='map'):
        """웨이포인트를 순서대로 — 앞 goal의 nav_done에서 다음 goal을 보낸다 (가짜 Nav2)."""
        poses = [dict(p) for p in poses]
        if not poses:
            raise RuntimeError('웨이포인트 없음')
        if self.wp_state == 'running':
            raise RuntimeError('이미 주행 중 — 먼저 취소(x)')
        self.wp_queue, self.wp_i = poses, 0
        self.wp_state, self.wp_error = 'running', ''
        self._wp_send_current()
        if self.wp_state == 'failed':
            raise RuntimeError(self.wp_error)

    def _wp_send_current(self):
        if self.wp_i >= len(self.wp_queue):
            self.wp_state = 'done'
            return
        try:
            self._send_nav_goal_now(self.wp_queue[self.wp_i])
        except Exception as exc:
            self.wp_state, self.wp_error = 'failed', str(exc)
            return
        self.nav_done = self._wp_done

    def _wp_done(self, ok):
        if self.wp_state != 'running':
            return
        if not ok:
            self.wp_state = 'failed'
            self.wp_error = '주행 중단'
            return
        self.wp_i += 1
        if self.wp_i >= len(self.wp_queue):
            self.wp_state = 'done'
            return
        self._wp_send_current()

    def nav_progress(self):
        """지도 화면 상태줄 — 웨이포인트 진행 (가짜 Nav2는 feedback이 없어 회복/남은 거리를 안 낸다)."""
        if not self.wp_queue or self.wp_state == 'idle':
            return None
        total = len(self.wp_queue)
        index = total if self.wp_state == 'done' else min(self.wp_i + 1, total)
        dist = None
        if self.wp_state == 'running':
            g = self.wp_queue[self.wp_i]
            dist = math.hypot(float(g['x']) - self.x, float(g['y']) - self.y)
        return {'state': self.wp_state, 'index': index, 'total': total,
                'recoveries': None, 'distance_remaining': dist, 'age': None, 'error': self.wp_error}

    def cancel_nav(self):
        if self.wp_state != 'running':
            return False
        self.wp_state = 'canceled'
        self._cancel_nav()
        return True

    def set_robot_pose(self, x, y, yaw_deg, frame_id='map'):
        self.x, self.y, self.yaw = float(x), float(y), math.radians(float(yaw_deg))
        self._cancel_nav()

    def robot_pose(self, frame_id='map'):
        return {'x': round(self.x, 3), 'y': round(self.y, 3), 'yaw_deg': round(math.degrees(self.yaw), 1)}

    def frame_pose(self, child, frame_id='map'):
        """RobotNav.frame_pose와 같은 계약 — (x, y, yaw[deg]). self.odom의 yaw는 rad."""
        if child != 'odom':
            return None
        x, y, yaw = self.odom
        return x, y, math.degrees(yaw)

    def _cancel_nav(self):
        self.path, done, self.nav_done = [], self.nav_done, None
        self.nav_active, self.goal_yaw = False, None
        if done:
            done(False)

    # ------------------------------------------------------------------ coordinator (skill 계획)
    def _status(self, step, state, reason=''):
        self.task = TaskStatus(plan_id=self.plan['id'] if self.plan else 0, step=step, state=state, reason=reason)
        self.node._deliver('/task/status', self.task)

    def _on_goals(self, msg):
        st = self.node.state
        if msg.plan_id == 0 and not msg.goals:   # 정지
            self._cancel_nav()
            self.plan = None
            self._status(0, 2, 'cancelled')
            return
        self.plan = {'id': msg.plan_id, 'goals': list(msg.goals), 'i': -1, 'since': 0.0}
        if st.emergency:
            self._status(0, 2, 'estop_blocked')
            self.plan = None
        elif st.operation != 'AUTO':
            self._status(0, 2, f'{st.operation} 모드')
            self.plan = None
        else:
            self._next_goal()

    def _next_goal(self):
        p = self.plan
        p['i'] += 1
        if p['i'] >= len(p['goals']):
            self._status(p['goals'][-1].step if p['goals'] else 0, 1, '')
            self.plan = None
            return
        g = p['goals'][p['i']]
        p['since'] = time.monotonic()
        self._status(g.step, 0, '')
        if g.skill in ('navigate', 'approach') and ',' in g.target:
            try:
                x, y = [float(t) for t in g.target.split(',')[:2]]
            except ValueError:
                return self._fail(g, 'nav_bad_target')
            path = self.find_path(x, y)
            if path is None:
                return self._fail(g, 'nav_failed')
            self.path, self.nav_active, self.goal_yaw = path, True, None
            self.nav_done = lambda ok, g=g: self._goal_done(g, ok, 'nav_failed')
            p['since'] = 0.0
        elif g.skill == 'pose':
            if g.target not in POSES:
                return self._fail(g, 'pose_unknown')
            self.targets.update(zip(ARM, POSES[g.target]))
        elif g.skill not in SKILL_S and g.skill != 'navigate':
            self._fail(g, 'unknown_skill')

    def _fail(self, g, why):
        self._status(g.step, 2, why)
        self.plan = None

    def _goal_done(self, g, ok, why):
        if self.plan is None:
            return
        self._next_goal() if ok else self._fail(g, why)

    def _step_plan(self, now):
        p = self.plan
        if not p or p['i'] < 0 or p['i'] >= len(p['goals']) or not p['since']:
            return
        g = p['goals'][p['i']]
        if now - p['since'] >= SKILL_S.get(g.skill, 2.0):
            self._next_goal()

    # ------------------------------------------------------------------ 미션 (mission_manager 흉내)
    def _mission_cmd(self, cmd):
        if cmd == 'abort' and self.mission:
            self.mission = None
            self.m.phase, self.m.error, self.m.note = 'ERROR', 'aborted', '중지됨'
            self._pub_mission()
        elif cmd == 'start' and not self.mission:
            self.mission = {'i': 0, 'since': time.monotonic(), 'placed': 0}
            self.m = MissionState(phase='GO_OBSERVE', note=MISSION[0][2], total=12)
            self._pub_mission()

    def _pub_mission(self):
        self.node._deliver('/mission/state', self.m)

    def _step_mission(self, now):
        ms = self.mission
        if not ms:
            return
        m, (phase, secs, note) = self.m, MISSION[ms['i']]
        m.phase, m.note = phase, note
        el = now - ms['since']
        if phase == 'READ':
            f = min(4, int(el / secs * 4) + 1)
            m.count_r = m.count_y = m.count_g = f
            m.order = 'YRG' if el >= secs - 0.2 else ''
        elif phase == 'BARCODE':
            m.barcode = 'BEE-0317' if el > 1 else ''
        elif phase == 'SURVEY':
            m.tray = [f'{i}:{i + 3}' for i in range(min(12, int(el / secs * 12) + 1))]
        elif phase == 'BUILD':
            ms['placed'] = min(12, int(el / 1.2))
            m.placed = ms['placed']
            m.note = f'퍽 {m.placed}/12 놓는 중'
            m.results = [f'puck {i + 3}: ok' for i in range(m.placed)][-6:]
            secs = 12 * 1.2
        if el >= secs and (phase != 'BUILD' or ms['placed'] >= 12):
            ms['i'] += 1
            ms['since'] = now
            if ms['i'] >= len(MISSION):
                self.mission = None
                m.phase, m.note, m.tray = 'DONE', '미션 완료', []
        self._pub_mission()

    # ------------------------------------------------------------------ 한 걸음 (20 Hz)
    def step(self):
        now = time.monotonic()
        if now - self.last < DT:
            return
        dt, self.last = min(now - self.last, 0.2), now
        self._move(now, dt)
        self._step_plan(now)
        if self.mission:
            self._step_mission(now)
        d = self.node._deliver
        d('/joint_states', self._joint_msg())
        for t in ('/odom', '/imu', '/pole_front_cam/color/camera_info', '/gripper_cam/color/camera_info'):
            d(t, _Msg())
        if self.mapping and self._every('scan', now, 0.2):   # 매핑 — 앞 카메라로 로봇 지도를 채움 (/map은 1 Hz)
            self._scan()
        if self._every('slow', now, 1.0):
            self.node.state._touch_state()   # 가짜 state_manager 생존 신호 — 없으면 15초 뒤 '연결 ✗'로 보고 CLI가 스스로 재시작함
            d('/tracked_objects', TrackedObjectArray(objects=[_Msg() for _ in range(12 if self.mission else 3)]))
            d('/diagnostics_agg', self._diag())
            d('/motor_status', MotorStatus(joints=list(self.joints), temperature=[38 + i * 2 for i in range(6)],
                                           text=['' for _ in range(6)], battery_voltage=self.batt))
            if self.map_on:   # Nav2가 도는 동안만 (지도 모드 NONE이면 없음)
                if self.mapping:
                    self._pub_map()
                d('/global_costmap/costmap', self._costmap(None))
                d('/local_costmap/costmap', self._costmap(2.0))
            self.batt = max(10.5, self.batt - 0.0005)
            if self.mission or self.task.plan_id:
                self._pub_mission()
        self._pub_plans(now)

    def _every(self, key, now, period):
        if now - self.slow.get(key, 0.0) >= period:
            self.slow[key] = now
            return True
        return False

    def _move(self, now, dt):
        st = self.node.state
        for n, tgt in self.targets.items():   # 팔 — 목표로 천천히 (최대 1.5 rad/s)
            cur = self.joints[n]
            self.joints[n] = cur + max(-1.5 * dt, min(1.5 * dt, tgt - cur))
        if st.emergency:
            self.vx = self.wz = 0.0
            return
        if self.path:   # 경로 따라가기 (0.25 m/s)
            tx, ty = self.path[0]
            while self.path and math.hypot(tx - self.x, ty - self.y) < 0.06:
                self.path.pop(0)
                if self.path:
                    tx, ty = self.path[0]
            if self.path:
                want = math.atan2(ty - self.y, tx - self.x)
                err = (want - self.yaw + math.pi) % (2 * math.pi) - math.pi
                self.wz = max(-1.5, min(1.5, 2.0 * err))
                self.vx = 0.25 if abs(err) < 0.6 else 0.0
        if self.nav_active and not self.path:   # 경로 끝 — 목표 방향이 있으면 제자리 회전으로 맞춘 뒤 도착 처리 (Nav2 goal의 yaw)
            err = 0.0 if self.goal_yaw is None else (self.goal_yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi
            if abs(err) > 0.05:
                self.vx, self.wz = 0.0, max(-1.5, min(1.5, 2.0 * err)) if abs(err) > 0.15 else (0.3 if err > 0 else -0.3)
            else:
                done, self.nav_done = self.nav_done, None
                self.nav_active, self.goal_yaw = False, None
                self.vx = self.wz = 0.0
                if done:
                    done(True)
        elif not self.path and now - self.vel_at > 0.6:   # 수동 명령이 끊기면 정지
            self.vx = self.wz = 0.0
        nx, ny = self.x + self.vx * math.cos(self.yaw) * dt, self.y + self.vx * math.sin(self.yaw) * dt
        if self.blocked(nx, ny) and not self.blocked(self.x, self.y):
            self.vx = 0.0   # 벽 — 멈춤
        else:
            self.x, self.y = nx, ny
        self.yaw = (self.yaw + self.wz * dt + math.pi) % (2 * math.pi) - math.pi
        self.wheel[0] += (self.vx - self.wz * WHEEL_L / 2) / WHEEL_R * dt
        self.wheel[1] += (self.vx + self.wz * WHEEL_L / 2) / WHEEL_R * dt

    def _joint_msg(self):
        names = list(self.joints) + ['wheel_left_joint', 'wheel_right_joint']
        pos = list(self.joints.values()) + self.wheel
        vel = [0.0] * 6 + [(self.vx - self.wz * WHEEL_L / 2) / WHEEL_R, (self.vx + self.wz * WHEEL_L / 2) / WHEEL_R]
        return JointState(name=names, position=pos, velocity=vel, effort=[0.0] * 8)

    def _diag(self):
        st = self.node.state
        return DiagnosticArray(status=[
            DiagnosticStatus(level=0, name='beebot: 상태', message='정상 (가짜)',
                             values=[KeyValue(key='모드', value=st.operation or '-')]),
            DiagnosticStatus(level=2 if st.emergency else 0, name='beebot: E-STOP',
                             message='비상정지' if st.emergency else '해제'),
            DiagnosticStatus(level=0, name='beebot: opencr', message='모터 에러 없음',
                             values=[KeyValue(key='받은 S 줄', value='48 Hz')])])

    def _costmap(self, half):
        """global = 전체 · local = 로봇 주변 half[m] 창 (같은 해상도)."""
        m = OccupancyGrid()
        if half is None:
            m.info = _Msg(width=self.w, height=self.h, resolution=self.res,
                          origin=_Msg(position=_Msg(x=self.ox, y=self.oy, z=0.0)))
            m.data = self.kcost
            return m
        n = int(2 * half / self.res)
        ox, oy = self.x - half, self.y - half
        m.info = _Msg(width=n, height=n, resolution=self.res, origin=_Msg(position=_Msg(x=ox, y=oy, z=0.0)))
        data = []
        for r in range(n):
            for c in range(n):
                k = self._cell(ox + (c + 0.5) * self.res, oy + (r + 0.5) * self.res)
                data.append(self.kcost[k[1] * self.w + k[0]] if k else 0)
        m.data = data
        return m

    def _pub_plans(self, now):
        if not self.path:
            return
        if not self._every('plan', now, 0.5):
            return
        pts = [(self.x, self.y)] + self.path
        for topic, sel in (('/plan', pts), ('/local_plan', pts[:25])):
            p = Path()
            p.header.frame_id = 'map'
            for x, y in sel:
                ps = PoseStamped()
                ps.pose.position.x, ps.pose.position.y = x, y
                p.poses.append(ps)
            self.node._deliver(topic, p)


def attach(app, world):
    """진짜 RobotNav의 TF·Nav2 부분을 가짜 월드로 바꿔 끼운다 (지도 화면 로봇 점 · goal · initialpose)."""
    ab = app.bus.nav
    ab.robot_pose, ab.frame_pose = world.robot_pose, world.frame_pose
    ab.send_nav_goal, ab.set_robot_pose = world.send_nav_goal, world.set_robot_pose
    ab.send_nav_waypoints, ab.nav_progress, ab.cancel_nav = (
        world.send_nav_waypoints, world.nav_progress, world.cancel_nav)
