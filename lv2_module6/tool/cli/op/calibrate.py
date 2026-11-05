"""run-cli calibrate — 칼리브레이션 계획을 /cmd/manual/cmd_vel로 재생하며 응답을 raw.csv로 기록한다.

계획(JSON)은 PC의 isaac calibration이 만든다 (sim과 같은 명령을 그대로 재생해야 비교가 된다):
  {"dt": 0.05, "label": "real", "steps": [[v, w], ...]}   # 스텝마다 (m/s, rad/s) — dt 간격으로 한 줄씩 보냄 · label = CSV label 열
기록 = isaac sim의 raw.csv와 같은 열 (t · lin_raw · ang_raw · lin_out · ang_out · odom_* · v_enc · w_enc · w_gyro · wheel_* · t_*):
  /odom · /imu · /joint_states (wheel_left_joint · wheel_right_joint) · /cmd_vel (smoother · collision_monitor까지 거쳐 실제로 나간 명령)
명령은 /cmd/manual/cmd_vel → manual_control(MANUAL) → /cmd_vel_manual — twist_mux 수동 경로 (순수 응답).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState

MAX_LIN, MAX_ANG = 0.26, 1.82   # bus.py와 같은 한계 — 계획이 넘으면 거부
MAX_SECONDS = 180.0
COLUMNS = ['t', 'lin_raw', 'ang_raw', 'lin_out', 'ang_out', 'odom_x', 'odom_y', 'odom_yaw', 'v_enc', 'w_enc', 'w_gyro',
           'wheel_l_vel', 'wheel_r_vel', 'wheel_l_pos', 'wheel_r_pos', 't_ros', 't_odom', 't_imu', 't_joint', 'label']


def load_plan(path: str) -> dict:
    """계획 파일 읽기 + 검사. 잘못되면 ValueError (사유 문자열)."""
    try:
        plan = json.loads(Path(path).read_text())
        dt, steps = float(plan['dt']), plan['steps']
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise ValueError(f'계획 파일을 못 읽음 ({path}): {exc!r}') from exc
    if not 0.02 <= dt <= 0.2:
        raise ValueError(f'dt {dt}s — 0.02~0.2 s여야 함')
    if not steps or len(steps) * dt > MAX_SECONDS:
        raise ValueError(f'스텝 {len(steps)}개 × {dt}s — 1~{MAX_SECONDS:.0f} s여야 함')
    for i, s in enumerate(steps):
        v, w = float(s[0]), float(s[1])
        if not (math.isfinite(v) and math.isfinite(w)) or abs(v) > MAX_LIN or abs(w) > MAX_ANG:
            raise ValueError(f'스텝 {i}: v={v} w={w} — 한계 {MAX_LIN} m/s · {MAX_ANG} rad/s 밖')
    plan['dt'], plan['label'] = dt, str(plan.get('label', 'real'))
    return plan


def _sec(ts) -> float:
    if ts is None:
        return float('nan')
    if hasattr(ts, 'nanoseconds'):  # rclpy Time
        return ts.nanoseconds / 1e9
    return float(ts.sec) + float(ts.nanosec) * 1e-9


class Recorder:
    """/odom · /imu · /joint_states · /cmd_vel의 최신 값 — spin하면서 쌓이고, row()가 한 줄로 만든다."""

    def __init__(self, node):
        self.node = node
        self.odom = self.imu = self.joint = None
        self.out = (0.0, 0.0)
        self.wheel = {'wheel_left_joint': (0.0, 0.0), 'wheel_right_joint': (0.0, 0.0)}   # (pos, vel)
        self.subs = [
            node.create_subscription(Odometry, '/odom', self._odom, qos_profile_sensor_data),
            node.create_subscription(Imu, '/imu', self._imu, qos_profile_sensor_data),
            node.create_subscription(JointState, '/joint_states', self._joint, qos_profile_sensor_data),
            node.create_subscription(TwistStamped, '/cmd_vel', self._out, 10),
        ]

    def _odom(self, m):
        self.odom = m

    def _imu(self, m):
        self.imu = m

    def _out(self, m):
        self.out = (m.twist.linear.x, m.twist.angular.z)

    def _joint(self, m):
        self.joint = m
        for name, pos, vel in zip(m.name, m.position, m.velocity):
            if name in self.wheel:
                self.wheel[name] = (pos, vel)

    def missing(self) -> list[str]:
        return [n for n, m in (('/odom', self.odom), ('/imu', self.imu), ('/joint_states', self.joint)) if m is None]

    def row(self, t: float, v: float, w: float, label: str) -> dict:
        o = self.odom
        p, q = o.pose.pose.position, o.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        (pl, wl), (pr, wr) = self.wheel['wheel_left_joint'], self.wheel['wheel_right_joint']
        return {'t': round(t, 3), 'lin_raw': v, 'ang_raw': w, 'lin_out': self.out[0], 'ang_out': self.out[1],
                'odom_x': p.x, 'odom_y': p.y, 'odom_yaw': yaw, 'v_enc': o.twist.twist.linear.x,
                'w_enc': o.twist.twist.angular.z, 'w_gyro': self.imu.angular_velocity.z,
                'wheel_l_vel': wl, 'wheel_r_vel': wr, 'wheel_l_pos': pl, 'wheel_r_pos': pr,
                't_ros': _sec(self.node.get_clock().now()), 't_odom': _sec(o.header.stamp),
                't_imu': _sec(self.imu.header.stamp), 't_joint': _sec(self.joint.header.stamp), 'label': label}

    def close(self):
        for s in self.subs:
            self.node.destroy_subscription(s)


STOW_TOL = 0.05   # [rad] skill_pose initial_pose_tolerance와 같게 — 이 안이면 이미 시작 자세
STOW_WAIT_S = 10.0


def ensure_stow(bus, move: bool = True) -> bool:
    """팔이 시작 자세(skill_pose initial_pose = stow)인지 — 아니면 move일 때 `arm initial`로 보내고 도달을 기다림.
    주행 칼리브레이션은 팔이 접혀 있어야 무게중심이 sim(기본 자세)과 같다. True = 시작 자세. move=False는 보기만."""
    pose = bus.initial_pose()
    if pose is None:
        return False
    targets = pose[0]
    bus._wait_joints()

    def err() -> float:
        return max(abs(bus.state.joints.get(n, math.inf) - v) for n, v in targets.items())

    if err() <= STOW_TOL:
        print('팔: 시작 자세')
        return True
    if not move:
        print(f'팔: 시작 자세가 아님 (최대 오차 {err():.2f} rad) — 재생할 때 시작 자세로 먼저 이동')
        return False
    print(f'팔이 시작 자세가 아님 (최대 오차 {err():.2f} rad) → 시작 자세로 이동 (충돌 회피 없음)')
    if bus.cmd_arm(argparse.Namespace(action='initial', targets=[], force=False)) != 0:
        return False
    end = time.monotonic() + STOW_WAIT_S
    while time.monotonic() < end and rclpy.ok():
        bus._poll(0.2)
        if err() <= STOW_TOL:
            print('팔: 시작 자세 도달')
            return True
    print(f'거부: 팔이 시작 자세에 못 감 (오차 {err():.2f} rad) — 걸렸는지 확인 (arm reboot · TEACH)', file=sys.stderr)
    return False


def run(bus, plan: dict, out: Path, check_only: bool = False) -> int:
    """계획 재생 + 기록. 끝나면(예외 · Ctrl+C · 비상정지 · 모드 이탈 포함) 반드시 정지 명령. 0 = 성공."""
    rec = Recorder(bus.node)
    steps, dt, label = plan['steps'], plan['dt'], plan['label']
    try:
        end = time.monotonic() + 3.0
        while rec.missing() and time.monotonic() < end and rclpy.ok():
            bus._poll(0.1)
        if rec.missing():
            print(f'거부: {", ".join(rec.missing())} 안 옴 — bringup · 컨트롤러 확인', file=sys.stderr)
            return 2
        if check_only:
            print(f'check-only — 준비됨 (조작 {bus.state.operation} · E-STOP {"ON" if bus.state.emergency else "OFF"} · '
                  f'계획 {len(steps)}스텝 {len(steps) * dt:.1f}s)')
            return 0
        rows, why = [], None
        print(f'칼리브레이션 재생: {len(steps)}스텝 · {len(steps) * dt:.1f}s · {label} → {out}')
        t0 = time.monotonic()
        try:
            for k, (v, w, *_) in enumerate(steps):
                if bus.state.emergency:
                    why = '비상정지'
                elif bus.state.operation != 'MANUAL':
                    why = f'조작 모드가 {bus.state.operation}로 바뀜'
                if why:
                    break
                bus._send_velocity(v, w)
                nxt = t0 + (k + 1) * dt
                while time.monotonic() < nxt:   # 다음 스텝까지 메시지를 받으며 기다림
                    rclpy.spin_once(bus.node, timeout_sec=min(0.01, max(0.0, nxt - time.monotonic())))
                rows.append(rec.row(k * dt, v, w, label))
        except KeyboardInterrupt:
            why = '중단 (Ctrl+C)'
        finally:
            for _ in range(5):   # 어떤 경우든 정지
                bus._send_velocity(0.0, 0.0)
                bus._poll(0.05)
        if why:
            print(f'중단: {why} — {len(rows)}/{len(steps)}스텝까지 (저장 안 함)', file=sys.stderr)
            return 1
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open('w', newline='') as f:
            cw = csv.DictWriter(f, fieldnames=COLUMNS)
            cw.writeheader()
            cw.writerows(rows)
        print(f'{len(rows)}줄 → {out}')
        return 0
    finally:
        rec.close()


if __name__ == '__main__':   # 자체 확인 (ROS 없이 계획 검사만): python3 -m op.calibrate  (tool/cli에서)
    import tempfile

    def check(obj):
        with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as f:
            json.dump(obj, f)
        return load_plan(f.name)

    assert check({'dt': 0.05, 'steps': [[0.1, 0.0]]})['label'] == 'real'
    for bad in ({'dt': 0.05, 'steps': [[0.5, 0.0]]}, {'dt': 0.5, 'steps': [[0.1, 0.0]]},
                {'dt': 0.05, 'steps': []}, {'dt': 0.05, 'steps': [[0.1, 9.0]]}, {'steps': [[0, 0]]}):
        try:
            check(bad)
        except ValueError:
            continue
        raise AssertionError(f'통과하면 안 됨: {bad}')
    print('ok — 계획 검사')
