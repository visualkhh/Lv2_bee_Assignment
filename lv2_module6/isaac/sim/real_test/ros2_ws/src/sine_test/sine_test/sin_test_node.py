"""Sine response test of one joint, settings from the yaml (config/sine.yaml, `joints.<name>`).

  ros2 run sine_test sin_test joint3                                   # hw.launch.py must be running
  ros2 run sine_test sin_test joint3 --ros-args -p config:=/path/my.yaml -p out_dir:=/path/logs
  q (or Ctrl+C): the goal is cancelled, the arm stops where it is, the log so far is saved

Run set_center first: the other joints hold the JTC's current command, and a warning is printed if they are
not at center. The tested joint moves slowly to its sine center (0 deg + offset; gripper: range middle + offset),
runs the sine with fade-in/out (one FollowJointTrajectory goal, JTC replays it at 100 Hz), settles at the sine
center and stays there. /joint_states (position, velocity, effort = servo current [A]) is logged next to the
analytic command.

Output: <out_dir>/<YYYYmmdd_HHMMSS>_<joint>/log.csv, meta.yaml, summary.yaml (Peak / RMS like Isaac Gain Tuner).
"""
import argparse
import csv
import math
import sys
import time
from functools import partial
from pathlib import Path

import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.utilities import remove_ros_args
from trajectory_msgs.msg import JointTrajectoryPoint

from sine_test.common import (ARM, CONTROLLER, DEFAULT_OUT_DIR, JOINTS, LINEAR, ArmNode, Quit, SineTest, center, dur,
                              move_time, run_main)

ACTION = CONTROLLER + '/follow_joint_trajectory'
START_DELAY = 0.3          # s between sending a goal and its header stamp, so the start time is known exactly
CENTER_WARN = math.radians(2.0)  # warn when another arm joint is this far from center


class SinTest(ArmNode):
    def __init__(self, joint):
        super().__init__('sin_test')
        self.joint = joint
        self.out_root = Path(self.declare_parameter('out_dir', DEFAULT_OUT_DIR).value)
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)
        self.goal_future = None    # send_goal future while waiting for acceptance
        self.goal_handle = None    # accepted goal while it runs
        self.recording = None      # (SineTest, t0) while the sine is logged
        self.rows = []
        self.on_state = self.log_row

    def log_row(self, msg):
        if not self.recording: return
        sine, t0 = self.recording
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - t0
        if t < 0: return
        cp, cv = sine(t)
        p, v, e = self.state[self.joint]
        self.rows.append([f'{t:.4f}', self.joint, f'{cp:.6f}', f'{cv:.6f}', f'{p:.6f}', f'{v:.6f}', f'{e:.4f}'])

    def send(self, points, t0):
        """points: list of (t, {joint: (pos, vel)}) relative to t0 (ROS time, s). Blocks until the goal ends."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        sec = int(t0)
        goal.trajectory.header.stamp.sec, goal.trajectory.header.stamp.nanosec = sec, int((t0 - sec) * 1e9)
        for t, q in points:
            goal.trajectory.points.append(JointTrajectoryPoint(
                positions=[q[j][0] for j in JOINTS], velocities=[q[j][1] for j in JOINTS], time_from_start=dur(t)))
        self.goal_future = self.client.send_goal_async(goal)
        self.spin_until(self.goal_future.done, 5.0, 'goal acceptance')
        self.goal_handle, self.goal_future = self.goal_future.result(), None
        if not self.goal_handle.accepted: raise RuntimeError('JTC rejected the trajectory')
        res = self.goal_handle.get_result_async()
        self.spin_until(res.done, points[-1][0] + START_DELAY + 10.0, 'trajectory end')
        self.goal_handle = None
        r = res.result().result
        if r.error_code != 0: self.get_logger().warning(f'JTC result {r.error_code}: {r.error_string}')

    def cancel(self):
        """Stop where the arm is: cancel the running goal (also one that was still waiting for acceptance)."""
        end = time.monotonic() + 2.0
        if self.goal_future is not None:
            while not self.goal_future.done() and time.monotonic() < end: rclpy.spin_once(self, timeout_sec=0.02)
            if self.goal_future.done() and self.goal_future.result().accepted: self.goal_handle = self.goal_future.result()
        if self.goal_handle is None: return
        fut = self.goal_handle.cancel_goal_async()
        while not fut.done() and time.monotonic() < end: rclpy.spin_once(self, timeout_sec=0.02)
        self.get_logger().warning('goal cancelled — the arm stops where it is')

    def plan(self, spec):
        """Build the SineTest in SI units and check limits / speed. Returns (sine, problem or None)."""
        j, lin = self.joint, self.joint in LINEAR
        unit = 1.0 if lin else math.pi / 180
        lo, hi, vmax = self.lim[j]
        c = center(j, spec, self.lim[j])
        amp = spec['amplitude_pct'] / 100 * (hi - lo) / 2 if 'amplitude_pct' in spec else spec['amplitude'] * unit
        sine = SineTest(c, amp, spec['period'], spec['cycles'], self.cfg['ramp_cycles'])
        m = self.cfg['limit_margin']['m'] if lin else math.radians(self.cfg['limit_margin']['deg'])
        if c - amp < lo + m or c + amp > hi - m:
            return sine, f'swing [{c - amp:.4f}, {c + amp:.4f}] outside limits [{lo + m:.4f}, {hi - m:.4f}] (margin incl.)'
        peak_v = max(abs(sine(i * 0.001)[1]) for i in range(int(sine.total * 1000)))
        if peak_v > self.cfg['speed_margin'] * vmax:
            return sine, f'peak speed {peak_v:.3f} > {self.cfg["speed_margin"]} x limit {vmax}'
        return sine, None

    def metrics(self, t_end):
        err = [float(r[4]) - float(r[2]) for r in self.rows if 0 <= float(r[0]) <= t_end]
        if not err: return None
        k = 1000.0 if self.joint in LINEAR else 180 / math.pi
        peak, rms = max(abs(e) for e in err) * k, math.sqrt(sum(e * e for e in err) / len(err)) * k
        cur = [abs(float(r[6])) for r in self.rows if not math.isnan(float(r[6]))]
        return {'peak_error': round(peak, 4), 'rms_error': round(rms, 4), 'unit': 'mm' if self.joint in LINEAR else 'deg',
                'samples': len(err), 'max_abs_current_A': round(max(cur), 4) if cur else None}

    def run(self):
        j = self.joint
        joints_cfg = self.cfg.get('joints') or {}
        if j not in joints_cfg or j not in JOINTS:
            where = f'{self.cfg_path} joints' if j not in joints_cfg else f'{CONTROLLER} joints {JOINTS}'
            self.get_logger().error(f"no such joint '{j}' in {where} — available: {[n for n in joints_cfg if n in JOINTS]}")
            return 1
        spec = joints_cfg[j]
        pose = self.wait_ready()
        sine, problem = self.plan(spec)
        if problem:
            self.get_logger().error(f'{j}: not run — {problem}')
            return 1
        far = [n for n in ARM if n != j and abs(pose[n]) > CENTER_WARN]
        if far: self.get_logger().warning(f'not at center: {", ".join(f"{n} {self.fmt(n, pose[n])}" for n in far)} — run set_center first?')
        self.spin_until(lambda: self.client.wait_for_server(timeout_sec=0.05), 10.0, ACTION)

        out = self.out_root / f'{time.strftime("%Y%m%d_%H%M%S")}_{j}'
        out.mkdir(parents=True, exist_ok=True)
        meta = {'joint': j, 'spec': spec, 'config': self.cfg, 'config_path': self.cfg_path, 'hold_pose': pose,
                'sine': {'center': sine.c, 'amplitude': sine.a, 'total_s': sine.total},
                'limits': {n: list(v) for n, v in self.lim.items()},
                'columns': 't [s since sine start], joint, cmd_pos, cmd_vel, pos, vel (rad, rad/s | m, m/s), effort [A]'}
        (out / 'meta.yaml').write_text(yaml.safe_dump(meta, sort_keys=False))
        self.get_logger().info(f'{j}: center {self.fmt(j, sine.c)}, amplitude {self.fmt(j, sine.a)}, period {spec["period"]} s, '
                               f'{spec["cycles"]} cycles (+fade), {sine.total:.1f} s (q: stop)')
        hold = lambda over: {n: (pose[n], 0.0) for n in JOINTS} | over
        summary, interrupted = None, False
        try:
            if abs(sine.c - pose[j]) > 1e-4:  # slow move to the sine center
                t = move_time(self.cfg, self.lim[j], sine.c - pose[j])
                self.get_logger().info(f'{j}: {self.fmt(j, pose[j])} -> {self.fmt(j, sine.c)} in {t:.1f} s')
                self.send([(t, hold({j: (sine.c, 0.0)}))], self.now() + START_DELAY)
            dt = 1.0 / self.cfg['sample_hz']
            pts = [(t, hold({j: sine(t)})) for t in (min(i * dt, sine.total) for i in range(1, math.ceil(sine.total / dt) + 1))]
            pts.append((sine.total + self.cfg['settle_s'], hold({j: (sine.c, 0.0)})))  # settle at the center and stay
            t0 = self.now() + START_DELAY
            self.recording = (sine, t0)
            try:
                self.send(pts, t0)
            finally:
                self.recording = None
            self.get_logger().info(f'{j}: done — holding at center {self.fmt(j, sine.c)}')
        except (Quit, KeyboardInterrupt):
            interrupted = True
            self.stopping()
            self.cancel()
        finally:
            with open(out / 'log.csv', 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t', 'joint', 'cmd_pos', 'cmd_vel', 'pos', 'vel', 'effort_A'])
                w.writerows(self.rows)
            summary = self.metrics(sine.total)
            (out / 'summary.yaml').write_text(yaml.safe_dump({j: summary, 'interrupted': interrupted}, sort_keys=False))
            if summary:
                self.get_logger().info(f'{j}: peak {summary["peak_error"]:.3f} {summary["unit"]}  rms {summary["rms_error"]:.3f} '
                                       f'{summary["unit"]}  max |I| {summary["max_abs_current_A"]} A' + ('  (interrupted)' if interrupted else ''))
            self.get_logger().info(f'saved {out}')
        return 0


def main():
    parser = argparse.ArgumentParser(description='Sine response test of one joint (settings: yaml joints.<joint_name>)')
    parser.add_argument('joint_name')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    run_main(partial(SinTest, args.joint_name))


if __name__ == '__main__':
    main()
