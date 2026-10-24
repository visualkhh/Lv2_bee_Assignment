"""Non-sine tests of one joint, settings from the yaml (config/profile.yaml, `joints.<name>.<mode>`).

  ros2 run sine_test profile_test joint2 hold     # gravity sag + holding current at several angles
  ros2 run sine_test profile_test joint2 step     # step response: speed saturation, overshoot, settling
  ros2 run sine_test profile_test joint2 ramp     # slow triangle: friction / backlash at the reversals
  q (or Ctrl+C): the goal is cancelled, the arm stops where it is, the log so far is saved

Same flow and output as sin_test (run set_center first; slow move to the start, one FollowJointTrajectory goal,
settle at the start pose and stay), only the command profile differs. Every profile starts and ends at
center (0 deg + offset). Output: <out_dir>/<out_subdir>/<YYYYmmdd_HHMMSS>_<joint>_<mode>/log.csv, meta.yaml, summary.yaml.
"""
import argparse
import math
import sys
from functools import partial

from rclpy.utilities import remove_ros_args

from sine_test.common import LINEAR, SHARE, center, move_time, run_main
from sine_test.sin_test_node import SinTest

MODES = ('hold', 'step', 'ramp')


class Profile:
    """Piecewise command: segments (duration, start, end, shape), shape 'cos' (smooth move, zero speed at both
    ends), 'lin' (constant speed) or 'step' (jumps to end at the segment start, then holds)."""

    def __init__(self, c, segments):
        self.c, self.segs = c, []
        t, p = 0.0, c
        for d, end, shape in segments:
            self.segs.append((t, t + d, p, end, shape))
            t, p = t + d, end
        self.total = t
        self.a = max(abs(s[3] - c) for s in self.segs) if self.segs else 0.0  # largest excursion from center
        self.breaks = [s[0] for s in self.segs]  # segment start times (step edges)

    def __call__(self, t):
        for t0, t1, p0, p1, shape in self.segs:
            if t < t1:
                u, d = max(t - t0, 0.0), t1 - t0
                if shape == 'step': return p1, 0.0
                if shape == 'lin': return p0 + (p1 - p0) * u / d, (p1 - p0) / d
                k = math.pi / d
                return p0 + (p1 - p0) * 0.5 * (1 - math.cos(k * u)), (p1 - p0) * 0.5 * k * math.sin(k * u)
        return (self.segs[-1][3] if self.segs else self.c), 0.0


class ProfileTest(SinTest):
    def __init__(self, joint, mode):
        super().__init__(joint, 'profile_test', str(SHARE / 'config' / 'profile.yaml'))
        self.mode = mode

    def run(self):
        spec = (self.cfg.get('joints') or {}).get(self.joint)
        if spec is not None and self.mode not in spec:  # refuse before waiting for the controller
            self.get_logger().error(f"no '{self.mode}' settings under joints.{self.joint} in {self.cfg_path} — "
                                    f"available: {[m for m in MODES if m in spec]}")
            return 1
        return super().run()

    def run_tag(self):
        return f'_{self.mode}' + super().run_tag()  # + optional yaml `tag` (e.g. _hold_grip)

    def describe(self, spec, prof):
        m = spec[self.mode]
        return f'{self.mode} {m}, center {self.fmt(self.joint, prof.c)}, largest excursion {self.fmt(self.joint, prof.a)}'

    def plan(self, spec):
        """Build the Profile in SI units and check limits / speed. Returns (profile, problem or None)."""
        j, lin = self.joint, self.joint in LINEAR
        if self.mode not in spec: return None, f"no '{self.mode}' settings under joints.{j} in {self.cfg_path}"
        m, unit = spec[self.mode], 1.0 if lin else math.pi / 180
        lo, hi, vmax = self.lim[j]
        c = center(j, spec, self.lim[j])
        dwell = m.get('dwell_s', 2.0)
        segs = []
        if self.mode == 'hold':  # slow move to each angle, dwell, back to center
            p = c
            for x in list(m['targets']) + [None]:
                q = c if x is None else c + x * unit
                segs.append((move_time(self.cfg, self.lim[j], q - p), q, 'cos'))
                if x is not None: segs.append((dwell, q, 'lin'))
                p = q
        elif self.mode == 'step':  # per size: +A, center, -A, center, each held for dwell
            for a in m['sizes']:
                for q in (c + a * unit, c, c - a * unit, c):
                    segs.append((dwell, q, 'step'))
        else:  # ramp: center -> +A -> -A -> center at constant speed, `cycles` times, short dwell at the turns
            a, v = m['amplitude'] * unit, m['speed'] * unit
            for _ in range(m.get('cycles', 1)):
                for q in (c + a, c - a, c):
                    segs.append((abs(q - (segs[-1][1] if segs else c)) / v, q, 'lin'))
        prof = Profile(c, segs)
        mg = self.cfg['limit_margin']['m'] if lin else math.radians(self.cfg['limit_margin']['deg'])
        pts = [s[3] for s in prof.segs] + [c]
        if min(pts) < lo + mg or max(pts) > hi - mg:
            return prof, f'range [{min(pts):.4f}, {max(pts):.4f}] outside limits [{lo + mg:.4f}, {hi - mg:.4f}] (margin incl.)'
        if self.mode != 'step':  # a step is meant to saturate the servo; the others must stay slow
            peak_v = max(abs(prof(i * 0.001)[1]) for i in range(int(prof.total * 1000)))
            if peak_v > self.cfg['speed_margin'] * vmax:
                return prof, f'peak speed {peak_v:.3f} > {self.cfg["speed_margin"]} x limit {vmax}'
        return prof, None


def main():
    parser = argparse.ArgumentParser(description='hold / step / ramp test of one joint (settings: yaml joints.<joint>.<mode>)')
    parser.add_argument('joint_name')
    parser.add_argument('mode', choices=MODES)
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    run_main(partial(ProfileTest, args.joint_name, args.mode))


if __name__ == '__main__':
    main()
