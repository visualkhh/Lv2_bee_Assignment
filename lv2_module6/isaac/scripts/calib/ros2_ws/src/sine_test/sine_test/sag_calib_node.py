"""Gravity sag data for the k_i fit: visit poses, stop, log servo current next to a hand-measured tip position.

  ros2 run sine_test sag_calib                                   # config/sag_poses.yaml, hw.launch.py running
  ros2 run sine_test sag_calib --ros-args -p config:=/path/poses.yaml
Per pose and approach direction (+ / -, joint2-4 come from approach_deg above / below the pose): press Enter (step
away from the arm first) -> slow move -> settle -> /joint_states averaged over avg_s -> you type the measured
fingertip "x y z" in mm (forward / left / up from the origin point of the config: chassis front edge, centre line,
floor; the same point on the fingers every time) -> one CSV row, stored in base_link coordinates (origin_mm added).
Empty line = skip this sample, q = quit. Ctrl+C: the arm stops where it is, the log so far is kept.
Output: <out_dir>/sag_<YYYYmmdd_HHMMSS>/log.csv, meta.yaml. effort = servo Present Current [A].
"""
import csv
import math
import statistics
import sys
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.signals import SignalHandlerOptions
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from sine_test.common import ARM, CONTROLLER, DEFAULT_OUT_DIR, JOINTS, SHARE, ArmNode, dur, move_time

POSES = str(SHARE / 'config' / 'sag_poses.yaml')
PITCH = ('joint2', 'joint3', 'joint4')  # gravity-loaded joints: their approach direction matters (backlash, friction)


def parse_xyz(line):
    """'x y z' (mm) -> (x, y, z), or None if it is not three numbers."""
    try:
        v = [float(s) for s in line.replace(',', ' ').split()]
    except ValueError:
        return None
    return tuple(v) if len(v) == 3 else None


class SagCalib(ArmNode):
    def __init__(self):
        super().__init__('sag_calib', config=POSES)
        self.out_dir = Path(self.declare_parameter('out_dir', DEFAULT_OUT_DIR).value)
        self.pub = self.create_publisher(JointTrajectory, CONTROLLER + '/joint_trajectory', 10)

    def publish(self, pose, t):
        msg = JointTrajectory(joint_names=JOINTS)
        msg.points = [JointTrajectoryPoint(positions=[pose[j] for j in JOINTS], velocities=[0.0] * len(JOINTS),
                                           time_from_start=dur(t))]
        self.pub.publish(msg)

    def goto(self, pose, cur):
        """Slow move of every joint at once (the slowest joint sets the time) + settle. Returns the new pose."""
        t = max(move_time(self.cfg, self.lim[j], cur[j] - pose[j]) for j in JOINTS)
        self.publish(pose, t)
        self.spin_for(t + self.cfg['settle_s'])
        return dict(pose)

    def target(self, deg, sign):
        """Pose in rad from {joint: deg}; sign != 0 shifts joint2-4 by sign * approach_deg. Refuses URDF limits."""
        pose = {j: math.radians(deg[j] + (sign * self.cfg['approach_deg'] if j in PITCH else 0.0)) for j in ARM}
        pose['gripper_left_joint'] = self.reference['gripper_left_joint']  # keep the opening
        for j in ARM:
            lo, hi, _ = self.lim[j]
            if not lo <= pose[j] <= hi:
                raise ValueError(f'{j} {math.degrees(pose[j]):.1f} deg outside URDF limits '
                                 f'[{math.degrees(lo):.1f}, {math.degrees(hi):.1f}]')
        return pose

    def average(self):
        buf = []
        self.on_state = lambda m: buf.append(dict(self.state))
        self.spin_for(self.cfg['avg_s'])
        self.on_state = None
        if not buf: raise RuntimeError('no /joint_states during the averaging window')
        mean = lambda j, k: statistics.fmean(s[j][k] for s in buf)
        std = lambda j, k: statistics.pstdev(s[j][k] for s in buf)
        return ({j: mean(j, 0) for j in JOINTS}, {j: mean(j, 2) for j in JOINTS}, {j: std(j, 2) for j in JOINTS}, len(buf))

    def hold_here(self):
        if self.reference is None: return
        self.publish(dict(self.reference), 0.1)
        self.get_logger().warning('stopped — holding the current pose')

    def run(self):
        cur = self.wait_ready()
        out = self.out_dir / time.strftime('sag_%Y%m%d_%H%M%S')
        out.mkdir(parents=True, exist_ok=True)
        (out / 'meta.yaml').write_text(yaml.safe_dump({'config': self.cfg_path, 'poses': self.cfg['poses'],
                                                       'approach_deg': self.cfg['approach_deg'],
                                                       'columns': 'pose, approach, cmd_/enc_ (rad), cur_/cur_std_ (A), '
                                                                  'meas_*_mm (hand measured fingertip, base_link frame)'}))
        header = (['pose', 'approach'] + [f'cmd_{j}' for j in ARM] + [f'enc_{j}' for j in JOINTS]
                  + [f'cur_{j}' for j in JOINTS] + [f'cur_std_{j}' for j in JOINTS]
                  + ['n_samples', 'meas_x_mm', 'meas_y_mm', 'meas_z_mm'])
        self.get_logger().info(f'logging to {out / "log.csv"}')
        rows = 0
        try:
            with open(out / 'log.csv', 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(header)
                for name, deg in self.cfg['poses'].items():
                    for sign, tag in ((+1, 'above'), (-1, 'below')):
                        pre, pose = self.target(deg, sign), self.target(deg, 0)
                        print(f'\n[{name} / approach {tag}] {" ".join(f"{j} {deg[j]:.1f}" for j in ARM)} deg')
                        if 'tip_mm' in deg:
                            print('  expected fingertip from origin: ' + ' '.join(f'{v:.0f}' for v in deg['tip_mm']) + ' mm')
                        ans = input('  Enter = move (hands off the arm), s = skip, q = quit: ').strip().lower()
                        if ans == 'q': raise KeyboardInterrupt
                        if ans == 's': continue
                        cur = self.goto(pre, cur)
                        cur = self.goto(pose, cur)
                        pos, eff, std, n = self.average()
                        print('  current [A]: ' + '  '.join(f'{j[-1]} {eff[j]:+.4f}' for j in ARM)
                              + f'   (enc err deg: ' + ' '.join(f'{math.degrees(pos[j] - pose[j]):+.2f}' for j in ARM) + ')')
                        while True:
                            line = input('  measured tip "x y z" mm (empty = skip, q = quit): ').strip()
                            if line.lower() == 'q': raise KeyboardInterrupt
                            xyz = parse_xyz(line) if line else ()
                            if xyz is not None: break
                            print('  need three numbers, e.g. 212.5 -8 95')
                        if not xyz: continue
                        xyz = tuple(a + b for a, b in zip(xyz, self.cfg['origin_mm']))
                        w.writerow([name, tag] + [f'{pose[j]:.5f}' for j in ARM] + [f'{pos[j]:.5f}' for j in JOINTS]
                                   + [f'{eff[j]:.5f}' for j in JOINTS] + [f'{std[j]:.5f}' for j in JOINTS] + [n, *xyz])
                        f.flush()
                        rows += 1
        except (KeyboardInterrupt, EOFError):
            self.hold_here()
        self.get_logger().info(f'{rows} rows saved: {out / "log.csv"}')
        return 0


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl+C -> KeyboardInterrupt, context stays up
    node = SagCalib()
    try:
        code = node.run() or 0
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
