"""Move the arm to its center pose (every arm joint 0 rad = servo upright), one joint at a time, joint5 -> joint1.

  ros2 run control_test set_center                    # hw.launch.py must be running
  ros2 run control_test set_center joint3             # only joint3 to center, the rest stays
  q (or Ctrl+C): the arm stops where it is and the node exits

Uses the arm_controller topic interface (/arm_controller/joint_trajectory). The other joints keep the JTC's current
command; the gripper keeps its opening. Each joint gets one slow segment (move_s, or longer for big moves), then
the node waits for the segment to end and reports the remaining error before the next joint.
"""
import argparse
import sys
from functools import partial

from rclpy.utilities import remove_ros_args
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from control_test.common import ARM, CONTROLLER, JOINTS, ArmNode, Quit, dur, move_time, run_main

SETTLE = 0.5  # s after each segment before the next joint


class SetCenter(ArmNode):
    def __init__(self, joint=None):
        super().__init__('set_center')
        self.joints = [joint] if joint else list(reversed(ARM))  # one joint, or joint5 -> joint1
        self.pub = self.create_publisher(JointTrajectory, CONTROLLER + '/joint_trajectory', 10)

    def publish(self, pose, t):
        msg = JointTrajectory(joint_names=JOINTS)  # header stamp 0 = start now
        msg.points = [JointTrajectoryPoint(positions=[pose[j] for j in JOINTS], velocities=[0.0] * len(JOINTS),
                                           time_from_start=dur(t))]
        self.pub.publish(msg)

    def hold_here(self):
        """Replace the running segment with the JTC's current command -> the arm stops where it is."""
        if self.reference is None: return
        self.publish(dict(self.reference), 0.1)
        self.get_logger().warning('stopped — holding the current pose')

    def run(self):
        if self.joints[0] not in ARM:
            self.get_logger().error(f"no such arm joint '{self.joints[0]}' — available: {ARM}")
            return 1
        pose = self.wait_ready()
        self.spin_until(lambda: self.pub.get_subscription_count() > 0, 5.0, CONTROLLER + '/joint_trajectory subscriber')
        self.get_logger().info(f'set_center: {" -> ".join(self.joints)} to 0 deg (q: stop)')
        try:
            for j in self.joints:
                lo, hi, _ = self.lim[j]
                target = 0.0
                if not lo <= target <= hi:
                    self.get_logger().error(f'{j}: center {target} outside URDF limits [{lo}, {hi}] — stopping')
                    return 1
                if abs(pose[j] - target) < 1e-4:
                    self.get_logger().info(f'{j}: already at center')
                    continue
                t = move_time(self.cfg, self.lim[j], pose[j] - target)
                self.get_logger().info(f'{j}: {self.fmt(j, pose[j])} -> {self.fmt(j, target)} in {t:.1f} s')
                pose[j] = target
                self.publish(pose, t)
                self.spin_for(t + SETTLE)
                self.get_logger().info(f'{j}: error {self.fmt(j, self.state[j][0] - target)}')
            what = 'center pose reached' if len(self.joints) > 1 else f'{self.joints[0]} at center'
            self.get_logger().info(f'{what} — now: ' + ', '.join(f'{j} {self.fmt(j, self.state[j][0])}' for j in ARM))
        except (Quit, KeyboardInterrupt):
            self.stopping()
            self.hold_here()
        return 0


def main():
    parser = argparse.ArgumentParser(description='Move arm joints to center (0 deg): all joint5 -> joint1, or one joint')
    parser.add_argument('joint_name', nargs='?', help=f'one of {ARM}; omit for all')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    run_main(partial(SetCenter, args.joint_name))


if __name__ == '__main__':
    main()
