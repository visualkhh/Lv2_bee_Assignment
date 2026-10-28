"""Puck grip tests of the gripper (gripper_left_joint hold profiles, settings in config/grip/).

  ros2 run control_test grip_test basic            # 9 -> 1.2 -> 9 mm (open -> closed at the 0.6 A limit -> open) (config/grip/grip.yaml)
  ros2 run control_test grip_test sweep box        # 9 -> 4.8 / 4.6 / 4.4 / 1.2 mm, open in between (grip_sweep.yaml)
  ros2 run control_test grip_test pull cyl_down    # closed (1.2 mm) for 30 s while the puck is pulled out (grip_pull.yaml)
  q (or Ctrl+C): the goal is cancelled, the gripper stops where it is, the log so far is saved

Same node as `profile_test gripper_left_joint hold` with the grip yaml as default config. The optional label
(puck type, pull direction) is appended to the run tag, so every puck / direction keeps its own latest run in
profile_analysis.py: logs/profile/<ts>_gripper_left_joint_hold_grip_sweep_box/.
"""
import argparse
import sys
from functools import partial

from rclpy.utilities import remove_ros_args

from control_test.common import SHARE, run_main
from control_test.profile_test_node import ProfileTest

GRIPPER = 'gripper_left_joint'
CONFIGS = {'basic': 'grip.yaml', 'sweep': 'grip_sweep.yaml', 'pull': 'grip_pull.yaml'}


class GripTest(ProfileTest):
    def __init__(self, test, label):
        super().__init__(GRIPPER, 'hold', 'grip_test', str(SHARE / 'config' / 'grip' / CONFIGS[test]))
        if label: self.cfg['tag'] = f"{self.cfg.get('tag', 'grip')}_{label}"


def main():
    parser = argparse.ArgumentParser(description='puck grip test of the gripper (settings: config/grip/*.yaml)')
    parser.add_argument('test', choices=CONFIGS)
    parser.add_argument('label', nargs='?', default='', help='puck type / pull direction, appended to the run tag (e.g. box, cyl_down)')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    run_main(partial(GripTest, args.test, args.label))


if __name__ == '__main__':
    main()
