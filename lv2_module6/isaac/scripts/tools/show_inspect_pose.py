import json, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rclpy
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from inspect_pose import look_at

import math
if len(sys.argv) > 1:  # joint1~5 [deg], e.g. 0,-96.4,18.6,110.4,0
    q = [math.radians(float(v)) for v in sys.argv[1].split(',')]
else:  # look at the start_box top centre
    trial = json.load(open('/tmp/beebot_trial.json'))
    q, info = look_at((trial['box'][0], trial['box'][1], 0.13))
    print('box', trial['box'], info, flush=True)
print('q [rad]', [round(v, 3) for v in q], flush=True)
rclpy.init()
n = rclpy.create_node('show_inspect')
pub = n.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
while pub.get_subscription_count() == 0:
    rclpy.spin_once(n, timeout_sec=0.5)
time.sleep(3)
pub.publish(JointTrajectory(joint_names=['joint1', 'joint2', 'joint3', 'joint4', 'joint5'],
                            points=[JointTrajectoryPoint(positions=[float(v) for v in q], time_from_start=Duration(sec=4))]))
rclpy.spin_once(n, timeout_sec=1.0)
print('sent', flush=True)
