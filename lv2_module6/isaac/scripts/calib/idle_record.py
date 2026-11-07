#!/usr/bin/env python3
"""Record /joint_states while the arm holds still -> encoder/current noise and update timing.

  python3 idle_record.py [seconds=60]      # hw.launch.py running, arm holding (no test node)

Writes logs/<YYYYmmdd_HHMMSS>_idle/log.csv (t_recv, t_stamp, <joint>_pos, <joint>_eff per joint) and prints a summary.
"""
import csv
import math
import sys
import time
from pathlib import Path

import rclpy
from sensor_msgs.msg import JointState

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_left_joint']
OUT = Path(__file__).resolve().parent / 'logs'


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    rclpy.init()
    node = rclpy.create_node('idle_record')
    rows = []

    def cb(msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in JOINTS): return
        eff = lambda i: msg.effort[i] if i < len(msg.effort) else float('nan')
        rows.append([time.time(), msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9]
                    + [msg.position[idx[j]] for j in JOINTS] + [eff(idx[j]) for j in JOINTS])

    node.create_subscription(JointState, '/joint_states', cb, 100)
    end = time.monotonic() + seconds
    print(f'recording {seconds:.0f} s ...')
    while time.monotonic() < end: rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    if not rows: sys.exit('no /joint_states received')

    d = OUT / (time.strftime('%Y%m%d_%H%M%S') + '_idle')
    d.mkdir(parents=True)
    with open(d / 'log.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_recv', 't_stamp'] + [f'{j}_pos' for j in JOINTS] + [f'{j}_eff' for j in JOINTS])
        w.writerows(rows)
    print(f'{len(rows)} msgs -> {d}/log.csv')


if __name__ == '__main__':
    main()
