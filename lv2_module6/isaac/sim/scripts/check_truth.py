#!/usr/bin/env python3
"""Isaac 정답 위치 ↔ ROS 위치 추정 비교 (twin.sh 실행 중) — 2초마다 한 줄.

  python3 check_truth.py        정답(opencr_bridge가 /tmp/bee_truth.txt) · TF map→base_footprint · odom→base_footprint · 차이
"""
import math
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def pose(buf, parent):
    t = buf.lookup_transform(parent, 'base_footprint', rclpy.time.Time()).transform
    q = t.rotation
    return t.translation.x, t.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    n = Node('check_truth')
    buf = Buffer()
    TransformListener(buf, n)
    last = 0.0
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time() - last < 2.0:
            continue
        last = time.time()
        try:
            tt, tx, ty, tyaw = (float(v) for v in open('/tmp/bee_truth.txt').read().split())
            mx, my, myaw = pose(buf, 'map')
            ox, oy, oyaw = pose(buf, 'odom')
        except Exception as e:  # noqa: BLE001
            print('기다림:', e)
            continue
        err = math.hypot(mx - tx, my - ty)
        print(f'정답 ({tx:+.2f}, {ty:+.2f}, {math.degrees(tyaw):+5.0f}°) · map 추정 ({mx:+.2f}, {my:+.2f}, {math.degrees(myaw):+5.0f}°) '
              f'오차 {err:.2f} m {math.degrees(math.remainder(myaw - tyaw, 2 * math.pi)):+.0f}° · odom ({ox:+.2f}, {oy:+.2f})', flush=True)


if __name__ == '__main__':
    main()
