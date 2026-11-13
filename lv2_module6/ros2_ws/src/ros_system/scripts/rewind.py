#!/usr/bin/env python3
"""경로 리와인드 — 실패한 주행의 실제 궤적(odom)을 따라 0.4 m 후진한다.

  /navigate_to_pose·/navigate_through_poses 액션이 ABORTED 되면 자동 시작.
  /odometry/filtered의 실제 주행 궤적을 기록해 두었다가 가장 최근 점에서
  뒤로 0.4 m 지점까지 역방향 pure-pursuit. /cmd_vel_smoothed로 발행하므로
  collision_monitor를 그대로 통과한다. 수동: ros2 service call /rewind std_srvs/srv/Trigger
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatusArray


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Rewind(Node):
    def __init__(self):
        super().__init__('rewind')
        self.traj = deque(maxlen=4000)   # [(x,y)] 실제 주행 궤적
        self.odom = None                 # (x,y,yaw)
        self.active = False
        self.target = None
        self.start_xy = None
        self.t0 = 0.0
        self.active_goals = set()
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 10)
        for topic in ('/navigate_to_pose/_action/status', '/navigate_through_poses/_action/status'):
            self.create_subscription(GoalStatusArray, topic, self.on_status, 10)
        self.create_service(Trigger, '/rewind', self.on_rewind)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel_smoothed', 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info('rewind ready — abort 시 자동 0.4 m 후진 · 수동 /rewind')

    def on_odom(self, msg):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))
        x, y = p.position.x, p.position.y
        if not self.traj or math.dist((x, y), self.traj[-1]) > 0.01:
            self.traj.append((x, y))

    def on_status(self, msg):
        for s in msg.status_list:
            key = bytes(s.goal_info.goal_id.uuid)
            if s.status in (1, 2, 3):        # ACCEPTED / EXECUTING / CANCELING
                self.active_goals.add(key)
            elif s.status == 6 and key in self.active_goals:  # ABORTED
                self.active_goals.discard(key)
                self.start_rewind()

    def on_rewind(self, _req, res):
        ok = self.start_rewind()
        res.success = ok
        res.message = f'rewind to {self.target}' if ok else 'no trajectory'
        return res

    def start_rewind(self):
        if self.active:
            return True
        if self.odom is None or len(self.traj) < 5:
            return False
        x, y, yaw = self.odom
        pts = list(self.traj)
        acc = 0.0
        idx = len(pts) - 1
        while idx > 0 and acc < 0.40:
            acc += math.dist(pts[idx], pts[idx - 1])
            idx -= 1
        tx, ty = pts[idx]
        dx, dy = tx - x, ty - y
        ex = math.cos(yaw) * dx + math.sin(yaw) * dy
        while idx > 0 and ex > -0.05:        # 목표는 반드시 로봇 뒤쪽
            idx -= 1
            tx, ty = pts[idx]
            dx, dy = tx - x, ty - y
            ex = math.cos(yaw) * dx + math.sin(yaw) * dy
        self.target = (tx, ty)
        self.start_xy = (x, y)
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.active = True
        self.get_logger().info(f'rewind start: idx={idx}/{len(pts)} target=({tx:.2f},{ty:.2f}) ex={ex:.2f}')
        return True

    def tick(self):
        if not self.active:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        x, y, yaw = self.odom
        tx, ty = self.target
        dx, dy = tx - x, ty - y
        ex = math.cos(yaw) * dx + math.sin(yaw) * dy
        ey = -math.sin(yaw) * dx + math.cos(yaw) * dy
        dist_to_target = math.hypot(dx, dy)
        moved = math.dist((x, y), self.start_xy)
        stop = dist_to_target < 0.06 or moved >= 0.45 or (now - self.t0) > 8.0 or ex > 0.05
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        if stop:
            self.get_logger().info(f'rewind stop: target={dist_to_target:.2f} moved={moved:.2f} ex={ex:.2f}')
            self.active = False
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
        else:
            msg.twist.linear.x = -0.05
            msg.twist.angular.z = max(-0.8, min(0.8, -1.2 * math.atan2(ey, -ex)))
        self.pub.publish(msg)


rclpy.init()
n = Rewind()
try:
    rclpy.spin(n)
finally:
    n.destroy_node()
    rclpy.shutdown()
