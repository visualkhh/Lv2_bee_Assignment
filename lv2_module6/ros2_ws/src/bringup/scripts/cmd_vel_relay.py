#!/usr/bin/env python3
"""stand_test.launch.py 전용: twist_mux 출력 /cmd_vel_mux → diff_drive /cmd_vel 그대로 넘김.

전체 bringup은 그 사이에 Nav2 velocity_smoother · collision_monitor가 있는데, 가벼운 팔 테스트 실행엔 Nav2가 없어
수동 주행(operator_tool drive · teleop)이 바퀴까지 안 갔다. 부드럽게 하기·충돌 감시는 없음 — 느리게만 몰 것.
"""
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__("cmd_vel_relay")
        pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(TwistStamped, "/cmd_vel_mux", pub.publish, 10)


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
