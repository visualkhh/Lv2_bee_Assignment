#!/usr/bin/env python3
"""Gazebo→ROS 릴레이 (develop/gazebo/relay.py): /gz_raw/* (시뮬 시간) → 벽시계 찍어 표준 토픽으로 재배포.

왜: Gazebo 스탬프(시뮬 시간)와 TF·EKF(벽시계)가 어긋나면 RViz 포인트·Camera·perception TF 조회가 실패.
QoS: 구독 reliable(브리지) → 발행 best_effort(파이프라인 관례: fake_camera·RealSense와 동일).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, PointCloud2
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry

RELAYS = [
    ("pole_front_cam", Image, "color/image_raw", "pole_front_cam_color_optical_frame"),
    ("pole_front_cam", Image, "aligned_depth_to_color/image_raw", "pole_front_cam_color_optical_frame"),
    ("pole_front_cam", CameraInfo, "color/camera_info", "pole_front_cam_color_optical_frame"),
    ("pole_front_cam", CameraInfo, "aligned_depth_to_color/camera_info", "pole_front_cam_color_optical_frame"),
    # gz points는 optical이 아니라 센서(링크) 프레임: x=전방, y=왼쪽, z=위.
    #   optical(+Z 전방)로 라벨링하면 RViz가 90° 돌려서 그리므로 링크 프레임으로 발행.
    ("pole_front_cam", PointCloud2, "depth/color/points", "pole_front_cam_link"),
    ("gripper_cam", Image, "color/image_raw", "gripper_cam_color_optical_frame"),
    ("gripper_cam", Image, "aligned_depth_to_color/image_raw", "gripper_cam_color_optical_frame"),
    ("gripper_cam", CameraInfo, "color/camera_info", "gripper_cam_color_optical_frame"),
    ("gripper_cam", CameraInfo, "aligned_depth_to_color/camera_info", "gripper_cam_color_optical_frame"),
    ("gripper_cam", PointCloud2, "depth/color/points", "gripper_cam_link"),
]


class GzRelay(Node):
    def __init__(self):
        super().__init__("gz_relay")
        # testbed mock 관절 → Gazebo JPC (미러: testbed가 리드, Gazebo가 따라옴)
        self.arm_pubs = {}
        for j in ["joint1", "joint2", "joint3", "joint4", "joint5",
                  "gripper_left_joint", "gripper_right_joint"]:
            self.arm_pubs[j] = self.create_publisher(Float64, f"/gz_arm/{j}/cmd", 10)

        def arm_cb(m):
            pos = dict(zip(m.name, m.position))
            for j, pub in self.arm_pubs.items():
                if j in pos:
                    msg = Float64()
                    msg.data = float(pos[j])
                    pub.publish(msg)

        self.create_subscription(JointState, "/joint_states", arm_cb,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        sub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        be_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        rel_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        for cam, typ, rest, frame in RELAYS:
            pubs = [self.create_publisher(typ, f"/{cam}/{rest}", be_qos),   # perception 등 센서 QoS
                    self.create_publisher(typ, f"/{cam}/{rest}", rel_qos)]  # RViz 기본값 등

            def cb(m, pubs=pubs, frame=frame):
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = frame  # gz 이름(beebot/...) → URDF 광학 프레임
                for p in pubs:
                    p.publish(m)

            self.create_subscription(typ, f"/gz_raw/{cam}/{rest}", cb, sub_qos)
        self.imu_pubs = [self.create_publisher(Imu, "/imu_gz", be_qos),
                         self.create_publisher(Imu, "/imu_gz", rel_qos)]

        def imu_cb(m):
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "imu_link"
            for p in self.imu_pubs:
                p.publish(m)

        self.create_subscription(Imu, "/gz_raw/imu_gz", imu_cb, sub_qos)

        # teleop → Gazebo와 mock을 같이 굴림 (RViz 로봇이 Gazebo를 따라오게)
        # testbed 체인은 TwistStamped (teleop stamped:=true와 같음)
        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", rel_qos)

        def cmd_cb(m):
            s = TwistStamped()
            s.header.stamp = self.get_clock().now().to_msg()
            s.header.frame_id = "base_footprint"
            s.twist = m
            self.cmd_pub.publish(s)

        self.create_subscription(Twist, "/gz/cmd_vel", cmd_cb, rel_qos)
        self.get_logger().info(f"중계 시작: {len(RELAYS)}개 토픽 + /imu_gz (best_effort·reliable 이중 발행)")


def main():
    rclpy.init()
    node = GzRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
