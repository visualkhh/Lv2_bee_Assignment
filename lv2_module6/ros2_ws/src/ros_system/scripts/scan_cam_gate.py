#!/usr/bin/env python3
"""scan_cam_gate — gripper_cam depth를 팔이 스캔 자세에 멈춰 있을 때만 RTAB-Map에 넘김

  gripper_cam은 팔 끝에 달려 움직임. 지도를 만들 땐 팔을 스캔 자세(config/scan_pose.yaml — 왼쪽 뒤를 봄)에 고정해 두 번째 카메라로 쓰고,
  그 밖(파지 · 수동 조작 · 움직이는 중)엔 depth를 0(측정 없음)으로 채워 넘김 → RTAB-Map이 그 프레임에서 특징점·3D를 못 얻어 지도를 안 망침.
  프레임을 버리지 않고 비운 depth로 넘기는 이유: RTAB-Map이 두 카메라를 함께 동기화해 받으므로(rgbd_cameras 2) 한쪽이 끊기면 둘 다 멈춤.
  입력: depth_in (gripper_cam 정렬 depth) · /joint_states · 출력: depth_out (같은 시각·프레임·크기)
  점구름도 같은 규칙: cloud_in → cloud_out — 스캔 자세면 카메라에서 min_range 안 점(손가락 ~0.1m · 폴대 ~0.17m · 적재함 ~0.3m)을 빼고,
  아니면 빈 점구름 (같은 시각). pole_front_cam 점구름과 합쳐(point_cloud_aggregator) RTAB-Map 격자·ICP에 들어감. cloud_in이 비면 점구름은 안 다룸
  nvblox(slam:=cuvslam_nvblox)도 같은 노드를 씀 — nvblox는 depth 영상으로 지도를 만들어서 영상에서 자기 몸을 빼야 함:
    depth_min_range > 0 → 카메라에서 그 안 픽셀을 0(측정 없음)으로 · gate_by_pose false → 자세와 상관없이 늘 넘김 (pole_front_cam)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState, PointCloud2


def near_zeroed(m, min_range):
    """depth 영상에서 min_range [m] 안 픽셀을 0(측정 없음)으로 — 16UC1(mm) · 32FC1(m). 그 밖 인코딩은 그대로"""
    if m.encoding == '16UC1':
        dtype, limit = np.uint16, min_range * 1000.0
    elif m.encoding == '32FC1':
        dtype, limit = np.float32, min_range
    else:
        return m
    d = np.frombuffer(bytes(m.data), dtype).reshape(m.height, m.step // np.dtype(dtype).itemsize).copy()
    d[(d > 0) & (d < limit)] = 0
    out = Image(header=m.header, height=m.height, width=m.width, encoding=m.encoding, is_bigendian=m.is_bigendian, step=m.step)
    out.data = d.tobytes()
    return out


class ScanCamGate(Node):
    def __init__(self):
        super().__init__('scan_cam_gate')
        self.joints = list(self.declare_parameter('scan_pose_joints', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']).value)
        self.pose = [float(v) for v in self.declare_parameter('scan_pose', [2.618, 0.681, -1.414, 1.082, 0.0]).value]
        self.tol = float(self.declare_parameter('scan_pose_tolerance', 0.05).value)
        self.max_vel = float(self.declare_parameter('max_joint_velocity', 0.02).value)  # [rad/s] 이보다 빠르면 움직이는 중
        self.q, self.v = {}, {}
        self.passed = self.blanked = 0
        self.pub = self.create_publisher(Image, self.declare_parameter('depth_out', '/gripper_cam/aligned_depth_to_color/image_scan').value,
                                         qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(Image, self.declare_parameter('depth_in', '/gripper_cam/aligned_depth_to_color/image_raw').value,
                                 self.on_depth, qos_profile_sensor_data)
        self.min_range = float(self.declare_parameter('min_range', 0.5).value)
        self.gate_by_pose = bool(self.declare_parameter('gate_by_pose', True).value)  # false = 자세와 상관없이 늘 넘김
        self.depth_min_range = float(self.declare_parameter('depth_min_range', 0.0).value)  # [m] > 0이면 depth에서 이 안 픽셀을 0으로
        cloud_in = self.declare_parameter('cloud_in', '/gripper_cam/depth/color/points').value
        cloud_out = self.declare_parameter('cloud_out', '/gripper_cam/depth/color/points_scan').value
        if cloud_in:
            self.cloud_pub = self.create_publisher(PointCloud2, cloud_out, qos_profile_sensor_data)
            self.create_subscription(PointCloud2, cloud_in, self.on_cloud, qos_profile_sensor_data)
        self.create_timer(10.0, lambda: self.get_logger().info(
            f'스캔 자세 {"맞음" if self.in_pose() else "아님"} · 넘김 {self.passed} · 비움 {self.blanked}'))

    def on_joints(self, m):
        for i, n in enumerate(m.name):
            if i < len(m.position):
                self.q[n] = m.position[i]
            if i < len(m.velocity):
                self.v[n] = m.velocity[i]

    def in_pose(self):
        if not self.gate_by_pose:
            return True
        for n, p in zip(self.joints, self.pose):
            if n not in self.q:
                return False
            d = (self.q[n] - p + np.pi) % (2 * np.pi) - np.pi  # joint1은 ±π가 같은 자리
            if abs(d) > self.tol or abs(self.v.get(n, 0.0)) > self.max_vel:
                return False
        return True

    def on_depth(self, m):
        if self.in_pose():
            self.passed += 1
            self.pub.publish(near_zeroed(m, self.depth_min_range) if self.depth_min_range > 0 else m)
            return
        self.blanked += 1
        out = Image(header=m.header, height=m.height, width=m.width, encoding=m.encoding, is_bigendian=m.is_bigendian, step=m.step)
        out.data = bytes(len(m.data))  # 0 = 측정 없음 (16UC1 mm · 32FC1 m 둘 다)
        self.pub.publish(out)


    def on_cloud(self, m):
        """스캔 자세면 가까운 점(자기 몸)만 빼고 넘김, 아니면 빈 점구름. x·y·z가 앞 12바이트(float32)인 점구름 (fake_camera·RealSense)"""
        out = PointCloud2(header=m.header, height=1, fields=m.fields, is_bigendian=m.is_bigendian, point_step=m.point_step,
                          is_dense=m.is_dense)
        if self.in_pose() and m.point_step >= 12 and len(m.data):
            raw = np.frombuffer(bytes(m.data), np.uint8).reshape(-1, m.point_step)
            xyz = raw[:, :12].copy().view(np.float32).reshape(-1, 3)
            keep = np.isfinite(xyz).all(1) & (np.linalg.norm(xyz, axis=1) > self.min_range)
            rows = raw[keep]
            out.data = rows.tobytes()
            out.width = int(len(rows))
        else:
            out.data = b''
            out.width = 0
        out.row_step = out.width * out.point_step
        self.cloud_pub.publish(out)


def main():
    rclpy.init()
    node = ScanCamGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
