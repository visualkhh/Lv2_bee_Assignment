#!/usr/bin/env python3
"""cuvslam_slam — RTAB-Map 대신 cuVSLAM(NVIDIA, GPU)으로 map→odom을 내는 시험용 노드 (slam:=cuvslam)

  입력: pole_front_cam 컬러 + 정렬 depth + camera_info (RTAB-Map과 같은 카메라 · RGBD 모드 — D435는 IMU가 없어 VIO 불가)
  출력: TF map→odom (odom→base_footprint는 EKF 그대로) · /cuvslam/path (RViz로 궤적 보기)
  map = 출발할 때의 base_footprint (RTAB-Map과 같음), x·y·yaw만 (RTAB-Map Reg/Force3DoF와 같음 — 경기장 바닥은 평평)
  /map(격자)는 안 냄 — cuVSLAM은 위치만 냄. 격자가 필요하면 nvblox 또는 RTAB-Map에 이 위치를 넣는 조합 (다음 단계)
  설치 (Jetson, JetPack 7 · Python 3.12 · CUDA 13 — 휠에 cudart 포함, 의존성은 pyyaml뿐):
    pip install --user --break-system-packages \\
      https://github.com/NVlabs/PyCuVSLAM/releases/download/v17.0.1/cuvslam-17.0.1+cu13-cp312-abi3-manylinux_2_39_aarch64.whl
  실행: ./start.sh map:=slam slam:=cuvslam use_nav2:=false use_moveit:=false
"""
import math

import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def mat(q, t):
    """쿼터니언(x, y, z, w) · 이동 → 4x4"""
    x, y, z, w = q
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = t
    return m


def planar(m):
    """4x4 → (x, y, yaw) — z·roll·pitch는 버림"""
    return m[0, 3], m[1, 3], math.atan2(m[1, 0], m[0, 0])


def map_to_odom(base_from_cam, world_from_cam, odom_from_base):
    """map = 출발 때 base_footprint. cuVSLAM world = 출발 때 카메라(OpenCV 축) →
    map_from_base = B·W·B⁻¹ (B = base_from_cam), map_from_odom = map_from_base·odom_from_base⁻¹ (평면으로)"""
    x, y, yaw = planar(base_from_cam @ world_from_cam @ np.linalg.inv(base_from_cam))
    ox, oy, oyaw = planar(odom_from_base)
    a = yaw - oyaw
    c, s = math.cos(a), math.sin(a)
    return x - (c * ox - s * oy), y - (s * ox + c * oy), a


def image(m, dtype, ch):
    """sensor_msgs/Image → numpy (row step 패딩 제거, cv_bridge 없이)"""
    a = np.frombuffer(bytes(m.data), dtype).reshape(m.height, m.step // np.dtype(dtype).itemsize)
    a = a[:, :m.width * ch]
    return a.reshape(m.height, m.width, ch) if ch > 1 else a


class CuvslamSlam(Node):
    def __init__(self):
        super().__init__('cuvslam_slam')
        self.base = self.declare_parameter('base_frame', 'base_footprint').value
        self.odom = self.declare_parameter('odom_frame', 'odom').value
        self.map = self.declare_parameter('map_frame', 'map').value
        self.depth_scale = float(self.declare_parameter('depth_scale_factor', 1000.0).value)  # 16UC1 mm → m
        self.tracker = self.base_from_cam = None
        self.last_ns = 0
        self.ok = self.lost = 0
        self.tf_buf = Buffer()
        TransformListener(self.tf_buf, self, spin_thread=True)  # 콜백 안에서 TF를 읽어도 막히지 않게
        self.tf_pub = TransformBroadcaster(self)
        self.path_pub = self.create_publisher(Path, '/cuvslam/path', 1)
        self.path = Path()
        self.path.header.frame_id = self.map
        self.info = None
        self.create_subscription(CameraInfo, self.declare_parameter('camera_info', '/pole_front_cam/color/camera_info').value,
                                 self.on_info, qos_profile_sensor_data)
        color = Subscriber(self, Image, self.declare_parameter('rgb', '/pole_front_cam/color/image_raw').value,
                           qos_profile=qos_profile_sensor_data)
        depth = Subscriber(self, Image, self.declare_parameter('depth', '/pole_front_cam/aligned_depth_to_color/image_raw').value,
                           qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([color, depth], 10, 0.02)
        self.sync.registerCallback(self.on_rgbd)
        self.create_timer(10.0, lambda: self.get_logger().info(
            f'추적 {self.ok} · 잃음 {self.lost} · 경로 {len(self.path.poses)}'))

    def on_info(self, m):
        self.info = m

    def start(self, frame):
        """첫 프레임: camera_info로 rig, URDF TF로 base_footprint←카메라. SLAM(루프 클로저) 켜고 평면 제약"""
        import cuvslam  # 설치 안 됐으면 여기서만 실패 (RTAB-Map 모드엔 영향 없음)
        try:
            t = self.tf_buf.lookup_transform(self.base, frame, Time()).transform
        except Exception as e:  # noqa: BLE001 — TF가 아직 안 왔으면 다음 프레임에
            self.get_logger().warn(f'TF {self.base}←{frame} 대기: {e}', throttle_duration_sec=5.0)
            return
        r = t.rotation
        self.base_from_cam = mat((r.x, r.y, r.z, r.w), (t.translation.x, t.translation.y, t.translation.z))
        k = self.info.k
        cam = cuvslam.Camera(size=(self.info.width, self.info.height), principal=(k[2], k[5]), focal=(k[0], k[4]),
                             distortion=cuvslam.Distortion(cuvslam.Distortion.Model.Pinhole))
        rgbd = cuvslam.Tracker.OdometryRGBDSettings()
        rgbd.depth_scale_factor = self.depth_scale
        rgbd.depth_camera_id = 0
        rgbd.enable_depth_stereo_tracking = False
        odom_cfg = cuvslam.Tracker.OdometryConfig(odometry_mode=cuvslam.Tracker.OdometryMode.RGBD, async_sba=True,
                                                  rgbd_settings=rgbd)
        self.tracker = cuvslam.Tracker(cuvslam.Rig([cam]), odom_cfg, cuvslam.Tracker.SlamConfig(planar_constraints=True))
        self.get_logger().info(f'cuVSLAM {cuvslam.get_version()[0]} 시작 · '
                               f'{self.info.width}x{self.info.height} · 카메라 {frame}')

    def on_rgbd(self, c, d):
        if self.info is None:
            return
        if self.tracker is None:
            self.start(c.header.frame_id)
            if self.tracker is None:
                return
        if (c.height, c.width) != (d.height, d.width):  # 정렬 depth는 컬러 크기여야 (RGBD 모드)
            self.get_logger().error(f'컬러 {c.width}x{c.height} ≠ depth {d.width}x{d.height}', throttle_duration_sec=5.0)
            return
        ns = Time.from_msg(c.header.stamp).nanoseconds
        if ns <= self.last_ns:  # cuVSLAM은 시각이 늘 늘어나야 함
            return
        self.last_ns = ns
        rgb = image(c, np.uint8, 3) if c.encoding in ('rgb8', 'bgr8') else image(c, np.uint8, 1)
        est, slam = self.tracker.track(ns, [rgb], depths=[image(d, np.uint16, 1)])
        if est.world_from_rig is None:
            self.lost += 1
            self.get_logger().warn('추적 잃음', throttle_duration_sec=2.0)
            return
        self.ok += 1
        p = slam if slam is not None else est.world_from_rig.pose  # 루프 클로저 반영된 SLAM 위치 우선
        try:
            o = self.tf_buf.lookup_transform(self.odom, self.base, c.header.stamp).transform
        except Exception:  # noqa: BLE001 — 그 시각 odom이 없으면 이 프레임은 TF만 건너뜀 (추적은 계속)
            return
        odom_from_base = mat((o.rotation.x, o.rotation.y, o.rotation.z, o.rotation.w),
                             (o.translation.x, o.translation.y, o.translation.z))
        world_from_cam = mat(p.rotation, p.translation)
        x, y, yaw = map_to_odom(self.base_from_cam, world_from_cam, odom_from_base)
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = c.header.stamp, self.map, self.odom
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf_pub.sendTransform(tf)
        bx, by, byaw = planar(self.base_from_cam @ world_from_cam @ np.linalg.inv(self.base_from_cam))
        ps = PoseStamped()
        ps.header = tf.header
        ps.pose.position.x, ps.pose.position.y = bx, by
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(byaw / 2), math.cos(byaw / 2)
        self.path.header.stamp = c.header.stamp
        self.path.poses.append(ps)
        if len(self.path.poses) % 5 == 0:  # 경로는 3Hz 정도로만
            self.path_pub.publish(self.path)


def main():
    rclpy.init()
    node = CuvslamSlam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
