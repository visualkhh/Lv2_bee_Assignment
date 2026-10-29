#!/usr/bin/env python3
"""cuvslam_slam — cuVSLAM(NVIDIA, GPU) 시각 위치 추정. 쓰는 방식은 launch 인자로 고른다 (ros_system.launch.py)

  slam:=rtabmap cuvslam_odom:=true (권장) — 시각 오도메트리만: /cuvslam/odom(속도) → EKF가 바퀴·IMU와 합침 → RTAB-Map은 그대로
                                            (지도 · 저장 · 위치추정 · Nav2 · explorer 전부 RTAB-Map 기준 그대로)
  slam:=cuvslam_nvblox                     — SLAM: 루프 클로저 켜고 TF map→odom · /cuvslam/path (/map 격자는 nvblox → nvblox_map)
  두 방식 모두 /cuvslam/odom을 냄 (EKF는 이 토픽이 없으면 그냥 바퀴·IMU만 씀)

  입력: pole_front_cam 컬러 + 정렬 depth + camera_info (RGBD 모드 — D435는 IMU가 없어 VIO 불가)
  map = 출발할 때의 base_footprint (RTAB-Map과 같음), x·y·yaw만 (RTAB-Map Reg/Force3DoF와 같음 — 경기장 바닥은 평평)
  카메라에서 min_depth 안 픽셀은 가림 — 화면 가운데 찍히는 자기 팔이 로봇과 같이 움직여 회전을 덜 잡게 함 (RTAB-Map Vis/MinDepth와 같은 이유)
  설치 (Jetson Orin + JetPack 7 — 자세히: documents/jetson-orin-nano/CUVSLAM.md): 배포 휠(cu13 aarch64)은 Thor 전용(SVE)이라 Orin에선 Illegal instruction → 소스 빌드
    sudo apt install cuda-nvcc-13-2 cuda-cudart-dev-13-2 libcublas-dev-13-2 libcusolver-dev-13-2 libcusparse-dev-13-2 \\
                     libnvjitlink-dev-13-2 cuda-nvrtc-dev-13-2 cuda-driver-dev-13-2
    git clone --branch v17.0.1 https://github.com/NVlabs/PyCuVSLAM.git ~/cuvslam/src
    PATH=/usr/local/cuda/bin:$PATH CUVSLAM_SRC_DIR=~/cuvslam/src CUVSLAM_DST_DIR=~/cuvslam/build \\
      ~/cuvslam/src/build_release.sh --build_lib --cuda_arch=87 --jobs=4
    CUVSLAM_BUILD_DIR=~/cuvslam/build pip install --user --break-system-packages ~/cuvslam/src/python/
"""
import math

import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
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


def base_pose(base_from_cam, world_from_cam):
    """cuVSLAM world = 출발 때 카메라(OpenCV 축) → 출발 때 base_footprint 기준 지금 base_footprint (x, y, yaw)
    map_from_base = B·W·B⁻¹ (B = base_from_cam)"""
    return planar(base_from_cam @ world_from_cam @ np.linalg.inv(base_from_cam))


def map_to_odom(base, odom_from_base):
    """map_from_odom = map_from_base · odom_from_base⁻¹ (평면). base = base_pose() 결과"""
    x, y, yaw = base
    ox, oy, oyaw = planar(odom_from_base)
    a = yaw - oyaw
    c, s = math.cos(a), math.sin(a)
    return x - (c * ox - s * oy), y - (s * ox + c * oy), a


def body_twist(prev, cur, dt):
    """두 평면 자세 사이 속도를 base_footprint 기준으로 (vx, vy, vyaw) — EKF에 넣는 값"""
    dx, dy = cur[0] - prev[0], cur[1] - prev[1]
    c, s = math.cos(prev[2]), math.sin(prev[2])
    dyaw = (cur[2] - prev[2] + math.pi) % (2 * math.pi) - math.pi
    return (c * dx + s * dy) / dt, (-s * dx + c * dy) / dt, dyaw / dt


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
        # true = 단독 SLAM(루프 클로저 + TF map→odom) · false = 시각 오도메트리만 (RTAB-Map이 map→odom)
        self.publish_map_tf = bool(self.declare_parameter('publish_map_tf', True).value)
        self.depth_scale = float(self.declare_parameter('depth_scale_factor', 1000.0).value)  # 16UC1 mm → m
        self.min_depth = float(self.declare_parameter('min_depth', 0.5).value)  # [m] 이 안 픽셀(자기 팔)은 추적에서 뺌
        # EKF에 주는 속도 분산 — 실측으로 맞출 값 (σ 0.05 m/s · 0.05 rad/s에서 시작)
        self.lin_var = float(self.declare_parameter('linear_variance', 0.0025).value)
        self.ang_var = float(self.declare_parameter('angular_variance', 0.0025).value)
        self.tracker = self.base_from_cam = None
        self.last_ns = 0
        self.prev = None  # (ns, (x, y, yaw)) — 직전 오도메트리 자세. 추적을 잃으면 None (cuVSLAM 좌표가 새로 시작하므로 그 사이 속도는 안 냄)
        self.ok = self.lost = 0
        self.tf_buf = Buffer()
        TransformListener(self.tf_buf, self, spin_thread=True)  # 콜백 안에서 TF를 읽어도 막히지 않게
        self.tf_pub = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, self.declare_parameter('odom_topic', '/cuvslam/odom').value, 10)
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
            f'추적 {self.ok} · 잃음 {self.lost}' + (f' · 경로 {len(self.path.poses)}' if self.publish_map_tf else '')))

    def on_info(self, m):
        self.info = m

    def start(self, frame):
        """첫 프레임: camera_info로 rig, URDF TF로 base_footprint←카메라. 단독 SLAM이면 루프 클로저(평면 제약) 켬"""
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
        slam_cfg = cuvslam.Tracker.SlamConfig(planar_constraints=True) if self.publish_map_tf else None
        self.tracker = cuvslam.Tracker(cuvslam.Rig([cam]), odom_cfg, slam_cfg)
        self.get_logger().info(f'cuVSLAM {cuvslam.get_version()[0]} 시작 ({"단독 SLAM" if slam_cfg else "시각 오도메트리"}) · '
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
        depth = image(d, np.uint16, 1)
        mask = ((depth > 0) & (depth < self.min_depth * self.depth_scale)).astype(np.uint8)  # 0이 아닌 픽셀 = 추적에서 뺌
        est, slam = self.tracker.track(ns, [rgb], masks=[mask], depths=[depth])
        if est.world_from_rig is None:
            self.lost += 1
            self.prev = None
            self.get_logger().warn('추적 잃음', throttle_duration_sec=2.0)
            return
        self.ok += 1
        p = est.world_from_rig.pose
        cur = base_pose(self.base_from_cam, mat(p.rotation, p.translation))
        if self.prev is not None:
            self.publish_odom(c.header.stamp, body_twist(self.prev[1], cur, (ns - self.prev[0]) * 1e-9))
        self.prev = (ns, cur)
        if self.publish_map_tf:
            self.publish_map(c.header.stamp, slam if slam is not None else p)  # 루프 클로저 반영된 SLAM 위치 우선

    def publish_odom(self, stamp, v):
        """속도만 (자세는 안 씀 — cuVSLAM 좌표는 추적을 잃으면 새로 시작해 자세를 EKF에 그대로 넣으면 튐)"""
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, self.odom, self.base
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = v
        cov = [0.0] * 36
        cov[0] = cov[7] = self.lin_var
        cov[35] = self.ang_var
        cov[14] = cov[21] = cov[28] = 1e6  # 안 쓰는 축
        o.twist.covariance = cov
        self.odom_pub.publish(o)

    def publish_map(self, stamp, p):
        try:
            o = self.tf_buf.lookup_transform(self.odom, self.base, stamp).transform
        except Exception:  # noqa: BLE001 — 그 시각 odom이 없으면 이 프레임은 TF만 건너뜀 (추적은 계속)
            return
        odom_from_base = mat((o.rotation.x, o.rotation.y, o.rotation.z, o.rotation.w),
                             (o.translation.x, o.translation.y, o.translation.z))
        base = base_pose(self.base_from_cam, mat(p.rotation, p.translation))
        x, y, yaw = map_to_odom(base, odom_from_base)
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, self.map, self.odom
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf_pub.sendTransform(tf)
        ps = PoseStamped()
        ps.header = tf.header
        ps.pose.position.x, ps.pose.position.y = base[0], base[1]
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(base[2] / 2), math.cos(base[2] / 2)
        self.path.header.stamp = stamp
        self.path.poses.append(ps)
        del self.path.poses[:-3000]  # 최근 3000개(15Hz면 약 3분20초)만 — 계속 쌓이면 발행이 무거워짐
        if self.ok % 5 == 0:  # 경로는 3Hz 정도로만
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
