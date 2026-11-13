"""Make the Isaac cameras look like the real RealSense topics (isaac_system.launch.py starts it).

The ROS side (ros2_ws packages) stays exactly as on the robot, so the differences are fixed here:

1. Depth: Isaac publishes 32FC1 [m] on aligned_depth_to_color/image_isaac. The RealSense driver publishes
   16UC1 [mm] on aligned_depth_to_color/image_raw, which cuvslam_slam reads as uint16. This node converts
   (NaN/inf/out of range -> 0 = no measurement) and keeps the header (the depth frame stamp).
2. CameraInfo: Isaac 6.1 runs its ROS bridge on the Jazzy backend over Lyrical libraries. Image messages
   arrive intact, but CameraInfo arrives with every float64 array shifted by one element (k[0] = fx lost),
   and cuVSLAM stops with "Focal length must be > 0.0". The Isaac helper publishes color/camera_info_isaac
   (kept for checking); this node publishes color/camera_info stamped with each color frame and
   aligned_depth_to_color/camera_info (same intrinsics) stamped with each depth frame. The stamps matter:
   nvblox pairs depth with camera_info by exact stamp, and Isaac can render color and depth on different ticks.

3. Depth noise, BEE_SIM_NOISE (default 1, 0 = off, same switch as the motion blur in isaac_startup.py): stereo
   error in the disparity domain like the D435 (baseline 50 mm, fx 383 px): disparity d = fx*B/z gets Gaussian noise of
   0.04 px * BEE_SIM_NOISE (half the ~0.08 px subpixel RMS of the D4 spec - unmeasured, so weaker than the real camera:
   2 mm at 1 m, 8 mm at 2 m) and is rounded to 1/32 px (D4 subpixel step); depth closer than 0.17 m (D435 min-Z,
   spec value, not scaled) and a random 0.5 % * BEE_SIM_NOISE of the pixels become 0. Fresh noise every frame.

ponytail: intrinsics are fixed here (640x480, fx = fy = 615.37 = Isaac pinhole of the D435 camera prims);
if a camera prim's resolution or focal length changes, update the parameters (or take k from
color/camera_info_isaac once it arrives intact).
"""
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


NOISE = float(os.environ.get('BEE_SIM_NOISE', '1'))
FB = 383.0 * 0.050          # D435 depth fx [px] * baseline [m]
SUBPIX_SIGMA = 0.04         # [px] half of the D4 ~0.08 px RMS
SUBPIX_STEP = 1.0 / 32.0    # [px]
MIN_Z = 0.17                # [m] D435 640x480 min-Z
DROPOUT = 0.005             # fraction of pixels at BEE_SIM_NOISE=1


def stereo_noise(z, scale, rng=np.random.default_rng()):
    """Depth [m] (NaN/inf/<=0 pass as invalid) -> the same with D435-like disparity noise, step, min range, dropout."""
    with np.errstate(divide='ignore', invalid='ignore'):
        d = FB / z
        d = d + rng.standard_normal(z.shape).astype(np.float32) * (SUBPIX_SIGMA * scale)
        d = np.round(d / SUBPIX_STEP) * SUBPIX_STEP
        z = np.where((d > 0) & np.isfinite(z) & (z > 0), FB / d, np.nan).astype(np.float32)  # no depth stays no depth
    z[z < MIN_Z] = np.nan
    z[rng.random(z.shape) < DROPOUT * scale] = np.nan
    return z


def to_16uc1(m, noise=0.0):
    """32FC1 [m] -> 16UC1 [mm], row padding dropped (noise > 0: stereo_noise first). Other encodings are returned unchanged."""
    if m.encoding != '32FC1':
        return m
    z = np.frombuffer(bytes(m.data), np.float32).reshape(m.height, m.step // 4)[:, :m.width]
    if noise > 0:
        z = stereo_noise(z, noise)
    d = np.nan_to_num(z * 1000.0, nan=0.0, posinf=0.0, neginf=0.0)
    d[(d < 0) | (d > 65535)] = 0
    d = d.astype(np.uint16)
    return Image(header=m.header, height=m.height, width=m.width, encoding='16UC1',
                 is_bigendian=0, step=m.width * 2, data=d.tobytes())


class CameraAdapter(Node):
    def __init__(self):
        super().__init__('isaac_camera_adapter')
        cams = self.declare_parameter('cameras', ['pole_front_cam', 'gripper_cam']).value
        w = self.declare_parameter('width', 640).value
        h = self.declare_parameter('height', 480).value
        f = self.declare_parameter('focal', 615.3728238119883).value
        cx, cy = (w / 2.0, h / 2.0)
        self.info = CameraInfo(width=w, height=h, distortion_model='plumb_bob', d=[0.0] * 5,
                               k=[f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0],
                               r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                               p=[f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0])
        qos = qos_profile_sensor_data
        self.subs = []
        for cam in cams:
            # each stream gets its own camera_info stamped with its own frame (like the RealSense driver):
            # nvblox pairs depth + camera_info by exact stamp, and Isaac does not always render color and depth on the same tick
            for stream, topic in (('color', 'image_raw'), ('aligned_depth_to_color', 'image_isaac')):
                info = self.create_publisher(CameraInfo, f'/{cam}/{stream}/camera_info', qos)
                image = self.create_publisher(Image, f'/{cam}/{stream}/image_raw', qos) if stream != 'color' else None
                self.subs.append(self.create_subscription(
                    Image, f'/{cam}/{stream}/{topic}', lambda m, i=info, o=image: self.on_image(m, i, o), qos))

    def on_image(self, m, info_pub, image_pub):
        if image_pub is not None:  # depth: convert and republish under the RealSense name
            image_pub.publish(to_16uc1(m, NOISE))
        self.info.header = m.header
        info_pub.publish(self.info)


def main():
    rclpy.init()
    rclpy.spin(CameraAdapter())


if __name__ == '__main__':
    a = np.array([[1.5, np.nan], [np.inf, 0.3]], np.float32)
    out = to_16uc1(Image(height=2, width=2, encoding='32FC1', step=8, data=a.tobytes()))
    assert out.encoding == '16UC1' and np.frombuffer(bytes(out.data), np.uint16).tolist() == [1500, 0, 0, 300]
    for zm, want_mm in ((1.0, 2.1), (2.0, 8.4)):  # sigma_z = z^2 * sigma_d / (fx*B)
        n = stereo_noise(np.full((300, 300), zm, np.float32), 1.0, np.random.default_rng(0))
        v = n[np.isfinite(n)]
        assert abs(v.std() * 1000 - want_mm) < 0.25 * want_mm and abs(v.mean() - zm) < 0.002, (zm, v.std(), v.mean())
        assert 0.002 < 1 - v.size / n.size < 0.01  # ~0.5 % dropout
    assert np.isnan(stereo_noise(np.full((4, 4), 0.1, np.float32), 1.0)).all()  # closer than min-Z -> no depth
    assert np.isnan(stereo_noise(np.array([[np.inf, np.nan, 0.0]], np.float32), 1.0)).all()  # invalid stays invalid
    print(f'[camera_adapter] depth noise BEE_SIM_NOISE={NOISE:g}' + ('' if NOISE > 0 else ' (off)'), flush=True)
    main()
