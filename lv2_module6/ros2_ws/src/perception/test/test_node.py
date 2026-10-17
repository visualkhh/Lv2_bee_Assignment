"""perception 기본 테스트: 영상이 없어도 노드가 떠서 빈 /tracked_objects와 진단을 낸다."""
import os
import sys
from pathlib import Path

import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

from perception_node import PerceptionNode  # noqa: E402
from perception.msg import TrackedObjectArray  # noqa: E402


def test_empty_without_model():
    os.environ.setdefault("ROS_DOMAIN_ID", "101")
    rclpy.init()
    try:
        node = PerceptionNode()
        got = {}

        def cb(msg):
            got["n"] = len(msg.objects)

        node.create_subscription(TrackedObjectArray, "/tracked_objects", cb, 5)
        from diagnostic_msgs.msg import DiagnosticArray

        diag = {}

        def dcb(msg):
            diag["names"] = [s.name for s in msg.status]

        node.create_subscription(DiagnosticArray, "/diagnostics", dcb, 1)
        # 파라미터 선언 확인 (test-bringup yaml 검사와 동일 항목)
        for p in ("rate_hz", "model_path", "confidence", "imgsz", "device"):
            assert node.has_parameter(p), p
        end = node.get_clock().now() + rclpy.duration.Duration(seconds=6)
        while node.get_clock().now() < end and ("n" not in got or "names" not in diag):
            rclpy.spin_once(node, timeout_sec=0.2)
        assert got.get("n") == 0
        assert diag.get("names") == ["perception: 상태"]
    finally:
        rclpy.shutdown()


def test_mission_qr():
    """QR 판이 보이는 영상을 넣으면 /mission_qr 로 문제 코드를 낸다 (MISSION.md §3-1)."""
    import cv2
    import numpy as np
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    os.environ.setdefault("ROS_DOMAIN_ID", "101")
    rclpy.init()
    try:
        node = PerceptionNode()
        q = cv2.QRCodeEncoder.create().encode("MISSION:STAND_UP")
        q = cv2.resize(q, (q.shape[1] * 6, q.shape[0] * 6), interpolation=cv2.INTER_NEAREST)
        img = np.full((480, 640, 3), 128, np.uint8)
        board = np.full((q.shape[0] + 40, q.shape[1] + 40), 255, np.uint8)
        board[20:-20, 20:-20] = q
        img[80:80 + board.shape[0], 180:180 + board.shape[1]] = board[:, :, None]
        msg = Image(height=480, width=640, encoding="bgr8", step=640 * 3, data=img.tobytes())
        # 추론에 필요한 깊이·내부파라미터·TF도 준다 (없으면 on_infer·to_map이 건너뜀)
        from sensor_msgs.msg import CameraInfo
        info = CameraInfo(width=640, height=480,
                          k=[500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0])
        node.on_info("pole_cam", info)
        depth = np.full((480, 640), 1000, np.uint16)  # 1m 평면 [mm]
        node.on_depth("pole_cam", Image(height=480, width=640, encoding="mono16", step=640 * 2,
                                        data=depth.tobytes()))
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
        from geometry_msgs.msg import TransformStamped
        tfb = StaticTransformBroadcaster(node)
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = "map", "pole_cam_color_optical_frame"
        t.transform.rotation.w = 1.0
        tfb.sendTransform(t)
        got = []
        node.create_subscription(String, "/mission_qr", lambda m: got.append(m.data),
                                 node.qr_pub.qos_profile)
        node.on_color("pole_cam", msg)
        end = node.get_clock().now() + rclpy.duration.Duration(seconds=4)
        while node.get_clock().now() < end and not got:
            rclpy.spin_once(node, timeout_sec=0.2)
        assert got == ["MISSION:STAND_UP"]
    finally:
        rclpy.shutdown()


def test_classify_color():
    """퍽 색 · 신호등 상태 (박스 안 HSV) — 조명 0.55~1.45배에서도 같은 답."""
    import numpy as np
    from perception_node import classify_color

    puck = {"red": (40, 40, 220), "green": (60, 180, 60), "blue": (200, 90, 30), "yellow": (40, 220, 230)}  # 시뮬 PALETTE(BGR)
    lamp_on = {"red": (40, 40, 220), "yellow": (40, 220, 230), "green": (60, 180, 60)}                      # 시뮬 TL_COLORS
    for gain in (0.55, 1.0, 1.45):
        for name, bgr in puck.items():
            roi = np.full((20, 20, 3), bgr, np.uint8)
            roi[:5] = (120, 120, 120)
            roi = np.clip(roi * gain, 0, 255).astype(np.uint8)
            assert classify_color("puck", roi) == name, (name, gain)
        for name, bgr in list(lamp_on.items()) + [("off", (12, 54, 57))]:
            roi = np.full((30, 14, 3), (40, 40, 40), np.uint8)
            roi[4:10, 4:10] = bgr
            roi = np.clip(roi * gain, 0, 255).astype(np.uint8)
            assert classify_color("traffic_light", roi) == name, (name, gain)
    assert classify_color("person", np.zeros((5, 5, 3), np.uint8)) == ""


def test_qr_crop():
    """멀리 있는 작은 QR 판(60px)도 박스만 잘라 확대하면 읽힌다 — 전체 화면으로는 못 읽음."""
    import cv2
    import numpy as np
    from perception_node import qr_crop

    q = cv2.QRCodeEncoder.create().encode("MISSION:STAND_UP")
    q = cv2.copyMakeBorder(q, 2, 2, 2, 2, cv2.BORDER_CONSTANT, value=255)
    small = cv2.resize(q, (60, 60), interpolation=cv2.INTER_AREA)
    img = np.full((480, 640, 3), 128, np.uint8)
    img[200:260, 300:360] = small[:, :, None]
    assert cv2.QRCodeDetector().detectAndDecode(qr_crop(img, (300, 200, 360, 260)))[0] == "MISSION:STAND_UP"
