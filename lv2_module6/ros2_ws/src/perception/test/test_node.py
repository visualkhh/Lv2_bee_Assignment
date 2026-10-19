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
        for p in ("rate_hz", "model_path", "confidence", "imgsz", "device", "sync_slop", "match_dist",
                  "pixel_sigma", "depth_sigma0", "depth_sigma_k", "process_noise", "process_noise_dynamic",
                  "self_filter", "base_frame", "self_links", "self_radius", "body_radius", "body_height", "track_frame",
                  "self_arm_hits"):
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
        depth_msg = Image(height=480, width=640, encoding="mono16", step=640 * 2, data=depth.tobytes())
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
        from geometry_msgs.msg import TransformStamped
        tfb = StaticTransformBroadcaster(node)
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = "odom", "pole_cam_color_optical_frame"  # 추적 좌표계 (track_frame)
        t.transform.rotation.w = 1.0
        tfb.sendTransform(t)
        got = []
        node.create_subscription(String, "/mission_qr", lambda m: got.append(m.data),
                                 node.qr_pub.qos_profile)
        node.on_frame("pole_cam", msg, depth_msg)  # 동기화된 컬러·depth 쌍
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

    puck = {"red": (40, 40, 220), "green": (60, 180, 60), "blue": (200, 90, 30), "yellow": (40, 220, 230),
            "orange": (30, 140, 250), "purple": (170, 60, 140), "white": (235, 235, 235), "black": (40, 40, 40)}  # 시뮬 PALETTE(BGR)
    lamp_on = {"red": (40, 40, 220), "yellow": (40, 220, 230), "green": (60, 180, 60)}                      # 시뮬 TL_COLORS
    for gain in (0.55, 1.0, 1.45):
        for name, bgr in puck.items():
            roi = np.full((20, 20, 3), bgr, np.uint8)
            roi[:3] = (60, 140, 190)  # 박스 위쪽에 섞인 바닥(모래색) — 가운데만 보므로 무시돼야 함
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


def _tf(parent, child, sec, x=0.0, rot=(0.0, 0.0, 0.0, 1.0)):
    from geometry_msgs.msg import TransformStamped
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.header.stamp.sec = sec
    t.transform.translation.x = x
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = rot
    return t


def test_tf_at_image_stamp():
    """움직이는 gripper_cam: TF를 영상 시각으로 찾는다 (최신 TF가 아니라). 그 시각이 없으면 최신으로 대신."""
    from builtin_interfaces.msg import Time

    os.environ.setdefault("ROS_DOMAIN_ID", "101")
    rclpy.init()
    try:
        node = PerceptionNode()
        for sec, x in ((100, 1.0), (101, 2.0), (102, 3.0)):  # 팔이 움직여 카메라가 1m → 3m
            node.tf.set_transform(_tf("odom", "gripper_cam_color_optical_frame", sec, x), "test")
        rot, trans = node.lookup_tf("gripper_cam_color_optical_frame", Time(sec=101))
        assert abs(trans[0] - 2.0) < 1e-6  # 찍힌 순간(101초) 자세 — 최신(3m)이 아님
        assert node.tf_fallback == 0
        rot, trans = node.lookup_tf("gripper_cam_color_optical_frame", Time(sec=200))  # 버퍼보다 미래
        assert abs(trans[0] - 3.0) < 1e-6 and node.tf_fallback == 1
    finally:
        rclpy.shutdown()


def test_fuse_two_cameras():
    """같은 퍽을 두 카메라가 직각 방향에서 보면 한 트랙으로 합쳐지고, 각 카메라의 depth 축 오차가 서로 줄어든다.
    같은 프레임의 가까운 두 퍽은 한 트랙으로 뭉치지 않는다 (1:1 배정)."""
    import numpy as np
    from builtin_interfaces.msg import Time
    from perception_node import meas_cov_cam

    os.environ.setdefault("ROS_DOMAIN_ID", "101")
    rclpy.init()
    try:
        node = PerceptionNode()
        r_cam = meas_cov_cam(2.0, 500.0, 500.0, node.pixel_sigma, node.depth_sigma0, node.depth_sigma_k)
        # pole_cam: depth 축 = map x · gripper_cam: depth 축 = map y (카메라→map 회전)
        rot_pole = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)
        rot_grip = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)
        truth = np.array([1.0, 1.0, 0.02])

        def det(z, rot):
            return {"class_name": "puck", "z": np.asarray(z, float), "R": rot @ r_cam @ rot.T,
                    "size": np.array([0.06, 0.06, 0.04]), "conf": 0.9, "px": (320, 240), "color": "red"}

        a = node.fuse("pole_cam", Time(sec=10), [det(truth + [0.05, 0.0, 0.0], rot_pole)])     # depth 축(x)으로 5cm 틀림
        b = node.fuse("gripper_cam", Time(sec=10), [det(truth + [0.0, -0.05, 0.0], rot_grip)])  # 같은 순간 · depth 축(y)으로 5cm 틀림
        assert a == b and len(a) == 1  # 두 카메라 검출이 같은 트랙 id로 (/perception/<cam>/detections의 id)
        assert len(node.tracks) == 1
        tr = next(iter(node.tracks.values()))
        assert set(tr["seen"]) == {"pole_cam", "gripper_cam"} and tr["camera"] == "gripper_cam"
        # 한 카메라만 믿었으면 5cm — 융합하면 상대 카메라의 정확한 가로 방향이 depth 축 오차를 잡아 준다
        assert np.linalg.norm(tr["p"] - truth) < 0.02, tr["p"]
        sz_one = r_cam[2, 2]
        assert tr["P"][0, 0] < sz_one * 0.2 and tr["P"][1, 1] < sz_one * 0.2

        # 같은 프레임에서 0.2m 떨어진 두 퍽 → 둘 다 남는다 (기존 트랙 하나에 둘 다 붙지 않음)
        ids = node.fuse("pole_cam", Time(sec=11), [det(truth, rot_pole), det(truth + [0.0, 0.2, 0.0], rot_pole)])
        assert len(node.tracks) == 2 and ids[0] == a[0] and ids[1] != a[0]
        # 다른 클래스는 가까워도 합치지 않음
        d = det(truth, rot_pole)
        d["class_name"] = "obstacle"
        node.fuse("pole_cam", Time(sec=11), [d])
        assert len(node.tracks) == 3
    finally:
        rclpy.shutdown()


def test_self_filter():
    """pole_cam이 앞으로 뻗은 팔을 obstacle로 잡아도 버린다 · 차체 위도 버린다 · 손끝 바로 아래 퍽은 남긴다 · 떨어진 물체는 남긴다"""
    import numpy as np
    from perception_node import is_self

    base = np.array([2.35, 0.0, 0.0])
    chain = [np.array(v) for v in ([2.40, 0.0, 0.19], [2.42, -0.02, 0.32], [2.52, -0.04, 0.33], [2.58, -0.05, 0.31],
                                   [2.65, -0.04, 0.29], [2.75, -0.05, 0.26])]  # shoulder … end_effector (실측 자세)
    body = (base, chain)
    args = (0.2, 0.6, 0.06)
    assert is_self("obstacle", np.array([2.62, -0.03, 0.31]), body, *args)      # 그리퍼를 장애물로
    assert is_self("obstacle", np.array([2.30, 0.05, 0.25]), body, *args)       # 차체·적재함 위
    assert not is_self("puck", np.array([2.72, -0.05, 0.22]), body, *args)      # 손가락 사이 아래 퍽 — 남김
    assert is_self("obstacle", np.array([2.72, -0.05, 0.22]), body, *args)      # 같은 자리라도 퍽이 아니면 손가락
    assert not is_self("puck", np.array([2.75, 0.30, 0.02]), body, *args)       # 옆 바닥 퍽
    assert not is_self("puck", np.array([2.54, 0.0, 0.02]), body, *args)        # 범퍼 바로 앞 바닥 퍽 (차체 반경 안이지만 바닥)
    assert not is_self("obstacle", np.array([3.20, 0.0, 0.10]), body, *args)    # 앞 장애물



def test_track_in_odom_publish_in_map():
    """추적은 odom, 발행은 그 순간의 map ← odom: RTAB-Map이 map→odom을 고치면 저장된 물체도 같이 옮겨짐 (로봇과 같은 보정)"""
    import numpy as np
    from builtin_interfaces.msg import Time

    os.environ.setdefault("ROS_DOMAIN_ID", "101")
    rclpy.init()
    try:
        node = PerceptionNode()
        node.tracks[1] = {"class_name": "puck", "p": np.array([1.0, 0.0, 0.02]), "P": np.diag([1e-4, 4e-4, 1e-4]),
                          "t": 0.0, "size": np.array([0.06, 0.06, 0.04]), "conf": 0.9, "last_seen": Time(),
                          "seen": {"pole_cam": 0.0}, "camera": "pole_cam", "px": (1, 1), "color": "red"}
        # map ← odom: 0.2m 옆 · 90° 회전 (RTAB-Map 보정)
        node.tf.set_transform(_tf("map", "odom", 0, 0.2, (0.0, 0.0, 0.7071068, 0.7071068)), "test")
        m = node.to_map(node.tracks[1])
        assert np.allclose(m["p"], [0.2, 1.0, 0.02], atol=1e-6)            # odom (1,0) → 90° 돌려 (0,1) + 0.2 옆
        assert np.allclose(np.diag(m["P"])[:2], [4e-4, 1e-4], atol=1e-9)    # 공분산 축도 같이 돔
        assert np.allclose(node.tracks[1]["p"], [1.0, 0.0, 0.02])           # 추적값(odom)은 그대로
        # 보정이 바뀌면 다음 발행에 그대로 반영
        node.tf.set_transform(_tf("map", "odom", 1, 0.0), "test")
        assert np.allclose(node.to_map(node.tracks[1])["p"], [1.0, 0.0, 0.02], atol=1e-6)
    finally:
        rclpy.shutdown()


def test_arm_in_box():
    """팔을 화면에 투영해 박스 안에 있으면 자기 팔 — 박스 일부만 팔이라 depth가 뒤 바닥을 가리켜도"""
    import numpy as np
    from perception_node import arm_pixels, box_has_arm

    # 카메라 = 원점, 광축 +z (rot = I) · 팔이 카메라 앞 1m에서 왼쪽→오른쪽으로 0.4m
    chain = [np.array([-0.2, 0.1, 1.0]), np.array([0.2, 0.1, 1.0])]
    px = arm_pixels(chain, np.eye(3), np.zeros(3), 500.0, 500.0, 320.0, 240.0)
    assert len(px) == 10 and np.allclose(px[0], [220.0, 290.0, 20.0]) and np.allclose(px[-1], [420.0, 290.0, 20.0])  # 두께 0.04m → 20px
    assert box_has_arm((200, 270, 300, 310), px, 3)        # 팔 왼쪽 일부를 덮은 박스
    assert box_has_arm((200, 240, 300, 275), px, 3)        # 중심선(v 290)은 밖이지만 팔 두께(±20px) 안 — 화면에 보이는 몸통
    assert not box_has_arm((200, 100, 300, 200), px, 3)    # 팔 위쪽 — 다른 물체
    assert not box_has_arm((435, 280, 460, 300), px, 3)    # 끝 한 점만 — 기준 미만
    behind = arm_pixels([np.array([0, 0, -1.0]), np.array([0.1, 0, -1.0])], np.eye(3), np.zeros(3), 500, 500, 320, 240)
    assert len(behind) == 0                                # 카메라 뒤는 안 찍힘
