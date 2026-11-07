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
                  "self_arm_hits", "miss_count", "miss_max_range", "miss_edge_px", "miss_occlusion_m",
                  "puck_colors", "hue_tol", "check_model_color", "size_long_min_cm", "size_long_max_cm",
                  "size_short_min_cm", "square_long_cm", "square_ratio", "stand_ratio", "edge_px", "near_px",
                  "depth_min_valid", "remember_min_hits"):
            assert node.has_parameter(p), p
        end = node.get_clock().now() + rclpy.duration.Duration(seconds=6)
        while node.get_clock().now() < end and ("n" not in got or "names" not in diag):
            rclpy.spin_once(node, timeout_sec=0.2)
        assert got.get("n") == 0
        assert diag.get("names") == ["perception: 상태"]
    finally:
        rclpy.shutdown()


def test_mission_qr():
    """QR 판이 보이는 영상을 넣으면 /mission/qr 로 문제 코드를 낸다 (documents/mission/README.md)."""
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
        node.on_info("pole_front_cam", info)
        depth = np.full((480, 640), 1000, np.uint16)  # 1m 평면 [mm]
        depth_msg = Image(height=480, width=640, encoding="mono16", step=640 * 2, data=depth.tobytes())
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
        from geometry_msgs.msg import TransformStamped
        tfb = StaticTransformBroadcaster(node)
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = "odom", "pole_front_cam_color_optical_frame"  # 추적 좌표계 (track_frame)
        t.transform.rotation.w = 1.0
        tfb.sendTransform(t)
        got = []
        node.create_subscription(String, "/mission/qr", lambda m: got.append(m.data),
                                 node.qr_pub.qos_profile)
        node.on_frame("pole_front_cam", msg, depth_msg)  # 동기화된 컬러·depth 쌍
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


def test_puck_color_real_green():
    """실물 초록 퍽 색(11/03 실측 H 64~85, RGB (19,128,71) ≈ H74)이 초록으로 · 빨강 퍽은 빨강 · 주황 컵홀더(H12~13)는 other"""
    import cv2
    import numpy as np
    from perception_node import puck_color

    def roi_h(h):  # H만 바꾼 진한 색 조각 (S 200 · V 150)
        return cv2.cvtColor(np.full((20, 20, 3), (h, 200, 150), np.uint8), cv2.COLOR_HSV2BGR)

    real_green = np.full((20, 20, 3), (71, 128, 19), np.uint8)  # 실물 초록 퍽 BGR (real_colors_1029)
    assert puck_color(real_green, ["red", "green"], 10)[0] == "green"
    for h in (64, 71, 77, 85, 90):
        assert puck_color(roi_h(h), ["red", "green"], 10)[0] == "green", h
    assert puck_color(roi_h(2), ["red", "green"], 10)[0] == "red"
    assert puck_color(roi_h(13), ["red", "green"], 10)[0] == "other"   # 주황 컵홀더
    assert puck_color(roi_h(110), ["red", "green"], 10)[0] == "other"  # 파랑


def test_settle_puck_color():
    """노드 색 확인은 분명히 다른 색일 때만 버림 (11/03) — 애매·어두우면 YOLO 색 · HSV가 다른 퍽 색이면 HSV 색"""
    import cv2
    import numpy as np
    from perception_node import puck_color, settle_puck_color

    def roi_hsv(h, s=200, v=150):
        return cv2.cvtColor(np.full((20, 20, 3), (h, s, v), np.uint8), cv2.COLOR_HSV2BGR)

    pc = ["red", "green"]

    def settle(roi, model):
        return settle_puck_color(roi, puck_color(roi, pc, 10)[0], model, pc)

    assert settle(roi_hsv(77), "green") == ("green", "")            # 실물 초록
    assert settle(roi_hsv(77), "red") == ("green", "")              # YOLO 색과 달라도 HSV 색 (버리지 않음)
    assert settle(roi_hsv(13), "red")[0] is None                    # 주황 컵홀더 → 다른 색(orange)
    assert settle(roi_hsv(110), "green")[0] is None                 # 파랑
    assert settle(roi_hsv(0, 40, 40), "red") == ("red", "")         # 어두워 색 화소 없음 → YOLO 색
    assert settle(roi_hsv(0, 40, 40), "")[0] is None                # YOLO 색도 없으면 버림
    assert settle_puck_color(roi_hsv(77), "green", "red", pc, use_model_color=False) == ("green", "")


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
        # pole_front_cam: depth 축 = map x · gripper_cam: depth 축 = map y (카메라→map 회전)
        rot_pole = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)
        rot_grip = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)
        truth = np.array([1.0, 1.0, 0.02])

        def det(z, rot):
            return {"class_name": "puck", "z": np.asarray(z, float), "R": rot @ r_cam @ rot.T,
                    "size": np.array([0.06, 0.06, 0.04]), "conf": 0.9, "px": (320, 240), "color": "red"}

        a = node.fuse("pole_front_cam", Time(sec=10), [det(truth + [0.05, 0.0, 0.0], rot_pole)])     # depth 축(x)으로 5cm 틀림
        b = node.fuse("gripper_cam", Time(sec=10), [det(truth + [0.0, -0.05, 0.0], rot_grip)])  # 같은 순간 · depth 축(y)으로 5cm 틀림
        assert a == b and len(a) == 1  # 두 카메라 검출이 같은 트랙 id로 (/perception/<cam>/detections의 id)
        assert len(node.tracks) == 1
        tr = next(iter(node.tracks.values()))
        assert set(tr["seen"]) == {"pole_front_cam", "gripper_cam"} and tr["camera"] == "gripper_cam"
        from perception_node import to_msg
        assert list(to_msg(a[0], tr).cameras) == ["pole_front_cam", "gripper_cam"]  # 끝 = 마지막(px) 카메라
        # 한 카메라만 믿었으면 5cm — 융합하면 상대 카메라의 정확한 가로 방향이 depth 축 오차를 잡아 준다
        assert np.linalg.norm(tr["p"] - truth) < 0.02, tr["p"]
        sz_one = r_cam[2, 2]
        assert tr["P"][0, 0] < sz_one * 0.2 and tr["P"][1, 1] < sz_one * 0.2

        # 같은 프레임에서 0.2m 떨어진 두 퍽 → 둘 다 남는다 (기존 트랙 하나에 둘 다 붙지 않음)
        ids = node.fuse("pole_front_cam", Time(sec=11), [det(truth, rot_pole), det(truth + [0.0, 0.2, 0.0], rot_pole)])
        assert len(node.tracks) == 2 and ids[0] == a[0] and ids[1] != a[0]
        # 다른 클래스는 가까워도 합치지 않음
        d = det(truth, rot_pole)
        d["class_name"] = "obstacle"
        node.fuse("pole_front_cam", Time(sec=11), [d])
        assert len(node.tracks) == 3
    finally:
        rclpy.shutdown()


def test_self_filter():
    """pole_front_cam이 앞으로 뻗은 팔을 obstacle로 잡아도 버린다 · 차체 위도 버린다 · 손끝 바로 아래 퍽은 남긴다 · 떨어진 물체는 남긴다"""
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
    # body_box (차체 직사각형): 상자 위 차체 앞 퍽은 남기고 적재함 퍽은 버림 · 차체가 돌아 있어도 base_frame 기준
    bb = [-0.25, 0.08, 0.15]
    assert is_self("puck", np.array([2.54, 0.0, 0.15]), body, *args)                          # 반경 0.2 — 차체 앞 19 cm 상자 위 퍽도 지움
    assert not is_self("puck", np.array([2.54, 0.0, 0.15]), (*body, np.eye(3)), *args, bb)    # 직사각형 — 남김
    assert is_self("puck", np.array([2.25, 0.03, 0.12]), (*body, np.eye(3)), *args, bb)       # 적재함 안 퍽
    yaw90 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    assert not is_self("puck", np.array([2.35, 0.19, 0.15]), (base, [], yaw90), *args, bb)    # 돌아 있는 차체 앞
    assert is_self("puck", np.array([2.35, -0.19, 0.15]), (base, [], yaw90), *args, bb)       # 돌아 있는 차체 뒤 (적재함)



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
                          "seen": {"pole_front_cam": 0.0}, "camera": "pole_front_cam", "px": (1, 1), "color": "red"}
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


# ---------- 기억 · 퍽 계열 합치기 (팀 확인 10/22) — 시나리오는 test/memory_check.py와 같음
FX_T = FY_T = 606.0
CX_T, CY_T, W_T, H_T = 323.0, 241.0, 640, 480


def _cam_rot(yaw_deg, pitch_deg=18.0):
    """yaw만큼 돌고 pitch만큼 아래를 보는 카메라(pole_front_cam URDF 18°)의 카메라 → odom 회전"""
    import math

    import numpy as np
    a, p = math.radians(yaw_deg), math.radians(pitch_deg)
    fwd = np.array([math.cos(p) * math.cos(a), math.cos(p) * math.sin(a), -math.sin(p)])  # 광학 z (앞)
    right = np.array([math.sin(a), -math.cos(a), 0.0])                                    # 광학 x (오른쪽)
    return np.stack([right, np.cross(fwd, right), fwd], axis=1)                           # 광학 y = z × x (아래)


def _view(yaw_deg, depth=None):
    """prune에 넘기는 그 순간 카메라 (rot, trans, fx, fy, cx, cy, depth_m) — 깊이 없으면 '뒤 배경 5m'"""
    import numpy as np
    d = depth if depth is not None else np.full((H_T, W_T), 5.0, np.float32)
    return _cam_rot(yaw_deg), np.array([0.0, 0.0, 0.43]), FX_T, FY_T, CX_T, CY_T, d


def _seen(p, v):
    """점 p가 그 카메라 화면 안인가 (가짜 검출을 만들지)"""
    import numpy as np
    pc = v[0].T @ (np.asarray(p, float) - v[1])
    if pc[2] < 0.1:
        return False
    u, vv = FX_T * pc[0] / pc[2] + CX_T, FY_T * pc[1] / pc[2] + CY_T
    return 0 <= u < W_T and 0 <= vv < H_T


def _det(p, cls="puck", color="red", posture=None):
    """fuse에 넣는 가짜 검출 (update_tracks가 만드는 것과 같은 칸)"""
    import numpy as np
    if posture is None:
        posture = "lying" if cls == "puck_lying" else "standing" if cls == "puck" else ""
    return {"class_name": cls, "z": np.asarray(p, float), "R": np.eye(3) * 1e-4, "size": np.array([0.06, 0.06, 0.04]),
            "conf": 0.9, "px": (1, 1), "color": color, "posture": posture}


def _step(node, t, v, objs):
    """한 프레임: 화면 안 물체만 검출로 넣고 fuse → prune (update_tracks와 같은 순서)"""
    from builtin_interfaces.msg import Time
    dets = [_det(**o) for o in objs if _seen(o["p"], v)]
    ids = node.fuse("pole_front_cam", Time(sec=int(t), nanosec=int(round((t % 1) * 1e9))), dets)
    node.prune(t, set(ids), v)
    return ids


def _names(node):
    from perception_node import to_msg
    return [to_msg(tid, tr).class_name for tid, tr in sorted(node.tracks.items())]


def test_remember_out_of_view():
    """퍽은 화면 밖으로 나가도 기억(같은 id로 다시 이어짐) · 사람은 2초 뒤 삭제 ·
    그 자리가 잘 보이는데 3번 연속 없으면(집어 감) 삭제 — 1·2번째엔 아직 남음
    (기억은 같은 자리에서 remember_min_hits(3)번 잡힌 뒤부터 — 처음 3프레임을 보고 시작)"""
    os.environ.setdefault("ROS_DOMAIN_ID", "101")
    rclpy.init()
    try:
        node = PerceptionNode()
        node.long_memory = True  # 긴 기억 규칙 시험 (11/04부터 기본은 끔 — 기억은 통합 쪽)
        puck = {"p": (1.0, 0.0, 0.03)}
        person = {"p": (1.5, 0.3, 0.9), "cls": "person", "color": ""}
        for t in (0.0, 0.1, 0.2):  # 처음 3프레임 — 확인(기억 대상)
            _step(node, t, _view(0), [puck, person])
        first = min(node.tracks)
        for k in range(1, 26):  # 왼쪽으로 90° 돌아 둘 다 화면 밖 · 0.2초마다 5초
            _step(node, 0.2 + 0.2 * k, _view(90), [puck, person])
        assert _names(node) == ["puck"]
        assert _step(node, 5.4, _view(0), [puck]) == [first] and len(node.tracks) == 1  # 다시 돌아봄 — 같은 id
        alive = []
        for k in range(3):  # 누가 집어 감
            _step(node, 5.6 + 0.2 * k, _view(0), [])
            alive.append(len(node.tracks))
        assert alive == [1, 1, 0]
    finally:
        rclpy.shutdown()


def test_no_long_memory_default():
    """(11/04) 기본값 long_memory False — 기억은 통합(판단) 쪽이 맡음: 퍽도 사람처럼 2초 넘게 안 보이면 지움
    · 보이는 동안·잠깐 놓쳤다 다시 보이면(2초 안) 같은 id (짧은 추적은 그대로)"""
    os.environ.setdefault("ROS_DOMAIN_ID", "101")
    rclpy.init()
    try:
        node = PerceptionNode()
        assert node.long_memory is False
        puck = {"p": (1.0, 0.0, 0.03)}
        for t in (0.0, 0.1, 0.2, 0.3):
            _step(node, t, _view(0), [puck])
        first = min(node.tracks)
        _step(node, 0.8, _view(90), [puck])                     # 잠깐 안 보임 (0.5초)
        assert _step(node, 1.0, _view(0), [puck]) == [first]     # 2초 안에 다시 → 같은 id
        for k in range(1, 16):  # 화면 밖 3초
            _step(node, 1.0 + 0.2 * k, _view(90), [puck])
        assert _names(node) == []                                # 기억 안 함 → 지움
    finally:
        rclpy.shutdown()


def test_remember_not_counted():
    """못 봤다고 세지 않는 경우 — 앞이 가려짐(그 자리 깊이가 가까움) · 3m 넘게 멂 · 깊이 없음(너무 가까움) → 계속 기억
    (3프레임 보고 확인한 뒤 3초 — 확인 전이었다면 2초 규칙으로 지워졌을 시간)"""
    import numpy as np

    os.environ.setdefault("ROS_DOMAIN_ID", "101")
    rclpy.init()
    try:
        node = PerceptionNode()
        node.long_memory = True  # 긴 기억 규칙 시험 (11/04부터 기본은 끔 — 기억은 통합 쪽)
        for p, depth in (((1.0, 0.0, 0.03), np.full((H_T, W_T), 0.5, np.float32)),   # 0.5m 앞에 무언가
                         ((3.5, 0.0, 0.03), None),                                    # 너무 멂
                         ((1.0, 0.0, 0.03), np.zeros((H_T, W_T), np.float32))):       # 깊이 0
            node.tracks.clear()
            for t in (0.0, 0.1, 0.2):
                _step(node, t, _view(0), [{"p": p}])
            for k in range(1, 16):
                _step(node, 0.2 + 0.2 * k, _view(0, depth), [])
            assert len(node.tracks) == 1, p
    finally:
        rclpy.shutdown()


def test_ghost_not_remembered():
    """유령 퍽 — 카메라 자세(TF)가 영상과 잠깐 어긋나 한 번만 엉뚱한 자리로 찍힌 퍽은 기억하지 않음 (2초 규칙으로 삭제).
    같은 때 3번 이상 잡힌 진짜 퍽은 기억 (10/22 시뮬레이터: 같은 초록 퍽이 9곳에 남았던 것)"""
    os.environ.setdefault("ROS_DOMAIN_ID", "101")
    rclpy.init()
    try:
        node = PerceptionNode()
        node.long_memory = True  # 긴 기억 규칙 시험 (11/04부터 기본은 끔 — 기억은 통합 쪽)
        real = {"p": (1.0, 0.0, 0.03)}
        ghost = {"p": (1.0, 0.6, 0.03)}  # TF가 틀린 한 프레임에만 찍힌 자리
        for t in (0.0, 0.1, 0.2):
            _step(node, t, _view(0), [real])
        _step(node, 0.3, _view(30), [ghost])  # 한 번만
        assert len(node.tracks) == 2
        for k in range(1, 16):  # 둘 다 화면 밖으로 돌아 3초
            _step(node, 0.3 + 0.2 * k, _view(150), [])
        assert [tuple(tr["p"]) for tr in node.tracks.values()] == [real["p"]]
    finally:
        rclpy.shutdown()


def test_puck_family_fuse():
    """퍽 계열 합치기 — 누움(pole) + 자세 모름(gripper)은 id 하나 · 발행 puck_lying /
    붙은 다른 색 퍽은 둘 / 색 못 정한 관측은 같은 퍽에 붙고 아는 색을 덮지 않음 / 신호등은 색(상태)이 바뀌어도 같은 물체"""
    from builtin_interfaces.msg import Time

    os.environ.setdefault("ROS_DOMAIN_ID", "101")
    rclpy.init()
    try:
        node = PerceptionNode()
        a = node.fuse("pole_front_cam", Time(), [_det((1.0, 0.0, 0.03), "puck_lying")])
        b = node.fuse("gripper_cam", Time(), [_det((1.01, 0.0, 0.03), "puck", posture="")])
        assert a == b and _names(node) == ["puck_lying"]
        node.tracks.clear()
        ids = node.fuse("pole_front_cam", Time(), [_det((1.0, 0.0, 0.03), color="red"),
                                                    _det((1.0, 0.05, 0.03), color="blue")])
        assert len(set(ids)) == 2
        node.tracks.clear()
        a = node.fuse("pole_front_cam", Time(), [_det((1.0, 0.0, 0.03), color="red")])
        b = node.fuse("pole_front_cam", Time(sec=1), [_det((1.0, 0.0, 0.03), color="unknown")])
        assert a == b and node.tracks[a[0]]["color"] == "red"
        node.tracks.clear()
        a = node.fuse("pole_front_cam", Time(), [_det((2.0, 0.0, 0.4), "traffic_light", "red")])
        b = node.fuse("pole_front_cam", Time(sec=1), [_det((2.0, 0.0, 0.4), "traffic_light", "green")])
        assert a == b and node.tracks[a[0]]["color"] == "green"
    finally:
        rclpy.shutdown()


# ---------- 퍽 판단 · 클래스 이름 (test/puck_tag_check.py 실물 기준을 옮김)
def test_split_class():
    """모델 이름 → 설계 이름 + 이름에 든 색 (새 모델 색별 클래스 · 팀 모델 둘 다)"""
    from perception_node import split_class
    assert split_class("puck_red") == ("puck", "red")
    assert split_class("puck_lying") == ("puck", "")
    assert split_class("puck") == ("puck", "")
    assert split_class("traffic_light_off") == ("traffic_light", "off")
    assert split_class("traffic_light") == ("traffic_light", "")
    assert split_class("person") == ("person", "")


class _Boxes:
    """ultralytics 결과 박스 흉내 — update_tracks가 쓰는 b.cls[0] · b.xyxy[0].tolist() · b.conf[0]"""

    class _Xyxy:
        def __init__(self, v):
            self.v = v

        def __getitem__(self, i):
            return self

        def tolist(self):
            return list(self.v)

    def __init__(self, c, xyxy):
        self.cls, self.xyxy, self.conf = [c], self._Xyxy(xyxy), [0.9]


def test_puck_judge_pipeline():
    """가짜 영상 + 가짜 YOLO 박스 → update_tracks → 트랙 (1m 앞, fx 606: 3cm ≈ 18px)
    서있는 퍽 puck · 누운 퍽 puck_lying · 주황(색) · YOLO 색과 HSV 색 다름 · 빨대(크기) · 큰 판(크기) ·
    깊이 없는 근접 퍽(위치 갱신 안 함) → 버림 · 신호등은 이름의 상태로"""
    import numpy as np
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped

    os.environ.setdefault("ROS_DOMAIN_ID", "101")
    rclpy.init()
    try:
        node = PerceptionNode()
        node.names = {0: "puck_red", 3: "puck_blue", 4: "traffic_light_red"}  # 새 모델(bee_v3ft) 이름
        node.self_filter = False
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = "odom", "pole_front_cam_color_optical_frame"
        t.transform.rotation.w = 1.0
        node.tf.set_transform_static(t, "test")
        bgr_of = {"red": (40, 40, 220), "orange": (30, 140, 250)}

        def run(box, color, depth_mm, cls=0):
            img = np.full((480, 640, 3), 128, np.uint8)
            dep = np.full((480, 640), 3000, np.uint16)  # 뒤 배경 3m
            x0, y0, x1, y1 = box
            img[y0:y1, x0:x1] = bgr_of[color]
            dep[y0:y1, x0:x1] = depth_mm
            s = {"bgr": img, "depth_mm": dep, "stamp": Time(sec=1), "frame_id": "pole_front_cam_color_optical_frame",
                 "fx": 606.0, "fy": 606.0, "cx": 320.0, "cy": 240.0}
            node.frames["pole_front_cam"] = {}
            node.tracks.clear()
            node.update_tracks("pole_front_cam", s, type("Res", (), {"boxes": [_Boxes(cls, box)]})())
            from perception_node import to_msg
            return [(to_msg(i, tr).class_name, tr["color"]) for i, tr in sorted(node.tracks.items())]

        assert run((300, 200, 318, 236), "red", 1000) == [("puck", "red")]            # 서있음 3×6cm
        assert run((300, 200, 336, 218), "red", 1000) == [("puck_lying", "red")]      # 누움 6×3cm
        assert run((300, 200, 318, 236), "orange", 1000) == []                        # 주황 컵홀더 — 색
        assert run((300, 200, 318, 236), "red", 1000, cls=3) == [("puck", "red")]     # YOLO 파랑 · HSV 빨강 → HSV 색 (11/03: 버리지 않음)
        assert run((300, 200, 306, 236), "red", 1000) == []                           # 빨대 1×6cm — 크기
        assert run((300, 100, 400, 300), "red", 1000) == []                           # 16×33cm 판 — 크기
        assert run((250, 150, 400, 400), "red", 0) == []                              # 깊이 없는 근접 퍽 — 위치 모름
        assert run((300, 100, 330, 200), "red", 2000, cls=4) == [("traffic_light", "red")]
    finally:
        rclpy.shutdown()


def test_resolve_model_path(tmp_path):
    """model_path — 비면 model.pt · 파일 이름만이면 설치 폴더(share/perception/model)에서 · 경로면 그대로"""
    from perception_node import resolve_model_path
    (tmp_path / "model.pt").write_bytes(b"")
    (tmp_path / "bee_yolo11n_v1.pt").write_bytes(b"")
    d = str(tmp_path)
    assert resolve_model_path("", d) == os.path.join(d, "model.pt")
    assert resolve_model_path("bee_yolo11n_v1.pt", d) == os.path.join(d, "bee_yolo11n_v1.pt")
    assert resolve_model_path("/x/y/my.pt", d) == "/x/y/my.pt"      # 전체 경로는 그대로
    assert resolve_model_path("none.pt", d) == "none.pt"            # 폴더에 없으면 그대로 (로드 실패 경고)
    (tmp_path / "model.pt").unlink()
    assert resolve_model_path("", d) == ""                          # model.pt도 없으면 모델 없음


# ---------- 표식 (발제 배포 표식: 퍽 AprilTag 0~11 · 장소 12~14 · 바코드) · 신호등 단계 구분
def test_tags_and_markers():
    """AprilTag 36h11 디코드 → 퍽 박스 안 번호 (번호와 색은 따로) / 신호등 두 색 이상 켜짐 = all / 태그 다른 퍽·장소는 다른 물체 · 번호 기억"""
    import cv2
    import numpy as np
    from builtin_interfaces.msg import Time
    from perception_node import classify_color, detect_tags, tag_detector, tag_in_box, to_msg

    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    img = np.full((240, 320, 3), 255, np.uint8)
    img[40:100, 40:100] = cv2.cvtColor(cv2.aruco.generateImageMarker(dic, 2, 60), cv2.COLOR_GRAY2BGR)     # R3
    img[40:120, 180:260] = cv2.cvtColor(cv2.aruco.generateImageMarker(dic, 13, 80), cv2.COLOR_GRAY2BGR)  # L2
    tags = detect_tags(tag_detector(), img)
    assert sorted(t[0] for t in tags) == [2, 13]
    assert tag_in_box(tags, (30, 30, 110, 110), set(range(12)))[0] == 2
    assert tag_in_box(tags, (170, 30, 270, 130), set(range(12))) == (None, None)  # 장소 태그는 퍽 번호 아님

    roi = np.full((40, 14, 3), (40, 40, 40), np.uint8)
    roi[2:10, 4:10] = (40, 40, 220)    # 빨강
    roi[16:24, 4:10] = (40, 220, 230)  # 노랑
    roi[30:38, 4:10] = (60, 180, 60)   # 초록
    assert classify_color("traffic_light", roi) == "all"

    os.environ.setdefault("ROS_DOMAIN_ID", "101")
    rclpy.init()
    try:
        node = PerceptionNode()
        a = node.fuse("gripper_cam", Time(), [{**_det((1.0, 0.0, 0.03), color="red"), "tag": 1},
                                              {**_det((1.02, 0.0, 0.03), color="red"), "tag": 2}])
        assert len(set(a)) == 2  # 붙은 같은 색 퍽 — 번호가 다르면 다른 물체
        b = node.fuse("pole_front_cam", Time(sec=1), [{**_det((1.0, 0.0, 0.03), color="red"), "tag": None}])
        assert b[0] in a and node.tracks[b[0]]["tag"] is not None  # 멀어서 못 읽은 관측이 아는 번호를 덮지 않음
        msg = to_msg(a[0], node.to_map(node.tracks[a[0]]))
        assert ("tag", "1") in [(kv.key, kv.value) for kv in msg.details]
    finally:
        rclpy.shutdown()


def test_barcode_code128():
    """장소 바코드(발제 P2-01, Code128) — zxing-cpp로 읽음 (OpenCV barcode는 EAN·UPC만이라 못 읽음). zxing-cpp 없으면 건너뜀"""
    import cv2
    import numpy as np
    import pytest
    zxingcpp = pytest.importorskip("zxingcpp")
    from perception_node import barcode_detector

    code = np.array(zxingcpp.create_barcode("P2-01", zxingcpp.BarcodeFormat.Code128).to_image(scale=2, add_quiet_zones=True))
    scene = np.full((360, 640), 180, np.uint8)
    scene[120:120 + code.shape[0], 200:200 + code.shape[1]] = code
    ok, texts, _, points = barcode_detector()(cv2.cvtColor(scene, cv2.COLOR_GRAY2BGR))
    assert ok and list(texts) == ["P2-01"]
    assert np.asarray(points).reshape(-1, 4, 2).shape == (1, 4, 2)  # 꼭짓점 4개 — 위치(깊이) 구하는 데 씀


# ---------- 퍽 방향 (그리퍼 손 각도) · 모양 · 거리
def test_puck_yaw():
    """대칭 주기 맞춤 · 정면 옆면 태그 → 사각기둥 yaw 0 (주기 90°) · 누운 퍽 점들의 긴 축 · 정사각형 점들은 방향 없음 · 모양 이름"""
    import cv2
    import numpy as np
    from perception_node import axis_yaw, detect_tags, puck_shape, tag_detector, tag_shape, tag_yaw, wrap_yaw

    assert abs(wrap_yaw(np.radians(100), np.pi / 2) - np.radians(10)) < 1e-9   # 사각기둥: 100° = 10°
    assert abs(wrap_yaw(np.radians(-170), np.pi) - np.radians(10)) < 1e-9      # 누운 퍽: −170° = 10°

    # 카메라가 추적 좌표 +x를 수평으로 봄 (광학 x → −y · y → −z · z → +x) — 정면에 세운 태그 = 퍽 옆면이 카메라를 향함
    rot = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker  # OpenCV 4.7+ · 4.6
    img = np.full((240, 320, 3), 255, np.uint8)
    img[90:150, 130:190] = cv2.cvtColor(marker(dic, 5, 60), cv2.COLOR_GRAY2BGR)
    tags = detect_tags(tag_detector(), img)
    assert [t[0] for t in tags] == [5] and tags[0][3].shape == (4, 2)  # 꼭짓점 4개도 돌려줌
    yaw, face = tag_yaw(tags[0][3], 0.016, 300.0, 300.0, 160.0, 120.0, rot)
    assert face == "side" and abs(wrap_yaw(yaw, np.pi / 2)) < np.radians(5)  # 법선이 카메라 쪽(−x) → 90° 주기로 0

    rng = np.random.default_rng(0)
    t = rng.uniform(-0.03, 0.03, 300)  # 길이 6cm · 폭 3cm 누운 퍽 — 긴 축 30°
    a = np.radians(30)
    pts = np.stack([t * np.cos(a), t * np.sin(a)], -1) + rng.normal(0, 0.003, (300, 2))
    assert abs(wrap_yaw(axis_yaw(pts, 30, 1.8) - a, np.pi)) < np.radians(5)
    assert axis_yaw(rng.uniform(-0.015, 0.015, (300, 2)), 30, 1.8) is None  # 정사각형(서있는 퍽 윗면) — 긴 축 없음
    assert axis_yaw(pts[:10], 30, 1.8) is None                               # 점이 너무 적음

    assert puck_shape("puck_red_box") == "box" and puck_shape("puck_green_cyl") == "cyl"
    assert puck_shape("puck") == "" and puck_shape("person") == ""

    # 몸통 면 태그 → 무조건 사각기둥 · 위·아래 면 태그나 자세 모름 → 모름
    assert tag_shape("side", "standing") == "box" and tag_shape("top", "lying") == "box"
    assert tag_shape("top", "standing") == "" and tag_shape("side", "lying") == ""
    assert tag_shape("side", "") == "" and tag_shape("", "standing") == ""


def test_location_yaw():
    """장소 태그(L1~L3, 48mm)가 박스 옆면에서 카메라를 바라봄 → location_box 물체에 yaw (태그 앞쪽 = 카메라 쪽 −x → 180°), 주기 360"""
    import cv2
    import numpy as np
    from builtin_interfaces.msg import Time
    from perception_node import detect_tags, tag_detector, tag_yaw, to_msg

    rot = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)  # 카메라가 추적 좌표 +x를 수평으로 봄 (test_puck_yaw와 같음)
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker  # OpenCV 4.7+ · 4.6
    img = np.full((240, 320, 3), 255, np.uint8)
    img[60:160, 110:210] = cv2.cvtColor(marker(dic, 13, 100), cv2.COLOR_GRAY2BGR)  # L2
    tags = detect_tags(tag_detector(), img)
    assert [t[0] for t in tags] == [13]
    yaw, face = tag_yaw(tags[0][3], 0.048, 300.0, 300.0, 160.0, 120.0, rot)
    assert face == "side" and abs(abs(yaw) - np.pi) < np.radians(5)  # 법선이 카메라 쪽(−x)

    os.environ.setdefault("ROS_DOMAIN_ID", "101")
    rclpy.init()
    try:
        node = PerceptionNode()
        assert abs(node.location_tag_size - 0.048) < 1e-9
        a = node.fuse("pole_front_cam", Time(), [{**_det((1.0, 0.0, 0.035), cls="location_box", color=""), "tag": 13,
                                                   "yaw": yaw, "yaw_period": 2 * np.pi}])
        msg = to_msg(a[0], node.to_map(node.tracks[a[0]]))
        q = msg.pose.pose.orientation
        assert abs(abs(2 * np.arctan2(q.z, q.w)) - np.pi) < np.radians(5)
        kv = {k.key: k.value for k in msg.details}
        assert kv["tag"] == "13" and kv["yaw_period"] == "360"
        # 방향 모르는 다음 관측이 아는 방향을 덮지 않음
        node.fuse("pole_front_cam", Time(sec=1), [{**_det((1.0, 0.0, 0.035), cls="location_box", color=""), "tag": 13}])
        assert node.tracks[a[0]]["yaw"] is not None
    finally:
        rclpy.shutdown()


# ---------- 상자 (사용자 11/02 — 로봇팔이 놓고 집으려면 상자 크기·가운데·방향 필요): depth 윗면 → start_box / location_box
def _box_depth(boxes, extra=None, cam_h=0.8):
    """위에서 비스듬히 내려다보는 카메라로 찍은 가짜 depth — boxes = [(가운데 x, y, 한 변, 높이, yaw[rad])] (바닥 z=0)
    extra = 그 밖의 물체 [(x, y, 반지름, 높이)] (원통 — 방해물). 화소마다 광선을 쏴서 처음 닿는 면까지 거리"""
    import numpy as np
    rot = _cam_rot(0.0, 50.0)
    trans = np.array([0.0, 0.0, cam_h])
    h, w = 240, 320
    fx = fy = 300.0
    vv, uu = np.mgrid[0:h, 0:w].astype(float)
    ray = np.stack([(uu - 160) / fx, (vv - 120) / fy, np.ones_like(uu)], -1) @ rot.T  # 추적 좌표 방향 (광학 z=1당)
    depth = np.where(ray[..., 2] < 0, -cam_h / np.minimum(ray[..., 2], -1e-9), 0.0)   # 바닥까지 (광학 z 거리)
    for (bx, by, side, bh, yaw) in boxes:   # 윗면만 (옆면은 생략 — 윗면 높이 점이 있으면 충분)
        t = (bh - cam_h) / np.minimum(ray[..., 2], -1e-9)
        px, py = trans[0] + t * ray[..., 0] - bx, trans[1] + t * ray[..., 1] - by
        lx, ly = px * np.cos(-yaw) - py * np.sin(-yaw), px * np.sin(-yaw) + py * np.cos(-yaw)
        hit = (ray[..., 2] < 0) & (abs(lx) < side / 2) & (abs(ly) < side / 2) & (t < depth)
        depth = np.where(hit, t, depth)
    for (ox, oy, r, oh) in (extra or []):   # 원통: 높이마다 원 안이면 닿음 (위쪽부터 촘촘히)
        for z in np.linspace(oh, 0.0, 40):
            t = (z - cam_h) / np.minimum(ray[..., 2], -1e-9)
            px, py = trans[0] + t * ray[..., 0] - ox, trans[1] + t * ray[..., 1] - oy
            hit = (ray[..., 2] < 0) & (px ** 2 + py ** 2 < r ** 2) & (t < depth)
            depth = np.where(hit, t, depth)
    return depth.astype(np.float32), rot, trans, fx, fy, 160.0, 120.0


def test_box_tops():
    """돌아간 205 상자 → 가운데·크기·방향(90° 주기) · 4개 붙인 판 → 큰 상자 하나 · 하얀 원통 방해물 · 작은 물체는 상자 아님"""
    import numpy as np
    from perception_node import find_box_tops, tag_on_box, wrap_yaw

    args = (0.0, (0.05, 0.09), 0.17, 1.0, 0.02)
    d, rot, trans, fx, fy, cx, cy = _box_depth([(1.0, 0.1, 0.205, 0.07, np.radians(30))])
    tops = find_box_tops(d, rot, trans, fx, fy, cx, cy, *args, step=2)
    assert len(tops) == 1
    c, (a, b), yaw, z = tops[0]
    assert np.linalg.norm(c - (1.0, 0.1)) < 0.03 and abs(a - 0.205) < 0.04 and abs(b - 0.205) < 0.04
    assert abs(wrap_yaw(yaw - np.radians(30), np.pi / 2)) < np.radians(6) and abs(z - 0.07) < 0.01
    assert tag_on_box((1.0 - 0.11, 0.1), tops[0], 0.08) and not tag_on_box((1.5, 0.1), tops[0], 0.08)

    # 4개 붙인 판(410×410) → 하나로, 크기 그대로 잼
    big = [(1.1 + dx, dy, 0.205, 0.07, 0.0) for dx in (-0.1025, 0.1025) for dy in (-0.1025, 0.1025)]
    tops = find_box_tops(*_box_depth(big), *args, step=2)
    assert len(tops) == 1 and abs(tops[0][1][0] - 0.41) < 0.05 and abs(tops[0][1][1] - 0.41) < 0.05

    # 하얀 원통 방해물(지름 20cm · 30cm) — 윗면 높이 단면이 원이지만 위로 이어져서 상자 아님 · 작은 물체(10cm)도 아님
    tops = find_box_tops(*_box_depth([(1.0, 0.0, 0.10, 0.07, 0.0)], extra=[(1.2, -0.3, 0.10, 0.30)]), *args, step=2)
    assert tops == []


def test_box_cut_and_arm_mask():
    """(11/04) 화면 가장자리에 걸친 상자 → box_cut True (노드는 방향을 '모름'으로) · 다 보이는 상자 → False
    상자 옆에 붙은 자기 그리퍼(같은 높이 작은 덩어리) → mask_px로 그 둘레를 빼면 상자 크기가 원래대로 (안 빼면 커짐)"""
    import numpy as np
    from perception_node import box_cut, find_box_tops

    args = (0.0, (0.05, 0.09), 0.17, 1.0, 0.02)
    d, rot, trans, fx, fy, cx, cy = _box_depth([(1.0, 0.0, 0.205, 0.07, np.radians(20))])   # 화면 가운데 · 다 보임
    tops = find_box_tops(d, rot, trans, fx, fy, cx, cy, *args, step=2)
    assert len(tops) == 1 and not box_cut(tops[0], rot, trans, fx, fy, cx, cy, 320, 240, 8)
    d, rot, trans, fx, fy, cx, cy = _box_depth([(1.0, 0.55, 0.205, 0.07, np.radians(20))])  # 화면 왼쪽 끝에 걸침
    tops = find_box_tops(d, rot, trans, fx, fy, cx, cy, *args, step=2)
    assert len(tops) == 1 and box_cut(tops[0], rot, trans, fx, fy, cx, cy, 320, 240, 8)

    grip = (1.0, 0.1025 + 0.03, 0.06, 0.07, 0.0)   # 상자 옆면에 붙은 6 cm 그리퍼 흉내
    d, rot, trans, fx, fy, cx, cy = _box_depth([(1.0, 0.0, 0.205, 0.07, 0.0), grip])
    tops = find_box_tops(d, rot, trans, fx, fy, cx, cy, *args, step=2)
    assert len(tops) == 1 and tops[0][1][0] > 0.28   # 그리퍼까지 합쳐서 커짐 (11/04 로봇 녹화와 같은 문제)
    q = rot.T @ (np.array([grip[0], grip[1], 0.07]) - trans)   # 팔 투영점 (arm_pixels가 주는 [u, v, r] 모양)
    arm = np.array([[fx * q[0] / q[2] + cx, fy * q[1] / q[2] + cy, fx * 0.04 / q[2]]])
    tops = find_box_tops(d, rot, trans, fx, fy, cx, cy, *args, step=2, mask_px=arm)
    assert len(tops) == 1 and tops[0][1][0] < 0.25   # 팔을 빼면 원래 상자 크기 (격자 여유 포함)


def test_box_tracks():
    """상자 검출 → start_box (주기 90) · 장소 태그 붙은 상자 → location_box(tag · 태그 방향 360°를 윗면 방향이 덮지 않음)
    · 같은 자리 start_box는 장소 상자가 생기면 지움 · 태그만 보이면 태그에서 반 변 안쪽"""
    import numpy as np
    from builtin_interfaces.msg import Time
    from perception_node import to_msg

    os.environ.setdefault("ROS_DOMAIN_ID", "101")
    rclpy.init()
    try:
        node = PerceptionNode()
        node.long_memory = True  # 긴 기억 규칙 시험 (11/04부터 기본은 끔 — 기억은 통합 쪽)
        assert list(node.location_box_size) == [0.205, 0.205, 0.07]
        sb = {**_det((1.0, 0.5, 0.07), cls="start_box", color=""), "size": np.array([0.2, 0.2, 0.07]),
              "yaw": 0.3, "yaw_period": np.pi / 2}
        a = node.fuse("pole_front_cam", Time(), [sb])
        msg = to_msg(a[0], node.to_map(node.tracks[a[0]]))
        kv = {k.key: k.value for k in msg.details}
        assert msg.class_name == "start_box" and kv["yaw_period"] == "90" and abs(msg.size.x - 0.2) < 1e-6
        # 장소 상자: 태그 방향(360°) 먼저 → 윗면 방향(90°)이 와도 안 바뀜
        lb = {**_det((2.0, 0.0, 0.07), cls="location_box", color=""), "tag": 12, "yaw": np.pi, "yaw_period": 2 * np.pi}
        b = node.fuse("pole_front_cam", Time(sec=1), [lb])
        node.fuse("pole_front_cam", Time(sec=2), [{**lb, "yaw": 0.1, "yaw_period": np.pi / 2}])
        assert node.tracks[b[0]]["yaw_period"] == 2 * np.pi and abs(node.tracks[b[0]]["yaw"] - np.pi) < 1e-9
        # 같은 상자를 뒤에서 봐 생긴 start_box → 장소 상자 자리라 지움
        c = node.fuse("pole_front_cam", Time(sec=3), [{**sb, "z": np.array([2.05, 0.0, 0.07])}])
        node.tracks[a[0]]["hits"] = node.tracks[b[0]]["hits"] = 5  # 여러 번 잡혀 기억하는 상자 (2초 규칙 밖)
        node.prune(3.0, set(c) | {a[0], b[0]}, _view(0.0))
        assert c[0] not in node.tracks and b[0] in node.tracks and a[0] in node.tracks
    finally:
        rclpy.shutdown()


def test_obb_v8_posture_and_lying_yaw():
    """OBB v8 (isaac-synthetic): 자세는 모델 이름 · 누운 퍽 축은 회전 박스 장축을 바닥 평면에 투영한 방향"""
    import numpy as np
    from perception_node import box_long_px, lying_yaw, model_posture
    assert model_posture("puck_red_box") == "standing"
    assert model_posture("puck_green_cyl_lying") == "lying"
    assert model_posture("puck_red") == "" and model_posture("puck") == ""
    # 카메라가 1 m 위에서 바로 아래를 봄 (광학 z → 아래 · x → 추적 x · y → 추적 −y), f 500 · 중심 (320, 240)
    rot = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
    trans = np.array([0.0, 0.0, 1.0])
    # 화면에서 가로로 긴 박스 (장축 = 영상 u = 추적 x) → yaw 0
    flat = [[300, 230], [340, 230], [340, 250], [300, 250]]
    assert abs(lying_yaw(flat, 500, 500, 320, 240, rot, trans, 0.0)) < 1e-6
    # 세로로 긴 박스 (장축 = 영상 v = 추적 −y) → ±90°
    tall = [[310, 220], [330, 220], [330, 260], [310, 260]]
    assert abs(abs(lying_yaw(tall, 500, 500, 320, 240, rot, trans, 0.0)) - np.pi / 2) < 1e-6
    # 정사각형에 가까우면 (끝면이 보임) 축을 못 믿음
    square = [[310, 230], [330, 230], [330, 250], [310, 250]]
    assert lying_yaw(square, 500, 500, 320, 240, rot, trans, 0.0) is None

    class Obb:
        xyxyxyxy = [np.array(flat, float)]
    assert box_long_px(Obb(), (0, 0, 10, 10)) == 40.0
    assert box_long_px(object(), (0, 0, 10, 25)) == 25.0


def test_box_rois():
    """상자 YOLO 네모(rois) 안의 점만 씀 — 네모가 상자 위면 그대로 찾고 · 다른 곳이면 못 찾음 · 빈 목록이면 상자 없음 (11/04)"""
    import numpy as np
    from perception_node import find_box_tops

    d, rot, trans, fx, fy, cx, cy = _box_depth([(1.0, 0.1, 0.205, 0.07, 0.3)])
    args = (0.0, (0.05, 0.09), 0.17, 1.0, 0.02)
    full = find_box_tops(d, rot, trans, fx, fy, cx, cy, *args, step=2)
    assert len(full) == 1
    c = np.array([1.0, 0.1, 0.07])
    q = rot.T @ (c - trans)
    u, v = fx * q[0] / q[2] + cx, fy * q[1] / q[2] + cy
    on = find_box_tops(d, rot, trans, fx, fy, cx, cy, *args, step=2, rois=[(u - 60, v - 40, u + 60, v + 40)])
    assert len(on) == 1 and np.linalg.norm(on[0][0] - full[0][0]) < 0.02
    assert find_box_tops(d, rot, trans, fx, fy, cx, cy, *args, step=2, rois=[(0, 0, 20, 20)]) == []
    assert find_box_tops(d, rot, trans, fx, fy, cx, cy, *args, step=2, rois=[]) == []


def test_capture_dice():
    """주사위 완성 사진: 장소 상자(tag 13) 윗면을 위에서 보는 카메라 → 정사각형으로 펴서 /mission/dice_image 발행 (11/04 발제)
    윗면 가운데에 빨간 점을 그려 두면 편 사진의 가운데가 빨강이어야 함 · 없는 태그를 요청하면 발행 안 함"""
    import cv2
    import numpy as np
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    from std_msgs.msg import Int32

    os.environ.setdefault("ROS_DOMAIN_ID", "101")
    rclpy.init()
    try:
        node = PerceptionNode()
        node.long_memory = True  # 긴 기억 규칙 시험 (11/04부터 기본은 끔 — 기억은 통합 쪽)
        node.dice_px = 200
        # 그리퍼캠이 상자(가운데 1.0, 0, 윗면 0.07) 바로 위 0.5 m에서 아래를 봄: 광학 z = -추적 z
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = "odom", "gripper_cam_color_optical_frame"
        t.transform.translation.x, t.transform.translation.z = 1.0, 0.57
        t.transform.rotation.x, t.transform.rotation.y = 1.0, 0.0  # x축 180° 회전 (아래를 봄)
        t.transform.rotation.w = 0.0
        node.tf.set_transform_static(t, "test")
        img = np.full((480, 640, 3), 90, np.uint8)
        cv2.circle(img, (320, 240), 12, (0, 0, 255), -1)  # 윗면 가운데 = 화면 가운데
        node.frames["gripper_cam"] = {"bgr": img, "stamp": Time(), "frame_id": "gripper_cam_color_optical_frame",
                                      "fx": 600.0, "fy": 600.0, "cx": 320.0, "cy": 240.0}
        node.tracks[1] = {"class_name": "location_box", "tag": 13, "p": np.array([1.0, 0.0, 0.07]),
                          "size": np.array([0.205, 0.205, 0.07]), "yaw": 0.0}
        got = []
        node.dice_pub.publish = lambda m: got.append(m)
        node.on_capture_dice(Int32(data=12))   # 없는 태그 → 발행 안 함
        assert got == []
        node.on_capture_dice(Int32(data=13))
        assert len(got) == 1 and got[0].width == 200 and got[0].height == 200
        top = np.frombuffer(bytes(got[0].data), np.uint8).reshape(200, 200, 3)
        b, g, r = top[95:105, 95:105].reshape(-1, 3).mean(0)
        assert r > 200 and b < 60  # 가운데가 빨강

        # (11/04) 폴캠도 상자를 비스듬히(아래로 40°) 다 보고 있을 때 → 더 수직인 그리퍼캠 사진을 골라야 함 (폴캠 화면은 파랑)
        pitch = np.radians(40.0)
        fwd = np.array([np.cos(pitch), 0.0, -np.sin(pitch)])           # 광학 z (앞)
        right = np.array([0.0, -1.0, 0.0])                              # 광학 x (오른쪽)
        rmat = np.stack([right, np.cross(fwd, right), fwd], -1)         # 열 = 광학 x·y·z 축 (추적 좌표)
        cam_p = np.array([1.0, 0.0, 0.07]) - fwd * 0.7                  # 상자 가운데를 0.7 m 앞에서 봄
        qw = np.sqrt(max(1.0 + np.trace(rmat), 1e-12)) / 2
        qx, qy, qz = ((rmat[2, 1] - rmat[1, 2]) / (4 * qw), (rmat[0, 2] - rmat[2, 0]) / (4 * qw),
                      (rmat[1, 0] - rmat[0, 1]) / (4 * qw))
        t2 = TransformStamped()
        t2.header.frame_id, t2.child_frame_id = "odom", "pole_front_cam_color_optical_frame"
        t2.transform.translation.x, t2.transform.translation.y, t2.transform.translation.z = (float(v) for v in cam_p)
        t2.transform.rotation.x, t2.transform.rotation.y, t2.transform.rotation.z, t2.transform.rotation.w = qx, qy, qz, qw
        node.tf.set_transform_static(t2, "test")
        node.frames["pole_front_cam"] = {"bgr": np.full((480, 640, 3), (255, 0, 0), np.uint8), "stamp": Time(),
                                         "frame_id": "pole_front_cam_color_optical_frame",
                                         "fx": 600.0, "fy": 600.0, "cx": 320.0, "cy": 240.0}
        got.clear()
        node.on_capture_dice(Int32(data=13))
        top = np.frombuffer(bytes(got[0].data), np.uint8).reshape(200, 200, 3)
        b, g, r = top[95:105, 95:105].reshape(-1, 3).mean(0)
        assert r > 200 and b < 60  # 그리퍼캠(수직) 사진 — 폴캠(파랑)이 아님
        del node.frames["gripper_cam"]
        got.clear()
        node.on_capture_dice(Int32(data=13))   # 그리퍼캠이 없으면 폴캠 사진
        top = np.frombuffer(bytes(got[0].data), np.uint8).reshape(200, 200, 3)
        assert top[95:105, 95:105].reshape(-1, 3).mean(0)[0] > 200

        # (11/04) 긴 기억을 끄면(기본) 요청한 태그 상자 트랙이 없어도 지금 보이는 가장 가까운 상자로 찍음
        node.long_memory = False
        got.clear()
        node.on_capture_dice(Int32(data=12))
        assert len(got) == 1
    finally:
        rclpy.shutdown()


def test_tag_gamma_glare():
    """광택 반사로 검은 칸이 회색으로 뜬 원기둥 윗면 태그 (Isaac gripper_cam 11/07 실측 잘라낸 것, 태그 0):
    감마로 어둡게 편 영상을 더하면 읽음 (원본만으로 읽히는지는 OpenCV 버전마다 달라 보지 않음 — 4.6은 못 읽음)"""
    import cv2
    from perception_node import detect_tags, tag_detector

    img = cv2.imread(str(Path(__file__).parent / "data" / "glare_tag0.png"))
    assert [t[0] for t in detect_tags(tag_detector(), img, 2.5)] == [0]


def test_tag_in_overlapping_boxes():
    """앞 퍽 윗면 태그가 뒤 퍽 박스 아래쪽에도 들 때 — 가운데에 더 가까운 앞 퍽 것 (11/07 Isaac seed 2010 실측 박스)"""
    import numpy as np
    from perception_node import tag_in_box

    corners = np.zeros((4, 2))
    tags = [(2, (1290.0, 200.0), None, corners)]
    front, back = (1243, 160, 1316, 285), (1283, 97, 1350, 198)
    ids = set(range(12))
    assert tag_in_box(tags, front, ids, [front, back])[0] == 2
    assert tag_in_box(tags, back, ids, [front, back]) == (None, None)
    assert tag_in_box(tags, back, ids)[0] == 2  # others 없이는 예전처럼 붙음


def test_tag_half_big():
    """가까워 크게(한 변 약 75 px) 찍힌 원기둥 윗면 태그 7 — 반 크기 영상을 더하면 읽음
    (원본만으로 읽히는지는 OpenCV 버전마다 다름 — 4.6은 못 읽고 CI의 새 버전은 읽음. 11/07 Isaac 디버그 영상에서 잘라 냄)"""
    import cv2
    from perception_node import detect_tags, tag_detector

    img = cv2.imread(str(Path(__file__).parent / "data" / "big_tag7.png"))
    tags = detect_tags(tag_detector(), img, half=True)
    assert [t[0] for t in tags] == [7]
    assert 60 < tags[0][2][2] - tags[0][2][0] < 120  # 꼭짓점은 원본 화소 (반 크기 좌표 × 2)


def test_merge_duplicate_pucks():
    """같은 퍽의 겹친 박스 둘 → 하나 (태그는 아는 쪽) · 3 cm 안 같은 퍽 트랙 둘 → 오래된 id 하나 · 태그·색이 다르면 안 합침"""
    import numpy as np
    from perception_node import merge_overlap_dets, merge_tracks

    d = lambda box, conf, **kw: {"class_name": "puck", "box": box, "conf": conf, "color": "red", **kw}
    out = merge_overlap_dets([d((100, 100, 140, 180), 0.6, tag=4), d((102, 98, 142, 175), 0.9), d((300, 100, 340, 180), 0.8)], 0.5)
    assert len(out) == 2 and out[0]["conf"] == 0.9 and out[0]["tag"] == 4
    assert len(merge_overlap_dets([d((100, 100, 140, 180), 0.6, tag=4), d((102, 98, 142, 175), 0.9, tag=5)], 0.5)) == 2  # 태그 다름
    assert len(merge_overlap_dets([d((100, 100, 140, 180), 0.6), {**d((102, 98, 142, 175), 0.9), "color": "green"}], 0.5)) == 2

    tr = lambda p, t, **kw: {"class_name": "puck", "p": np.array(p), "P": np.eye(3), "t": t, "color": "red", "hits": 3, "seen": {}, **kw}
    tracks = {1: tr([0.257, 0.026, 0.13], 1.0), 2: tr([0.258, 0.034, 0.13], 2.0, tag=4), 3: tr([0.172, -0.03, 0.13], 2.0)}
    assert merge_tracks(tracks, 0.03) == {2: 1}
    assert sorted(tracks) == [1, 3] and tracks[1]["tag"] == 4 and tracks[1]["hits"] == 6 and tracks[1]["p"][1] == 0.034
    tracks = {1: tr([0.25, 0.03, 0.13], 1.0, tag=4), 2: tr([0.26, 0.03, 0.13], 2.0, tag=10)}
    assert merge_tracks(tracks, 0.03) == {} and len(tracks) == 2  # 태그가 다르면 다른 퍽


def test_puck_center_from_surface():
    """퍽 depth 점(앞 표면) → 중심: 비스듬히 보면 시선 수평 방향으로 15 mm 안쪽 · 바로 위에서 보면 그대로 · 옆으로는 안 움직임"""
    import numpy as np
    from perception_node import puck_center

    cam = np.array([0.09, 0.03, 0.31])
    surf = np.array([0.25, 0.03, 0.12])                       # 앞으로 0.16 · 아래로 0.19 → 약 50° 내려다봄
    c = puck_center(surf, cam, 0.015, 45.0)
    assert abs(c[1] - surf[1]) < 1e-9 and abs(c[2] - surf[2]) < 1e-9
    assert 0.010 < c[0] - surf[0] < 0.015                     # cos 50° / cos 45° ≈ 0.91 → 13.6 mm
    flat = puck_center(np.array([0.40, 0.03, 0.12]), cam, 0.015, 45.0)
    assert abs(flat[0] - 0.415) < 1e-9                        # 얕게 보면 15 mm 전부
    top = puck_center(np.array([0.09, 0.03, 0.12]), cam, 0.015, 45.0)
    assert np.allclose(top, [0.09, 0.03, 0.12])               # 바로 위
    assert np.allclose(puck_center(surf, cam, 0.0, 45.0), surf)  # 끔


def test_tag_center_top_and_side():
    """태그 꼭짓점 + depth → 퍽 중심: 윗면 태그는 태그 가운데 그대로 · 옆면 태그는 면 법선 반대로 반폭 15 mm 안쪽"""
    import numpy as np
    from perception_node import tag_center

    fx = fy = 600.0
    cx, cy = 320.0, 240.0

    def view(cam, fwd, up):  # 카메라 → 추적 좌표 회전 (광학: x 오른쪽 · y 아래 · z 앞)
        z = np.asarray(fwd, float) / np.linalg.norm(fwd)
        x = np.cross(z, up)
        x /= np.linalg.norm(x)
        return np.stack([x, np.cross(z, x), z], 1), np.asarray(cam, float)

    def corners_of(rot, trans, centre, ax, ay, hs=0.008):  # 태그 꼭짓점 화소 (aruco 순서: 왼위 오위 오아 왼아)
        pts = [centre + (-hs) * ax + hs * ay, centre + hs * ax + hs * ay, centre + hs * ax - hs * ay, centre - hs * ax - hs * ay]
        out = []
        for p in pts:
            c = rot.T @ (p - trans)
            out.append([fx * c[0] / c[2] + cx, fy * c[1] / c[2] + cy])
        return np.array(out), (rot.T @ (centre - trans))[2]

    # 윗면: 위에서 비스듬히 · 퍽 윗면 중심 (0.25, 0.02, 0.13)
    rot, trans = view([0.10, 0.02, 0.32], [0.15, 0.0, -0.19], [0, 0, 1])
    top = np.array([0.25, 0.02, 0.13])
    cs, z = corners_of(rot, trans, top, np.array([0, -1.0, 0]), np.array([1.0, 0, 0]))
    depth = np.full((480, 640), z, np.float32)
    c = tag_center(cs, depth, 0.016, 0.015, fx, fy, cx, cy, rot, trans)
    assert c is not None and np.linalg.norm(c - top) < 0.002

    # 옆면: 카메라를 향한 −x 면 (면 중심 (0.235, 0.02, 0.10)) → 퍽 중심 (0.25, 0.02, 0.10)
    rot, trans = view([0.05, 0.02, 0.16], [0.18, 0.0, -0.06], [0, 0, 1])
    face = np.array([0.235, 0.02, 0.10])
    cs, z = corners_of(rot, trans, face, np.array([0, -1.0, 0]), np.array([0, 0, 1.0]))
    depth = np.full((480, 640), z, np.float32)
    c = tag_center(cs, depth, 0.016, 0.015, fx, fy, cx, cy, rot, trans)
    assert c is not None and np.linalg.norm(c[:2] - [0.25, 0.02]) < 0.002

    # depth 없음 (D435 Min-Z보다 가까움) → solvePnP 위치로 대신
    c = tag_center(cs, np.zeros((480, 640), np.float32), 0.016, 0.015, fx, fy, cx, cy, rot, trans)
    assert c is not None and np.linalg.norm(c[:2] - [0.25, 0.02]) < 0.002

    # 손목 카메라로 집기 직전: 윗면 태그 8 cm 위에서 내려다봄, depth 없음
    rot, trans = view([0.25, 0.02, 0.21], [0.0, 0.0, -1.0], [1.0, 0, 0])
    cs, _ = corners_of(rot, trans, top, np.array([0, -1.0, 0]), np.array([1.0, 0, 0]))
    c = tag_center(cs, np.zeros((480, 640), np.float32), 0.016, 0.015, fx, fy, cx, cy, rot, trans)
    assert c is not None and np.linalg.norm(c - top) < 0.001


def test_arm_near_drops_finger_depth():
    """손가락(가까움)이 퍽 영역 일부를 가리면 그 depth만 빼고 퍽 depth로 · 손가락이 대부분이거나 마스크가 없으면 안 뺌"""
    import numpy as np
    from perception_node import arm_near, box_depth_m

    d = np.full((90, 90), 0.25, np.float32)
    d[30:60, 30:45] = 0.11                      # 손가락 (영역 가운데 1/3의 절반)
    arm = np.zeros(d.shape, bool)
    arm[:, :50] = True
    assert abs(box_depth_m(d, (0, 0, 90, 90), 0.3, arm) - 0.25) < 1e-6
    assert not arm_near(d[30:60, 30:60], None).any()
    full = np.full((30, 30), 0.11, np.float32)
    assert not arm_near(full, np.ones((30, 30), bool)).any()  # 전부 손가락처럼 가까움 → 기준도 그 값 → 안 뺌
