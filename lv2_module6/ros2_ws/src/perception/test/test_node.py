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
                  "self_filter", "base_frame", "self_links", "self_radius", "body_footprint", "body_height", "track_frame",
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
    """QR 판이 보이는 영상을 넣으면 /mission/barcode 로 작업 지시 문자열을 낸다 (documents/mission/README.md)."""
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
        node.create_subscription(String, "/mission/barcode", lambda m: got.append(m.data),
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
    body = (np.eye(3), base, chain)
    args = ((0.08, -0.245, 0.165), 0.6, 0.06)
    assert is_self("obstacle", np.array([2.62, -0.03, 0.31]), body, *args)      # 그리퍼를 장애물로
    assert is_self("obstacle", np.array([2.30, 0.05, 0.25]), body, *args)       # 차체·적재함 위
    assert not is_self("puck", np.array([2.72, -0.05, 0.22]), body, *args)      # 손가락 사이 아래 퍽 — 남김
    assert is_self("obstacle", np.array([2.72, -0.05, 0.22]), body, *args)      # 같은 자리라도 퍽이 아니면 손가락
    assert not is_self("puck", np.array([2.75, 0.30, 0.02]), body, *args)       # 옆 바닥 퍽
    assert not is_self("puck", np.array([2.54, 0.0, 0.02]), body, *args)        # 범퍼 바로 앞 바닥 퍽 (차체 반경 안이지만 바닥)
    assert not is_self("obstacle", np.array([3.20, 0.0, 0.10]), body, *args)    # 앞 장애물
    assert not is_self("puck", np.array([2.50, 0.06, 0.13]), body, *args)       # 박스(앞 1.6 cm) 위 퍽 — 예전 원(0.2)이면 버렸음
    yaw90 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])     # 차체가 +y를 볼 때는 외곽도 같이 돎
    assert is_self("obstacle", np.array([2.35, -0.20, 0.25]), (yaw90, base, chain), *args)  # 뒤쪽 적재함 위


def test_plane_depth():
    """깊이 없는 퍽: 아래를 보는 카메라의 화면 가운데 광선이 z = 0.13 평면과 만나는 깊이"""
    import numpy as np
    from perception_node import plane_depth

    cam = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}
    down = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])  # 광학 z → 추적 −z
    assert abs(plane_depth((320, 240), down, np.array([0.5, 0.0, 0.24]), cam, 0.13) - 0.11) < 1e-9
    assert plane_depth((320, 240), np.eye(3), np.array([0.5, 0.0, 0.24]), cam, 0.13) == 0.0  # 위를 보면 안 만남



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


def test_remember_not_counted():
    """못 봤다고 세지 않는 경우 — 앞이 가려짐(그 자리 깊이가 가까움) · 3m 넘게 멂 · 깊이 없음(너무 가까움) → 계속 기억
    (3프레임 보고 확인한 뒤 3초 — 확인 전이었다면 2초 규칙으로 지워졌을 시간)"""
    import numpy as np

    os.environ.setdefault("ROS_DOMAIN_ID", "101")
    rclpy.init()
    try:
        node = PerceptionNode()
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
        assert run((300, 200, 318, 236), "red", 1000, cls=3) == []                    # YOLO 파랑 · HSV 빨강
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
    """AprilTag 36h11 디코드 → 퍽 박스 안 번호 / 신호등 두 색 이상 켜짐 = all / 태그 다른 퍽·장소는 다른 물체 · 번호 기억"""
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
    assert tag_in_box(tags, (30, 30, 110, 110), set(range(12))) == 2
    assert tag_in_box(tags, (170, 30, 270, 130), set(range(12))) is None  # 장소 태그는 퍽 번호 아님

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
        assert msg.color == "red"  # 색은 HSV 관측 그대로 (태그 번호로 덮지 않음)
        node.fuse("gripper_cam", Time(sec=2), [{**_det((1.0, 0.0, 0.03), color="red"), "tag": 1, "shape": "cuboid"}])
        node.fuse("pole_front_cam", Time(sec=3), [{**_det((1.0, 0.0, 0.03), color="red"), "tag": None, "shape": ""}])
        msg = to_msg(a[0], node.to_map(node.tracks[a[0]]))
        assert ("shape", "cuboid") in [(kv.key, kv.value) for kv in msg.details]  # 옆에서 본 관측이 모양을 덮지 않음
    finally:
        rclpy.shutdown()


def test_classify_shape():
    """위에서 본 퍽 색 마스크 → 원 = cylinder · 정사각형 = cuboid (윗면 흰 태그 라벨 구멍이 있어도) · 작으면 모름"""
    import cv2
    import numpy as np
    from perception_node import classify_shape, color_mask

    img = np.zeros((120, 120, 3), np.uint8)
    cv2.circle(img, (60, 60), 40, (40, 40, 220), -1)
    cv2.rectangle(img, (48, 48), (72, 72), (255, 255, 255), -1)  # 윗면 태그 라벨
    assert classify_shape(color_mask(img, "red", 10)) == "cylinder"
    img[:] = 0
    cv2.rectangle(img, (20, 20), (100, 100), (60, 180, 60), -1)
    cv2.rectangle(img, (48, 48), (72, 72), (255, 255, 255), -1)
    assert classify_shape(color_mask(img, "green", 10)) == "cuboid"
    assert classify_shape(color_mask(img, "red", 10)) == ""  # 그 색이 없음
    small = np.zeros((20, 20), np.uint8)
    cv2.circle(small, (10, 10), 5, 255, -1)
    assert classify_shape(small) == ""


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
