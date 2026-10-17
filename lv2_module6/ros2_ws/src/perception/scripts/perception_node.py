#!/usr/bin/env python3
"""🛠 perception (YOLO nano, 담당: 전승혜) — 설계: documents/ros2-design/02-sense.md

  입력: pole_cam·gripper_cam 컬러·정렬 depth·camera_info (best_effort)
  출력: /tracked_objects (perception/TrackedObjectArray, reliable) — vlm_task_planner · coordinator · grasp_scorer

  camera_info 내부 파라미터로 3D 역투영 → TF(header.stamp 시각)로 map 기준 변환 → 최근접 ID 추적.
  모델 파일이 없으면 빈 목록을 내보낸다 (진단 WARN) — test-bringup 기동 검사용.
"""
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PointStamped
from perception.msg import TrackedObject, TrackedObjectArray
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener

QR_GAP_S = 10.0  # 이만큼 QR이 안 보였다가 다시 읽히면 새 미션 지역으로 보고 재발행 [s]
# 모델 없을 때의 기본 클래스 — 모델이 있으면 모델에 저장된 이름(model.names)을 쓴다 (학습 클래스가 바뀌어도 코드 수정 없음)
CLASSES = ["puck", "puck_lying", "goal_zone", "station_zone", "qr_board", "obstacle", "person", "traffic_light"]
# 크기 추정 실패 시 클래스 기본값 (sx, sy, sz) [m]
SIZE_FALLBACK = {
    "puck": (0.06, 0.06, 0.04),
    "puck_lying": (0.04, 0.06, 0.05),
    "goal_zone": (0.30, 0.30, 0.01),
    "station_zone": (0.45, 0.45, 0.01),
    "qr_board": (0.01, 0.30, 0.30),
    "obstacle": (0.20, 0.20, 0.30),
    "person": (0.40, 0.40, 1.70),
    "traffic_light": (0.14, 0.14, 0.45),
}


def sensor_qos():
    return QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=5)


# HSV 색상(OpenCV H: 0~180) 기준 — 퍽 색 · 신호등 램프 색
HUES = {"red": (0, 180), "yellow": (30,), "green": (60,), "blue": (120,)}


def _nearest_hue(h):
    return min(HUES, key=lambda c: min(min(abs(h - x), 180 - abs(h - x)) for x in HUES[c]))


def classify_color(cls, roi):
    """박스 안 색 → 퍽: 채도 있는 화소의 대표 색상, 신호등: 켜진 램프(밝고 채도 높은 화소) 색 · 없으면 off. 그 외 ''"""
    if roi.size == 0 or cls not in ("puck", "puck_lying", "traffic_light"):
        return ""
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0].ravel(), hsv[..., 1].ravel(), hsv[..., 2].ravel()
    if cls == "traffic_light":
        lit = (s > 100) & (v > 90)  # 켜짐 V 180~230 · 꺼짐 45~57 (시뮬 램프) — 조명 0.55~1.45배에서도 갈리는 경계
        if lit.sum() < 4:
            return "off"
        sel = h[lit]
    else:
        sel = h[(s > 80) & (v > 50)]
        if sel.size < 8:
            return "unknown"
    # 빨강은 0과 180 양끝에 걸치므로 원형 거리로 가장 많은 색을 고른다
    votes = {}
    for hv in sel[:: max(1, sel.size // 400)]:
        c = _nearest_hue(int(hv))
        votes[c] = votes.get(c, 0) + 1
    return max(votes, key=votes.get)


def qr_crop(bgr, box):
    """YOLO qr_board 박스 → 여유 15% 잘라 짧은 변 300px 이상으로 확대 (QR 모듈이 뭉개지지 않게)"""
    x0, y0, x1, y1 = box
    h, w = bgr.shape[:2]
    mx, my = int((x1 - x0) * 0.15), int((y1 - y0) * 0.15)
    crop = bgr[max(0, y0 - my):min(h, y1 + my), max(0, x0 - mx):min(w, x1 + mx)]
    k = max(1.0, 300.0 / max(1, min(crop.shape[:2])))
    return cv2.resize(crop, None, fx=k, fy=k, interpolation=cv2.INTER_CUBIC) if k > 1 else crop


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("model_path", "")
        self.declare_parameter("confidence", 0.4)
        self.declare_parameter("imgsz", 416)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("cpu_threads", 2)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        conf = float(self.get_parameter("confidence").value)
        self.conf_th = conf
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value)

        model_path = str(self.get_parameter("model_path").value)
        if not model_path:
            try:
                import os

                from ament_index_python import get_package_share_directory
                cand = os.path.join(get_package_share_directory("perception"), "model", "best.pt")
                if os.path.exists(cand):
                    model_path = cand
            except Exception:  # noqa: BLE001
                pass
        self.model = None
        self.names = dict(enumerate(CLASSES))  # 클래스 번호 → 이름 (모델이 있으면 모델 것으로)
        self.model_path = model_path
        if model_path:
            try:
                from ultralytics import YOLO
                if self.device == "cpu":  # torch는 기본으로 코어를 전부 씀 → 같은 머신의 제어·Nav2가 밀림 (도커 7코어 중 5코어 실측)
                    import torch
                    torch.set_num_threads(max(1, int(self.get_parameter("cpu_threads").value)))
                self.model = YOLO(model_path)
                self.names = dict(self.model.names)
                self.get_logger().info(f"YOLO 모델 로드: {model_path} (device={self.device})")
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f"YOLO 로드 실패 ({e}) — 빈 목록 발행")
        else:
            self.get_logger().warning("model_path 없음 — 빈 목록 발행 (기동 검사용)")

        self.bridge = CvBridge()
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.lock = threading.Lock()
        self.frames = {}  # cam → dict(bgr, depth_mm, fx, fy, cx, cy, frame_id, stamp)
        self.tracks = {}  # id → dict(class_name, x, y, z, sx, sy, sz, last_seen)
        self.next_id = 1
        self.published = 0
        # QR 문제 (documents/MISSION.md §3-1): A 지역 QR 판을 읽어 /mission_qr 로 — 모델 없이도 동작
        self.qr = cv2.QRCodeDetector()
        self.qr_text = ""
        self.qr_seen = 0.0   # 마지막으로 QR을 읽은 시각 — QR_GAP_S 넘게 안 보였다 다시 보이면 같은 내용도 재발행 (다음 미션 지역)
        self.qr_pub = self.create_publisher(
            String, "/mission_qr",
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, history=QoSHistoryPolicy.KEEP_LAST))

        for cam in ("pole_cam", "gripper_cam"):
            self.create_subscription(Image, f"/{cam}/color/image_raw",
                                     lambda m, c=cam: self.on_color(c, m), sensor_qos())
            self.create_subscription(Image, f"/{cam}/aligned_depth_to_color/image_raw",
                                     lambda m, c=cam: self.on_depth(c, m), sensor_qos())
            self.create_subscription(CameraInfo, f"/{cam}/color/camera_info",
                                     lambda m, c=cam: self.on_info(c, m), sensor_qos())
        self.pub = self.create_publisher(TrackedObjectArray, "/tracked_objects", 5)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 1)
        self.create_timer(1.0 / max(self.rate_hz, 0.1), self.on_publish)
        self.create_timer(0.25, self.on_infer)  # 카메라당 최대 4Hz 추론
        self.create_timer(1.0, self.on_diag)
        self.create_timer(0.5, self.on_qr)

    # ---------- 입력 캐시
    def _slot(self, cam):
        with self.lock:
            return self.frames.setdefault(cam, {})

    def on_color(self, cam, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"{cam} 변환 실패: {e}", throttle_duration_sec=5.0)
            return
        s = self._slot(cam)
        s["bgr"] = np.asarray(bgr)
        s["stamp"] = msg.header.stamp
        s["frame_id"] = msg.header.frame_id or f"{cam}_color_optical_frame"

    def on_depth(self, cam, msg):
        try:
            d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"{cam} 깊이 실패: {e}", throttle_duration_sec=5.0)
            return
        s = self._slot(cam)
        s["depth_mm"] = np.asarray(d)
        s["stamp"] = msg.header.stamp

    def on_info(self, cam, msg):
        s = self._slot(cam)
        s["fx"], s["fy"] = float(msg.k[0]), float(msg.k[4])
        s["cx"], s["cy"] = float(msg.k[2]), float(msg.k[5])

    # ---------- 추론 (4Hz, 카메라당 최대 4Hz)
    def on_infer(self):
        if self.model is None:
            return
        with self.lock:
            cams = [(c, dict(s)) for c, s in self.frames.items()
                    if s.get("bgr") is not None and s.get("depth_mm") is not None
                    and s.get("fx") and time.time() - s.get("infer_t", 0.0) > 0.25]
        for cam, s in cams:
            try:
                res = self.model.predict(s["bgr"], imgsz=self.imgsz, conf=self.conf_th,
                                         device=self.device, verbose=False)[0]
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f"추론 실패: {e}", throttle_duration_sec=5.0)
                continue
            with self.lock:
                self.frames[cam]["infer_t"] = time.time()
            self.update_tracks(cam, s, res)

    def update_tracks(self, cam, s, res):
        h, w = s["bgr"].shape[:2]
        depth_m = s["depth_mm"].astype(np.float32) / 1000.0
        now_stamp = s["stamp"]
        qr_boxes = []
        for b in res.boxes:
            cls = self.names.get(int(b.cls[0]))
            if cls is None:
                continue
            x0, y0, x1, y1 = (int(v) for v in b.xyxy[0].tolist())
            x0, y0 = max(x0, 0), max(y0, 0)
            x1, y1 = min(x1, w - 1), min(y1, h - 1)
            if x1 <= x0 or y1 <= y0:
                continue
            region = depth_m[y0:y1, x0:x1]
            valid = region[(region > 0.05) & (region < 6.0)]
            if valid.size < 8:
                continue
            zm = float(np.median(valid))
            u, v = (x0 + x1) / 2.0, (y0 + y1) / 2.0
            px = (u - s["cx"]) * zm / s["fx"]
            py = (v - s["cy"]) * zm / s["fy"]
            mp = self.to_map(s["frame_id"], now_stamp, px, py, zm)
            if mp is None:
                continue
            if cls == "qr_board":
                qr_boxes.append((x0, y0, x1, y1))
            color = classify_color(cls, s["bgr"][y0:y1, x0:x1])
            fb = SIZE_FALLBACK.get(cls, (0.1, 0.1, 0.1))
            sx = max((x1 - x0) * zm / s["fx"], 0.01)
            sy = max((y1 - y0) * zm / s["fy"], 0.01)
            tid = self.match(cls, mp[0], mp[1], mp[2])
            with self.lock:
                self.tracks[tid] = {"class_name": cls, "x": mp[0], "y": mp[1], "z": mp[2],
                                    "sx": sx, "sy": sy, "sz": fb[2],
                                    "conf": float(b.conf[0]), "last_seen": now_stamp,
                                    "camera": cam, "px": (int(u), int(v)), "color": color}
        with self.lock:
            self.frames[cam]["qr_boxes"] = qr_boxes  # on_qr가 이 박스 안만 해독
            now_s = now_stamp.sec + now_stamp.nanosec * 1e-9
            for tid in [t for t, tr in self.tracks.items()
                        if now_s - (tr["last_seen"].sec + tr["last_seen"].nanosec * 1e-9) > 2.0]:
                del self.tracks[tid]

    def match(self, cls, x, y, z):
        best, best_d = None, 0.4
        with self.lock:
            items = list(self.tracks.items())
        for tid, tr in items:
            if tr["class_name"] != cls:
                continue
            d = math.dist((x, y, z), (tr["x"], tr["y"], tr["z"]))
            if d < best_d:
                best, best_d = tid, d
        if best is None:
            with self.lock:
                best = self.next_id
                self.next_id += 1
        return best

    def to_map(self, frame_id, stamp, x, y, z):
        from geometry_msgs.msg import PointStamped
        pt = PointStamped()
        pt.header.frame_id = frame_id
        pt.header.stamp = stamp
        pt.point.x, pt.point.y, pt.point.z = x, y, z
        try:
            t = self.tf.lookup_transform("map", frame_id, rclpy.time.Time(),
                                         timeout=rclpy.duration.Duration(seconds=0.2))
            out = do_transform_point(pt, t)
            return (out.point.x, out.point.y, out.point.z)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"TF 실패: {e}", throttle_duration_sec=5.0)
            return None

    # ---------- 발행 (10Hz)
    def on_publish(self):
        out = TrackedObjectArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        with self.lock:
            items = sorted(self.tracks.items())
        for tid, tr in items:
            o = TrackedObject()
            o.id = tid
            o.class_name = tr["class_name"]
            o.confidence = float(tr.get("conf", 0.5))
            o.pose.pose.position.x = tr["x"]
            o.pose.pose.position.y = tr["y"]
            o.pose.pose.position.z = tr["z"]
            o.pose.pose.orientation.w = 1.0
            o.size.x, o.size.y, o.size.z = tr["sx"], tr["sy"], tr["sz"]
            o.last_seen = tr["last_seen"]
            o.camera = tr.get("camera", "")
            o.px = [int(v) for v in tr.get("px", (0, 0))]
            o.color = tr.get("color", "")
            out.objects.append(o)
        self.pub.publish(out)
        with self.lock:
            self.published += 1

    # ---------- QR (2Hz) — 내용이 바뀌었거나, QR_GAP_S 넘게 안 보이다 다시 읽혔을 때 발행
    # (한 맵에 미션 지역이 여러 곳 — 같은 문제가 두 지역에 있어도 지역마다 한 번씩. transient_local: 늦게 붙어도 받음)
    def on_qr(self):
        with self.lock:
            imgs = [(c, s["bgr"], list(s.get("qr_boxes", []))) for c, s in self.frames.items()
                    if s.get("bgr") is not None]
        for cam, bgr, boxes in imgs:
            # YOLO가 찾은 qr_board 박스 안만 (여유 15% + 작으면 확대 — 멀리 있는 판도 모듈이 보이게).
            # 모델이 없으면(기동 검사 · 모델 없는 CI) 화면 전체
            crops = [bgr] if self.model is None else [qr_crop(bgr, b) for b in boxes]
            text = ""
            for crop in crops:
                try:
                    text = self.qr.detectAndDecode(crop)[0]
                except cv2.error:
                    text = ""
                if text:
                    break
            if not text:
                continue
            now = time.time()
            fresh = text != self.qr_text or now - self.qr_seen > QR_GAP_S
            self.qr_seen = now
            if fresh:
                self.qr_text = text
                self.qr_pub.publish(String(data=text))
                self.get_logger().info(f"QR 문제 ({cam}): {text}")
            return

    def on_diag(self):
        s = DiagnosticStatus()
        s.name = "perception: 상태"
        s.hardware_id = "beebot"
        if self.model is None:
            s.level = DiagnosticStatus.WARN
            s.message = "모델 없음(기본 대기)"
        else:
            s.level = DiagnosticStatus.OK
            s.message = "인지 중"
        with self.lock:
            n = len(self.tracks)
            pub = self.published
        s.values = [KeyValue(key="추적 물체", value=str(n)),
                    KeyValue(key="발행한 /tracked_objects", value=str(pub)),
                    KeyValue(key="QR 문제", value=self.qr_text or "(아직 없음)")]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [s]
        self.diag_pub.publish(msg)


def main():
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
