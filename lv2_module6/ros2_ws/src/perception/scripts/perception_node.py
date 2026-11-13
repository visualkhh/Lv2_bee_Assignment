#!/usr/bin/env python3
"""🛠 perception (YOLO nano, 담당: 전승혜) — 설계: documents/project/ros/02-sense.md

[입력 / 출력]
  입력: pole_front_cam · gripper_cam 컬러 · 정렬 depth · camera_info (best_effort)
  출력: /tracked_objects (perception/TrackedObjectArray, map, reliable) — 두 카메라 융합 결과 (coordinator · grasp_scorer)
        /perception/<cam>/detections (같은 타입) — 융합 전 카메라별 한 프레임 검출 (id = 붙은 트랙 id, covariance = 관측 공분산)
          → 시뮬레이터가 카메라별 검출과 융합 결과를 나란히 그림 (디버그용 — 소비 노드 없음)
        /perception/<cam>/debug_image — 박스를 그린 영상 (보는 쪽이 있을 때만) · /mission/qr · /mission/dice_image(_raw) · /diagnostics

[좌표 · 추적]
  컬러·depth를 시각 맞춘 쌍으로 받아 camera_info로 3D 역투영 → 영상 시각의 TF로 odom(track_frame)에 놓고 추적.
  발행할 때만 그 순간의 map ← odom으로 바꿈 — SLAM이 map을 고쳐도 트랙이 밀리거나 섞이지 않게.

[두 카메라 융합]
  같은 물체를 두 카메라가 보면 한 트랙에 칼만 갱신. 관측 공분산: depth 축 σz = depth_sigma0 + depth_sigma_k·z²,
  가로·세로 σ = pixel_sigma·z/f → 가까이서 본 카메라가 더 무거움. 결과 공분산은 pose.covariance(위치 3×3).
  퍽 계열(puck · puck_lying)은 한 종류로 합치고 자세는 따로 둠 — 두 카메라가 자세를 다르게 봐도 id 하나.

[퍽 판정 — 색 · 크기 · 자세 · 방향]
  색: HSV(puck_color) → settle_puck_color — 분명히 다른 색일 때만 버리고, 어둡거나 애매하면 YOLO 이름의 색.
  크기: depth로 잰 실제 크기(cm)로 judge_puck. 자세: 윗면 높이(posture_from_height) 먼저, 모르면 옆에서 볼 때만 가로·세로 비율.
  방향(yaw): 태그 자세 · 누운 퍽 긴 축 (대칭 주기 90°/180°). 나갈 때 이름은 puck / puck_lying + color (split_class).

[표식 — AprilTag · 바코드] (documents/mission/README.md)
  AprilTag tag36h11 — 퍽 16mm 번호 0~11 → 퍽 트랙 details tag (번호와 색은 상관없음 — 색은 영상으로만).
    16mm 태그는 gripper_cam이 위에서 가까이 봐야 읽힘. 장소 48mm 번호 12~14 (L1~L3) → location_box.
  바코드 (Code128, P2-01~04 — 신호등 옆 벽) = 작업 지시 문자열 → class barcode (details content) + /mission/qr.

[상자]
  depth로 바닥 위 상자 윗면(205×205×70 mm, 붙이면 더 큰 판)을 찾음 (find_box_tops) → 위치 = 윗면 가운데 · size = 잰 크기 · yaw 주기 90°.
  윗면에 장소 태그(12~14)가 있으면 location_box (yaw = 윗면 변, 주기 90°) · 없으면 start_box.
  태그만 보이고 윗면을 못 찾으면 태그에서 상자 안쪽으로 반 변(location_box_size) 옮긴 곳이 location_box.

[신호등]
  상태 = red · yellow · green · off · all (두 색 이상 동시에 켜짐 — 신호등 미션 단계 구분 신호).

[기억]
  모든 물체가 TRACK_TIMEOUT_S(2초) 넘게 안 보이면 삭제 — 지금 보이는 것만 발행, 긴 기억은 통합(판단) 노드가 맡음.

[모델 파일]
  model_path가 비면 설치 폴더의 model.pt, 이름만 주면 설치 폴더에서 찾음. 모델이 없으면 빈 목록 발행 (진단 WARN — 기동 검사용).
  box_model_path(상자 YOLO)가 있으면 그 네모 안에서만 상자 윗면을 찾음.
"""
import functools
import os
import threading
import time

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from perception.msg import TrackedObject, TrackedObjectArray
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32, String
from tf2_ros import Buffer, TransformListener

QR_GAP_S = 10.0  # 바코드가 이만큼 안 보였다가 다시 읽히면 같은 내용도 /mission/qr 재발행 [s] (barcode_once 끔일 때)
# 모델 없을 때의 기본 클래스 — 모델이 있으면 모델에 저장된 이름(model.names)을 쓴다 (학습 클래스가 바뀌어도 코드 수정 없음)
CLASSES = ["puck", "puck_lying", "goal_zone", "station_zone", "qr_board", "obstacle", "person", "traffic_light"]
# YOLO가 아니라 표식 디코드 · depth로 만드는 물체 (클래스 이름):
#   location_box = 장소 태그(12~14)가 붙은 상자 (놓는 곳 L1~L3) · start_box = 태그 없는 상자 (퍽이 처음 있던 곳) · barcode = 바코드
# 크기 추정 실패 시 클래스 기본값 (sx, sy, sz) [m]
SIZE_FALLBACK = {
    "puck": (0.03, 0.03, 0.06),        # 발제: 30×30×60mm 사각기둥 · Ø30×60mm 원기둥
    "puck_lying": (0.06, 0.03, 0.03),
    "goal_zone": (0.30, 0.30, 0.01),
    "station_zone": (0.45, 0.45, 0.01),
    "qr_board": (0.01, 0.30, 0.30),
    "obstacle": (0.20, 0.20, 0.30),
    "person": (0.40, 0.40, 1.70),
    "traffic_light": (0.14, 0.14, 0.45),
    "location_box": (0.205, 0.205, 0.07),  # 장소 상자 205×205×70 mm (윗면을 depth로 재면 잰 크기를 씀)
    "start_box": (0.205, 0.205, 0.07),     # 시작 상자 — 같은 205×205×70 mm
    "barcode": (0.10, 0.01, 0.05),
}
DYNAMIC_CLASSES = ("person",)  # 스스로 움직이는 물체 — 예측 단계 공정 잡음을 process_noise_dynamic으로 크게
TRACK_TIMEOUT_S = 2.0          # 이만큼 어느 카메라에도 안 보이면 트랙 삭제 [s] (모든 물체)
ROT_VAR_UNKNOWN = 1e3          # 방향을 모름 → pose.covariance 회전 대각 (사실상 모름)
YAW_VAR_KNOWN = np.radians(10.0) ** 2  # yaw를 구했을 때 yaw 분산 [rad²] (σ 10° 가정)


PUCK_CLASSES = ("puck", "puck_lying")
NO_COLOR = ("", "unknown")  # 색을 못 정한 값 — 퍽 합칠 때 '색이 다름'으로 보지 않음
BODY_MIN_Z = 0.05  # 차체 판정은 이 높이 위만 [m]


def kind(cls):
    """클래스 이름 → 트랙에 저장·합칠 때 쓰는 종류. 퍽 계열(puck · puck_lying)은 모두 'puck' (자세는 트랙의 posture에 따로).
    두 카메라가 같은 퍽의 자세를 다르게 봐도 id가 하나가 되게"""
    return "puck" if cls in PUCK_CLASSES else cls


def same_object(tr, d):
    """트랙 tr에 검출 d를 붙일 수 있나 → True/False
    같은 종류 · 태그 번호(둘 다 있을 때) · 바코드 내용(둘 다 있을 때)이 같아야 하고, 퍽이면 색도 같아야 (붙은 다른 색 퍽을 뭉치지 않게).
    색을 못 정한 쪽(빈 값 · unknown)이 있으면 색으로 가르지 않음. 신호등은 색(램프 상태)이 바뀌어도 같은 물체"""
    if tr["class_name"] != kind(d["class_name"]):
        return False
    a, b = tr.get("tag"), d.get("tag")  # 태그 번호가 둘 다 있으면 같아야 (퍽 0~11 · 장소 12~14)
    if a is not None and b is not None and a != b:
        return False
    if tr.get("content") and d.get("content") and tr["content"] != d["content"]:  # 바코드 내용
        return False
    if tr["class_name"] != "puck":
        return True
    a, b = tr.get("color", ""), d.get("color", "")
    return a in NO_COLOR or b in NO_COLOR or a == b


def puck_center(p, cam, offset, full_deg):
    """퍽 depth 점 p(보이는 표면, 추적 좌표) → 퍽 중심 쪽으로 옮긴 점 [m]. cam = 카메라 위치.
    카메라 → 점 수평 방향으로 offset × min(1, cos(내려다본 각)/cos(full_deg)) 만큼 옮김.
    비스듬히 보면 앞면(중심에서 반지름만큼 카메라 쪽)을 재고, 바로 위에서 보면 윗면 가운데라 거의 안 옮김.
    offset ≤ 0이거나 카메라 바로 위·아래면 p 그대로"""
    if offset <= 0:
        return p
    d = np.asarray(p, float) - np.asarray(cam, float)
    h = float(np.hypot(d[0], d[1]))
    if h < 1e-6:
        return p
    cos_down = h / float(np.linalg.norm(d))  # 내려다본 각의 cos (수평으로 보면 1)
    k = min(1.0, cos_down / np.cos(np.radians(full_deg)))
    out = np.array(p, float)
    out[:2] += d[:2] / h * offset * k
    return out


def box_iou(a, b):
    """화면 박스 (x0, y0, x1, y1) 두 개의 IoU (0~1) · 안 겹치면 0"""
    w = min(a[2], b[2]) - max(a[0], b[0])
    h = min(a[3], b[3]) - max(a[1], b[1])
    if w <= 0 or h <= 0:
        return 0.0
    inter = w * h
    return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter)


def merge_overlap_dets(dets, iou_min):
    """한 프레임에서 같은 퍽이 박스 둘로 나온 것을 하나로 → 남은 검출 목록 (iou_min ≤ 0이면 그대로).
    퍽 계열끼리 같은 물체일 수 있고(same_object) 화면 박스 IoU ≥ iou_min이면 신뢰도 높은 쪽만 남기고,
    남긴 쪽이 모르는 태그·모양·자세는 버린 쪽 것으로 채움.
    왜: fuse는 검출을 트랙에 1:1로 붙여서, 두 번째 박스가 같은 퍽의 중복 트랙이 됨"""
    if iou_min <= 0:
        return dets
    out = []
    for d in sorted(dets, key=lambda d: -d.get("conf", 0.0)):
        k = next((o for o in out if d.get("box") is not None and o.get("box") is not None and kind(o["class_name"]) == "puck"
                  and kind(d["class_name"]) == "puck" and same_object({**o, "class_name": kind(o["class_name"])}, d)
                  and box_iou(o["box"], d["box"]) >= iou_min), None)
        if k is None:
            out.append(d)
            continue
        for key in ("tag", "shape", "posture"):
            if k.get(key) in (None, "") and d.get(key) not in (None, ""):
                k[key] = d[key]
    return out


def merge_tracks(tracks, dist_m):
    """같은 퍽인데 트랙이 둘인 것을 합침 — 퍽 트랙끼리 dist_m [m] 안이고 같은 물체일 수 있으면(same_object) 오래된(작은) id로.
    위치·분산은 더 최근에 갱신된 쪽, 태그·모양·자세·방향·색은 아는 쪽.
    tracks를 바꾸고 {지운 id: 남긴 id}를 돌려줌 (dist_m ≤ 0이면 빈 dict). 왜: 한번 생긴 중복 트랙은 다른 단계에서 다시 합쳐지지 않음"""
    gone = {}
    if dist_m <= 0:
        return gone
    ids = sorted(t for t, tr in tracks.items() if tr["class_name"] == "puck")
    for i, a in enumerate(ids):
        if a in gone:
            continue
        for b in ids[i + 1:]:
            if b in gone:
                continue
            ta, tb = tracks[a], tracks[b]
            if np.linalg.norm(np.asarray(ta["p"]) - np.asarray(tb["p"])) >= dist_m or not same_object(ta, tb):
                continue
            if tb["t"] > ta["t"]:
                ta["p"], ta["P"], ta["t"] = tb["p"], tb["P"], tb["t"]
            for key in ("tag", "shape", "posture", "yaw", "yaw_period"):
                if ta.get(key) in (None, "") and tb.get(key) not in (None, ""):
                    ta[key] = tb[key]
            if ta.get("color", "") in NO_COLOR:
                ta["color"] = tb.get("color", "")
            ta["seen"] = {**tb.get("seen", {}), **ta.get("seen", {})}
            del tracks[b]
            gone[b] = a
    return gone


def seg_dist(p, a, b):
    """점 p와 선분 a-b 사이 거리 [m]"""
    ab = b - a
    t = float(np.clip((p - a) @ ab / max(float(ab @ ab), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def is_self(cls, p, body, body_radius, body_height, self_radius, body_box=()):
    """추적 좌표 점 p가 로봇 자기 몸인가 → True/False — body = (차체 원점, [팔 체인 링크 원점…], 차체 회전) (robot_body, 추적 좌표)
    차체: body_box (x_min, x_max, y_half — base_frame 기준 직사각형)가 있으면 그 안, 없으면 원점에서 수평 body_radius 안 ·
    높이 body_height 아래 (적재함·폴·차체). 팔: 이웃 링크를 잇는 선분에서 self_radius 안.
    퍽은 마지막 선분(gripper_link→손끝, 손가락 사이)을 빼고 본다 — 집으려고 손끝 바로 아래 둔 퍽을 지우지 않게"""
    base, chain = body[0], body[1]
    # 바닥(5cm 아래)은 차체로 보지 않음 — 범퍼 바로 앞 퍽이 차체 반경 안에 걸려 지워지지 않게
    if base[2] + BODY_MIN_Z < p[2] < base[2] + body_height:
        if len(body_box) == 3 and len(body) > 2:
            x, y, _ = body[2].T @ (np.asarray(p) - base)  # base_frame 좌표
            if body_box[0] < x < body_box[1] and abs(y) < body_box[2]:
                return True
        elif np.hypot(p[0] - base[0], p[1] - base[1]) < body_radius:
            return True
    segs = list(zip(chain[:-1], chain[1:]))
    if cls in PUCK_CLASSES:
        segs = segs[:-1]
    return any(seg_dist(p, a, b) < self_radius for a, b in segs)


def arm_pixels(chain, rot, trans, fx, fy, cx, cy, radius=0.04, per_seg=10):
    """팔 체인(추적 좌표 링크 원점들)을 그 카메라 화면에 투영 → [[u, v, r], …] (r = 팔 두께 radius의 픽셀 반경).
    이웃 링크 사이를 per_seg개로 나눠 찍음. 링크 원점은 팔 중심선이라 화면에 보이는 팔 몸통보다 안쪽(아래 끝 밖)에 떨어짐
    → 두께만큼 넓혀 판정. rot·trans = 카메라 → 추적 좌표 (lookup_tf). 카메라 뒤 점은 뺌 · 링크가 2개 미만이면 빈 배열(0×3)"""
    if len(chain) < 2:
        return np.zeros((0, 3))
    pts = np.concatenate([a + (b - a) * t[:, None] for a, b in zip(chain[:-1], chain[1:])
                          for t in [np.linspace(0.0, 1.0, per_seg)]])
    cam = (pts - trans) @ rot  # = rot.T @ (p − trans)
    cam = cam[cam[:, 2] > 0.05]
    return np.stack([fx * cam[:, 0] / cam[:, 2] + cx, fy * cam[:, 1] / cam[:, 2] + cy, fx * radius / cam[:, 2]], -1)


def box_has_arm(box, arm_px, min_hits):
    """검출 박스와 팔(투영점 ± 두께)이 겹치는 점이 min_hits개 이상인가 → True면 영상에서 팔을 잡은 것 · 팔 점이 없으면 False"""
    if not len(arm_px):
        return False
    x0, y0, x1, y1 = box
    u, v, r = arm_px[:, 0], arm_px[:, 1], arm_px[:, 2]
    inside = (u + r >= x0) & (u - r <= x1) & (v + r >= y0) & (v - r <= y1)
    return int(inside.sum()) >= min_hits


def stamp_s(stamp):  # ROS 시각 → 초 [s]
    return stamp.sec + stamp.nanosec * 1e-9


def quat_to_rot(q):
    """geometry_msgs Quaternion → 3×3 회전 행렬"""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def meas_cov_cam(z, fx, fy, pixel_sigma, depth_sigma0, depth_sigma_k):
    """깊이 z [m]에서 본 한 점의 관측 공분산 3×3 [m²] — 카메라 광학 프레임(x 오른쪽 · y 아래 · z 앞)"""
    sz = depth_sigma0 + depth_sigma_k * z * z
    return np.diag([(pixel_sigma * z / fx) ** 2, (pixel_sigma * z / fy) ** 2, sz * sz])


def kf_update(p, P, z, R):
    """위치 상태(정지 모델, H=I) 칼만 갱신 — 관측 z, 관측 공분산 R → (새 p, 새 P, 칼만 이득 K)"""
    K = P @ np.linalg.inv(P + R)
    p = p + K @ (z - p)
    P = (np.eye(3) - K) @ P
    return p, 0.5 * (P + P.T), K


def sensor_qos():
    # 카메라 구독 QoS (best_effort) · depth=5: 컬러·depth 동기화가 한쪽 프레임을 기다릴 여유 (1이면 도착 순서가 조금만 어긋나도 쌍이 안 맞음)
    return QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=5)


# 색 이름별 기준 H (OpenCV H: 0~180) — 빨강 0·180 · 주황 17 · 노랑 30 · 초록 70·84 · 파랑 110 · 보라 142
#   주황 17: 밝은 조명에서 주황 H가 올라가도 노랑(30)과 갈리게 (경계 23.5)
#   초록 70·84 두 기준: ± hue_tol(10) = H 60~94 — 실물 초록 퍽의 넓은 H 폭을 덮고 노랑·파랑과는 떨어져 있음
HUES = {"red": (0, 180), "orange": (17,), "yellow": (30,), "green": (70, 84), "blue": (110,), "purple": (142,)}
LAMP_COLORS = ("red", "yellow", "green")  # 신호등 램프 — 이 셋 중에서만
ACHROMATIC_S = 60    # 채도가 이보다 낮은 화소가 대부분이면 무채색 퍽 (흰색 · 검정)
WHITE_V, BLACK_V = 110, 80  # 무채색일 때 밝기 중앙값 경계 — 이상이면 흰색 · 이하면 검정 · 사이는 unknown


def hue_dist(h, name):  # H 값(정수 배열) → 색 name의 기준 H들까지 원형 거리 중 가장 작은 것 (빨강은 0과 180 양끝)
    return np.min([np.minimum(abs(h - x), 180 - abs(h - x)) for x in HUES[name]], axis=0)


def _hue_votes(sel, names=None):
    """H 값들(sel) → {색 이름: 표 수} — 최대 약 400개를 골라 화소마다 가장 가까운 색(names 안, 없으면 HUES 전체)에 한 표
    거리가 같으면 names 앞쪽 색 · dict 순서는 처음 표를 받은 순서"""
    names = list(names or HUES)
    h = sel[:: max(1, sel.size // 400)].astype(int)
    votes = {}
    for i in np.argmin([hue_dist(h, c) for c in names], axis=0):
        votes[names[i]] = votes.get(names[i], 0) + 1
    return votes


def classify_color(cls, roi):
    """박스 영상 roi(BGR) → 색 이름
      퍽: 박스 가운데(가장자리 20%씩 뺌 — 바닥이 섞이지 않게) 화소의 대표 색 · 채도 없는 화소가 대부분이면 white/black/unknown
          · 색 화소가 너무 적으면 unknown
      신호등: 켜진 램프(밝고 채도 높은 화소) 색 red/yellow/green · 두 색 이상이면 all · 켜진 게 없으면 off
      그 밖 클래스 · 빈 영상 → ''"""
    if roi.size == 0 or cls not in ("puck", "puck_lying", "traffic_light"):
        return ""
    if cls != "traffic_light":
        h0, w0 = roi.shape[:2]
        mh, mw = int(h0 * 0.2), int(w0 * 0.2)
        roi = roi[mh:h0 - mh, mw:w0 - mw] if h0 - 2 * mh > 0 and w0 - 2 * mw > 0 else roi
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0].ravel(), hsv[..., 1].ravel(), hsv[..., 2].ravel()
    if cls == "traffic_light":
        lit = (s > 100) & (v > 90)  # 켜진 램프 = 채도·밝기 높음 (꺼진 램프는 어두움)
        if lit.sum() < 4:
            return "off"
        votes = _hue_votes(h[lit], LAMP_COLORS)
        total = sum(votes.values())
        if sum(1 for n in votes.values() if n >= 0.15 * total) >= 2:
            return "all"  # 두 색 이상 동시에 켜짐 — 신호등 미션 단계 구분 신호 (세 색 동시 3초)
        return max(votes, key=votes.get)
    else:
        if (s < ACHROMATIC_S).mean() > 0.6:
            med = float(np.median(v))
            return "white" if med >= WHITE_V else "black" if med <= BLACK_V else "unknown"
        sel = h[(s > 80) & (v > 50)]
        if sel.size < 8:
            return "unknown"
    # 빨강은 0과 180 양끝에 걸치므로 원형 거리로 가장 많은 색을 고른다
    votes = _hue_votes(sel)
    return max(votes, key=votes.get)


def resolve_model_path(model_path, model_dir):
    """model_path 파라미터 → 실제 가중치 파일 경로 ('' = 모델 없음 — 빈 목록 발행)
      ''              → model_dir/model.pt (있으면, 없으면 '')
      파일 이름만     → model_dir/<이름> (예: bee_yolo11n_v<N>.pt — models/yolo/*.pt가 share/perception/model에 설치됨) · 없으면 그대로
      그 밖(경로)     → 그대로 (없으면 YOLO 로드 실패 경고)"""
    if not model_path:
        cand = os.path.join(model_dir, "model.pt") if model_dir else ""
        return cand if cand and os.path.exists(cand) else ""
    if os.sep not in model_path and model_dir:
        cand = os.path.join(model_dir, model_path)
        if os.path.exists(cand):
            return cand
    return model_path


def split_class(name):
    """YOLO 클래스 이름 → (설계 이름, 이름에 든 색 또는 '') — 모델이 색별로 배워도 나갈 때는 설계 이름 + color
      색 있는 이름: puck_red → ('puck', 'red') · traffic_light_off → ('traffic_light', 'off')
      색 없는 이름: puck · puck_lying → ('puck', '') (자세는 이름 대신 높이·크기로) · 그 밖은 그대로 ('person', '')"""
    if name.startswith("puck"):
        return "puck", next((t for t in name.split("_")[1:] if t in HUES), "")
    if name.startswith("traffic_light_"):
        state = name[len("traffic_light_"):]
        return "traffic_light", state if state in LAMP_COLORS + ("off",) else ""
    return name, ""


def puck_shape(name):
    """YOLO 퍽 클래스 이름 → 모양 'box'(사각기둥) · 'cyl'(원기둥) · ''(모양 없는 이름 · 퍽 아님)
    모양을 배운 모델의 이름(puck_red_box · puck_green_cyl …)에서 읽음 — 방향(yaw) 대칭 주기가 모양마다 달라서 필요"""
    if not name.startswith("puck"):
        return ""
    return next((t for t in name.split("_")[1:] if t in ("box", "cyl")), "")


def model_posture(name):
    """퍽 클래스 이름의 자세 — 자세를 배운 모델 이름(puck_<색>_<모양>[_lying])에서만 앎:
    puck_red_box → 'standing' · puck_red_cyl_lying → 'lying' · 모양 없는 이름(puck_red · puck) → ''"""
    if not puck_shape(name):
        return ""
    return "lying" if name.endswith("_lying") else "standing"


def lying_yaw(corners, fx, fy, cx, cy, rot, trans, z_plane, min_aspect=1.3):
    """누운 퍽의 OBB 꼭짓점 4개 [px] → 퍽 축 방향 yaw [rad, −π/2~π/2) (추적 좌표) · 모르면 None.
    OBB 장축 양끝(짧은 변 가운데) 두 픽셀에서 시선을 쏴 퍽 중심 높이 z_plane 평면과 만나는 두 점의 방향.
    장축÷단축 < min_aspect면 끝면이 카메라를 향한 것 → 축을 못 믿어 None. rot·trans = 카메라 광학 → 추적 좌표"""
    c = np.asarray(corners, float).reshape(4, 2)
    e0, e1 = np.linalg.norm(c[1] - c[0]), np.linalg.norm(c[2] - c[1])
    if min(e0, e1) <= 0 or max(e0, e1) / min(e0, e1) < min_aspect:
        return None
    ends = [(c[1] + c[2]) / 2, (c[3] + c[0]) / 2] if e0 >= e1 else [(c[0] + c[1]) / 2, (c[2] + c[3]) / 2]
    pts = []
    for u, v in ends:
        d = rot @ np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        if abs(d[2]) < 1e-6 or (z_plane - trans[2]) / d[2] <= 0:
            return None  # 시선이 평면과 나란하거나 카메라 뒤에서 만남
        pts.append(trans + d * (z_plane - trans[2]) / d[2])
    dx, dy = pts[1][:2] - pts[0][:2]
    return wrap_yaw(float(np.arctan2(dy, dx)), np.pi)


def box_long_px(b, xyxy):
    """검출 박스 b의 긴 변 [px] — OBB면 회전 박스 변, 아니면 축 정렬 박스(xyxy) 변"""
    if hasattr(b, "xyxyxyxy"):
        c = np.asarray(b.xyxyxyxy[0].tolist(), float).reshape(4, 2)
        return float(max(np.linalg.norm(c[1] - c[0]), np.linalg.norm(c[2] - c[1])))
    x0, y0, x1, y1 = xyxy
    return float(max(x1 - x0, y1 - y0))


def tag_shape(face, posture):
    """태그가 붙은 면(tag_yaw의 side/top)과 자세 → 'box'(몸통 면 태그 = 확실히 사각기둥) · ''(이것만으로는 모름)
    규칙: 원기둥은 위·아래 면에만 태그, 몸통(옆) 면에 태그가 있으면 무조건 사각기둥
      서있는 퍽 + 법선 수평(side) = 몸통 면 → box
      누운 퍽 + 법선 수직(top) = 위를 보는 몸통 면 → box (누운 원기둥의 위·아래 면 태그는 법선이 수평)
    반대(서있는데 top · 누웠는데 side)는 위·아래 면 태그라 두 모양 모두 가능 → '' (지금 사각기둥은 2면에만 태그가 있어
    '몸통 태그가 안 보임'으로 원기둥이라 할 수 없음) · 자세 모름도 ''"""
    if (posture == "standing" and face == "side") or (posture == "lying" and face == "top"):
        return "box"
    return ""


def arm_near(patch, arm, margin=0.05):
    """depth 영역 patch [m]와 팔 마스크 arm → 뺄 화소 bool 배열 (True = 팔·손가락 depth라 물체가 아님)
    팔 마스크 안에서, 물체 depth 기준(영역 depth의 80 백분위)보다 margin 넘게 가까운 화소만 True.
    margin 5 cm: 손가락은 카메라에서 약 10 cm · 퍽은 17 cm 넘음 (더 작으면 사각 퍽 윗면까지 빠짐).
    손가락이 영역 대부분이면 기준도 손가락이라 아무것도 안 뺌 (손 밑에서 집는 중 퍽까지 지우지 않게)
    마스크가 없거나 유효 화소가 8개 미만이면 전부 False"""
    if arm is None or not arm.any():
        return np.zeros(patch.shape, bool)
    ok = np.isfinite(patch) & (patch > 0.05)
    if ok.sum() < 8:
        return np.zeros(patch.shape, bool)
    return arm & ok & (patch < float(np.percentile(patch[ok], 80)) - margin)


def arm_mask_image(arm_px, shape):
    """arm_pixels [[u, v, r], …] → 팔·손가락이 찍힌 화소 bool 영상 (반경 = 팔 두께 1.5배 + 6 px — find_box_tops와 같음) · 없으면 None"""
    if arm_px is None or not len(arm_px):
        return None
    m = np.zeros(shape[:2], np.uint8)
    for u, v, r in arm_px:
        if -100 < u < shape[1] + 100 and -100 < v < shape[0] + 100:
            cv2.circle(m, (int(u), int(v)), int(r * 1.5 + 6), 1, -1)
    return m.astype(bool) if m.any() else None


def box_depth_m(depth_m, box, min_valid, arm=None):
    """퍽 박스 가운데 1/3 영역의 깊이 중앙값 [m] — 가장자리 배경이 섞이지 않게. 0(측정 실패)·팔 depth(arm_near)는 뺌
    측정된 화소가 영역의 min_valid(비율)보다 적으면 0.0 (= 깊이 없음)
    왜: 퍽이 너무 가까우면 퍽 화소는 0이 되고 사이로 보인 배경 몇 화소만 남아 엉뚱한 거리가 나옴"""
    x0, y0, x1, y1 = box
    bw, bh = x1 - x0, y1 - y0
    sl = (slice(y0 + bh // 3, y1 - bh // 3 + 1), slice(x0 + bw // 3, x1 - bw // 3 + 1))
    patch = depth_m[sl]
    near = arm_near(patch, arm[sl] if arm is not None else None)  # 손가락 depth는 뺌 (섞이면 퍽 위치가 틀어짐)
    valid = patch[np.isfinite(patch) & (patch > 0.05) & ~near]
    if valid.size == 0 or valid.size < min_valid * patch.size:
        return 0.0
    return float(np.median(valid))


PUCK_TOPS = {"standing": 0.06, "lying": 0.03}  # 받침 위 퍽 윗면 높이 [m] — 서 있음 60 mm · 누움 30 mm (퍽 30×30×60 · Ø30×60)


def posture_from_height(depth_m, box, zm, rot, trans, fx, fy, cx, cy, floor_z, supports=(0.0, 0.07), tol=0.02, min_pts=15,
                        margin=0.01):
    """퍽 자세를 '윗면 높이'로 판정 — 카메라 각도와 상관없음 (가로·세로 비율은 옆에서 볼 때만 맞음)
      ① 박스 안에서 깊이가 퍽 거리(zm) ±4 cm인 화소만 = 퍽 자체 (뒤 바닥·벽 빼기) → 추적 좌표 3D 점
      ② 퍽 윗면 높이 = 그 점들 높이의 90% 값 (튀는 점 몇 개는 무시)
      ③ 받침(바닥 floor_z · 상자 윗면 floor_z + 0.07) 위로 6 cm면 standing · 3 cm면 lying — 4가지 중 가장 가까운 것
         (바닥 서있음 6 · 바닥 누움 3 · 상자 위 서있음 13 · 상자 위 누움 10 cm → 서로 3 cm 넘게 떨어져서 카메라 각도와 상관없이 구분)
      가장 가까운 것도 tol(2 cm)보다 멀거나 점이 min_pts보다 적으면 '' (모름 — 기울어진 퍽 · 깊이 부족)
      ④ 서있음 후보와 누움 후보가 margin(1 cm) 안으로 비슷하게 가까우면 '' (애매) → 노드는 옆에서 볼 때 가로·세로 비율로 대신함
    → (자세 standing/lying/'', 받침 바닥 위 윗면 높이 [m] 또는 None — zm ≤ 0이거나 점이 모자랄 때)"""
    if zm <= 0:
        return "", None
    x0, y0, x1, y1 = box
    patch = depth_m[y0:y1 + 1, x0:x1 + 1]
    vv, uu = np.nonzero(np.isfinite(patch) & (np.abs(patch - zm) < 0.04) & (patch > 0.05))
    if len(vv) < min_pts:
        return "", None
    z = patch[vv, uu]
    pts = np.stack([(uu + x0 - cx) * z / fx, (vv + y0 - cy) * z / fy, z], -1) @ rot.T + trans
    top = float(np.percentile(pts[:, 2], 90)) - floor_z
    errs = {}  # 자세마다 가장 가까운 받침 기준 오차
    for sup in supports:
        for pose, hgt in PUCK_TOPS.items():
            errs[pose] = min(errs.get(pose, 1e9), abs(top - (sup + hgt)))
    best = min(errs, key=errs.get)
    other = max(errs, key=errs.get)
    if errs[best] > tol or errs[other] - errs[best] < margin:
        return "", top
    return best, top


def view_from_side(rot, max_down_deg=35.0):
    """카메라가 옆에서 보는가 → True/False (광학 z축이 수평에서 아래로 max_down_deg 안) — 가로·세로 비율 자세 판정은 이때만 믿음"""
    return float(-rot[2, 2]) < np.sin(np.radians(max_down_deg))


def puck_color(roi, colors, hue_tol):
    """퍽 박스 안 색 → (색 이름, H 중앙값). colors = 퍽으로 볼 색 이름들 (HUES 안의 것)
    가장자리 20%씩 뺀 안쪽에서 채도·밝기 있는 화소마다 가장 가까운 색에 투표 — 그 색 기준 H에서 hue_tol 넘게 멀면 'other'에 투표
    (classify_color처럼 가장 가까운 색으로 억지로 맞추면 주황 물체 H12~13이 빨강이 됨)
    색 화소가 너무 적거나 영상이 비면 ('unknown', -1)"""
    h0, w0 = roi.shape[:2]
    mh, mw = int(h0 * 0.2), int(w0 * 0.2)
    roi = roi[mh:h0 - mh, mw:w0 - mw]
    names = [c for c in colors if c in HUES]
    if roi.size == 0 or not names:
        return "unknown", -1
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV).reshape(-1, 3)
    h = hsv[(hsv[:, 1] > 80) & (hsv[:, 2] > 50), 0].astype(int)  # classify_color와 같은 채도·밝기 하한 (바닥·그림자·흰 반사 뺌)
    if h.size < 8:
        return "unknown", -1
    # 빨강은 0과 180 양끝에 걸치므로 원형 거리
    d = np.stack([hue_dist(h, c) for c in names])
    pick = np.where(d.min(axis=0) <= hue_tol, d.argmin(axis=0), len(names))
    votes = np.bincount(pick, minlength=len(names) + 1)
    return (names + ["other"])[int(np.argmax(votes))], int(np.median(h))


# 디버그 영상 네모 색 (BGR, 색 이름별) · 이름표 글자 (영어 — OpenCV 기본 글꼴은 한글을 못 씀)
DEBUG_BGR = {"red": (0, 0, 255), "green": (0, 200, 0), "yellow": (0, 220, 255), "off": (160, 160, 160), "all": (255, 255, 255)}
DEBUG_SHAPE = {"box": "box", "cyl": "cyl"}
DEBUG_POSTURE = {"standing": "standing", "lying": "lying"}


def settle_puck_color(roi, color, model_color, puck_colors, use_model_color=True):
    """퍽 색 최종 결정 — 분명히 다른 색일 때만 버림 → (색, '') · 버리면 (None, 버린 이유)
    color = puck_color 결과 (red · green · other · unknown), model_color = YOLO 이름의 색 (puck_red → red, 없으면 '')
      · HSV가 퍽 색(puck_colors) → 그 색. YOLO 색과 달라도 HSV 색을 씀 (화소를 직접 잰 값)
      · HSV가 퍽 색 범위 밖(other) → 화소마다 가장 가까운 색으로 다시 투표:
          주황·노랑·파랑·보라 등 분명히 다른 색이 많으면 버림 ('다른 색(…)')
          그래도 퍽 색 쪽이 많으면(어둡거나 조명으로 조금 벗어남) → YOLO 색 (없으면 그 가까운 색)
      · HSV 색 화소가 너무 적음(unknown: 어둡거나 흰빛 반사) → YOLO 색이 있으면 그것, 없으면 버림 ('색 없음')
    use_model_color False면 YOLO 색은 안 씀 (HSV만)"""
    model_color = model_color if use_model_color and model_color in puck_colors else ""
    if color in puck_colors:
        return color, ""
    if color == "other":
        near, _ = puck_color(roi, list(HUES), 90)  # 모든 색 중 가장 가까운 색으로 화소 투표 (허용 폭 90 = 다 투표)
        if near not in puck_colors:
            return None, f"다른 색({near})"
        return (model_color or near), ""
    if model_color:
        return model_color, ""
    return None, "색 없음"


def tag_detector():
    """AprilTag tag36h11 검출 함수 (OpenCV aruco — 새 라이브러리 없이) · aruco가 없으면 None.
    OpenCV 4.7+ ArucoDetector 클래스 · 4.6 이하(Ubuntu 24.04 apt python3-opencv 4.6) detectMarkers 함수 둘 다.
    돌려주는 함수(gray, min_px=None) → (corners, ids, rejected)
      min_px: 태그 후보로 볼 최소 둘레 [화소] — 영상 크기와 상관없이 같은 화소 기준 (minMarkerPerimeterRate = min_px / 긴 변).
      None이면 예전 값 0.01 (640px에서 둘레 6.4px). 둘레가 작은 후보까지 다 검사하면 느림 — 젯슨 py-spy 11/11: 태그가 노드 CPU의 37%.
      노드는 tag_min_perimeter_px(16)·tag_half_min_perimeter_px(24)를 씀: 실물 155장 + Isaac 800장에서 찾은 태그 같고 시간 약 1/5"""
    aruco = getattr(cv2, "aruco", None)
    if aruco is None or not hasattr(aruco, "DICT_APRILTAG_36h11"):
        return None
    if hasattr(aruco, "ArucoDetector"):
        params = aruco.DetectorParameters()
        params.minMarkerPerimeterRate = 0.01
        det = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11), params)

        def detect(gray, min_px=None):
            rate = 0.01 if min_px is None else min_px / max(gray.shape[:2])
            if params.minMarkerPerimeterRate != rate:  # 바뀔 때만 다시 넣음
                params.minMarkerPerimeterRate = rate
                det.setDetectorParameters(params)
            return det.detectMarkers(gray)
        return detect
    params = aruco.DetectorParameters_create()
    dic = aruco.Dictionary_get(aruco.DICT_APRILTAG_36h11)

    def detect_old(gray, min_px=None):
        params.minMarkerPerimeterRate = 0.01 if min_px is None else min_px / max(gray.shape[:2])
        return aruco.detectMarkers(gray, dic, parameters=params)
    return detect_old


@functools.lru_cache(maxsize=4)
def gamma_lut(gamma):
    """회색조 감마 표 256칸 (out = 255 · (in/255)^gamma) — gamma > 1이면 어둡게: 반사로 뜬 회색을 검정 쪽으로"""
    return np.array([255.0 * (v / 255.0) ** gamma for v in range(256)], np.uint8)


BARCODE_LIB = "zxing-cpp"  # barcode_detector가 고른 라이브러리 이름 (진단 표시용)


def barcode_detector():
    """바코드 읽기 함수를 돌려줌 — 함수(bgr) → (ok, 문자열들, 종류들, 꼭짓점들[N×4×2]) · 쓸 라이브러리가 없으면 None.
    발제 바코드(P2-01 — 영문·하이픈)는 Code128이라 OpenCV barcode(EAN·UPC 숫자 상품코드만)로는 못 읽음
    → zxing-cpp(pip zxing-cpp)가 있으면 그것, 없으면 OpenCV (EAN·UPC만)"""
    try:
        import zxingcpp
    except ImportError:
        zxingcpp = None
    global BARCODE_LIB
    if zxingcpp is not None:
        # 1D(줄무늬) 바코드만 찾음 — 발제 바코드는 Code128 · QR 같은 2D 코드는 미션에 없음.
        # 2D 검색(QR·DataMatrix·Aztec·PDF417)이 빠져 한 번 읽는 시간이 줄어듦 (젯슨 py-spy 11/11: 바코드가 노드 CPU의 17%)
        formats = getattr(zxingcpp.BarcodeFormat, "LinearCodes", None)  # zxing-cpp 2.x · 없으면(옛 버전) 전부 찾음
        BARCODE_LIB = "zxing-cpp (1D: Code128·Code39 등)" if formats is not None else "zxing-cpp (Code128·Code39·QR 등)"

        def read(bgr):
            rs = [r for r in (zxingcpp.read_barcodes(bgr, formats=formats) if formats is not None else zxingcpp.read_barcodes(bgr))
                  if r.text]
            pts = [np.array([[p.x, p.y] for p in (r.position.top_left, r.position.top_right,
                                                   r.position.bottom_right, r.position.bottom_left)], float) for r in rs]
            return bool(rs), [r.text for r in rs], [str(r.format) for r in rs], (np.array(pts) if rs else None)
        return read
    mod = getattr(cv2, "barcode", None)
    if mod is None or not hasattr(mod, "BarcodeDetector"):
        return None
    BARCODE_LIB = "OpenCV barcode (EAN·UPC만 — 발제 Code128은 못 읽음, pip install zxing-cpp)"
    det = mod.BarcodeDetector()
    # 4.8+ detectAndDecodeWithType · 4.5.3~4.7 detectAndDecode (같은 순서로 돌려줌)
    return det.detectAndDecodeWithType if hasattr(det, "detectAndDecodeWithType") else det.detectAndDecode


def detect_tags(detector, bgr, gamma=0.0, half=False, min_px=None, half_min_px=None):
    """BGR 영상 → [(태그 번호, 중심 (u, v), 박스 (x0, y0, x1, y1), 꼭짓점 4×2)] [px] · 검출기가 없으면 []
    꼭짓점 순서는 aruco 그대로 (태그 기준 왼쪽 위 → 오른쪽 위 → 오른쪽 아래 → 왼쪽 아래) — 방향(tag_yaw)에 씀
    gamma > 0: 어둡게 편 영상(감마)으로 한 번 더 찾아 합침 — 광택 반사로 검은 칸이 회색으로 뜬 태그도 읽히게
    half: 반 크기 영상으로도 한 번 더 — 가까워 크게 찍힌 태그는 원본에서 테두리를 못 찾을 때가 있음
    min_px · half_min_px: 원본·감마 영상과 반 크기 영상에서 태그 후보 최소 둘레 [px] (tag_detector) — None이면 예전 값
      반 크기 영상은 크게 찍힌 태그를 찾는 용이라 작은 후보를 볼 필요가 없음
    여러 영상에서 같은 태그(같은 번호, 중심 8 px 안)는 한 번만"""
    if detector is None:
        return []
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    found = []
    # (영상, 배율, 최소 둘레) — 찾은 꼭짓점 = 영상 좌표 × 배율
    passes = [(gray, 1.0, min_px)] + ([(cv2.LUT(gray, gamma_lut(gamma)), 1.0, min_px)] if gamma > 0 else []) + \
        ([(cv2.resize(gray, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA), 2.0, half_min_px)] if half else [])
    for img, scale, px in passes:
        corners, ids, _ = detector(img) if px is None else detector(img, px)
        if ids is None:
            continue
        for c, i in zip(corners, ids.ravel()):
            pts = c.reshape(-1, 2) * scale
            if not any(j == i and np.hypot(*(p.mean(0) - pts.mean(0))) < 8.0 for j, p in found):  # 두 영상 모두 찾은 태그는 한 번만
                found.append((i, pts))
    out = []
    for i, pts in found:
        (x0, y0), (x1, y1) = pts.min(0), pts.max(0)
        out.append((int(i), (float(pts[:, 0].mean()), float(pts[:, 1].mean())), (int(x0), int(y0), int(x1) + 1, int(y1) + 1),
                    pts.astype(np.float64)))
    return out


def clip_box(xyxy, w, h):
    """YOLO 박스 → 화면(w×h) 안으로 자른 정수 (x0, y0, x1, y1) [px]"""
    x0, y0, x1, y1 = (int(v) for v in xyxy)
    return max(x0, 0), max(y0, 0), min(x1, w - 1), min(y1, h - 1)


def tag_in_box(tags, box, ids, others=()):
    """퍽 박스 안(여유 10%)에 중심이 있는 퍽 태그(번호가 ids 안) → (번호, 꼭짓점 4×2) · 없으면 (None, None)
    others = 이 영상의 다른 퍽 박스들: 태그가 다른 박스에도 들고 그 박스 가운데에 더 가까우면(박스 크기로 나눈 거리) 그쪽 것
    (겹친 두 퍽에서 앞 퍽 태그가 뒤 퍽 번호로 붙지 않게)"""
    def inside(b, u, v):
        mx, my = (b[2] - b[0]) * 0.1, (b[3] - b[1]) * 0.1
        return b[0] - mx <= u <= b[2] + mx and b[1] - my <= v <= b[3] + my

    def off(b, u, v):  # 박스 가운데에서 떨어진 정도 (박스 폭·높이 단위)
        return np.hypot((u - (b[0] + b[2]) / 2) / max(b[2] - b[0], 1), (v - (b[1] + b[3]) / 2) / max(b[3] - b[1], 1))

    for tid, (u, v), _, corners in tags:
        if tid in ids and inside(box, u, v) and \
                not any(o != tuple(box) and inside(o, u, v) and off(o, u, v) < off(box, u, v) for o in others):
            return tid, corners
    return None, None


# ---------- 방향 (그리퍼로 집을 때 손을 돌릴 각도) — 추적 좌표(odom) 바닥 위 yaw [rad]
#   모양이 대칭이라 각도는 주기(period) 안에서만 뜻이 있음: 사각기둥 90° · 누운 퍽(긴 축) 180° · 서있는 원기둥은 방향 없음
def wrap_yaw(yaw, period):
    """yaw [rad]를 [−period/2, period/2)로 — 대칭 모양은 period만큼 돌려도 같은 자세"""
    return float((yaw + period / 2.0) % period - period / 2.0)


def tag_yaw(corners, size_m, fx, fy, cx, cy, rot):
    """태그 꼭짓점 [px] → (yaw [rad, 추적 좌표], 붙은 면 'side'/'top') · 못 구하거나 비스듬한 면이면 (None, '')
    solvePnP(IPPE_SQUARE — 정사각형 표식 전용)로 카메라 기준 태그 자세 → rot(카메라 → 추적 좌표)로 돌림.
      태그 법선(z축 — 태그 앞쪽)이 거의 수평 → 옆면 태그: yaw = 법선 방향 (사각기둥 옆면 · 누운 원기둥 바닥면 = 긴 축)
      거의 수직 → 윗면 태그: yaw = 태그 x축 방향 (사각기둥 윗면 — 태그를 면 모서리에 맞춰 붙였다고 봄)
    영상 왜곡은 0으로 봄 (RealSense 컬러는 왜곡이 작음)"""
    h = size_m / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)  # aruco 꼭짓점 순서와 같은 태그 좌표
    k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], np.float64)
    try:
        ok, rvec, _ = cv2.solvePnP(obj, np.asarray(corners, np.float64).reshape(4, 1, 2), k, None,
                                   flags=cv2.SOLVEPNP_IPPE_SQUARE)
    except cv2.error:
        return None, ""
    if not ok:
        return None, ""
    r_tag = rot @ cv2.Rodrigues(rvec)[0]  # 태그 축 → 추적 좌표
    n, xa = r_tag[:, 2], r_tag[:, 0]
    if abs(n[2]) < 0.5:   # 법선이 수평에서 30° 안 → 옆면
        return float(np.arctan2(n[1], n[0])), "side"
    if abs(n[2]) > 0.87:  # 법선이 수직에서 30° 안 → 윗면
        return float(np.arctan2(xa[1], xa[0])), "top"
    return None, ""       # 비스듬한 면 — 잘못 구한 자세일 수 있어 안 씀


def tag_pnp(corners, size_m, fx, fy, cx, cy):
    """태그 꼭짓점 [px] → 카메라 광학 좌표 기준 (rvec, tvec) · 못 구하면 None (solvePnP IPPE_SQUARE, 영상 왜곡 0으로 봄)
    tvec = 카메라 → 태그 가운데 [m] — depth 없이 태그 크기(size_m)만으로 나옴. 가까울수록(태그가 크게 찍힐수록) 정확"""
    hs = size_m / 2.0
    obj = np.array([[-hs, hs, 0], [hs, hs, 0], [hs, -hs, 0], [-hs, -hs, 0]], np.float64)
    k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], np.float64)
    try:
        ok, rvec, tvec = cv2.solvePnP(obj, np.asarray(corners, np.float64).reshape(4, 1, 2), k, None,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
    except cv2.error:
        return None
    return (rvec, tvec.reshape(3)) if ok else None


def tag_center(corners, depth_m, size_m, half_w, fx, fy, cx, cy, rot, trans, arm=None):
    """서 있는 퍽의 태그 꼭짓점 → 퍽 중심 (추적 좌표, [m]) · solvePnP 실패 · 비스듬한 면이면 None
    태그 안쪽 절반 depth의 중앙값(팔 depth 뺌)으로 태그 가운데의 3D 점 — 태그는 면 위에 있어 표면 → 중심 가정이 필요 없음.
      depth가 없으면(D435 최소 거리보다 가까움 — 집기 직전) solvePnP 위치(tvec)로 대신
      윗면 태그: 그 점이 곧 윗면 중심 (xy = 퍽 중심) · 옆면 태그: 면 법선 반대로 half_w [m] 들어감
    노드는 태그를 읽은 퍽에서 이 값을 박스 depth + 표면 보정(puck_center)보다 먼저 씀"""
    pts = np.asarray(corners, np.float64).reshape(4, 2)
    pnp = tag_pnp(pts, size_m, fx, fy, cx, cy)
    if pnp is None:
        return None
    rvec, tvec = pnp
    u, v = pts.mean(0)
    (x0, y0), (x1, y1) = pts.min(0), pts.max(0)
    qx, qy = (x1 - x0) / 4, (y1 - y0) / 4
    h, w = depth_m.shape[:2]
    sl = (slice(max(int(y0 + qy), 0), min(int(np.ceil(y1 - qy)), h)), slice(max(int(x0 + qx), 0), min(int(np.ceil(x1 - qx)), w)))
    patch = depth_m[sl]
    valid = patch[np.isfinite(patch) & (patch > 0.05) & ~arm_near(patch, arm[sl] if arm is not None else None)]
    if valid.size >= 3:
        z = float(np.median(valid))
        p = rot @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z]) + trans
    else:
        p = rot @ tvec + trans
    n = (rot @ cv2.Rodrigues(rvec)[0])[:, 2]  # 태그 법선 (추적 좌표)
    if abs(n[2]) > 0.87:  # 윗면
        return p
    if abs(n[2]) < 0.5:   # 옆면 — 법선은 면 바깥(카메라 쪽) · 수평 성분만
        nh = np.array([n[0], n[1], 0.0]) / max(float(np.hypot(n[0], n[1])), 1e-9)
        if nh @ (trans - p) < 0:  # 법선이 카메라 반대쪽으로 나왔으면 뒤집음
            nh = -nh
        return p - nh * half_w
    return None


def axis_yaw(points_xy, min_points, min_elong):
    """퍽 화소의 바닥 위 점들(추적 좌표 x·y) → 긴 축 yaw [rad] (주성분) · 점이 min_points보다 적거나 길쭉하지 않으면 None
    누운 퍽처럼 한 방향으로 길 때만 (긴 축 표준편차 ÷ 짧은 축 ≥ min_elong) — 정사각형에 가까우면 방향을 못 정함"""
    p = np.asarray(points_xy, float)
    if len(p) < min_points:
        return None
    w, v = np.linalg.eigh(np.cov((p - p.mean(0)).T))  # 고윳값 오름차순
    if w[0] <= 0 or np.sqrt(w[1] / w[0]) < min_elong:
        return None
    return float(np.arctan2(v[1, 1], v[0, 1]))


def find_box_tops(depth_m, rot, trans, fx, fy, cx, cy, floor_z, h_range, min_side, max_side, cell, step=4,
                  max_range=3.0, avoid=None, rois=None, mask_px=None):
    """depth 한 장 → 바닥 위 상자 윗면들 [(가운데 xy[2], 크기(긴 변, 짧은 변), yaw [rad, 주기 90°], 윗면 높이 z)] (추적 좌표)
    상자 = 205×205×70 골판지 상자 (붙여 놓으면 더 큰 판). 방법:
      ① depth를 step 화소마다 골라 추적 좌표 점으로 (max_range 안) · avoid(xy, 반경)이면 그 원 안(로봇 차체)은 뺌
      ② 바닥(floor_z)에서 h_range 높이(윗면 근처)인 점 → 바닥 격자(cell m)에 찍음 → 닫힘 연산으로 퍽 그림자 구멍을 메움
      ③ 덩어리마다 가장 작은 기울어진 사각형(cv2.minAreaRect) → 짧은 변 ≥ min_side, 긴 변 ≤ max_side만 상자
      ④ 사각형 안에 h_range보다 높은 점(0.15~0.6m 높이)이 많으면 버림 — 원통 방해물·다리처럼 위로 이어지는 물체 (상자 위 퍽은 작아서 괜찮음)
    rot·trans = 카메라 → 추적 좌표 (lookup_tf) · depth_m [m]
    rois = 상자 YOLO 네모들 [(x0, y0, x1, y1) 화소] — 주면 그 안의 점만 씀 (책상·벽을 상자로 잡지 않게)
           · 빈 목록이면 [] · None이면 화면 전체
    mask_px = 로봇 자기 팔을 화면에 투영한 점들 [[u, v, r], …] (arm_pixels) — 그 둘레(팔 두께 1.5배 + 6 px) 화소는 뺌
           (그리퍼가 상자 윗면과 같은 높이에 찍혀 상자에 합쳐지지 않게)
    윗면 점이 10개 미만이면 []"""
    if rois is not None and not len(rois):
        return []
    h, w = depth_m.shape[:2]
    vv, uu = np.mgrid[0:h:step, 0:w:step]
    z = depth_m[vv, uu]
    ok = (z > 0.15) & (z < max_range)
    if rois is not None:
        inside = np.zeros_like(ok)
        for x0, y0, x1, y1 in rois:
            inside |= (uu >= x0) & (uu <= x1) & (vv >= y0) & (vv <= y1)
        ok &= inside
    if mask_px is not None and len(mask_px):  # 자기 팔 둘레는 상자 점에서 뺌
        for u, v, r in mask_px:
            ok &= (uu - u) ** 2 + (vv - v) ** 2 > (1.5 * r + 6.0) ** 2
    if not ok.any():
        return []
    z, uu, vv = z[ok], uu[ok], vv[ok]
    pts = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z], -1) @ rot.T + trans
    if avoid is not None:
        pts = pts[np.hypot(pts[:, 0] - avoid[0][0], pts[:, 1] - avoid[0][1]) > avoid[1]]
    hz = pts[:, 2] - floor_z
    top = pts[(hz > h_range[0]) & (hz < h_range[1])]
    tall = pts[(hz > 0.15) & (hz < 0.6)]
    if len(top) < 10:
        return []
    org = top[:, :2].min(0) - cell * 2
    ij = ((top[:, :2] - org) / cell).astype(int)
    grid = np.zeros((ij[:, 1].max() + 3, ij[:, 0].max() + 3), np.uint8)
    grid[ij[:, 1], ij[:, 0]] = 255
    grid = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, lab = cv2.connectedComponents(grid)
    out = []
    for k in range(1, n):
        ys, xs = np.nonzero(lab == k)
        if len(xs) < 4:
            continue
        cells = np.stack([xs, ys], -1).astype(np.float32) * cell + org.astype(np.float32)
        (mx, my), (a, b), ang = cv2.minAreaRect(cells)
        a, b = a + cell, b + cell  # 격자 칸 가운데끼리 잰 길이 → 칸 하나만큼 넓힘
        if min(a, b) < min_side or max(a, b) > max_side:
            continue
        yaw = wrap_yaw(np.radians(ang), np.pi / 2)
        if len(tall):  # 사각형 안 높은 점 — 사각형 축으로 돌려서 안쪽인지 봄
            c, sn = np.cos(-yaw), np.sin(-yaw)
            d = tall[:, :2] - (mx, my)
            loc = np.stack([d[:, 0] * c - d[:, 1] * sn, d[:, 0] * sn + d[:, 1] * c], -1)
            inside = (abs(loc[:, 0]) < a / 2) & (abs(loc[:, 1]) < b / 2)
            if inside.sum() > 0.3 * len(xs):  # 윗면 칸 수에 비해 높은 점이 많음 → 상자 아님
                continue
        top_z = float(np.median(top[(abs(top[:, 0] - mx) < a / 2) & (abs(top[:, 1] - my) < b / 2), 2]))
        out.append((np.array([mx, my]), (max(a, b), min(a, b)), yaw, top_z))
    return out


def tag_on_box(tag_xy, box, margin):
    """장소 태그 위치(xy)가 상자 윗면 사각형(find_box_tops 결과 하나)을 margin [m]만큼 넓힌 안에 있는가 → True/False"""
    c, (a, b), yaw, _ = box
    d = np.asarray(tag_xy, float) - c
    if yaw is None:  # 잘린 상자(방향 모름, box_cut) — 사각형을 돌릴 수 없으니 가운데에서 대각선 반 + margin 안이면 그 상자
        return float(np.hypot(d[0], d[1])) < a / 2 * np.sqrt(2.0) + margin
    co, sn = np.cos(-yaw), np.sin(-yaw)
    lx, ly = d[0] * co - d[1] * sn, d[0] * sn + d[1] * co
    return abs(lx) < a / 2 + margin and abs(ly) < a / 2 + margin  # 긴 변 기준 (축 구분 없이 넉넉하게)


def box_cut(box, rot, trans, fx, fy, cx, cy, w, h, edge_px):
    """상자 윗면 사각형(find_box_tops 결과 하나)이 화면 가장자리에 잘렸는가 — 네 꼭짓점을 화면에 다시 투영해
    하나라도 가장자리 edge_px 안쪽 띠(또는 화면 밖·카메라 뒤)에 있으면 True.
    잘린 상자는 보이는 일부로 사각형을 그려서 방향이 크게 틀림
    → 호출하는 쪽에서 방향을 '모름'(None)으로 — 다른 카메라나 앞서 온전히 본 방향을 덮지 않게 (fuse는 None이면 옛 방향 유지)"""
    (mx, my), (la, lb), yaw, top_z = box
    c, s = np.cos(yaw), np.sin(yaw)
    for dx, dy in ((la, lb), (-la, lb), (-la, -lb), (la, -lb)):
        p = np.array([mx + (dx * c - dy * s) / 2, my + (dx * s + dy * c) / 2, top_z])
        q = rot.T @ (p - trans)  # 추적 좌표 → 카메라
        if q[2] < 0.05:
            return True
        u, v = fx * q[0] / q[2] + cx, fy * q[1] / q[2] + cy
        if not (edge_px <= u <= w - 1 - edge_px and edge_px <= v <= h - 1 - edge_px):
            return True
    return False


def to_msg(tid, tr):
    """트랙(또는 한 검출) dict와 id → TrackedObject — 위치 공분산 3×3을 pose.covariance에.
    방향: yaw를 알면 pose.orientation(바닥 위 회전만) + 공분산 yaw 대각 YAW_VAR_KNOWN, 모르면 단위 쿼터니언 + ROT_VAR_UNKNOWN.
    details: posture · state(신호등) · shape · yaw_period[deg] · distance[m] · tag · content — 값이 있을 때만"""
    o = TrackedObject()
    o.id = tid
    o.class_name = tr["class_name"]
    if o.class_name == "puck" and tr.get("posture") == "lying":  # 트랙은 퍽 계열을 'puck'으로 저장 → 발행은 설계 이름 puck / puck_lying
        o.class_name = "puck_lying"
    o.confidence = float(tr.get("conf", 0.5))
    o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = (float(v) for v in tr["p"])
    yaw = tr.get("yaw")
    o.pose.pose.orientation.w = 1.0
    if yaw is not None:  # z축 회전만 — (0, 0, sin(yaw/2), cos(yaw/2))
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = float(np.sin(yaw / 2)), float(np.cos(yaw / 2))
    cov = np.zeros((6, 6))
    cov[:3, :3] = tr["P"]
    cov[3, 3] = cov[4, 4] = ROT_VAR_UNKNOWN  # 기울기(roll·pitch)는 추정 안 함
    cov[5, 5] = YAW_VAR_KNOWN if yaw is not None else ROT_VAR_UNKNOWN
    o.pose.covariance = [float(v) for v in cov.ravel()]
    o.size.x, o.size.y, o.size.z = (float(v) for v in tr["size"])
    o.last_seen = tr["last_seen"]
    # 본 카메라를 마지막 본 시각 순으로 — 끝은 px를 잰 카메라 (같은 시각이면 시각만으론 순서를 모름)
    seen, last = tr.get("seen", {}), tr.get("camera", "")
    o.cameras = [c for c in sorted(seen, key=seen.get) if c != last] + ([last] if last else [])
    o.px = [int(v) for v in tr.get("px", (0, 0))]
    o.color = tr.get("color", "")
    details = []
    if o.class_name in ("puck", "puck_lying") and tr.get("posture"):  # 퍽 자세 (color와 별도)
        details.append(KeyValue(key="posture", value=tr["posture"]))
    elif o.class_name == "traffic_light" and o.color:  # 신호등 상태 (color에도 같이 있음)
        details.append(KeyValue(key="state", value=o.color))
    if tr.get("shape"):  # 퍽 모양 box(사각기둥) · cyl(원기둥)
        details.append(KeyValue(key="shape", value=tr["shape"]))
    if yaw is not None:  # 방향 대칭 주기 [deg] — 이만큼 돌려도 같은 자세 (90 = 사각기둥·상자 윗면 · 180 = 누운 퍽 · 360 = 장소 태그)
        details.append(KeyValue(key="yaw_period", value=str(int(round(np.degrees(tr["yaw_period"]))))))
    if tr.get("distance") is not None:  # 로봇(base_footprint) 중심 ~ 물체 중심 바닥 위 거리 [m] — 발행 순간
        details.append(KeyValue(key="distance", value=f'{tr["distance"]:.3f}'))
    if tr.get("tag") is not None:  # AprilTag 번호 — 퍽 0~11 · 장소 상자 12~14
        details.append(KeyValue(key="tag", value=str(tr["tag"])))
    if tr.get("content"):  # 바코드 문자열 (장소 작업 지시)
        details.append(KeyValue(key="content", value=tr["content"]))
    o.details = details
    return o


# 노드 파라미터 (이름, 기본값, self 속성 이름, 변환) — 속성이 None이면 쓰는 곳에서 직접 읽음
PARAMS = (
    ("rate_hz", 10.0, "rate_hz", float),                  # /tracked_objects 발행 주기 [Hz]
    ("model_path", "", None, str),                        # YOLO 가중치 ('' = 설치 폴더 model.pt · 이름만 = 설치 폴더에서 · 경로)
    ("confidence", 0.4, "conf_th", float),                # YOLO 신뢰도 하한
    ("imgsz", 416, "imgsz", int),                         # YOLO 입력 크기 [px]
    ("device", "cpu", "device", str),                     # 추론 장치 (cpu · cuda 등)
    ("cpu_threads", 2, None, int),                        # cpu 추론 때 torch 스레드 수
    ("sync_slop", 0.05, None, float),                     # 컬러·depth 시각 차 허용 [s]
    ("match_dist", 0.4, "match_dist", float),             # 검출을 트랙에 붙이는 최대 거리 [m]
    ("puck_center_offset", 0.015, "puck_center_offset", float),    # [m] 퍽 표면 → 중심 (반지름·반폭) — puck_center (0 = 끔)
    ("puck_center_full_deg", 45.0, "puck_center_full_deg", float),  # 이보다 얕게 내려다보면 offset 전부 · 바로 위면 0 [deg]
    ("tag_center", True, "tag_center", bool),             # 태그를 읽은 서 있는 퍽은 태그 가운데로 중심 (tag_center)
    ("det_merge_iou", 0.5, "det_merge_iou", float),       # 한 프레임에서 겹친 같은 퍽 박스를 하나로 (0 = 끔) — merge_overlap_dets
    ("track_merge_m", 0.03, "track_merge_m", float),      # 이 안의 같은 퍽 트랙 둘을 하나로 [m] (0 = 끔) — merge_tracks
    ("pixel_sigma", 4.0, "pixel_sigma", float),           # 화면 위치 오차 [px] (가로·세로 σ = pixel_sigma·z/f)
    ("depth_sigma0", 0.005, "depth_sigma0", float),       # depth 오차 상수항 [m]
    ("depth_sigma_k", 0.01, "depth_sigma_k", float),      # depth 오차 거리² 계수 [1/m] (σz = sigma0 + k·z²)
    ("process_noise", 0.01, "q", float),                  # 정지 물체 예측 잡음 [m²/s]
    ("process_noise_dynamic", 1.0, "q_dyn", float),       # 움직이는 물체(사람) 예측 잡음 [m²/s]
    ("track_frame", "odom", "track_frame", str),          # 추적·융합 좌표계 (끊김 없음) — 발행은 map
    ("self_filter", True, "self_filter", bool),           # 로봇 자기 몸(차체·팔) 검출을 버림
    ("base_frame", "base_footprint", "base_frame", str),  # 로봇 차체 프레임 (바닥 높이 · 거리 기준)
    ("self_links", ["shoulder_link", "upper_arm_link", "forearm_link", "wrist_link", "gripper_link", "end_effector_link"],
     "self_links", lambda v: [str(x) for x in v]),        # 팔 체인 링크 (순서대로)
    ("self_radius", 0.06, "self_radius", float),          # 팔 선분에서 이 안이면 자기 팔 [m]
    ("body_radius", 0.2, "body_radius", float),           # 차체 원점에서 수평 이 안이면 차체 [m] (body_box 없을 때)
    ("body_height", 0.6, "body_height", float),           # 차체로 보는 높이 상한 [m]
    ("body_box", [0.0], "body_box", lambda v: [float(x) for x in v]),  # [x_min, x_max, y_half] base_frame 직사각형 [m] — 3개면 body_radius 대신 씀
    ("self_arm_hits", 3, "self_arm_hits", int),           # 퍽이 아닌 검출 박스 안에 팔 투영점이 이만큼 있으면 자기 팔 (0 = 안 봄)
    ("agnostic_nms", False, "agnostic_nms", bool),        # True면 클래스가 달라도 겹친 박스는 하나만 (한 퍽에 서있음·누움 박스가 같이 나올 때)
    ("trust_model_posture", False, "trust_model_posture", bool),  # True면 퍽 자세를 모델 이름(puck_<색>_<모양>_lying)으로 (높이·크기 판정 대신)
    ("yaw_min_aspect", 1.3, "yaw_min_aspect", float),     # OBB 장축÷단축이 이보다 작으면 끝면이 보이는 것 — 누운 퍽 축 yaw 안 냄
    ("fine_min_px", 30, "fine_min_px", float),            # 퍽 박스 긴 변이 이보다 작으면(멂) 모양·모델 자세는 안 냄 [px]
    # 퍽 판단 (judge_puck · settle_puck_color) — 크기 기준은 실물 측정값 · 원기둥·사각기둥 같은 기준
    ("puck_colors", ["red", "green"], "puck_colors", lambda v: [str(x) for x in v]),  # 퍽으로 볼 색 (빨강·초록)
    ("hue_tol", 10, "hue_tol", int),                      # 기준 H에서 이만큼 안이어야 그 색 (주황 H12~13은 빨강 밖)
    ("check_model_color", True, "check_model_color", bool),  # YOLO 이름의 색(puck_red)을 색 결정에 씀 (settle_puck_color: 애매·어두우면 YOLO 색)
    ("size_long_min_cm", 2.5, "long_min", float),         # 박스 긴 변 하한 [cm] (끝면·윗면 3×3만 보이면 2.5~3.9)
    ("size_long_max_cm", 10.0, "long_max", float),        # 긴 변 상한 [cm] (누운 퍽 6.5~7.2 · 멀면 박스가 넉넉해져 8.5)
    ("size_short_min_cm", 2.2, "short_min", float),       # 짧은 변 하한 [cm] (빨대 1.0~1.5 거름)
    ("square_long_cm", 4.5, "square_long", float),        # 긴 변이 이보다 작고 정사각형에 가까우면 끝면·윗면 — 자세 모름 [cm]
    ("square_ratio", [0.8, 1.25], "square_ratio", lambda v: [float(x) for x in v]),  # 정사각형으로 보는 높이÷폭 범위
    ("stand_ratio", 0.8, "stand_ratio", float),           # 높이÷폭 이 이상이면 서있음, 미만이면 누움
    ("edge_px", 5, "edge_px", int),                       # 박스가 화면 가장자리 이 안에 닿으면 잘린 퍽 — 크기 하한·자세 안 봄 [px]
    ("near_px", 70, "near_px", int),                      # 깊이 없는데 박스 짧은 변이 이 이상이면 너무 가까운 퍽 (3cm 퍽 70px ≈ 25cm) [px]
    ("depth_min_valid", 0.5, "depth_min_valid", float),   # 박스 가운데 깊이 화소가 이 비율보다 적으면 깊이 없음
    # 표식 — 퍽 번호는 어느 퍽에 붙을지 무작위 → 번호로 색을 정하지 않음 (색은 영상으로)
    ("puck_tags", list(range(12)), "puck_tag_ids", lambda v: {int(x) for x in v}),  # 퍽 AprilTag 번호 (0~11)
    ("location_tags", [12, 13, 14], "location_tags", lambda v: {int(x) for x in v}),  # 장소 AprilTag 번호 (L1~L3)
    # 퍽 방향 (그리퍼 손 각도) — 태그 자세(tag_yaw) 우선, 없으면 누운 퍽은 퍽 화소 점들의 긴 축(axis_yaw)
    ("puck_tag_size_m", 0.016, "puck_tag_size", float),  # 퍽 태그 검은 사각형 한 변 [m] (발제 16mm)
    ("yaw_min_points", 30, "yaw_min_points", int),        # 긴 축을 구할 최소 점 수 (깊이 있는 퍽 색 화소)
    ("yaw_min_elong", 1.8, "yaw_min_elong", float),       # 긴 축 ÷ 짧은 축 (표준편차) 이 이상이어야 긴 축 방향을 믿음 (누운 퍽 60×30mm = 2)
    ("location_tag_size_m", 0.048, "location_tag_size", float),  # 장소 태그 검은 사각형 한 변 [m] (흰 여백 뺀 48mm)
    # 상자 (find_box_tops) — 205×205×70 mm 골판지 상자 · 붙이면 더 큰 판
    ("box_detect", True, "box_detect", bool),             # depth로 상자 윗면 찾기 (False면 장소 태그 위치만)
    ("box_top_height_m", [0.05, 0.09], "box_h", lambda v: [float(x) for x in v]),  # 윗면으로 볼 바닥 위 높이 범위 [m] (상자 70mm)
    ("box_min_side_m", 0.17, "box_min_side", float),      # 짧은 변이 이보다 작으면 상자 아님 [m] (205mm − 여유 · 일부만 보인 윗면도 버림)
    ("box_max_side_m", 1.0, "box_max_side", float),       # 긴 변이 이보다 크면 상자 아님 [m] (4개 붙인 820mm까지)
    ("box_cell_m", 0.02, "box_cell", float),              # 바닥 격자 한 칸 [m]
    ("box_tag_margin_m", 0.08, "box_tag_margin", float),  # 장소 태그가 윗면 사각형에서 이만큼 안이면 그 상자의 태그 [m]
    ("box_edge_px", 0, "box_edge_px", float),             # 윗면 꼭짓점이 화면 밖(가장자리 이 화소 안 포함)이면 잘린 상자 → 방향 모름 [px]
    ("location_box_size", [0.205, 0.205, 0.07], "location_box_size",
     lambda v: np.array([float(x) for x in v])),          # 윗면을 못 찾았을 때 장소 상자 크기 [m] — 태그에서 반 변만큼 안쪽이 가운데
    # 상자 전용 YOLO — 있으면 그 네모 안에서만 depth 윗면을 찾음 · 비면 화면 전체
    ("box_model_path", "", None, str),                    # 예: bee_yolo11n_box_v1.pt (model_path처럼 이름만 쓰면 설치 폴더에서)
    ("box_conf", 0.5, "box_conf", float),                 # 상자 YOLO 신뢰도 하한
    ("box_roi_margin", 0.1, "box_roi_margin", float),     # 상자 네모를 이 비율만큼 넓혀서 씀 (네모가 윗면 끝을 살짝 자를 때)
    # 주사위 완성 사진 — 요청이 오면 장소 상자 윗면을 위에서 본 사진으로 펴서 발행 (on_capture_dice)
    ("dice_image_px", 480, "dice_px", int),               # 상자 윗면을 반듯하게 편 정사각형 사진 한 변 [px]
    ("dice_image_margin", 0.1, "dice_margin", float),     # 윗면 둘레를 이 비율만큼 더 담음 (가장자리 퍽이 잘리지 않게)
    ("tag_gamma", 2.5, "tag_gamma", float),               # AprilTag를 이 감마로 어둡게 편 영상에서 한 번 더 찾음 (0 = 끔) — detect_tags
    ("tag_half", True, "tag_half", bool),                 # 반 크기 영상에서도 한 번 더 찾음 — 가까워 크게 찍힌 태그 (detect_tags)
    # 태그 후보 최소 둘레 [px] — 이보다 작은 사각형은 검사 안 함 (CPU를 크게 줄임). 16 px = 한 변 4 px, 이보다 작은 태그는 원래 못 읽음.
    # 실물 녹화 155장 + Isaac 800장: 예전(6.4 · 3.2 px)과 찾은 태그 같음 · 태그 찾기 시간 약 1/5 (11/11)
    ("tag_min_perimeter_px", 16.0, "tag_min_px", float),       # 원본·감마 영상
    ("tag_half_min_perimeter_px", 24.0, "tag_half_min_px", float),  # 반 크기 영상 (크게 찍힌 태그용)
    ("barcode_once", True, "barcode_once", bool),         # 바코드를 한 번 읽으면 그 뒤로 바코드 찾기를 안 하고 읽은 내용만 0.5초마다 다시 냄 (CPU 절약)
)


class PerceptionNode(Node):
    def __init__(self):
        # 파라미터 읽기 · 모델 로드 · 구독/발행/타이머 만들기
        super().__init__("perception")
        for name, default, attr, conv in PARAMS:
            self.declare_parameter(name, default)
            if attr:
                setattr(self, attr, conv(self.get_parameter(name).value))
        self.body_box = self.body_box if len(self.body_box) == 3 else []  # 3개가 아니면 body_radius를 씀
        self._out_tf = None  # 마지막으로 성공한 map ← track_frame (회전, 이동) — TF가 잠깐 끊겨도 이걸로
        self.tag_det = tag_detector()
        self.barcode = barcode_detector()

        model_path = str(self.get_parameter("model_path").value)
        try:
            from ament_index_python import get_package_share_directory
            model_dir = os.path.join(get_package_share_directory("perception"), "model")
        except Exception:  # noqa: BLE001
            model_dir = ""
        model_path = resolve_model_path(model_path, model_dir)
        self.model = None
        self.names = dict(enumerate(CLASSES))  # 클래스 번호 → 이름 (모델이 있으면 모델 것으로)
        self.model_path = model_path
        if model_path:
            try:
                from ultralytics import YOLO
                if self.device == "cpu":  # torch는 기본으로 코어를 전부 씀 → 같은 머신의 제어·Nav2가 밀리지 않게 제한
                    import torch
                    torch.set_num_threads(max(1, int(self.get_parameter("cpu_threads").value)))
                self.model = YOLO(model_path)
                self.names = dict(self.model.names)
                self.get_logger().info(f"YOLO 모델 로드: {model_path} (device={self.device})")
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f"YOLO 로드 실패 ({e}) — 빈 목록 발행")
        else:
            self.get_logger().warning("model_path 없음 — 빈 목록 발행 (기동 검사용)")
        self.box_model = None
        box_path = resolve_model_path(str(self.get_parameter("box_model_path").value), model_dir) \
            if str(self.get_parameter("box_model_path").value) else ""
        if box_path:
            try:
                from ultralytics import YOLO
                self.box_model = YOLO(box_path)
                self.get_logger().info(f"상자 YOLO 모델 로드: {box_path}")
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f"상자 YOLO 로드 실패 ({e}) — 화면 전체에서 상자를 찾음")

        self.bridge = CvBridge()
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.lock = threading.Lock()
        # 단계별 처리 시간 — (카메라, 단계) → [합 ms, 횟수, 최대 ms] · on_diag가 1초마다 평균·최대로 내보내고 비움 (prof_add)
        self.prof = {}
        self.prof_last = {}  # 카메라 → 직전 추론 시작 시각 (perf_counter) — 추론 간격(실제 Hz) 재기용
        self.frames = {}  # cam → dict(bgr, depth_mm, fx, fy, cx, cy, frame_id, stamp)
        self.tracks = {}  # id → dict(class_name(퍽 계열은 puck), posture, p[3], P[3×3], t, size, last_seen, seen{cam: t}, ...)
        self.next_id = 1
        self.published = 0
        self.synced = {}       # cam → 동기화된 컬러·depth 쌍 수
        self.tf_fallback = 0   # 영상 시각 TF가 없어 최신 TF로 대신한 횟수
        self.self_dropped = 0  # 자기 몸(팔·차체)이라 버린 검출 수
        # 바코드 → /mission/qr (토픽 이름은 옛 QR 미션 그대로 — 지금은 바코드 원문을 mission_manager로 보내는 통로)
        self.qr_text = ""      # 마지막으로 /mission/qr에 낸 바코드 내용
        self.barcode_text = ""  # 읽은 바코드 내용 — barcode_once면 이게 차면 더 안 읽고 이 값만 다시 냄
        self.qr_seen = 0.0   # 마지막으로 바코드를 읽은 시각 — QR_GAP_S 넘게 안 보였다 다시 보이면 같은 내용도 재발행
        # 주사위 완성 사진 — 늦게 붙어도 받게 transient_local · 요청 /mission/capture_dice (Int32 태그 번호, 0 = 가장 가까운 장소 상자)
        dice_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, history=QoSHistoryPolicy.KEEP_LAST)
        self.dice_pub = self.create_publisher(Image, "/mission/dice_image", dice_qos)
        self.dice_raw_pub = self.create_publisher(Image, "/mission/dice_image_raw", dice_qos)
        self.create_subscription(Int32, "/mission/capture_dice", self.on_capture_dice, 10)
        self.qr_pub = self.create_publisher(
            String, "/mission/qr",
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, history=QoSHistoryPolicy.KEEP_LAST))

        # 컬러·정렬 depth는 시각을 맞춘 쌍으로만 받는다 (다른 순간의 컬러·depth를 섞지 않게) — camera_info는 고정값이라 따로
        slop = float(self.get_parameter("sync_slop").value)
        self._syncs = []
        for cam in ("pole_front_cam", "gripper_cam"):
            color_sub = message_filters.Subscriber(self, Image, f"/{cam}/color/image_raw", qos_profile=sensor_qos())
            depth_sub = message_filters.Subscriber(self, Image, f"/{cam}/aligned_depth_to_color/image_raw",
                                                   qos_profile=sensor_qos())
            sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=5, slop=slop)
            sync.registerCallback(lambda c, d, cam=cam: self.on_frame(cam, c, d))
            self._syncs.append((color_sub, depth_sub, sync))
            self.create_subscription(CameraInfo, f"/{cam}/color/camera_info",
                                     lambda m, c=cam: self.on_info(c, m), sensor_qos())
        self.pub = self.create_publisher(TrackedObjectArray, "/tracked_objects", 5)
        self.det_pubs = {cam: self.create_publisher(TrackedObjectArray, f"/perception/{cam}/detections", 5)
                         for cam in ("pole_front_cam", "gripper_cam")}
        # 디버그 영상 (박스 그린 컬러 영상) — RViz Image · rqt_image_view로 봄. 구독자가 없으면 그리지 않음
        self.debug_pubs = {cam: self.create_publisher(Image, f"/perception/{cam}/debug_image", sensor_qos())
                           for cam in ("pole_front_cam", "gripper_cam")}
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 1)
        self.create_timer(1.0 / max(self.rate_hz, 0.1), self.on_publish)
        self.create_timer(0.25, self.on_infer)  # 카메라당 최대 4Hz 추론
        self.create_timer(1.0, self.on_diag)
        self.create_timer(0.5, self.read_barcodes)  # 바코드 2Hz

    # ---------- 입력 캐시
    def _slot(self, cam):  # 카메라별 최신 프레임 dict (없으면 만듦)
        with self.lock:
            return self.frames.setdefault(cam, {})

    def on_frame(self, cam, color, depth):
        """동기화된 컬러·depth 한 쌍을 카메라별 최신 프레임으로 저장 — 시각·프레임은 컬러 기준 (depth는 컬러에 정렬된 영상 [mm]).
        변환 실패면 경고만 하고 버림"""
        try:
            bgr = self.bridge.imgmsg_to_cv2(color, desired_encoding="bgr8")
            d = self.bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"{cam} 변환 실패: {e}", throttle_duration_sec=5.0)
            return
        s = self._slot(cam)
        with self.lock:
            s["bgr"] = np.asarray(bgr)
            s["depth_mm"] = np.asarray(d)
            s["stamp"] = color.header.stamp
            s["frame_id"] = color.header.frame_id or f"{cam}_color_optical_frame"
            self.synced[cam] = self.synced.get(cam, 0) + 1

    def on_info(self, cam, msg):  # camera_info → 내부 파라미터 fx · fy · cx · cy [px]
        s = self._slot(cam)
        s["fx"], s["fy"] = float(msg.k[0]), float(msg.k[4])
        s["cx"], s["cy"] = float(msg.k[2]), float(msg.k[5])

    # ---------- 추론 (타이머 4Hz, 카메라당 최대 4Hz)
    def on_infer(self):
        # 새 프레임이 있는 카메라마다 YOLO 추론 → update_tracks · 모델이 없으면 아무것도 안 함
        if self.model is None:
            return
        # 간격 검사는 '추론 시작 시각' 기준 0.2초 — 타이머(0.25초)와 같은 값이면 아슬아슬하게 못 넘겨 한 번씩 건너뜀
        with self.lock:
            cams = [(c, dict(s)) for c, s in self.frames.items()
                    if s.get("bgr") is not None and s.get("depth_mm") is not None
                    and s.get("fx") and time.time() - s.get("infer_t", 0.0) > 0.2]
        for cam, s in cams:
            t_start = time.time()
            t0 = time.perf_counter()  # 처리 시간 재기 시작 (YOLO · 전체)
            if cam in self.prof_last:  # 직전 추론 시작부터 지금까지 = 추론 간격 (0.25초 목표 · 길면 밀리는 중)
                self.prof_add(cam, "간격", self.prof_last[cam], t0)
            self.prof_last[cam] = t0
            try:
                res = self.model.predict(s["bgr"], imgsz=self.imgsz, conf=self.conf_th, agnostic_nms=self.agnostic_nms,
                                         device=self.device, verbose=False)[0]
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f"추론 실패: {e}", throttle_duration_sec=5.0)
                continue
            self.prof_add(cam, "YOLO", t0)
            with self.lock:
                self.frames[cam]["infer_t"] = t_start
            self.update_tracks(cam, s, res)
            self.prof_add(cam, "전체", t0)  # YOLO + 태그 + 상자 + 융합 + 발행 (한 카메라 한 프레임)

    def prof_add(self, cam, key, t0, t1=None):
        """처리 시간 기록: perf_counter 값 t0부터 t1(없으면 지금)까지 [ms] → on_diag가 1초마다 카메라·단계별 평균/최대로 냄"""
        ms = ((time.perf_counter() if t1 is None else t1) - t0) * 1000.0
        with self.lock:
            p = self.prof.setdefault((cam, key), [0.0, 0, 0.0])
            p[0] += ms
            p[1] += 1
            p[2] = max(p[2], ms)

    def update_tracks(self, cam, s, res):
        # 한 카메라 한 프레임 처리: YOLO 결과(res) · 태그 · depth 상자 → 검출 목록 → 트랙에 융합(fuse) · 정리(prune)
        # → /perception/<cam>/detections 발행 (+ 디버그 영상). TF가 없으면 끝 (위치를 못 구함)
        h, w = s["bgr"].shape[:2]
        depth_m = s["depth_mm"].astype(np.float32) / 1000.0
        now_stamp = s["stamp"]
        tf = self.lookup_tf(s["frame_id"], now_stamp)
        obb = getattr(res, "obb", None)  # OBB 모델은 res.obb — xyxy·cls·conf는 같고 회전 꼭짓점 xyxyxyxy가 더 있음 · 보통 모델은 res.boxes
        boxes_in = obb if obb is not None else res.boxes
        if tf is None:
            return
        rot, trans = tf
        t_tag = time.perf_counter()
        tags = detect_tags(self.tag_det, s["bgr"], self.tag_gamma, self.tag_half, self.tag_min_px, self.tag_half_min_px)
        self.prof_add(cam, "AprilTag", t_tag)
        # 로봇 몸(차체·팔) — 자기 몸 필터와 상자 찾기에 같이 씀
        body = self.robot_body(now_stamp) if self.self_filter and (len(boxes_in) or self.box_detect) else None
        # 팔을 화면에 투영: 박스 일부만 팔이면 depth 중앙값이 팔 뒤 바닥·벽이라 3D 판정(is_self)을 빠져나감
        # → 투영한 팔이 (퍽이 아닌) 박스 안에 있으면 버림
        arm_all = arm_pixels(body[1], rot, trans, s["fx"], s["fy"], s["cx"], s["cy"]) if body is not None else np.zeros((0, 3))
        arm_px = arm_all if self.self_arm_hits > 0 else np.zeros((0, 3))  # 퍽 박스에 팔이 걸렸나 (self_arm_hits 0이면 안 봄)
        arm_mask = arm_mask_image(arm_px, depth_m.shape) if len(arm_px) else None  # 퍽 depth에서 손가락을 뺄 때
        dets = []
        # 바닥 높이 = 로봇 base_footprint 높이 (TF 없으면 추적 좌표 0) — 퍽 자세(높이 판정)와 상자 윗면 찾기에 같이 씀
        base_now = self.lookup_tf(self.base_frame, now_stamp, quiet=True)
        floor_z = float(base_now[1][2]) if base_now is not None else 0.0
        side_view = view_from_side(rot)  # 옆에서 봄 → 높이로 자세를 모를 때 가로·세로 비율 판정을 씀
        s["drops"] = drops = []  # 버린 퍽 박스와 이유 — 디버그 영상에 회색 박스 옆에 씀
        puck_boxes = [clip_box(b.xyxy[0].tolist(), w, h) for b in boxes_in  # 태그를 겹친 퍽 중 어디에 붙일지 (tag_in_box)
                      if split_class(self.names.get(int(b.cls[0]), ""))[0] == "puck"]
        for b in boxes_in:
            tag, tag_c = None, None
            shape, yaw, yaw_period = "", None, 0.0  # 퍽만 채움
            name = self.names.get(int(b.cls[0]))
            if name is None:
                continue
            cls, model_color = split_class(name)  # 모델 이름(puck_red) → 설계 이름(puck) + 이름에 든 색
            if cls == "box":
                continue  # OBB 모델의 상자 클래스 — 상자는 depth 윗면(start_box · location_box)으로 따로 찾음
            x0, y0, x1, y1 = clip_box(b.xyxy[0].tolist(), w, h)
            if x1 <= x0 or y1 <= y0:
                continue
            posture = ""
            if cls == "puck":
                # 퍽 판단 — 깊이(min_valid 비율) · 색 · 크기·자세 · 잘림·근접 (자세는 YOLO 이름이 아니라 높이·크기로 — trust_model_posture 끔일 때)
                zm = box_depth_m(depth_m, (x0, y0, x1, y1), self.depth_min_valid, arm_mask)
                if zm <= 0 and self.tag_center:  # 너무 가까워 depth 없음 → 퍽 태그가 보이면 그 거리(solvePnP)로 대신
                    _, corners = tag_in_box(tags, (x0, y0, x1, y1), self.puck_tag_ids, puck_boxes)
                    pnp = tag_pnp(corners, self.puck_tag_size, s["fx"], s["fy"], s["cx"], s["cy"]) if corners is not None else None
                    if pnp is not None and pnp[1][2] > 0:
                        zm = float(pnp[1][2])
                roi = s["bgr"][y0:y1, x0:x1]
                color, _ = puck_color(roi, self.puck_colors, self.hue_tol)
                color, _ = settle_puck_color(roi, color, model_color, self.puck_colors, self.check_model_color)
                if color is None:  # 분명히 다른 색 · 색도 없고 YOLO 색도 없음
                    drops.append(((x0, y0, x1, y1), "color"))
                    continue
                m = self.edge_px
                clipped = x0 <= m or y0 <= m or x1 >= w - 1 - m or y1 >= h - 1 - m
                near = min(x1 - x0, y1 - y0) >= self.near_px
                wcm = (x1 - x0) * zm / s["fx"] * 100 if zm > 0 else -1.0
                hcm = (y1 - y0) * zm / s["fy"] * 100 if zm > 0 else -1.0
                ok, why, posture = self.judge_puck(wcm, hcm, color, clipped, "", near)  # 색은 위에서 정함 → 크기·깊이·자세만
                # 자세: 윗면 높이로 먼저 (카메라 각도와 상관없음) · 모르면 옆에서 볼 때만 가로·세로 비율 판정을 씀
                #   (위에서 내려다보면 서 있는 퍽도 옆면이 같이 보여 박스가 길어짐 → 비율로는 '누움'으로 틀림)
                if ok and zm > 0 and not clipped:
                    h_post, _ = posture_from_height(depth_m, (x0, y0, x1, y1), zm, rot, trans, s["fx"], s["fy"], s["cx"], s["cy"],
                                                    floor_z, (0.0, float(self.location_box_size[2])))
                    posture = h_post or (posture if side_view else "")
                fine = box_long_px(b, (x0, y0, x1, y1)) >= self.fine_min_px  # 멀어 작으면 모양·자세를 픽셀로 못 가림
                m_posture = model_posture(name) if fine else ""
                if self.trust_model_posture:
                    # 크기는 퍽인지 거르는 데만 · 자세는 모델 이름 (OBB 모델은 누운 퍽 박스가 돌아가 있어 가로÷세로로는 못 봄)
                    # 이 설정을 켜면 위 높이 판정보다 모델 자세가 우선 (박스가 작으면 '' = 모름)
                    posture = m_posture
                if not ok or zm <= 0:
                    # 퍽 아님 → 버림 · 퍽인데 깊이 없음(너무 가까움) → 위치를 몰라 이번엔 갱신 안 함 = 트랙은 마지막 위치 유지
                    # (2초 규칙은 그대로)
                    drops.append(((x0, y0, x1, y1), f"{why or 'no depth'} {wcm:.1f}x{hcm:.1f}cm{' clip' if clipped else ''}"))
                    continue
                if posture == "lying":
                    cls = "puck_lying"  # 발행 이름은 설계대로 puck / puck_lying
                tag, corners = tag_in_box(tags, (x0, y0, x1, y1), self.puck_tag_ids, puck_boxes)  # 번호만 붙임 — 색은 위에서 영상으로 정한 그대로
                tag_y, face = (tag_yaw(corners, self.puck_tag_size, s["fx"], s["fy"], s["cx"], s["cy"], rot)
                               if corners is not None else (None, ""))
                shape = tag_shape(face, posture) or (puck_shape(name) if fine else "")  # 몸통 면 태그가 보이면 YOLO 모양보다 우선
                tag_c = (tag_center(corners, depth_m, self.puck_tag_size, self.puck_center_offset, s["fx"], s["fy"], s["cx"], s["cy"], rot, trans, arm_mask)
                         if corners is not None and posture != "lying" and self.tag_center else None)
                yaw, yaw_period = None, 0.0
                if posture == "lying" and hasattr(b, "xyxyxyxy"):  # OBB 모델: 회전 박스 장축 → 누운 퍽 축 (depth 점보다 먼저)
                    c3 = rot @ np.array([((x0 + x1) / 2 - s["cx"]) * zm / s["fx"], ((y0 + y1) / 2 - s["cy"]) * zm / s["fy"], zm]) + trans
                    yaw = lying_yaw(b.xyxyxyxy[0].tolist(), s["fx"], s["fy"], s["cx"], s["cy"], rot, trans, c3[2], self.yaw_min_aspect)
                    yaw_period = np.pi if yaw is not None else 0.0
                if yaw is None:
                    yaw, yaw_period = self.puck_yaw(s, depth_m, (x0, y0, x1, y1), color, shape, posture, tag_y, face, rot, trans)
            else:
                region = depth_m[y0:y1, x0:x1]
                valid = region[(region > 0.05) & (region < 6.0)]
                if valid.size < 8:
                    continue
                zm = float(np.median(valid))
                # 신호등: 이름에 상태가 있으면(traffic_light_red) 그대로 · 없으면(traffic_light) 램프 HSV로
                color = model_color or classify_color(cls, s["bgr"][y0:y1, x0:x1])
            u, v = (x0 + x1) / 2.0, (y0 + y1) / 2.0
            px = (u - s["cx"]) * zm / s["fx"]
            py = (v - s["cy"]) * zm / s["fy"]
            z_map = rot @ np.array([px, py, zm]) + trans
            if cls in PUCK_CLASSES:  # depth는 보이는 표면 — 중심은 시선 방향으로 더 안쪽
                z_map = puck_center(z_map, trans, self.puck_center_offset, self.puck_center_full_deg)
                # 태그 가운데에서 구한 중심이 더 정확 — 단 박스 depth 추정과 3 cm 넘게 다르면 안 믿음
                #   (손가락 바로 아래 퍽은 태그 depth에 손가락이 섞일 수 있음)
                if tag_c is not None and np.linalg.norm(tag_c - z_map) < 0.03:
                    z_map = tag_c
            arm_in_box = cls not in PUCK_CLASSES and box_has_arm((x0, y0, x1, y1), arm_px, self.self_arm_hits)  # 퍽은 그리퍼 밑에서 집으므로 뺌
            if body is not None and (arm_in_box or is_self(cls, z_map, body, self.body_radius, self.body_height, self.self_radius, self.body_box)):
                with self.lock:
                    self.self_dropped += 1
                drops.append(((x0, y0, x1, y1), "self"))
                continue
            r_cam = meas_cov_cam(zm, s["fx"], s["fy"], self.pixel_sigma, self.depth_sigma0, self.depth_sigma_k)
            fb = SIZE_FALLBACK.get(cls, (0.1, 0.1, 0.1))
            dets.append({
                "class_name": cls,
                "z": z_map,
                "R": rot @ r_cam @ rot.T,
                "size": np.array([max((x1 - x0) * zm / s["fx"], 0.01),
                                  max((y1 - y0) * zm / s["fy"], 0.01), fb[2]]),
                "conf": float(b.conf[0]), "px": (int(u), int(v)),
                "color": color,
                "posture": posture,  # 퍽: standing / lying / ''(모름 — 합칠 때 아는 자세를 덮지 않음) · 그 외 ''
                "tag": tag,          # 퍽 AprilTag 번호 (못 읽으면 None — 합칠 때 아는 번호를 덮지 않음)
                "shape": shape,      # 퍽 모양 box · cyl · ''(모름 — 합칠 때 아는 모양을 덮지 않음)
                "yaw": yaw, "yaw_period": yaw_period,  # 퍽 방향 (추적 좌표) · 못 구하면 None — 합칠 때 아는 방향을 덮지 않음
                "box": (x0, y0, x1, y1),  # 영상 박스 — 디버그 영상에 그릴 때만 씀 (트랙에는 안 들어감)
            })
        # 상자 윗면 (depth) — 로봇 차체 둘레는 뺌 · 바닥 높이 floor_z는 위에서 구한 것
        avoid = (base_now[1][:2], self.body_radius) if base_now is not None else None
        t_box = time.perf_counter()  # 상자 시간 = 상자 YOLO(있으면) + depth 윗면 찾기
        rois = self.box_rois(s["bgr"]) if self.box_detect else None  # 상자 YOLO 네모 (모델 없으면 None = 화면 전체)
        # 자기 팔(그리퍼 포함) 투영 점(arm_all, 위에서 구함) — 상자 윗면 점에서 뺌
        boxes = (find_box_tops(depth_m, rot, trans, s["fx"], s["fy"], s["cx"], s["cy"], floor_z, self.box_h,
                               self.box_min_side, self.box_max_side, self.box_cell, avoid=avoid, rois=rois,
                               mask_px=arm_all)
                 if self.box_detect else [])
        if self.box_detect:
            self.prof_add(cam, "상자", t_box)
        # 화면 가장자리에 잘린 상자 → 방향 모름 (가운데·크기는 그대로 씀 · 방향은 다른 카메라/앞선 관측 값 유지)
        hh, ww = depth_m.shape[:2]
        boxes = [(c, sz, None if box_cut((c, sz, byaw, tz), rot, trans, s["fx"], s["fy"], s["cx"], s["cy"],
                                         ww, hh, self.box_edge_px) else byaw, tz)
                 for c, sz, byaw, tz in boxes]
        box_tag = {}  # 상자 번호(boxes 안 순서) → (태그 번호, 태그 yaw 또는 None)
        for tid_tag, (u, v), box, corners in tags:  # 장소 태그 (L1~L3) — 붙은 상자를 location_box로
            if tid_tag not in self.location_tags:
                continue
            d = self.marker_det(s, box, (u, v), rot, trans, "location_box")
            if d is None:
                continue
            # 장소 태그는 상자 윗면(위치는 정해지지 않음) → 태그로는 방향을 안 정함 (방향 = 윗면 변, 아래 상자 검출)
            l_yaw, face = tag_yaw(corners, self.location_tag_size, s["fx"], s["fy"], s["cx"], s["cy"], rot)
            l_yaw = l_yaw if face == "side" else None
            on = [i for i, bx in enumerate(boxes) if tag_on_box(d["z"][:2], bx, self.box_tag_margin)]
            if on:  # 윗면을 찾은 상자에 붙은 태그 → 그 상자에 번호·방향 (아래 상자 검출에서 같이 만듦)
                box_tag[on[0]] = (tid_tag, l_yaw)
                continue
            # 윗면을 못 찾음 (멀거나 일부만 보임) → 상자 위치 = 태그 위치 (상자 가운데와 최대 약 10 cm 어긋남 · 방향 모름)
            d["tag"] = tid_tag
            d["size"] = self.location_box_size.copy()
            if l_yaw is not None:
                d["z"] = d["z"] - self.location_box_size[0] / 2.0 * np.array([np.cos(l_yaw), np.sin(l_yaw), 0.0])
                d["yaw"], d["yaw_period"] = l_yaw, 2 * np.pi  # 대칭 없음 (360°)
            dets.append(d)
        for i, (c, (a, b), byaw, top_z) in enumerate(boxes):
            p_img = rot.T @ (np.array([c[0], c[1], top_z]) - trans)  # 윗면 가운데의 카메라 좌표 — 측정 분산 · 화면 위치용
            r_cam = meas_cov_cam(float(p_img[2]), s["fx"], s["fy"], self.pixel_sigma, self.depth_sigma0, self.depth_sigma_k)
            d = {"class_name": "start_box", "z": np.array([c[0], c[1], top_z]), "R": rot @ r_cam @ rot.T,
                 "size": np.array([a, b, max(top_z - floor_z, 0.01)]), "conf": 1.0,
                 "px": (int(s["fx"] * p_img[0] / p_img[2] + s["cx"]), int(s["fy"] * p_img[1] / p_img[2] + s["cy"])),
                 "color": "", "posture": "", "yaw": byaw, "yaw_period": np.pi / 2}  # 윗면 사각형 변 방향 — 정사각형이라 90°마다 같음
            if i in box_tag:  # 장소 태그가 윗면에 있는 상자 → location_box (방향 = 윗면 변, 90° 주기)
                d["class_name"], d["tag"] = "location_box", box_tag[i][0]
                if box_tag[i][1] is not None:
                    d["yaw"], d["yaw_period"] = box_tag[i][1], 2 * np.pi
            dets.append(d)
        now_s = stamp_s(now_stamp)
        dets = merge_overlap_dets(dets, self.det_merge_iou)
        with self.lock:
            tids = self.fuse(cam, now_stamp, dets)
            gone = merge_tracks(self.tracks, self.track_merge_m)
            tids = [gone.get(t, t) for t in tids]
            self.prune(now_s)
        out = TrackedObjectArray()
        out.header.stamp = now_stamp
        out.header.frame_id = "map"
        out.objects = [to_msg(tid, self.to_map({**d, "p": d["z"], "P": d["R"], "last_seen": now_stamp, "camera": cam}))
                       for tid, d in zip(tids, dets)]
        self.det_pubs[cam].publish(out)
        if self.debug_pubs[cam].get_subscription_count() > 0:  # 보는 쪽이 있을 때만 그림 (평소 CPU 안 씀)
            self.publish_debug(cam, s, res, tags, dets, tids)

    def publish_debug(self, cam, s, res, tags, dets, tids):
        """디버그 영상 /perception/<cam>/debug_image 발행 — 이 프레임 컬러 영상 위에 박스를 그림 (화면 확인용)
          회색 얇은 박스 = YOLO가 찾은 것 전부 (이름 · 신뢰도) · 버린 퍽·자기 몸은 빨간 글씨 'drop: 이유'
          색 굵은 박스 = 받아들여 트랙에 붙은 것 (#트랙 id · 이름 · 색/상태 · 모양 · 자세 · 태그)
            퍽·신호등 = 그 색 · 사람 파랑 · 방해물 흰색 · 바코드 보라 · 그 밖 파랑
          흰 박스 = AprilTag (번호) · 하늘색 점 = 영상 박스가 없는 검출 (상자 · 장소 태그). 변환 실패면 발행 안 함"""
        img = s["bgr"].copy()
        for b in res.boxes:  # YOLO 원본
            x0, y0, x1, y1 = (int(v) for v in b.xyxy[0].tolist())
            cv2.rectangle(img, (x0, y0), (x1, y1), (128, 128, 128), 1)
            cv2.putText(img, f"{self.names.get(int(b.cls[0]), '?')} {float(b.conf[0]):.2f}", (x0, max(y1 + 12, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)
        for (x0, y0, x1, y1), why in s.get("drops", []):  # 버린 퍽 — 이유
            cv2.putText(img, f"drop: {why}", (x0, min(y1 + 26, img.shape[0] - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
        for tag_id, _, box, _ in tags:  # AprilTag
            cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 255, 255), 1)
            cv2.putText(img, f"tag{tag_id}", (int(box[0]), max(int(box[1]) - 3, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        for d, tid in zip(dets, tids):  # 받아들인 검출
            cls = d["class_name"]
            # 네모 색: 퍽·신호등은 그 색 · 사람 파랑 · 방해물 흰색 · 바코드 보라 · 그 밖 파랑
            col = DEBUG_BGR.get(d.get("color", ""), (255, 0, 0)) if cls in PUCK_CLASSES + ("traffic_light",) else \
                {"person": (255, 0, 0), "obstacle": (255, 255, 255), "barcode": (200, 0, 200)}.get(cls, (255, 0, 0))
            # 이름표: #트랙 이름 색 · 모양(사각/원기둥) · 자세(서 있음/누움 — 모르면 생략) · 태그
            text = f"#{tid} {cls}" + (f" {d['color']}" if d.get("color") else "") + \
                (f" {DEBUG_SHAPE.get(d['shape'], d['shape'])}" if d.get("shape") else "") + \
                (f" {DEBUG_POSTURE.get(d['posture'], d['posture'])}" if d.get("posture") else "") + \
                (f" tag{d['tag']}" if d.get("tag") is not None else "")
            if d.get("box") is not None:
                x0, y0, x1, y1 = d["box"]
                cv2.rectangle(img, (x0, y0), (x1, y1), col, 2)
                (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                ty = max(y0 - 4, th + 4)
                cv2.rectangle(img, (x0, ty - th - 4), (x0 + tw + 4, ty + 3), (0, 0, 0), -1)  # 글씨 뒤 검은 띠 (잘 보이게)
                cv2.putText(img, text, (x0 + 2, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
            else:  # 상자 · 장소 태그만 — 박스 없이 가운데 점
                u, v = d["px"]
                cv2.circle(img, (int(u), int(v)), 6, (255, 200, 0), -1)
                cv2.putText(img, text, (int(u) + 8, int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 2)
        try:
            msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        except Exception:  # noqa: BLE001
            return
        msg.header.stamp, msg.header.frame_id = s["stamp"], s["frame_id"]
        self.debug_pubs[cam].publish(msg)

    def box_rois(self, bgr):
        """상자 전용 YOLO로 상자 네모들 → [(x0, y0, x1, y1)] [px] (box_roi_margin만큼 넓힘) · 모델이 없거나 추론 실패면 None (화면 전체)"""
        if self.box_model is None:
            return None
        try:
            r = self.box_model.predict(bgr, imgsz=self.imgsz, conf=self.box_conf, device=self.device, verbose=False)[0]
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"상자 추론 실패: {e}", throttle_duration_sec=5.0)
            return None
        h, w = bgr.shape[:2]
        out = []
        for b in r.boxes:
            if r.names[int(b.cls[0])] != "box":
                continue
            x0, y0, x1, y1 = b.xyxy[0].tolist()
            mx, my = (x1 - x0) * self.box_roi_margin, (y1 - y0) * self.box_roi_margin
            out.append((max(0.0, x0 - mx), max(0.0, y0 - my), min(w - 1.0, x1 + mx), min(h - 1.0, y1 + my)))
        return out

    def on_capture_dice(self, msg):
        """주사위 완성 사진 요청 (std_msgs/Int32: 장소 태그 번호 12~14 · 0이면 트랙에 있는 장소 상자 중 로봇에서 가장 가까운 것)
        → 그 상자 윗면의 네 꼭짓점을 카메라 화면에 투영 → 윗면을 반듯한 정사각형(dice_image_px)으로 펴서 /mission/dice_image 로 한 번 발행
          사진 = 상자 네모 변에 맞춰 반듯하게 (맵 방향과 상관없음 · 정사각형 상자라 어느 변이 위로 와도 됨) · 아래쪽 = 태그 쪽(yaw 방향)
          카메라: 상자 윗면과 퍽 윗면(+6 cm)이 둘 다 화면 안에 들어오는 카메라 중 가장 수직으로 내려다보는 것
            (비스듬한 사진을 펴면 퍽이 길게 늘어짐)
          펴는 높이: 바닥 + location_box_size 높이(0.07 m) — 트랙 위치가 태그 높이일 수도 있어서 상자 높이로 고정
          원본(펴기 전) 사진은 /mission/dice_image_raw 로 같이 발행 (확인용)
          장소 상자 트랙이 없으면: 지금 트랙에 있는 상자(location_box · start_box) 중 가장 가까운 것으로,
            그것도 없으면 경고만. 어느 카메라에도 윗면이 다 안 들어오면 원본 사진(그리퍼캠 우선)만 /mission/dice_image_raw 로"""
        with self.lock:
            locs = [(tid, dict(tr)) for tid, tr in self.tracks.items() if tr["class_name"] == "location_box"]
            any_box = [(tid, dict(tr)) for tid, tr in self.tracks.items() if tr["class_name"] in ("location_box", "start_box")]
            frames = {c: dict(s) for c, s in self.frames.items() if s.get("bgr") is not None and s.get("fx")}
        want = int(msg.data)
        if want:
            locs = [x for x in locs if x[1].get("tag") == want]
        if not locs and any_box:
            # 팔을 상자 위로 옮기는 동안 장소 태그가 화면에서 안 보이면 장소 상자 트랙이 2초 뒤 지워짐
            # → 지금 보이는 상자 중 로봇에서 가장 가까운 것으로 찍음
            self.get_logger().warning(f"주사위 사진: 장소 상자(tag {want or '아무거나'}) 트랙 없음 — 지금 보이는 가장 가까운 상자로 찍음")
            locs = any_box
        if not locs:
            self.get_logger().warning(f"주사위 사진: 장소 상자(tag {want or '아무거나'})를 아직 못 찾음 — 사진 없이 무시")
            return
        if len(locs) > 1:  # 로봇에서 가장 가까운 것
            base = self.lookup_tf(self.base_frame, rclpy.time.Time().to_msg(), quiet=True)
            bxy = base[1][:2] if base is not None else np.zeros(2)
            locs.sort(key=lambda x: float(np.hypot(*(np.asarray(x[1]["p"][:2]) - bxy))))
        tid, tr = locs[0]
        base = self.lookup_tf(self.base_frame, rclpy.time.Time().to_msg(), quiet=True)
        floor_z = float(base[1][2]) if base is not None else 0.0
        c = np.asarray(tr["p"], float).copy()
        c[2] = floor_z + float(self.location_box_size[2])  # 상자 윗면 높이 (태그 높이가 아니라)
        side = float(max(tr["size"][0], tr["size"][1])) * (1.0 + 2 * self.dice_margin)
        yaw = tr.get("yaw") if tr.get("yaw") is not None else 0.0
        # 윗면 네 꼭짓점 (추적 좌표): 사진 위왼쪽 · 위오른쪽 · 아래오른쪽 · 아래왼쪽 — 아래 = 태그 쪽(yaw 방향)
        fwd = np.array([np.cos(yaw), np.sin(yaw), 0.0]) * side / 2      # 태그 쪽(상자 밖)
        lft = np.array([-np.sin(yaw), np.cos(yaw), 0.0]) * side / 2     # 상자 변 방향 (왼·오른쪽 구분은 의미 없음)
        corners = [c - fwd + lft, c - fwd - lft, c + fwd - lft, c + fwd + lft]
        puck_up = np.array([0.0, 0.0, SIZE_FALLBACK["puck"][2]])  # 퍽 높이 0.06 m — 퍽 윗면이 사진 밖으로 잘리지 않는지 같이 봄
        best = None  # (수직 정도, cam, s, uv) — 상자 윗면·퍽 윗면 꼭짓점이 다 화면 안인 카메라 중 가장 수직으로 내려다보는 것
        for cam, s in frames.items():
            tf = self.lookup_tf(s["frame_id"], s["stamp"], quiet=True)
            if tf is None:
                continue
            rot, trans = tf
            h, w = s["bgr"].shape[:2]
            uv, fits = [], True
            for p in corners + [q + puck_up for q in corners]:
                q = rot.T @ (p - trans)
                if q[2] < 0.05:
                    fits = False
                    break
                u, v = s["fx"] * q[0] / q[2] + s["cx"], s["fy"] * q[1] / q[2] + s["cy"]
                fits &= -0.02 * w <= u <= 1.02 * w and -0.02 * h <= v <= 1.02 * h
                uv.append((u, v))
            if not fits:
                continue
            down = float(-rot[2, 2])  # 카메라 앞쪽(광학 z축)이 아래(−z)를 향한 정도 — 1이면 수직으로 내려다봄
            if best is None or down > best[0]:
                best = (down, cam, s, uv[:4])  # 펴는 건 상자 윗면 네 꼭짓점으로
        if best is not None:
            down, cam, s, uv = best
            n = self.dice_px
            hmat = cv2.getPerspectiveTransform(np.float32(uv), np.float32([[0, 0], [n, 0], [n, n], [0, n]]))
            top = cv2.warpPerspective(s["bgr"], hmat, (n, n), borderValue=(114, 114, 114))
            tilt = float(np.degrees(np.arccos(np.clip(down, -1.0, 1.0))))
            self.publish_dice(top, s, f"tag{tr.get('tag')} 상자 윗면 ({cam} · 수직에서 {tilt:.0f}° 기울어짐)")
            self.publish_dice(s["bgr"], s, "원본", raw=True)
            return
        s = frames.get("gripper_cam") or next(iter(frames.values()), None)
        if s is not None:
            self.get_logger().warning("주사위 사진: 상자 윗면이 카메라에 다 안 보임 — 원본 사진만 발행")
            self.publish_dice(s["bgr"], s, "원본(윗면 안 보임)", raw=True)

    def publish_dice(self, img, s, what, raw=False):  # 주사위 사진 발행 (raw면 /mission/dice_image_raw) · 변환 실패면 경고만
        try:
            m = self.bridge.cv2_to_imgmsg(np.ascontiguousarray(img), encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"주사위 사진 변환 실패: {e}")
            return
        m.header.stamp, m.header.frame_id = s["stamp"], s["frame_id"]
        (self.dice_raw_pub if raw else self.dice_pub).publish(m)
        self.get_logger().info(f"주사위 사진 발행: {what}")

    def marker_det(self, s, box, center, rot, trans, cls):
        """표식(장소 태그 · 바코드) 영상 박스 [px]와 가운데 → 검출 dict (위치 = 박스 안 깊이 중앙값으로 역투영, 추적 좌표)
        박스가 비거나 깊이 화소가 4개 미만이면 None"""
        h, w = s["depth_mm"].shape[:2]
        x0, y0, x1, y1 = max(box[0], 0), max(box[1], 0), min(box[2], w - 1), min(box[3], h - 1)
        if x1 <= x0 or y1 <= y0:
            return None
        region = s["depth_mm"][y0:y1, x0:x1].astype(np.float32) / 1000.0
        valid = region[(region > 0.05) & (region < 6.0)]
        if valid.size < 4:
            return None
        zm = float(np.median(valid))
        u, v = center
        z_map = rot @ np.array([(u - s["cx"]) * zm / s["fx"], (v - s["cy"]) * zm / s["fy"], zm]) + trans
        r_cam = meas_cov_cam(zm, s["fx"], s["fy"], self.pixel_sigma, self.depth_sigma0, self.depth_sigma_k)
        return {"class_name": cls, "z": z_map, "R": rot @ r_cam @ rot.T,
                "size": np.array([max((x1 - x0) * zm / s["fx"], 0.01), max((y1 - y0) * zm / s["fy"], 0.01),
                                  SIZE_FALLBACK[cls][2]]),
                "conf": 1.0, "px": (int(u), int(v)), "color": "", "posture": ""}

    def puck_yaw(self, s, depth_m, box, color, shape, posture, tag_y, face, rot, trans):
        """퍽 한 개의 방향 → (yaw [rad, 추적 좌표], 대칭 주기 [rad]) · 못 구하면 (None, 0.0)
        tag_y, face = tag_yaw 결과 (태그 못 읽으면 None, '') — 모양 판단(tag_shape)에도 쓰여 호출하는 쪽에서 한 번만 구함
          누운 퍽: 퍽 색 화소의 바닥 위 점들의 긴 축 (주기 180°) — 안 되면 원기둥 바닥면 태그 법선(= 긴 축)
          서있는 원기둥: 어느 쪽에서 봐도 같음 → 방향 없음
          그 밖(서있는 사각기둥 · 모양·자세 모름): 태그 자세 (주기 90° — 옆면이면 법선, 윗면이면 태그 x축)"""
        if posture == "lying":
            yaw = axis_yaw(self.puck_points_xy(s, depth_m, box, color, rot, trans), self.yaw_min_points, self.yaw_min_elong)
            if yaw is None and shape == "cyl" and face == "side":
                yaw = tag_y
            return (wrap_yaw(yaw, np.pi), np.pi) if yaw is not None else (None, 0.0)
        if shape == "cyl" and posture == "standing":
            return None, 0.0
        if tag_y is None or (shape == "cyl" and face == "top"):  # 원기둥 윗면 태그는 방향이 없음
            return None, 0.0
        return wrap_yaw(tag_y, np.pi / 2), np.pi / 2

    def puck_points_xy(self, s, depth_m, box, color, rot, trans):
        """퍽 박스 안에서 퍽 색(color ± hue_tol)이고 깊이가 있는 화소 → 추적 좌표 바닥 위 점 (N×2) [m]. 최대 400점 · 색을 모르면 빈 배열"""
        x0, y0, x1, y1 = box
        if color not in HUES:
            return np.zeros((0, 2))
        hsv = cv2.cvtColor(s["bgr"][y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
        h = hsv[..., 0].astype(int)
        dh = hue_dist(h, color)
        z = depth_m[y0:y1, x0:x1]
        vv, uu = np.nonzero((dh <= self.hue_tol) & (hsv[..., 1] > 80) & (hsv[..., 2] > 50) & (z > 0.05) & (z < 3.0))
        if len(vv) > 400:
            k = np.linspace(0, len(vv) - 1, 400).astype(int)
            vv, uu = vv[k], uu[k]
        zz = z[vv, uu]
        cam = np.stack([(uu + x0 - s["cx"]) * zz / s["fx"], (vv + y0 - s["cy"]) * zz / s["fy"], zz], -1)
        return (cam @ rot.T + trans)[:, :2]

    def judge_puck(self, wcm, hcm, color, clipped=False, model_color="", near=False):
        """퍽 후보 박스의 실제 크기(폭·높이 cm, 깊이 없으면 ≤0)·색·잘림·YOLO 이름의 색·가까움 → (퍽인가, 아니면 이유, 자세)
        자세 = standing / lying · 잘렸거나 끝면·윗면만 보이거나 깊이가 없으면 '' (모름) — test/puck_tag_check.py judge와 같음"""
        # 색 검사를 먼저 — 깊이가 없어도 색은 볼 수 있음
        if color == "unknown":
            return False, "색 없음", ""     # 흰·회색 물체 (퍽은 색이 있음)
        if color not in self.puck_colors:
            return False, "색", ""          # 퍽 색 밖 (other = 주황·보라 등)
        if self.check_model_color and model_color and model_color != color:
            return False, "색 불일치", ""   # YOLO 이름의 색과 HSV 색이 다름 (노드는 model_color=''로 불러 여기는 안 씀)
        if wcm <= 0 or hcm <= 0:
            if clipped or near:
                # D435는 약 17cm보다 가까우면 깊이를 못 잼 — 집기 직전이라 놓치면 안 됨: 퍽은 맞음, 거리·자세 모름
                return True, "", ""
            return False, "depth 없음", ""  # 작게 보이는데 깊이 없음 = 반사 등 → 위치를 몰라 퍽으로 안 냄
        long_cm, short_cm = max(wcm, hcm), min(wcm, hcm)
        if clipped:
            # 퍽 일부가 화면 밖 → 박스가 실제보다 작게만 나옴 → 하한·자세는 안 봄. 상한을 넘으면 거리가 틀린 것 (배경을 잼)
            if long_cm > self.long_max:
                return False, "거리 이상", ""
            return True, "", ""
        if not (self.long_min <= long_cm <= self.long_max) or short_cm < self.short_min:
            return False, "크기", ""
        ratio = hcm / wcm
        if long_cm < self.square_long and self.square_ratio[0] <= ratio <= self.square_ratio[1]:
            return True, "", ""  # 끝면·윗면(3×3)만 보임 — 서 있는지 누웠는지 박스로는 모름 (gripper_cam이 위에서 본 서있는 퍽)
        return True, "", "standing" if ratio >= self.stand_ratio else "lying"

    def fuse(self, cam, stamp, dets):
        """한 프레임의 검출들을 트랙에 1:1로 붙여 칼만 갱신 (lock 안에서 호출).
        후보 = 같은 종류(same_object — 퍽 계열은 하나 · 퍽은 색도) · match_dist 안 → 마할라노비스 거리 작은 쌍부터 배정. 남은 검출은 새 트랙.
        트랙의 class_name은 kind(퍽 계열 = 'puck'), 퍽 자세는 posture에 따로 → 발행할 때 puck / puck_lying (to_msg)
        → 검출마다 붙은 트랙 id"""
        t = stamp_s(stamp)
        pred = {}
        for tid, tr in self.tracks.items():
            q = self.q_dyn if tr["class_name"] in DYNAMIC_CLASSES else self.q
            pred[tid] = tr["P"] + np.eye(3) * q * max(0.0, t - tr["t"])
        pairs = []
        for i, d in enumerate(dets):
            for tid, tr in self.tracks.items():
                if not same_object(tr, d):
                    continue
                e = d["z"] - tr["p"]
                if float(np.linalg.norm(e)) >= self.match_dist:
                    continue
                pairs.append((float(e @ np.linalg.solve(pred[tid] + d["R"], e)), i, tid))
        used_d, used_t = {}, set()
        for _, i, tid in sorted(pairs):
            if i in used_d or tid in used_t:
                continue
            used_d[i] = tid
            used_t.add(tid)
            d, tr = dets[i], self.tracks[tid]
            tr["p"], tr["P"], k = kf_update(tr["p"], pred[tid], d["z"], d["R"])
            a = float(np.trace(k)) / 3.0  # 이번 관측을 믿은 정도 — 크기도 같은 비율로
            tr["size"] = tr["size"] + a * (d["size"] - tr["size"])
            tr["t"] = max(tr["t"], t)
            tr["last_seen"] = stamp if t >= stamp_s(tr["last_seen"]) else tr["last_seen"]
            tr["seen"][cam] = t
            tr.update(conf=d["conf"], camera=cam, px=d["px"])
            if tr["class_name"] != "puck" or d["color"] not in NO_COLOR:  # 퍽은 색을 못 정한 관측이 아는 색을 덮지 않게
                tr["color"] = d["color"]
            if d.get("posture"):  # 자세를 아는 관측만 자세를 바꿈 ('모름'은 덮어쓰지 않음)
                tr["posture"] = d["posture"]
            if d.get("tag") is not None:  # 태그 번호는 한 번 읽으면 트랙에 남김 (멀어져 못 읽어도 덮지 않음)
                tr["tag"] = d["tag"]
            if d.get("content"):
                tr["content"] = d["content"]
            if d.get("shape"):  # 모양은 아는 관측만
                tr["shape"] = d["shape"]
            if d.get("yaw") is not None and tr.get("yaw") is not None and d["yaw_period"] < tr.get("yaw_period", 0.0):
                pass  # 장소 상자: 태그로 구한 방향(360°)을 윗면 사각형 방향(90°)으로 덮지 않음
            elif d.get("yaw") is not None:  # 방향은 구한 관측의 최신값 (평균 내지 않음 — 집기 직전 그리퍼캠이 가장 정확)
                tr["yaw"], tr["yaw_period"] = d["yaw"], d["yaw_period"]
            elif d.get("posture") and d["posture"] != tr.get("yaw_posture"):
                tr["yaw"] = None  # 자세가 바뀜(세움 ↔ 누움) → 옛 방향은 뜻이 없음
            if d.get("posture"):
                tr["yaw_posture"] = d["posture"]
        for i, d in enumerate(dets):
            if i in used_d:
                continue
            self.tracks[self.next_id] = {
                "class_name": kind(d["class_name"]), "posture": d.get("posture", ""),
                "p": d["z"], "P": d["R"], "t": t, "size": d["size"],
                "conf": d["conf"], "last_seen": stamp, "seen": {cam: t},
                "camera": cam, "px": d["px"], "color": d["color"], "tag": d.get("tag"), "content": d.get("content", ""),
                "shape": d.get("shape", ""), "yaw": d.get("yaw"), "yaw_period": d.get("yaw_period", 0.0),
                "yaw_posture": d.get("posture", "")}
            used_d[i] = self.next_id
            self.next_id += 1
        return [used_d[i] for i in range(len(dets))]

    def prune(self, now_s):
        """한 카메라 한 추론 뒤 트랙 정리 (lock 안에서 호출, self.tracks를 바꿈) — now_s = 그 프레임 시각 [s]
          장소 상자 자리(box_min_side 안)에 있는 start_box는 삭제 (같은 상자를 태그 없는 쪽에서 본 것)
          2초 규칙: 모든 물체는 TRACK_TIMEOUT_S 넘게 어느 카메라에도 안 보이면 삭제"""
        loc = [tr["p"][:2] for tr in self.tracks.values() if tr["class_name"] == "location_box"]
        for tid, tr in list(self.tracks.items()):
            # 같은 상자를 태그 없이(뒤·옆에서) 봐서 생긴 start_box → 그 자리에 장소 상자가 있으면 지움
            if tr["class_name"] == "start_box" and any(np.linalg.norm(tr["p"][:2] - q) < self.box_min_side for q in loc):
                del self.tracks[tid]
                continue
            if now_s - tr["t"] > TRACK_TIMEOUT_S:
                del self.tracks[tid]

    def robot_body(self, stamp):
        """그 시각 로봇 몸 → (차체 원점, [팔 체인 링크 원점…], 차체 회전 3×3) — 추적 좌표(odom). 차체 TF가 없으면 None (필터 안 함)"""
        base = self.lookup_tf(self.base_frame, stamp, quiet=True)
        if base is None:
            return None
        chain = [t[1] for t in (self.lookup_tf(link, stamp, quiet=True) for link in self.self_links) if t is not None]
        return base[1], chain, base[0]

    def to_map(self, tr):
        """추적 좌표(odom) 트랙·검출 dict → map 기준 사본 (위치·공분산·yaw 회전) — 지금의 map ← odom (SLAM 보정 최신값)
        TF가 잠깐 없으면 마지막 성공값으로 · 한 번도 없었으면(SLAM 기동 전) 바꾸지 않고 그대로"""
        try:
            t = self.tf.lookup_transform("map", self.track_frame, rclpy.time.Time())
            tt = t.transform.translation
            self._out_tf = (quat_to_rot(t.transform.rotation), np.array([tt.x, tt.y, tt.z]))
        except Exception as e:  # noqa: BLE001
            if self._out_tf is None:  # 아직 map이 없음 (SLAM 기동 전) — odom 그대로 (map 원점 = odom 원점으로 시작)
                self.get_logger().warning(f"map ← {self.track_frame} TF 없음 — 그대로 발행: {e}", throttle_duration_sec=5.0)
                return tr
        rot, trans = self._out_tf
        out = {**tr, "p": rot @ np.asarray(tr["p"]) + trans, "P": rot @ np.asarray(tr["P"]) @ rot.T}
        if tr.get("yaw") is not None:  # 방향도 map ← odom 회전(yaw)만큼 돌림 — 대칭 주기 안으로 다시 맞춤
            out["yaw"] = wrap_yaw(tr["yaw"] + float(np.arctan2(rot[1, 0], rot[0, 0])), tr["yaw_period"])
        return out

    def lookup_tf(self, frame_id, stamp, quiet=False):
        """track_frame(odom) ← 카메라 TF를 영상 시각으로 (팔이 움직이는 중에도 찍힌 순간의 자세). 그 시각이 버퍼에 없으면 최신으로 대신.
        기다리지 않는다 (단일 스레드 실행기에서 기다리면 TF 수신도 같이 멈춤) → (회전 3×3, 이동 3) 또는 None"""
        try:
            t = self.tf.lookup_transform(self.track_frame, frame_id, rclpy.time.Time.from_msg(stamp))
        except Exception:  # noqa: BLE001
            try:
                t = self.tf.lookup_transform(self.track_frame, frame_id, rclpy.time.Time())
                if not quiet:
                    with self.lock:
                        self.tf_fallback += 1
            except Exception as e:  # noqa: BLE001
                if not quiet:
                    self.get_logger().warning(f"TF 실패: {e}", throttle_duration_sec=5.0)
                return None
        tr = t.transform.translation
        return quat_to_rot(t.transform.rotation), np.array([tr.x, tr.y, tr.z])

    # ---------- 발행 (rate_hz, 기본 10Hz)
    def on_publish(self):
        # 모든 트랙을 map 기준으로 바꿔 /tracked_objects 발행 (로봇까지 거리 details distance 포함)
        out = TrackedObjectArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        with self.lock:
            items = [(tid, dict(tr)) for tid, tr in sorted(self.tracks.items())]
        # 로봇 ~ 물체 거리: 지금 로봇(base_footprint) 위치와 트랙 위치를 같은 추적 좌표(odom)에서 바닥 위 거리로 — 로봇 TF가 없으면 생략
        base = self.lookup_tf(self.base_frame, rclpy.time.Time().to_msg(), quiet=True)
        if base is not None:
            for _, tr in items:
                tr["distance"] = float(np.hypot(tr["p"][0] - base[1][0], tr["p"][1] - base[1][1]))
        out.objects = [to_msg(tid, self.to_map(tr)) for tid, tr in items]  # 추적은 odom — 발행 순간의 map ← odom으로
        self.pub.publish(out)
        with self.lock:
            self.published += 1

    # ---------- 바코드 (2Hz) — 작업 지시 문자열 (P2-01 …) → 물체 barcode(위치 = 붙은 곳)로 추적 + /mission/qr
    def read_barcodes(self):
        # 바코드 라이브러리가 없으면 아무것도 안 함 · TF가 없으면 /mission/qr만 (물체는 안 만듦)
        if self.barcode is None:
            return
        if self.barcode_once and self.barcode_text:
            # 이미 읽음 → 바코드 찾기(zxing, CPU 큼)는 더 안 하고 읽은 내용만 다시 냄. mission_manager는 바코드 단계가
            # 시작된 뒤에 받은 /mission/qr만 쓰므로(mission_manager.cpp barcode_phase), 그 전에 읽었어도 단계 안에서 받게.
            self.qr_pub.publish(String(data=self.barcode_text))
            return
        with self.lock:
            frames = [(c, dict(s)) for c, s in self.frames.items()
                      if s.get("bgr") is not None and s.get("depth_mm") is not None and s.get("fx")]
        for cam, s in frames:
            try:
                ok, infos, _, points = self.barcode(s["bgr"])
            except cv2.error:
                continue
            if not ok or points is None:
                continue
            tf = self.lookup_tf(s["frame_id"], s["stamp"], quiet=True)
            dets = []
            for text, pts in zip(infos, points):
                if not text:
                    continue
                pts = np.asarray(pts).reshape(-1, 2)
                (x0, y0), (x1, y1) = pts.min(0), pts.max(0)
                if tf is not None:
                    d = self.marker_det(s, (int(x0), int(y0), int(x1) + 1, int(y1) + 1),
                                        (float(pts[:, 0].mean()), float(pts[:, 1].mean())), *tf, "barcode")
                    if d is not None:
                        d["content"] = text
                        dets.append(d)
                now = time.time()
                if text != self.qr_text or now - self.qr_seen > QR_GAP_S:
                    self.qr_text = text
                    self.qr_pub.publish(String(data=text))
                    self.get_logger().info(f"바코드 ({cam}): {text}" + (" — 이제 바코드는 안 읽음 (barcode_once)" if self.barcode_once else ""))
                self.qr_seen = now
                self.barcode_text = text
            if dets:
                with self.lock:
                    self.fuse(cam, s["stamp"], dets)

    def on_diag(self):
        # 1초마다 /diagnostics — 모델 · 트랙 수 · 융합 · 동기화 · TF 대체 · 자기 몸 버림 · QR · AprilTag · 바코드 상태
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
            # 최근 TRACK_TIMEOUT_S 안에 두 카메라가 모두 본 트랙 = 실제로 융합 중인 물체
            fused = sum(1 for tr in self.tracks.values()
                        if len(tr["seen"]) > 1 and max(tr["seen"].values()) - min(tr["seen"].values()) < TRACK_TIMEOUT_S)
            pub = self.published
            synced = dict(self.synced)
            fallback = self.tf_fallback
            dropped = self.self_dropped
            prof, self.prof = self.prof, {}  # 지난 1초 처리 시간 (가져가고 비움)
        s.values = [KeyValue(key="추적 물체", value=str(n)),
                    KeyValue(key="두 카메라 융합 물체", value=str(fused)),
                    KeyValue(key="동기화 프레임",
                             value=" · ".join(f"{c} {k}" for c, k in sorted(synced.items())) or "0"),
                    KeyValue(key="TF 최신 대체", value=str(fallback)),
                    KeyValue(key="자기 몸이라 버린 검출", value=str(dropped)),
                    KeyValue(key="발행한 /tracked_objects", value=str(pub)),
                    KeyValue(key="AprilTag (퍽 번호·장소)", value=f"OpenCV {cv2.__version__} " + ("aruco 있음" if self.tag_det else "aruco 없음 — 퍽 번호 못 읽음")),
                    KeyValue(key="바코드 (장소 작업 지시)", value=(BARCODE_LIB + (f" · 읽음: {self.barcode_text}" if self.barcode_text else ""))
                             if self.barcode else "없음 — pip install zxing-cpp")]
        # 카메라마다 한 줄: 단계 평균/최대 [ms] (지난 1초) — 간격이 250보다 길면 추론이 밀리는 중, 전체가 길면 어느 단계 탓인지 봄
        for cam in sorted({c for c, _ in prof}):
            parts = [f"{k} {prof[(cam, k)][0] / prof[(cam, k)][1]:.0f}/{prof[(cam, k)][2]:.0f}"
                     for k in ("YOLO", "AprilTag", "상자", "전체", "간격") if (cam, k) in prof]
            s.values.append(KeyValue(key=f"처리 시간 ms 평균/최대 {cam}", value=" · ".join(parts)))
        if self.tag_det is None and s.level == DiagnosticStatus.OK:
            s.level, s.message = DiagnosticStatus.WARN, "AprilTag 못 읽음 (OpenCV aruco 없음)"
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
