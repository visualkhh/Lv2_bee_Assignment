#!/usr/bin/env python3
"""🛠 perception (YOLO nano, 담당: 전승혜) — 설계: documents/project/ros/02-sense.md

  입력: pole_front_cam·gripper_cam 컬러·정렬 depth·camera_info (best_effort)
  출력: /tracked_objects (perception/TrackedObjectArray, reliable) — 두 카메라 융합 결과 · vlm_task_planner · coordinator · grasp_scorer
        /perception/<cam>/detections (같은 타입) — 융합 전 카메라별 한 프레임 검출 (id = 붙은 트랙 id, covariance = 관측 공분산)
          → 시뮬레이터가 카메라별 검출과 융합 결과를 나란히 그림 (디버그용 — 소비 노드 없음)

  추적은 odom 기준(track_frame), 발행만 map 기준 — RTAB-Map이 map→odom을 고칠 때마다(잘못 고쳐도) map에 저장해 둔 물체가
  옛 보정에 남아 로봇 기준으로 엉뚱한 곳에 보였고, 보정 전후 관측이 다른 기준으로 섞여 합쳐졌음 (실측: 제자리 여러 바퀴 뒤 puck이 수십 cm 밀림).
  odom은 끊김 없이 이어짐 → 추적·융합은 거기서, /tracked_objects는 발행 순간의 map→odom으로 바꿔 냄 (로봇과 같은 보정으로 같이 움직임).
  컬러·정렬 depth를 시각 동기화(ApproximateTime)해 한 쌍으로 받고, camera_info 내부 파라미터로 3D 역투영
  → 영상 시각(header.stamp)의 TF로 odom 기준 변환 (gripper_cam처럼 팔과 함께 움직여도 찍힌 순간의 자세로) → 추적.
  두 카메라 융합: 같은 물체를 pole_front_cam·gripper_cam이 보면 한 트랙에 칼만 갱신으로 합친다.
    관측 공분산은 카메라 좌표에서 depth 축 σz = depth_sigma0 + depth_sigma_k·z² (RealSense 오차는 거리² 비례),
    가로·세로 σ = pixel_sigma·z/f → 그 시각 TF 회전으로 map에 돌려 넣는다 → 가까이서 본 gripper_cam이 자연히 더 무겁고,
    두 카메라가 다른 방향에서 보면 서로의 depth 축 오차를 줄여 준다. 결과 공분산은 pose.covariance(위치 3×3)로 발행.
    퍽 계열(puck · puck_lying)은 한 종류로 합치고 자세는 따로 둔다 — 두 카메라가 자세를 다르게 봐도 id 하나.
  퍽 판단 (judge_puck): YOLO가 퍽이라 해도 깊이 50% 규칙 · 색(puck_colors ± hue_tol) · 실제 크기(cm)로 다시 보고, 자세도 크기로
    (YOLO 이름의 자세는 실물에서 틀렸음). 모델이 색별로 배워도(puck_red) 나갈 때는 설계 이름 puck / puck_lying + color (split_class).
  기억 (팀 확인 10/22): 퍽·목표 구역·QR 판은 안 보여도 목록에 남김 (같은 id). 그 자리가 잘 보이는데 miss_count번 연속 못 찾으면 지움.
    사람·장애물·신호등은 TRACK_TIMEOUT_S 안 보이면 지움. 지금 보이는지는 받는 쪽이 last_seen으로 구분.
  모델 파일이 없으면 빈 목록을 내보낸다 (진단 WARN) — test-bringup 기동 검사용.
  표식 (documents/mission/README.md · 발제 「AprilTag와 바코드 실제 배포 표식」): AprilTag tag36h11
    퍽 윗면 16mm — R1~R4 = 0~3 · G1~G4 = 4~7 · B1~B4 = 8~11 → 그 퍽 트랙의 details tag (한 번 읽으면 기억, 색도 태그 기준)
      (16mm는 pole_front_cam으로는 안 읽힘 — gripper_cam이 위에서 내려다봐야 함: coordinator inspect)
    장소 48mm — L1~L3 = 12~14 → class location 물체 (details tag, 안 보여도 기억)
    바코드(줄무늬, P2-01 등) = 장소에 붙은 작업 지시 문자열 → class barcode 물체 (details content, 위치 = 붙은 곳) + /mission/qr
  신호등 상태: red · yellow · green · off · all(두 색 이상 동시에 켜짐 — 신호등 미션 단계 구분 신호)
"""
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
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

QR_GAP_S = 10.0  # 이만큼 QR이 안 보였다가 다시 읽히면 새 미션 지역으로 보고 재발행 [s]
# 모델 없을 때의 기본 클래스 — 모델이 있으면 모델에 저장된 이름(model.names)을 쓴다 (학습 클래스가 바뀌어도 코드 수정 없음)
CLASSES = ["puck", "puck_lying", "goal_zone", "station_zone", "qr_board", "obstacle", "person", "traffic_light"]
# YOLO가 아니라 표식 디코드로 만드는 물체 — AprilTag 장소 태그 · 바코드
MARKER_CLASSES = ("location", "barcode")
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
    "location": (0.06, 0.06, 0.01),    # 장소 태그 48mm (흰 여백 포함 60mm)
    "barcode": (0.10, 0.01, 0.05),
}
DYNAMIC_CLASSES = ("person",)  # 스스로 움직이는 물체 — 예측 단계 공정 잡음을 process_noise_dynamic으로 크게
TRACK_TIMEOUT_S = 2.0          # 기억하지 않는 물체(사람·장애물·신호등)가 이만큼 어느 카메라에도 안 보이면 트랙 삭제 [s]
# 안 보여도 기억하는 물체 (팀 확인 10/22) — 스스로 움직이지 않고 나중에 다시 찾아가야 하는 것. 클래스 이름이 이 접두어로 시작하면 기억
#   지우는 때 = 그 자리가 가려지지 않고 잘 보였어야 하는데(expected_visible) miss_count번 연속 못 찾음 (옮겨짐 · 집어 감)
#   화면 밖 · 너무 멂 · 화면 가장자리 · 깊이 없음 · 앞에 다른 물체 → 못 봤다고 세지 않음
REMEMBER_PREFIXES = ("puck", "goal_zone", "station_zone", "qr_board", "location", "barcode")
ROT_VAR_UNKNOWN = 1e3          # 방향은 추정하지 않음 → pose.covariance 회전 대각 (사실상 모름)


PUCK_CLASSES = ("puck", "puck_lying")
NO_COLOR = ("", "unknown")  # 색을 못 정한 값 — 퍽 합칠 때 '색이 다름'으로 보지 않음
BODY_MIN_Z = 0.05  # 차체 판정은 이 높이 위만 [m]


def remembered(cls):
    """안 보여도 기억하는 클래스인가 (REMEMBER_PREFIXES)"""
    return cls.startswith(REMEMBER_PREFIXES)


def kind(cls):
    """트랙에 저장·합칠 때 쓰는 종류 — 퍽 계열(puck · puck_lying)은 모두 'puck' (자세는 트랙의 posture에 따로).
    같은 퍽을 pole_front_cam(옆에서 '서있음')과 gripper_cam(위에서 다르게)이 다른 이름으로 봐도 id가 하나가 되게"""
    return "puck" if cls in PUCK_CLASSES else cls


def same_object(tr, d):
    """트랙 tr에 검출 d를 붙일 수 있는 종류인가 — 같은 종류 + 퍽이면 색도 같아야 (붙은 다른 색 퍽을 한 트랙으로 뭉치지 않게).
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


def expected_visible(p, rot, trans, fx, fy, cx, cy, depth_m, max_range, edge_px, occlusion_m):
    """추적 좌표의 점 p가 이 카메라 영상에 '가려지지 않고 보였어야' 하는가 — 기억한 물체가 사라졌는지 판정할 때만 씀.
    rot·trans = 카메라 → 추적 좌표 (lookup_tf) · depth_m = 그 프레임 깊이 영상 [m].
    아니라고 보는 경우 (= 못 봤다고 셀 수 없음):
      카메라 뒤 · 0.1m보다 가까움 · max_range보다 멂 / 화면 밖 · 가장자리 edge_px 안 /
      그 픽셀 둘레(5×5)에 깊이가 없음 (D435는 약 17cm 안쪽을 못 잼) / 그 픽셀 깊이가 점까지 거리보다 occlusion_m 넘게 가까움 (앞에 다른 물체·팔)"""
    pc = rot.T @ (np.asarray(p, float) - trans)  # 카메라 광학 좌표 (x 오른쪽 · y 아래 · z 앞)
    z = float(pc[2])
    if z < 0.1 or z > max_range:
        return False
    u, v = fx * pc[0] / z + cx, fy * pc[1] / z + cy
    h, w = depth_m.shape[:2]
    if not (edge_px <= u < w - edge_px and edge_px <= v < h - edge_px):
        return False
    ui, vi = int(u), int(v)
    win = depth_m[max(0, vi - 2):vi + 3, max(0, ui - 2):ui + 3]
    vals = win[win > 0.05]
    if vals.size == 0:
        return False
    return float(np.median(vals)) >= z - occlusion_m


def seg_dist(p, a, b):
    """점 p와 선분 a-b 사이 거리"""
    ab = b - a
    t = float(np.clip((p - a) @ ab / max(float(ab @ ab), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def is_self(cls, p, body, body_radius, body_height, self_radius):
    """map 점 p가 로봇 자기 몸인가 — body = (차체 원점, [팔 체인 링크 원점…]) (map).
    차체: 원점에서 수평 body_radius 안 · 높이 body_height 아래 (적재함·폴·차체). 팔: 이웃 링크를 잇는 선분에서 self_radius 안.
    퍽은 마지막 선분(gripper_link→손끝, 손가락 사이)을 빼고 본다 — 집으려고 손끝 바로 아래 둔 퍽을 지우지 않게"""
    base, chain = body
    # 바닥(5cm 아래)은 차체로 보지 않음 — 범퍼 바로 앞 퍽이 차체 반경 안에 걸려 지워지지 않게
    if np.hypot(p[0] - base[0], p[1] - base[1]) < body_radius and base[2] + BODY_MIN_Z < p[2] < base[2] + body_height:
        return True
    segs = list(zip(chain[:-1], chain[1:]))
    if cls in PUCK_CLASSES:
        segs = segs[:-1]
    return any(seg_dist(p, a, b) < self_radius for a, b in segs)


def arm_pixels(chain, rot, trans, fx, fy, cx, cy, radius=0.04, per_seg=10):
    """팔 체인(추적 좌표 링크 원점들)을 그 카메라 화면에 투영 → [[u, v, r], …] (r = 팔 두께 radius의 픽셀 반경).
    이웃 링크 사이를 per_seg개로 나눠 찍음. 링크 원점은 팔 중심선이라 화면에 보이는 팔 몸통보다 안쪽(아래 끝 밖)에 떨어짐
    (실측: 스캔 자세 팔이 pole_front_cam 왼쪽 아래 구석에 보이는데 중심선은 v 477~554) → 두께만큼 넓혀 판정.
    rot·trans = 카메라 → 추적 좌표 (lookup_tf). 카메라 뒤 점은 뺌"""
    if len(chain) < 2:
        return np.zeros((0, 3))
    pts = np.concatenate([a + (b - a) * t[:, None] for a, b in zip(chain[:-1], chain[1:])
                          for t in [np.linspace(0.0, 1.0, per_seg)]])
    cam = (pts - trans) @ rot  # = rot.T @ (p − trans)
    cam = cam[cam[:, 2] > 0.05]
    return np.stack([fx * cam[:, 0] / cam[:, 2] + cx, fy * cam[:, 1] / cam[:, 2] + cy, fx * radius / cam[:, 2]], -1)


def box_has_arm(box, arm_px, min_hits):
    """검출 박스와 팔(투영점 ± 두께)이 겹치는 점이 min_hits개 이상 — 영상에서 팔을 잡은 것"""
    if not len(arm_px):
        return False
    x0, y0, x1, y1 = box
    u, v, r = arm_px[:, 0], arm_px[:, 1], arm_px[:, 2]
    inside = (u + r >= x0) & (u - r <= x1) & (v + r >= y0) & (v - r <= y1)
    return int(inside.sum()) >= min_hits


def stamp_s(stamp):
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
    """카메라 광학 프레임(x 오른쪽 · y 아래 · z 앞)에서 본 한 점의 관측 공분산 [m²]"""
    sz = depth_sigma0 + depth_sigma_k * z * z
    return np.diag([(pixel_sigma * z / fx) ** 2, (pixel_sigma * z / fy) ** 2, sz * sz])


def kf_update(p, P, z, R):
    """위치 상태(정지 모델, H=I) 칼만 갱신 → (p, P, K)"""
    K = P @ np.linalg.inv(P + R)
    p = p + K @ (z - p)
    P = (np.eye(3) - K) @ P
    return p, 0.5 * (P + P.T), K


def sensor_qos():
    # depth=5: 컬러·depth ApproximateTimeSynchronizer가 한쪽 프레임을 기다릴 여유.
    # depth=1로 줄이면 두 스트림의 도착 순서가 조금만 어긋나도 sync가 굶는다.
    return QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=5)


# HSV 색상(OpenCV H: 0~180) 기준 — 퍽 색 (시뮬레이터 scene.PALETTE 실측 H: 빨강 0 · 주황 15 · 노랑 28 · 초록 60 · 파랑 109 · 보라 142).
# 주황은 17: 밝은 조명(1.45배)에서 빨강 채널이 포화되면 주황 H가 22.6까지 올라감 — 노랑(30)과의 경계를 23.5로
HUES = {"red": (0, 180), "orange": (17,), "yellow": (30,), "green": (60,), "blue": (110,), "purple": (142,)}
LAMP_COLORS = ("red", "yellow", "green")  # 신호등 램프 — 이 셋 중에서만
ACHROMATIC_S = 60    # 채도가 이보다 낮은 화소가 대부분이면 무채색 퍽 (흰색 · 검정)
WHITE_V, BLACK_V = 110, 80  # 무채색일 때 밝기 중앙값 경계 — 조명 0.55~1.45배 (흰 235 → 129 · 검정 40 → 58)


def _nearest_hue(h, names=None):
    return min(names or HUES, key=lambda c: min(min(abs(h - x), 180 - abs(h - x)) for x in HUES[c]))


def classify_color(cls, roi):
    """박스 안 색 → 퍽: 박스 가운데(가장자리 20%씩 뺌 — 바닥이 섞이지 않게) 화소의 대표 색상, 채도 없는 화소가 대부분이면
    흰색·검정 / 신호등: 켜진 램프(밝고 채도 높은 화소) 색 · 없으면 off. 그 외 ''"""
    if roi.size == 0 or cls not in ("puck", "puck_lying", "traffic_light"):
        return ""
    if cls != "traffic_light":
        h0, w0 = roi.shape[:2]
        mh, mw = int(h0 * 0.2), int(w0 * 0.2)
        roi = roi[mh:h0 - mh, mw:w0 - mw] if h0 - 2 * mh > 0 and w0 - 2 * mw > 0 else roi
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0].ravel(), hsv[..., 1].ravel(), hsv[..., 2].ravel()
    if cls == "traffic_light":
        lit = (s > 100) & (v > 90)  # 켜짐 V 180~230 · 꺼짐 45~57 (시뮬 램프) — 조명 0.55~1.45배에서도 갈리는 경계
        if lit.sum() < 4:
            return "off"
        sel, names = h[lit], LAMP_COLORS
        votes = {}
        for hv in sel[:: max(1, sel.size // 400)]:
            c = _nearest_hue(int(hv), names)
            votes[c] = votes.get(c, 0) + 1
        total = sum(votes.values())
        if sum(1 for n in votes.values() if n >= 0.15 * total) >= 2:
            return "all"  # 두 색 이상 동시에 켜짐 — 신호등 미션 단계 구분 신호 (세 색 동시 3초)
        return max(votes, key=votes.get)
    else:
        if (s < ACHROMATIC_S).mean() > 0.6:
            med = float(np.median(v))
            return "white" if med >= WHITE_V else "black" if med <= BLACK_V else "unknown"
        sel, names = h[(s > 80) & (v > 50)], None
        if sel.size < 8:
            return "unknown"
    # 빨강은 0과 180 양끝에 걸치므로 원형 거리로 가장 많은 색을 고른다
    votes = {}
    for hv in sel[:: max(1, sel.size // 400)]:
        c = _nearest_hue(int(hv), names)
        votes[c] = votes.get(c, 0) + 1
    return max(votes, key=votes.get)


def resolve_model_path(model_path, model_dir):
    """model_path 파라미터 → 실제 가중치 파일 경로 ('' = 모델 없음 — 빈 목록 발행)
      ''              → model_dir/model.pt (팀 기본 — 있으면)
      파일 이름만     → model_dir/<이름> (예: bee_yolo11n_v1.pt — ros2_ws/model/yolo/*.pt가 share/perception/model에 설치됨)
      그 밖(경로)     → 그대로 (전체 경로를 준 경우 · 없으면 YOLO 로드 실패 경고)"""
    if not model_path:
        cand = os.path.join(model_dir, "model.pt") if model_dir else ""
        return cand if cand and os.path.exists(cand) else ""
    if os.sep not in model_path and model_dir:
        cand = os.path.join(model_dir, model_path)
        if os.path.exists(cand):
            return cand
    return model_path


def split_class(name):
    """YOLO 클래스 이름 → (설계 이름, 이름에 든 색) — 모델이 색별로 배워도 나갈 때는 설계 이름 + color (받는 쪽 수정 없음)
      새 모델: puck_red → ('puck', 'red') · traffic_light_off → ('traffic_light', 'off')
      팀 모델: puck · puck_lying → ('puck', '') (자세는 클래스 이름 대신 크기로 판단) · 그 밖은 그대로 ('person', '')"""
    if name.startswith("puck"):
        return "puck", next((t for t in name.split("_")[1:] if t in HUES), "")
    if name.startswith("traffic_light_"):
        state = name[len("traffic_light_"):]
        return "traffic_light", state if state in LAMP_COLORS + ("off",) else ""
    return name, ""


def box_depth_m(depth_m, box, min_valid):
    """퍽 박스 가운데 1/3 영역의 깊이 중앙값 [m] — 가장자리 배경이 섞이지 않게. 0(측정 실패) 제외
    측정된 화소가 영역의 min_valid(비율)보다 적으면 0.0 (= 깊이 없음):
    퍽이 18cm보다 가까우면 퍽 화소는 0이 되고 사이로 보인 배경 몇 화소만 남아, 그 중앙값(1.2m)을 퍽 거리로 쓰던 문제 (10/21 실측)"""
    x0, y0, x1, y1 = box
    bw, bh = x1 - x0, y1 - y0
    patch = depth_m[y0 + bh // 3:y1 - bh // 3 + 1, x0 + bw // 3:x1 - bw // 3 + 1]
    valid = patch[np.isfinite(patch) & (patch > 0.05)]
    if valid.size == 0 or valid.size < min_valid * patch.size:
        return 0.0
    return float(np.median(valid))


def puck_color(roi, colors, hue_tol):
    """퍽 박스 안 색 → (색 이름, H 중앙값). colors = 퍽으로 볼 색 이름들 (HUES 안의 것)
    가장자리 20%씩 뺀 안쪽에서 채도·밝기 있는 화소마다 가장 가까운 색에 투표 — 그 색 기준 H에서 hue_tol 넘게 멀면 'other'에 투표
    (classify_color처럼 가장 가까운 색으로 억지로 맞추면 주황 컵홀더 H12~13이 빨강이 됨 — 10/21 실측)
    색 화소가 너무 적으면 ('unknown', -1)"""
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
    d = np.stack([np.min([np.minimum(abs(h - x), 180 - abs(h - x)) for x in HUES[c]], axis=0) for c in names])
    pick = np.where(d.min(axis=0) <= hue_tol, d.argmin(axis=0), len(names))
    votes = np.bincount(pick, minlength=len(names) + 1)
    return (names + ["other"])[int(np.argmax(votes))], int(np.median(h))


def qr_crop(bgr, box):
    """YOLO qr_board 박스 → 여유 15% 잘라 짧은 변 300px 이상으로 확대 (QR 모듈이 뭉개지지 않게)"""
    x0, y0, x1, y1 = box
    h, w = bgr.shape[:2]
    mx, my = int((x1 - x0) * 0.15), int((y1 - y0) * 0.15)
    crop = bgr[max(0, y0 - my):min(h, y1 + my), max(0, x0 - mx):min(w, x1 + mx)]
    k = max(1.0, 300.0 / max(1, min(crop.shape[:2])))
    return cv2.resize(crop, None, fx=k, fy=k, interpolation=cv2.INTER_CUBIC) if k > 1 else crop


def tag_detector():
    """AprilTag tag36h11 검출 함수 (OpenCV aruco — 새 라이브러리 없이) · aruco가 없으면 None.
    OpenCV 4.7+ ArucoDetector 클래스 · 4.6 이하(Ubuntu 24.04 apt python3-opencv 4.6) detectMarkers 함수 둘 다.
    작은 퍽 태그(16mm)도 잡게 둘레 하한을 낮춤 (기본 0.03 = 640px에서 둘레 19px)"""
    aruco = getattr(cv2, "aruco", None)
    if aruco is None or not hasattr(aruco, "DICT_APRILTAG_36h11"):
        return None
    if hasattr(aruco, "ArucoDetector"):
        params = aruco.DetectorParameters()
        params.minMarkerPerimeterRate = 0.01
        det = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11), params)
        return det.detectMarkers
    params = aruco.DetectorParameters_create()
    params.minMarkerPerimeterRate = 0.01
    dic = aruco.Dictionary_get(aruco.DICT_APRILTAG_36h11)
    return lambda gray: aruco.detectMarkers(gray, dic, parameters=params)


BARCODE_LIB = "zxing-cpp"  # barcode_detector가 고른 것 (진단 표시)


def barcode_detector():
    """바코드 검출 함수 → (ok, 문자열들, 종류들, 꼭짓점들[N×4×2]) · 없으면 None.
    발제 바코드(P2-01 — 영문·하이픈)는 Code128 계열이라 OpenCV barcode(EAN·UPC 숫자 상품코드만)로는 못 읽음 (실측: 4.6·5.0 둘 다 실패)
    → zxing-cpp(pip zxing-cpp)가 있으면 그것, 없으면 OpenCV (EAN·UPC만)"""
    try:
        import zxingcpp
    except ImportError:
        zxingcpp = None
    global BARCODE_LIB
    if zxingcpp is not None:
        BARCODE_LIB = "zxing-cpp (Code128·Code39·QR 등)"

        def read(bgr):
            rs = [r for r in zxingcpp.read_barcodes(bgr) if r.text]
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


def detect_tags(detector, bgr):
    """영상 → [(태그 번호, 중심 (u, v), 박스 (x0, y0, x1, y1))]"""
    if detector is None:
        return []
    corners, ids, _ = detector(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
    if ids is None:
        return []
    out = []
    for c, i in zip(corners, ids.ravel()):
        pts = c.reshape(-1, 2)
        (x0, y0), (x1, y1) = pts.min(0), pts.max(0)
        out.append((int(i), (float(pts[:, 0].mean()), float(pts[:, 1].mean())), (int(x0), int(y0), int(x1) + 1, int(y1) + 1)))
    return out


def tag_in_box(tags, box, ids):
    """퍽 박스 안(여유 10%)에 중심이 있는 퍽 태그 번호 — 없으면 None"""
    x0, y0, x1, y1 = box
    mx, my = (x1 - x0) * 0.1, (y1 - y0) * 0.1
    for tid, (u, v), _ in tags:
        if tid in ids and x0 - mx <= u <= x1 + mx and y0 - my <= v <= y1 + my:
            return tid
    return None


def tag_color(tid, tag_colors):
    """퍽 태그 번호 → 색 (tag_colors = {색: [번호…]}) · 모르면 ''"""
    return next((c for c, ids in tag_colors.items() if tid in ids), "")


def to_msg(tid, tr):
    """트랙(또는 한 검출) dict → TrackedObject — 위치 공분산 3×3을 pose.covariance에, 방향은 모름"""
    o = TrackedObject()
    o.id = tid
    o.class_name = tr["class_name"]
    if o.class_name == "puck" and tr.get("posture") == "lying":  # 트랙은 퍽 계열을 'puck'으로 저장 → 발행은 설계 이름 puck / puck_lying
        o.class_name = "puck_lying"
    o.confidence = float(tr.get("conf", 0.5))
    o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = (float(v) for v in tr["p"])
    o.pose.pose.orientation.w = 1.0
    cov = np.zeros((6, 6))
    cov[:3, :3] = tr["P"]
    cov[3, 3] = cov[4, 4] = cov[5, 5] = ROT_VAR_UNKNOWN
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
    if tr.get("tag") is not None:  # AprilTag 번호 — 퍽 0~11 · 장소 12~14
        details.append(KeyValue(key="tag", value=str(tr["tag"])))
    if tr.get("content"):  # 바코드 문자열 (장소 작업 지시)
        details.append(KeyValue(key="content", value=tr["content"]))
    o.details = details
    return o


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("model_path", "")
        self.declare_parameter("confidence", 0.4)
        self.declare_parameter("imgsz", 416)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("cpu_threads", 2)
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("match_dist", 0.4)
        self.declare_parameter("pixel_sigma", 4.0)
        self.declare_parameter("depth_sigma0", 0.005)
        self.declare_parameter("depth_sigma_k", 0.01)
        self.declare_parameter("process_noise", 0.01)
        self.declare_parameter("process_noise_dynamic", 1.0)
        self.declare_parameter("track_frame", "odom")  # 추적·융합 좌표계 (끊김 없음) — 발행은 map
        self.track_frame = str(self.get_parameter("track_frame").value)
        self._out_tf = None  # 마지막으로 성공한 map ← track_frame (회전, 이동) — TF가 잠깐 끊겨도 이걸로
        self.declare_parameter("self_filter", True)
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("self_links", ["shoulder_link", "upper_arm_link", "forearm_link", "wrist_link",
                                              "gripper_link", "end_effector_link"])
        self.declare_parameter("self_radius", 0.06)
        self.declare_parameter("body_radius", 0.2)
        self.declare_parameter("body_height", 0.6)
        self.declare_parameter("self_arm_hits", 3)  # 퍽이 아닌 검출 박스 안에 팔 투영점이 이만큼 있으면 자기 팔 (0 = 안 봄)
        # 기억하는 물체(REMEMBER_PREFIXES) 지우기 기준 — expected_visible
        self.declare_parameter("miss_count", 3)            # 잘 보이는 자리에서 연속 못 찾은 횟수 — 이만큼이면 지움
        self.declare_parameter("miss_max_range", 3.0)      # 이보다 먼 자리는 못 찾아도 세지 않음 [m]
        self.declare_parameter("miss_edge_px", 40)         # 화면 가장자리 이만큼 안은 세지 않음 [px]
        self.declare_parameter("miss_occlusion_m", 0.10)   # 그 자리 깊이가 이만큼 넘게 가까우면 가려짐 [m]
        self.declare_parameter("remember_min_hits", 3)     # 같은 자리에서 이만큼 잡혀야 기억 (그 전엔 2초 규칙 — 유령 퍽 방지)
        self.miss_count = int(self.get_parameter("miss_count").value)
        self.remember_min_hits = int(self.get_parameter("remember_min_hits").value)
        self.miss_max_range = float(self.get_parameter("miss_max_range").value)
        self.miss_edge_px = int(self.get_parameter("miss_edge_px").value)
        self.miss_occlusion_m = float(self.get_parameter("miss_occlusion_m").value)
        # 퍽 판단 (judge_puck) — 기준값은 test/puck_tag_check.py 실물 측정 (~/yolo_puck test_results_20261021.md · 10/22 bee_v3ft)
        #   퍽 색·크기는 발제문이 나오면 yaml에서 바꿀 것 (모양 원기둥·사각기둥은 구분 안 함 — 팀 결정)
        self.declare_parameter("puck_colors", ["red", "yellow", "green", "blue"])  # 퍽으로 볼 색 (파랑 = 지금 실물 테스트용)
        self.declare_parameter("hue_tol", 10)               # 기준 H에서 이만큼 안이어야 그 색 (빨간 퍽 H1~10 · 주황 컵홀더 H12~13 실측)
        self.declare_parameter("check_model_color", True)   # YOLO 이름의 색(puck_red)과 HSV 색이 다르면 퍽 아님
        self.declare_parameter("size_long_min_cm", 2.5)     # 박스 긴 변 하한 [cm] (끝면·윗면 3×3만 보이면 2.5~3.9)
        self.declare_parameter("size_long_max_cm", 10.0)    # 긴 변 상한 [cm] (누운 퍽 6.5~7.2 · 멀면 박스가 넉넉해져 8.5)
        self.declare_parameter("size_short_min_cm", 2.2)    # 짧은 변 하한 [cm] (빨대 1.0~1.5 거름)
        self.declare_parameter("square_long_cm", 4.5)       # 긴 변이 이보다 작고 정사각형에 가까우면 끝면·윗면 — 자세 모름
        self.declare_parameter("square_ratio", [0.8, 1.25])  # 정사각형으로 보는 높이÷폭 범위
        self.declare_parameter("stand_ratio", 0.8)          # 높이÷폭 이 이상이면 서있음, 미만이면 누움
        self.declare_parameter("edge_px", 5)                # 박스가 화면 가장자리 이 안에 닿으면 잘린 퍽 — 크기 하한·자세 안 봄
        self.declare_parameter("near_px", 70)               # 깊이 없는데 박스 짧은 변이 이 이상이면 너무 가까운 퍽 (3cm 퍽 70px ≈ 25cm)
        self.declare_parameter("depth_min_valid", 0.5)      # 박스 가운데 깊이 화소가 이 비율보다 적으면 깊이 없음
        # 표식 (발제 배포 표식) — 퍽 윗면 AprilTag 번호 → 색, 장소 태그 번호
        self.declare_parameter("puck_tags_red", [0, 1, 2, 3])
        self.declare_parameter("puck_tags_green", [4, 5, 6, 7])
        self.declare_parameter("puck_tags_blue", [8, 9, 10, 11])
        self.declare_parameter("location_tags", [12, 13, 14])
        self.tag_colors = {c: [int(v) for v in self.get_parameter(f"puck_tags_{c}").value] for c in ("red", "green", "blue")}
        self.puck_tag_ids = {t for ids in self.tag_colors.values() for t in ids}
        self.location_tags = {int(v) for v in self.get_parameter("location_tags").value}
        self.tag_det = tag_detector()
        self.barcode = barcode_detector()
        self.puck_colors = [str(v) for v in self.get_parameter("puck_colors").value]
        self.hue_tol = int(self.get_parameter("hue_tol").value)
        self.check_model_color = bool(self.get_parameter("check_model_color").value)
        self.long_min = float(self.get_parameter("size_long_min_cm").value)
        self.long_max = float(self.get_parameter("size_long_max_cm").value)
        self.short_min = float(self.get_parameter("size_short_min_cm").value)
        self.square_long = float(self.get_parameter("square_long_cm").value)
        self.square_ratio = [float(v) for v in self.get_parameter("square_ratio").value]
        self.stand_ratio = float(self.get_parameter("stand_ratio").value)
        self.edge_px = int(self.get_parameter("edge_px").value)
        self.near_px = int(self.get_parameter("near_px").value)
        self.depth_min_valid = float(self.get_parameter("depth_min_valid").value)
        self.self_filter = bool(self.get_parameter("self_filter").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.self_links = [str(v) for v in self.get_parameter("self_links").value]
        self.self_radius = float(self.get_parameter("self_radius").value)
        self.self_arm_hits = int(self.get_parameter("self_arm_hits").value)
        self.body_radius = float(self.get_parameter("body_radius").value)
        self.body_height = float(self.get_parameter("body_height").value)
        self.match_dist = float(self.get_parameter("match_dist").value)
        self.pixel_sigma = float(self.get_parameter("pixel_sigma").value)
        self.depth_sigma0 = float(self.get_parameter("depth_sigma0").value)
        self.depth_sigma_k = float(self.get_parameter("depth_sigma_k").value)
        self.q = float(self.get_parameter("process_noise").value)
        self.q_dyn = float(self.get_parameter("process_noise_dynamic").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        conf = float(self.get_parameter("confidence").value)
        self.conf_th = conf
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value)

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
        self.tracks = {}  # id → dict(class_name(퍽 계열은 puck), posture, p[3], P[3×3], t, size, last_seen, seen{cam: t}, miss, ...)
        self.next_id = 1
        self.published = 0
        self.synced = {}       # cam → 동기화된 컬러·depth 쌍 수
        self.tf_fallback = 0   # 영상 시각 TF가 없어 최신 TF로 대신한 횟수
        self.self_dropped = 0  # 자기 몸(팔·차체)이라 버린 검출 수
        # QR 문제 (documents/mission/README.md §3-1): A 지역 QR 판을 읽어 /mission/qr 로 — 모델 없이도 동작
        self.qr = cv2.QRCodeDetector()
        self.qr_text = ""
        self.qr_seen = 0.0   # 마지막으로 QR을 읽은 시각 — QR_GAP_S 넘게 안 보였다 다시 보이면 같은 내용도 재발행 (다음 미션 지역)
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
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 1)
        self.create_timer(1.0 / max(self.rate_hz, 0.1), self.on_publish)
        self.create_timer(0.25, self.on_infer)  # 카메라당 최대 4Hz 추론
        self.create_timer(1.0, self.on_diag)
        self.create_timer(0.5, self.on_qr)

    # ---------- 입력 캐시
    def _slot(self, cam):
        with self.lock:
            return self.frames.setdefault(cam, {})

    def on_frame(self, cam, color, depth):
        """동기화된 컬러·depth 한 쌍 — 시각·프레임은 컬러 기준 (depth는 컬러에 정렬된 영상)"""
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
        tf = self.lookup_tf(s["frame_id"], now_stamp)
        if tf is None:  # 위치는 못 구해도 QR 판 박스는 남김 — QR은 위치 없이 읽힘 (TF·지도가 아직 없을 때도)
            boxes = [tuple(max(0, int(v)) for v in b.xyxy[0].tolist()) for b in res.boxes
                     if self.names.get(int(b.cls[0])) == "qr_board"]
            with self.lock:
                self.frames[cam]["qr_boxes"] = boxes
            return
        rot, trans = tf
        tags = detect_tags(self.tag_det, s["bgr"])
        body = self.robot_body(now_stamp) if self.self_filter and len(res.boxes) else None
        # 영상에서 팔 판정: 박스 일부만 팔이면 depth 중앙값이 팔 뒤 바닥·벽까지라 3D 판정(is_self)을 빠져나갔음
        # (실측: 스캔 자세 팔을 obstacle 0.5~0.7로 잡고 위치는 팔 뒤 바닥 (0.42, 0.20, −0.03)) → 팔을 화면에 투영해 박스 안에 있으면 버림
        arm_px = (arm_pixels(body[1], rot, trans, s["fx"], s["fy"], s["cx"], s["cy"])
                  if body is not None and self.self_arm_hits > 0 else np.zeros((0, 2)))
        dets = []
        qr_boxes = []
        for b in res.boxes:
            tag = None
            name = self.names.get(int(b.cls[0]))
            if name is None:
                continue
            cls, model_color = split_class(name)  # 모델 이름(puck_red) → 설계 이름(puck) + 이름에 든 색
            x0, y0, x1, y1 = (int(v) for v in b.xyxy[0].tolist())
            x0, y0 = max(x0, 0), max(y0, 0)
            x1, y1 = min(x1, w - 1), min(y1, h - 1)
            if x1 <= x0 or y1 <= y0:
                continue
            posture = ""
            if cls == "puck":
                # 퍽 판단 — 깊이 50% 규칙 · 색 · 크기·자세 · 잘림·근접 (YOLO 이름의 자세는 안 믿음 — 실물에서 틀렸음)
                zm = box_depth_m(depth_m, (x0, y0, x1, y1), self.depth_min_valid)
                color, _ = puck_color(s["bgr"][y0:y1, x0:x1], self.puck_colors, self.hue_tol)
                m = self.edge_px
                clipped = x0 <= m or y0 <= m or x1 >= w - 1 - m or y1 >= h - 1 - m
                near = min(x1 - x0, y1 - y0) >= self.near_px
                wcm = (x1 - x0) * zm / s["fx"] * 100 if zm > 0 else -1.0
                hcm = (y1 - y0) * zm / s["fy"] * 100 if zm > 0 else -1.0
                ok, _, posture = self.judge_puck(wcm, hcm, color, clipped, model_color, near)
                if not ok or zm <= 0:
                    # 퍽 아님 → 버림 · 퍽인데 깊이 없음(너무 가까움) → 위치를 몰라 이번엔 갱신 안 함 = 트랙은 마지막 위치 유지
                    # (그 자리는 깊이가 없어 expected_visible이 '못 봄'으로 세지 않음 → 기억도 안 지워짐)
                    continue
                if posture == "lying":
                    cls = "puck_lying"  # 발행 이름은 설계대로 puck / puck_lying
                tag = tag_in_box(tags, (x0, y0, x1, y1), self.puck_tag_ids)
                if tag is not None:
                    color = tag_color(tag, self.tag_colors) or color  # 윗면 태그가 색보다 확실 (R·G·B 번호대)
            else:
                region = depth_m[y0:y1, x0:x1]
                valid = region[(region > 0.05) & (region < 6.0)]
                if valid.size < 8:
                    continue
                zm = float(np.median(valid))
                # 신호등: 새 모델은 이름에 상태가 있음(traffic_light_red) → 그대로 · 팀 모델(traffic_light)은 램프 HSV로
                color = model_color or classify_color(cls, s["bgr"][y0:y1, x0:x1])
            u, v = (x0 + x1) / 2.0, (y0 + y1) / 2.0
            px = (u - s["cx"]) * zm / s["fx"]
            py = (v - s["cy"]) * zm / s["fy"]
            if cls == "qr_board":
                qr_boxes.append((x0, y0, x1, y1))
            z_map = rot @ np.array([px, py, zm]) + trans
            arm_in_box = cls not in PUCK_CLASSES and box_has_arm((x0, y0, x1, y1), arm_px, self.self_arm_hits)  # 퍽은 그리퍼 밑에서 집으므로 뺌
            if body is not None and (arm_in_box or is_self(cls, z_map, body, self.body_radius, self.body_height, self.self_radius)):
                with self.lock:
                    self.self_dropped += 1
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
                "tag": tag,          # 퍽 윗면 AprilTag 번호 (못 읽으면 None — 합칠 때 아는 번호를 덮지 않음)
            })
        for tid_tag, (u, v), box in tags:  # 장소 태그 (L1~L3) — YOLO가 아니라 태그 자체가 물체
            if tid_tag not in self.location_tags:
                continue
            d = self.marker_det(s, box, (u, v), rot, trans, "location")
            if d is not None:
                d["tag"] = tid_tag
                dets.append(d)
        now_s = stamp_s(now_stamp)
        view = (rot, trans, s["fx"], s["fy"], s["cx"], s["cy"], depth_m)  # 이 카메라 이 프레임 — 기억한 물체 지우기 판정용
        with self.lock:
            self.frames[cam]["qr_boxes"] = qr_boxes  # on_qr가 이 박스 안만 해독
            tids = self.fuse(cam, now_stamp, dets)
            self.prune(now_s, set(tids), view)
        out = TrackedObjectArray()
        out.header.stamp = now_stamp
        out.header.frame_id = "map"
        out.objects = [to_msg(tid, self.to_map({**d, "p": d["z"], "P": d["R"], "last_seen": now_stamp, "camera": cam}))
                       for tid, d in zip(tids, dets)]
        self.det_pubs[cam].publish(out)

    def marker_det(self, s, box, center, rot, trans, cls):
        """표식(장소 태그 · 바코드) 영상 박스 → 검출 dict (위치 = 박스 안 깊이 중앙값) · 깊이 없으면 None"""
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

    def judge_puck(self, wcm, hcm, color, clipped=False, model_color="", near=False):
        """퍽 후보 박스의 실제 크기(폭·높이 cm, 깊이 없으면 ≤0)·색·잘림·YOLO 이름의 색·가까움 → (퍽인가, 아니면 이유, 자세)
        자세 = standing / lying · 잘렸거나 끝면·윗면만 보이거나 깊이가 없으면 '' (모름) — test/puck_tag_check.py judge와 같음"""
        # 색 검사를 먼저 — 깊이가 없어도 색은 볼 수 있음
        if color == "unknown":
            return False, "색 없음", ""     # 흰·회색 물체 (퍽은 색이 있음)
        if color not in self.puck_colors:
            return False, "색", ""          # 퍽 색 밖 (other = 주황·보라 등)
        if self.check_model_color and model_color and model_color != color:
            return False, "색 불일치", ""   # YOLO는 파랑이라는데 HSV는 빨강 (주황 컵홀더가 그렇게 통과했었음 — 실측)
        if wcm <= 0 or hcm <= 0:
            if clipped or near:
                # D435는 약 16~18cm보다 가까우면 깊이를 못 잼 — 집기 직전이라 놓치면 안 됨: 퍽은 맞음, 거리·자세 모름
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
            tr["miss"] = 0  # 다시 잡혔으니 '연속 못 찾음'을 처음부터
            tr["hits"] = tr.get("hits", 1) + 1  # 같은 자리에서 잡힌 횟수 — remember_min_hits 이상이면 기억 (prune)
            tr.update(conf=d["conf"], camera=cam, px=d["px"])
            if tr["class_name"] != "puck" or d["color"] not in NO_COLOR:  # 퍽은 색을 못 정한 관측이 아는 색을 덮지 않게
                tr["color"] = d["color"]
            if d.get("posture"):  # 자세를 아는 관측만 자세를 바꿈 ('모름'은 덮어쓰지 않음)
                tr["posture"] = d["posture"]
            if d.get("tag") is not None:  # 태그 번호는 한 번 읽으면 기억 (멀어지면 못 읽어도 덮지 않음)
                tr["tag"] = d["tag"]
            if d.get("content"):
                tr["content"] = d["content"]
        for i, d in enumerate(dets):
            if i in used_d:
                continue
            self.tracks[self.next_id] = {
                "class_name": kind(d["class_name"]), "posture": d.get("posture", ""),
                "p": d["z"], "P": d["R"], "t": t, "size": d["size"],
                "conf": d["conf"], "last_seen": stamp, "seen": {cam: t}, "miss": 0, "hits": 1,
                "camera": cam, "px": d["px"], "color": d["color"], "tag": d.get("tag"), "content": d.get("content", "")}
            used_d[i] = self.next_id
            self.next_id += 1
        return [used_d[i] for i in range(len(dets))]

    def prune(self, now_s, matched, view):
        """한 카메라 한 추론 뒤 트랙 정리 (lock 안에서 호출) — matched = 이번에 검출이 붙은 id들 ·
        view = 그 카메라 (rot, trans, fx, fy, cx, cy, depth_m)
          기억 안 하는 물체(사람·장애물·신호등): TRACK_TIMEOUT_S 넘게 어느 카메라에도 안 보이면 삭제 (움직이거나 색이 바뀌어 옛 정보가 위험)
          기억하는 물체(REMEMBER_PREFIXES): 시간으로는 안 지움. 이번에 안 잡혔는데 그 자리가 잘 보였어야 하면 miss +1 → miss_count면 삭제
            단 remember_min_hits번 넘게 잡히기 전(확인 전)에는 기억 안 하는 물체처럼 2초 규칙 — 카메라 자세(TF)가 영상과
            잠깐 어긋나 한두 번 엉뚱한 자리로 찍힌 '유령 퍽'이 남지 않게 (10/22 시뮬레이터: 같은 퍽이 9곳에 남았음)"""
        for tid, tr in list(self.tracks.items()):
            if not remembered(tr["class_name"]) or tr.get("hits", 1) < self.remember_min_hits:
                if now_s - tr["t"] > TRACK_TIMEOUT_S:
                    del self.tracks[tid]
                continue
            if tid in matched:
                continue
            if expected_visible(tr["p"], *view, self.miss_max_range, self.miss_edge_px, self.miss_occlusion_m):
                tr["miss"] = tr.get("miss", 0) + 1
                if tr["miss"] >= self.miss_count:
                    del self.tracks[tid]

    def robot_body(self, stamp):
        """그 시각 로봇 몸 (차체 원점, [팔 체인 링크 원점…]) — map 기준. 차체 TF가 없으면 None (필터 안 함)"""
        base = self.lookup_tf(self.base_frame, stamp, quiet=True)
        if base is None:
            return None
        chain = [t[1] for t in (self.lookup_tf(link, stamp, quiet=True) for link in self.self_links) if t is not None]
        return base[1], chain

    def to_map(self, tr):
        """추적 좌표(odom) 트랙·검출 dict → map 기준 사본 (위치·공분산 회전) — 지금의 map ← odom (RTAB-Map 보정 최신값)"""
        try:
            t = self.tf.lookup_transform("map", self.track_frame, rclpy.time.Time())
            tt = t.transform.translation
            self._out_tf = (quat_to_rot(t.transform.rotation), np.array([tt.x, tt.y, tt.z]))
        except Exception as e:  # noqa: BLE001
            if self._out_tf is None:  # 아직 map이 없음 (RTAB-Map 기동 전) — odom 그대로 (map 원점 = odom 원점으로 시작)
                self.get_logger().warning(f"map ← {self.track_frame} TF 없음 — 그대로 발행: {e}", throttle_duration_sec=5.0)
                return tr
        rot, trans = self._out_tf
        return {**tr, "p": rot @ np.asarray(tr["p"]) + trans, "P": rot @ np.asarray(tr["P"]) @ rot.T}

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

    # ---------- 발행 (10Hz)
    def on_publish(self):
        out = TrackedObjectArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        with self.lock:
            items = [(tid, dict(tr)) for tid, tr in sorted(self.tracks.items())]
        out.objects = [to_msg(tid, self.to_map(tr)) for tid, tr in items]  # 추적은 odom — 발행 순간의 map ← odom으로
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
        self.read_barcodes()

    # ---------- 바코드 (2Hz) — 장소에 붙은 작업 지시 문자열 (발제 P2-01 …). 물체(barcode, 위치 = 붙은 곳)로 기억 + /mission/qr
    def read_barcodes(self):
        if self.barcode is None:
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
                    self.get_logger().info(f"바코드 ({cam}): {text}")
                self.qr_seen = now
            if dets:
                with self.lock:
                    self.fuse(cam, s["stamp"], dets)

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
            # 최근 TRACK_TIMEOUT_S 안에 두 카메라가 모두 본 트랙 = 실제로 융합 중인 물체
            fused = sum(1 for tr in self.tracks.values()
                        if len(tr["seen"]) > 1 and max(tr["seen"].values()) - min(tr["seen"].values()) < TRACK_TIMEOUT_S)
            pub = self.published
            synced = dict(self.synced)
            fallback = self.tf_fallback
            dropped = self.self_dropped
        s.values = [KeyValue(key="추적 물체", value=str(n)),
                    KeyValue(key="두 카메라 융합 물체", value=str(fused)),
                    KeyValue(key="동기화 프레임",
                             value=" · ".join(f"{c} {k}" for c, k in sorted(synced.items())) or "0"),
                    KeyValue(key="TF 최신 대체", value=str(fallback)),
                    KeyValue(key="자기 몸이라 버린 검출", value=str(dropped)),
                    KeyValue(key="발행한 /tracked_objects", value=str(pub)),
                    KeyValue(key="QR 문제", value=self.qr_text or "(아직 없음)"),
                    KeyValue(key="AprilTag (퍽 번호·장소)", value=f"OpenCV {cv2.__version__} " + ("aruco 있음" if self.tag_det else "aruco 없음 — 퍽 번호 못 읽음")),
                    KeyValue(key="바코드 (장소 작업 지시)", value=BARCODE_LIB if self.barcode else "없음 — pip install zxing-cpp")]
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
