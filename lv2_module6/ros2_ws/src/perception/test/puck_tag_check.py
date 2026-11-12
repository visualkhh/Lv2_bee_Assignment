#!/usr/bin/env python3
"""퍽 인지 + AprilTag 번호 붙이기 테스트 노드 — RealSense 2대(pole_front_cam · gripper_cam)

  멀리서는 2.5cm 태그가 안 읽히므로 퍽은 YOLO로 찾고, 가까워져 태그가 읽히면 그 퍽에 태그 번호를 붙인다.
  (완성본은 perception_node.py에 옮길 예정 — 여기는 test 폴더의 확인용)

  입력: /<cam>/color/image_raw · /<cam>/aligned_depth_to_color/image_raw · /<cam>/color/camera_info (best_effort)
  출력: /puck_tag_check/<cam>/pucks (std_msgs/String, JSON) — 한 프레임에서 찾은 퍽 목록
          [{"is_puck": true, "reason": "", "posture": "standing", "class": "puck", "score": 0.91,
            "color": "red", "hue": 5, "width_cm": 4.9, "height_cm": 5.3,
            "distance": 0.52, "x": 0.02, "y": 0.01, "z": 0.52, "tag": 3}, ...]
          (is_puck = 아래 ⑤·⑥ 판단 · reason = 퍽 아닌 이유 · posture = standing / lying / ''(잘려서 모름) ·
           tag_state = confirmed(태그 확인) / candidate(아직) / fake(가까운데 태그 없음) / ''(퍽 아님·끔) ·
           clipped = 박스가 화면 가장자리에 닿음(퍽 일부가 화면 밖) ·
           tag = -1: 태그 못 읽음 · 좌표는 카메라 기준 m · 크기 = 박스의 실제 폭×높이, -1 = depth 없음)
        로그 (카메라마다 0.5초에 한 번):
          "pole_front_cam ✅퍽 서있음 red(H5) 4.9x5.3cm: 0.520 m (x +0.020, ...) [puck 0.91] tag 3"
          "pole_front_cam ❌퍽아님(크기) red(H12) 7.3x4.5cm: 0.780 m (...) [puck_lying 0.73] tag 없음"

  방법 (카메라마다 infer_hz 주기로):
    ① YOLO로 퍽 후보 박스 (puck_classes에 든 클래스만 — 클래스 이름의 모양·자세는 안 믿음)
    ② 박스 가운데 1/3 영역 depth 중앙값 = z → camera_info로 박스 중심을 3D 역투영 → 직선거리
    ③ 색: 박스 가장자리 20%씩 뺀 안쪽 화소 중 채도·밝기 있는 것의 H(색상)를 4색 중 가까운 쪽에 투표
       (perception_node.py의 classify_color와 같은 방식) — 기준 H에서 hue_tol 넘게 벗어난 화소는 'other'(주황·보라 등)
    ④ 같은 영상에서 AprilTag 검출 → 태그 중심이 들어간 퍽 박스(여럿이면 가장 작은 것)에 그 번호를 붙임
    ⑥ 태그로 진짜 퍽 확인 (tag_confirm): 멀리는 퍽 '후보' · 가까이서 태그가 읽히면 '확인'(계속 유지) ·
       tag_check_m 안인데 tag_miss번 연속 태그가 없으면 '가짜' → 퍽 아님 (손에 든 같은 색 원기둥·컵홀더)
       프레임 사이 같은 물체는 화면 위치(track_px)·색으로 이어 붙임 — TF가 없는 테스트용 (perception_node는 odom 트랙에)
    ⑤ 퍽 판단 (모양 원기둥/사각기둥은 구분 안 함 — 팀 결정): 크기가 퍽 범위 안 + 색이 puck_colors 안
       자세: 높이÷폭 ≥ stand_ratio → 서 있음, 미만 → 누움 (팀 모델 puck/puck_lying 클래스는 실물에서 틀려 안 씀)
       박스가 화면 가장자리(edge_px)에 닿으면 잘린 퍽 → 크기 하한·자세는 건너뜀 (상한 넘으면 "거리 이상") ·
         깊이 없음인데 잘렸거나 화면에서 크면(near_px — 깊이 카메라 최소 거리보다 가까움) YOLO + 색만으로 퍽, 거리 모름 ·
         박스 가운데 깊이 화소가 depth_min_valid보다 적으면 깊이 없음 (배경 몇 화소로 엉뚱한 거리 안 냄) ·
         작고 정사각형에 가까운 박스(끝면·윗면 3×3)는 자세 모름
       YOLO 클래스 이름에 색이 있으면(puck_box_blue 등) HSV 색과 다르면 "색 불일치"로 퍽 아님 (check_model_color)
       기준값은 2026-10-21 책상 위 파란 사각기둥 실측 (~/yolo_puck/test_results_20261021.md 4절)
       — 로봇 카메라 높이·각도에서 다시 재서 파라미터만 바꿀 것

  실행: ROS 파이썬엔 YOLO(ultralytics)가 없어 ~/yolo_puck/.venv 파이썬을 쓰되, 시스템 파이썬 경로를 '뒤에' 덧붙임
        (venv의 numpy 2가 앞에 오면 ROS 메시지 모듈이 깨짐 — 뒤에 붙이면 시스템 numpy 1.26 · cv2 4.6이 먼저 잡힘)
        source /opt/ros/<배포판>/setup.bash
        export FASTDDS_DEFAULT_PROFILES_FILE=<저장소>/lv2_module6/ros2_ws/src/ros_system/config/fastdds.xml
        PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages ~/yolo_puck/.venv/bin/python puck_tag_check.py
        (모델 바꾸기) ... puck_tag_check.py --ros-args -p model_path:=<best.pt> -p puck_classes:="['puck_cylinder','puck_box']"
        (카메라 창 안 띄우기 — 화면 없는 로봇) ... puck_tag_check.py --ros-args -p show:=false
"""
import json
import math
import time
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

# 같은 test 폴더의 AprilTag 테스트에서 그대로 가져옴 (태그 종류 · 태그 검출 · 카메라 QoS)
from apriltag_check import FAMILIES, make_detector, sensor_qos

# 팀 YOLO 모델 (ros2_ws/model/yolo/best.pt) — 이 파일 위치 기준: test → perception → src → ros2_ws
TEAM_MODEL = Path(__file__).resolve().parents[3] / "model" / "yolo" / "best.pt"

# 색 기준 H — OpenCV H(색상)는 0~180. 빨강은 0과 180 양끝에 걸침 (자줏빛 빨강은 170 근처라 180 쪽에 붙음)
# 값은 perception_node.py HUES에서 퍽 후보 4색만 — 실물 조명에서 H가 어긋나면 로그의 H 값을 보고 조정
# (어떤 색을 퍽으로 볼지는 파라미터 puck_colors — 대회 색: 빨강·노랑·초록, 파랑은 지금 실물 테스트용)
PUCK_HUES = {"red": (0, 180), "yellow": (30,), "green": (60,), "blue": (110,)}
SAT_MIN, VAL_MIN = 80, 50  # 이보다 채도·밝기가 낮은 화소(바닥·그림자·흰 반사)는 색 판단에서 뺌 (perception_node.py와 같음)


def box_depth_m(depth, box, min_valid=0.5):
    """퍽 박스 가운데 1/3 영역의 depth 중앙값 [m] — 가장자리 배경이 섞이지 않게, 0(측정 실패) 제외, 없으면 0.0

    측정된 화소가 영역의 min_valid보다 적으면 0.0 (= 깊이 없음):
    퍽이 18cm보다 가까우면 퍽 화소는 0이 되고 사이로 보인 배경 몇 화소만 남아, 그 중앙값(1.2m)을 퍽 거리로 쓰던 문제 (실측)
    """
    x0, y0, x1, y1 = box
    bw, bh = x1 - x0, y1 - y0
    patch = depth[y0 + bh // 3:y1 - bh // 3 + 1, x0 + bw // 3:x1 - bw // 3 + 1].astype(np.float32)
    if depth.dtype == np.uint16:
        patch = patch / 1000.0  # mm → m (RealSense 실기는 16UC1 mm)
    valid = patch[np.isfinite(patch) & (patch > 0)]
    if valid.size == 0 or valid.size < min_valid * patch.size:
        return 0.0
    return float(np.median(valid))


def puck_color(bgr, box, hue_tol):
    """박스 안쪽 화소의 대표 색 → (색 이름, H 중앙값)
    색 화소가 너무 적으면 ("unknown", -1) · 화소 다수가 어느 기준 H에서도 hue_tol 넘게 벗어나면 ("other", H) — 주황·보라 등"""
    x0, y0, x1, y1 = box
    mw, mh = int((x1 - x0) * 0.2), int((y1 - y0) * 0.2)  # 가장자리 20%씩 뺌 — 바닥·배경이 섞이지 않게
    roi = bgr[y0 + mh:y1 - mh, x0 + mw:x1 - mw]
    if roi.size == 0:
        return "unknown", -1
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV).reshape(-1, 3)
    h = hsv[(hsv[:, 1] > SAT_MIN) & (hsv[:, 2] > VAL_MIN), 0].astype(int)
    if h.size < 8:
        return "unknown", -1
    # 화소마다 가장 가까운 색(빨강은 원형 거리)에 투표 — 가장 가까운 색도 hue_tol 넘게 멀면 "other"에 투표 → 가장 많은 쪽
    # (예전엔 4색 중 하나로 억지로 맞춰 주황 컵홀더 H12~13이 빨강이 됐음)
    dist = {c: np.min([np.minimum(abs(h - x), 180 - abs(h - x)) for x in hs], axis=0) for c, hs in PUCK_HUES.items()}
    names = list(dist) + ["other"]
    d = np.stack([dist[c] for c in names[:-1]])
    pick = np.where(d.min(axis=0) <= hue_tol, d.argmin(axis=0), len(names) - 1)
    votes = np.bincount(pick, minlength=len(names))
    return names[int(np.argmax(votes))], int(np.median(h))


class PuckTagCheck(Node):
    def __init__(self):
        super().__init__("puck_tag_check")
        cams = self.declare_parameter("cameras", ["pole_front_cam", "gripper_cam"]).value
        model_path = self.declare_parameter("model_path", str(TEAM_MODEL)).value
        # 퍽으로 볼 YOLO 클래스 — 팀 모델은 puck · puck_lying, 내 모델(~/yolo_puck runs/puck_v2)은 puck_cylinder · puck_box
        self.puck_classes = set(self.declare_parameter("puck_classes", ["puck", "puck_lying"]).value)
        self.conf = self.declare_parameter("confidence", 0.4).value  # 검출 신뢰도 하한 (perception.yaml과 같음)
        self.imgsz = self.declare_parameter("imgsz", 640).value      # YOLO 입력 크기 — 노트북 GPU라 640 (팀 도커 CPU는 416)
        infer_hz = self.declare_parameter("infer_hz", 5.0).value     # 카메라당 추론 주기 상한 [Hz] — 30fps를 다 돌리면 GPU·CPU 낭비
        # 퍽 크기 판단 [cm] — 크기는 모델마다 다름 (박스를 얼마나 딱 맞게 그리나) → 쓰는 모델 sdg_v1 실측 기준
        #   sdg_v1: 서 있음 3.1~3.7 × 4.1~6.4 · 누움 6.4~7.0 × 2.9~3.6 · 컵홀더는 YOLO가 아예 안 잡음 · 배경 오검출 17.7cm
        #   (팀 모델은 박스가 넉넉해 서 있음 4.4~5.2 × 4.3~5.9 · 컵홀더 6.9~7.4로 잡음 — 팀 모델이면 기준 다시)
        # 긴 변·짧은 변 하한 — 끝면·윗면(3×3)만 보이면 2.5~3.9 × 2.4~3.9 (실측) → 그보다 조금 아래
        #   빨대는 sdg_v1에서 폭 1.0~1.5라 짧은 변 하한에 걸림
        self.long_min = self.declare_parameter("size_long_min_cm", 2.5).value
        # 긴 변 상한 — 터무니없이 큰 박스(배경 17.7)만 거름. 0.5m 누운 퍽 6.4~7.0인데 0.7m에선 박스가 넉넉해져 8.0~8.5로 나옴(실측)
        # → 딱 맞추면 진짜 퍽을 놓쳐서 넉넉하게. 비슷한 크기의 다른 물체는 YOLO(sdg_v1)와 색(puck_colors)이 거름
        self.long_max = self.declare_parameter("size_long_max_cm", 10.0).value
        self.short_min = self.declare_parameter("size_short_min_cm", 2.2).value
        # 박스가 정사각형에 가깝고(높이÷폭 square_ratio 범위) 작으면(긴 변 < square_long_cm) 끝면·윗면만 보이는 것
        #   → 위에서 본 서 있는 퍽인지, 끝면이 보이는 누운 퍽인지 박스로는 구분 못 함 → 자세 '모름'
        self.square_long = self.declare_parameter("square_long_cm", 4.5).value
        self.square_ratio = self.declare_parameter("square_ratio", [0.8, 1.25]).value
        # 깊이 없음인데 박스 짧은 변이 이 픽셀 이상이면 퍽이 깊이 카메라 최소 거리보다 가까운 것 (3cm 퍽이 70px ≈ 25cm)
        #   → YOLO + 색으로 퍽, 거리 모름 (집기 직전 gripper_cam 거리)
        self.near_px = self.declare_parameter("near_px", 70).value
        # 박스 가운데 영역에서 깊이가 잡힌 화소가 이 비율보다 적으면 깊이 없음으로 봄 (배경 몇 화소로 거리 내지 않게)
        self.min_valid = self.declare_parameter("depth_min_valid", 0.5).value
        self.stand_ratio = self.declare_parameter("stand_ratio", 0.8).value      # 높이÷폭 이 이상이면 서 있음 (sdg_v1 서 있음 1.1~2.0 / 누움 0.43~0.53)
        self.edge_px = self.declare_parameter("edge_px", 5).value                # 박스가 화면 가장자리 이 픽셀 안에 닿으면 '잘림' — 크기·자세 판단 건너뜀
        # 퍽으로 볼 색 — 대회: 빨강·노랑·초록 (아직 확정 아님) · 파랑은 지금 가진 실물 퍽 테스트용
        self.puck_colors = set(self.declare_parameter("puck_colors", ["red", "yellow", "green", "blue"]).value)
        # 기준 H에서 이만큼 안이어야 그 색 — 실측: 빨간 퍽 H1~10 · 파란 퍽 H104~109 · 주황 컵홀더 H12~13(→ other)
        # 빨강과 주황 사이가 2뿐이라 실물 빨간 퍽 조명별로 다시 잴 것
        self.hue_tol = self.declare_parameter("hue_tol", 10).value
        # YOLO 클래스 이름에 색이 있으면(puck_box_blue · puck_red 등) HSV 색과 같아야 퍽 — 색 없는 클래스(puck)면 자동으로 안 봄
        self.check_model_color = self.declare_parameter("check_model_color", True).value
        family = self.declare_parameter("tag_family", "36h11").value
        # 태그로 진짜 퍽 확인 — "진짜 퍽엔 태그가 붙어 있다"를 이용해 손에 든 같은 색 원기둥·컵홀더 같은 가짜를 거름
        #   멀리(태그를 못 읽는 거리)는 퍽 '후보' 그대로 · 가까이서 태그가 읽히면 '확인' (그 물체는 계속 확인 상태)
        #   가까운데 tag_miss번 연속 태그가 없으면 '가짜' → 퍽 아님. 태그 규칙(몇 면·크기)이 미정이라 끌 수 있게
        # 기본 끔 — 지금 실물 퍽엔 태그가 없어 켜면 진짜 퍽도 가까이서 가짜가 됨. 태그 붙인 퍽으로 시험할 땐 -p tag_confirm:=true
        self.tag_confirm = self.declare_parameter("tag_confirm", False).value
        self.tag_check_m = self.declare_parameter("tag_check_m", 0.75).value  # 이 거리 안이면 태그가 보여야 함 [m] (2.4cm 태그 ~0.75m 실측)
        self.tag_miss = self.declare_parameter("tag_miss", 6).value          # 가까운데 연속으로 태그 없음 이 횟수면 가짜 (5Hz → 약 1.2초)
        self.track_px = self.declare_parameter("track_px", 60).value         # 프레임 사이 같은 물체로 볼 화면 거리 [px] (테스트 노드엔 TF가 없어 화면 위치로)
        self.track_forget_s = self.declare_parameter("track_forget_s", 1.5).value  # 이만큼 안 보인 화면 물체는 잊음 [s]
        slop = self.declare_parameter("sync_slop", 0.05).value       # 컬러·depth를 한 쌍으로 묶는 시각 차 한도 [s]
        # 카메라마다 창을 띄워 박스·판단을 그림 (사람이 눈으로 확인용) — 화면 없는 로봇에서는 false
        self.show = self.declare_parameter("show", True).value
        if family not in FAMILIES:
            raise ValueError(f"tag_family는 {list(FAMILIES)} 중 하나 — 받은 값: {family}")

        # YOLO 모델 — 처음 한 번 읽음 (GPU 있으면 GPU)
        import torch  # 무거워서 여기서 import
        from ultralytics import YOLO
        self.device = 0 if torch.cuda.is_available() else "cpu"
        self.model = YOLO(model_path)
        self.names = dict(self.model.names)  # 클래스 번호 → 이름 (모델에 저장된 것)
        missing = self.puck_classes - set(self.names.values())
        if missing:
            self.get_logger().warning(f"모델에 없는 퍽 클래스 {sorted(missing)} — 모델 클래스: {sorted(self.names.values())}")

        self.detect_tags = make_detector(family)
        self.bridge = CvBridge()
        self.period = 1.0 / max(infer_hz, 0.1)
        self.info = {}       # cam → (fx, fy, cx, cy)
        self.last_run = {}   # cam → 마지막 추론 시각 (infer_hz 제한)
        self.last_log = {}   # cam → 마지막 로그 시각 (카메라마다 0.5초에 한 번)
        # cam → 화면 물체 목록 [{u, v, color, confirmed, notag, t}] — 태그 확인을 프레임 사이로 이어 세기 위한 것
        #   (perception_node는 odom 트랙에 이 정보를 붙일 예정 — 여기선 TF가 없어 화면 위치로 이어 붙임)
        self.vtracks = {}
        self.pubs = {cam: self.create_publisher(String, f"/puck_tag_check/{cam}/pucks", 5) for cam in cams}
        self._keep = []      # 구독 객체가 사라지지 않게 보관
        for cam in cams:
            # 컬러·정렬 depth는 시각을 맞춘 쌍으로만 받는다 (apriltag_check.py와 같음)
            color_sub = message_filters.Subscriber(self, Image, f"/{cam}/color/image_raw", qos_profile=sensor_qos())
            depth_sub = message_filters.Subscriber(self, Image, f"/{cam}/aligned_depth_to_color/image_raw",
                                                   qos_profile=sensor_qos())
            sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=5, slop=slop)
            sync.registerCallback(lambda c, d, cam=cam: self.on_frame(cam, c, d))
            info_sub = self.create_subscription(CameraInfo, f"/{cam}/color/camera_info",
                                                lambda m, cam=cam: self.on_info(cam, m), sensor_qos())
            self._keep += [color_sub, depth_sub, sync, info_sub]
        self.get_logger().info(f"모델 {model_path} (device {self.device}) · 퍽 클래스 {sorted(self.puck_classes)} · "
                               f"태그 tag{family} · 카메라 {', '.join(cams)}")

    def on_info(self, cam, msg):
        """내부값 K = [fx 0 cx; 0 fy cy; 0 0 1] 에서 필요한 4개만"""
        k = msg.k
        self.info[cam] = (k[0], k[4], k[2], k[5])

    def judge(self, wcm, hcm, color, clipped=False, model_color="", near=False):
        """크기(폭·높이 cm)·색·잘림·YOLO가 말한 색·가까움 → (퍽인가, 아니면 이유, 자세)
        자세는 퍽일 때만 standing / lying · 잘린 박스나 끝면·윗면만 보이면 '' (모름)
        model_color = YOLO 클래스 이름에 든 색 (sdg_v1 'puck_box_blue' → 'blue') · 색 없는 클래스('puck')면 ''
        near = 박스 짧은 변이 near_px 이상 (화면에서 큼 = 아주 가까움)"""
        # 색 검사를 먼저 — 깊이가 없어도 색은 볼 수 있음
        if color == "unknown":
            return False, "색 없음", ""     # 흰·회색 물체 (퍽은 색이 있음)
        if color not in self.puck_colors:
            return False, "색", ""          # 퍽 색 목록 밖 (other = 주황·보라 등, 또는 목록에서 뺀 색)
        if self.check_model_color and model_color and model_color != color:
            # YOLO는 파란 퍽이라는데 HSV는 빨강 — 진짜 퍽이면 둘이 같아야 함 (실측: 주황 컵홀더가 puck_cylinder_blue · red(H9)로 통과했음)
            return False, "색 불일치", ""
        if wcm <= 0 or hcm <= 0:
            if clipped or near:
                # RealSense D435는 약 16~18cm보다 가까우면 깊이를 못 잼 (실측) — 그 거리면 퍽이 화면에서 잘리거나 크게 보임
                # → 집기 직전이라 놓치면 안 됨: YOLO + 색으로 퍽, 거리·자세는 모름
                return True, "", ""
            return False, "depth 없음", ""  # 작게 보이는데 깊이 없음 = 반사 등 다른 이유 → 위치를 몰라 퍽으로 안 냄
        long_cm, short_cm = max(wcm, hcm), min(wcm, hcm)
        if clipped:
            # 박스가 화면 가장자리에 닿음 = 퍽 일부가 화면 밖 → 박스가 실제보다 '작게'만 나옴
            # (실측: 0.18m에서 퍽 아래가 잘려 3.3×3.6cm → 크기 하한에 걸렸음 — gripper_cam 집기 직전 거리)
            # → 작은 쪽(하한)·자세는 안 봄. 그래도 상한보다 크면 거리가 틀린 것 (실측: 배경을 재 22.2×20.4cm · 1.13m)
            #   → 틀린 위치를 내느니 이 장면은 버림 (다음 장면에서 거리가 맞으면 다시 잡힘)
            if long_cm > self.long_max:
                return False, "거리 이상", ""
            return True, "", ""
        if not (self.long_min <= long_cm <= self.long_max) or short_cm < self.short_min:
            return False, "크기", ""
        ratio = hcm / wcm
        if long_cm < self.square_long and self.square_ratio[0] <= ratio <= self.square_ratio[1]:
            return True, "", ""  # 끝면·윗면(3×3)만 보임 — 서 있는지 누웠는지 박스로는 모름
        return True, "", "standing" if ratio >= self.stand_ratio else "lying"

    def on_frame(self, cam, color, depth):
        """동기화된 컬러·depth 한 쌍 → 퍽 검출 · 거리 · 색 · 태그 번호 → 발행"""
        now = time.monotonic()
        if now - self.last_run.get(cam, 0.0) < self.period:
            return  # infer_hz 주기보다 빨리 온 프레임은 건너뜀
        if cam not in self.info:
            self.get_logger().warning(f"{cam} camera_info 아직 없음", throttle_duration_sec=5.0)
            return
        self.last_run[cam] = now
        try:
            bgr = self.bridge.imgmsg_to_cv2(color, desired_encoding="bgr8")
            d = self.bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"{cam} 변환 실패: {e}", throttle_duration_sec=5.0)
            return
        fx, fy, cx, cy = self.info[cam]
        h, w = bgr.shape[:2]

        # ① YOLO — 퍽 클래스만
        res = self.model.predict(bgr, imgsz=self.imgsz, conf=self.conf, device=self.device, verbose=False)[0]
        pucks = []
        for (x0, y0, x1, y1), c, s in zip(res.boxes.xyxy.int().tolist(), res.boxes.cls.tolist(), res.boxes.conf.tolist()):
            name = self.names.get(int(c), str(int(c)))
            if name not in self.puck_classes:
                continue
            box = (max(0, x0), max(0, y0), min(w, x1), min(h, y1))
            # ② 거리 — 박스 가운데 depth → 박스 중심 3D
            z = box_depth_m(d, box, self.min_valid)
            u, v = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
            x, y = ((u - cx) * z / fx, (v - cy) * z / fy) if z > 0 else (0.0, 0.0)
            # ③ 색
            col, hue = puck_color(bgr, box, self.hue_tol)
            # 실제 크기 — 박스 픽셀 × 거리 ÷ 초점거리 (핀홀 모델). 퍽 3×3×6cm: 서 있으면 폭 3 × 높이 6, 누우면 대략 6 × 3
            # (판단 기준은 실물 측정값을 보고 정할 예정 — 지금은 값만 냄)
            wcm = round((box[2] - box[0]) * z / fx * 100, 1) if z > 0 else -1.0
            hcm = round((box[3] - box[1]) * z / fy * 100, 1) if z > 0 else -1.0
            # ⑤ 퍽 판단 · 자세 — 크기·색으로 (YOLO 클래스 이름의 모양·자세는 안 믿음)
            # 화면 가장자리 edge_px 안에 박스 변이 닿으면 잘린 퍽
            m = self.edge_px
            clipped = box[0] <= m or box[1] <= m or box[2] >= w - m or box[3] >= h - m
            # YOLO 클래스 이름에 든 색 — '_'로 나눈 조각 중 색 이름 (없으면 '')
            model_color = next((t for t in name.split("_") if t in PUCK_HUES), "")
            near = min(box[2] - box[0], box[3] - box[1]) >= self.near_px
            is_puck, reason, posture = self.judge(wcm, hcm, col, clipped, model_color, near)
            pucks.append({"is_puck": is_puck, "reason": reason, "posture": posture, "clipped": clipped,
                          "class": name, "score": round(s, 2), "color": col, "hue": hue,
                          "width_cm": wcm, "height_cm": hcm,
                          "distance": round(math.sqrt(x * x + y * y + z * z), 3) if z > 0 else -1.0,
                          "x": round(x, 3), "y": round(y, 3), "z": round(z, 3), "tag": -1, "box": box})

        # ④ AprilTag — 태그 중심이 들어간 퍽 박스 중 가장 작은 것에 번호를 붙임
        corners, ids = self.detect_tags(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
        loose = []  # 퍽 박스 밖의 태그 (로그로만 — YOLO가 퍽을 놓쳤거나 태그만 따로 있는 경우)
        if ids is not None:
            for cn, tag_id in zip(corners, ids.flatten()):
                tu, tv = cn.reshape(4, 2).mean(axis=0)
                inside = [p for p in pucks if p["box"][0] <= tu <= p["box"][2] and p["box"][1] <= tv <= p["box"][3]]
                if inside:
                    min(inside, key=lambda p: (p["box"][2] - p["box"][0]) * (p["box"][3] - p["box"][1]))["tag"] = int(tag_id)
                else:
                    loose.append(int(tag_id))

        # ⑥ 태그로 진짜 퍽 확인 (tag_confirm) — 위 ⑤에서 퍽으로 본 것만
        if self.tag_confirm:
            self.confirm_by_tag(cam, now, pucks)
        else:
            for p in pucks:
                p["tag_state"] = ""

        if self.show:
            self.draw(cam, bgr, pucks, corners, ids)
        for p in pucks:
            del p["box"]  # 박스 픽셀은 발행하지 않음 (내부 계산용)
        self.pubs[cam].publish(String(data=json.dumps(pucks)))

        # 로그 — 카메라마다 0.5초에 한 번 (퍽이나 태그가 보일 때만)
        if (pucks or loose) and now - self.last_log.get(cam, 0.0) >= 0.5:
            self.last_log[cam] = now
            for p in pucks:
                dist = f"{p['distance']:.3f} m (x {p['x']:+.3f}, y {p['y']:+.3f}, z {p['z']:.3f})" \
                    if p["distance"] > 0 else "거리 모름(depth 없음)"
                tag = f"tag {p['tag']}" if p["tag"] >= 0 else "tag 없음"
                size = f"{p['width_cm']:.1f}x{p['height_cm']:.1f}cm" if p["width_cm"] > 0 else "크기 ?"
                # 자세 모름의 이유: 화면에 잘림 / 깊이 못 잴 만큼 가까움 / 끝면·윗면(3×3)만 보임
                why_pose = "잘림" if p["clipped"] else "가까움" if p["distance"] <= 0 else "끝면"
                pose_txt = {"standing": "서있음", "lying": "누움"}.get(p["posture"], f"({why_pose}·자세?)")
                tag_txt = {"confirmed": "·태그확인", "candidate": "·후보"}.get(p.get("tag_state", ""), "")
                verdict = f"✅퍽{tag_txt} {pose_txt}" if p["is_puck"] \
                    else f"❌퍽아님({p['reason']})"
                self.get_logger().info(f"{cam} {verdict} {p['color']}(H{p['hue']}) {size}: {dist} "
                                       f"[{p['class']} {p['score']:.2f}] {tag}")
            if loose:
                self.get_logger().info(f"{cam} 퍽 박스 밖 태그: {loose}")

    def confirm_by_tag(self, cam, now, pucks):
        """퍽 검출마다 tag_state = confirmed(태그 확인) / candidate(아직 모름) / fake(가까운데 계속 태그 없음 → 퍽 아님)
        같은 물체 판단: 화면 중심이 track_px 안 · 색이 같은 화면 물체 중 가장 가까운 것 (테스트 노드엔 TF가 없어 화면 위치로)"""
        tracks = [t for t in self.vtracks.get(cam, []) if now - t["t"] < self.track_forget_s]  # 오래 안 보인 건 잊음
        used = set()
        for p in pucks:
            p["tag_state"] = ""
            if not p["is_puck"]:
                continue  # 크기·색에서 이미 퍽 아님
            x0, y0, x1, y1 = p["box"]
            u, v = (x0 + x1) / 2, (y0 + y1) / 2
            cand = [(math.hypot(t["u"] - u, t["v"] - v), i) for i, t in enumerate(tracks)
                    if i not in used and t["color"] == p["color"]]
            cand = [c for c in cand if c[0] <= self.track_px]
            if cand:
                i = min(cand)[1]
                tr = tracks[i]
            else:
                tr = {"confirmed": False, "notag": 0}
                tracks.append(tr)
                i = len(tracks) - 1
            used.add(i)
            tr.update(u=u, v=v, color=p["color"], t=now)
            # 태그가 보여야 하는 가까운 거리인가 — 거리를 재면 tag_check_m 안 · 못 재면(너무 가까움) 화면에서 크면(near_px)
            close = (0 < p["distance"] <= self.tag_check_m) or \
                (p["distance"] <= 0 and min(x1 - x0, y1 - y0) >= self.near_px)
            if p["tag"] >= 0:
                tr["confirmed"], tr["notag"] = True, 0   # 태그 읽힘 → 진짜 퍽 (이후 태그가 안 보여도 확인 유지)
            elif close and not tr["confirmed"]:
                tr["notag"] += 1                         # 가까운데 태그 없음 — 연속으로 셈
            elif not close:
                tr["notag"] = 0                          # 멀어지면 다시 처음부터 (멀면 원래 못 읽음)
            if tr["confirmed"]:
                p["tag_state"] = "confirmed"
            elif tr["notag"] >= self.tag_miss:
                p["tag_state"] = "fake"
                p["is_puck"], p["reason"], p["posture"] = False, "태그 없음", ""  # 손에 든 같은 색 원기둥·컵홀더 등
            else:
                p["tag_state"] = "candidate"
        self.vtracks[cam] = tracks

    def draw(self, cam, bgr, pucks, corners, ids):
        """창(이름 = 카메라)에 판단 결과를 그림 — 퍽 초록 · 퍽 아님 빨강 · 태그 노랑 (OpenCV 글씨는 한글이 안 돼 영어)
        추론 주기(infer_hz)마다 갱신되므로 창도 초당 그만큼만 바뀜"""
        img = bgr.copy()
        why = {"크기": "size", "색": "color", "색 없음": "no color", "depth 없음": "no depth",
               "색 불일치": "color mismatch", "거리 이상": "bad depth", "태그 없음": "no tag"}
        for p in pucks:
            x0, y0, x1, y1 = p["box"]
            col = (0, 200, 0) if p["is_puck"] else (0, 0, 255)  # BGR — 초록 / 빨강
            if p["is_puck"]:
                why_pose = "clip" if p["clipped"] else "near" if p["distance"] <= 0 else "end"  # 자세 모름의 이유
                head = "PUCK " + {"standing": "stand", "lying": "lying"}.get(p["posture"], f"{why_pose} ?")
            else:
                head = "NOT " + why.get(p["reason"], p["reason"])
            if p["tag"] >= 0:
                head += f" tag{p['tag']}"
            if p.get("tag_state") == "confirmed":
                head += " TAG OK"  # 가까이서 태그를 읽어 진짜 퍽으로 확인됨 (이후 계속 유지)
            info = f"{p['color']} {p['width_cm']:.1f}x{p['height_cm']:.1f}cm {p['distance']:.2f}m" \
                if p["distance"] > 0 else f"{p['color']} no depth"
            cv2.rectangle(img, (x0, y0), (x1, y1), col, 2)
            cv2.putText(img, head, (x0, max(15, y0 - 22)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
            cv2.putText(img, info, (x0, max(30, y0 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
        if ids is not None:
            for cn, tag_id in zip(corners, ids.flatten()):
                pts = cn.reshape(4, 2).astype(int)
                cv2.polylines(img, [pts], True, (0, 255, 255), 2)
                cv2.putText(img, f"id{tag_id}", tuple(pts[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        cv2.imshow(cam, img)
        cv2.waitKey(1)  # 창 갱신 (이게 없으면 창이 안 그려짐)


def main():
    rclpy.init()
    node = PuckTagCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C · 외부 종료 때 에러 출력 없이 끝냄
        pass
    except RuntimeError:
        # 영상 메시지를 받는 도중 Ctrl+C가 오면 rclpy가 RuntimeError(Unable to convert call argument)를 냄 (실측)
        # — 이미 종료 중(rclpy.ok() False)일 때만 삼키고, 그 밖의 진짜 에러는 그대로 보여 줌
        if rclpy.ok():
            raise
    finally:
        cv2.destroyAllWindows()  # show 창 닫기
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
