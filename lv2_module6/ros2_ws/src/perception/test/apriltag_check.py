#!/usr/bin/env python3
"""AprilTag 거리 확인용 테스트 노드 — RealSense 2대(pole_front_cam · gripper_cam)로 태그를 찾아 카메라~태그 거리를 발행

  입력: /<cam>/color/image_raw · /<cam>/aligned_depth_to_color/image_raw · /<cam>/color/camera_info (best_effort)
        (cam = pole_front_cam, gripper_cam — 팀 develop 4cbe321과 같은 이름 · perception_node.py와 같은 토픽)
  출력: /apriltag_check/<cam>/tag_<번호>/distance (std_msgs/Float32, m) — 태그 번호마다 처음 보일 때 토픽이 생김
        로그: "pole_front_cam tag 3: 0.523 m (x +0.012, y -0.034, z 0.521)" (카메라 기준 · x 오른쪽 · y 아래 · z 앞)

  방법: 컬러·정렬 depth를 시각 맞춘 쌍으로 받아(ApproximateTime)
        ① 컬러에서 OpenCV aruco로 AprilTag 검출(네 꼭짓점 + 번호)
        ② 태그 가운데 작은 영역의 depth 중앙값 = 태그까지 앞 방향 거리 z (태그 크기 몰라도 됨)
        ③ camera_info 내부값으로 가운데 픽셀을 3D(x, y, z)로 역투영 → 직선거리 √(x²+y²+z²) 발행

  ※ 팀 자동 검사(CMakeLists.txt의 ament_add_pytest_test)는 test/test_node.py만 돌림 — 이 파일은 사람이 직접 실행해 확인하는 용도
  실행: source /opt/ros/<배포판>/setup.bash
        export FASTDDS_DEFAULT_PROFILES_FILE=<저장소>/lv2_module6/ros2_ws/src/ros_system/config/fastdds.xml
          ↑ 꼭 필요 — 없으면 영상(약 900KB)이 Fast DDS 기본 공유메모리 칸(512KB)을 못 타서 한 장도 안 들어옴 (실측)
            카메라 노드 터미널에도 똑같이 지정
        python3 apriltag_check.py                                   (기본: tag36h11, 카메라 2대)
        python3 apriltag_check.py --ros-args -p tag_family:=25h9     (태그 종류 바꾸기)
        python3 apriltag_check.py --ros-args -p show:=false          (카메라 창 안 띄우기 — 화면 없는 로봇)
"""
import math
import time

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32

# 태그 종류 이름 → OpenCV 사전 (OpenCV 4.6에 들어 있는 AprilTag 종류)
FAMILIES = {
    "16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


def sensor_qos():
    """카메라 토픽용 QoS — RealSense 드라이버가 best_effort로 내므로 받는 쪽도 맞춤 (perception_node.py와 동일)"""
    return QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=5)


def make_detector(family):
    """AprilTag 검출 함수를 만든다 — OpenCV 4.7부터 API가 바뀌어(ArucoDetector) 두 버전 모두 지원"""
    dictionary = cv2.aruco.getPredefinedDictionary(FAMILIES[family])
    if hasattr(cv2.aruco, "ArucoDetector"):  # OpenCV 4.7 이상
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG  # 꼭짓점을 AprilTag 방식으로 다듬음
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        return lambda gray: detector.detectMarkers(gray)[:2]
    params = cv2.aruco.DetectorParameters_create()  # OpenCV 4.6 (노트북 · ROS 기본)
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    return lambda gray: cv2.aruco.detectMarkers(gray, dictionary, parameters=params)[:2]


def center_depth_m(depth, corners, min_valid=0.5):
    """태그 가운데 영역의 depth 중앙값 [m] — 0(측정 실패)은 빼고, 없으면 0.0

    영역 크기 = 태그 한 변의 1/4 (최소 3px): 태그 가장자리 밖 배경 depth가 섞이지 않게 안쪽만 본다.
    depth는 16UC1(mm, RealSense 실기)과 32FC1(m, 일부 시뮬레이터) 둘 다 받는다.
    측정된 화소가 영역의 min_valid보다 적으면 0.0 (= 깊이 없음):
    태그가 17cm보다 가까우면 태그 화소는 0이 되고 배경 몇 화소만 남아 1.03 · 1.43m로 튀던 문제 (실측)
    """
    u, v = corners.mean(axis=0)
    side = np.linalg.norm(corners[0] - corners[1])  # 위쪽 변 길이 [px]
    r = max(1, int(side / 8))                       # 반지름 = 한 변의 1/8 → 영역 한 변 = 1/4
    h, w = depth.shape[:2]
    x1, x2 = max(0, int(u) - r), min(w, int(u) + r + 1)
    y1, y2 = max(0, int(v) - r), min(h, int(v) + r + 1)
    patch = depth[y1:y2, x1:x2].astype(np.float32)
    if depth.dtype == np.uint16:
        patch = patch / 1000.0  # mm → m
    valid = patch[np.isfinite(patch) & (patch > 0)]
    if valid.size == 0 or valid.size < min_valid * patch.size:
        return 0.0
    return float(np.median(valid))


class AprilTagCheck(Node):
    def __init__(self):
        super().__init__("apriltag_check")
        cams = self.declare_parameter("cameras", ["pole_front_cam", "gripper_cam"]).value
        family = self.declare_parameter("tag_family", "36h11").value
        slop = self.declare_parameter("sync_slop", 0.05).value  # 컬러·depth를 한 쌍으로 묶는 시각 차 한도 [s]
        # 태그 가운데에서 깊이가 잡힌 화소가 이 비율보다 적으면 깊이 없음 (배경 몇 화소로 엉뚱한 거리 안 냄)
        self.min_valid = self.declare_parameter("depth_min_valid", 0.5).value
        # 카메라마다 창을 띄워 태그 테두리·번호·거리를 그림 (사람이 눈으로 확인용) — 화면 없는 로봇에서는 false
        self.show = self.declare_parameter("show", True).value
        if family not in FAMILIES:
            raise ValueError(f"tag_family는 {list(FAMILIES)} 중 하나 — 받은 값: {family}")
        self.detect = make_detector(family)
        self.bridge = CvBridge()
        self.info = {}  # cam → (fx, fy, cx, cy) — camera_info 내부값
        self.pubs = {}  # (cam, 태그 번호) → 거리 발행기 (처음 보인 태그만 만듦)
        # (cam, 태그 번호) → 마지막 로그 시각 — ROS throttle은 '코드 한 줄' 단위라 카메라 2대가 한 자리를 나눠 써서
        # 30Hz인 pole_front_cam 로그가 15Hz인 gripper_cam 로그를 덮었음 → 카메라·태그마다 따로 0.5초에 한 줄
        self.last_log = {}
        self._keep = []  # 구독 객체가 사라지지 않게 보관
        for cam in cams:
            # 컬러·정렬 depth는 시각을 맞춘 쌍으로만 받는다 (다른 순간의 컬러·depth를 섞지 않게)
            color_sub = message_filters.Subscriber(self, Image, f"/{cam}/color/image_raw", qos_profile=sensor_qos())
            depth_sub = message_filters.Subscriber(self, Image, f"/{cam}/aligned_depth_to_color/image_raw",
                                                   qos_profile=sensor_qos())
            sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=5, slop=slop)
            sync.registerCallback(lambda c, d, cam=cam: self.on_frame(cam, c, d))
            # camera_info는 고정값이라 따로 받아 둔다
            info_sub = self.create_subscription(CameraInfo, f"/{cam}/color/camera_info",
                                                lambda m, cam=cam: self.on_info(cam, m), sensor_qos())
            self._keep += [color_sub, depth_sub, sync, info_sub]
        self.get_logger().info(f"AprilTag tag{family} 대기 중 — 카메라: {', '.join(cams)}")

    def on_info(self, cam, msg):
        """내부값 K = [fx 0 cx; 0 fy cy; 0 0 1] 에서 필요한 4개만"""
        k = msg.k
        self.info[cam] = (k[0], k[4], k[2], k[5])

    def on_frame(self, cam, color, depth):
        """동기화된 컬러·depth 한 쌍 → 태그 검출 → 태그마다 거리 발행"""
        if cam not in self.info:
            self.get_logger().warning(f"{cam} camera_info 아직 없음", throttle_duration_sec=5.0)
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(color, desired_encoding="bgr8")  # 창에 그릴 컬러
            d = self.bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"{cam} 변환 실패: {e}", throttle_duration_sec=5.0)
            return
        img = bgr.copy() if self.show else None
        corners, ids = self.detect(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))  # 검출은 흑백으로 충분
        if ids is None:
            if self.show:
                cv2.imshow(cam, img)  # 태그가 없어도 영상은 보여 줌
                cv2.waitKey(1)
            return
        fx, fy, cx, cy = self.info[cam]
        for c, tag_id in zip(corners, ids.flatten()):
            c = c.reshape(4, 2)  # 네 꼭짓점 (u, v) [px]
            z = center_depth_m(d, c, self.min_valid)
            if self.show:  # 태그 테두리(노랑) + 번호 · 거리 (OpenCV 글씨는 한글이 안 돼 영어)
                pts = c.astype(int)
                cv2.polylines(img, [pts], True, (0, 255, 255), 2)
                cv2.putText(img, f"id{tag_id} " + (f"{z:.2f}m(z)" if z > 0 else "no depth"), tuple(pts[0]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            if z <= 0.0:
                self.get_logger().warning(f"{cam} tag {tag_id}: depth 없음 (너무 가깝거나 반사)",
                                          throttle_duration_sec=2.0)
                continue
            # 가운데 픽셀(u, v) + 거리 z → 카메라 기준 3D (핀홀 모델 역투영)
            u, v = c.mean(axis=0)
            x, y = (u - cx) * z / fx, (v - cy) * z / fy
            dist = math.sqrt(x * x + y * y + z * z)
            key = (cam, int(tag_id))
            if key not in self.pubs:
                topic = f"/apriltag_check/{cam}/tag_{tag_id}/distance"
                self.pubs[key] = self.create_publisher(Float32, topic, 5)
                # 처음 본 태그는 바로 알림 — 토픽은 한 번 생기면 안 지워지므로 언제 무엇을 봤는지 남겨 둠
                self.get_logger().info(f"새 태그 발견: {cam} tag {tag_id} → {topic}")
            self.pubs[key].publish(Float32(data=dist))
            now = time.monotonic()
            if now - self.last_log.get(key, 0.0) >= 0.5:  # 카메라·태그마다 0.5초에 한 줄
                self.last_log[key] = now
                self.get_logger().info(f"{cam} tag {tag_id}: {dist:.3f} m (x {x:+.3f}, y {y:+.3f}, z {z:.3f})")
        if self.show:
            cv2.imshow(cam, img)  # 창 이름 = 카메라
            cv2.waitKey(1)        # 창 갱신 (이게 없으면 창이 안 그려짐)


def main():
    rclpy.init()
    node = AprilTagCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C · 외부 종료(SIGTERM) 때 에러 출력 없이 끝냄
        pass
    finally:
        cv2.destroyAllWindows()  # show 창 닫기
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
