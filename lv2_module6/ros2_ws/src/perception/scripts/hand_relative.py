#!/usr/bin/env python3
"""🛠 hand_relative — 손 카메라 상대값 (학습 보정 정책 입력) · 계산은 hand_geom.py

  입력: /hand_relative/target (perception/HandTarget, 팔 skill 노드(skills ArmIO)가 닿는 구간 시작 전에 보냄 · reliable + transient_local)
        /perception/gripper_cam/detections (perception/TrackedObjectArray, map 기준 · 영상 시각)
        TF map → base_link, base_link → end_effector_link (영상 시각, 없으면 최신)
  출력: /hand_relative (perception/HandRelative, base_link 기준) — rl_arm_correction이 정책 관측으로 씀
  왜: 시뮬 학습 정책(isaac/rl/bee_isaac/tasks)은 "손 카메라로 잰 퍽 − 손 / 칸 − 손 안 퍽"을 보고 보정함 (2026-11-03 배포 결정:
      닿는 구간은 시뮬과 같은 기준 동작 + 학습 보정 → 그 입력을 로봇에서 만들어야 함).
  주기: cell·puck은 손 카메라 검출이 올 때마다 (≤ 4Hz), slot은 rate_hz (카메라 없이 TF만 — 적재함은 로봇에 고정).
"""
import math

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
from perception.msg import HandRelative, HandTarget, TrackedObjectArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

import hand_geom as hg

# 적재함 칸 가운데 (base_link, 칸 바닥 높이) — isaac/scripts/tools/slot_table.py SLOTS와 같음 (URDF basket_slot_0~11)
SLOTS = [(x, y, 0.09125) for x in (-0.16984, -0.10484, -0.03984) for y in (-0.0975, -0.0325, 0.0325, 0.0975)]


def detail(o, key):
    for kv in o.details:
        if kv.key == key:
            return kv.value
    return ""


def yaw_of_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class HandRelativeNode(Node):
    def __init__(self):
        super().__init__("hand_relative")
        self.declare_parameter("base_frame", "base_link")  # 시뮬 정책 입력 좌표 (isaac/rl/bee_isaac/tasks: base_link)
        self.declare_parameter("ee_frame", "end_effector_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("rate_hz", 20.0)  # slot (카메라 없음) 발행 주기
        self.declare_parameter("max_age_s", 1.0)  # 이보다 오래된 검출은 안 씀
        g = lambda k: self.get_parameter(k).value
        self.base, self.ee, self.map = g("base_frame"), g("ee_frame"), g("map_frame")
        self.max_age = float(g("max_age_s"))
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.target = HandTarget(kind="none")
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(HandTarget, "/hand_relative/target", self.on_target, latched)
        self.create_subscription(TrackedObjectArray, "/perception/gripper_cam/detections", self.on_dets, 5)
        self.pub = self.create_publisher(HandRelative, "/hand_relative", 10)
        self.create_timer(1.0 / float(g("rate_hz")), self.on_timer)
        self.sent, self.last_valid = 0, None  # 상태용: 보낸 수 · 마지막 valid 시각
        self.diag = Updater(self)  # /diagnostics "hand_relative: 상태" (1초마다) → aggregator 인지
        self.diag.setHardwareID("none")
        self.diag.add("상태", self.on_diag)

    def on_diag(self, s):
        kind = self.target.kind
        age = None if self.last_valid is None else (self.get_clock().now() - self.last_valid).nanoseconds * 1e-9
        if kind == "none":
            s.summary(DiagnosticStatus.OK, "대기 (target 없음)")
        elif age is None or age > self.max_age:
            s.summary(DiagnosticStatus.WARN, f"{kind}: 유효한 상대값 없음 (검출·TF 확인)")
        else:
            s.summary(DiagnosticStatus.OK, f"{kind} 발행 중")
        s.add("target", kind)
        s.add("보낸 수", str(self.sent))
        s.add("마지막 유효 (s 전)", "-" if age is None else f"{age:.2f}")
        return s

    def publish(self, m):
        self.pub.publish(m)
        self.sent += 1
        if m.valid:
            self.last_valid = self.get_clock().now()

    def on_target(self, msg):
        self.target = msg
        self.get_logger().info(f"target {msg.kind} puck_tag {msg.puck_tag} box_tag {msg.box_tag} slot {msg.slot}")

    def frames(self, stamp):
        """(base_t, base_R) map에서 본 base_link · (ee_t, ee_R) base_link에서 본 손끝 — 영상 시각, 안 되면 최신."""
        when = ([(Time.from_msg(stamp), Duration(seconds=0.05))] if stamp is not None else []) + [(Time(), Duration(seconds=0.0))]
        out = []
        for parent, child in ((self.map, self.base), (self.base, self.ee)):
            tr = None
            for at, wait in when:
                try:
                    tr = self.tf.lookup_transform(parent, child, at, wait)
                    break
                except Exception:  # noqa: BLE001 — TF가 아직 없음 / 그 시각 없음 → 최신으로
                    continue
            if tr is None:
                return None
            p, q = tr.transform.translation, tr.transform.rotation
            out.append((np.array([p.x, p.y, p.z]), hg.quat_to_rot(q.x, q.y, q.z, q.w)))
        return out

    def base_msg(self, stamp, kind):
        m = HandRelative()
        m.header.frame_id, m.kind, m.valid = self.base, kind, False
        if stamp is not None:
            m.header.stamp = stamp
        m.axis.z = 1.0
        return m

    def on_timer(self):
        t = self.target
        if t.kind != "slot":
            return
        fr = self.frames(None)
        m = self.base_msg(self.get_clock().now().to_msg(), "slot")
        if fr is not None and 0 <= t.slot < len(SLOTS):
            (_, _), (ee_t, ee_R) = fr
            rel = np.array(SLOTS[t.slot]) - hg.bottom_in_hand(ee_t, ee_R)
            m.rel.x, m.rel.y, m.rel.z = (float(v) for v in rel)
            m.valid = True
        self.publish(m)

    def on_dets(self, msg):
        t = self.target
        if t.kind not in ("puck", "cell"):
            return
        m = self.base_msg(msg.header.stamp, t.kind)
        fr = self.frames(msg.header.stamp)
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        m.age = float(max(age, 0.0))
        if fr is None or age > self.max_age:
            self.publish(m)
            return
        (base_t, base_R), (ee_t, ee_R) = fr
        base_yaw = math.atan2(base_R[1, 0], base_R[0, 0])
        if t.kind == "cell":
            box = None
            for o in msg.objects:
                if o.class_name == "location_box" and detail(o, "tag") == str(t.box_tag):
                    p = o.pose.pose.position
                    box = (p.x, p.y, p.z)
            pt = (t.point.x, t.point.y, t.point.z)
            plan = (t.box_point.x, t.box_point.y, t.box_point.z) if t.box_tag >= 0 else None
            cell_b = hg.to_base(hg.cell_now(pt, plan, box), base_t, base_R)
            rel = cell_b - hg.bottom_in_hand(ee_t, ee_R)
            m.rel.x, m.rel.y, m.rel.z = (float(v) for v in rel)
            m.valid = True  # 상자를 못 봐도 계획 칸으로 (손 위치는 TF로 앎)
            self.publish(m)
            return
        pucks = []  # (tag, centre_b, standing, yaw_b, is_box)
        for o in msg.objects:
            if o.class_name not in ("puck", "puck_lying"):
                continue
            standing = o.class_name == "puck"
            p = o.pose.pose.position
            c = hg.to_base(hg.puck_centre((p.x, p.y, p.z), standing), base_t, base_R)
            known_yaw = bool(detail(o, "yaw_period"))
            yaw_b = hg.wrap(yaw_of_quat(o.pose.pose.orientation) - base_yaw, 2 * math.pi) if known_yaw else None
            tag = int(detail(o, "tag")) if detail(o, "tag").lstrip("-").isdigit() else -1
            pucks.append((tag, c, standing, yaw_b, detail(o, "shape") == "box"))
        k = hg.pick_target(pucks, ee_t, t.puck_tag)
        if k is not None:
            tag, c, standing, yaw_b, is_box = pucks[k]
            rel = c - ee_t
            m.rel.x, m.rel.y, m.rel.z = (float(v) for v in rel)
            m.is_box = is_box
            m.face_yaw = float(hg.face_yaw(yaw_b, hg.tool_yaw(ee_R), is_box))
            ax = hg.puck_axis(standing, yaw_b, c)
            m.axis.x, m.axis.y, m.axis.z = (float(v) for v in ax)
            nb = hg.neighbour(pucks, k)
            if nb is not None:
                m.neighbour = True
                m.neighbour_rel.x, m.neighbour_rel.y, m.neighbour_rel.z = (float(v) for v in nb)
            m.valid = True
        self.publish(m)


def main():
    rclpy.init()
    node = HandRelativeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
