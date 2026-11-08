#!/usr/bin/env python3
"""🛠 hand_relative — 손 카메라 상대값 (학습 보정 정책 입력) · 계산은 hand_geom.py

  입력: /hand_relative/target (perception/HandTarget, 팔 skill 노드(skills ArmIO)가 닿는 구간 시작 전에 보냄 · reliable + transient_local)
        /perception/gripper_cam/detections (perception/TrackedObjectArray, map 기준 · 영상 시각)
        TF map → base_link, base_link → end_effector_link (영상 시각, 없으면 최신)
  출력: /hand_relative (perception/HandRelative, base_link 기준) — rl_arm_correction이 정책 관측으로 씀
  왜: 시뮬 학습 정책(isaac/rl/bee_isaac/tasks)은 "손 카메라로 잰 퍽 − 손 / 칸 − 손 안 퍽"을 보고 보정함 (2026-11-03 배포 결정:
      닿는 구간은 시뮬과 같은 기준 동작 + 학습 보정 → 그 입력을 로봇에서 만들어야 함).
  주기: 모두 rate_hz. slot·slot_puck은 카메라 없이 TF만 (적재함은 로봇에 고정 · 적재함 안 퍽은 perception이 자기 몸으로 지움). cell·puck은 마지막 검출(≤ 4Hz)을 map 기준으로 들고,
        발행 때마다 지금 손 위치로 다시 뺌 (hand_geom.held_relative — 시뮬 pick_env와 같은 정의, 11/07) · stamp = 그 검출의 영상 시각.
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
        self.declare_parameter("rate_hz", 20.0)  # 발행 주기 (slot · 들고 있는 cell·puck 검출)
        self.declare_parameter("max_age_s", 0.45)  # 이보다 오래된 검출은 안 씀 — 간격 0.25 + 지연 0.15 + 0.05 (rl_arm_correction max_cam_age_s, 시뮬 meas_age_max)
        g = lambda k: self.get_parameter(k).value
        self.base, self.ee, self.map = g("base_frame"), g("ee_frame"), g("map_frame")
        self.max_age = float(g("max_age_s"))
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.target = HandTarget(kind="none")
        self.held = None  # (stamp, hand_geom.held_relative 입력, neighbour_rel 또는 None) — 마지막 cell·puck 검출
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
        self.held = None
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
        if t.kind in ("puck", "cell"):
            self.publish_held()
            return
        if t.kind not in ("slot", "slot_puck"):
            return
        fr = self.frames(None)
        m = self.base_msg(self.get_clock().now().to_msg(), t.kind)
        if fr is not None and 0 <= t.slot < len(SLOTS):
            (_, _), (ee_t, ee_R) = fr
            rel = (np.array(SLOTS[t.slot]) - hg.bottom_in_hand(ee_t, ee_R) if t.kind == "slot"
                   else hg.slot_puck_rel(SLOTS[t.slot], ee_t))
            m.rel.x, m.rel.y, m.rel.z = (float(v) for v in rel)
            m.valid = True
        self.publish(m)

    def publish_held(self):
        """마지막 검출을 지금 손 기준으로 · 없거나 max_age_s보다 오래되면 보내지 않음 (on_dets가 invalid를 이미 보냄)."""
        if self.held is None:
            return
        stamp, h, nb = self.held
        age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds * 1e-9
        fr = self.frames(None)
        m = self.base_msg(stamp, h["kind"])
        m.age = float(max(age, 0.0))
        if fr is None or age > self.max_age:
            self.held = None
            self.publish(m)
            return
        (base_t, base_R), (ee_t, ee_R) = fr
        rel, fy, ax = hg.held_relative(h, base_t, base_R, ee_t, ee_R)
        m.rel.x, m.rel.y, m.rel.z = (float(v) for v in rel)
        m.face_yaw = fy
        m.axis.x, m.axis.y, m.axis.z = (float(v) for v in ax)
        m.is_box = bool(h.get("is_box", False))
        if h["kind"] == "puck":  # 계획이 겨냥한 퍽 − 지금 손 (HandTarget.point, map)
            p = self.target.point
            ins = hg.to_base(hg.aimed_centre((p.x, p.y, p.z)), base_t, base_R) - ee_t
            m.insp_rel.x, m.insp_rel.y, m.insp_rel.z = (float(v) for v in ins)
        if nb is not None:
            m.neighbour = True
            m.neighbour_rel.x, m.neighbour_rel.y, m.neighbour_rel.z = (float(v) for v in nb)
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
        if fr is None or age > self.max_age:  # 못 쓰는 검출: 들던 값이 있으면 그대로 (타이머가 max_age_s로 끝냄)
            if self.held is None:
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
            # 상자를 못 봐도 계획 칸으로 (손 위치는 TF로 앎)
            self.held = (msg.header.stamp, {"kind": "cell", "c_map": hg.cell_now(pt, plan, box)}, None)
            self.publish_held()
            return
        pucks = []  # (tag, centre_b, standing, yaw_b, is_box, centre_map, yaw_map)
        for o in msg.objects:
            if o.class_name not in ("puck", "puck_lying"):
                continue
            standing = o.class_name == "puck"
            p = o.pose.pose.position
            c_map = hg.puck_centre((p.x, p.y, p.z), standing)
            c = hg.to_base(c_map, base_t, base_R)
            known_yaw = bool(detail(o, "yaw_period"))
            yaw_map = yaw_of_quat(o.pose.pose.orientation) if known_yaw else None
            yaw_b = hg.wrap(yaw_map - base_yaw, 2 * math.pi) if known_yaw else None
            tag = int(detail(o, "tag")) if detail(o, "tag").lstrip("-").isdigit() else -1
            pucks.append((tag, c, standing, yaw_b, detail(o, "shape") == "box", c_map, yaw_map))
        k = hg.pick_target([q[:5] for q in pucks], ee_t, t.puck_tag)
        if k is None:  # 목표 퍽이 이 영상에 없음 (태그가 화면 밖 · 너무 가까워 위치 없음) → 들던 값은 max_age_s까지 계속 보냄
            if self.held is None:
                self.publish(m)
            return
        _, _, standing, _, is_box, c_map, yaw_map = pucks[k]
        held = {"kind": "puck", "c_map": c_map, "standing": standing, "yaw_map": yaw_map, "is_box": is_box}
        self.held = (msg.header.stamp, held, hg.neighbour([q[:5] for q in pucks], k))
        self.publish_held()


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
