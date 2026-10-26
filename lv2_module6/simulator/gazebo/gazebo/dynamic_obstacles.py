#!/usr/bin/env python3
"""무작위 동적 장애물 런타임.

- /gz/cmd_vel 속도 명령이 살아 있을 때만 빈 후보 지점에서
  사람 형상(원기둥 몸통 + 머리 + 팔다리)을 create 서비스로 소환한다.
  정지하면 active를 모두 remove한다.
- 살아 있는 동안 빈 지점 사이를 천천히 이동하고, 수명이 끝나면 remove로
  확실히 제거한다. 따라서 월드/ RViz 마커에 장애물이 누적되지 않는다.
- `/gz_dynamic/markers`, `/gz_dynamic/info`를 발행한다.

사용:
  python3 dynamic_obstacles.py --state /tmp/beebot_scene.json
"""
import argparse
import json
import math
import random
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

try:
    import gz.transport as gz
    from gz.msgs.boolean_pb2 import Boolean
    from gz.msgs.entity_factory_pb2 import EntityFactory
    from gz.msgs.entity_pb2 import Entity
    from gz.msgs.pose_pb2 import Pose
except ImportError:
    gz = None

WORLD = "default"


def call(node, service, request, request_type, response_type, timeout=1500):
    try:
        ok, response = node.request(service, request, request_type, response_type, timeout)
        return bool(ok) and bool(getattr(response, "data", False))
    except Exception as exc:  # noqa: BLE001
        print(f"[dynamic] service {service} failed: {exc}", flush=True)
        return False


def remove_model(gz_node, name):
    request = Entity()
    request.name = name
    request.type = Entity.MODEL
    return call(gz_node, f"/world/{WORLD}/remove", request, Entity, Boolean)


def set_pose(gz_node, name, x, y, z, yaw):
    request = Pose()
    request.name = name
    request.position.x = float(x)
    request.position.y = float(y)
    request.position.z = float(z)
    request.orientation.z = math.sin(yaw / 2.0)
    request.orientation.w = math.cos(yaw / 2.0)
    return call(gz_node, f"/world/{WORLD}/set_pose", request, Pose, Boolean)


def human_sdf(name, size, color):
    """퍽과 구분되는 간단한 사람 형상(원기둥 몸통 + 머리 + 팔다리)."""
    w, d, h = size
    r, g, b = color
    skin = (0.93, 0.76, 0.60)
    pants = (0.12, 0.15, 0.24)
    torso_r = w * 0.34
    torso_h = h * 0.40
    torso_z = h * 0.05
    head_r = w * 0.33
    head_z = h * 0.375
    leg_h = h * 0.53
    leg_z = -h * 0.5 + leg_h / 2.0
    leg_x = w * 0.25
    leg_r = w * 0.075
    arm_h = h * 0.36
    arm_z = h * 0.14
    arm_x = w * 0.43
    arm_r = w * 0.060
    mass = max(0.2, w * d * h * 120.0)
    ixx = mass * (3.0 * (w * 0.34) ** 2 + h * h) / 12.0
    izz = mass * (w * 0.34) ** 2 / 2.0
    return f"""
<sdf version="1.8">
  <model name="{name}">
    <static>false</static>
    <self_collide>false</self_collide>
    <link name="link">
      <inertial>
        <mass>{mass:.6g}</mass>
        <inertia><ixx>{ixx:.6g}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{ixx:.6g}</iyy><iyz>0</iyz><izz>{izz:.6g}</izz></inertia>
      </inertial>
      <collision name="collision">
        <pose>0 0 0 0 0 0</pose>
        <geometry><cylinder><radius>{w * 0.34:.6g}</radius><length>{h * 0.82:.6g}</length></cylinder></geometry>
      </collision>
      <visual name="torso">
        <pose>0 0 {torso_z:.6g} 0 0 0</pose>
        <geometry><cylinder><radius>{torso_r:.6g}</radius><length>{torso_h:.6g}</length></cylinder></geometry>
        <material><ambient>{r:.5g} {g:.5g} {b:.5g} 1</ambient>
          <diffuse>{r:.5g} {g:.5g} {b:.5g} 1</diffuse><specular>0.05 0.05 0.05 1</specular></material>
      </visual>
      <visual name="head">
        <pose>0 0 {head_z:.6g} 0 0 0</pose>
        <geometry><sphere><radius>{head_r:.6g}</radius></sphere></geometry>
        <material><ambient>{skin[0]:.5g} {skin[1]:.5g} {skin[2]:.5g} 1</ambient>
          <diffuse>{skin[0]:.5g} {skin[1]:.5g} {skin[2]:.5g} 1</diffuse></material>
      </visual>
      <visual name="leg_left">
        <pose>{-leg_x:.6g} 0 {leg_z:.6g} 0 0 0</pose>
        <geometry><cylinder><radius>{leg_r:.6g}</radius><length>{leg_h:.6g}</length></cylinder></geometry>
        <material><ambient>{pants[0]:.5g} {pants[1]:.5g} {pants[2]:.5g} 1</ambient>
          <diffuse>{pants[0]:.5g} {pants[1]:.5g} {pants[2]:.5g} 1</diffuse></material>
      </visual>
      <visual name="leg_right">
        <pose>{leg_x:.6g} 0 {leg_z:.6g} 0 0 0</pose>
        <geometry><cylinder><radius>{leg_r:.6g}</radius><length>{leg_h:.6g}</length></cylinder></geometry>
        <material><ambient>{pants[0]:.5g} {pants[1]:.5g} {pants[2]:.5g} 1</ambient>
          <diffuse>{pants[0]:.5g} {pants[1]:.5g} {pants[2]:.5g} 1</diffuse></material>
      </visual>
      <visual name="arm_left">
        <pose>{-arm_x:.6g} 0 {arm_z:.6g} 0 0 0</pose>
        <geometry><cylinder><radius>{arm_r:.6g}</radius><length>{arm_h:.6g}</length></cylinder></geometry>
        <material><ambient>{r:.5g} {g:.5g} {b:.5g} 1</ambient>
          <diffuse>{r:.5g} {g:.5g} {b:.5g} 1</diffuse></material>
      </visual>
      <visual name="arm_right">
        <pose>{arm_x:.6g} 0 {arm_z:.6g} 0 0 0</pose>
        <geometry><cylinder><radius>{arm_r:.6g}</radius><length>{arm_h:.6g}</length></cylinder></geometry>
        <material><ambient>{r:.5g} {g:.5g} {b:.5g} 1</ambient>
          <diffuse>{r:.5g} {g:.5g} {b:.5g} 1</diffuse></material>
      </visual>
      <visual name="neck">
        <pose>0 0 {head_z - head_r - 0.018:.6g} 0 0 0</pose>
        <geometry><cylinder><radius>{w * 0.11:.6g}</radius><length>0.036</length></cylinder></geometry>
        <material><ambient>{skin[0]:.5g} {skin[1]:.5g} {skin[2]:.5g} 1</ambient>
          <diffuse>{skin[0]:.5g} {skin[1]:.5g} {skin[2]:.5g} 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


def segment_hits_box(x0, y0, x1, y1, box, margin):
    """2D 선분과 축정렬 박스의 교차 (slab method)."""
    dx, dy = x1 - x0, y1 - y0
    tmin, tmax = 0.0, 1.0
    for p, d, lo, hi in (
        (x0, dx, box["x"] - box["sx"] / 2.0 - margin, box["x"] + box["sx"] / 2.0 + margin),
        (y0, dy, box["y"] - box["sy"] / 2.0 - margin, box["y"] + box["sy"] / 2.0 + margin),
    ):
        if abs(d) < 1e-12:
            if p < lo or p > hi:
                return False
            continue
        t1, t2 = (lo - p) / d, (hi - p) / d
        if t1 > t2:
            t1, t2 = t2, t1
        tmin, tmax = max(tmin, t1), min(tmax, t2)
        if tmin > tmax:
            return False
    return True


class DynamicObstacles(Node):
    def __init__(self, state):
        super().__init__("gz_dynamic_obstacles")
        self.state = state
        self.cfg = state.get("dynamic") or {}
        self.points = list(self.cfg.get("spawn_points") or [])
        self.walls = list(state.get("walls") or [])
        self.static_objects = [o for o in state.get("objects", [])
                               if o.get("kind") in ("obstacle", "traffic_light")]
        self.max_active = int(self.cfg.get("max_active", 0))
        self.lifetime = float(self.cfg.get("lifetime_sec", 12.0))
        self.respawn = float(self.cfg.get("respawn_sec", 3.0))
        self.speed = float(self.cfg.get("speed_mps", 0.16))
        self.size = [float(v) for v in self.cfg.get("size", [0.22, 0.22, 0.60])]
        self.colors = [tuple(c) for c in self.cfg.get("colors", [[0.8, 0.25, 0.1]])]
        self.rng = random.Random(int(time.time() * 1000) & 0x7FFFFFFF)
        self.gz_node = gz.Node() if gz is not None else None
        self.active = {}
        self.spawn_seq = 0
        self.next_spawn = time.monotonic()
        self.last_update = time.monotonic()
        self.last_cmd_at = 0.0
        self.last_odom_at = 0.0
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.odom_speed = 0.0
        self.moving = False
        self.was_moving = False
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, "/gz_dynamic/markers", qos)
        self.info_pub = self.create_publisher(String, "/gz_dynamic/info", qos)
        self.marker_pub.publish(self._delete_all())
        # 실제 이동 중에만 동적 장애물을 운용한다. 키보드/내비가 속도를 보내지 않거나
        # 정지하면 아래 tick에서 active를 모두 제거한다.
        self.cmd_sub = self.create_subscription(Twist, "/gz/cmd_vel", self._cmd_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, "/gz/odom", self._odom_cb, 10)
        self.timer = self.create_timer(0.20, self.tick)
        self.get_logger().info(
            f"dynamic obstacles: max={self.max_active} lifetime={self.lifetime:.1f}s "
            f"respawn={self.respawn:.1f}s points={len(self.points)}")

    def _cmd_cb(self, msg):
        self.last_cmd_at = time.monotonic()
        self.cmd_linear = abs(float(msg.linear.x)) + abs(float(msg.linear.y))
        self.cmd_angular = abs(float(msg.angular.z))

    def _odom_cb(self, msg):
        self.last_odom_at = time.monotonic()
        v = msg.twist.twist
        self.odom_speed = math.hypot(float(v.linear.x), float(v.linear.y))

    def is_moving(self):
        now = time.monotonic()
        # /gz/odom의 twist는 이 월드에서 마지막 명령을 그대로 유지하는 경우가 있어
        # 실제 정지 판정에 쓰지 않는다. 속도 명령이 살아 있을 때만 이동 중으로 본다.
        cmd_fresh = now - self.last_cmd_at < 0.8
        return bool(cmd_fresh and (self.cmd_linear >= 0.015 or self.cmd_angular >= 0.05))

    def _name(self, slot):
        # 같은 슬롯을 remove 직후 재사용하면 Gazebo의 비동기 삭제와 create가
        # 충돌해 모델이 없는 채 active로 남을 수 있다. 매번 고유 이름을 쓴다.
        self.spawn_seq += 1
        return f"gz_dyn_{slot}_{self.spawn_seq}"

    def _delete_all(self):
        msg = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        msg.markers.append(marker)
        return msg

    def _point_clear_for_spawn(self, point):
        for item in self.active.values():
            if math.hypot(point["x"] - item["x"], point["y"] - item["y"]) < 0.32:
                return False
        return True

    def _segment_clear(self, x0, y0, x1, y1):
        for wall in self.walls:
            if segment_hits_box(x0, y0, x1, y1, wall, 0.035):
                return False
        for obj in self.static_objects:
            p = obj["pose"]
            radius = max(obj["size"][0], obj["size"][1]) / 2.0 + 0.06
            if segment_hits_box(x0, y0, x1, y1,
                                {"x": p["x"], "y": p["y"], "sx": radius * 2.0,
                                 "sy": radius * 2.0}, 0.0):
                return False
        return True

    def _choose_target(self, item):
        candidates = [p for p in self.points
                      if math.hypot(p["x"] - item["x"], p["y"] - item["y"]) > 0.18]
        self.rng.shuffle(candidates)
        for point in candidates[:30]:
            if self._segment_clear(item["x"], item["y"], point["x"], point["y"]):
                return dict(point)
        return dict(self.rng.choice(candidates or self.points))

    def _spawn(self, slot):
        if not self.points:
            return
        points = [p for p in self.points if self._point_clear_for_spawn(p)]
        self.rng.shuffle(points)
        for point in points[:12]:
            name = self._name(slot)
            color = self.rng.choice(self.colors)
            request = EntityFactory()
            request.sdf = human_sdf(name, self.size, color)
            request.name = name
            request.allow_renaming = False
            if not call(self.gz_node, f"/world/{WORLD}/create", request, EntityFactory, Boolean):
                continue
            z = self.size[2] / 2.0 + 0.012
            if not set_pose(self.gz_node, name, point["x"], point["y"], z, 0.0):
                remove_model(self.gz_node, name)
                continue
            item = {
                "slot": slot, "name": name, "x": point["x"], "y": point["y"], "z": z,
                "yaw": 0.0, "target": None, "born": time.monotonic(),
                "color": color,
            }
            item["target"] = self._choose_target(item)
            self.active[slot] = item
            self.get_logger().info(
                f"spawn {name} at ({point['x']:+.2f}, {point['y']:+.2f}) "
                f"life={self.lifetime:.1f}s")
            return

    def _remove(self, slot, reason="lifetime"):
        item = self.active.pop(slot, None)
        if item is None:
            return
        remove_model(self.gz_node, item["name"])
        self.marker_pub.publish(self._delete_all())
        self.get_logger().info(f"remove {item['name']} ({reason})")

    def tick(self):
        now = time.monotonic()
        dt = min(0.5, max(0.01, now - self.last_update))
        self.last_update = now
        self.moving = self.is_moving()

        if not self.moving:
            # 미션 수행 중이거나 정지 중에는 장애물을 남기지 않는다.
            if self.active:
                for slot in list(self.active):
                    self._remove(slot, reason="stopped")
            if self.was_moving:
                self.marker_pub.publish(self._delete_all())
            self.was_moving = False
            self.next_spawn = now
            self.marker_pub.publish(MarkerArray())
            self.info_pub.publish(String(data=json.dumps({
                "time": self.get_clock().now().nanoseconds / 1e9,
                "moving": False, "active": []}, ensure_ascii=False)))
            return

        if not self.was_moving:
            # 이동을 시작하면 바로 한 마리부터 보이게 한다.
            self.next_spawn = now
            self.was_moving = True

        for slot, item in list(self.active.items()):
            if now - item["born"] >= self.lifetime:
                self._remove(slot)
                # 다른 슬롯의 재등장 예약을 뒤로 미루지 않는다.
                self.next_spawn = min(self.next_spawn, now + self.respawn)
                continue
            target = item.get("target")
            if target is None or math.hypot(target["x"] - item["x"],
                                            target["y"] - item["y"]) < 0.04:
                item["target"] = self._choose_target(item)
                target = item["target"]
            dx, dy = target["x"] - item["x"], target["y"] - item["y"]
            dist = math.hypot(dx, dy)
            if dist > 1e-6:
                step = min(dist, self.speed * dt)
                item["x"] += dx / dist * step
                item["y"] += dy / dist * step
                # 모델의 로컬 X가 어깨 폭이므로, 진행 방향에 수직으로 세운다.
                item["yaw"] = math.atan2(dy, dx) + math.pi / 2.0
                set_pose(self.gz_node, item["name"], item["x"], item["y"],
                         item["z"], item["yaw"])

        if len(self.active) < self.max_active and now >= self.next_spawn:
            for slot in range(self.max_active):
                if slot not in self.active:
                    self._spawn(slot)
                    self.next_spawn = now + self.respawn
                    break

        msg = MarkerArray()
        info = {"time": self.get_clock().now().nanoseconds / 1e9,
                "moving": True, "active": []}
        for slot, item in self.active.items():
            w, d, h = self.size
            stamp = self.get_clock().now().to_msg()
            cy, sy = math.cos(item["yaw"]), math.sin(item["yaw"])
            r, g, b = item["color"]
            skin = (0.93, 0.76, 0.60)
            pants = (0.12, 0.15, 0.24)

            def add_part(part_id, marker_type, dx, dz, scale, color):
                part = Marker()
                part.header.frame_id = "world"
                part.header.stamp = stamp
                part.ns = "gz_dynamic"
                part.id = slot * 10 + part_id
                part.type = marker_type
                part.action = Marker.ADD
                part.pose.position.x = item["x"] + dx * cy
                part.pose.position.y = item["y"] + dx * sy
                part.pose.position.z = item["z"] + dz
                part.pose.orientation.z = math.sin(item["yaw"] / 2.0)
                part.pose.orientation.w = math.cos(item["yaw"] / 2.0)
                part.scale.x, part.scale.y, part.scale.z = scale
                part.color.r, part.color.g, part.color.b = color
                part.color.a = 0.95
                msg.markers.append(part)

            add_part(0, Marker.CYLINDER, 0.0, h * 0.05,
                     (w * 0.68, d * 0.68, h * 0.40), (r, g, b))
            add_part(1, Marker.SPHERE, 0.0, h * 0.375,
                     (w * 0.66, d * 0.66, w * 0.66), skin)
            add_part(2, Marker.CYLINDER, -w * 0.25, -h * 0.235,
                     (w * 0.15, w * 0.15, h * 0.53), pants)
            add_part(3, Marker.CYLINDER, w * 0.25, -h * 0.235,
                     (w * 0.15, w * 0.15, h * 0.53), pants)
            add_part(4, Marker.CYLINDER, -w * 0.43, h * 0.14,
                     (w * 0.12, w * 0.12, h * 0.36), (r, g, b))
            add_part(5, Marker.CYLINDER, w * 0.43, h * 0.14,
                     (w * 0.12, w * 0.12, h * 0.36), (r, g, b))

            label = Marker()
            label.header.frame_id = "world"
            label.header.stamp = stamp
            label.ns = "gz_dynamic_labels"
            label.id = 100 + slot
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x, label.pose.position.y = item["x"], item["y"]
            label.pose.position.z = item["z"] + h / 2.0 + 0.12
            label.pose.orientation.w = 1.0
            label.scale.z = 0.06
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f"PERSON {slot} ({self.lifetime - (now - item['born']):.1f}s)"
            msg.markers.append(label)
            info["active"].append({"slot": slot, "name": item["name"],
                                   "x": item["x"], "y": item["y"],
                                   "remaining": max(0.0, self.lifetime - (now - item["born"]))})
        self.marker_pub.publish(msg)
        self.info_pub.publish(String(data=json.dumps(info, ensure_ascii=False)))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--state", default="/tmp/beebot_scene.json")
    args = ap.parse_args()
    state = json.loads(Path(args.state).read_text())
    rclpy.init()
    node = DynamicObstacles(state)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for slot in list(node.active):
            node._remove(slot, reason="shutdown")
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
