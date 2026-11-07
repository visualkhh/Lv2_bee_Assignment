#!/usr/bin/env python3
"""통행로 위 갠트리 신호등 런타임.

- generate_world.py가 만든 두 개의 통행로 갠트리 신호등(상부 빔 + 기둥 +
  하우징 + 꺼진 렌즈 3개)을 읽는다.
- 일정 시간마다 빨강 → 초록 → 주황 순서로 바꾼다. 색별 발광 구 3개를
  미리 만들어 두고, 켜진 색만 렌즈 앞으로 set_pose하고 나머지는 바닥 아래로
  숨긴다. create/remove를 반복하지 않으므로 모델이 쌓이지 않는다.
- RViz용 /gz_traffic/markers와 /gz_traffic/info도 발행한다.

사용:
  python3 traffic_lights.py --state /tmp/beebot_scene.json
"""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

try:
    import gz.transport as gz
    from gz.msgs.boolean_pb2 import Boolean
    from gz.msgs.entity_factory_pb2 import EntityFactory
    from gz.msgs.entity_pb2 import Entity
    from gz.msgs.pose_pb2 import Pose
except ImportError:  # ROS만 있는 환경에서도 import는 가능하게
    gz = None

COLORS = {
    "red": (0.95, 0.04, 0.04),
    "orange": (1.00, 0.45, 0.00),
    "green": (0.04, 0.85, 0.16),
}
WORLD = "default"


def call(node, service, request, request_type, response_type, timeout=1800):
    """gz-transport 서비스 호출. (ok, response.data)를 돌려준다."""
    try:
        ok, response = node.request(service, request, request_type, response_type, timeout)
        return bool(ok) and bool(getattr(response, "data", False))
    except Exception as exc:  # noqa: BLE001 - 서비스 미기동 시 계속 돌아야 함
        print(f"[traffic] service {service} failed: {exc}", flush=True)
        return False


def remove_model(gz_node, name):
    request = Entity()
    request.name = name
    request.type = Entity.MODEL
    return call(gz_node, f"/world/{WORLD}/remove", request, Entity, Boolean)


def make_lit_sdf(name, color):
    r, g, b = color
    return f"""
<sdf version="1.8">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="lit">
        <geometry><sphere><radius>0.032</radius></sphere></geometry>
        <material>
          <ambient>{r:.5g} {g:.5g} {b:.5g} 1</ambient>
          <diffuse>{r:.5g} {g:.5g} {b:.5g} 1</diffuse>
          <emissive>{r:.5g} {g:.5g} {b:.5g} 1</emissive>
          <specular>0.02 0.02 0.02 1</specular>
          <lighting>false</lighting>
        </material>
        <cast_shadows>false</cast_shadows>
      </visual>
    </link>
  </model>
</sdf>"""


def set_pose(gz_node, name, x, y, z):
    request = Pose()
    request.name = name
    request.position.x = float(x)
    request.position.y = float(y)
    request.position.z = float(z)
    request.orientation.w = 1.0
    return call(gz_node, f"/world/{WORLD}/set_pose", request, Pose, Boolean)


class TrafficLights(Node):
    def __init__(self, state):
        super().__init__("gz_traffic_lights")
        self.state = state
        self.traffic = state.get("traffic") or {}
        self.lights = self.traffic.get("lights") or []
        self.interval = float(self.traffic.get("interval_sec", 3.0))
        self.order = list(self.traffic.get("order") or ("red", "green", "orange"))
        self.gz_node = gz.Node() if gz is not None else None
        self.started = time.monotonic()
        self.last_phase = {}
        self.lit_models = {}
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, "/gz_traffic/markers", qos)
        self.info_pub = self.create_publisher(String, "/gz_traffic/info", qos)
        self.marker_pub.publish(self._delete_all())
        self._create_lit_models()
        self.timer = self.create_timer(0.25, self.tick)
        self.get_logger().info(
            f"traffic lights: {[l['name'] for l in self.lights]} "
            f"interval={self.interval:.1f}s order={self.order}")

    def _delete_all(self):
        msg = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        msg.markers.append(marker)
        return msg

    def phase_for(self, index, now=None):
        if not self.order:
            return "red"
        elapsed = (time.monotonic() if now is None else now) - self.started
        return self.order[(int(elapsed / max(0.2, self.interval)) + int(index)) % len(self.order)]

    def lens_for(self, light, color):
        for lens in light.get("lenses", []):
            if lens.get("color") == color:
                return lens
        return (light.get("lenses") or [{}])[0]

    def _create_lit_models(self):
        """색별 발광 구를 한 번만 만들고, 초기에는 바닥 아래에 숨긴다."""
        if self.gz_node is None:
            return
        for light in self.lights:
            models = {}
            pose = light["pose"]
            for color in ("red", "orange", "green"):
                name = f"{light['name']}_lit_{color}"
                # start.sh는 월드를 새로 만들므로 여기서 remove하지 않는다.
                # (없는 모델 remove는 Gazebo 로그에 오류만 남긴다.)
                request = EntityFactory()
                request.sdf = make_lit_sdf(name, COLORS[color])
                request.name = name
                request.allow_renaming = False
                if not call(self.gz_node, f"/world/{WORLD}/create",
                            request, EntityFactory, Boolean):
                    print(f"[traffic] create failed: {name}", flush=True)
                    continue
                set_pose(self.gz_node, name, pose["x"], pose["y"], -0.5)
                models[color] = name
            light["_lit_models"] = models

    def swap_lit(self, light, color):
        """현재 색만 렌즈 앞에 두고 나머지 색 구는 바닥 아래로 숨긴다."""
        pose = light["pose"]
        models = light.get("_lit_models") or {}
        active = None
        for name_color in ("red", "orange", "green"):
            name = models.get(name_color)
            if name is None:
                continue
            if name_color == color:
                lens = self.lens_for(light, color)
                offset = lens.get("world_offset") or [0.0, 0.0, 0.0]
                active = (pose["x"] + offset[0], pose["y"] + offset[1],
                          pose["z"] + offset[2])
                set_pose(self.gz_node, name, *active)
            else:
                set_pose(self.gz_node, name, pose["x"], pose["y"], -0.5)
        return active

    def tick(self):
        if not self.lights:
            return
        msg = MarkerArray()
        info = {"time": self.get_clock().now().nanoseconds / 1e9, "lights": []}
        for light in self.lights:
            phase = self.phase_for(light.get("index", 0))
            if self.last_phase.get(light["name"]) != phase:
                target = self.swap_lit(light, phase)
                self.last_phase[light["name"]] = phase
                if target is not None:
                    light["_lit_position"] = target
            target = light.get("_lit_position") or (
                light["pose"]["x"], light["pose"]["y"], light["pose"]["z"])
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "gz_traffic"
            marker.id = int(light.get("index", 0))
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = target
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.075
            r, g, b = COLORS.get(phase, (1.0, 1.0, 1.0))
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = r, g, b, 1.0
            msg.markers.append(marker)

            label = Marker()
            label.header = marker.header
            label.ns = "gz_traffic_labels"
            label.id = 100 + int(light.get("index", 0))
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x, label.pose.position.y = target[0], target[1]
            label.pose.position.z = target[2] + 0.16
            label.pose.orientation.w = 1.0
            label.scale.z = 0.065
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f"{light.get('label', light['name'])}: {phase.upper()}"
            msg.markers.append(label)
            info["lights"].append({
                "name": light["name"], "label": light.get("label", ""),
                "phase": phase, "x": target[0], "y": target[1], "z": target[2],
            })
        self.marker_pub.publish(msg)
        self.info_pub.publish(String(data=json.dumps(info, ensure_ascii=False)))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--state", default="/tmp/beebot_scene.json")
    args = ap.parse_args()
    state = json.loads(Path(args.state).read_text())
    rclpy.init()
    node = TrafficLights(state)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for light in node.lights:
            for name in (light.get("_lit_models") or {}).values():
                if node.gz_node is not None:
                    remove_model(node.gz_node, name)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
