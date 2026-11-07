#!/usr/bin/env python3
"""Gazebo 씬(장애물·AprilTag 퍽·분산 미션 태그·목표 구역·신호등 하우징)의 실제 pose를 RViz/ROS로 발행.

- 마커/포즈의 frame은 Gazebo world다. /world/<world>/pose/info에서 읽은 **진짜** 모델 pose를 쓴다.
- RViz 고정 프레임이 base_footprint여도 보이도록 base_footprint → world TF를 같이 낸다.
  (world는 base_footprint의 자식으로 붙는다. world→base 역변환이 로봇의 실제 Gazebo pose다.)
- /world/<world>/pose/info를 직접 gz-transport로 구독하므로 ros_gz_bridge 설정은 필요 없다.
- 토픽:
    /gz_obstacles/markers  visualization_msgs/MarkerArray  (RViz MarkerArray, 기존 이름 유지)
    /gz_obstacles/poses    geometry_msgs/PoseArray         (씬 전체 실제 pose)
    /gz_mission/info       std_msgs/String                 (미션·태그 매핑 JSON, transient_local)

퍽은 6면 태그가 붙은 30×30×60 mm 상자다. 마커는 실제 Gazebo pose를 따라가므로
로봇이 퍽을 밀거나 들면 RViz에서도 같이 움직인다.
동적 장애물과 켜진 신호등은 각각 dynamic_obstacles.py / traffic_lights.py가
/gz_dynamic/markers, /gz_traffic/markers로 별도 발행한다.
"""
import argparse
import copy
import json
import math
import sys
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

try:  # Gazebo가 없거나 Python 바인딩이 없는 환경에서도 마커는 스폰 pose로 뜨게
    import gz.transport as gz_transport
    from gz.msgs.pose_v_pb2 import Pose_V
except Exception:  # pragma: no cover - Gazebo 설치 환경에서는 정상
    gz_transport = None
    Pose_V = None


def normalize_quat(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x / n, y / n, z / n, w / n


def quat_rotate(q, v):
    """q * (0,v) * q^-1 — q는 정규화된 (x,y,z,w)."""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def inverse_pose(pose):
    """T_world_base → T_base_world (translation, quaternion)."""
    q = normalize_quat((pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w))
    q_inv = (-q[0], -q[1], -q[2], q[3])
    t = (pose.position.x, pose.position.y, pose.position.z)
    rotated = quat_rotate(q_inv, t)
    return (-rotated[0], -rotated[1], -rotated[2]), q_inv


def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def gz_pose_to_ros(p):
    out = Pose()
    out.position.x = float(p.position.x)
    out.position.y = float(p.position.y)
    out.position.z = float(p.position.z)
    out.orientation.x = float(p.orientation.x)
    out.orientation.y = float(p.orientation.y)
    out.orientation.z = float(p.orientation.z)
    out.orientation.w = float(p.orientation.w)
    return out


def state_pose_to_ros(p):
    out = Pose()
    out.position.x = float(p["x"])
    out.position.y = float(p["y"])
    out.position.z = float(p["z"])
    qx, qy, qz, qw = euler_to_quat(float(p.get("roll", 0.0)),
                                   float(p.get("pitch", 0.0)),
                                   float(p.get("yaw", 0.0)))
    out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = qx, qy, qz, qw
    return out


class GzScenePublisher(Node):
    def __init__(self, args):
        super().__init__("gz_scene_publisher")
        self.args = args
        state_path = Path(args.state)
        if not state_path.is_file():
            raise FileNotFoundError(f"state 파일 없음: {state_path}")
        self.state = json.loads(state_path.read_text())
        self.objects = self.state.get("objects", [])
        self.mission = self.state.get("mission", {})
        self.frame_id = self.state.get("frame_id", "world")
        self.lock = threading.Lock()
        self.gz_poses = {}
        self.gz_seen = False
        self.first_markers = True

        qos = QoSProfile(depth=10,
                         history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.marker_pub = self.create_publisher(MarkerArray, args.markers_topic, qos)
        self.pose_pub = self.create_publisher(PoseArray, args.poses_topic, qos)
        info_qos = QoSProfile(depth=1,
                              history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mission_pub = self.create_publisher(String, args.mission_topic, info_qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / max(args.rate, 1.0), self.publish)
        self.gz_node = None
        if gz_transport is not None and Pose_V is not None:
            try:
                self.gz_node = gz_transport.Node()
                topic = f"/world/{self.state.get('world', 'default')}/pose/info"
                self.gz_node.subscribe(Pose_V, topic, self._on_gz_pose)
                self.get_logger().info(f"Gazebo pose 구독: {topic}")
            except Exception as e:  # Gazebo transport가 아직 없어도 ROS 마커는 동작
                self.gz_node = None
                self.get_logger().warning(f"Gazebo pose 구독 실패 ({e}); 스폰 pose 사용")
        else:
            self.get_logger().warning("gz.transport 없음 — 스폰 pose로 마커 발행")

        self._publish_mission_info()
        counts = {}
        for obj in self.objects:
            counts[obj.get("kind", "obstacle")] = counts.get(obj.get("kind", "obstacle"), 0) + 1
        self.get_logger().info(
            f"{len(self.objects)}개 씬 객체 {counts} → {args.markers_topic} / {args.poses_topic} "
            f"(frame={self.frame_id}, mission={self.mission.get('name', '?')})")

    def _publish_mission_info(self):
        payload = {"mission": self.mission, "tag_map": self.state.get("tag_map", {})}
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.mission_pub.publish(msg)

    def _on_gz_pose(self, msg):
        poses = {}
        for p in msg.pose:
            if p.name:
                poses[p.name] = gz_pose_to_ros(p)
        with self.lock:
            self.gz_poses.update(poses)
            self.gz_seen = True

    def _robot_pose(self):
        with self.lock:
            pose = self.gz_poses.get("beebot")
        if pose is not None:
            return pose
        # 아직 Gazebo pose가 없을 때: 스폰 위치(지면 z=0)로 world 프레임을 둔다.
        s = self.state.get("spawn", {})
        return state_pose_to_ros({"x": s.get("x", 0.0), "y": s.get("y", 0.0),
                                  "z": 0.0, "yaw": s.get("yaw", 0.0)})

    def _object_pose(self, obj):
        with self.lock:
            pose = self.gz_poses.get(obj["name"])
        return pose if pose is not None else state_pose_to_ros(obj["pose"])

    def _marker_scale_type(self, obj):
        kind = obj.get("kind", "obstacle")
        shape = obj.get("shape", "box")
        if shape == "cylinder":
            diameter, height = obj["size"]
            return Marker.CYLINDER, (diameter, diameter, height)
        if shape == "plane":
            sx, sy = obj["size"][0], obj["size"][1]
            return Marker.CUBE, (sx, sy, 0.003 if kind == "mission_tag" else 0.002)
        sx, sy, sz = obj["size"]
        return Marker.CUBE, (sx, sy, sz)

    def _label(self, obj, pose):
        kind = obj.get("kind", "obstacle")
        p = pose.position
        if kind == "puck":
            return (f"{obj['name']} [{obj.get('tag_id')}] "
                    f"({p.x:+.2f}, {p.y:+.2f})", 0.012)
        if kind == "mission_tag":
            active = " *ACTIVE*" if obj.get("active") else ""
            return (f"MISSION {obj.get('mission', '')}: {obj.get('label', '')} "
                    f"[{obj.get('tag_id')}]{active}", 0.018)
        if kind == "traffic_light":
            return (f"{obj.get('label', obj['name'])} ({p.x:+.2f}, {p.y:+.2f})",
                    0.016)
        if kind == "zone_tag":
            return (f"ZONE {obj.get('label', '')} [{obj.get('tag_id')}]", 0.016)
        if kind == "zone":
            return (f"{obj.get('label', obj['name'])} "
                    f"({p.x:+.2f}, {p.y:+.2f})", 0.016)
        return (f"{obj['name']} ({p.x:+.2f}, {p.y:+.2f})", 0.014)

    def publish(self):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        if self.first_markers:
            clear = Marker()
            clear.header.frame_id = self.frame_id
            clear.header.stamp = now
            clear.action = Marker.DELETEALL
            markers.markers.append(clear)
            self.first_markers = False
        pose_array = PoseArray()
        pose_array.header.frame_id = self.frame_id
        pose_array.header.stamp = now

        for i, obj in enumerate(self.objects):
            pose = self._object_pose(obj)
            pose_array.poses.append(pose)
            marker_type, scale = self._marker_scale_type(obj)
            kind = obj.get("kind", "obstacle")

            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = now
            m.ns = "gz_obstacles"
            m.id = i
            m.type = marker_type
            m.action = Marker.ADD
            m.pose = pose
            m.scale.x, m.scale.y, m.scale.z = scale
            r, g, b = obj.get("color", [1.0, 0.5, 0.0])[:3]
            if kind == "mission_tag" and obj.get("active"):
                r, g, b = 0.10, 0.80, 0.30
            m.color.r, m.color.g, m.color.b = float(r), float(g), float(b)
            m.color.a = 0.55 if kind == "zone" else (0.95 if kind in ("mission_tag", "zone_tag") else 0.85)
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0
            markers.markers.append(m)

            text, text_h = self._label(obj, pose)
            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = now
            label.ns = "gz_obstacles_labels"
            label.id = 1000 + i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = copy.deepcopy(pose)
            if kind in ("mission_tag", "zone_tag"):
                label.pose.position.z += max(scale[0], scale[1]) / 2.0 + 0.02
            elif kind == "zone":
                label.pose.position.z += 0.02
            else:
                label.pose.position.z += max(scale[2] / 2.0, 0.0) + 0.025
            label.scale.z = text_h
            label.color.r = label.color.g = label.color.b = 0.95
            label.color.a = 0.95
            label.text = text
            markers.markers.append(label)

        # world ← base_footprint: Gazebo 로봇 pose의 역변환.
        robot = self._robot_pose()
        t_inv, q_inv = inverse_pose(robot)
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "base_footprint"
        tf.child_frame_id = self.frame_id
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = t_inv
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q_inv
        self.tf_broadcaster.sendTransform(tf)

        self.marker_pub.publish(markers)
        self.pose_pub.publish(pose_array)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--state", default="/tmp/beebot_scene.json")
    ap.add_argument("--markers-topic", default="/gz_obstacles/markers")
    ap.add_argument("--poses-topic", default="/gz_obstacles/poses")
    ap.add_argument("--mission-topic", default="/gz_mission/info")
    ap.add_argument("--rate", type=float, default=10.0)
    args = ap.parse_args()
    rclpy.init()
    node = None
    try:
        node = GzScenePublisher(args)
        rclpy.spin(node)
    except (KeyboardInterrupt, FileNotFoundError) as e:
        if isinstance(e, FileNotFoundError):
            print(f"FAIL: {e}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
