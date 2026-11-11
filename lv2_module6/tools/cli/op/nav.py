"""RobotNav — 로봇 위치(map TF) 읽기 · initialpose · Nav2 goal 한 점 보내기. 지도 화면(MapPage)과 위치 지정 메뉴가 쓴다.

Bus가 node·_poll을 제공한다. docker 소스는 가짜 rclpy에 TF·액션이 없어서 파일 브리지/컨테이너 CLI로 돌린다.
"""
from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path

import rclpy


class RobotNav:
    def __init__(self, bus):
        self.bus = bus
        self.node = bus.node
        self._nav_cli = None
        self._pub_reset = None
        self._tf_buf = None  # map → base_footprint 조회용
        self._tf_listener = None

    def set_robot_pose(self, x: float, y: float, yaw_deg: float, frame_id: str = 'map') -> None:
        """로봇 위치를 지도 위 (x, y, yaw)로.
        faker는 /cmd/fake_opencr/set_pose 텔레포트, 실기는 /initialpose (cuVSLAM 등 위치추정 노드가 받는다)."""
        if self._is_sim():
            self._reset_sim_pose({'x': x, 'y': y, 'yaw_deg': yaw_deg}, frame_id)
        else:
            self._publish_initialpose(x, y, yaw_deg, frame_id)

    def _publish_initialpose(self, x: float, y: float, yaw_deg: float, frame_id: str) -> None:
        from geometry_msgs.msg import PoseWithCovarianceStamped
        yaw = math.radians(float(yaw_deg))
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        pub = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        try:
            for _ in range(3):
                pub.publish(msg)
                self.bus._poll(0.2)
            if pub.get_subscription_count() == 0:
                raise RuntimeError('/initialpose 구독자 없음 — cuVSLAM initialpose 지원(재시작) 확인')
        finally:
            self.node.destroy_publisher(pub)
        print(f'[initialpose] map ({x:+.2f}, {y:+.2f}, {yaw_deg:+.1f}°)')

    def robot_pose(self, frame_id: str = 'map') -> dict | None:
        """지금 로봇 자세 — 기다리지 않음 (지도 화면이 0.3초마다 부른다).
        docker는 호스트에 tf2_ros가 없으므로 fake_opencr의 map 프레임 ground_truth를 쓴다 (odom 아님)."""
        if os.environ.get('CLI_SOURCE') == 'docker' and frame_id == 'map':
            truth = self._docker_ground_truth()
            if truth is not None:
                return truth
        try:
            x, y, yaw = self._lookup_pose(frame_id)
        except Exception:
            return None
        return {'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(math.degrees(yaw), 1)}

    def _docker_ground_truth(self) -> dict | None:
        debug = getattr(self.node, 'debug_dir', None)
        if not debug:
            return None
        try:
            doc = json.loads((Path(debug) / 'output' / 'topic' / 'fake_opencr' / 'ground_truth' / 'message').read_text())
            pose = (doc.get('data') or {}).get('pose') or {}
            pos, q = pose.get('position') or {}, pose.get('orientation') or {}
            x, y = float(pos['x']), float(pos['y'])
            qx, qy, qz, qw = (float(q.get(k, 0.0)) for k in ('x', 'y', 'z', 'w'))
            if qw == 0.0 and qz == 0.0:
                qw = 1.0
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            return {'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(math.degrees(yaw), 1)}
        except (OSError, ValueError, KeyError, TypeError):
            return None

    def frame_pose(self, child: str, frame_id: str = 'map') -> tuple[float, float, float] | None:
        """map 기준 다른 프레임(odom 등)의 (x, y, yaw[deg]) — 기다리지 않음, 없으면 None. 지도 화면의 M·O 표시(위치 + 방향)용."""
        try:
            x, y, yaw = self._lookup_pose(frame_id, child)
        except Exception:
            return None
        return x, y, math.degrees(yaw)

    def _lookup_pose(self, frame_id: str, child: str = 'base_footprint') -> tuple[float, float, float]:
        """지금 map(frame_id) 기준 child(기본 base_footprint)의 (x, y, yaw[rad]) — TF가 없으면 예외."""
        from rclpy.time import Time
        if self._tf_buf is None:
            from tf2_ros import Buffer, TransformListener
            self._tf_buf = Buffer()
            self._tf_listener = TransformListener(self._tf_buf, self.node)
        tr = self._tf_buf.lookup_transform(frame_id, child, Time())
        t, q = tr.transform.translation, tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return float(t.x), float(t.y), yaw

    def _nav_client(self):
        if self._nav_cli is None:
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient
            self._nav_cli = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        return self._nav_cli

    def _reset_sim_pose(self, pose: dict, frame_id: str) -> None:
        """시뮬 로봇을 leg 시작점으로 텔레포트 — A가 맵 밖으로 나가도 B는 기획 가능.
        fake_opencr 있으면 /cmd/fake_opencr/set_pose(진짜 위치), mock이면
        /diff_drive_controller/set_odometry(odom). 둘 다 쏜다 — 각 모드에서 하나씩만 먹는다."""
        import math as _math
        from geometry_msgs.msg import PoseStamped
        yaw = _math.radians(float(pose.get('yaw_deg', 0.0)))
        if self._pub_reset is None:
            from std_msgs.msg import String as _S  # noqa — 임포트 순서 고정용 더미
            self._pub_reset = self.node.create_publisher(PoseStamped, '/cmd/fake_opencr/set_pose', 10)
        m = PoseStamped()
        m.header.frame_id = frame_id
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.pose.position.x = float(pose['x'])
        m.pose.position.y = float(pose['y'])
        m.pose.orientation.z = _math.sin(yaw / 2.0)
        m.pose.orientation.w = _math.cos(yaw / 2.0)
        for _ in range(3):
            self._pub_reset.publish(m)
            self.bus._poll(0.2)
        try:
            from control_msgs.srv import SetOdometry
            cli = self.node.create_client(SetOdometry, '/diff_drive_controller/set_odometry')
            if cli.wait_for_service(timeout_sec=2.0):
                req = SetOdometry.Request()
                req.x = float(pose['x'])
                req.y = float(pose['y'])
                req.yaw = yaw
                fut = cli.call_async(req)
                rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        except Exception:
            pass
        self.bus._poll(1.0)  # TF·EKF가 점프를 따라잡을 시간

    def send_nav_goal(self, pose: dict, frame_id: str = 'map') -> None:
        """Nav2 NavigateToPose 한 점을 보내고 바로 돌아온다 — 지도 화면 goal 찍기용 (진행은 로봇이 한다).
        docker 소스는 가짜 rclpy에 nav2_msgs.action이 없으므로 컨테이너의 ros2 CLI로 보낸다."""
        if os.environ.get('CLI_SOURCE') == 'docker':
            self._send_nav_goal_docker(pose, frame_id)
            return
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateToPose

        cli = self._nav_client()
        if not cli.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('/navigate_to_pose 액션 서버 없음 — 지도/로컬라이제이션/Nav2 확인')
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pose['x'])
        goal.pose.pose.position.y = float(pose['y'])
        yaw = math.radians(float(pose.get('yaw_deg', 0.0)))
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        send = cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=5.0)
        if not send.done():
            raise RuntimeError('이동 목표: goal 전송 timeout')
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('이동 목표: goal 거부됨')

    def _send_nav_goal_docker(self, pose: dict, frame_id: str) -> None:
        """docker: debug/input/action/navigate_to_pose/goal 파일 브리지 → 컨테이너 service_action_bag_input → 진짜 Nav2.
        (rclpy action을 호스트에 흉내내지 않는다 — 컨테이너가 이미 액션을 받아 준다.)"""
        debug = getattr(self.node, 'debug_dir', None) or os.environ.get('CLI_DEBUG_DIR')
        if not debug:
            raise RuntimeError('docker debug 경로를 모름 — --debug-dir 확인')
        debug = Path(debug)
        name = 'navigate_to_pose'
        folder = debug / 'input' / 'action' / name
        folder.mkdir(parents=True, exist_ok=True)
        goal_id = f'cli-{os.getpid()}-{int(time.time() * 1000)}'
        yaw = math.radians(float(pose.get('yaw_deg', 0.0)))
        payload = {'goal_id': goal_id, 'data': {'pose': {
            'position': {'x': float(pose['x']), 'y': float(pose['y']), 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0,
                            'z': math.sin(yaw / 2.0), 'w': math.cos(yaw / 2.0)},
        }}}
        goal = folder / 'goal'
        tmp = folder / f'.goal-{os.getpid()}-{int(time.time() * 1000)}'
        tmp.write_text(json.dumps(payload, ensure_ascii=False))
        os.replace(tmp, goal)
        accept_path = debug / 'output' / 'action' / name / 'accept'
        end = time.monotonic() + 5.0
        while time.monotonic() < end and rclpy.ok():
            try:
                doc = json.loads(accept_path.read_text())
            except (OSError, ValueError):
                time.sleep(0.1)
                continue
            if str(doc.get('goal_id', '')) == goal_id:
                if not doc.get('accepted'):
                    raise RuntimeError(f'goal 거부: {doc.get("error") or "이유 없음"}')
                print(f'[docker] Nav2 goal 전송 → ({pose["x"]:.2f}, {pose["y"]:.2f}, {pose.get("yaw_deg", 0):.0f}°)')
                return
            time.sleep(0.1)
        raise RuntimeError('action 브리지 응답 없음 — service_action_bag_input/Nav2 확인')

    def _is_sim(self) -> bool:
        """테스트베드(faker)인가 — fake_opencr가 /cmd/fake_opencr/set_pose를 듣고 있으면 텔레포트가 된다."""
        try:
            if self.node.count_subscribers('/cmd/fake_opencr/set_pose') > 0:
                return True
        except Exception:
            pass
        try:
            return any(n == 'fake_opencr' for n, _ in self.node.get_node_names_and_namespaces())
        except Exception:
            return False
