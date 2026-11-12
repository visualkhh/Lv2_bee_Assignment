"""RobotNav — 로봇 위치(map TF) 읽기 · initialpose · Nav2 goal 보내기. 지도 화면(MapPage)과 위치 지정 메뉴가 쓴다.

Bus가 node·_poll을 제공한다. docker 소스는 가짜 rclpy에 TF·액션이 없어서 파일 브리지/컨테이너 CLI로 돌린다.
웨이포인트 여러 개는 NavigateToPose를 순서대로 보낸다 — 각 구간 경로는 Nav2 planner가 채운다.
"""
from __future__ import annotations

import json
import math
import os
import subprocess
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
        self._feedback = None             # 실기 NavigateToPose feedback 최신
        self._feedback_goal_id = None     # docker 파일 브리지 goal_id
        self._seq: list[dict] = []        # 보낼 웨이포인트 목록
        self._seq_i = 0                   # 지금 보낸 웨이포인트 index
        self._seq_through = False         # 2개 이상이면 NavigateThroughPoses 한 번에
        self._seq_remaining = None        # feedback의 number_of_poses_remaining
        self._seq_state = 'idle'          # idle | running | done | failed | canceled
        self._seq_error = ''
        self._seq_frame = 'map'
        self._seq_goal_id = None          # 지금 goal의 id (docker 파일 비교용)
        self._seq_handle = None           # 실기 action goal handle
        self._seq_result_fut = None
        self._through_cli = None          # 실기 NavigateThroughPoses action client
        self._last_result_poll = 0.0      # docker result 파일 확인 간격 (tick)

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

    def _docker_odom_pose(self) -> tuple[float, float, float] | None:
        """odom 프레임 기준 base_footprint (x, y, yaw[rad]) — EKF(/odometry/filtered) 우선, 없으면 바퀴 /odom."""
        debug = getattr(self.node, 'debug_dir', None)
        if not debug:
            return None
        for name in ('odometry/filtered', 'odom'):
            try:
                doc = json.loads((Path(debug) / 'output' / 'topic' / name / 'message').read_text())
                data = doc.get('data') or {}
                pose = data.get('pose') or {}
                if 'pose' in pose:  # nav_msgs/Odometry: data.pose.pose
                    pose = pose['pose']
                pos, q = pose.get('position') or {}, pose.get('orientation') or {}
                x, y = float(pos['x']), float(pos['y'])
                qx, qy, qz, qw = (float(q.get(k, 0.0)) for k in ('x', 'y', 'z', 'w'))
                if qw == 0.0 and qz == 0.0:
                    qw = 1.0
                yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
                return x, y, yaw
            except (OSError, ValueError, KeyError, TypeError):
                continue
        return None

    def _docker_frame_pose(self, child: str) -> tuple[float, float, float] | None:
        """docker에서 map 기준 odom 원점 — T_map_odom = T_map_base · T_odom_base⁻¹.
        호스트 rclpy에는 TF가 없지만 ground_truth(map→base)와 EKF(odom→base) 파일로 같은 값을 계산한다."""
        if child != 'odom':
            return None
        truth, odom = self._docker_ground_truth(), self._docker_odom_pose()
        if truth is None or odom is None:
            return None
        mx, my = truth['x'], truth['y']
        myaw = math.radians(truth['yaw_deg'])
        ox, oy, oyaw = odom
        # T_odom_base⁻¹ 의 평행이동 = -R(odom_yaw)ᵀ · t
        ix = -(math.cos(oyaw) * ox + math.sin(oyaw) * oy)
        iy = -(-math.sin(oyaw) * ox + math.cos(oyaw) * oy)
        # map 프레임으로 회전·이동
        x = mx + math.cos(myaw) * ix - math.sin(myaw) * iy
        y = my + math.sin(myaw) * ix + math.cos(myaw) * iy
        yaw = math.atan2(math.sin(myaw - oyaw), math.cos(myaw - oyaw))
        return x, y, math.degrees(yaw)

    def frame_pose(self, child: str, frame_id: str = 'map') -> tuple[float, float, float] | None:
        """map 기준 다른 프레임(odom 등)의 (x, y, yaw[deg]) — 기다리지 않음, 없으면 None. 지도 화면의 M·O 표시(위치 + 방향)용."""
        if os.environ.get('CLI_SOURCE') == 'docker':
            p = self._docker_frame_pose(child)
            if p is not None:
                return p
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
        if os.environ.get('CLI_SOURCE') == 'docker':
            self._docker_reset_odom(float(pose['x']), float(pose['y']), yaw)
        else:
            self._reset_odom(float(pose['x']), float(pose['y']), yaw)
        self.bus._poll(1.0)  # TF·EKF가 점프를 따라잡을 시간

    def _reset_odom(self, x: float, y: float, yaw: float) -> None:
        """odom(바퀴)과 EKF 상태를 같은 자세로 — 텔레포트는 바퀴가 안 굴러서 odom/EKF가 옛 자리에 남는다.
        안 옮기면 local costmap(odom)이 옛 자리 기준으로 그려져 지도 위 로봇과 어긋난다."""
        from control_msgs.srv import SetOdometry
        from robot_localization.srv import SetPose

        cli = self.node.create_client(SetOdometry, '/diff_drive_controller/set_odometry')
        if cli.wait_for_service(timeout_sec=2.0):
            req = SetOdometry.Request()
            req.x, req.y, req.yaw = float(x), float(y), float(yaw)
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)

        pose_cli = self.node.create_client(SetPose, '/set_pose')
        if pose_cli.wait_for_service(timeout_sec=2.0):
            req = SetPose.Request()
            req.pose.header.frame_id = 'odom'
            req.pose.pose.pose.position.x = float(x)
            req.pose.pose.pose.position.y = float(y)
            req.pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
            req.pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
            req.pose.pose.covariance[0] = req.pose.pose.covariance[7] = req.pose.pose.covariance[35] = 0.01
            fut = pose_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)

    def _docker_reset_odom(self, x: float, y: float, yaw: float) -> None:
        """도커: 컨테이너 안에서 odom·EKF 리셋 서비스를 호출한다 (호스트 CLI는 서비스 타입 플러그인을 못 씀)."""
        import shlex
        container = os.environ.get('BEEBOT_CONTAINER', 'lv2_module6')
        cov = [0.01 if i in (0, 7, 35) else 0.0 for i in range(36)]
        odom_goal = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 6)}
        pose_goal = {'pose': {
            'header': {'frame_id': 'odom'},
            'pose': {'pose': {'position': {'x': round(x, 3), 'y': round(y, 3), 'z': 0.0},
                              'orientation': {'x': 0.0, 'y': 0.0,
                                              'z': round(math.sin(yaw / 2.0), 6),
                                              'w': round(math.cos(yaw / 2.0), 6)}},
                     'covariance': cov}}}
        script = (
            'source /opt/ros/lyrical/setup.bash && '
            f'ros2 service call /diff_drive_controller/set_odometry control_msgs/srv/SetOdometry '
            f'{shlex.quote(json.dumps(odom_goal))} >/dev/null && '
            f'ros2 service call /set_pose robot_localization/srv/SetPose {shlex.quote(json.dumps(pose_goal))} >/dev/null')
        try:
            subprocess.run(['docker', 'exec', container, 'bash', '-lc', script],
                           text=True, capture_output=True, timeout=20, check=False)
        except (OSError, subprocess.TimeoutExpired):
            pass  # 텔레포트 자체는 됐다 — 리셋 실패는 화면상 위치 어긋남으로만 보인다

    # ---------------------------------------------------------------- Nav2 goal · 웨이포인트 순차 주행
    def send_nav_goal(self, pose: dict, frame_id: str = 'map') -> None:
        """Nav2 NavigateToPose 한 점 — send_nav_waypoints([pose])와 같다."""
        self.send_nav_waypoints([pose], frame_id)

    def send_nav_waypoints(self, poses: list[dict], frame_id: str = 'map') -> None:
        """웨이포인트 여러 개 — 2개 이상이면 NavigateThroughPoses 한 번에 보낸다 (중간에 안 끊김).
        1개면 예전처럼 NavigateToPose. 각 구간 경로는 Nav2 planner가 채운다."""
        poses = [dict(p) for p in poses]
        if not poses:
            raise RuntimeError('웨이포인트 없음')
        if self._seq_state == 'running':
            raise RuntimeError('이미 Nav2 주행 중 — 먼저 취소(x)하세요')
        self._seq, self._seq_frame = poses, frame_id
        self._seq_through = len(poses) > 1
        self._seq_i = 0 if not self._seq_through else len(poses) - 1
        self._seq_remaining = len(poses) if self._seq_through else None
        self._seq_state, self._seq_error = 'running', ''
        self._seq_goal_id = None
        self._seq_handle = None
        self._seq_result_fut = None
        self._feedback = None
        self._feedback_goal_id = None
        self._seq_send_current()
        if self._seq_state == 'failed':
            raise RuntimeError(self._seq_error)

    def _seq_send_current(self) -> None:
        self._feedback = None
        self._feedback_goal_id = None
        if self._seq_through:
            try:
                if os.environ.get('CLI_SOURCE') == 'docker':
                    self._seq_goal_id = self._send_nav_through_docker(self._seq, self._seq_frame)
                    self._feedback_goal_id = self._seq_goal_id
                else:
                    self._seq_send_real_through(self._seq)
            except Exception as exc:
                self._seq_state = 'failed'
                self._seq_error = str(exc)
            return
        if self._seq_i >= len(self._seq):
            self._seq_state = 'done'
            return
        try:
            if os.environ.get('CLI_SOURCE') == 'docker':
                self._seq_goal_id = self._send_nav_goal_docker(self._seq[self._seq_i], self._seq_frame)
                self._feedback_goal_id = self._seq_goal_id
            else:
                self._seq_send_real(self._seq[self._seq_i])
        except Exception as exc:
            self._seq_state = 'failed'
            self._seq_error = str(exc)

    def _through_client(self):
        if self._through_cli is None:
            from nav2_msgs.action import NavigateThroughPoses
            from rclpy.action import ActionClient
            self._through_cli = ActionClient(self.node, NavigateThroughPoses, '/navigate_through_poses')
        return self._through_cli

    def _seq_send_real_through(self, poses: list[dict]) -> None:
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateThroughPoses

        cli = self._through_client()
        if not cli.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('/navigate_through_poses 액션 서버 없음 — bt_navigator 재시작/설정 확인')
        goal = NavigateThroughPoses.Goal()
        goal.poses.header.frame_id = self._seq_frame
        goal.poses.header.stamp = self.node.get_clock().now().to_msg()
        for pose in poses:
            ps = PoseStamped()
            ps.header.frame_id = self._seq_frame
            ps.header.stamp = goal.poses.header.stamp
            ps.pose.position.x = float(pose['x'])
            ps.pose.position.y = float(pose['y'])
            yaw = math.radians(float(pose.get('yaw_deg', 0.0)))
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            goal.poses.goals.append(ps)
        gid = f'cli-{os.getpid()}-{int(time.time() * 1000)}-through'
        self._seq_goal_id = gid
        self._seq_handle = None
        self._seq_result_fut = None
        self._seq_remaining = len(poses)
        send = cli.send_goal_async(goal, feedback_callback=lambda m, g=gid: self._on_feedback(m, g))
        send.add_done_callback(lambda f, g=gid: self._on_seq_response(f, g))

    def _seq_send_real(self, pose: dict) -> None:
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateToPose

        cli = self._nav_client()
        if not cli.wait_for_server(timeout_sec=5.0 if self._seq_i == 0 else 0.5):
            raise RuntimeError('/navigate_to_pose 액션 서버 없음 — 지도/로컬라이제이션/Nav2 확인')
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._seq_frame
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pose['x'])
        goal.pose.pose.position.y = float(pose['y'])
        yaw = math.radians(float(pose.get('yaw_deg', 0.0)))
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        gid = f'cli-{os.getpid()}-{int(time.time() * 1000)}-{self._seq_i}'
        self._seq_goal_id = gid
        self._seq_handle = None
        self._seq_result_fut = None
        send = cli.send_goal_async(goal, feedback_callback=lambda m, g=gid: self._on_feedback(m, g))
        send.add_done_callback(lambda f, g=gid: self._on_seq_response(f, g))

    def _on_seq_response(self, future, goal_id: str) -> None:
        """goal 수락 여부 — 실기 rclpy 콜백 (UI 폴링이 spin한다)."""
        if goal_id != self._seq_goal_id or self._seq_state != 'running':
            return
        try:
            handle = future.result()
        except Exception as exc:
            self._seq_fail(f'goal 전송 실패: {exc}')
            return
        if handle is None or not handle.accepted:
            self._seq_fail('goal 거부됨')
            return
        self._seq_handle = handle
        self._seq_result_fut = handle.get_result_async()
        self._seq_result_fut.add_done_callback(lambda f, g=goal_id: self._on_seq_result(f, g))

    def _on_seq_result(self, future, goal_id: str) -> None:
        if goal_id != self._seq_goal_id or self._seq_state != 'running':
            return
        try:
            from action_msgs.msg import GoalStatus
            wrapped = future.result()
            ok = wrapped.status == GoalStatus.STATUS_SUCCEEDED
            error = getattr(getattr(wrapped, 'result', None), 'error_msg', '') or f'status={wrapped.status}'
        except Exception as exc:
            self._seq_fail(f'결과 수신 실패: {exc}')
            return
        if ok:
            if self._seq_through:
                self._seq_state = 'done'
                self._seq_remaining = 0
            else:
                self._seq_advance()
        else:
            self._seq_fail(error)

    def _seq_advance(self) -> None:
        """현재 goal 성공 — 다음 웨이포인트를 보낸다. 마지막이면 done."""
        self._seq_i += 1
        if self._seq_i >= len(self._seq):
            self._seq_state = 'done'
            return
        self._seq_send_current()

    def _seq_fail(self, reason: str) -> None:
        self._seq_state = 'failed'
        self._seq_error = reason

    def cancel_nav(self) -> bool:
        """주행 중인 웨이포인트 취소 — docker는 cancel 파일, 실기는 action cancel."""
        if self._seq_state != 'running':
            return False
        self._seq_state = 'canceled'
        if os.environ.get('CLI_SOURCE') == 'docker':
            debug = getattr(self.node, 'debug_dir', None)
            if debug and self._seq_goal_id:
                name = 'navigate_through_poses' if self._seq_through else 'navigate_to_pose'
                folder = Path(debug) / 'input' / 'action' / name
                folder.mkdir(parents=True, exist_ok=True)
                tmp = folder / f'.cancel-{os.getpid()}-{int(time.time() * 1000)}'
                tmp.write_text(json.dumps({'goal_id': self._seq_goal_id}))
                os.replace(tmp, folder / 'cancel')
        elif self._seq_handle is not None:
            self._seq_handle.cancel_goal_async()
        return True

    def _poll_docker_result(self) -> None:
        """docker result 파일 — 단일 goal이면 다음 웨이포인트를 보내고, through-poses면 끝났는지 본다."""
        if self._seq_state != 'running' or not self._seq_goal_id:
            return
        debug = getattr(self.node, 'debug_dir', None)
        if not debug:
            return
        name = 'navigate_through_poses' if self._seq_through else 'navigate_to_pose'
        try:
            doc = json.loads((Path(debug) / 'output' / 'action' / name / 'result').read_text())
        except (OSError, ValueError, TypeError):
            return
        if str(doc.get('goal_id', '')) != str(self._seq_goal_id):
            return
        status = str(doc.get('status', ''))
        if status == 'succeeded':
            if self._seq_through:
                self._seq_state = 'done'
                self._seq_remaining = 0
            else:
                self._seq_advance()
        elif status in ('aborted', 'canceled', 'unknown_goal'):
            data = doc.get('data') or {}
            self._seq_fail(data.get('error_msg') or status)

    def tick(self) -> None:
        """이벤트 루프가 자주 부른다 — docker는 result 파일을 0.1초 간격으로 확인해 다음 웨이포인트를 바로 보낸다.
        지도 화면 redraw(2초)만 기다리면 웨이포인트마다 2초씩 멈춘다."""
        if self._seq_state != 'running' or os.environ.get('CLI_SOURCE') != 'docker':
            return
        now = time.monotonic()
        if now - self._last_result_poll < 0.1:
            return
        self._last_result_poll = now
        self._poll_docker_result()

    def preview_path(self, poses: list[dict], frame_id: str = 'map') -> list[tuple[float, float]]:
        """이동 없이 planner에 경로만 계산 — [(x, y), …]. 1개면 ComputePathToPose · 여러 개면 ComputePathThroughPoses."""
        if not poses:
            raise RuntimeError('웨이포인트 없음')
        if os.environ.get('CLI_SOURCE') == 'docker':
            return self._preview_path_docker(poses, frame_id)
        return self._preview_path_real(poses, frame_id)

    def _preview_path_real(self, poses: list[dict], frame_id: str) -> list[tuple[float, float]]:
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
        from rclpy.action import ActionClient

        stamp = self.node.get_clock().now().to_msg()
        def ps(p):
            m = PoseStamped()
            m.header.frame_id = frame_id
            m.header.stamp = stamp
            m.pose.position.x, m.pose.position.y = float(p['x']), float(p['y'])
            yaw = math.radians(float(p.get('yaw_deg', 0.0)))
            m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
            return m

        if len(poses) > 1:
            cli = ActionClient(self.node, ComputePathThroughPoses, '/compute_path_through_poses')
            if not cli.wait_for_server(timeout_sec=5.0):
                raise RuntimeError('/compute_path_through_poses 서버 없음 — planner_server 확인')
            goal = ComputePathThroughPoses.Goal()
            goal.goals.header.frame_id = frame_id
            goal.goals.header.stamp = stamp
            goal.goals.goals = [ps(p) for p in poses]
            goal.use_start = False
        else:
            cli = ActionClient(self.node, ComputePathToPose, '/compute_path_to_pose')
            if not cli.wait_for_server(timeout_sec=5.0):
                raise RuntimeError('/compute_path_to_pose 서버 없음 — planner_server 확인')
            goal = ComputePathToPose.Goal()
            goal.goal = ps(poses[0])
            goal.use_start = False

        send = cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=5.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('planner가 goal을 거부')
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=15.0)
        out = res.result()
        if out is None:
            raise RuntimeError('planner 응답 없음')
        return [(p.pose.position.x, p.pose.position.y) for p in out.result.path.poses]

    def _preview_path_docker(self, poses: list[dict], frame_id: str) -> list[tuple[float, float]]:
        """도커: 컨테이너 안에서 ros2 action send_goal → 결과 YAML의 path를 읽는다 (로봇은 안 움직임)."""
        container = os.environ.get('BEEBOT_CONTAINER', 'lv2_module6')
        stamp = {'sec': 0, 'nanosec': 0}
        pss = [{'header': {'stamp': stamp, 'frame_id': frame_id},
                'pose': {'position': {'x': float(p['x']), 'y': float(p['y']), 'z': 0.0},
                         'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}}
               for p in poses]
        if len(poses) > 1:
            action, typ = '/compute_path_through_poses', 'nav2_msgs/action/ComputePathThroughPoses'
            goal = {'goals': {'header': {'stamp': stamp, 'frame_id': frame_id}, 'goals': pss}, 'use_start': False}
        else:
            action, typ = '/compute_path_to_pose', 'nav2_msgs/action/ComputePathToPose'
            goal = {'goal': pss[0], 'use_start': False}
        import shlex
        script = ('source /opt/ros/lyrical/setup.bash && '
                  f'ros2 action send_goal {action} {typ} {shlex.quote(json.dumps(goal, separators=(",", ":")))}')
        try:
            run = subprocess.run(['docker', 'exec', container, 'bash', '-lc', script],
                                 text=True, capture_output=True, timeout=40)
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise RuntimeError(f'도커 미리보기 실패: {exc}') from exc
        if run.returncode != 0:
            err = (run.stderr or run.stdout or '').strip().splitlines()[-1:]
            raise RuntimeError('planner 미리보기 실패: ' + (err[0] if err else 'send_goal 오류'))
        return self._parse_path_yaml(run.stdout)

    @staticmethod
    def _parse_path_yaml(text: str) -> list[tuple[float, float]]:
        """`ros2 action send_goal` 결과에서 path.poses[].pose.position 만 읽는다.

        ros2cli가 path 아래 중첩 필드를 2칸으로 내어 유효한 YAML이 아니다 (path: 4칸 · header: 2칸) —
        Result: 이후의 position 블록을 줄 단위로 훑어 x·y만 모은다 (orientation은 position 밖이라 무시)."""
        cut = text.find('Result:')
        body = text[cut + len('Result:'):] if cut >= 0 else text
        pts: list[tuple[float, float]] = []
        in_pos, x = False, None
        for line in body.splitlines():
            s = line.strip()
            if s == 'position:':
                in_pos, x = True, None
                continue
            if not in_pos:
                continue
            if s.startswith('x:'):
                x = float(s.split(':', 1)[1])
            elif s.startswith('y:') and x is not None:
                pts.append((x, float(s.split(':', 1)[1])))
                in_pos, x = False, None
        return pts

    def nav_progress(self) -> dict | None:
        """웨이포인트 진행 상황 — index(1부터) · total · state · 회복/남은 거리. 보낸 적 없으면 None."""
        if not self._seq or self._seq_state == 'idle':
            return None
        if os.environ.get('CLI_SOURCE') == 'docker' and self._seq_state == 'running':
            self._poll_docker_result()
        fb = self._current_feedback() or {}
        total = len(self._seq)
        if self._seq_through:
            remaining = fb.get('poses_remaining')
            if remaining is not None:
                try:
                    self._seq_remaining = int(remaining)
                except (TypeError, ValueError):
                    pass
            if self._seq_state == 'done':
                index = total
            else:
                rem = self._seq_remaining if self._seq_remaining is not None else total
                index = max(1, min(total, total - int(rem) + 1))
        else:
            index = total if self._seq_state == 'done' else min(self._seq_i + 1, max(1, total))
        return {'state': self._seq_state, 'index': index, 'total': total,
                'poses_remaining': self._seq_remaining if self._seq_through else None,
                'recoveries': fb.get('recoveries'), 'distance_remaining': fb.get('distance_remaining'),
                'age': fb.get('age'), 'error': self._seq_error}

    def nav_feedback(self) -> dict | None:
        """최근 NavigateToPose feedback — 회복 횟수·남은 거리. (이전 호출부 호환)"""
        p = self.nav_progress()
        if p is None or p['state'] != 'running' or p.get('recoveries') is None:
            return None
        return {'recoveries': p['recoveries'], 'distance_remaining': p['distance_remaining'], 'age': p['age']}

    def _current_feedback(self) -> dict | None:
        if os.environ.get('CLI_SOURCE') == 'docker':
            debug = getattr(self.node, 'debug_dir', None)
            if not debug or not self._seq_goal_id:
                return None
            name = 'navigate_through_poses' if self._seq_through else 'navigate_to_pose'
            path = Path(debug) / 'output' / 'action' / name / 'feedback'
            try:
                doc = json.loads(path.read_text())
                if str(doc.get('goal_id', '')) != str(self._seq_goal_id):
                    return None
                data = doc.get('data') or {}
                remaining = data.get('poses_remaining')
                return {'recoveries': int(data.get('recoveries', 0)),
                        'distance_remaining': float(data.get('distance_remaining', 0.0)),
                        'poses_remaining': int(remaining) if remaining is not None else None,
                        'age': max(0.0, time.time() - path.stat().st_mtime)}
            except (OSError, ValueError, TypeError):
                return None
        fb = self._feedback
        if fb is None or fb.get('goal_id') != self._seq_goal_id:
            return None
        return {'recoveries': fb['recoveries'], 'distance_remaining': fb['distance_remaining'],
                'poses_remaining': fb.get('poses_remaining'),
                'age': max(0.0, time.monotonic() - fb.get('at', time.monotonic()))}

    def _on_feedback(self, msg, goal_id: str | None = None):
        if goal_id is not None and goal_id != self._seq_goal_id:
            return
        fb = getattr(msg, 'feedback', msg)
        remaining = getattr(fb, 'number_of_poses_remaining', None)
        self._feedback = {
            'goal_id': goal_id,
            'recoveries': int(getattr(fb, 'number_of_recoveries', 0) or 0),
            'distance_remaining': float(getattr(fb, 'distance_remaining', 0.0) or 0.0),
            'poses_remaining': int(remaining) if remaining is not None else None,
            'at': time.monotonic(),
        }

    def _docker_goal(self, name: str, goal_id: str, payload: dict) -> str:
        """docker 파일 브리지에 goal을 쓰고 accept를 기다린다. 보낸 goal_id를 돌려준다."""
        debug = getattr(self.node, 'debug_dir', None) or os.environ.get('CLI_DEBUG_DIR')
        if not debug:
            raise RuntimeError('docker debug 경로를 모름 — --debug-dir 확인')
        debug = Path(debug)
        folder = debug / 'input' / 'action' / name
        folder.mkdir(parents=True, exist_ok=True)
        goal = folder / 'goal'
        tmp = folder / f'.goal-{os.getpid()}-{int(time.time() * 1000)}'
        tmp.write_text(json.dumps(payload, ensure_ascii=False))
        os.replace(tmp, goal)
        accept_path = debug / 'output' / 'action' / name / 'accept'
        end = time.monotonic() + 20.0   # service_action_bag_input timeout_s와 같게 — 재시작 직후 액션 발견이 늦을 수 있다
        while time.monotonic() < end and rclpy.ok():
            try:
                doc = json.loads(accept_path.read_text())
            except (OSError, ValueError):
                time.sleep(0.1)
                continue
            if str(doc.get('goal_id', '')) == goal_id:
                if not doc.get('accepted'):
                    raise RuntimeError(f'goal 거부: {doc.get("error") or "이유 없음"}')
                return goal_id
            time.sleep(0.1)
        raise RuntimeError('action 브리지 응답 없음 — service_action_bag_input/Nav2 확인')

    def _send_nav_goal_docker(self, pose: dict, frame_id: str) -> str:
        """docker: debug/input/action/navigate_to_pose/goal 파일 브리지 → service_action_bag_input → 진짜 Nav2."""
        goal_id = f'cli-{os.getpid()}-{int(time.time() * 1000)}'
        self._feedback_goal_id = goal_id
        yaw = math.radians(float(pose.get('yaw_deg', 0.0)))
        payload = {'goal_id': goal_id, 'data': {'pose': {
            'position': {'x': float(pose['x']), 'y': float(pose['y']), 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0,
                            'z': math.sin(yaw / 2.0), 'w': math.cos(yaw / 2.0)},
        }}}
        return self._docker_goal('navigate_to_pose', goal_id, payload)

    def _send_nav_through_docker(self, poses: list[dict], frame_id: str) -> str:
        """docker: debug/input/action/navigate_through_poses/goal — 여러 점을 한 액션으로."""
        goal_id = f'cli-{os.getpid()}-{int(time.time() * 1000)}-through'
        self._feedback_goal_id = goal_id
        arr = []
        for pose in poses:
            yaw = math.radians(float(pose.get('yaw_deg', 0.0)))
            arr.append({
                'position': {'x': float(pose['x']), 'y': float(pose['y']), 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0,
                                'z': math.sin(yaw / 2.0), 'w': math.cos(yaw / 2.0)},
            })
        payload = {'goal_id': goal_id, 'data': {'frame_id': frame_id, 'poses': arr}}
        return self._docker_goal('navigate_through_poses', goal_id, payload)

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
