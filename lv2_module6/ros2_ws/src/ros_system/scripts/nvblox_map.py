#!/usr/bin/env python3
"""nvblox_map — nvblox 2D 점유 격자를 /map으로 + 저장 지도 불러오기

  /map은 latched(reliable + transient_local)여야 Nav2 global_costmap static_layer(map_subscribe_transient_local) ·
  explorer · state_manager · RViz · 시뮬레이터가 그대로 받는다. nvblox ~/static_occupancy_grid는 주기적으로 나오는 보통 토픽이라
  QoS가 안 맞아 직접은 못 받음 → 마지막 격자를 rate_hz로 /map에 다시 냄 (새 격자가 왔을 때만 — static_layer가 매번 다시 그리지 않게)
  load_path가 있으면(map:=saved) 시작할 때 /nvblox_node/load_map으로 불러옴. 저장은 nvblox after_shutdown_map_save_path(끌 때 자동)
  저장 지도의 좌표 = 그때 출발 자리 — 지금 출발 자리가 같아야 맞음 (운영 규칙: 출발 구역 같은 자리·같은 방향에서 켬)
"""
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data


class NvbloxMap(Node):
    def __init__(self):
        super().__init__('nvblox_map')
        self.frame = self.declare_parameter('map_frame', 'map').value
        self.load_path = self.declare_parameter('load_path', '').value
        rate = float(self.declare_parameter('rate_hz', 1.0).value)
        self.grid = None
        self.fresh = False
        self.n_in = self.n_out = 0
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(OccupancyGrid, self.declare_parameter('map_out', '/map').value, latched)
        # best effort로 구독 — 발행 쪽이 reliable이든 best effort든 받음
        self.create_subscription(OccupancyGrid, self.declare_parameter('grid_in', '/nvblox_node/static_occupancy_grid').value,
                                 self.on_grid, qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self.tick)
        self.create_timer(10.0, lambda: self.get_logger().info(
            f'격자 받음 {self.n_in} · /map 냄 {self.n_out}' + ('' if self.grid else ' — nvblox 격자 아직 없음')))
        if self.load_path:
            self.load()

    def on_grid(self, m):
        if m.header.frame_id and m.header.frame_id != self.frame:
            self.get_logger().error(f'격자 프레임 {m.header.frame_id} ≠ {self.frame} — nvblox global_frame 확인',
                                    throttle_duration_sec=10.0)
            return
        self.grid = m
        self.fresh = True
        self.n_in += 1

    def tick(self):
        if not self.fresh:
            return
        self.fresh = False
        self.grid.header.frame_id = self.frame
        self.pub.publish(self.grid)
        self.n_out += 1

    def load(self):
        from nvblox_msgs.srv import FilePath  # nvblox 설치된 곳에서만 (launch check_slam이 확인)
        cli = self.create_client(FilePath, '/nvblox_node/load_map')
        if not cli.wait_for_service(timeout_sec=60.0):
            self.get_logger().error(f'nvblox load_map 서비스 없음 — {self.load_path} 못 불러옴 (빈 지도로 시작)')
            return
        req = FilePath.Request()
        req.file_path = self.load_path
        cli.call_async(req).add_done_callback(lambda f: self.get_logger().info(
            f'지도 불러오기 {self.load_path}: {"성공" if f.result() and f.result().success else "실패 — 빈 지도로 시작"}'))


def main():
    rclpy.init()
    node = NvbloxMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
