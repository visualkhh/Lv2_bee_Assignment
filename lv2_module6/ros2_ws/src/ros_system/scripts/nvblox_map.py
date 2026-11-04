#!/usr/bin/env python3
"""nvblox_map — nvblox 2D 점유 격자를 /map으로 + 저장 지도 불러오기

  /map은 latched(reliable + transient_local)여야 Nav2 global_costmap static_layer(map_subscribe_transient_local) ·
  explorer · state_manager · RViz · 시뮬레이터가 그대로 받는다. nvblox ~/static_occupancy_grid는 주기적으로 나오는 보통 토픽이라
  QoS가 안 맞아 직접은 못 받음 → 마지막 격자를 rate_hz로 /map에 다시 냄 (새 격자가 왔을 때만 — static_layer가 매번 다시 그리지 않게)
  load_path가 있으면(map:=saved) 시작할 때 /nvblox_node/load_map으로 불러옴. 저장은 nvblox after_shutdown_map_save_path(끌 때 자동)
  pad_m: 격자를 출발 자리(map 원점) 둘레 ±pad_m까지 '모름'(-1)으로 넓혀 냄 — nvblox 격자는 카메라가 본 곳만이라 로봇이 그 밖에
    서면 Nav2가 '출발점이 지도 밖'(error 203)으로 경로를 못 만들고 탐사가 멈춤 (2026-11-04 Isaac 실측). 0 = 넓히지 않음
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
        self.pad_m = float(self.declare_parameter('pad_m', 7.0).value)  # 경기장 4.4 × 6.3 m — 출발 자리가 어디든 덮음
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
        self.pub.publish(pad(self.grid, self.pad_m) if self.pad_m > 0 else self.grid)
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


def pad(m, pad_m):
    """격자를 [-pad_m, pad_m]² (map 원점 기준)까지 넓힌 새 격자 — 원래 칸은 그대로, 새 칸은 -1 (모름). 원점은 칸에 맞춤"""
    r, w, h = m.info.resolution, m.info.width, m.info.height
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    x0, y0 = min(ox, -pad_m), min(oy, -pad_m)
    x0, y0 = ox - round((ox - x0) / r) * r, oy - round((oy - y0) / r) * r  # 원래 격자와 칸 경계가 맞게
    W = max(w + round((ox - x0) / r), round((max(ox + w * r, pad_m) - x0) / r))
    H = max(h + round((oy - y0) / r), round((max(oy + h * r, pad_m) - y0) / r))
    dx, dy = round((ox - x0) / r), round((oy - y0) / r)
    data = [-1] * (W * H)
    src = m.data
    for j in range(h):
        data[(j + dy) * W + dx:(j + dy) * W + dx + w] = src[j * w:(j + 1) * w]
    out = OccupancyGrid()
    out.header = m.header
    out.info = m.info
    out.info.width, out.info.height = W, H
    out.info.origin.position.x, out.info.origin.position.y = x0, y0
    out.data = data
    return out


def _check():
    m = OccupancyGrid()
    m.info.resolution, m.info.width, m.info.height = 0.5, 2, 1
    m.info.origin.position.x, m.info.origin.position.y = 1.0, 0.0
    m.data = [0, 100]
    o = pad(m, 2.0)
    assert (o.info.width, o.info.height, o.info.origin.position.x, o.info.origin.position.y) == (8, 8, -2.0, -2.0), o.info
    k = (0 - -2.0) / 0.5  # 원래 (1.0, 0.0) 칸 → 새 (6, 4)
    assert o.data[int(k) * 8 + 6] == 0 and o.data[int(k) * 8 + 7] == 100 and o.data.count(-1) == 62


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
