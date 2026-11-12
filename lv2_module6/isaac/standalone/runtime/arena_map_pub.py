#!/usr/bin/env python3
"""map_server 형식 지도(map.yaml + pgm)를 OccupancyGrid로 한 번 낸다 (latched) — isaac_system.launch.py가 /arena/static으로 띄움.

  python3 arena_map_pub.py <map.yaml> [토픽=/arena/static]

nav2 map_server 대신 쓰는 이유: map_server는 lifecycle 노드라 lifecycle_manager가 change_state 서비스로 켜야 하는데,
Isaac 기동 직후(노드 40여 개)에는 그 응답이 버려져(failed to send response … timeout) Configuring에서 멈췄다 (11-12 실측).
그러면 /arena/static이 안 나와 /map이 nvblox 격자(64x64)만 남고 박스 접근 목표가 전부 costmap 밖(outside bounds)이 됐다.
이 노드는 서비스 없이 transient_local로 내기만 하므로 늦게 뜬 nvblox_map도 받는다.
"""
import os
import sys

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from PIL import Image
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def load(path):
    """map.yaml → (cells int8 [행 0 = 지도 아래], resolution, origin). trinary: 점유 100 · 빈 곳 0 · 모름 -1."""
    with open(path) as f:
        y = yaml.safe_load(f)
    img = np.array(Image.open(os.path.join(os.path.dirname(path), y['image'])).convert('L'), np.float64)
    occ = img / 255.0 if y.get('negate', 0) else (255.0 - img) / 255.0
    cells = np.full(img.shape, -1, np.int8)
    cells[occ > y['occupied_thresh']] = 100
    cells[occ < y['free_thresh']] = 0
    return np.flipud(cells), float(y['resolution']), y['origin']


def main():
    path, topic = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else '/arena/static')
    cells, res, origin = load(path)
    rclpy.init()
    node = rclpy.create_node('arena_map_pub')
    pub = node.create_publisher(OccupancyGrid, topic, QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    m = OccupancyGrid()
    m.header.frame_id = 'map'
    m.header.stamp = node.get_clock().now().to_msg()
    m.info.resolution = res
    m.info.height, m.info.width = cells.shape
    m.info.origin.position.x, m.info.origin.position.y = float(origin[0]), float(origin[1])
    m.info.origin.orientation.w = 1.0
    m.data = cells.ravel().tolist()
    pub.publish(m)
    node.get_logger().info(f'{path} → {topic} ({m.info.width}x{m.info.height} @ {res} m, 점유 {int((cells == 100).sum())}칸)')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # launch가 끌 때 (SIGINT)
        pass


if __name__ == '__main__':
    if len(sys.argv) < 2:  # 자체 확인: 0 → 점유, 254 → 빈 곳, 128 → 모름 (map_server와 같은 문턱) · 영상 맨 위 줄 = 지도 맨 위(행 끝)
        import tempfile
        d = tempfile.mkdtemp()
        Image.fromarray(np.array([[0, 254], [128, 254]], np.uint8)).save(os.path.join(d, 'm.pgm'))
        with open(os.path.join(d, 'm.yaml'), 'w') as f:
            f.write('image: m.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        c, _, _ = load(os.path.join(d, 'm.yaml'))
        assert c.tolist() == [[-1, 0], [100, 0]], c.tolist()
        print('ok (자체 확인) — 쓰는 법: arena_map_pub.py <map.yaml> [토픽]')
        sys.exit(0)
    main()
