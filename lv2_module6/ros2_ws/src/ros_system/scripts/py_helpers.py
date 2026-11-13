#!/usr/bin/env python3
"""py_helpers — 가벼운 rclpy 노드 여럿을 한 프로세스에서 (rclpy 표준: 노드 여럿 + MultiThreadedExecutor)

  python 노드 하나 = 인터프리터 + rclpy + DDS 참가자 ≈ 115~130 MB PSS (젯슨 실측 11-14). 같이 띄우면 그 바탕을 한 번만 냄.
  py_helpers rewind pole_depth_gate nvblox_map   (인자 = 띄울 노드 이름 — ros_system.launch.py py_helpers_one_process)
  노드마다 기본 콜백 그룹(서로 배타)이라 한 노드 안의 콜백 순서는 따로 띄울 때와 같고, 노드끼리는 스레드를 나눠 씀.
  파라미터는 launch가 넘긴 것을 노드마다 자기가 선언한 이름만 읽음 (세 노드 파라미터 이름이 겹치지 않음).
"""
import sys

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.utilities import remove_ros_args

import nvblox_map
import rewind
import scan_cam_gate

NODES = {'rewind': rewind.Rewind,
         'nvblox_map': nvblox_map.NvbloxMap,
         'pole_depth_gate': lambda: scan_cam_gate.ScanCamGate('pole_depth_gate')}


def main():
    rclpy.init()
    names = remove_ros_args(sys.argv)[1:]
    nodes = [NODES[n]() for n in names]
    ex = MultiThreadedExecutor(num_threads=len(nodes))
    for n in nodes:
        ex.add_node(n)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
