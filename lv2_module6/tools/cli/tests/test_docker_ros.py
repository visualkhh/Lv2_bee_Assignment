#!/usr/bin/env python3
"""DockerRos 단위 시험 — ROS·도커 없이: python3 tools/cli/tests/test_docker_ros.py"""
import json
import shutil
import struct
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))   # tools/cli

root = Path(tempfile.mkdtemp(prefix='debug-ros-'))


def write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)


def write_grid(path: Path, width: int, height: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    head = json.dumps({'frame': 'map', 'stamp': 1.0, 'resolution': 0.05,
                       'width': width, 'height': height, 'origin': [-0.1, -0.2, 0.0]})
    raw = bytes(v & 0xFF for v in (0, 100, -1, 0))
    path.write_bytes(head.encode() + b'\n' + raw)


try:
    write(root / 'output' / 'topic' / 'state' / 'operation_mode' / 'message',
          '{"type": "std_msgs/msg/String", "data": {"data": "AUTO"}}')
    write(root / 'output' / 'topic' / 'joint_states' / 'message',
          '{"type": "sensor_msgs/msg/JointState", "data": '
          '{"header": {"stamp": {"sec": 1, "nanosec": 2}, "frame_id": "base_link"}, '
          '"name": ["joint1"], "position": [0.5], "velocity": [0.1], "effort": [0.2]}}')
    write(root / 'output' / 'topic' / 'diagnostics_agg' / 'message',
          '{"type": "diagnostic_msgs/msg/DiagnosticArray", "data": {"status": '
          '[{"level": 1, "name": "테스트", "message": "주의", '
          '"values": [{"key": "k", "value": "v"}]}]}}')
    write_grid(root / 'output' / 'topic' / 'map' / 'data', 2, 2)

    write(root / 'topic' / 'cmd' / 'operation_mode' / 'info',
          'Type: std_msgs/msg/String\nPublisher count: 0\nSubscription count: 1\n'
          '  - /state_manager (reliable, volatile)\n')
    write(root / 'topic' / 'joint_states' / 'info',
          'Type: sensor_msgs/msg/JointState\nPublisher count: 1\n'
          '  - /joint_state_broadcaster (reliable, volatile)\n'
          'Subscription count: 2\n  - /operator_tool (reliable, volatile)\n')
    write(root / 'node' / 'list', '/state_manager\n/joint_state_broadcaster\n')
    write(root / 'node' / 'state_manager' / 'params',
          'map_dir: /ws/map\nmap_mode: MAPPING\nmap_name: \n'
          'use_sim_time: false\nsome_list: [1, 2, 3]\n')
    (root / 'input' / 'service' / 'map_saver' / 'save_map').mkdir(parents=True)

    from ros.docker.DockerRos import DockerRos
    DockerRos.install(root)

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter_client import AsyncParameterClient
    from std_msgs.msg import Bool, Empty, String
    from sensor_msgs.msg import JointState
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import TwistStamped
    from nav2_msgs.srv import SaveMap

    node = Node('test')
    assert node.get_name() == 'test'

    # 구독 — 첫 폴링에 현재 파일을 준다
    got = []
    node.create_subscription(String, '/state/operation_mode', lambda m: got.append(m.data), 10)
    joints = []
    node.create_subscription(JointState, '/joint_states',
                             lambda m: joints.append((m.name, m.position)), 10)
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.01)
        if got and joints:
            break
    assert got == ['AUTO'], got
    assert joints == [(['joint1'], [0.5])], joints

    # 그래프
    assert 'state_manager' in node.get_node_names(), node.get_node_names()
    assert ('state_manager', '/') in node.get_node_names_and_namespaces()
    infos = node.get_publishers_info_by_topic('/joint_states')
    assert [i.node_name for i in infos] == ['joint_state_broadcaster'], infos

    # 발행 — input/topic 파일
    pub = node.create_publisher(String, '/cmd/operation_mode', 10)
    assert pub.get_subscription_count() == 1
    pub.publish(String(data='MANUAL'))
    doc = json.loads((root / 'input' / 'topic' / 'cmd' / 'operation_mode' / 'message').read_text())
    assert doc == {'type': 'std_msgs/msg/String', 'data': {'data': 'MANUAL'}}, doc

    node.create_publisher(Empty, '/cmd/manual/reboot_motors', 10).publish(Empty())
    doc = json.loads((root / 'input' / 'topic' / 'cmd' / 'manual' / 'reboot_motors' / 'message').read_text())
    assert doc == {'type': 'std_msgs/msg/Empty', 'data': {}}, doc

    tw = TwistStamped()
    tw.header.stamp = node.get_clock().now().to_msg()
    tw.header.frame_id = 'base_footprint'
    tw.twist.linear.x = 0.15
    tw.twist.angular.z = 0.8
    node.create_publisher(TwistStamped, '/cmd/manual/cmd_vel', 10).publish(tw)
    doc = json.loads((root / 'input' / 'topic' / 'cmd' / 'manual' / 'cmd_vel' / 'message').read_text())
    assert doc['type'] == 'geometry_msgs/msg/TwistStamped'
    assert doc['data']['header']['frame_id'] == 'base_footprint'
    assert doc['data']['header']['stamp']['sec'] > 0
    assert abs(doc['data']['twist']['linear']['x'] - 0.15) < 1e-9

    # 지도 — output/topic/map/data (int8)
    maps = []
    node.create_subscription(OccupancyGrid, '/map', maps.append, 10)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.01)
        if maps:
            break
    assert maps, 'map 콜백 없음'
    assert (maps[0].info.width, maps[0].info.height) == (2, 2)
    assert maps[0].data == [0, 100, -1, 0], maps[0].data

    # 파라미터 — debug/node/<FQN>/params
    pc = AsyncParameterClient(node, '/state_manager')
    assert pc.wait_for_services(0.5)
    names = pc.list_parameters().result().result.names
    assert 'map_dir' in names and 'some_list' in names, names
    vals = pc.get_parameters(['map_dir', 'use_sim_time', 'some_list']).result().values
    descs = pc.describe_parameters(['map_dir', 'use_sim_time', 'some_list']).result().descriptors
    assert [d.type for d in descs] == [4, 1, 7], [d.type for d in descs]
    assert vals[0].string_value == '/ws/map' and vals[1].bool_value is False
    assert list(vals[2].integer_array_value) == [1, 2, 3]
    assert pc.set_parameters([object()]).result().results[0].successful is False

    # 서비스 — 요청 파일 → 응답 파일
    cli = node.create_client(SaveMap, '/map_saver/save_map')
    assert cli.wait_for_service(0.5)
    fut = cli.call_async(SaveMap.Request(map_url='/ws/map/default/map'))
    req = json.loads((root / 'input' / 'service' / 'map_saver' / 'save_map' / 'message').read_text())
    assert req['data']['map_url'] == '/ws/map/default/map', req
    assert not fut.done()
    write(root / 'output' / 'service' / 'map_saver' / 'save_map' / 'message',
          json.dumps({'request_id': req['request_id'], 'ok': True, 'data': {'result': True}}))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=1.0)
    assert fut.done() and fut.result().result is True, fut.result()

    # 실패 응답 — 예외 대신 ok=False/error를 결과로 (CLI가 거부 문구를 보여 준다)
    fut2 = cli.call_async(SaveMap.Request())
    req2 = json.loads((root / 'input' / 'service' / 'map_saver' / 'save_map' / 'message').read_text())
    write(root / 'output' / 'service' / 'map_saver' / 'save_map' / 'message',
          json.dumps({'request_id': req2['request_id'], 'ok': False, 'error': '지원 안 함'}))
    assert fut2.done() and fut2.result().ok is False
    assert fut2.result().error == '지원 안 함'

    print('PASS test_docker_ros')
finally:
    shutil.rmtree(root, ignore_errors=True)
