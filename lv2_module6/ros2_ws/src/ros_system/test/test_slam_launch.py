#!/usr/bin/env python3
"""SLAM(cuVSLAM + nvblox)이 어떤 노드·지도 파일 경로로 펼쳐지는지 — GPU 없는 테스트베드에서도 launch 구조만은 늘 검사

  ros_system.launch.py를 띄우지 않고, 인자를 넣은 LaunchContext에서 각 Node의 condition만 평가해 켜지는 노드 이름을 모은다.
  python3 test_slam_launch.py  (colcon test — CMakeLists ament_add_test)
"""
import os
import sys

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

LAUNCH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'launch', 'ros_system.launch.py')


def nodes(**args):
    """인자(문자열) → 켜지는 노드 이름 집합 (이름 없는 노드는 실행 파일 이름)"""
    import importlib.util
    spec = importlib.util.spec_from_file_location('ros_system_launch', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ctx = LaunchContext()
    on = set()
    for e in mod.generate_launch_description().entities:
        if isinstance(e, DeclareLaunchArgument):
            ctx.launch_configurations.setdefault(e.name, perform_substitutions(ctx, e.default_value))
            continue
        if isinstance(e, (OpaqueFunction, RegisterEventHandler)):
            continue
        ctx.launch_configurations.update(args)
        if isinstance(e, Node) and (e.condition is None or e.condition.evaluate(ctx)):
            name = e._Node__node_name or e.node_executable  # 실행 전엔 node_name 속성이 없음 — 넘긴 name (없으면 실행 파일 이름)
            on.add(name if isinstance(name, str) else perform_substitutions(ctx, name))
    return on


def params(node_name, **args):
    """켜지는 노드 하나의 dict 파라미터 (yaml 파일 경로는 빼고) — 인자에 따라 바뀌는 값 확인용"""
    import importlib.util
    from launch_ros.utilities import evaluate_parameters
    spec = importlib.util.spec_from_file_location('ros_system_launch', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ctx = LaunchContext()
    for e in mod.generate_launch_description().entities:
        if isinstance(e, DeclareLaunchArgument):
            ctx.launch_configurations.setdefault(e.name, perform_substitutions(ctx, e.default_value))
            continue
        ctx.launch_configurations.update(args)
        if isinstance(e, Node) and e._Node__node_name == node_name:
            out = {}
            for p in evaluate_parameters(ctx, e._Node__parameters):
                if isinstance(p, dict):
                    out.update(p)
            return out
    return {}


def main():
    base = {'use_cameras': 'false', 'use_ros2_control': 'false', 'use_nav2': 'false', 'use_twist_mux': 'false'}
    on = nodes(**base)
    off = nodes(**base, use_slam='false')
    slam = {'cuvslam_slam', 'nvblox_node', 'nvblox_map', 'pole_depth_gate', 'gripper_depth_gate'}
    rtab = {'rtabmap', 'rgbdx_sync', 'scan_cam_gate', 'scan_cloud_merge', 'pole_rgbd_sync', 'gripper_rgbd_sync'}  # 예전 SLAM — 남아 있으면 안 됨
    # 지도 파일: 매핑이면 끌 때 저장(불러오기 없음) · 위치추정이면 시작할 때 불러오기(저장 안 함 — 불러온 지도를 안 덮어씀)
    m = {**base, 'nvblox_map': '/maps/a/nvblox.nvblx'}
    save_map = params('nvblox_node', **m, localization='false').get('after_shutdown_map_save_path')
    save_loc = params('nvblox_node', **m, localization='true').get('after_shutdown_map_save_path')
    load_map = params('nvblox_map', **m, localization='false').get('load_path')
    load_loc = params('nvblox_map', **m, localization='true').get('load_path')
    checks = [
        ('SLAM 노드 전부 (cuVSLAM · depth gate ×2 · nvblox · nvblox_map)', slam <= on),
        ('RTAB-Map 노드 없음', not (rtab & on)),
        ('use_slam:=false: SLAM 노드 없음', not (slam & off)),
        ('매핑: 끌 때 nvblox 지도 저장', save_map == '/maps/a/nvblox.nvblx'),
        ('매핑: 불러오기 없음', load_map == ''),
        ('위치추정: 시작할 때 불러옴', load_loc == '/maps/a/nvblox.nvblx'),
        ('위치추정: 저장 안 함', save_loc == ''),
        ('cuVSLAM이 map→odom', params('cuvslam_slam', **base).get('publish_map_tf') is True),
    ]
    bad = [name for name, ok in checks if not ok]
    for name, ok in checks:
        print(('PASS ' if ok else 'FAIL ') + name)
    if bad:
        print(f'save={save_map!r}/{save_loc!r} load={load_map!r}/{load_loc!r}\non={sorted(on)}\noff={sorted(off)}')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
