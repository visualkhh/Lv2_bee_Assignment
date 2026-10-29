#!/usr/bin/env python3
"""slam:= 두 가지가 어떤 노드로 펼쳐지는지 — GPU 없는 테스트베드에서도 launch 구조만은 늘 검사 (한쪽만 고쳐 다른 쪽이 깨지는 것 방지)

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
    rtab = nodes(**base, slam='rtabmap')
    nv = nodes(**base, slam='cuvslam_nvblox')
    rtab_odom = nodes(**base, slam='rtabmap', cuvslam_odom='true')
    off = nodes(**base, slam='cuvslam_nvblox', use_rtabmap='false')
    nv_only = {'cuvslam_slam', 'nvblox_node', 'nvblox_map', 'pole_depth_gate', 'gripper_depth_gate'}
    rtab_only = {'rtabmap', 'rgbdx_sync', 'scan_cam_gate', 'scan_cloud_merge', 'pole_rgbd_sync', 'gripper_rgbd_sync'}
    checks = [
        ('rtabmap: RTAB-Map 노드 전부', rtab_only <= rtab),
        ('rtabmap: cuVSLAM·nvblox 없음', not (nv_only & rtab)),
        ('cuvslam_nvblox: cuVSLAM·nvblox 노드 전부', nv_only <= nv),
        ('cuvslam_nvblox: RTAB-Map 없음', not (rtab_only & nv)),
        ('rtabmap + cuvslam_odom: cuvslam_slam만 더해짐', rtab_odom - rtab == {'cuvslam_slam'}),
        ('use_rtabmap:=false: SLAM 노드 없음', not ((nv_only | rtab_only) & off)),
    ]
    # 지도 파일: 매핑이면 끌 때 저장(불러오기 없음) · 위치추정이면 시작할 때 불러오기(저장 안 함 — 불러온 지도를 안 덮어씀)
    m = {**base, 'slam': 'cuvslam_nvblox', 'nvblox_map': '/maps/a/nvblox.nvblx'}
    save_map = params('nvblox_node', **m, localization='false').get('after_shutdown_map_save_path')
    save_loc = params('nvblox_node', **m, localization='true').get('after_shutdown_map_save_path')
    load_map = params('nvblox_map', **m, localization='false').get('load_path')
    load_loc = params('nvblox_map', **m, localization='true').get('load_path')
    checks += [
        ('매핑: 끌 때 nvblox 지도 저장', save_map == '/maps/a/nvblox.nvblx'),
        ('매핑: 불러오기 없음', load_map == ''),
        ('위치추정: 시작할 때 불러옴', load_loc == '/maps/a/nvblox.nvblx'),
        ('위치추정: 저장 안 함', save_loc == ''),
        ('cuvslam_nvblox: cuVSLAM이 map→odom', params('cuvslam_slam', **base, slam='cuvslam_nvblox').get('publish_map_tf') is True),
        ('rtabmap + cuvslam_odom: map→odom은 RTAB-Map', params('cuvslam_slam', **base, slam='rtabmap', cuvslam_odom='true')
         .get('publish_map_tf') is False),
    ]
    bad = [name for name, ok in checks if not ok]
    for name, ok in checks:
        print(('PASS ' if ok else 'FAIL ') + name)
    if bad:
        print(f'save={save_map!r}/{save_loc!r} load={load_map!r}/{load_loc!r}')
        print(f'rtabmap={sorted(rtab)}\ncuvslam_nvblox={sorted(nv)}\nrtabmap+odom={sorted(rtab_odom)}\noff={sorted(off)}')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
