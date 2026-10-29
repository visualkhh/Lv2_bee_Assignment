"""전체 실행: ros_system(📦 제공 패키지) + 🛠 직접 작성 노드.

  ros2 launch bringup bringup.launch.py
  ros2 launch bringup bringup.launch.py use_ros_system:=false            # 📦 대신 faker 등 다른 입력 (faker.launch.py가 씀)
  ros2 launch bringup bringup.launch.py learning_enabled:=false           # 학습 노드 Δ=0 · 프리셋 A 고정 (비교 기준)
  ros2 launch bringup bringup.launch.py command:="빨간 퍽을 목표 구역으로 옮겨"   # user_input에 텍스트 명령 한 번
지도 (실기·테스트베드 공통 — 지도 한 벌 = 폴더 하나 <map_dir>/<map_name>/ rtabmap.db · map.pgm · map.yaml):
  ros2 launch bringup bringup.launch.py map:=slam                 # 매핑: 새 폴더 <날짜_시각>/ (map_name:=으로 이름 지정, 있는 이름이면 거부)
  ros2 launch bringup bringup.launch.py map:=saved                # 위치추정: 가장 최근 폴더 (기본)
  ros2 launch bringup bringup.launch.py map:=saved map_name:=20261017_163731
  map_dir 기본 ~/.ros/beebot_maps (실기 start.sh는 ros2_ws/map, 테스트베드 faker는 /ws/map)
  map:=course 는 테스트베드 전용 (RTAB-Map 없이 faker의 코스 지도 map_server)
  실행 중 바꾸기(재시작 없음): 스캔 ↔ 위치추정 /cmd/map_mode · 지도 갈아끼우기 /cmd/map_name — state_manager
재시작 (state_manager — 이 launch 명령줄 그대로 다시): ros2 topic pub --once /cmd/restart std_msgs/msg/String "{data: 'all'}"
  'all map:=slam' (그 인자만 바꿔) · 'node perception' (그 노드만, 같은 파라미터)

🛠 노드 (패키지 · 담당):
  perception: perception, body_filter, signal_decoder(신호등 암호 → /mission/signal) (전승혜)
  plan:       user_input(외부 텍스트 입력), vlm_task_planner, coordinator (권형중)
  ml:         grasp_scorer, rl_nav_tuner, rl_nav_correction (송정혁)
              rl_arm_correction은 노드가 아니라 컨트롤러 플러그인(ml/RlArmCorrection) — ros_system의 controller_manager가 올린다
  control:    arm_safety_monitor, manual_control (김현하)
상태: 각 노드 /diagnostics → diagnostic_aggregator → /diagnostics_agg (config/diagnostics.yaml)
"""

import os
import sys
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


ML_NODES = ('grasp_scorer', 'rl_nav_tuner', 'rl_nav_correction')


def node(package, executable, overrides=None, condition=None):
    # 파라미터: 패키지 config/<패키지>.yaml → launch 인자(overrides)가 덮어씀
    config = PathJoinSubstitution([FindPackageShare(package), 'config', f'{package}.yaml'])
    return Node(package=package, executable=executable, output='screen', parameters=[config, overrides or {}],
                condition=condition)


# ros_system에 그대로 넘기는 하드웨어·서버 인자 (기본값 = 실기). 테스트베드 faker가 대역에 맞게 덮어씀
ROS_SYSTEM_ARGS = {
    'urdf': ('default', '로봇 모델 = lv2_module6/urdf/<이 이름>/urdf.xacro (폴더 하나 = 모델 하나, 메시는 그 폴더 meshes/)'),
    'mock_hardware': ('false', 'true면 OpenCR 없이 BeebotSystemHardware가 명령을 상태로 되돌림'),
    'mock_effort': ('0.0', 'mock일 때 팔 관절 effort [A]'),
    'port': ('/dev/opencr', 'OpenCR 포트 (udev 고정 이름)'),
    'arm_correction_type': ('ml/RlArmCorrection', 'rl_arm_correction 컨트롤러 플러그인 (테스트베드 ML 대역: faker/FakeRlArmCorrection)'),
    'use_cameras': ('true', 'RealSense 2대 (테스트베드는 faker의 fake_camera가 대신 → false)'),
    'use_pole_front_cam': ('true', 'pole_front_cam만 끄기/켜기 (한 대만 연결할 때)'),
    'use_gripper_cam': ('true', 'gripper_cam만 끄기/켜기 (한 대만 연결할 때)'),
    'pole_front_cam_serial': ('', 'pole_front_cam RealSense serial_no — 빈 값이면 첫 장치 (2대면 반드시 지정)'),
    'gripper_cam_serial': ('', 'gripper_cam RealSense serial_no — 빈 값이면 첫 장치 (2대면 반드시 지정)'),
    'pole_front_cam_profile': ('640x360x15', 'pole_front_cam RGB/depth 프로파일 (WxHxFPS)'),
    'gripper_cam_profile': ('640x360x15', 'gripper_cam RGB/depth 프로파일 (WxHxFPS)'),
    'camera_decimation_enable': ('true', 'RealSense depth decimation 필터 (점군 밀도 감소)'),
    'camera_decimation_magnitude': ('2', 'decimation 배율 (2=1/4 점, 3=1/9 점)'),
    'pole_front_cam_pointcloud': ('true', 'pole_front_cam PointCloud2 발행 (body_filter/Nav2)'),
    'gripper_cam_pointcloud': ('false', 'gripper_cam PointCloud2 발행 (grasp_scorer용, 필요할 때만 true)'),
    'use_ekf': ('true', 'robot_localization EKF'),
    'use_nav2': ('true', '진짜 Nav2 서버'),
    'use_moveit': ('true', '진짜 move_group — coordinator의 pick/place가 씀 (ros_system 기본은 false라 실기에서 팔 작업이 안 됐음)'),
}


def setup_map(context):
    """지도 정하기 (실기·테스트베드 공통): map:=slam → 새 폴더(기본 날짜_시각) 매핑 · saved → 고른 폴더(기본 default, 없으면 가장 최근) 위치추정 ·
    course → RTAB-Map 없음(테스트베드 faker의 코스 지도). ros_system에 넘길 use_rtabmap · localization · rtabmap_db와
    state_manager 시작 값(map_mode · map_name · map_dir)을 정한다. ros_system을 안 띄우면(use_ros_system:=false) 확인만 건너뜀"""
    mode = LaunchConfiguration('map').perform(context)
    d = os.path.expanduser(LaunchConfiguration('map_dir').perform(context) or '~/.ros/beebot_maps')
    name = LaunchConfiguration('map_name').perform(context)
    if mode not in ('course', 'slam', 'saved'):
        raise RuntimeError(f'map:={mode} — slam | saved (테스트베드는 course도)')
    no_map = False
    if LaunchConfiguration('use_ros_system').perform(context).lower() == 'true' and mode != 'course':
        if mode == 'slam':
            name = name or time.strftime('%Y%m%d_%H%M%S')
            if os.path.exists(os.path.join(d, name, 'rtabmap.db')):
                raise RuntimeError(f'이미 있는 지도: {d}/{name} — 덮어쓰지 않음, 다른 map_name')
            os.makedirs(os.path.join(d, name), exist_ok=True)  # RTAB-Map은 DB 폴더를 안 만듦
        else:
            saved = sorted(p for p in (os.listdir(d) if os.path.isdir(d) else [])
                           if os.path.exists(os.path.join(d, p, 'rtabmap.db')))
            if not saved:
                # 센서·지도가 아직 없어도 state_manager와 나머지 노드는 뜨게 한다.
                print(f'WARN: 저장한 지도 없음: {d}/<이름>/rtabmap.db — RTAB-Map 없이 시작', file=sys.stderr)
                name = ''
                no_map = True
            else:
                name = name or ('default' if 'default' in saved else saved[-1])  # 기본 지도 폴더 = default (rtabmap.db가 있어야)
                if name not in saved:
                    print(f'WARN: 지도 없음: {d}/{name}/rtabmap.db — RTAB-Map 없이 시작 (있는 것: {", ".join(saved)})',
                          file=sys.stderr)
                    name = ''
                    no_map = True
    if mode == 'course':
        name = ''
    use_rtabmap = 'false' if mode == 'course' or no_map else 'true'
    return [SetLaunchConfiguration('map_dir', d), SetLaunchConfiguration('map_name', name),
            SetLaunchConfiguration('map_mode', {'course': 'COURSE', 'slam': 'MAPPING', 'saved': 'LOCALIZATION'}[mode]),
            SetLaunchConfiguration('use_rtabmap', use_rtabmap),
            SetLaunchConfiguration('localization', 'false' if mode == 'slam' else 'true'),
            SetLaunchConfiguration('rtabmap_db', os.path.join(d, name, 'rtabmap.db'))]


def generate_launch_description():
    learning = {'enabled': LaunchConfiguration('learning_enabled')}
    return LaunchDescription([
        DeclareLaunchArgument('use_ros_system', default_value='true',
                              description='false면 ros_system(📦)을 띄우지 않음'),
        DeclareLaunchArgument('use_ml', default_value='true',
                              description='false면 ml 노드(grasp_scorer · rl_nav_tuner · rl_nav_correction)를 띄우지 않음 — 아래 셋의 기본값'),
        *[DeclareLaunchArgument(f'use_{n}', default_value=LaunchConfiguration('use_ml'),
                                description=f'false면 {n}을 띄우지 않음 (faker가 그 대역 fake_{n}을 띄울 때)') for n in ML_NODES],
        DeclareLaunchArgument('learning_enabled', default_value='true',
                              description='false면 학습 노드가 Δ=0 · 기본 프리셋 (노드는 그대로 뜸)'),
        DeclareLaunchArgument('vlm_endpoint', default_value='',
                              description='호스트 ai-service 주소 (비면 템플릿 고정, 예: http://127.0.0.1:8000)'),
        DeclareLaunchArgument('command', default_value='',
                              description='user_input이 시작 후 한 번 보낼 텍스트 명령'),
        DeclareLaunchArgument('restart_prepare', default_value='',
                              description='/cmd/restart all 때 다시 띄우기 전 셸 명령 (state_manager — 테스트베드 faker = build-ws)'),
        DeclareLaunchArgument('restart_log', default_value='/tmp/bringup.log',
                              description='/cmd/restart all 때 다시 띄운 launch 출력 파일 (state_manager)'),
        DeclareLaunchArgument('map', default_value='saved',
                              description='slam = 새 폴더에 매핑 · saved = 저장 지도로 위치추정 (테스트베드 course = 코스 지도, RTAB-Map 없음)'),
        DeclareLaunchArgument('map_dir', default_value='',
                              description='지도 폴더들이 있는 곳 — <map_dir>/<map_name>/rtabmap.db (비면 ~/.ros/beebot_maps)'),
        DeclareLaunchArgument('map_name', default_value='',
                              description='지도 폴더 이름 — slam: 비우면 날짜_시각으로 새로 · saved: 비우면 가장 최근'),
        *[DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in ROS_SYSTEM_ARGS.items()],
        OpaqueFunction(function=setup_map),

        # GroupAction: ros_system의 launch 인자가 이 파일 인자와 섞이지 않게 범위 분리 — 넘길 것만 골라서
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('ros_system'), 'launch', 'ros_system.launch.py'])),
            launch_arguments={**{k: LaunchConfiguration(k) for k in ROS_SYSTEM_ARGS},
                              'use_rtabmap': LaunchConfiguration('use_rtabmap'),
                              'localization': LaunchConfiguration('localization'),
                              'rtabmap_db': LaunchConfiguration('rtabmap_db')}.items())],
            condition=IfCondition(LaunchConfiguration('use_ros_system'))),
        # RTAB-Map(slam·saved)일 때: 지도 저장 서버 — /map_saver/save_map → <map_dir>/<map_name>/map.pgm·yaml (시뮬레이터 [지도 저장])
        Node(package='nav2_map_server', executable='map_saver_server', name='map_saver', output='screen',
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_ros_system'), "'.lower() == 'true' and '",
                                                     LaunchConfiguration('use_rtabmap'), "' == 'true'"]))),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map_saver',
             output='screen', parameters=[{'autostart': True, 'node_names': ['map_saver'], 'bond_timeout': 60.0}],
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_ros_system'), "'.lower() == 'true' and '",
                                                     LaunchConfiguration('use_rtabmap'), "' == 'true'"]))),

        node('perception', 'perception_node'),
        node('plan', 'user_input', {'command': ParameterValue(LaunchConfiguration('command'), value_type=str)}),  # 공백·빈 값도 문자열로
        node('plan', 'vlm_task_planner',
             {'vlm_endpoint': ParameterValue(LaunchConfiguration('vlm_endpoint'), value_type=str)}),
        # coordinator: 지도 만들 때 팔을 스캔 자세로 — 자세는 ros_system/config/scan_pose.yaml 하나 (RTAB-Map scan_cam_gate와 같은 값)
        Node(package='plan', executable='coordinator', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('plan'), 'config', 'plan.yaml']),
                         PathJoinSubstitution([FindPackageShare('ros_system'), 'config', 'scan_pose.yaml'])]),
        node('plan', 'explorer'),
        *[node('ml', n, learning, condition=IfCondition(LaunchConfiguration(f'use_{n}'))) for n in ML_NODES],
        node('control', 'arm_safety_monitor'),
        node('control', 'manual_control'),
        node('perception', 'body_filter'),
        node('perception', 'signal_decoder'),
        node('manager', 'state_manager', {'map_mode': LaunchConfiguration('map_mode'),
                                          'map_name': ParameterValue(LaunchConfiguration('map_name'), value_type=str),
                                          'map_dir': ParameterValue(LaunchConfiguration('map_dir'), value_type=str),
                                          'restart_prepare': ParameterValue(LaunchConfiguration('restart_prepare'), value_type=str),
                                          'restart_log': ParameterValue(LaunchConfiguration('restart_log'), value_type=str)}),
        Node(package='diagnostic_aggregator', executable='aggregator_node', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('bringup'), 'config', 'diagnostics.yaml'])]),
    ])
