"""팔 작업 테스트용 가벼운 실행 (세우기 stand · 집기 pick): 팔·그리퍼·카메라·인지·MoveIt·coordinator만.

  LAUNCH_FILE=stand_test.launch.py ./start.sh   # 저장소 ros2_ws/ — 가상환경 켜고 start.sh 환경 확인(카메라 serial 등) 그대로

전체 bringup(start.sh)에서 빼는 것 — Jetson 메모리·CPU를 많이 먹는데 팔 테스트엔 안 쓰는 것:
  SLAM(cuVSLAM·nvblox·지도) · Nav2 서버 전부 · pole_front_cam 점구름 · body_filter · signal_decoder ·
  진단 aggregator
남기는 것: skill 노드 · hand_relative · ros_system(controller_manager · 컨트롤러 · robot_state_publisher · RealSense 2대 · EKF(odom TF) · twist_mux · move_group) ·
  perception · coordinator · grasp_scorer · manual_control · arm_safety_monitor · state_manager (모드 전환 · CLI) ·
  cmd_vel_relay (Nav2 대신 /cmd_vel_mux → /cmd_vel — 수동 주행이 바퀴까지, 부드럽게 하기·충돌 감시 없음)
지도가 없어 map→odom은 고정(항등) — 켠 자리가 map 원점.
Nav2가 없으니 팔이 닿지 않는 퍽(로봇 중심 reach_m 0.30 m 밖)은 다가가지 못하고 실패(approach_failed) — 퍽을 인식시킨 뒤
수동 주행(operator_tool drive)으로 가까이 가서 명령한다 (퍽은 안 보여도 기억됨).
"""

import importlib.util
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# bringup.launch.py의 하드웨어·카메라 인자와 node()를 그대로 (start.sh가 넘기는 인자가 같게)
_spec = importlib.util.spec_from_file_location('bringup_main', os.path.join(os.path.dirname(__file__), 'bringup.launch.py'))
_main = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_main)
node = _main.node

# 이 실행에서 고정하는 ros_system 인자 (지도·Nav2 없음 · 점구름 끔 · MoveIt 켬)
FIXED = {'use_slam': 'false', 'localization': 'true', 'nvblox_map': '', 'use_nav2': 'false', 'use_cmd_chain': 'false', 'use_moveit': 'true',
         'pole_front_cam_pointcloud': 'false', 'gripper_cam_pointcloud': 'false'}


def generate_launch_description():
    passthrough = {k: v for k, v in _main.ROS_SYSTEM_ARGS.items() if k not in FIXED}
    learning = {'enabled': LaunchConfiguration('learning_enabled')}
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in passthrough.items()],
        DeclareLaunchArgument('miss_min_range', default_value='0.7', description='이 거리 안의 물체는 못 봐도 안 지움 (전체 bringup 기본 0.1)'),
        DeclareLaunchArgument('perception_device', default_value='cpu', description='YOLO 장치 cpu | cuda (perception.yaml device를 덮어씀 — Isaac 시연 PC는 cuda)'),
        DeclareLaunchArgument('grasp_check', default_value='true', description='잡기 판정'),
        DeclareLaunchArgument('grasp_min_effort', default_value='0.25', description='잡기 판정 전류 하한 [A]'),
        DeclareLaunchArgument('learning_enabled', default_value='true', description='false면 정책 Δ 0'),
        DeclareLaunchArgument('map_dir', default_value='', description='start.sh 호환용 (쓰지 않음)'),
        # 고정 인자도 밖(전역)에 둔다 — ros_system이 spawner 끝난 뒤(이벤트 처리기)에 use_moveit 등을 다시 읽는데,
        # GroupAction 안에서만 넘기면 그때는 사라져 launch 전체가 죽음 ("launch configuration 'use_moveit' does not exist")
        *[SetLaunchConfiguration(k, v) for k, v in FIXED.items()],

        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('ros_system'), 'launch', 'ros_system.launch.py'])),
            launch_arguments={k: LaunchConfiguration(k) for k in {**passthrough, **FIXED}}.items())]),

        # 다가가면 퍽이 카메라 0.7 m 안(화면 아래·가까움)에 들어와 YOLO가 놓침 → 그 안의 물체는 못 봐도 지우지 않음 (기억 유지)
        node('perception', 'perception_node', {'miss_min_range': ParameterValue(LaunchConfiguration('miss_min_range'), value_type=float),
                                               'device': LaunchConfiguration('perception_device')}),
        node('perception', 'hand_relative'),
        node('plan', 'coordinator'),
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('skills'), 'launch', 'skills.launch.py'])),
            launch_arguments={'grasp_check': LaunchConfiguration('grasp_check'),
                              'grasp_min_effort': LaunchConfiguration('grasp_min_effort')}.items())]),
        node('ml', 'grasp_scorer', learning),
        Node(package='tf2_ros', executable='static_transform_publisher', output='screen',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom']),
        node('control', 'arm_safety_monitor'),
        node('control', 'manual_control'),
        # Nav2(velocity_smoother · collision_monitor)가 없으니 twist_mux 출력을 바퀴로 바로 — 수동 주행용
        Node(package='bringup', executable='cmd_vel_relay', output='screen'),
        node('manager', 'state_manager', {'map_mode': 'LOCALIZATION',
                                          'map_name': ParameterValue('', value_type=str),
                                          'map_dir': ParameterValue(LaunchConfiguration('map_dir'), value_type=str)}),
    ])
