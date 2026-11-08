"""Isaac 인식 단독 시험 ROS — perception_node + 카메라 어댑터 + 고정 자세 TF만 (ros2_control · MoveIt · skill 없음).

  pick_load_demo.sh --perception-only가 씀. 팔은 Isaac에서 --arm-q 자세로 시작해 그대로 있으므로 joint_states도 같은 값으로 고정 발행.
  본체 고정(--fix-base)이라 map = odom = base_footprint (정적 TF).
  팔 자세: /tmp/beebot_arm_q (manipulator_env.py가 씀), 없으면 BEEBOT_ARM_Q joint1~5 [deg] · /tmp/beebot_perception_params: perception 파라미터 덮어쓰기
"""
import math
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS_FILE = '/tmp/beebot_perception_params'
ARM_Q_FILE = '/tmp/beebot_arm_q'  # manipulator_env.py와 같은 경로
URDF = os.path.join(HERE, '..', '..', '..', 'urdf', 'default', 'urdf.xacro')


def generate_launch_description():
    extra = open(PARAMS_FILE).read().split() if os.path.exists(PARAMS_FILE) else []
    # manipulator_env.py가 리셋마다 쓰는 지금 자세 (--arm-q box면 시행마다 다름) · 없으면 BEEBOT_ARM_Q
    q = [math.radians(float(v)) for v in (open(ARM_Q_FILE).read() if os.path.exists(ARM_Q_FILE) else os.environ['BEEBOT_ARM_Q']).split(',')]
    desc = ParameterValue(Command(['xacro ', URDF, ' mock_hardware:=true']), value_type=str)
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': desc}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'robot_description': desc, **{f'zeros.joint{k + 1}': v for k, v in enumerate(q)}}]),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['--frame-id', 'map', '--child-frame-id', 'odom']),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['--frame-id', 'odom', '--child-frame-id', 'base_footprint']),
        Node(package='perception', executable='perception_node', output='screen',
             # 덮어쓰기: perception.yaml의 노드 이름 키(perception:)가 launch dict(/**)보다 우선이고, 명령줄도 뒤에 온 것이 이김
             #   → yaml을 ros_arguments 맨 앞에 두고 그 뒤 -p로
             # PARAMS_FILE에 "body_radius:=0.1 self_radius:=0.04"처럼 적으면 시험용 덮어쓰기 (리셋마다 다시 읽음 — Isaac 재시작 없이)
             ros_arguments=['--params-file', PathJoinSubstitution([FindPackageShare('perception'), 'config', 'perception.yaml'])] + [a for kv in ['device:=cuda', 'miss_min_range:=0.1', *extra]
                            for a in ('-p', kv)]),
        ExecuteProcess(cmd=['python3', os.path.join(HERE, 'camera_adapter.py'), '--ros-args', '-p', 'use_sim_time:=true'], output='screen'),
    ])
