"""ml 패키지 노드만 단독 실행.

전체 로봇은 bringup/bringup.launch.py를 쓴다. 이 파일은 grasp_scorer,
rl_nav_tuner, rl_nav_correction만 띄운다. rl_arm_correction은 노드가 아니라
controller_manager가 올리는 chainable controller 플러그인이라 여기 포함되지 않는다.

  ros2 launch ml ml.launch.py
  ros2 launch ml ml.launch.py learning_enabled:=false
  ros2 launch ml ml.launch.py use_rl_nav_correction:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


ML_NODES = ('grasp_scorer', 'rl_nav_tuner', 'rl_nav_correction')


def node(name, learning_enabled):
    config = PathJoinSubstitution([FindPackageShare('ml'), 'config', 'ml.yaml'])
    return Node(package='ml', executable=name, output='screen',
                parameters=[config, {
                    'enabled': ParameterValue(LaunchConfiguration(learning_enabled), value_type=bool),
                }],
                condition=IfCondition(LaunchConfiguration(f'use_{name}')))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_ml', default_value='true',
                              description='false면 ml 노드 전체를 띄우지 않음 — 아래 use_* 기본값'),
        *[DeclareLaunchArgument(f'use_{n}', default_value=LaunchConfiguration('use_ml'),
                                description=f'false면 {n}을 띄우지 않음')
          for n in ML_NODES],
        DeclareLaunchArgument('learning_enabled', default_value='true',
                              description='false면 학습 노드가 Δ=0 · 기본 프리셋 (노드는 그대로 뜸)'),
        *[node(n, 'learning_enabled') for n in ML_NODES],
    ])
