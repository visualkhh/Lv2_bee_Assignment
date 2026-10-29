"""plan 패키지 노드만 단독 실행.

전체 로봇은 bringup/bringup.launch.py를 쓴다. 이 파일은 user_input,
vlm_task_planner, coordinator, explorer만 띄운다. Nav2·MoveIt·인지 노드는
따로 실행해야 한다.

  ros2 launch plan plan.launch.py
  ros2 launch plan plan.launch.py command:="빨간 퍽을 목표 구역으로 옮겨"
  ros2 launch plan plan.launch.py vlm_endpoint:=http://127.0.0.1:8000
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def node(executable, overrides=None):
    config = PathJoinSubstitution([FindPackageShare('plan'), 'config', 'plan.yaml'])
    return Node(package='plan', executable=executable, output='screen',
                parameters=[config, overrides or {}])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('command', default_value='',
                              description='user_input이 시작 후 한 번 보낼 텍스트 명령 (비면 안 보냄)'),
        DeclareLaunchArgument('vlm_endpoint', default_value='',
                              description='호스트 ai-service 주소 (비면 템플릿 고정)'),
        node('user_input', {
            'command': ParameterValue(LaunchConfiguration('command'), value_type=str),
        }),
        node('vlm_task_planner', {
            'vlm_endpoint': ParameterValue(LaunchConfiguration('vlm_endpoint'), value_type=str),
        }),
        node('coordinator'),
        node('explorer'),
    ])
