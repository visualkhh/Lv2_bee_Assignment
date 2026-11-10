"""plan 패키지 노드만 단독 실행.

전체 로봇은 bringup/bringup.launch.py를 쓴다. 이 파일은
coordinator만 띄운다. Nav2·MoveIt·인지 노드는
따로 실행해야 한다.

  ros2 launch plan plan.launch.py
"""

from launch import LaunchDescription

from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def node(executable, overrides=None):
    config = PathJoinSubstitution([FindPackageShare('plan'), 'config', 'plan.yaml'])
    return Node(package='plan', executable=executable, output='screen',
                parameters=[config, overrides or {}])


def generate_launch_description():
    return LaunchDescription([
        node('coordinator'),
    ])
