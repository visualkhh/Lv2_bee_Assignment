"""manager 패키지 노드만 단독 실행.

전체 로봇은 bringup/bringup.launch.py를 쓴다. 이 파일은 state_manager 하나를
파라미터와 함께 시험할 때 쓴다.

  ros2 launch manager manager.launch.py
  ros2 launch manager manager.launch.py map_mode:=MAPPING map_dir:=/tmp/maps
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('manager'), 'config', 'manager.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('map_mode', default_value='LOCALIZATION',
                              description='초기 지도 모드: LOCALIZATION | MAPPING | COURSE'),
        DeclareLaunchArgument('map_dir', default_value='',
                              description='지도 폴더들이 있는 곳 (비면 비워 둠)'),
        DeclareLaunchArgument('map_name', default_value='',
                              description='현재 지도 폴더 이름'),
        DeclareLaunchArgument('restart_prepare', default_value='',
                              description='/cmd/restart all 전에 실행할 셸 명령'),
        DeclareLaunchArgument('restart_log', default_value='/tmp/bringup.log',
                              description='/cmd/restart all 때 다시 띄운 launch 출력 파일'),
        Node(package='manager', executable='state_manager', output='screen',
             parameters=[config, {
                 'map_mode': ParameterValue(LaunchConfiguration('map_mode'), value_type=str),
                 'map_dir': ParameterValue(LaunchConfiguration('map_dir'), value_type=str),
                 'map_name': ParameterValue(LaunchConfiguration('map_name'), value_type=str),
                 'restart_prepare': ParameterValue(LaunchConfiguration('restart_prepare'), value_type=str),
                 'restart_log': ParameterValue(LaunchConfiguration('restart_log'), value_type=str),
             }]),
    ])
