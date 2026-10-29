"""perception 패키지 노드만 단독 실행.

전체 로봇은 bringup/bringup.launch.py를 쓴다. 이 파일은 YOLO 인지 노드 ·
body_filter(자기 몸 점구름 필터) · signal_decoder(신호등 암호 → /mission/signal)를 카메라 토픽과 함께 시험할 때 쓴다.

  ros2 launch perception perception.launch.py
  ros2 launch perception perception.launch.py device:=cuda model_path:=/path/to/model.pt
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('perception'), 'config', 'perception.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value='',
                              description='YOLO 가중치 경로 (비면 share/perception/model/model.pt 자동 탐색)'),
        DeclareLaunchArgument('device', default_value='cpu',
                              description='추론 장치: cpu | cuda'),
        DeclareLaunchArgument('rate_hz', default_value='10.0',
                              description='/tracked_objects 발행 주기 [Hz]'),
        Node(package='perception', executable='perception_node', output='screen',
             parameters=[config, {
                 'model_path': ParameterValue(LaunchConfiguration('model_path'), value_type=str),
                 'device': ParameterValue(LaunchConfiguration('device'), value_type=str),
                 'rate_hz': ParameterValue(LaunchConfiguration('rate_hz'), value_type=float),
             }]),
        Node(package='perception', executable='body_filter', output='screen', parameters=[config]),
        Node(package='perception', executable='signal_decoder', output='screen', parameters=[config]),
    ])
