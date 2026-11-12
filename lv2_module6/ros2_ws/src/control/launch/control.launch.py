"""control 패키지 노드만 단독 실행.

전체 로봇은 bringup/bringup.launch.py를 쓴다. 이 파일은 control 노드만 띄워
토픽·서비스 단위로 시험할 때 쓴다. BeebotSystemHardware는 ROS 노드가 아니라
ros2_control 하드웨어 플러그인이므로 여기서 띄우지 않는다 (ros_system의
controller_manager가 URDF <ros2_control>로 로드한다).

  ros2 launch control control.launch.py
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def node(executable):
    config = PathJoinSubstitution([FindPackageShare('control'), 'config', 'control.yaml'])
    return Node(package='control', executable=executable, output='screen', parameters=[config])


def generate_launch_description():
    return LaunchDescription([
        node('arm_safety_monitor'),
        node('manual_control'),
    ])
