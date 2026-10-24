"""Real-arm sine test: robot_state_publisher + ros2_control (BeebotSystemHardware from the main ws `control`
package) + joint_state_broadcaster + JTC, then sine_test_node once the controllers are up.

  ros2 launch sine_test sine_test.launch.py                       # real robot, /dev/opencr
  ros2 launch sine_test sine_test.launch.py mock_hardware:=true   # no robot: hardware echoes commands
  ros2 launch sine_test sine_test.launch.py config:=/path/my.yaml out_dir:=/path/logs

The main bringup must be stopped (only one process can hold the OpenCR port).
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, Shutdown, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# install/sine_test/share/sine_test/launch -> real_test/ros2_ws -> lv2_module6
WS = Path(__file__).absolute().parents[5]  # no resolve(): --symlink-install would point into src/
LV2 = WS.parents[3]


def generate_launch_description():
    arg = LaunchConfiguration
    share = FindPackageShare('sine_test')
    controllers = PathJoinSubstitution([share, 'config', 'controllers.yaml'])
    robot_description = ParameterValue(Command([
        'xacro ', arg('urdf'), ' mock_hardware:=', arg('mock_hardware'), ' port:=', arg('port')]), value_type=str)
    activate = ExecuteProcess(
        cmd=['ros2', 'control', 'set_hardware_component_state', 'BeebotSystem', 'active'], output='screen')
    spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', 'arm_controller', '--controller-manager', '/controller_manager',
                   '--param-file', controllers])
    test = Node(
        package='sine_test', executable='sine_test_node', output='screen', emulate_tty=True,
        parameters=[{'config': arg('config'), 'out_dir': arg('out_dir'), 'mock_hardware': arg('mock_hardware')}])
    return LaunchDescription([
        DeclareLaunchArgument('mock_hardware', default_value='false', description='true: no robot, hardware echoes commands'),
        DeclareLaunchArgument('port', default_value='/dev/opencr', description='OpenCR serial port'),
        DeclareLaunchArgument('urdf', default_value=str(LV2 / 'urdf' / 'beebot_mesh' / 'urdf.xacro')),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([share, 'config', 'sine.yaml'])),
        DeclareLaunchArgument('out_dir', default_value=str(WS.parent / 'logs')),

        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),
        # controller_manager reads the URDF from /robot_description (robot_state_publisher)
        Node(package='controller_manager', executable='ros2_control_node', output='screen', parameters=[controllers]),
        # hardware starts inactive (controllers.yaml) so the real pose is read before anything is written
        TimerAction(period=2.0, actions=[activate]),
        RegisterEventHandler(OnProcessExit(target_action=activate, on_exit=[spawner])),
        RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=[test])),
        RegisterEventHandler(OnProcessExit(target_action=test, on_exit=[Shutdown(reason='sine test finished')])),
    ])
