"""Real-arm test bringup: robot_state_publisher + ros2_control (BeebotSystemHardware from the main ws `control`
package) + joint_state_broadcaster + arm_controller (JTC). Stays up; run the test nodes in another terminal:

  ros2 launch control_test hw.launch.py                       # real robot, /dev/opencr
  ros2 launch control_test hw.launch.py mock_hardware:=true   # no robot: hardware echoes commands
  ros2 run control_test set_center                            # joint5 -> joint1 to 0 deg
  ros2 run control_test sin_test joint3                       # one joint's sine test (config/sine/sine.yaml)

The main bringup must be stopped (only one process can hold the OpenCR port).
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# install/control_test/share/control_test/launch -> real_test/ros2_ws -> lv2_module6
WS = Path(__file__).absolute().parents[5]  # no resolve(): --symlink-install would point into src/
LV2 = WS.parents[3]


def generate_launch_description():
    arg = LaunchConfiguration
    controllers = PathJoinSubstitution([FindPackageShare('control_test'), 'config', 'controllers.yaml'])
    robot_description = ParameterValue(Command([
        'xacro ', arg('urdf'), ' mock_hardware:=', arg('mock_hardware'), ' port:=', arg('port'),
        ' mesh_uri:=', arg('mesh_uri')]), value_type=str)
    activate = ExecuteProcess(
        cmd=['ros2', 'control', 'set_hardware_component_state', 'BeebotSystem', 'active'], output='screen')
    spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', 'arm_controller', '--controller-manager', '/controller_manager',
                   '--param-file', controllers])
    return LaunchDescription([
        DeclareLaunchArgument('mock_hardware', default_value='false', description='true: no robot, hardware echoes commands'),
        DeclareLaunchArgument('port', default_value='/dev/opencr', description='OpenCR serial port'),
        DeclareLaunchArgument('urdf', default_value=str(LV2 / 'urdf' / 'beebot_mesh' / 'urdf.xacro')),
        # resolved where the URDF is displayed (e.g. RViz on a PC): that machine needs ros_system installed
        DeclareLaunchArgument('mesh_uri', default_value='package://ros_system/urdf/beebot_mesh/meshes',
                              description='mesh folder URI written into the URDF (same as the main ros_system launch)'),

        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),
        # controller_manager reads the URDF from /robot_description (robot_state_publisher)
        Node(package='controller_manager', executable='ros2_control_node', output='screen', parameters=[controllers]),
        # hardware starts inactive (controllers.yaml) so the real pose is read before anything is written
        TimerAction(period=2.0, actions=[activate]),
        RegisterEventHandler(OnProcessExit(target_action=activate, on_exit=[spawner])),
    ])
