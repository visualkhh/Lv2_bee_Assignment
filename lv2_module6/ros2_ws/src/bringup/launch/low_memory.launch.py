"""Opt-in Jetson operation profile; all robot nodes remain under normal bringup.

LAUNCH_FILE=low_memory.launch.py ./start.sh map:=slam map_backend:=nvblox
For new mapping: map:=slam map_backend:=nvblox (uses lightweight nvblox config).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    defaults = {
        'map': 'none',
        'map_backend': 'nvblox',
        'navigation_throttle': 'true',
        'navigation_infer_hz': '1.0',
        'cv_threads': '1',
        'nvblox_params_file': PathJoinSubstitution([
            FindPackageShare('ros_system'), 'config', 'nvblox_low_memory.yaml']),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('bringup'), 'launch', 'bringup.launch.py'])),
            launch_arguments={k: LaunchConfiguration(k) for k in defaults}.items()),
    ])
