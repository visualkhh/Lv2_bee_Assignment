"""Manipulator + camera test bringup: ros_system with everything that is not the arm or the cameras switched off.

  ros2 launch ./arm_cam.launch.py   # in scripts/calib
  ros2 launch .../arm_cam.launch.py use_moveit:=true      # + move_group (stand / pick need it)
  ros2 launch .../arm_cam.launch.py use_gripper_cam:=false
  ros2 launch .../arm_cam.launch.py twist_state:=false   # raw encoder angles in /joint_states (default true: + gravity twist)
  ros2 launch .../arm_cam.launch.py mock_hardware:=true use_cameras:=false      # no robot (use another ROS_DOMAIN_ID)

Up: robot_state_publisher, ros2_control (BeebotSystemHardware), joint_state_broadcaster, gripper_controller,
rl_arm_correction -> arm_controller, RealSense gripper_cam + pole_front_cam.
Off: EKF (no odom TF), SLAM, Nav2, twist_mux, perception, coordinator, state_manager.
The camera serials come from POLE_FRONT_CAM_SERIAL / GRIPPER_CAM_SERIAL (~/.bashrc). The main bringup must be
stopped (one process holds the OpenCR port): ./stop.sh in ros2_ws first.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

ARGS = {  # name: default, passed through to ros_system.launch.py
    'mock_hardware': 'false',
    'use_cameras': 'true',
    'use_moveit': 'false',
    'use_gripper_cam': 'true',
    'use_pole_front_cam': 'true',
    'pole_front_cam_serial': os.environ.get('POLE_FRONT_CAM_SERIAL', ''),
    'gripper_cam_serial': os.environ.get('GRIPPER_CAM_SERIAL', ''),
    'gripper_cam_pointcloud': 'false',
    'pole_front_cam_pointcloud': 'false',
    'twist_state': 'true',  # /joint_states·TF = encoder + gravity twist (real joint angle); false = raw encoder (stiffness runs)
}
OFF = ['use_ekf', 'use_slam', 'use_nav2', 'use_twist_mux']


def generate_launch_description():
    passed = {k: LaunchConfiguration(k) for k in ARGS}
    passed.update({k: 'false' for k in OFF})
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('ros_system'), 'launch', 'ros_system.launch.py'])),
            launch_arguments=passed.items()),
    ])
