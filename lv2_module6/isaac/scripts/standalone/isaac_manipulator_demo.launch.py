"""Isaac Sim 팔 시연 공용 ROS — SLAM·Nav2 없이 bringup stand_test + Isaac 카메라 어댑터 (pick_load_demo.sh가 씀).

  ISAAC_ROS_LAUNCH=isaac_manipulator_demo.launch.py ISAAC_AUTOPLAY=1 isaac/scripts/standalone/isaac_sim.sh
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        # pick_load_demo.sh --no-policy: use: false로 바꾼 모델 폴더 사본 — skill 노드(skill_node model_dir)만 읽음
        *([SetParameter(name='model_dir', value=os.environ['BEEBOT_MODEL_DIR'])] if os.environ.get('BEEBOT_MODEL_DIR') else []),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('bringup'), 'launch', 'stand_test.launch.py'])),
            launch_arguments={'port': '/tmp/opencr', 'use_cameras': 'false',
                              'grasp_min_effort': '0.0', 'miss_min_range': '0.1', 'perception_device': 'cuda',
                              # 상태 = 엔코더 + 비틀림 (기본) · pick_load_demo.sh --no-twist-state 또는 비틀림 없는 Isaac이면 엔코더 그대로
                              'twist_state': 'false' if '0' in (os.environ.get('BEEBOT_TWIST_STATE'), os.environ.get('OPENCR_TWIST_SCALE')) else 'true'}.items()),
        ExecuteProcess(cmd=['python3', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'camera_adapter.py'),
                            '--ros-args', '-p', 'use_sim_time:=true'], output='screen'),
    ])
