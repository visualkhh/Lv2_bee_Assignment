"""Isaac Sim 전체 미션용 ROS — isaac_system.launch.py(실기 bringup 전체)에 미션에 필요한 기본값만 바꿔 넘긴다.

  isaac/mission/isaac.sh        # Isaac + 이 launch (Play 뒤 ROS가 뜸)
  ros2 launch isaac/mission/mission.launch.py map:=saved   # 따로 띄울 때 · 인자는 bringup 인자 그대로

isaac_system 기본(map:=slam)은 매핑이라 bringup이 인지를 끄고(perception:=auto) 조작 모드가 MANUAL이다.
  그대로면 coordinator가 미션 단계를 manual_mode로 거절하고, /tracked_objects가 없어 SURVEY에서 멈춘다.
  → use_perception:=true · operation:=AUTO (실기에서 지도를 고르면 state_manager가 AUTO로 다시 띄우는 것과 같은 상태)
launch 인자는 include에도 그대로 보이므로 isaac_system.launch.py는 고치지 않는다.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

ARGS = {
    'use_perception': 'true',   # YOLO · signal_decoder · hand_relative (매핑 중에도)
    'operation': 'AUTO',        # coordinator가 skill을 실행하는 모드
}
RUNTIME = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'standalone', 'runtime')


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(RUNTIME, 'isaac_system.launch.py'))),
    ])
