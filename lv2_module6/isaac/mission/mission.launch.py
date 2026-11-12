"""Isaac Sim 전체 미션용 ROS — isaac_system.launch.py(실기 bringup 전체)에 미션에 필요한 기본값만 바꿔 넘긴다.

  isaac/mission/isaac.sh        # Isaac + 이 launch (Play 뒤 ROS가 뜸)
  ros2 launch isaac/mission/mission.launch.py map:=saved   # 따로 띄울 때 · 인자는 bringup 인자 그대로

isaac_system 기본(map:=slam)은 매핑이라 bringup이 인지를 끄고(perception:=auto) 조작 모드가 MANUAL이다.
  그대로면 coordinator가 미션 단계를 manual_mode로 거절하고, /tracked_objects가 없어 SURVEY에서 멈춘다.
  → use_perception:=true · operation:=AUTO (실기에서 지도를 고르면 state_manager가 AUTO로 다시 띄우는 것과 같은 상태)
launch 인자는 include에도 그대로 보이므로 isaac_system.launch.py는 고치지 않는다.

경기장 벽: /map = nvblox_map이 walls_in 격자 위에 nvblox를 겹친 것 (ros_system/scripts/nvblox_map.py).
  실기 walls_in은 /arena/walls(mission_manager가 plan.yaml 벽 선분 20개로 그림). Isaac에서는 scene_sim.usda 벽 · 기둥을
  그대로 그린 map/scene_sim/map.yaml(make_map.py)을 map_server가 /arena/static으로 내고, nvblox_map을
  walls_in:=/arena/static으로 다시 띄운다 (node_params.py — ros2_ws 패키지는 그대로). 상자 · 퍽은 nvblox가 보는 대로 겹침.
  끄기: arena_map:='' (실기처럼 /arena/walls). 지도 원점 = USD beebot 자세 — 장면에서 로봇을 옮기면 make_map.py를 다시.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
RUNTIME = os.path.join(HERE, '..', 'standalone', 'runtime')
ARGS = {
    'use_perception': 'true',   # YOLO · signal_decoder · hand_relative (매핑 중에도)
    'operation': 'AUTO',        # coordinator가 skill을 실행하는 모드
    'arena_map': os.path.join(HERE, 'map', 'scene_sim', 'map.yaml'),   # nvblox_map walls_in으로 줄 지도 ('' = /arena/walls)
}
WALLS = '/arena/static'


def generate_launch_description():
    arena = IfCondition(PythonExpression(["'", LaunchConfiguration('arena_map'), "' != ''"]))
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(RUNTIME, 'isaac_system.launch.py'))),
        Node(package='nav2_map_server', executable='map_server', name='arena_static_map', output='screen', condition=arena,
             parameters=[{'yaml_filename': LaunchConfiguration('arena_map'), 'topic_name': WALLS, 'frame_id': 'map',
                          'use_sim_time': True}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_arena_static',
             output='screen', condition=arena,
             parameters=[{'autostart': True, 'node_names': ['arena_static_map'], 'use_sim_time': True}]),
        ExecuteProcess(cmd=['python3', os.path.join(RUNTIME, 'node_params.py'), 'nvblox_map', f'walls_in:={WALLS}'],
                       output='screen', condition=arena),
    ])
