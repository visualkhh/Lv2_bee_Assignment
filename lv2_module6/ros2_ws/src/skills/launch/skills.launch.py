"""skill 노드 — 계획 단계마다 액션 서버 하나 (/skills/<skill>). coordinator가 단계를 맡김.

  팔 skill (컨테이너 arm_skills — 한 프로세스, skill마다 노드 하나):
    skill_pose · skill_inspect · skill_pick · skill_place · skill_load · skill_unload · skill_stand (stand · stand_pick)
  주행 skill (컨테이너 base_skills): skill_navigate · skill_approach

  파라미터: config/skills.yaml (노드별) + params/arm.yaml (팔 공통) + params/basket.yaml (자세·적재함 표)
           + params/stand_table.yaml (세우기 표) + ros_system/config/scan_pose.yaml (스캔 자세)
  학습 정책: share/skills/model/<skill>/ (meta.yaml use: true면 씀)

  ros2 launch skills skills.launch.py
  ros2 launch skills skills.launch.py grasp_check:=false   # 퍽 없이 끝까지 닫히는 mock 그리퍼
"""
import os

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def table(name):
    with open(os.path.join(get_package_share_directory('skills'), 'params', name)) as f:
        return yaml.safe_load(f) or {}


def launch_setup(context):
    # 표(yaml)는 실행할 때 읽음 (--show-args 같은 검사에서는 패키지 설치 없이도 돎)
    share = get_package_share_directory('skills')
    config = os.path.join(share, 'config', 'skills.yaml')
    try:  # 스캔 자세 값은 ros_system 하나 (nvblox scan_cam_gate와 같은 값) — ros_system 없이 skill만 띄우면 skill_pose 기본값
        scan_pose = [os.path.join(get_package_share_directory('ros_system'), 'config', 'scan_pose.yaml')]
    except PackageNotFoundError:
        scan_pose = []
    arm = table('arm.yaml')
    arm['grasp_check'] = ParameterValue(LaunchConfiguration('grasp_check'), value_type=bool)
    arm['grasp_min_effort'] = ParameterValue(LaunchConfiguration('grasp_min_effort'), value_type=float)
    basket, stand = table('basket.yaml'), table('stand_table.yaml')

    def skill(cls, name, *extra):
        return ComposableNode(package='skills', plugin=f'skills::{cls}', name=name, parameters=[config, *extra])

    return [
        ComposableNodeContainer(
            name='arm_skills', namespace='', package='rclcpp_components', executable='component_container', output='screen',
            composable_node_descriptions=[
                skill('PoseSkill', 'skill_pose', arm, basket, *scan_pose),
                skill('InspectSkill', 'skill_inspect', arm),
                skill('PickSkill', 'skill_pick', arm, basket),  # basket: pose_carry_* (적재함 쪽에서 퍽으로 갈 때 경유)
                skill('PlaceSkill', 'skill_place', arm),
                skill('LoadSkill', 'skill_load', arm, basket),
                skill('UnloadSkill', 'skill_unload', arm, basket),
                skill('StandSkill', 'skill_stand', arm, stand),
            ]),
        ComposableNodeContainer(
            name='base_skills', namespace='', package='rclcpp_components', executable='component_container', output='screen',
            composable_node_descriptions=[
                skill('NavigateSkill', 'skill_navigate'),
                skill('ApproachSkill', 'skill_approach'),
            ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('grasp_check', default_value='true',
                              description='잡기 판정 (닫은 뒤 그리퍼 위치 + 전류). mock 그리퍼는 퍽 없이 끝까지 닫혀 false'),
        DeclareLaunchArgument('grasp_min_effort', default_value='0.25',
                              description='잡기 판정 전류 하한 [A] — Isaac은 그리퍼 전류를 안 내서 0 (위치만, opencr_bridge)'),
        OpaqueFunction(function=launch_setup),
    ])
