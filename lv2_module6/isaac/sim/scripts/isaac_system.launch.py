"""ROS side for Isaac Sim — isaac_sim.sh가 Isaac과 같이 띄운다 (따로: ros2 launch <이 파일>).

  bringup (실기와 같은 전체 launch — ros_system + plan·manager·control·perception·ml)에 Isaac용 인자만 바꿔 넘김:
    ros2_control: port:=/tmp/opencr — Isaac 안 opencr_bridge.py(가상 OpenCR)와 S/C 시리얼
    use_cameras:=false (토픽은 Isaac + camera_adapter.py가 실기 이름·형식으로 냄) · map:=slam (매핑)
    use_sim_time: 인자가 아니라 SetParameter로 모든 노드에 (ros2_ws 패키지는 수정하지 않음)
    그 밖은 bringup 기본값 = 실기 (Nav2 · MoveIt · twist_mux · coordinator 시작 자세 등)
    바꾸기: ros2 launch isaac_system.launch.py use_ml:=false map:=saved …
    perception_node는 torch · ultralytics · zxingcpp가 시스템 python3에 있어야 뜬다 (없으면 그 노드만 죽음)
  pole_front_cam 점구름: depth_image_proc
    Isaac 6.1 + Lyrical에서 ROS2 Camera Helper `depth_pcl`은 Play 직후 segfault (Jazzy 백엔드로 도는
    PointCloud2 직렬화) → Isaac은 영상(컬러·정렬 depth)까지만 내고 점구름은 여기서 만든다.
    gripper_cam 점구름은 실물도 기본으로 꺼져 있어 만들지 않는다.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

# bringup 인자 — 실기와 같은 전체 실행(bringup: ros_system + plan·manager·control·perception·ml), 기본값만 Isaac용
ARGS = {
    'port': '/tmp/opencr',       # opencr_bridge.py(가상 OpenCR)
    'use_cameras': 'false',      # 카메라 토픽은 Isaac이 실기 이름으로 냄
    'map': 'slam',               # 매핑 — 새 폴더 ~/.ros/beebot_maps/<날짜_시각>/ (저장 지도로: map:=saved)
}

# beebot.usda 기본 자세 [deg] 0 · -109.435 · 78.5 · 34.1 · 0 → [rad] (joint1~5)
SCAN_POSE = 'scan_pose:=[0.0, -1.91, 1.37008, 0.59516, 0.0]'
NODE_PARAMS = {  # 노드 이름: 명령줄 끝에 덧붙일 -p 값 (--after: 그 노드들을 먼저 고친 뒤에)
    'state_manager': ['--after', 'coordinator,pole_depth_gate,gripper_depth_gate'],  # MAPPING은 스캔 자세를 고친 뒤에 냄
    'coordinator': [SCAN_POSE],
    'pole_depth_gate': [SCAN_POSE],
    'gripper_depth_gate': [SCAN_POSE],
}

NS = '/pole_front_cam'
DEPTH = NS + '/aligned_depth_to_color/image_raw'
INFO = NS + '/color/camera_info'
DEPTH_INFO = NS + '/aligned_depth_to_color/camera_info'  # image_transport가 찾는 자리 — 같은 RenderProduct라 color K와 같다


def generate_launch_description():
    best_effort = {f'qos_overrides.{t}.subscription.reliability': 'best_effort' for t in (DEPTH, INFO, DEPTH_INFO)}
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()],
        # Isaac 시계(/clock) — ros2_ws 패키지는 손대지 않고 여기서 모든 노드(bringup·ros_system include 포함)에 준다
        SetParameter(name='use_sim_time', value=True),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('bringup'), 'launch', 'bringup.launch.py'])),
            launch_arguments={k: LaunchConfiguration(k) for k in ARGS}.items()),
        # Isaac 카메라 → 실기 RealSense 토픽 모양 (depth 32FC1→16UC1, camera_info) — 설명: camera_adapter.py
        ExecuteProcess(cmd=['python3', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'camera_adapter.py'),
                            '--ros-args', '-p', 'use_sim_time:=true'], output='screen'),
        # bringup 노드 몇 개를 고친 파라미터로 다시 띄움 — 설명: node_params.py
        #   state_manager: use_sim_time -p가 params 파일 앞에 붙어 manager.yaml의 LOCALIZATION을 읽음(map:=slam인데 EXPLORE 거절) → -p를 뒤로
        #   coordinator · pole/gripper_depth_gate: 스캔 자세 = beebot.usda 기본 자세 (scan_pose.yaml 자세는 팔이 처져 2초마다 다시 감)
        *[ExecuteProcess(cmd=['python3', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'node_params.py'), name, *extra],
                         output='screen')
          for name, extra in NODE_PARAMS.items()],
        # Isaac 영상은 best_effort(SENSOR_DATA) — reliable로 구독하면 연결되지 않는다. 노드는 lazy 구독(points 구독자가 있어야 입력을 받음)
        Node(package='depth_image_proc', executable='point_cloud_xyz_node', name='pole_front_cam_points', output='screen',
             remappings=[('image_rect', DEPTH), ('points', NS + '/depth/color/points')],
             parameters=[{'use_sim_time': True, **best_effort}]),
    ])
