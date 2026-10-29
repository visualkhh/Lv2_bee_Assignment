"""ROS side for Isaac Sim — isaac_sim.sh가 Isaac과 같이 띄운다 (따로: ros2 launch <이 파일>).

  ros_system (실기와 같은 launch) 중 Isaac과 맞는 것만:
    ros2_control: port:=/tmp/opencr — Isaac 안 opencr_bridge.py(가상 OpenCR)와 S/C 시리얼
    끄는 것: 카메라 드라이버(토픽은 Isaac이 같은 이름으로 냄) · RTAB-Map · Nav2 · EKF · twist_mux
    필요하면 켜기: ros2 launch isaac_system.launch.py use_ekf:=true use_nav2:=true …
  pole_front_cam 점구름: depth_image_proc
    Isaac 6.1 + Lyrical에서 ROS2 Camera Helper `depth_pcl`은 Play 직후 segfault (Jazzy 백엔드로 도는
    PointCloud2 직렬화) → Isaac은 정렬 depth·camera_info까지만 내고 점구름은 여기서 만든다.
    gripper_cam 점구름은 실물도 기본으로 꺼져 있어 만들지 않는다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# ros_system 인자 — 기본값만 Isaac용, 명령줄에서 바꿀 수 있다
ARGS = {
    'port': '/tmp/opencr',
    'use_cameras': 'false',
    'use_rtabmap': 'false',
    'use_nav2': 'false',
    'use_ekf': 'false',
    'use_twist_mux': 'false',
}

NS = '/pole_front_cam'
DEPTH = NS + '/aligned_depth_to_color/image_raw'
INFO = NS + '/color/camera_info'
DEPTH_INFO = NS + '/aligned_depth_to_color/camera_info'  # image_transport가 찾는 자리 — 같은 RenderProduct라 color K와 같다


def generate_launch_description():
    best_effort = {f'qos_overrides.{t}.subscription.reliability': 'best_effort' for t in (DEPTH, INFO, DEPTH_INFO)}
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_system'), 'launch', 'ros_system.launch.py'])),
            launch_arguments={k: LaunchConfiguration(k) for k in ARGS}.items()),
        # Isaac 영상은 best_effort(SENSOR_DATA) — reliable로 구독하면 연결되지 않는다. 노드는 lazy 구독(points 구독자가 있어야 입력을 받음)
        Node(package='depth_image_proc', executable='point_cloud_xyz_node', name='pole_front_cam_points', output='screen',
             remappings=[('image_rect', DEPTH), (DEPTH_INFO, INFO), ('points', NS + '/depth/color/points')],
             parameters=[{'use_sim_time': True, **best_effort}]),
    ])
