"""📦 제공 패키지 기동 (실기용). 설계: documents/ros2-design/09-nodes.md "직접 작성 노드와 제공 노드"

  ros2 launch ros_system ros_system.launch.py
  ros2 launch ros_system ros_system.launch.py use_rtabmap:=false pole_cam_serial:=123456
  ros2 launch ros_system ros_system.launch.py use_nav2:=true            # config/nav2_params.yaml

  ros2 launch ros_system ros_system.launch.py mock_hardware:=true   # 실기 없이 controller_manager (명령 → 상태 되돌림)

기본으로 켜는 것: robot_state_publisher(URDF) + ros2_control(controller_manager + 컨트롤러),
                  RealSense ×2(pole_cam·gripper_cam), robot_localization EKF, RTAB-Map(위치추정 모드)
  - Nav2 (use_nav2:=true, config/nav2_params.yaml): 서버를 직접 띄운다 — nav2_bringup navigation_launch는
          velocity_smoother 입력을 /cmd_vel_nav로 고정해 rl_nav_correction·twist_mux를 건너뛰므로 안 씀.
          속도 체인: controller·behavior → /cmd_vel_nav → rl_nav_correction → /cmd_vel_corrected → twist_mux
                     → /cmd_vel_mux → velocity_smoother → /cmd_vel_smoothed → collision_monitor → /cmd_vel
          map→odom·/map은 RTAB-Map (faker는 map_server + 항등 TF)
TODO(권형중) 아직 자리만 있는 것 — 설정 패키지가 필요:
  - move_group (+ MTC capability): config/moveit 으로 해소 (use_moveit:=true) — pick_ik·octomap 센서(pole_cam 점구름)는 아직
테스트베드에서는 faker가 이 패키지들 대신 같은 토픽·액션을 낸다 (ros2 launch faker faker.launch.py).
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

CONTROLLERS = ['joint_state_broadcaster', 'imu_sensor_broadcaster', 'diff_drive_controller',
               'rl_arm_correction', 'arm_controller',  # 체인은 뒤에서부터 (rl_arm_correction → arm_controller)
               'gripper_controller']


def flag(name, default, description):
    return DeclareLaunchArgument(name, default_value=default, description=description)


def urdf_xacro(*args):
    """xacro 명령 — urdf:=<폴더>의 urdf.xacro, 메시 경로는 그 폴더 meshes/ (mesh_uri 인자가 없는 모델이면 xacro가 무시)"""
    name = LaunchConfiguration('urdf')
    return ['xacro ', share('urdf', name, 'urdf.xacro'),
            ' mesh_uri:=package://ros_system/urdf/', name, '/meshes', *args]


def camera(name, serial_arg, fps):
    # camera_namespace:='' + camera_name → /pole_cam/color/image_raw 처럼 설계의 토픽 이름에 맞춘다
    return Node(
        package='realsense2_camera', executable='realsense2_camera_node',
        namespace='', name=name, output='screen',
        condition=IfCondition(LaunchConfiguration('use_cameras')),
        parameters=[{
            'serial_no': LaunchConfiguration(serial_arg),
            'camera_name': name,
            'rgb_camera.color_profile': f'640x480x{fps}',
            'depth_module.depth_profile': f'640x480x{fps}',
            'align_depth.enable': True,
            'pointcloud.enable': True,
            'publish_tf': False,  # 카메라 프레임은 URDF(robot_state_publisher)가 낸다 — 하나만 남김
        }])


def share(*path):
    return PathJoinSubstitution([FindPackageShare('ros_system'), *path])


def spawner():
    # 컨트롤러별 파라미터·리매핑은 spawner가 컨트롤러 노드에 넘긴다
    # (ros2_control_node에 준 파일·리매핑은 controller_manager 자신에게만 적용됨).
    # 리매핑은 모든 컨트롤러에 같이 가지만 ~/cmd_vel·~/odom은 diff_drive, ~/imu는 imu broadcaster에만 있다.
    return Node(
        package='controller_manager', executable='spawner', output='screen',
        condition=IfCondition(LaunchConfiguration('use_ros2_control')),
        arguments=CONTROLLERS + [
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',  # 재시작 직후 DDS 탐색·빌드로 느려지면 60초 안에 못 붙고 FATAL → 여유 120초
            '--service-call-timeout', '30',  # 기동 직후 CPU가 몰리면 10초(기본) 안에 응답 못 할 때가 있다
            '--param-file', share('config', 'controllers.yaml'),
            '--controller-ros-args', '-r ~/cmd_vel:=/cmd_vel',
            '--controller-ros-args', '-r ~/odom:=/odom',
            '--controller-ros-args', '-r ~/imu:=/imu'])


def load_yaml(*path):
    """config/moveit/*.yaml의 /**/ros__parameters 안쪽을 dict로 (MoveIt SA 생성 launch와 같은 방식)"""
    full = os.path.join(get_package_share_directory('ros_system'), *path)
    with open(full) as f:
        data = yaml.safe_load(f)
    if isinstance(data, dict) and '/**' in data and isinstance(data['/**'], dict):
        inner = data['/**'].get('ros__parameters', data['/**'])
        return inner if isinstance(inner, dict) else data
    return data


def move_group_node():
    """진짜 move_group — use_moveit:=true일 때만 (실기). SRDF는 파일 내용 문자열로 넘긴다."""
    srdf_path = os.path.join(
        get_package_share_directory('ros_system'), 'config', 'moveit', 'beebot.srdf')
    with open(srdf_path) as f:
        robot_description_semantic = f.read()
    return Node(
        package='moveit_ros_move_group', executable='move_group', name='move_group', output='screen',
        condition=IfCondition(LaunchConfiguration('use_moveit')),
            parameters=[
                {'robot_description': ParameterValue(Command(urdf_xacro(
                    ' mock_hardware:=false port:=/dev/opencr')), value_type=str),
                 'robot_description_semantic': robot_description_semantic,
                 'robot_description_kinematics': load_yaml('config', 'moveit', 'kinematics.yaml'),
                 'robot_description_planning': load_yaml('config', 'moveit', 'joint_limits.yaml'),
                 'planning_pipelines': ['ompl'],
                 'default_planning_pipeline': 'ompl',
                 'ompl': load_yaml('config', 'moveit', 'ompl_planning.yaml')},
                load_yaml('config', 'moveit', 'moveit_controllers.yaml'),
            ])


def generate_launch_description():
    robot_description = ParameterValue(Command(urdf_xacro(
        ' mock_hardware:=', LaunchConfiguration('mock_hardware'),
        ' mock_effort:=', LaunchConfiguration('mock_effort'),
        ' port:=', LaunchConfiguration('port'))), value_type=str)
    ros2_control = IfCondition(LaunchConfiguration('use_ros2_control'))
    # spawner는 한 번만 만든다 — OnProcessExit target과 기동 목록이 같은 객체여야 한다
    spawner_action = spawner()
    return LaunchDescription([
        flag('urdf', 'beebot_mesh', '로봇 모델 = lv2_module6/urdf/<이 이름>/urdf.xacro (폴더 하나 = 모델 하나, 메시는 그 폴더 meshes/)'),
        flag('use_ros2_control', 'true', 'robot_state_publisher + controller_manager + 컨트롤러'),
        flag('mock_hardware', 'false', 'true면 BeebotSystemHardware가 실기 대신 명령을 상태로 되돌림'),
        flag('mock_effort', '0.0', 'mock일 때 팔 관절 effort [A]'),
        flag('port', '/dev/opencr', 'OpenCR 포트 (udev 고정 이름)'),
        flag('arm_correction_type', 'ml/RlArmCorrection', 'rl_arm_correction 컨트롤러 플러그인 (테스트베드 ML 대역: faker/FakeRlArmCorrection)'),
        flag('use_twist_mux', 'true', '바퀴 속도 명령 고르기 (자동 · 수동 · 비상정지 잠금)'),
        flag('use_cameras', 'true', 'RealSense 2대 (pole_cam·gripper_cam)'),
        flag('pole_cam_serial', "''", 'pole_cam serial_no (빈 값이면 첫 장치)'),
        flag('gripper_cam_serial', "''", 'gripper_cam serial_no'),
        flag('use_ekf', 'true', 'robot_localization EKF'),
        flag('use_rtabmap', 'true', 'RTAB-Map'),
        flag('localization', 'true', 'true=위치추정 모드(운영, rtabmap_db 지도로), false=매핑 모드(rtabmap_db를 새로 만듦)'),
        flag('rtabmap_db', '~/.ros/rtabmap.db', 'RTAB-Map 지도 DB — 매핑이 여기에 쓰고 위치추정이 여기서 읽음 (테스트베드: /ws/map/<이름>/rtabmap.db)'),
        flag('use_nav2', 'true', '진짜 Nav2 서버 (planner · controller · behavior · bt_navigator · velocity_smoother · collision_monitor)'),
        flag('nav2_params_file', share('config', 'nav2_params.yaml'), 'Nav2 파라미터 파일'),
        flag('use_moveit', 'false', 'true면 진짜 move_group (config/moveit) — spawner 완료 후 기동 (동시 기동 레이스 회피)'),

        Node(
            package='robot_state_publisher', executable='robot_state_publisher', output='screen',
            condition=ros2_control, parameters=[{'robot_description': robot_description}]),
        # controller_manager는 /robot_description 토픽(robot_state_publisher)에서 URDF를 받는다
        Node(
            package='controller_manager', executable='ros2_control_node', output='screen',
            condition=ros2_control,
            parameters=[share('config', 'controllers.yaml'),
                        {'rl_arm_correction.type': LaunchConfiguration('arm_correction_type')}]),
        spawner_action,
        # move_group은 spawner가 끝난 뒤에 (컨트롤러 로드 레이스 회피 — 동시 기동 시 spawner FATAL 실측)
        RegisterEventHandler(OnProcessExit(
            target_action=spawner_action,
            on_exit=[move_group_node()])),

        # 자동(/cmd_vel_corrected) · 수동(/cmd_vel_manual) 중 고르고 /state/emergency(← /cmd/e_stop, state_manager)면 막는다 → /cmd_vel_mux → velocity_smoother
        Node(
            package='twist_mux', executable='twist_mux', output='screen',
            condition=IfCondition(LaunchConfiguration('use_twist_mux')),
            parameters=[share('config', 'twist_mux.yaml')],
            remappings=[('cmd_vel_out', '/cmd_vel_mux')]),

        camera('pole_cam', 'pole_cam_serial', 30),
        camera('gripper_cam', 'gripper_cam_serial', 15),

        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
            condition=IfCondition(LaunchConfiguration('use_ekf')),
            parameters=[share('config', 'ekf.yaml')]),

        # 설계 3-1: 하향 카메라라 시각 오도메트리 끔, x·y·yaw만 (Reg/Force3DoF)
        # 격자(/map)는 body_filter가 자기 몸을 뺀 점구름으로 (Grid/Sensor 0 = scan_cloud). depth 영상(Grid/Sensor 1)으로 만들면
        # pole_cam에 찍힌 팔이 로봇 둘레 벽으로 남아 출발점이 늘 장애물 → planner가 매번 실패, 자동 탐사가 한 곳도 못 감 (실측)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py'])),
            condition=IfCondition(LaunchConfiguration('use_rtabmap')),
            launch_arguments={
                'rgb_topic': '/pole_cam/color/image_raw',
                'depth_topic': '/pole_cam/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/pole_cam/color/camera_info',
                'subscribe_scan_cloud': 'true',
                'scan_cloud_topic': '/pole_cam/depth/color/points_filtered',
                'frame_id': 'base_footprint',
                'odom_topic': '/odometry/filtered',
                # odom은 토픽 대신 TF(odom→base_footprint, EKF)에서 영상 시각으로 읽음 — 토픽 동기화는 odom 50Hz·큐 10(=0.2초)이라
                # 영상이 그보다 늦게 오면(카메라 지연·렌더 지연 ~1초 실측) 묶음이 안 맞아 'Did not receive data'로 지도를 못 만듦
                'odom_frame_id': 'odom',
                'visual_odometry': 'false',
                'approx_sync': 'true',
                'qos': '2',  # best effort 구독 — 카메라가 best effort(fake_camera SensorData)든 reliable이든 받음 (1=reliable이면 best effort 발행을 못 받음)
                'localization': LaunchConfiguration('localization'),
                'database_path': LaunchConfiguration('rtabmap_db'),
                'map_topic': '/map',  # 기본은 이름공간 안 /rtabmap/map — Nav2 global costmap · 시뮬레이터는 /map
                # 매핑 모드면 DB를 새로 (--delete_db_on_start). localization 값은 'true'·'True' 둘 다 올 수 있어 lower()
                'rtabmap_args': PythonExpression(["'--Reg/Force3DoF true --Grid/Sensor 0' + ('' if '",
                                                  LaunchConfiguration('localization'),
                                                  "'.lower() == 'true' else ' --delete_db_on_start')"]),
                'rviz': 'false',
                'rtabmap_viz': 'false',
            }.items()),

        *nav2_nodes(),
    ])


def nav2_nodes():
    """진짜 Nav2 서버 — 노드 이름 = nav2_params.yaml 최상위 키. 관리자 둘로 나눠 올림:
      lifecycle_manager_base       velocity_smoother · collision_monitor — 바퀴 명령 체인 (odom만 있으면 뜸 → 수동 조작은 지도 없이도)
      lifecycle_manager_navigation planner · controller · behavior · bt_navigator — map 필요 (RTAB-Map이 아직 map→odom을 못 내도
                                   이쪽만 실패하고 바퀴 체인은 산다. 하나로 묶으면 global_costmap 실패에 수동 조작까지 막혔음)
    리매핑만 우리 속도 체인에 맞춘다 (위 모듈 설명)."""
    on = IfCondition(LaunchConfiguration('use_nav2'))
    params = [LaunchConfiguration('nav2_params_file')]

    def server(package, executable, name=None, remaps=(), extra=()):
        return Node(package=package, executable=executable, name=name or executable, output='screen',
                    condition=on, parameters=params + list(extra), remappings=list(remaps))
    return [
        server('nav2_planner', 'planner_server'),
        server('nav2_controller', 'controller_server', remaps=[('cmd_vel', '/cmd_vel_nav')]),
        server('nav2_behaviors', 'behavior_server', remaps=[('cmd_vel', '/cmd_vel_nav')]),
        # 행동 트리 = config/navigate_to_pose.xml (기본 BT에서 후진만 0.10m로) — 파일 경로라 yaml이 아니라 여기서
        server('nav2_bt_navigator', 'bt_navigator',
               extra=[{'default_nav_to_pose_bt_xml': share('config', 'navigate_to_pose.xml')}]),
        server('nav2_velocity_smoother', 'velocity_smoother',
               remaps=[('cmd_vel', '/cmd_vel_mux'), ('cmd_vel_smoothed', '/cmd_vel_smoothed')]),
        server('nav2_collision_monitor', 'collision_monitor'),  # 입출력 토픽은 yaml (cmd_vel_in/out_topic)
        server('nav2_lifecycle_manager', 'lifecycle_manager', name='lifecycle_manager_base'),
        server('nav2_lifecycle_manager', 'lifecycle_manager', name='lifecycle_manager_navigation'),
    ]
