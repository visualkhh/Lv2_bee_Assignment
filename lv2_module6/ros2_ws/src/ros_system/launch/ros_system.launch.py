"""📦 제공 패키지 기동 (실기용). 설계: documents/project/ros/09-nodes.md "직접 작성 노드와 제공 노드"

  ros2 launch ros_system ros_system.launch.py
  ros2 launch ros_system ros_system.launch.py use_rtabmap:=false pole_front_cam_serial:=123456
  ros2 launch ros_system ros_system.launch.py use_nav2:=true            # config/nav2_params.yaml

  ros2 launch ros_system ros_system.launch.py mock_hardware:=true   # 실기 없이 controller_manager (명령 → 상태 되돌림)
  ros2 launch ros_system ros_system.launch.py cuvslam_odom:=true    # cuVSLAM 시각 오도메트리 → EKF (documents/jetson-orin-nano/CUVSLAM.md)

기본으로 켜는 것: robot_state_publisher(URDF) + ros2_control(controller_manager + 컨트롤러),
                  RealSense ×2(pole_front_cam·gripper_cam), robot_localization EKF, RTAB-Map(위치추정 모드)
  - Nav2 (use_nav2:=true, config/nav2_params.yaml): 서버를 직접 띄운다 — nav2_bringup navigation_launch는
          velocity_smoother 입력을 /cmd_vel_nav로 고정해 rl_nav_correction·twist_mux를 건너뛰므로 안 씀.
          속도 체인: controller·behavior → /cmd_vel_nav → rl_nav_correction → /cmd_vel_corrected → twist_mux
                     → /cmd_vel_mux → velocity_smoother → /cmd_vel_smoothed → collision_monitor → /cmd_vel
          map→odom·/map은 RTAB-Map (faker는 map_server + 항등 TF)
TODO(권형중) 아직 자리만 있는 것 — 설정 패키지가 필요:
  - move_group (+ MTC capability): config/moveit 으로 해소 (use_moveit:=true) — pick_ik·octomap 센서(pole_front_cam 점구름)는 아직
테스트베드에서는 faker가 이 패키지들 대신 같은 토픽·액션을 낸다 (ros2 launch faker faker.launch.py).
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

CONTROLLERS = ['joint_state_broadcaster', 'imu_sensor_broadcaster', 'diff_drive_controller',
               'rl_arm_correction', 'arm_controller',  # 체인은 뒤에서부터 (rl_arm_correction → arm_controller)
               'gripper_controller']

# Jetson(ARM)의 realsense2_camera 4.57.7은 pointcloud 필터를 NEON 구현으로 등록해
# 파라미터 이름이 pointcloud.enable이 아니라 pointcloud__neon_.enable이다.
# pointcloud.enable만 주면 조용히 무시되어 publisher가 0개가 된다.
POINTCLOUD_PARAM = 'pointcloud__neon_' if os.uname().machine in ('aarch64', 'arm64') else 'pointcloud'


def flag(name, default, description):
    return DeclareLaunchArgument(name, default_value=default, description=description)


def urdf_xacro(*args):
    """xacro 명령 — urdf:=<폴더>의 urdf.xacro, 메시 경로는 그 폴더 meshes/ (mesh_uri 인자가 없는 모델이면 xacro가 무시)"""
    name = LaunchConfiguration('urdf')
    return ['xacro ', share('urdf', name, 'urdf.xacro'),
            ' mesh_uri:=package://ros_system/urdf/', name, '/meshes', *args]


def camera(name, serial_arg, profile_arg, pointcloud_arg, use_arg):
    # camera_namespace:='' + camera_name → /pole_front_cam/color/image_raw 처럼 설계의 토픽 이름에 맞춘다
    # use_cameras는 전체 스위치, use_arg는 카메라별 스위치 (한 대만 연결한 상태에서 나머지를 끌 수 있다).
    enabled = PythonExpression([
        "'", LaunchConfiguration('use_cameras'), "'.lower() == 'true' and '",
        LaunchConfiguration(use_arg), "'.lower() == 'true'"])
    return Node(
        package='realsense2_camera', executable='realsense2_camera_node',
        namespace='', name=name, output='screen',
        condition=IfCondition(enabled),
        respawn=True, respawn_delay=2.0,  # 카메라를 나중에 꽂아도 다시 잡도록
        parameters=[{
            # launch 인자로 들어온 serial은 숫자처럼 보여도 ROS 파라미터는 string이어야 한다.
            'serial_no': ParameterValue(LaunchConfiguration(serial_arg), value_type=str),
            'camera_name': name,
            # 네트워크가 터지지 않게 낮춘 기본값. 640x360x15는 D435/D435i 지원 프로파일.
            'rgb_camera.color_profile': LaunchConfiguration(profile_arg),
            'depth_module.depth_profile': LaunchConfiguration(profile_arg),
            # RealSense 기본은 SYSTEM_DEFAULT(reliable) — 영상은 best_effort가 맞다.
            # SENSOR_DATA = best_effort · volatile · keep_last(5) (드라이버가 depth를 1로 바꾸는 파라미터는 없다).
            'color_qos': 'SENSOR_DATA',
            'depth_qos': 'SENSOR_DATA',
            f'{POINTCLOUD_PARAM}.pointcloud_qos': 'SENSOR_DATA',
            # 점군 밀도를 1/4로 줄인다 (depth 640x360 -> 320x180). RGB 해상도는 유지.
            'decimation_filter.enable': LaunchConfiguration('camera_decimation_enable'),
            'decimation_filter.filter_magnitude': ParameterValue(
                LaunchConfiguration('camera_decimation_magnitude'), value_type=int),
            'align_depth.enable': True,
            f'{POINTCLOUD_PARAM}.enable': LaunchConfiguration(pointcloud_arg),
            # 적외선 스트림은 쓰지 않는다. 두 카메라를 같은 USB 버스에 물리면
            # IR 영상까지 대역폭을 나눠 써서 depth가 'overflow video frame'으로 막힌다.
            'enable_infra1': False,
            'enable_infra2': False,
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
        flag('urdf', 'default', '로봇 모델 = lv2_module6/urdf/<이 이름>/urdf.xacro (폴더 하나 = 모델 하나, 메시는 그 폴더 meshes/)'),
        flag('use_ros2_control', 'true', 'robot_state_publisher + controller_manager + 컨트롤러'),
        flag('mock_hardware', 'false', 'true면 BeebotSystemHardware가 실기 대신 명령을 상태로 되돌림'),
        flag('mock_effort', '0.0', 'mock일 때 팔 관절 effort [A]'),
        flag('port', '/dev/opencr', 'OpenCR 포트 (udev 고정 이름)'),
        flag('arm_correction_type', 'ml/RlArmCorrection', 'rl_arm_correction 컨트롤러 플러그인 (테스트베드 ML 대역: faker/FakeRlArmCorrection)'),
        flag('use_twist_mux', 'true', '바퀴 속도 명령 고르기 (자동 · 수동 · 비상정지 잠금)'),
        flag('use_cameras', 'true', 'RealSense 2대 (pole_front_cam·gripper_cam) 전체 스위치'),
        flag('use_pole_front_cam', 'true', 'pole_front_cam 개별 스위치 (use_cameras와 AND)'),
        flag('use_gripper_cam', 'true', 'gripper_cam 개별 스위치 (use_cameras와 AND)'),
        flag('pole_front_cam_serial', "''", 'pole_front_cam serial_no (빈 값이면 첫 장치)'),
        flag('gripper_cam_serial', "''", 'gripper_cam serial_no'),
        flag('pole_front_cam_profile', '640x360x15', 'pole_front_cam RGB/depth 프로파일 (WxHxFPS)'),
        flag('gripper_cam_profile', '640x360x15', 'gripper_cam RGB/depth 프로파일 (WxHxFPS)'),
        flag('camera_decimation_enable', 'true', 'RealSense depth decimation 필터 (점군 밀도 감소)'),
        flag('camera_decimation_magnitude', '2', 'decimation 배율 (2=1/4 점, 3=1/9 점)'),
        flag('pole_front_cam_pointcloud', 'true', 'pole_front_cam PointCloud2 발행 (body_filter/Nav2용)'),
        flag('gripper_cam_pointcloud', 'false', 'gripper_cam PointCloud2 발행 (grasp_scorer용, 필요할 때만 true)'),
        flag('use_ekf', 'true', 'robot_localization EKF'),
        flag('use_rtabmap', 'true', 'RTAB-Map'),
        flag('slam', 'rtabmap', 'rtabmap | cuvslam (cuVSLAM 단독 시험: map→odom만, /map 격자 없음 — use_nav2:=false로)'),
        flag('cuvslam_odom', 'false', 'true면 cuVSLAM 시각 오도메트리(/cuvslam/odom)를 EKF에 — slam:=rtabmap과 함께 (지도는 RTAB-Map)'),
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

        camera('pole_front_cam', 'pole_front_cam_serial', 'pole_front_cam_profile',
               'pole_front_cam_pointcloud', 'use_pole_front_cam'),
        camera('gripper_cam', 'gripper_cam_serial', 'gripper_cam_profile',
               'gripper_cam_pointcloud', 'use_gripper_cam'),

        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
            condition=IfCondition(LaunchConfiguration('use_ekf')),
            parameters=[share('config', 'ekf.yaml')]),

        # 설계 3-1: 하향 카메라라 시각 오도메트리 끔, x·y·yaw만 (Reg/Force3DoF)
        # 격자(/map)는 body_filter가 자기 몸을 뺀 점구름으로 (Grid/Sensor 0 = scan_cloud). depth 영상(Grid/Sensor 1)으로 만들면
        # pole_front_cam에 찍힌 팔이 로봇 둘레 벽으로 남아 출발점이 늘 장애물 → planner가 매번 실패, 자동 탐사가 한 곳도 못 감 (실측)
        # 초기 위치: RTAB-Map은 이름공간 안 /rtabmap/initialpose를 구독 → 표준 /initialpose로 (RViz '2D Pose Estimate' · 시뮬레이터 지도
        # 우클릭 '초기 위치 지정'). 위치추정 모드에서 위치를 잃었을 때 "여기쯤"이라고 알려 줌
        # 카메라 두 대: pole_front_cam(앞 · 늘) + gripper_cam(지도 만들 때 팔을 스캔 자세에 고정 — 왼쪽 뒤를 봄, config/scan_pose.yaml)
        #   pole_front_cam      rgb · depth · info → rgbd_sync → rgbd_image0
        #   gripper_cam   rgb · depth(scan_cam_gate: 스캔 자세에 멈춰 있을 때만 그대로, 아니면 0) · info → rgbd_sync → rgbd_image1
        #   rgbdx_sync    둘을 묶어 rgbd_images → rtabmap (rgbd_cameras 0 = 묶음 메시지) + scan_cloud (몸 뺀 pole_front_cam 점구름)
        # rtabmap_launch는 카메라 두 대 인자가 없어 노드를 직접 띄움 — 파라미터·리매핑은 rtabmap_launch와 같게 (이름공간 rtabmap · 노드 rtabmap:
        #   state_manager가 /rtabmap/rtabmap/set_mode_* · load_database를 부름)
        *rtabmap_nodes(),

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

def rtabmap_nodes():
    """RTAB-Map (설명은 generate_launch_description 주석) — use_rtabmap일 때만"""
    on = IfCondition(PythonExpression(["'", LaunchConfiguration('use_rtabmap'), "'.lower() == 'true' and '",
                                       LaunchConfiguration('slam'), "' == 'rtabmap'"]))
    loc = PythonExpression(["'", LaunchConfiguration('localization'), "'.lower() == 'true'"])
    # 매핑 모드면 DB를 새로 (--delete_db_on_start). localization 값은 'true'·'True' 둘 다 올 수 있어 lower()
    # 클로저(루프·근접)를 엄격하게 — 제자리 회전 때 아래를 보는 pole_front_cam이 같은 모양 미로 벽만 보며 잘못 맞춘 클로저를
    # 받아들여, odom은 맞는데(실제와 2cm·0.7°) map→odom이 0.21m·−7.9° 틀어져 지도가 깨졌음 (실측 2026-10-19):
    #   Vis/MinInliers 20→30→50→35 (특징점 대응 더 요구 — 아래) · RGBD/OptimizeMaxError 3→1.5 (odom과 많이 어긋나는 클로저 거부)
    #   Reg/Strategy 0→2 (시각으로 찾고 점구름 ICP로 다시 맞춤 — scan_cloud = 몸 뺀 pole_front_cam 점구름)
    #   RGBD/AngularUpdate 0.1→0.3 rad (제자리 회전 중 비슷한 노드가 줄줄이 쌓이지 않게)
    #   RGBD/ProximityPathMaxNeighbors 0 — Reg/Strategy 2면 RTAB-Map이 1로 켜는 '점구름만으로 근접 검출'을 끔:
    #     통로에선 양쪽 벽 점구름이 옆으로는 맞아도 통로 방향으로는 미끄러져, 통로 방향으로 0.18m 튄 보정을 받아들였음 (실측)
    #   Vis/MinDepth · Kp/MinDepth 0.5 — pole_front_cam 화면 가운데에 로봇 자기 팔이 찍힘(카메라에서 0.2~0.35m). 팔은 로봇과 같이 돌아
    #     그 특징점은 영상에서 안 움직임 → 클로저가 회전을 덜 잡음 (실측: 위치 1~2cm는 맞는데 회전 평균 2~6°, 늘 실제보다 작게).
    #     바닥·벽은 화면 맨 아래도 0.59m 이상 (카메라 높이 0.42 · 18° 하향 · 세로 화각 ~54°) → 0.5m 안 특징점만 버림
    #   Grid 바닥 판정을 법선 → 높이로: 바닥(base_footprint) 위 3cm까지 바닥 · 3~60cm 장애물 (Nav2 costmap과 같은 기준 — 경기장 바닥은 평평)
    #     법선(이웃 20점 기울기 45°)은 gripper_cam이 비스듬히 본 먼 바닥(성기고 depth 잡음 큼)을 장애물로 찍어
    #     지도 통로가 통째로 검게 칠해졌음 (실측: 저장 지도 장애물 3878칸 · 빈 칸 564칸)
    #   Rtabmap/LoopThr 0.11→0.2→0.15 · Vis/MinInliers 30→50→35 (카메라 두 대·팔 특징점 제거 뒤 다시 풂 — 0.2·50이면 같은 곳에 돌아와도 클로저를 하나도 안 받음) — 그래도 같은 모양 벽·바닥에서 루프 클로저 12개를 받아 지도가 15.8° 돌았음
    #     (실측 2026-10-19 — 실제 경기장도 미로라 같은 착각이 날 수 있어 판정 자체를 엄격하게)
    rtabmap_args = PythonExpression(["'--Reg/Force3DoF true --Grid/Sensor 0 --Vis/MinInliers 35 "
                      "--RGBD/OptimizeMaxError 1.5 --Reg/Strategy 2 --RGBD/AngularUpdate 0.3 "
                      "--RGBD/ProximityPathMaxNeighbors 0 --Rtabmap/LoopThr 0.15 "
                      "--Vis/MinDepth 0.5 --Kp/MinDepth 0.5 "
                      "--Grid/NormalsSegmentation false --Grid/MaxGroundHeight 0.03 --Grid/MaxObstacleHeight 0.6' + ('' if '",
                      LaunchConfiguration('localization'),
                      "'.lower() == 'true' else ' --delete_db_on_start')"])
    # best effort 구독 — 카메라가 best effort(fake_camera SensorData)든 reliable이든 받음.
    # queue 10은 컬러·depth·두 카메라 approximate sync용이다. 여기를 1로 줄이면
    # 한쪽 프레임이 먼저 도착할 때 짝을 못 찾아 RTAB-Map이 'Did not receive data'로 멈출 수 있다.
    qos = {'qos': 2, 'qos_camera_info': 2}

    def rgbd_sync(name, rgb, depth, info, out):
        return Node(package='rtabmap_sync', executable='rgbd_sync', name=name, namespace='rtabmap', output='screen', condition=on,
                    parameters=[{'approx_sync': True, 'topic_queue_size': 10, 'sync_queue_size': 10, **qos}],
                    remappings=[('rgb/image', rgb), ('depth/image', depth), ('rgb/camera_info', info), ('rgbd_image', out)])
    return [
        # cuVSLAM (scripts/cuvslam_slam.py): slam:=cuvslam이면 단독 SLAM(map→odom), cuvslam_odom:=true면 시각 오도메트리만 → EKF
        Node(package='ros_system', executable='cuvslam_slam', name='cuvslam_slam', output='screen',
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_rtabmap'), "'.lower() == 'true' and ('",
                                                     LaunchConfiguration('slam'), "' == 'cuvslam' or '",
                                                     LaunchConfiguration('cuvslam_odom'), "'.lower() == 'true')"])),
             parameters=[{'publish_map_tf': ParameterValue(PythonExpression(["'", LaunchConfiguration('slam'), "' == 'cuvslam'"]),
                                                           value_type=bool)}]),
        Node(package='ros_system', executable='scan_cam_gate', name='scan_cam_gate', output='screen', condition=on,
             parameters=[share('config', 'scan_pose.yaml'),
                         {'depth_in': '/gripper_cam/aligned_depth_to_color/image_raw',
                          'depth_out': '/gripper_cam/aligned_depth_to_color/image_scan',
                          'cloud_in': '/gripper_cam/depth/color/points', 'cloud_out': '/gripper_cam/depth/color/points_scan',
                          'min_range': 0.5}]),
        # 점구름 둘(몸 뺀 pole_front_cam · 스캔 자세일 때만 gripper_cam)을 base_footprint 하나로 → RTAB-Map scan_cloud (격자 /map · ICP)
        Node(package='rtabmap_util', executable='point_cloud_aggregator', name='scan_cloud_merge', namespace='rtabmap', output='screen',
             condition=on,
             parameters=[{'count': 2, 'frame_id': 'base_footprint', 'approx_sync': True, 'topic_queue_size': 10,
                          'sync_queue_size': 10, 'qos': 2}],
             remappings=[('cloud1', '/pole_front_cam/depth/color/points_filtered'), ('cloud2', '/gripper_cam/depth/color/points_scan'),
                         ('combined_cloud', '/rtabmap/scan_cloud')]),
        rgbd_sync('pole_rgbd_sync', '/pole_front_cam/color/image_raw', '/pole_front_cam/aligned_depth_to_color/image_raw',
                  '/pole_front_cam/color/camera_info', '/rtabmap/rgbd_image0'),
        rgbd_sync('gripper_rgbd_sync', '/gripper_cam/color/image_raw', '/gripper_cam/aligned_depth_to_color/image_scan',
                  '/gripper_cam/color/camera_info', '/rtabmap/rgbd_image1'),
        Node(package='rtabmap_sync', executable='rgbdx_sync', name='rgbdx_sync', namespace='rtabmap', output='screen', condition=on,
             parameters=[{'rgbd_cameras': 2, 'approx_sync': True, 'topic_queue_size': 10, 'sync_queue_size': 10, 'qos': 2}],
             remappings=[('rgbd_image0', '/rtabmap/rgbd_image0'), ('rgbd_image1', '/rtabmap/rgbd_image1'),
                         ('rgbd_images', '/rtabmap/rgbd_images')]),
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', namespace='rtabmap', output='screen', condition=on,
             arguments=[rtabmap_args],
             parameters=[{
                 'subscribe_depth': False, 'subscribe_rgb': False, 'subscribe_rgbd': True, 'rgbd_cameras': 0,
                 'subscribe_scan_cloud': True,
                 'frame_id': 'base_footprint', 'map_frame_id': 'map', 'odom_frame_id': 'odom', 'publish_tf': True,
                 # odom은 토픽 대신 TF(odom→base_footprint, EKF)에서 영상 시각으로 읽음 — 토픽 동기화는 odom 50Hz·큐 10(=0.2초)이라
                 # 영상이 그보다 늦게 오면(카메라 지연·렌더 지연 ~1초 실측) 묶음이 안 맞아 'Did not receive data'로 지도를 못 만듦
                 # odom(TF) 링크 분산 — rtabmap_launch 기본 회전 0.01(σ 5.7°)이라 클로저가 회전을 마음대로 끌고 감.
                 # 실측 (회전 시험, 2026-10-19): EKF(IMU) yaw는 80초 끝 오차 0.15° · 클로저는 위치 1~2cm로 정확하나 회전이 평균 2~3°, 최대 6.6° 틀림
                 # → map→odom −6.5°로 지도가 비틀림. 회전은 odom을 믿게(σ 0.6°) — 어긋나는 클로저는 OptimizeMaxError가 거부
                 'odom_tf_angular_variance': 0.0001, 'odom_tf_linear_variance': 0.001,
                 'odom_sensor_sync': True, 'wait_for_transform': 0.2, 'approx_sync': True,
                 'topic_queue_size': 10, 'sync_queue_size': 10,
                 'qos_image': 2, 'qos_scan': 2, 'qos_odom': 2, 'qos_camera_info': 2,
                 'database_path': LaunchConfiguration('rtabmap_db'),
                 # RTAB-Map 파라미터는 문자열이어야 함 — 'true'를 그냥 넘기면 launch가 bool로 바꿔 rtabmap이 죽음 (InvalidParameterType, 실측)
                 'Mem/IncrementalMemory': ParameterValue(PythonExpression(["'false' if ", loc, " else 'true'"]), value_type=str),
                 'Mem/InitWMWithAllNodes': ParameterValue(PythonExpression(["'true' if ", loc, " else 'false'"]), value_type=str),
             }],
             remappings=[('rgbd_images', '/rtabmap/rgbd_images'), ('scan_cloud', '/rtabmap/scan_cloud'),  # 두 카메라 합친 점구름
                         ('odom', '/odometry/filtered'), ('map', '/map'),  # 기본은 이름공간 안 /rtabmap/map — Nav2 · 시뮬레이터는 /map
                         # 초기 위치: 표준 /initialpose로 (RViz '2D Pose Estimate' · 시뮬레이터 지도 우클릭 '초기 위치 지정')
                         ('initialpose', '/initialpose')]),
    ]
