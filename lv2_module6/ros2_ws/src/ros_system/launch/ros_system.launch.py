"""📦 제공 패키지 기동 (실기용). 설계: documents/project/ros/09-nodes.md "직접 작성 노드와 제공 노드"

  ros2 launch ros_system ros_system.launch.py
  ros2 launch ros_system ros_system.launch.py use_slam:=false pole_front_cam_serial:=123456
  ros2 launch ros_system ros_system.launch.py use_nav2:=true            # config/nav2_params.yaml

  ros2 launch ros_system ros_system.launch.py mock_hardware:=true   # 실기 없이 controller_manager (명령 → 상태 되돌림)
  ros2 launch ros_system ros_system.launch.py localization:=false   # 매핑 — 지도를 새로 (끌 때 nvblox_map 파일에 저장)

기본으로 켜는 것: robot_state_publisher(URDF) + ros2_control(controller_manager + 컨트롤러),
                  RealSense ×2(pole_front_cam·gripper_cam), robot_localization EKF, SLAM(위치추정 모드)
  - SLAM (Jetson GPU — documents/jetson-orin-nano/CUVSLAM.md): 위쪽 노드는 map→odom TF · /map(latched) · 지도 원점 = 출발 자리만 받음
      cuVSLAM(scripts/cuvslam_slam.py) → map→odom · /cuvslam/odom(EKF)
      + nvblox(depth → 3D TSDF → 2D 점유 격자) → scripts/nvblox_map.py가 /map(latched)으로 · 지도 파일 nvblox.nvblx
  - Nav2 (use_nav2:=true, config/nav2_params.yaml): 서버를 직접 띄운다 — nav2_bringup navigation_launch는
          velocity_smoother 입력을 /cmd_vel_nav로 고정해 twist_mux를 건너뛰므로 안 씀.
          속도 체인: controller·behavior → /cmd_vel_nav → twist_mux
                     → /cmd_vel_mux → velocity_smoother → /cmd_vel_smoothed → collision_monitor → /cmd_vel
          map→odom·/map은 SLAM (faker는 map_server + fake_cuvslam)
TODO(권형중) 아직 자리만 있는 것 — 설정 패키지가 필요:
  - move_group (+ MTC capability): config/moveit 으로 해소 (use_moveit:=true) — IK는 pick_ik (kinematics.yaml) · octomap 센서는 sensors_3d.yaml (pole_front_cam points_no_floor — body_filter가 몸·바닥 뺀 것)
테스트베드에서는 faker가 이 패키지들 대신 같은 토픽·액션을 낸다 (ros2 launch faker faker.launch.py).
"""

import importlib.util
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node, SetRemap
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# spawner 하나가 순서대로 켜고, 하나가 실패하면 거기서 멈춘다 → 실패하기 쉬운 팔 체인(관절이 URDF 한계 밖이면
# rl_arm_correction 예외)을 맨 뒤에 — 그래야 팔이 안 떠도 바퀴·그리퍼는 뜬다 (2026-11-02 실기: 그리퍼가 안 열림)
CONTROLLERS = ['joint_state_broadcaster', 'imu_sensor_broadcaster', 'diff_drive_controller',
               'gripper_controller',
               'rl_arm_correction', 'arm_controller']  # 체인은 뒤에서부터 (rl_arm_correction → arm_controller)

# Jetson(ARM)의 realsense2_camera 4.57.7은 pointcloud 필터를 NEON 구현으로 등록해
# 파라미터 이름이 pointcloud.enable이 아니라 pointcloud__neon_.enable이다.
# pointcloud.enable만 주면 조용히 무시되어 publisher가 0개가 된다.
POINTCLOUD_PARAM = 'pointcloud__neon_' if os.uname().machine in ('aarch64', 'arm64') else 'pointcloud'

# 끌 때(Ctrl+C·/cmd/restart) 빨리 포기할 노드 — launch 기본은 SIGINT → 5초 → SIGTERM → 5초 → SIGKILL(최대 10초).
# Nav2 서버(costmap 정리 중)·lifecycle_manager·move_group이 SIGINT에 멈춰 늘 10초를 다 채웠다(CI 로그). 지킬 상태가 없으니 3+3초.
# 하드웨어(ros2_control_node — OpenCR 정지)·RealSense(USB 놓기)·nvblox(매핑이면 끌 때 지도 저장)는 기본 5+5초 그대로.
QUICK_STOP = {'sigterm_timeout': '3', 'sigkill_timeout': '3'}


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
            # 하드웨어 twist_state와 같은 값을 rl_arm_correction에 (다른 컨트롤러는 이 파라미터를 선언 안 해 무시)
            '--controller-ros-args', ['--param twist_state:=', LaunchConfiguration('twist_state')],  # -p는 spawner가 자기 --param-file로 읽음
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
        condition=IfCondition(LaunchConfiguration('use_moveit')), **QUICK_STOP,
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
                load_yaml('config', 'moveit', 'sensors_3d.yaml'),  # octomap 센서 — 점구름 장애물을 피해 계획 (이슈 #126)
            ])


def generate_launch_description():
    robot_description = ParameterValue(Command(urdf_xacro(
        ' mock_hardware:=', LaunchConfiguration('mock_hardware'),
        ' mock_effort:=', LaunchConfiguration('mock_effort'),
        ' port:=', LaunchConfiguration('port'),
        ' twist_state:=', LaunchConfiguration('twist_state'))), value_type=str)
    ros2_control = IfCondition(LaunchConfiguration('use_ros2_control'))
    # spawner는 한 번만 만든다 — OnProcessExit target과 기동 목록이 같은 객체여야 한다
    spawner_action = spawner()
    return LaunchDescription([
        flag('urdf', 'default', '로봇 모델 = lv2_module6/urdf/<이 이름>/urdf.xacro (폴더 하나 = 모델 하나, 메시는 그 폴더 meshes/)'),
        flag('use_ros2_control', 'true', 'robot_state_publisher + controller_manager + 컨트롤러'),
        flag('mock_hardware', 'false', 'true면 BeebotSystemHardware가 실기 대신 명령을 상태로 되돌림'),
        flag('mock_effort', '0.0', 'mock일 때 팔 관절 effort [A]'),
        flag('port', '/dev/opencr', 'OpenCR 포트 (udev 고정 이름)'),
        flag('twist_state', 'true', 'true면 joint2·3 상태 = 엔코더 + 중력 비틀림 (rl_arm_correction twist_state도 같이) · false = 엔코더 각 그대로 (강성 측정)'),
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
        flag('ekf_imu', 'true', 'EKF yaw 회전속도 = IMU 자이로 · false면 바퀴 (자이로 영점이 틀어지면 서 있어도 yaw가 돎 — 11/08 실기 0.2~2.3°/s)'),
        flag('use_slam', 'true', 'SLAM(cuVSLAM + nvblox) — false면 map→odom·/map 없음 (테스트베드 faker map:=course는 map_server가 대신)'),
        flag('localization', 'true', 'true=위치추정 모드(운영, 저장 지도로), false=매핑 모드(지도를 새로 만듦)'),
        flag('nvblox_map', '~/.ros/nvblox.nvblx', 'nvblox 지도 파일 — 매핑이면 끌 때 여기에 저장, 위치추정이면 시작할 때 여기서 불러옴'),
        flag('use_cmd_chain', 'true', '바퀴 명령 체인 (velocity_smoother · collision_monitor · lifecycle_manager_base) — use_nav2와 별개: 지도 없이(map:=none)도 수동 조작이 되려면 늘 필요'),
        flag('use_nav2', 'true', '진짜 Nav2 서버 (planner · controller · behavior · bt_navigator) — 바퀴 명령 체인(velocity_smoother · collision_monitor)은 use_cmd_chain'),
        flag('nav2_composition', 'true', 'Nav2 서버·관리자를 한 프로세스(nav2_container, 컴포넌트)로 — 메모리 절약 · false면 서버마다 프로세스 (예전)'),
        flag('py_helpers_one_process', 'true', 'python 보조 노드(pole_depth_gate · nvblox_map · rewind)를 한 프로세스(py_helpers)로 — 메모리 절약 · false면 따로 (예전)'),
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

        # 자동(/cmd_vel_nav) · 수동(/cmd_vel_manual) 중 고르고 /state/emergency(← /cmd/e_stop, state_manager)면 막는다 → /cmd_vel_mux → velocity_smoother
        Node(
            package='twist_mux', executable='twist_mux', output='screen',
            condition=IfCondition(LaunchConfiguration('use_twist_mux')),
            parameters=[share('config', 'twist_mux.yaml')],
            remappings=[('cmd_vel_out', '/cmd_vel_mux')]),

        camera('pole_front_cam', 'pole_front_cam_serial', 'pole_front_cam_profile',
               'pole_front_cam_pointcloud', 'use_pole_front_cam'),
        camera('gripper_cam', 'gripper_cam_serial', 'gripper_cam_profile',
               'gripper_cam_pointcloud', 'use_gripper_cam'),

        *[Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_ekf'), "' == 'true' and '",
                                                    LaunchConfiguration('ekf_imu'), f"' == '{imu}'"])),
            parameters=[share('config', 'ekf.yaml')] + ([] if imu == 'true' else [  # 바퀴 vx·vy + yaw 회전속도, IMU 안 씀
                {'imu0_config': [False] * 15, 'odom0_config': [False] * 6 + [True, True, False, False, False, True] + [False] * 3}]))
          for imu in ('true', 'false')],

        # SLAM = cuVSLAM(위치 · map→odom) + nvblox(지도 · /map) — cuvslam_nvblox_nodes() 설명
        *slam_nodes(),

        *nav2_nodes(),
    ])


# 관리자를 노드보다 이만큼 늦게 띄움 [s] (11-13): 막 뜬 노드에 바로 change_state를 보내면 요청은 처리되는데 응답이 관리자에게
#   못 돌아오는 경우가 있음 (응답 통로가 아직 안 이어짐 — 로그 60개 중 5개, 매번 시작 직후 첫 전환). Nav2 관리자는 응답을 무한정
#   기다려 그대로 멈춤 (reset.sh로만 풀림). 늦게 띄우면 연결이 끝난 뒤라 경합이 줄어듦 — 다시 나면 감시·재시도 스크립트를 더함
LIFECYCLE_DELAY_S = 4.0


def when(*names):
    """launch 인자가 모두 true일 때 켜는 조건 — '!이름'은 false일 때"""
    expr = []
    for n in names:
        expr += [' and ' if expr else '', "'", LaunchConfiguration(n.lstrip('!')), "'.lower() " + ('!=' if n[0] == '!' else '==') + " 'true'"]
    return IfCondition(PythonExpression(expr))


def nav2_nodes():
    """진짜 Nav2 서버 — 노드 이름 = nav2_params.yaml 최상위 키. 관리자 둘로 나눠 올림:
      lifecycle_manager_base       velocity_smoother · collision_monitor — 바퀴 명령 체인 (odom만 있으면 뜸 → 수동 조작은 지도 없이도)
      lifecycle_manager_navigation planner · controller · behavior · bt_navigator — map 필요 (SLAM이 아직 map→odom을 못 내도
                                   이쪽만 실패하고 바퀴 체인은 산다. 하나로 묶으면 global_costmap 실패에 수동 조작까지 막혔음)
    리매핑만 우리 속도 체인에 맞춘다 (위 모듈 설명).
    nav2_composition:=true(기본) — Nav2 표준(nav2_bringup use_composition)처럼 서버·관리자를 컴포넌트로 한 프로세스(nav2_container)에.
      프로세스마다 드는 rclcpp·DDS 바탕 메모리(관리자 하나 ~100 MB PSS)를 한 번만 냄 — 젯슨 8 GB가 바닥나 재부팅됨 (11-14).
      노드 이름·파라미터·리매핑·관리자 둘 나눔은 그대로. false = 예전처럼 서버마다 프로세스."""
    params = [LaunchConfiguration('nav2_params_file')]
    bt_xml = {'default_nav_to_pose_bt_xml': share('config', 'navigate_to_pose.xml'),
              'default_nav_through_poses_bt_xml': share('config', 'navigate_through_poses.xml')}
    # (패키지, 실행 파일, 컴포넌트 플러그인, 리매핑, 추가 파라미터, 켜는 인자)
    #   use_cmd_chain = 바퀴 명령 체인 — bringup이 지도 없음(map:=none)이면 use_nav2를 끄는데, 그때도 수동 조작은 돼야 한다
    servers = [
        ('nav2_planner', 'planner_server', 'nav2_planner::PlannerServer', [], [], 'use_nav2'),
        ('nav2_controller', 'controller_server', 'nav2_controller::ControllerServer', [('cmd_vel', '/cmd_vel_nav')], [], 'use_nav2'),
        ('nav2_behaviors', 'behavior_server', 'behavior_server::BehaviorServer', [('cmd_vel', '/cmd_vel_nav')], [], 'use_nav2'),
        # 행동 트리 = config/navigate_to_pose.xml · navigate_through_poses.xml
        #   (기본 BT에서 후진 0.10m · through-poses는 가까운 웨이포인트 제거 반경 0.15m) — 파일 경로라 yaml이 아니라 여기서
        ('nav2_bt_navigator', 'bt_navigator', 'nav2_bt_navigator::BtNavigator', [], [bt_xml], 'use_nav2'),
        ('nav2_velocity_smoother', 'velocity_smoother', 'nav2_velocity_smoother::VelocitySmoother',
         [('cmd_vel', '/cmd_vel_mux'), ('cmd_vel_smoothed', '/cmd_vel_smoothed')], [], 'use_cmd_chain'),
        ('nav2_collision_monitor', 'collision_monitor', 'nav2_collision_monitor::CollisionMonitor', [], [], 'use_cmd_chain'),  # 입출력 토픽은 yaml
    ]
    managers = [('lifecycle_manager_base', 'use_cmd_chain'), ('lifecycle_manager_navigation', 'use_nav2')]

    def server(package, executable, name, remaps, extra, on):
        return Node(package=package, executable=executable, name=name, output='screen', condition=when(on, '!nav2_composition'),
                    parameters=params + extra, remappings=remaps, **QUICK_STOP)

    def component(package, plugin, name, remaps, extra, on):
        return LoadComposableNodes(target_container='/nav2_container', condition=when(on, 'nav2_composition'), composable_node_descriptions=[
            ComposableNode(package=package, plugin=plugin, name=name, parameters=params + extra, remappings=remaps)])
    lm = ('nav2_lifecycle_manager', 'lifecycle_manager', 'nav2_lifecycle_manager::LifecycleManager')
    return [
        *[server(pkg, exe, exe, r, x, on) for pkg, exe, _, r, x, on in servers],
        Node(package='rclcpp_components', executable='component_container_isolated', name='nav2_container', output='screen',
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('nav2_composition'), "'.lower() == 'true' and ('",
                                                     LaunchConfiguration('use_nav2'), "'.lower() == 'true' or '",
                                                     LaunchConfiguration('use_cmd_chain'), "'.lower() == 'true')"])),
             **QUICK_STOP),
        *[component(pkg, plugin, exe, r, x, on) for pkg, exe, plugin, r, x, on in servers],
        TimerAction(period=LIFECYCLE_DELAY_S, actions=[
            *[server(lm[0], lm[1], name, [], [], on) for name, on in managers],
            *[component(lm[0], lm[2], name, [], [], on) for name, on in managers],
        ]),
    ]

def check_slam(context):
    """cuVSLAM·nvblox 설치 확인 — 없는 걸 띄우다 노드가 하나씩 죽는 대신 시작할 때 한 번에 알림"""
    if LaunchConfiguration('use_slam').perform(context).lower() != 'true':
        return []
    missing = [] if importlib.util.find_spec('cuvslam') else ['cuvslam (PyCuVSLAM — documents/jetson-orin-nano/CUVSLAM.md)']
    try:
        get_package_share_directory('nvblox_ros')
    except Exception:  # noqa: BLE001 — PackageNotFoundError
        missing.append('nvblox_ros (isaac_ros_nvblox — documents/jetson-orin-nano/CUVSLAM.md "nvblox")')
    if missing:
        raise RuntimeError('SLAM(cuVSLAM + nvblox)에 필요한 것이 없음: ' + ', '.join(missing) +
                           ' — GPU 없는 곳(테스트베드)은 faker fake_slam:=true(기본) 또는 use_slam:=false로')
    return []


def slam_nodes():
    """SLAM = cuVSLAM(위치) + nvblox(지도) — use_slam일 때. 위쪽이 받는 것: map→odom TF(cuVSLAM) · /map latched(nvblox_map) · 원점 = 출발 자리
      cuvslam_slam   pole_front_cam RGB-D → map→odom (루프 클로저) + /cuvslam/odom (EKF odom1)
      depth gate     scan_cam_gate 재사용 — nvblox에 줄 depth에서 카메라 0.5 m 안(자기 팔·손가락·폴대)을 0(측정 없음)으로.
                     pole_front_cam만 — gripper_cam(팔 끝)은 스캔 자세가 없어져(11-09) 지도에 안 넣음.
                     Nav2 costmap은 몸 뺀 점구름(body_filter)을 쓰지만 nvblox는 depth 영상을 받으므로 영상에서 뺌
      nvblox_node    depth(pole_front_cam) + TF(map→카메라) → TSDF → 2D 점유 격자 (config/nvblox.yaml). 매핑이면 끌 때 nvblox_map 파일에 저장
      nvblox_map     ~/static_occupancy_grid → /map (latched, 1 Hz) · 위치추정이면 시작할 때 저장 지도를 load_map"""
    on = IfCondition(LaunchConfiguration('use_slam'))
    loc = ["'", LaunchConfiguration('localization'), "'.lower() == 'true'"]
    nvblox_file = ["__import__('os').path.expanduser('", LaunchConfiguration('nvblox_map'), "')"]
    save_path = PythonExpression(["'' if ", *loc, " else ", *nvblox_file])  # 매핑일 때만 저장 (위치추정이면 불러온 지도를 안 덮어씀)
    load_path = PythonExpression([*nvblox_file, " if ", *loc, " else ''"])

    def gate_params(cam):
        return {'gate_by_pose': False, 'depth_min_range': 0.5, 'cloud_in': '',
                'depth_in': f'/{cam}/aligned_depth_to_color/image_raw',
                'depth_out': f'/{cam}/aligned_depth_to_color/image_nvblox'}
    nvblox_map_params = {'load_path': ParameterValue(load_path, value_type=str)}
    # py_helpers_one_process:=true(기본) — 아래 python 노드 셋(depth gate · nvblox_map · rewind)을 한 프로세스(py_helpers)로.
    #   python 노드마다 ~120 MB PSS(인터프리터·rclpy·DDS)를 한 번만 냄 (젯슨 8 GB 바닥 → 재부팅, 11-14). false = 예전처럼 따로
    separate = when('use_slam', '!py_helpers_one_process')

    def camera_remaps(i, cam):
        return [(f'camera_{i}/depth/image', f'/{cam}/aligned_depth_to_color/image_nvblox'),
                (f'camera_{i}/depth/camera_info', f'/{cam}/aligned_depth_to_color/camera_info'),
                (f'camera_{i}/color/image', f'/{cam}/color/image_raw'),
                (f'camera_{i}/color/camera_info', f'/{cam}/color/camera_info')]
    return [
        OpaqueFunction(function=check_slam),
        Node(package='ros_system', executable='cuvslam_slam', name='cuvslam_slam', output='screen', condition=on,
             parameters=[{'publish_map_tf': True}]),
        Node(package='ros_system', executable='scan_cam_gate', name='pole_depth_gate', output='screen', condition=separate,
             parameters=[gate_params('pole_front_cam')]),
        Node(package='nvblox_ros', executable='nvblox_node', name='nvblox_node', output='screen', condition=on,
             parameters=[share('config', 'nvblox.yaml'),
                         {'after_shutdown_map_save_path': ParameterValue(save_path, value_type=str)}],
             remappings=camera_remaps(0, 'pole_front_cam')),
        Node(package='ros_system', executable='nvblox_map', name='nvblox_map', output='screen', condition=separate,
             parameters=[nvblox_map_params]),
        # 주행 리와인드 — 실패(ABORTED) 뒤 실제 주행 궤적을 0.4 m 후진. 실기·테스트베드 공통 (SLAM 없어도 뜸)
        Node(package='ros_system', executable='rewind', name='rewind', output='screen',
             condition=IfCondition(PythonExpression(["not ('", LaunchConfiguration('use_slam'), "'.lower() == 'true' and '",
                                                     LaunchConfiguration('py_helpers_one_process'), "'.lower() == 'true')"]))),
        # 셋을 한 프로세스로 — 이름 없이 띄움 (name을 주면 __node 리매핑이 세 노드 모두에 걸림). 노드 이름은 인자로
        Node(package='ros_system', executable='py_helpers', output='screen', condition=when('use_slam', 'py_helpers_one_process'),
             arguments=['pole_depth_gate', 'nvblox_map', 'rewind'], parameters=[gate_params('pole_front_cam'), nvblox_map_params]),
    ]
