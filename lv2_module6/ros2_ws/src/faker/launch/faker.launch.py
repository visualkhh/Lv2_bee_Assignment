"""테스트베드 실행: 진짜 bringup(🛠 노드 + ros_system 📦 + 지도 처리 — 실기와 같은 launch) + 도커에 없는 📦의 대역 + 호스트 파일 연동.
  bringup에는 대역에 맞는 인자만 넘김 (use_cameras:=false · mock_hardware · arm_correction_type · ML use_*) — 지도 map:=slam|saved 처리는 bringup 것 그대로.

  ros2 launch faker faker.launch.py
  ros2 launch faker faker.launch.py mock_effort:=1.5        # 팔 전류 과다 → arm_safety_monitor 접촉 감지
  ros2 launch faker faker.launch.py mock_hardware:=true     # OpenCR 없이 (fake_opencr 안 띄움 — controller_manager mock이 명령을 상태로 되돌림, 물리 없음)
  ros2 launch faker faker.launch.py ml:=real                # ML 대역 대신 진짜 ml 노드·컨트롤러 (GPU·모델 없이도 돌 때만)
  ros2 launch faker faker.launch.py grasp_scorer:=real      # 하나만 진짜로 (rl_arm_correction도 같음, 기본 = ml 값)
  지도 map:= (fake_opencr 충돌 판정은 시뮬레이터 월드 — 로봇 지도와 별개):
  ros2 launch faker faker.launch.py                         # slam (기본): 센서로 새 지도 — /pole_front_cam/depth/color/points → /map
  지도 한 벌 = 폴더 하나: /ws/map/<map_name>/ nvblox.nvblx (FAKESLAM1 — 이 노드가 씀) · map.pgm · map.yaml (사람이 보는 미리보기)
  ros2 launch faker faker.launch.py map:=saved              # 저장 지도로 위치추정 — nvblox.nvblx를 읽어 /map
                                                             # GPU 있는 곳에서 진짜 SLAM을 쓰려면 fake_slam:=false (ros_system check_slam)

실제로 띄우는 📦 (ros_system):
  robot_state_publisher(URDF) · controller_manager + control/BeebotSystemHardware(가상 시리얼 ↔ fake_opencr) + 컨트롤러 · robot_localization EKF
  · 진짜 Nav2 (planner · controller · behavior · bt_navigator · velocity_smoother · collision_monitor)
fake_slam:=true (기본) — 실기 SLAM(cuVSLAM + nvblox)의 대신:
  map→odom         fake_cuvslam: 진짜 위치(fake_opencr ground_truth)로 보정 · mock_hardware:=true면 입력이 없어 idle이라 항등 TF가 대신
  fake_nvblox      MAPPING: /pole_front_cam/depth/color/points + TF(map←camera) → 점유 격자 → /map(transient_local) · nvblox.nvblx 저장
                   LOCALIZATION: 저장한 nvblox.nvblx → /map · ESDF slice (/nvblox_node/combined_esdf_pointcloud)
대역 (도커 이미지에 없는 📦 · GPU·모델이 없는 ML):
  ML 대역 (ml 패키지와 1:1 — 같은 노드 이름·토픽, 하나씩 진짜/대역 선택):
    fake_grasp_scorer       → 노드 grasp_scorer
    FakeRlArmCorrection     → 컨트롤러 rl_arm_correction 자리 (플러그인, Δ=0)
  fake_camera ×2   realsense2_camera (pole_front_cam, gripper_cam) — 시뮬레이터가 debug/input/hardware/camera/에 쓴 프레임.
                   시뮬레이터 없이(headless)면 회색 영상 + 현재 /map을 ray-casting한 depth (costmap·collision_monitor가 벽을 봄)
  fake_opencr      OpenCR (기본. mock_hardware:=true면 안 뜸 — 유일한 OpenCR, 미끄럼·충돌·통신 끊김, 진짜 위치 ~/ground_truth 발행 · 환경은 파라미터 · /cmd/fake_opencr/set_pose)
  move_group       진짜 (bringup use_moveit:=true 기본 — 실기와 같음, MTC 대역 fake_moveit 삭제됨)
호스트 파일 연동 (통제실·시뮬레이션):
  monitor_manager  토픽 → debug/topic/<토픽>/message (JSON) · image.jpg
  input_manager    debug/input/topic/<토픽>/message (JSON) → 토픽
ml:=real이면 진짜 ml 노드를 띄운다 — 모델이 없거나 learning_enabled:=false면 기본 동작(Δ=0·규칙)으로 돌아야 한다.
"""

import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def arg(name, default, description):
    return DeclareLaunchArgument(name, default_value=default, description=description)


def include(package, launch_file, args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])),
        launch_arguments=args.items())


ML_NODES = ('grasp_scorer',)   # ml 패키지 노드 — 대역 fake_<이름>
ML = ML_NODES + ('rl_arm_correction',)                              # + 컨트롤러 플러그인


def fake(executable, name, overrides=None, respawn=False, condition=None):
    # 파라미터: config/faker.yaml(노드 이름별) → launch 인자(overrides)가 덮어씀
    config = PathJoinSubstitution([FindPackageShare('faker'), 'config', 'faker.yaml'])
    return Node(package='faker', executable=executable, name=name, output='screen', parameters=[config, overrides or {}],
                respawn=respawn, respawn_delay=1.0, condition=condition)


# 테스트베드 SLAM 대역 — fake_cuvslam + fake_nvblox (MAPPING은 센서로 지도를 쌓는다)
FAKE_SLAM = PythonExpression(["'", LaunchConfiguration('fake_slam'), "' == 'true'"])


def default_vlm_endpoint():
    """호스트 ai-service 주소 — macOS Docker Desktop은 host.docker.internal이 호스트를 가리키지만, 리눅스 network_mode: host에선
    그 이름이 아예 안 풀림(실측: 요청이 한 번도 안 가고 템플릿 폴백 → '계획 보류'). host망이면 호스트 = 127.0.0.1"""
    try:
        socket.gethostbyname('host.docker.internal')
        return 'http://host.docker.internal:8000'
    except OSError:
        return 'http://127.0.0.1:8000'


def is_real(name):
    return PythonExpression(["'", LaunchConfiguration(name), "' == 'real'"])


def generate_launch_description():
    return LaunchDescription([
        arg('command', '빨간 퍽을 목표 구역으로 옮겨', 'user_input 텍스트 명령 (빈 값이면 안 보냄)'),
        arg('ml', 'fake', 'fake = ML 대역(도커·CI 기본, GPU 불필요) · real = 진짜 ml 노드·컨트롤러 — 아래 넷의 기본값'),
        *[arg(n, LaunchConfiguration('ml'), f'{n}: fake | real (기본 = ml)') for n in ML],
        arg('learning_enabled', 'true', '학습 노드 on/off (ml:=real일 때)'),
        arg('vlm_endpoint', default_vlm_endpoint(),
            '호스트 ai-service 주소 (비면 템플릿 고정) — 기본: host.docker.internal이 풀리면(macOS) 그것, 아니면(리눅스 host망) 127.0.0.1'),
        arg('course', 'dice_maze', '시뮬레이터 월드 코스 = faker/world/<이 이름>.yaml (fake_opencr 충돌 모델 — 로봇 지도 아님) — 기본 dice_maze · 옛 미로: twin_closed_maze · Isaac 학습 미로: isaac_maze'),
        arg('start_pose', '[0.734, 0.950, 0.0]', 'mission_manager 출발 자세 arena [x, y, yaw] (비면 plan.yaml) — 시뮬레이터는 +X를 보고 켬'),
        arg('map', 'none', 'none = 지도 없음(기본) · slam = 센서로 새 지도 · saved = 저장한 nvblox.nvblx로 위치추정'),
        arg('operation', 'MANUAL', '초기 조작 모드: MANUAL(기본) | AUTO | EXPLORE | TEACH'),
        arg('fake_slam', 'true', 'true = 테스트베드 SLAM 대역(fake_cuvslam · fake_nvblox — 센서로 지도) · false = 진짜 cuVSLAM·nvblox (GPU)'),
        arg('map_dir', '/ws/map', '지도 폴더 (호스트 ros2_ws/map 바인드) — 지도마다 <map_name>/ nvblox.nvblx · map.pgm · map.yaml'),
        arg('map_name', '', '지도 폴더 이름 — slam: 비우면 날짜_시각으로 새로 · saved: 비우면 default (nvblox.nvblx 없으면 가장 최근 — bringup이 정함)'),
        arg('use_monitor', 'true', 'false = monitor_manager 끔 (호스트 가벼운 실행용 — PASS 판정에서 기록 검사 제외)'),
        arg('mock_hardware', 'false', 'false(기본) = controller_manager가 가상 시리얼(/dev/opencr)로 fake_opencr와 통신 · true = OpenCR 없이 mock (물리 없음)'),
        arg('mock_effort', '0.0', '팔 관절 전류 [A] (mock_hardware:=true일 때만 — fake_opencr면 시뮬레이터 [팔 붙잡기])'),
        arg('urdf', 'default', '로봇 모델 = lv2_module6/urdf/<이 이름>/urdf.xacro (폴더 하나 = 모델 하나, 메시는 그 폴더 meshes/) — 시뮬레이터 [로봇 모델]이 재시작 인자로 바꿈'),
        arg('use_gz_cameras', 'false', 'true = Gazebo ros_gz_bridge가 카메라 토픽 직접 발행 — fake_camera(파일 브리지) 끔'),
        arg('debug_dir', '/debug', '호스트와 공유하는 폴더 (topic/ · input/ · output/)'),

        # 진짜 bringup 그대로 (🛠 노드 + ros_system 📦 + 지도 처리) — 대역에 맞는 인자만 덮어씀
        include('bringup', 'bringup.launch.py', {
            'command': LaunchConfiguration('command'),
            'start_pose': LaunchConfiguration('start_pose'),
            'learning_enabled': LaunchConfiguration('learning_enabled'),
            'vlm_endpoint': LaunchConfiguration('vlm_endpoint'),
            'map': LaunchConfiguration('map'),
            'operation': LaunchConfiguration('operation'),
            'fake_slam': LaunchConfiguration('fake_slam'),
            'map_dir': LaunchConfiguration('map_dir'),
            'map_name': LaunchConfiguration('map_name'),
            'urdf': LaunchConfiguration('urdf'),
            'mock_hardware': LaunchConfiguration('mock_hardware'),
            'mock_effort': LaunchConfiguration('mock_effort'),
            'use_cameras': 'false',  # RealSense 대신 fake_camera (아래)
            'grasp_check': 'false',  # fake_opencr·mock 그리퍼는 퍽 없이 끝까지 닫힘 — 잡기 판정을 하면 늘 빈손
            # /cmd/restart all (state_manager): 다시 띄우기 전에 빌드 (고친 코드 반영 — 예전 test-faker 0 재시작과 같게) · 출력은 test-faker 로그로
            'restart_prepare': 'build-ws',
            'restart_log': '/tmp/faker.log',
            'arm_correction_type': PythonExpression(
                ["'ml/RlArmCorrection' if '", LaunchConfiguration('rl_arm_correction'), "' == 'real' else 'faker/FakeRlArmCorrection'"]),
            **{f'use_{n}': is_real(n) for n in ML_NODES},
        }),

        fake('fake_camera', 'pole_front_cam', respawn=True,
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_gz_cameras'), "' == 'false'"]))),
        fake('fake_camera', 'gripper_cam', respawn=True,
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_gz_cameras'), "' == 'false'"]))),
        fake('fake_opencr', 'fake_opencr', {
            'map_yaml': PathJoinSubstitution([FindPackageShare('faker'), 'world', [LaunchConfiguration('course'), '.yaml']]),
        }, condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mock_hardware'), "' == 'false'"]))),
        *[fake(f'fake_{n}', n, condition=IfCondition(PythonExpression(["'", LaunchConfiguration(n), "' == 'fake'"])))
          for n in ML_NODES],
        # fake_slam(기본): fake_cuvslam(map→odom) + fake_nvblox(/map·ESDF — MAPPING은 점구름으로 지도를 쌓는다)
        # 진짜 SLAM(fake_slam:=false)이면 ros_system이 지도·map→odom을 발행 — 여기는 끈다
        # map → odom: fake_opencr가 있으면 진짜 위치로 보정 (fake_cuvslam — 걸리거나 미끄러져도 위치가 안 틀어짐) · mock이면 항등
        Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_odom', output='screen',
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('fake_slam'), "' == 'true' and '",
                                                     LaunchConfiguration('mock_hardware'), "' == 'true'"])),
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom']),
        # mock에선 ground_truth가 없어 idle — map→odom은 위 항등이 잡음
        fake('fake_cuvslam', 'fake_cuvslam', condition=IfCondition(FAKE_SLAM)),
        # MAPPING: /pole_front_cam/depth/color/points로 지도를 쌓아 /map 발행 · nvblox.nvblx 저장
        # LOCALIZATION: 저장한 nvblox.nvblx를 읽어 /map 발행 · ESDF
        fake('fake_nvblox', 'nvblox_node', {
            'map_dir': ParameterValue(LaunchConfiguration('map_dir'), value_type=str),
            'map_name': ParameterValue(LaunchConfiguration('map_name'), value_type=str),  # 20261108_125457 → int로 안 바뀌게
            'map_mode': ParameterValue(LaunchConfiguration('map_mode'), value_type=str),
        }, condition=IfCondition(FAKE_SLAM)),
        fake('monitor_manager', 'monitor_manager', {'debug_dir': LaunchConfiguration('debug_dir')}, respawn=True,  # 죽으면 시뮬레이터가 다 멈춰 보임 → 다시 띄움
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_monitor'), "' == 'true'"]))),
        fake('input_manager', 'input_manager', {'debug_dir': LaunchConfiguration('debug_dir')}),
        fake('service_action_monitor', 'service_action_monitor', {'debug_dir': LaunchConfiguration('debug_dir')}),
        fake('service_action_bag_input', 'service_action_bag_input', {'debug_dir': LaunchConfiguration('debug_dir')}),
    ])
