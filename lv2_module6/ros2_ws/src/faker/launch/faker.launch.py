"""테스트베드 실행: 진짜 bringup(🛠 노드 + ros_system 📦 + 지도 처리 — 실기와 같은 launch) + 도커에 없는 📦의 대역 + 호스트 파일 연동.
  bringup에는 대역에 맞는 인자만 넘김 (use_cameras:=false · mock_hardware · arm_correction_type · ML use_*) — 지도 map:=slam|saved 처리는 bringup 것 그대로.

  ros2 launch faker faker.launch.py
  ros2 launch faker faker.launch.py mock_effort:=1.5        # 팔 전류 과다 → arm_safety_monitor 접촉 감지
  ros2 launch faker faker.launch.py mock_hardware:=true     # OpenCR 없이 (fake_opencr 안 띄움 — controller_manager mock이 명령을 상태로 되돌림, 물리 없음)
  ros2 launch faker faker.launch.py ml:=real                # ML 대역 대신 진짜 ml 노드·컨트롤러 (GPU·모델 없이도 돌 때만)
  ros2 launch faker faker.launch.py grasp_scorer:=real      # 하나만 진짜로 (rl_nav_tuner · rl_nav_correction · rl_arm_correction도 같음, 기본 = ml 값)
  지도 map:= (fake_opencr 충돌 판정은 늘 실제 코스 — 월드. 로봇 쪽 지도만 바뀜):
  ros2 launch faker faker.launch.py                         # course (기본): 코스 지도를 외운 상태 (map_server + map→odom 항등)
  지도 한 벌 = 폴더 하나: /ws/map/<map_name>/ rtabmap.db (위치추정용 정본) · map.pgm · map.yaml (사람이 보는 미리보기)
  ros2 launch faker faker.launch.py map:=slam               # ① 매핑: 새 폴더 /ws/map/<날짜_시각>/ (map_name:=으로 이름 지정 가능, 있는 이름이면 거부)
                                                             #    시뮬레이터 수동 조작으로 몰고 다니면 RTAB-Map이 pole_front_cam(컬러+깊이)으로 → rtabmap.db
                                                             #    다 돌았으면 시뮬레이터 [지도 저장] (/map_saver/save_map → 같은 폴더 map.pgm · map.yaml)
                                                             #    시뮬레이터 [스캔 시작]·[위치추정으로] → /cmd/map_mode → state_manager가 RTAB-Map 모드 전환
                                                             #    (재시작 없이) → /state/map_mode 발행
                                                             #    끝(Ctrl+C)에 RTAB-Map이 DB를 닫음 — /ws/map = 호스트 ros2_ws/map
  ros2 launch faker faker.launch.py map:=saved              # ② 미션: default 폴더(없으면 가장 최근)의 지도로 시작 (RTAB-Map 위치추정 모드가 rtabmap.db를 한 번 읽어 /map · map→odom)
  ros2 launch faker faker.launch.py map:=saved map_name:=20261017_223015   #    폴더를 골라서
                                                             #    map.pgm·yaml은 사람이 보는 미리보기 — 위치추정은 rtabmap.db (카메라 장면 기억)가 있어야 함

실제로 띄우는 📦 (ros_system):
  robot_state_publisher(URDF) · controller_manager + control/BeebotSystemHardware(가상 시리얼 ↔ fake_opencr) + 컨트롤러 · robot_localization EKF
  · 진짜 Nav2 (planner · controller · behavior · bt_navigator · velocity_smoother · collision_monitor)
map:=course일 때만 — 실기에서 RTAB-Map이 하는 일의 대신 (시뮬레이터 = 코스를 알고 있음):
  map_server       코스 지도 /map (maps/<course>.yaml — course/tools/build_map.py · build_dice.py, 원점 = 출발 칸)
  map→odom         fake_localizer: 진짜 위치(fake_opencr ground_truth)로 보정 · mock_hardware:=true면 항등 (RTAB-Map 몫의 대신)
대역 (도커 이미지에 없는 📦 · GPU·모델이 없는 ML):
  ML 대역 (ml 패키지와 1:1 — 같은 노드 이름·토픽, 하나씩 진짜/대역 선택):
    fake_grasp_scorer       → 노드 grasp_scorer
    fake_rl_nav_tuner       → 노드 rl_nav_tuner
    fake_rl_nav_correction  → 노드 rl_nav_correction
    FakeRlArmCorrection     → 컨트롤러 rl_arm_correction 자리 (플러그인, Δ=0)
  fake_camera ×2   realsense2_camera (pole_front_cam, gripper_cam) — 시뮬레이터가 debug/input/hardware/camera/에 쓴 프레임만 (없으면 안 냄)
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
from launch_ros.substitutions import FindPackageShare


def arg(name, default, description):
    return DeclareLaunchArgument(name, default_value=default, description=description)


def include(package, launch_file, args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])),
        launch_arguments=args.items())


ML_NODES = ('grasp_scorer', 'rl_nav_tuner', 'rl_nav_correction')   # ml 패키지 노드 — 대역 fake_<이름>
ML = ML_NODES + ('rl_arm_correction',)                              # + 컨트롤러 플러그인


def fake(executable, name, overrides=None, respawn=False, condition=None):
    # 파라미터: config/faker.yaml(노드 이름별) → launch 인자(overrides)가 덮어씀
    config = PathJoinSubstitution([FindPackageShare('faker'), 'config', 'faker.yaml'])
    return Node(package='faker', executable=executable, name=name, output='screen', parameters=[config, overrides or {}],
                respawn=respawn, respawn_delay=1.0, condition=condition)


# 코스 지도(외운 지도)로 도나 — map:=course
COURSE_MAP = PythonExpression(["'", LaunchConfiguration('map'), "' == 'course'"])


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
        arg('course', 'twin_closed_maze', '코스 = faker/maps/<이 이름>.yaml (fake_opencr 충돌 · map:=course 지도) — 주사위 미션: dice_maze'),
        arg('start_pose', '', 'mission_manager 출발 자세 arena [x, y, yaw] (비면 plan.yaml) — 시뮬레이터 dice_maze는 "[0.734, 0.950, 0.0]" (+X를 보고 켬)'),
        arg('map', 'course', 'course = 코스 지도 외움 · slam = RTAB-Map 매핑(rtabmap.db 새로) · saved = 저장한 rtabmap.db로 위치추정'),
        arg('map_dir', '/ws/map', '지도 폴더 (호스트 ros2_ws/map 바인드) — 지도마다 <map_name>/ rtabmap.db · map.pgm · map.yaml'),
        arg('map_name', '', '지도 폴더 이름 — slam: 비우면 날짜_시각으로 새로 · saved: 비우면 default (rtabmap.db 없으면 가장 최근 — bringup이 정함)'),
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
            'map_dir': LaunchConfiguration('map_dir'),
            'map_name': LaunchConfiguration('map_name'),
            'urdf': LaunchConfiguration('urdf'),
            'mock_hardware': LaunchConfiguration('mock_hardware'),
            'mock_effort': LaunchConfiguration('mock_effort'),
            'use_cameras': 'false',  # RealSense 대신 fake_camera (아래)
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
            'map_yaml': PathJoinSubstitution([FindPackageShare('faker'), 'maps', [LaunchConfiguration('course'), '.yaml']]),
        }, condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mock_hardware'), "' == 'false'"]))),
        *[fake(f'fake_{n}', n, condition=IfCondition(PythonExpression(["'", LaunchConfiguration(n), "' == 'fake'"])))
          for n in ML_NODES],
        # map:=course일 때만: 코스 지도 + map→odom 항등 (외운 지도)
        # 매핑·저장 지도면 RTAB-Map이 지도·map→odom을 발행 — 여기는 끈다
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             condition=IfCondition(COURSE_MAP),
             parameters=[{'yaml_filename': PathJoinSubstitution([FindPackageShare('faker'), 'maps', [LaunchConfiguration('course'), '.yaml']])}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map', output='screen',
             condition=IfCondition(COURSE_MAP),
             parameters=[{'autostart': True, 'node_names': ['map_server'], 'bond_timeout': 60.0}]),
        # map → odom: fake_opencr가 있으면 진짜 위치로 보정 (fake_localizer — 걸리거나 미끄러져도 위치가 안 틀어짐) · mock이면 항등
        Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_odom', output='screen',
             condition=IfCondition(PythonExpression([COURSE_MAP, " and '", LaunchConfiguration('mock_hardware'), "' == 'true'"])),
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom']),
        fake('fake_localizer', 'fake_localizer',
             condition=IfCondition(PythonExpression([COURSE_MAP, " and '", LaunchConfiguration('mock_hardware'), "' == 'false'"]))),
        fake('monitor_manager', 'monitor_manager', {'debug_dir': LaunchConfiguration('debug_dir')}, respawn=True,  # 죽으면 시뮬레이터가 다 멈춰 보임 → 다시 띄움
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_monitor'), "' == 'true'"]))),
        fake('input_manager', 'input_manager', {'debug_dir': LaunchConfiguration('debug_dir')}),
        fake('service_action_monitor', 'service_action_monitor', {'debug_dir': LaunchConfiguration('debug_dir')}),
        fake('service_action_input', 'service_action_input', {'debug_dir': LaunchConfiguration('debug_dir')}),
    ])
