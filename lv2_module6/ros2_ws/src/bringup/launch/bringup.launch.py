"""전체 실행: ros_system(📦 제공 패키지) + 🛠 직접 작성 노드.

  ros2 launch bringup bringup.launch.py
  ros2 launch bringup bringup.launch.py use_ros_system:=false            # 📦 대신 faker 등 다른 입력 (faker.launch.py가 씀)
  ros2 launch bringup bringup.launch.py learning_enabled:=false           # 학습 노드 Δ=0 · 프리셋 A 고정 (비교 기준)
  ros2 launch bringup bringup.launch.py use_foxglove:=false               # Foxglove 브리지 끔 (기본 auto = 설치돼 있으면 ws://<로봇>:8765)
지도 (실기·테스트베드 공통 — 지도 한 벌 = 폴더 하나 <map_dir>/<map_name>/ nvblox.nvblx · map.pgm · map.yaml):
  ros2 launch bringup bringup.launch.py map:=slam                 # 매핑: 새 폴더 <날짜_시각>/ (map_name:=으로 이름 지정, 있는 이름이면 거부)
  ros2 launch bringup bringup.launch.py map:=saved                # 위치추정: 가장 최근 폴더 (기본)
  ros2 launch bringup bringup.launch.py map:=saved map_name:=20261017_163731
  map_dir 기본 ~/.ros/beebot_maps (실기 start.sh는 ros2_ws/map, 테스트베드 faker는 /ws/map)
  faker 테스트베드는 fake_slam:=true(기본) — 로봇이 센서로 지도를 쌓고(fake_nvblox) 저장한 지도로 위치추정한다 (GPU 불필요)
  (진짜 cuVSLAM·nvblox는 fake_slam:=false + GPU)
  실행 중 바꾸기: 스캔 ↔ 위치추정 /cmd/map_mode · 지도 갈아끼우기 /cmd/map_name — state_manager가 인자만 바꿔 launch 전체를 다시 띄움
재시작 (state_manager — 이 launch 명령줄 그대로 다시): ros2 topic pub --once /cmd/restart std_msgs/msg/String "{data: 'all'}"
  'all map:=slam' (그 인자만 바꿔) · 'node perception' (그 노드만, 같은 파라미터)

🛠 노드 (패키지 · 담당):
  perception: perception, body_filter, signal_decoder(신호등 암호 → /mission/signal) (전승혜)
  plan:       coordinator (권형중)
  ml:         grasp_scorer (송정혁)
              rl_arm_correction은 노드가 아니라 컨트롤러 플러그인(ml/RlArmCorrection) — ros_system의 controller_manager가 올린다
  control:    arm_safety_monitor, manual_control (김현하)
상태: 각 노드 /diagnostics → diagnostic_aggregator → /diagnostics_agg (config/diagnostics.yaml)
"""

import os
import subprocess
import sys
import time

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


ML_NODES = ('grasp_scorer',)


def node(package, executable, overrides=None, condition=None):
    # 파라미터: 패키지 config/<패키지>.yaml → launch 인자(overrides)가 덮어씀
    config = PathJoinSubstitution([FindPackageShare(package), 'config', f'{package}.yaml'])
    return Node(package=package, executable=executable, output='screen', parameters=[config, overrides or {}],
                condition=condition)


# ros_system에 그대로 넘기는 하드웨어·서버 인자 (기본값 = 실기). 테스트베드 faker가 대역에 맞게 덮어씀
ROS_SYSTEM_ARGS = {
    'urdf': ('default', '로봇 모델 = lv2_module6/urdf/<이 이름>/urdf.xacro (폴더 하나 = 모델 하나, 메시는 그 폴더 meshes/)'),
    'mock_hardware': ('false', 'true면 OpenCR 없이 BeebotSystemHardware가 명령을 상태로 되돌림'),
    'mock_effort': ('0.0', 'mock일 때 팔 관절 effort [A]'),
    'port': ('/dev/opencr', 'OpenCR 포트 (udev 고정 이름)'),
    'twist_state': ('true', 'true면 joint2·3 상태 = 엔코더 + 중력 비틀림 (BeebotSystemHardware) → TF = 실제 팔 · false = 엔코더 각 그대로 (강성 측정)'),
    'arm_correction_type': ('ml/RlArmCorrection', 'rl_arm_correction 컨트롤러 플러그인 (테스트베드 ML 대역: faker/FakeRlArmCorrection)'),
    'use_cameras': ('true', 'RealSense 2대 (테스트베드는 faker의 fake_camera가 대신 → false)'),
    'use_pole_front_cam': ('true', 'pole_front_cam만 끄기/켜기 (한 대만 연결할 때)'),
    'use_gripper_cam': ('auto', 'gripper_cam: auto = gripper_cam_serial 장치가 USB에 감지될 때만 켬(뽑아 두면 노드·의존 노드 안 띄움) · true/false = 강제'),
    'pole_front_cam_serial': ('', 'pole_front_cam RealSense serial_no — 빈 값이면 첫 장치 (2대면 반드시 지정)'),
    'gripper_cam_serial': ('', 'gripper_cam RealSense serial_no — 빈 값이면 첫 장치 (2대면 반드시 지정)'),
    'pole_front_cam_profile': ('640x360x15', 'pole_front_cam RGB/depth 프로파일 (WxHxFPS)'),
    'gripper_cam_profile': ('640x360x15', 'gripper_cam RGB/depth 프로파일 (WxHxFPS)'),
    'camera_decimation_enable': ('true', 'RealSense depth decimation 필터 (점군 밀도 감소)'),
    'camera_decimation_magnitude': ('2', 'decimation 배율 (2=1/4 점, 3=1/9 점)'),
    'pole_front_cam_pointcloud': ('true', 'pole_front_cam PointCloud2 발행 (body_filter/Nav2)'),
    'gripper_cam_pointcloud': ('false', 'gripper_cam PointCloud2 발행 (grasp_scorer용, 필요할 때만 true)'),
    'use_ekf': ('true', 'robot_localization EKF'),
    'ekf_imu': ('true', 'EKF yaw 회전속도 = IMU 자이로 · false면 바퀴'),
    'use_nav2': ('true', '진짜 Nav2 서버'),
    'use_moveit': ('true', '진짜 move_group — coordinator의 pick/place가 씀 (ros_system 기본은 false라 실기에서 팔 작업이 안 됐음)'),
}


# 지도 폴더 안 지도 파일 (nvblox) — 이 파일이 있는 폴더 = 저장한 지도 (state_manager kMapFile 목록도 같은 기준)
MAP_FILE = 'nvblox.nvblx'


def detected_serials():
    """USB에 꽂힌 RealSense serial 목록 (`rs-enumerate-devices -s` 출력) — 못 알아내면 None.
    launch가 (다시) 뜰 때마다 부른다: state_manager의 launch 재시작은 start.sh를 안 거치고 기록된 인자를 그대로 쓰므로 감지는 여기서 해야 한다."""
    try:
        r = subprocess.run(['rs-enumerate-devices', '-s'], capture_output=True, text=True, timeout=10)
        return r.stdout if r.returncode == 0 else None
    except (OSError, subprocess.SubprocessError):
        return None


def setup_map(context):
    """지도 정하기 (실기·테스트베드 공통): map:=none → 지도 없음(NONE) · slam → 새 폴더(기본 날짜_시각) 매핑 · saved → 고른 폴더(기본 default, 없으면 가장 최근) 위치추정.
    ros_system에 넘길 use_slam · localization · nvblox_map과 state_manager 시작 값(map_mode · map_name · map_dir)을 정한다.
    ros_system을 안 띄우면(use_ros_system:=false) 확인만 건너뜀. 지도가 없으면(saved인데 저장 지도 없음) NONE으로 시작한다.

    fake_slam:=true (faker 테스트베드) — 지도 모드·map_saver는 그대로 두고 ros_system의 진짜 SLAM만 끈다.
    MAPPING은 fake_nvblox가 센서 점구름으로, LOCALIZATION은 저장한 nvblox.nvblx를 읽어 /map을 낸다."""
    mode = LaunchConfiguration('map').perform(context)
    fake_slam = LaunchConfiguration('fake_slam').perform(context).lower() == 'true'
    use_ros_system = LaunchConfiguration('use_ros_system').perform(context).lower() == 'true'
    map_file = MAP_FILE
    d = os.path.expanduser(LaunchConfiguration('map_dir').perform(context) or '~/.ros/beebot_maps')
    name = LaunchConfiguration('map_name').perform(context)
    if mode not in ('none', 'slam', 'saved'):
        raise RuntimeError(f'map:={mode} — none | slam | saved')
    no_map = False
    if use_ros_system and mode != 'none':
        if mode == 'slam':
            name = name or time.strftime('%Y%m%d_%H%M%S')
            if os.path.exists(os.path.join(d, name, map_file)):
                raise RuntimeError(f'이미 있는 지도: {d}/{name}/{map_file} — 덮어쓰지 않음, 다른 map_name')
            os.makedirs(os.path.join(d, name), exist_ok=True)  # nvblox는 저장할 폴더를 안 만듦
        else:
            saved = sorted(p for p in (os.listdir(d) if os.path.isdir(d) else [])
                           if os.path.exists(os.path.join(d, p, map_file)))
            if not saved:
                # 센서·지도가 아직 없어도 state_manager와 나머지 노드는 뜨게 한다 — NONE으로 시작.
                print(f'WARN: 저장한 지도 없음: {d}/<이름>/{map_file} — NONE(지도 없음)으로 시작', file=sys.stderr)
                name = ''
                no_map = True
            else:
                name = name or ('default' if 'default' in saved else saved[-1])  # 기본 지도 폴더 = default (지도 파일이 있어야)
                if name not in saved:
                    print(f'WARN: 지도 없음: {d}/{name}/{map_file} — NONE(지도 없음)으로 시작 (있는 것: {", ".join(saved)})',
                          file=sys.stderr)
                    name = ''
                    no_map = True
    if mode == 'none':
        name = ''
    effective = 'none' if mode == 'none' or no_map else mode
    use_slam = 'false' if effective == 'none' else 'true'
    # use_slam = 지도 의미(모드·map_saver가 켜져야 하나) · use_real_slam = ros_system의 cuVSLAM·nvblox를 띄울지
    use_real_slam = 'false' if fake_slam else use_slam
    map_mode = {'none': 'NONE', 'slam': 'MAPPING', 'saved': 'LOCALIZATION'}[effective]
    # 그립퍼 캠: 뽑아 두면 노드가 respawn(2 s)하며 장치를 계속 훑어 CPU·USB를 먹는다 — 감지될 때만 켠다.
    ug = LaunchConfiguration('use_gripper_cam').perform(context).lower()
    if ug not in ('auto', 'true', 'false'):
        raise RuntimeError(f'use_gripper_cam:={ug} — auto | true | false')
    if ug == 'auto':
        serial = LaunchConfiguration('gripper_cam_serial').perform(context)
        found = detected_serials() if serial else None
        ug = 'true' if (found is None or serial in found) else 'false'   # 못 알아내면 예전처럼 켬
        if ug == 'false':
            print(f'WARN: gripper_cam serial {serial} 감지 안 됨 — gripper_cam · hand_relative를 안 띄움', file=sys.stderr)
    # 인지(YOLO 등)는 지도 만들 때(MAPPING) 쓰이지 않는다 — 실기에서는 CPU 코어 하나의 절반을 먹어 주행이 끊겼다 (2026-11-09).
    #   perception:=auto(기본) = 매핑이면 끔 · true/false = 강제. faker 테스트베드(fake_slam)는 그대로 켠다.
    pm = LaunchConfiguration('perception').perform(context).lower()
    if LaunchConfiguration('use_mission').perform(context).lower() == 'false':
        pm = 'false'   # 미션을 끄면 인지도 끔 — 아래 use_perception:=true가 다시 켤 수 있다
    up = LaunchConfiguration('use_perception').perform(context).lower()   # use_perception:=true|false가 perception:=보다 우선 (use_* 인자와 같은 모양)
    if up in ('true', 'false'):
        pm = up
    if pm not in ('auto', 'true', 'false'):
        raise RuntimeError(f'perception:={pm} — auto | true | false')
    use_perception = 'false' if (pm == 'false' or (pm == 'auto' and effective == 'slam' and not fake_slam)) else 'true'
    # 지도/위치추정이 없으면 Nav2도 띄우지 않는다 — 지도 없는 costmap·BT가 헛도는 걸 막는다.
    use_nav2 = 'false' if effective == 'none' else 'true'
    nvblox_map = os.path.join(d, name, map_file) if effective != 'none' else ''
    return [SetLaunchConfiguration('map_dir', d), SetLaunchConfiguration('map_name', name),
            SetLaunchConfiguration('map_mode', map_mode),
            SetLaunchConfiguration('use_slam', use_slam),
            SetLaunchConfiguration('use_real_slam', use_real_slam),
            SetLaunchConfiguration('use_nav2', use_nav2),
            SetLaunchConfiguration('localization', 'false' if effective == 'slam' else 'true'),
            SetLaunchConfiguration('nvblox_map', nvblox_map),
            SetLaunchConfiguration('use_perception', use_perception),
            SetLaunchConfiguration('use_gripper_cam', ug),
            SetLaunchConfiguration('use_hand_relative', 'true' if use_perception == 'true' and ug == 'true' else 'false')]

def foxglove(context):
    """Foxglove(맥·아이패드 앱)용 WebSocket 브리지 — auto: foxglove_bridge가 설치돼 있을 때만 · 접속 ws://<로봇 IP 또는 bee.local>:<foxglove_port>
    부하: 보는 쪽이 구독한 토픽만 보냄 (아무도 안 붙으면 discovery뿐) — 영상은 볼 때만 열 것"""
    use = LaunchConfiguration('use_foxglove').perform(context).lower()
    if use == 'false':
        return []
    try:
        get_package_share_directory('foxglove_bridge')
    except PackageNotFoundError:
        if use == 'true':
            raise RuntimeError('use_foxglove:=true인데 foxglove_bridge가 없음 — sudo apt install ros-lyrical-foxglove-bridge')
        print('[bringup] foxglove_bridge 없음 — Foxglove 안 띄움 (설치: sudo apt install ros-lyrical-foxglove-bridge)')
        return []
    return [Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge', output='screen',
                 parameters=[{'port': int(LaunchConfiguration('foxglove_port').perform(context)), 'address': '0.0.0.0',
                              'send_buffer_limit': 100_000_000}])]


def generate_launch_description():
    learning = {'enabled': LaunchConfiguration('learning_enabled')}
    return LaunchDescription([
        DeclareLaunchArgument('use_ros_system', default_value='true',
                              description='false면 ros_system(📦)을 띄우지 않음'),
        DeclareLaunchArgument('use_mission', default_value='true',
                              description='false면 미션에 쓰는 것을 모두 안 띄움 — mission_manager · coordinator · skill 노드(arm_skills·base_skills) · 인지(perception_node·signal_decoder·hand_relative) · grasp_scorer. 주행·수동 조작·body_filter는 그대로 (use_perception:=true 등 개별 인자가 우선)'),
        DeclareLaunchArgument('use_mission_manager', default_value='true',
                              description='false면 mission_manager를 띄우지 않음 — 도면 벽(/arena/walls)이 /map에 안 겹침 (매핑·주행 시험용, 미션 불가)'),
        DeclareLaunchArgument('use_perception', default_value='auto',
                              description='false면 인지 노드(perception_node · signal_decoder · hand_relative)를 안 띄움 · true면 항상 · auto(기본)면 perception:=를 따름 (body_filter는 Nav2용이라 늘 뜸)'),
        DeclareLaunchArgument('use_ml', default_value='true',
                              description='false면 ml 노드(grasp_scorer)를 띄우지 않음'),
        *[DeclareLaunchArgument(f'use_{n}', default_value=LaunchConfiguration('use_ml'),
                                description=f'false면 {n}을 띄우지 않음 (faker가 그 대역 fake_{n}을 띄울 때)') for n in ML_NODES],
        DeclareLaunchArgument('learning_enabled', default_value='true',
                              description='false면 학습 노드가 Δ=0 · 기본 프리셋 (노드는 그대로 뜸)'),
        DeclareLaunchArgument('operation', default_value='MANUAL',
                              description='초기 조작 모드: MANUAL(기본) | AUTO'),
        DeclareLaunchArgument('start_pose', default_value='',
                              description='mission_manager 출발 자세 arena [x, y, yaw] YAML (비면 plan.yaml start_pose_arena)'),
        DeclareLaunchArgument('grasp_check', default_value='true',
                              description='coordinator 잡기 판정(그리퍼 위치 + 전류) — 물리 없는 대역(faker)은 손가락이 끝까지 닫혀 늘 빈손이라 false'),
        DeclareLaunchArgument('restart_prepare', default_value='',
                              description='/cmd/restart all 때 다시 띄우기 전 셸 명령 (state_manager — 테스트베드 faker = build-ws)'),
        DeclareLaunchArgument('restart_log', default_value='/tmp/bringup.log',
                              description='/cmd/restart all 때 다시 띄운 launch 출력 파일 (state_manager)'),
        DeclareLaunchArgument('map', default_value='none',
                              description='none = 지도 없음(기본) · slam = 새 폴더에 센서로 매핑 · saved = 저장 지도로 위치추정 (faker는 fake_slam:=true)'),
        DeclareLaunchArgument('map_dir', default_value='',
                              description='지도 폴더들이 있는 곳 — <map_dir>/<map_name>/nvblox.nvblx (비면 ~/.ros/beebot_maps)'),
        DeclareLaunchArgument('map_name', default_value='',
                              description='지도 폴더 이름 — slam: 비우면 날짜_시각으로 새로 · saved: 비우면 가장 최근'),
        DeclareLaunchArgument('perception', default_value='auto',
                              description='인지 노드(perception_node · signal_decoder · hand_relative): auto = 매핑(map:=slam)이면 끔 · true = 항상 · false = 항상 끔. body_filter는 Nav2 충돌감시용이라 그대로'),
        DeclareLaunchArgument('fake_slam', default_value='false',
                              description='true = faker SLAM 대역(코스 map_server · fake_cuvslam · fake_nvblox) — ros_system의 진짜 cuVSLAM·nvblox만 끔'),
        DeclareLaunchArgument('use_foxglove', default_value='auto',
                              description='Foxglove 브리지 — auto: foxglove_bridge가 설치돼 있으면 켬 · true(없으면 오류) · false'),
        DeclareLaunchArgument('foxglove_port', default_value='8765', description='Foxglove WebSocket 포트 (ws://<로봇>:<이 포트>)'),
        *[DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in ROS_SYSTEM_ARGS.items()],
        OpaqueFunction(function=setup_map),
        OpaqueFunction(function=foxglove),

        # GroupAction: ros_system의 launch 인자가 이 파일 인자와 섞이지 않게 범위 분리 — 넘길 것만 골라서
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('ros_system'), 'launch', 'ros_system.launch.py'])),
            launch_arguments={**{k: LaunchConfiguration(k) for k in ROS_SYSTEM_ARGS},
                              'use_slam': LaunchConfiguration('use_real_slam'),
                              'localization': LaunchConfiguration('localization'),
                              'nvblox_map': LaunchConfiguration('nvblox_map')}.items())],
            condition=IfCondition(LaunchConfiguration('use_ros_system'))),
        # SLAM(slam·saved)일 때: 지도 저장 서버 (사람이 보는 미리보기 — 위치추정은 nvblox.nvblx) — /map_saver/save_map → <map_dir>/<map_name>/map.pgm·yaml (시뮬레이터 [지도 저장])
        Node(package='nav2_map_server', executable='map_saver_server', name='map_saver', output='screen',
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_ros_system'), "'.lower() == 'true' and '",
                                                     LaunchConfiguration('use_slam'), "' == 'true'"]))),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map_saver',
             output='screen', parameters=[{'autostart': True, 'node_names': ['map_saver'], 'bond_timeout': 60.0}],
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_ros_system'), "'.lower() == 'true' and '",
                                                     LaunchConfiguration('use_slam'), "' == 'true'"]))),

        node('perception', 'perception_node', condition=IfCondition(LaunchConfiguration('use_perception'))),
        # coordinator: 계획 단계를 skill 노드에 맡기는 순서 담당 (적재함 장부 · 모드·비상정지 취소)
        node('plan', 'coordinator', condition=IfCondition(LaunchConfiguration('use_mission'))),
        # skill 노드 — 단계마다 액션 서버 /skills/<skill> (팔: arm_skills 컨테이너 · 주행: base_skills). 학습 정책은 skill마다 models/skills/<skill>/
        #   시작 자세(stow)는 skill_pose
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('skills'), 'launch', 'skills.launch.py'])),
            launch_arguments={'grasp_check': LaunchConfiguration('grasp_check')}.items())],
            condition=IfCondition(LaunchConfiguration('use_mission'))),
        # 주사위 미션 — /cmd/mission start로 시작 (abort로 멈춤). 출발 자세만 launch 인자로 덮어씀
        node('plan', 'mission_manager', condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('use_mission'), "'.lower() == 'true' and '", LaunchConfiguration('use_mission_manager'),
             "'.lower() == 'true' and '", LaunchConfiguration('start_pose'), "' == ''"]))),
        node('plan', 'mission_manager', {'start_pose_arena': ParameterValue(LaunchConfiguration('start_pose'), value_type=None)},
             condition=IfCondition(PythonExpression(
                 ["'", LaunchConfiguration('use_mission'), "'.lower() == 'true' and '", LaunchConfiguration('use_mission_manager'),
                  "'.lower() == 'true' and '", LaunchConfiguration('start_pose'), "' != ''"]))),
        *[node('ml', n, learning, condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('use_mission'), "'.lower() == 'true' and '", LaunchConfiguration(f'use_{n}'), "'.lower() == 'true'"])))
          for n in ML_NODES],
        node('control', 'arm_safety_monitor'),
        node('control', 'manual_control'),
        node('perception', 'body_filter'),
        node('perception', 'signal_decoder', condition=IfCondition(LaunchConfiguration('use_perception'))),
        node('perception', 'hand_relative', condition=IfCondition(LaunchConfiguration('use_hand_relative'))),  # 손 카메라 상대값 → rl_arm_correction 정책 입력
        node('manager', 'state_manager', {'operation': LaunchConfiguration('operation'),
                                          'map_mode': LaunchConfiguration('map_mode'),
                                          'map_name': ParameterValue(LaunchConfiguration('map_name'), value_type=str),
                                          'map_dir': ParameterValue(LaunchConfiguration('map_dir'), value_type=str),
                                          'restart_prepare': ParameterValue(LaunchConfiguration('restart_prepare'), value_type=str),
                                          'restart_log': ParameterValue(LaunchConfiguration('restart_log'), value_type=str)}),
        Node(package='diagnostic_aggregator', executable='aggregator_node', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('bringup'), 'config', 'diagnostics.yaml'])]),
    ])
