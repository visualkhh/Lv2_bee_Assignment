"""가짜 하드웨어 — fake_opencr(OpenCR 대역) 하나만. 실기의 OpenCR처럼 ROS 스택과 따로 켜 둔다.

  ros2 launch faker fake_hardware.launch.py [course:=dice_maze]

왜 따로: 지도 모드 전환 등으로 state_manager가 ROS 스택(faker.launch)을 다시 띄워도 실기 OpenCR·로봇은 그대로다.
  fake_opencr가 그 launch 안에 있으면 함께 재시작돼 로봇이 출발점으로 순간이동하고 팔이 stow로 돌아갔다 (2026-11-11).
  test-faker가 이걸 먼저 띄우고, faker.launch는 fake_opencr가 이미 떠 있으면 다시 띄우지 않는다.
  따로 안 띄우면(CI·직접 실행) faker.launch가 예전처럼 이 파일을 포함해 함께 띄운다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def stow_joints():
    """켤 때 팔 자세 = skills/params/basket.yaml pose_stow (실기는 꺼져 있는 동안 팔이 stow로 접혀 있음) — 못 읽으면 [] (0 자세)."""
    try:
        import yaml
        from ament_index_python.packages import get_package_share_directory
        path = f"{get_package_share_directory('skills')}/params/basket.yaml"
        with open(path) as f:
            return [float(v) for v in yaml.safe_load(f)['pose_stow']]
    except Exception as exc:  # skills 미설치 등 — 시뮬은 0 자세로 켜짐
        print(f'[faker] stow 자세를 못 읽음 ({exc}) — 팔 0 자세로 시작')
        return []


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('course', default_value='isaac_maze',
                              description='시뮬레이터 월드 코스 = faker/world/<이 이름>.yaml (fake_opencr 충돌 모델 — 로봇 지도 아님)'),
        Node(package='faker', executable='fake_opencr', name='fake_opencr', output='screen', parameters=[
            PathJoinSubstitution([FindPackageShare('faker'), 'config', 'faker.yaml']),
            PathJoinSubstitution([FindPackageShare('faker'), 'config', 'robot_footprint.yaml']),   # 몸 외곽 = 실제 로봇 USD
            {'map_yaml': PathJoinSubstitution([FindPackageShare('faker'), 'world', [LaunchConfiguration('course'), '.yaml']]),
             'start_joints': stow_joints() or [0.0] * 5},   # 켤 때 = stow (basket.yaml pose_stow)
        ]),
    ])
