"""ROS 대역 — 진짜 rclpy 대신 쓰는 두 가지 백엔드.

  fake    FakeRos   프로세스 안 가짜 ROS (ROS·도커 없이 UI/UX 시험)
  docker  DockerRos 도커 test-faker가 /debug에 쓰는 파일을 폴링하는 브리지

operator_tool은 import 시점에 rclpy를 잡으므로, 반드시 operator_tool을 import하기 전에
install()을 부른다 (cli/__main__.py가 그 순서를 지킨다).
"""
from __future__ import annotations

from pathlib import Path

CLI = Path(__file__).resolve().parents[1]   # tools/cli


def default_debug_dir() -> Path:
    """호스트면 레포의 lv2_module6/debug, 컨테이너면 /debug."""
    if Path('/debug').is_dir():
        return Path('/debug')
    return CLI.parent.parent / 'debug'      # tools/cli → tools → lv2_module6


def install(source: str = 'fake', debug_dir=None) -> None:
    if source == 'fake':
        from .fake.FakeRos import FakeRos
        FakeRos.install()
    elif source == 'docker':
        from .docker.DockerRos import DockerRos
        DockerRos.install(debug_dir or default_debug_dir())
    else:
        raise ValueError(f'모르는 ROS 대역: {source!r} (fake | docker)')
