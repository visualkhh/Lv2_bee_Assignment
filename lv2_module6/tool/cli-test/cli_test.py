#!/usr/bin/env python3
"""cli-test — ROS 없이 operator_tool UI/UX 시험.

  ./tool/run-cli-test status
  ./tool/run-cli-test mode manual
  ./tool/run-cli-test menu        # 대화형 메뉴 (터미널)

가짜 ROS층(fake_ros)으로 진짜 operator_tool 코드를 그대로 돌린다.
발행은 [PUB ...]으로 찍히고, mode/estop/map 요청은 가짜 state_manager가
받아 상태를 바꾼다. 파라미터 명령은 거부 경로 UX만 확인된다.
"""
from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
STATE_FILE = HERE / '.fake_state.json'  # 실행 사이 가짜 상태 유지 (mode manual → teleop 같은 흐름 시험용)

sys.path.insert(0, str(HERE.parent / 'cli'))
sys.path.insert(0, str(HERE))

import json as _json

import fake_ros  # noqa: E402

fake_ros.install()

import operator_tool as ot  # noqa: E402


def _load_saved() -> dict:
    try:
        return _json.loads(STATE_FILE.read_text())
    except Exception:
        return {}


def _save(state) -> None:
    try:
        STATE_FILE.write_text(_json.dumps({
            'values': state.values,
            'emergency': state.emergency,
        }, ensure_ascii=False))
    except Exception:
        pass


def _seed(app: ot.OperatorTool):
    """가짜 bringup 상태 — status/menu가 볼 화면용."""
    st = app.state
    st.values.update({
        'operation_mode': 'AUTO',
        'map_mode': 'COURSE',
        'map_name': 'default',
        'map_list': 'default\n20261017_223015',
        'devices': 'pole_front_cam, gripper_cam, opencr, imu, wheels, arm, gripper',
        'topics': '/joint_states · /odom · /tracked_objects (42개)',
        'restartable': 'all · perception · coordinator · manual_control',
        'restart': '',
    })
    st.values.update(_load_saved().get('values', {}))
    st.emergency = _load_saved().get('emergency', False)
    st.on_emergency(False)
    st.on_devices(['pole_front_cam', 'gripper_cam', 'opencr', 'imu', 'wheels', 'arm', 'gripper'])
    st.on_device_detail('pole_front_cam', 'camera|active|/pole_front_cam/color/image_raw')
    st.on_device_detail('gripper_cam', 'camera|active|/gripper_cam/color/image_raw')
    st.on_device_detail('opencr', 'controller|active|/joint_states,/odom')
    st.on_device_detail('imu', 'sensor|active|/imu')
    st.on_joints(
        ['joint1', 'joint2', 'joint3', 'joint4', 'joint5',
         'gripper_left_joint', 'wheel_left_joint', 'wheel_right_joint'],
        [0.0, -0.5, 0.8, 0.4, 0.0, 0.026, 1.2, 1.2],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5])
    st.on_diagnostics([
        fake_ros.DiagnosticStatus(level=0, name='beebot: 상태', message='정상',
                                  values=[fake_ros.KeyValue(key='모드', value='AUTO')]),
        fake_ros.DiagnosticStatus(level=1, name='beebot: opencr', message='모터 에러 없음 · S 줄 지연',
                                  values=[fake_ros.KeyValue(key='받은 S 줄', value='48 Hz')])])
    st.on_say('컵 가져왔어요')
    try:
        st.on_objects([1, 2, 3])
    except Exception:  # noqa: BLE001
        pass
    try:
        st.on_task(7, 3, 1, '')
    except Exception:  # noqa: BLE001
        pass


_orig_init = ot.OperatorTool.__init__

_APPS: dict = {}


def _init(self):
    _orig_init(self)
    _APPS['node'] = self
    _seed(self)


ot.OperatorTool.__init__ = _init


if __name__ == '__main__':
    code = ot.main()
    node = _APPS.get('node')
    if node is not None:
        _save(node.state)
    raise SystemExit(code)
