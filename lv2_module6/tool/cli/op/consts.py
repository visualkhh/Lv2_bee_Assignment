"""로봇 상수 + 파라미터 텍스트 변환 + argparse 숫자 검증."""
from __future__ import annotations

import argparse
import math

from rcl_interfaces.msg import ParameterType


ARM_JOINTS = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5')
WHEEL_JOINTS = ('wheel_left_joint', 'wheel_right_joint')
GRIPPER_JOINT = 'gripper_left_joint'
GRIPPER_OPEN = 0.0265
GRIPPER_CLOSE = 0.0

DIAG_LEVELS = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}

_PARAM_TYPE_NAMES = {
    ParameterType.PARAMETER_NOT_SET: 'not_set',
    ParameterType.PARAMETER_BOOL: 'bool',
    ParameterType.PARAMETER_INTEGER: 'int',
    ParameterType.PARAMETER_DOUBLE: 'double',
    ParameterType.PARAMETER_STRING: 'string',
    ParameterType.PARAMETER_BYTE_ARRAY: 'byte[]',
    ParameterType.PARAMETER_BOOL_ARRAY: 'bool[]',
    ParameterType.PARAMETER_INTEGER_ARRAY: 'int[]',
    ParameterType.PARAMETER_DOUBLE_ARRAY: 'double[]',
    ParameterType.PARAMETER_STRING_ARRAY: 'string[]',
}

_BOOL_TRUE = ('true', '1', 'yes', 'y', 'on')
_BOOL_FALSE = ('false', '0', 'no', 'n', 'off')


def _parse_bool_text(text: str) -> bool:
    low = text.strip().lower()
    if low in _BOOL_TRUE:
        return True
    if low in _BOOL_FALSE:
        return False
    raise ValueError(f'불리언이 아님 (true/false/1/0): {text}')


def _param_value_to_py(ptype: int, value) -> object:
    """ParameterValue 메시지 → 파이썬 값 (표시용)."""
    if ptype == ParameterType.PARAMETER_BOOL:
        return bool(value.bool_value)
    if ptype == ParameterType.PARAMETER_INTEGER:
        return int(value.integer_value)
    if ptype == ParameterType.PARAMETER_DOUBLE:
        return float(value.double_value)
    if ptype == ParameterType.PARAMETER_STRING:
        return str(value.string_value)
    if ptype == ParameterType.PARAMETER_BYTE_ARRAY:
        return list(value.byte_array_value)
    if ptype == ParameterType.PARAMETER_BOOL_ARRAY:
        return [bool(v) for v in value.bool_array_value]
    if ptype == ParameterType.PARAMETER_INTEGER_ARRAY:
        return [int(v) for v in value.integer_array_value]
    if ptype == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return [float(v) for v in value.double_array_value]
    if ptype == ParameterType.PARAMETER_STRING_ARRAY:
        return [str(v) for v in value.string_array_value]
    return None


def _parse_param_text(text: str, ptype: int):
    """입력 문자열 → 파라미터 타입에 맞는 파이썬 값 (set용)."""
    t = text.strip()
    if ptype == ParameterType.PARAMETER_BOOL:
        return _parse_bool_text(t)
    if ptype == ParameterType.PARAMETER_INTEGER:
        try:
            return int(t)
        except ValueError:
            return int(t, 0)  # 0x·0o·0b 접두사 허용
    if ptype == ParameterType.PARAMETER_DOUBLE:
        return float(t)
    if ptype == ParameterType.PARAMETER_STRING:
        return t
    if ptype in (ParameterType.PARAMETER_BOOL_ARRAY,
                 ParameterType.PARAMETER_INTEGER_ARRAY,
                 ParameterType.PARAMETER_DOUBLE_ARRAY,
                 ParameterType.PARAMETER_STRING_ARRAY,
                 ParameterType.PARAMETER_BYTE_ARRAY):
        if t == '':
            return []
        items = [x.strip() for x in t.split(',')]
        if ptype == ParameterType.PARAMETER_BOOL_ARRAY:
            return [_parse_bool_text(x) for x in items]
        if ptype == ParameterType.PARAMETER_INTEGER_ARRAY:
            return [int(x) for x in items]
        if ptype in (ParameterType.PARAMETER_DOUBLE_ARRAY, ParameterType.PARAMETER_BYTE_ARRAY):
            return [float(x) for x in items]
        return items
    # NOT_SET: 있는 그대로 추론
    low = t.lower()
    if low in _BOOL_TRUE:
        return True
    if low in _BOOL_FALSE:
        return False
    try:
        return int(t)
    except ValueError:
        pass
    try:
        return int(t, 0)
    except ValueError:
        pass
    try:
        return float(t)
    except ValueError:
        pass
    return t

# 대화형 메뉴 반환값
BACK = 'back'
MAIN = 'main'
QUIT = 'quit'
STAY = 'stay'
REFRESH = 'refresh'
TIMEOUT = '__timeout__'  # 입력 대기 시간 초과 → 화면만 다시 그림 (에러 아님)

# 메뉴 입력 대기 [s] — 이 시간 동안 입력이 없으면 헤더 상태바를 다시 그린다.
MENU_REFRESH_S = 2.0

RESET = '\033[0m'
BOLD = '\033[1m'
DIM = '\033[2m'
REV = '\033[7m'
RED = '\033[31m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
CYAN = '\033[36m'



def _positive_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise argparse.ArgumentTypeError('0보다 큰 유한한 숫자여야 합니다')
    return number


def _nonnegative_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise argparse.ArgumentTypeError('0 이상의 유한한 숫자여야 합니다')
    return number
