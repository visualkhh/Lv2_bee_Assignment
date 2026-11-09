#!/usr/bin/env python3
"""arm_view 단위 시험 — ROS 없이: python3 tools/cli/tests/test_arm_view.py"""
import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))   # tools/cli
from ros.fake.FakeRos import FakeRos  # noqa: E402

FakeRos.install()
from op.views.arm import ArmView  # noqa: E402
from op.views.arm.arm_view import FINE, STEP  # noqa: E402
from op.consts import ARM_JOINTS, ARM_LIMITS  # noqa: E402

cur = {n: 0.0 for n in ARM_JOINTS}
v = ArmView(ARM_JOINTS, ARM_LIMITS, cur.get, color=False)
assert v.changes() == {}, '처음엔 바꾼 게 없음'

# ←→ = 목표만 ∓2° (현재값은 그대로) · , . = ∓1°
assert v.on_key('right')
assert abs(v.targets['joint1'] - STEP) < 1e-9 and cur['joint1'] == 0.0
assert v.on_key('.') and abs(v.targets['joint1'] - (STEP + FINE)) < 1e-9
assert set(v.changes()) == {'joint1'}
assert v.on_key('down') and v.sel == 1
assert v.on_key('up') and v.on_key('up') and v.sel == len(ARM_JOINTS) - 1, '↑는 맨 위에서 맨 아래로'

# 한계 밖으로 못 감 (joint3 위쪽 한계 1.396 rad)
v.sel = 2
for _ in range(200):
    v.on_key('right')
assert abs(v.targets['joint3'] - ARM_LIMITS['joint3'][1]) < 1e-9, '위쪽 한계에서 멈춤'
for _ in range(400):
    v.on_key('left')
assert abs(v.targets['joint3'] - ARM_LIMITS['joint3'][0]) < 1e-9, '아래쪽 한계에서 멈춤'

# z = 목표를 현재값으로 · 현재 각을 모르면 목표를 못 만든다
assert v.on_key('z') and v.changes() == {}
unknown = ArmView(ARM_JOINTS, ARM_LIMITS, lambda n: None, color=False)
assert unknown.on_key('right') and unknown.targets == {}, '현재 각을 모르면 움직일 목표 없음'

# 그리기 — 관절마다 막대 한 줄 + 한계 줄 · 현재(●) 목표(◆)
v.on_key('right')
lines = v.render(100)
assert len(lines) == 2 * len(ARM_JOINTS)
txt = '\n'.join(lines)
assert '●' in txt and '◆' in txt and 'joint1' in txt

# 모르는 키는 hook으로 (Enter 적용 등) · hook이 없으면 안 먹음
assert not v.on_key('x')
seen = []
v.extra_key = lambda k: seen.append(k) or True
assert v.on_key('enter') and seen == ['enter']

# ARM_LIMITS가 URDF의 <limit lower upper>와 같은지 (URDF가 있을 때만 — 배포 패키지엔 없을 수 있음)
urdf = HERE.parents[2] / 'urdf' / 'default' / 'urdf.xacro'
if urdf.is_file():
    found = {m.group(1): (float(m.group(2)), float(m.group(3))) for m in re.finditer(
        r'<joint name="(joint\d)"[^>]*>.*?<limit [^>]*lower="([^"]+)" upper="([^"]+)"', urdf.read_text(), re.S)}
    for n, (lo, hi) in ARM_LIMITS.items():
        assert n in found, f'URDF에 {n} 없음'
        assert math.isclose(found[n][0], lo, abs_tol=1e-5) and math.isclose(found[n][1], hi, abs_tol=1e-5), \
            f'{n} 한계가 URDF와 다름: consts {lo, hi} / URDF {found[n]}'
print('PASS test_arm_view' + ('' if urdf.is_file() else ' (URDF 없음 — 한계 비교는 건너뜀)'))
