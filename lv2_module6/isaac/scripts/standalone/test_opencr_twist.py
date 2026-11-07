#!/usr/bin/env python3
"""opencr_bridge.gravity_torques must stay equal to the URDF model the stiffness was fitted with. No Isaac needed.

  python3 isaac/scripts/standalone/test_opencr_twist.py
"""
import ast
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'calib'))
from arm_model import Arm

# only the function: importing opencr_bridge needs Isaac and opens the serial port
tree = ast.parse((HERE / 'opencr_bridge.py').read_text())
fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'gravity_torques')
ns = {'math': math}
exec(compile(ast.Module([fn], []), 'opencr_bridge.py', 'exec'), ns)

arm = Arm()
for q in ([0.0, 0.0, 0.0], [-0.28, 0.98, 0.87], [0.28, 0.3, 1.0], [0.1, -0.04, 1.65], [-0.5, 1.2, 0.3]):
    tau, _ = arm.gravity_and_lever([0.0, *q, 0.0])
    got = ns['gravity_torques'](*q)
    print(q, 'bridge', np.round(got, 4), 'urdf', np.round(tau[1:3], 4))
    assert np.allclose(got, tau[1:3], atol=0.02), (q, got, tau[1:3])
print('ok')
