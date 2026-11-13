#!/usr/bin/env python3
"""Image ownership/stride/endian regression tests; no ROS or GPU required."""
import array
import ast
import pathlib
import types
import numpy as np

src = pathlib.Path(__file__).resolve().parents[1] / 'scripts' / 'cuvslam_slam.py'
tree = ast.parse(src.read_text())
fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'image')
ns = {'np': np}
exec(compile(ast.Module(body=[fn], type_ignores=[]), str(src), 'exec'), ns)
image = ns['image']

def msg(data, width, height, step, endian=False):
    return types.SimpleNamespace(data=data, width=width, height=height, step=step, is_bigendian=endian)

rgb = array.array('B', range(18))
m = msg(rgb, 3, 2, 9)
a = image(m, np.uint8, 3)
assert a.shape == (2, 3, 3) and a.flags.c_contiguous
assert np.shares_memory(a, np.frombuffer(rgb, np.uint8))
assert a[1, 2, 2] == 17
# Buffer remains alive after the message object is gone.
del m, rgb
assert a[1, 2, 2] == 17

padded = array.array('B', [1, 2, 3, 4, 99, 5, 6, 7, 8, 99])
b = image(msg(padded, 4, 2, 5), np.uint8, 1)
assert b.tolist() == [[1, 2, 3, 4], [5, 6, 7, 8]] and b.flags.c_contiguous
assert not np.shares_memory(b, np.frombuffer(padded, np.uint8))

for endian in (False, True):
    wire = np.array([[0, 499, 500], [1000, 2000, 65535]], dtype='>u2' if endian else '<u2')
    raw = array.array('B', wire.tobytes())
    before = raw.tobytes()
    depth = image(msg(raw, 3, 2, 6, endian), np.uint16, 1)
    assert depth.dtype == np.dtype(np.uint16) and depth.flags.c_contiguous
    assert depth.tolist() == [[0, 499, 500], [1000, 2000, 65535]]
    assert raw.tobytes() == before
    # Existing min-depth mask semantics are unchanged at the exact boundary.
    assert ((depth > 0) & (depth < 500)).astype(np.uint8).tolist() == [[0, 1, 0], [0, 0, 0]]

for bad in (msg(array.array('B', [1]), 3, 2, 3), msg(array.array('B', [1]*6), 3, 2, 2)):
    try:
        image(bad, np.uint8, 1)
    except ValueError:
        pass
    else:
        raise AssertionError('malformed frame must fail')
print('test_cuvslam_image OK: shared packed storage, padded rows, lifetime, endian, mask, invalid frames')
