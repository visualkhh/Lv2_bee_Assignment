"""Re-export rsl_rl TorchScript policies to self-contained ONNX (weights inside, batch 1) for ml/rl_arm_correction.

train.py's ONNX copy leaves its weights in a side .onnx.data file that is not copied, so that .onnx has no weights.
usage: python export_onnx.py policies/place_residual_sag.pt ../ros2_ws/model/skills/place_cell/place_cell.onnx
(robot: rl_arm_correction runs model/skills/<skill>/<skill>.policy from ml/scripts/export_policy.py — the .onnx is the reference copy)
also writes <dst>.check = one random input + the torch output (float32), for the C++ test
"""
import sys

import numpy as np
import onnx
import torch

src, dst = sys.argv[1], sys.argv[2]
m = torch.jit.load(src, map_location="cpu")
n = m.obs_normalizer._mean.shape[1]
torch.onnx.export(m, torch.zeros(1, n), dst, input_names=["obs"], output_names=["actions"], opset_version=17,
                  dynamo=False)
onnx.checker.check_model(onnx.load(dst))  # fails if the weights went to a side file
x = torch.randn(1, n)
with torch.no_grad():
    y = m(x)
torch.cat([x.ravel(), y.ravel()]).numpy().astype(np.float32).tofile(dst + ".check")
print(dst, n)