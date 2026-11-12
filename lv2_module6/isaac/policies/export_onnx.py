"""TorchScript policy (.pt, rsl_rl or sb3) -> the self-contained ONNX the robot runs (ml::OnnxPolicy, weights inside, batch 1).

usage: python export_onnx.py policies/place_residual_sag.pt ../models/skills/place_cell/place_cell.onnx
       python export_onnx.py plan_next.pt ../models/plan/plan_next.onnx
train.py's ONNX copy leaves its weights in a side .onnx.data file that is not copied, so that .onnx has no weights — use this instead.
Writes metadata bee_test_x / bee_test_y (one random input + the torch output): ml::OnnxPolicy::load checks it, so a wrong export fails on load.
"""
import sys

import onnx
import torch

src, dst = sys.argv[1], sys.argv[2]
m = torch.jit.load(src, map_location="cpu").eval()
n = next(v for k, v in m.state_dict().items() if k.endswith("weight") and v.dim() == 2).shape[1]
torch.onnx.export(m, torch.zeros(1, n), dst, input_names=["obs"], output_names=["actions"], opset_version=17,
                  dynamo=False)
torch.manual_seed(0)
x = torch.randn(1, n)
with torch.no_grad():
    y = m(x)
model = onnx.load(dst)
for k, v in {"bee_source": src, "bee_test_x": x, "bee_test_y": y}.items():
    e = model.metadata_props.add()
    e.key, e.value = k, v if isinstance(v, str) else ",".join(f"{float(t):.9g}" for t in v.reshape(-1))
onnx.checker.check_model(model)
onnx.save_model(model, dst, save_as_external_data=False)
print(dst, n, tuple(y.shape))
