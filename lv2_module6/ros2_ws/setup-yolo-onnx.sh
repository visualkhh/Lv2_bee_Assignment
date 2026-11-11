#!/usr/bin/env bash
# Install the Python inference stack for the configured YOLO ONNX model.
# Keep apt's NumPy/OpenCV visible for ROS cv_bridge; do not install pip OpenCV.
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$HERE/../.." && pwd)"
YOLO_SOURCE_VENV="${YOLO_SOURCE_VENV:-$PROJECT_ROOT/.venv-yolo}"

if [ ! -x "$YOLO_SOURCE_VENV/bin/python" ]; then
  python3 -m venv --system-site-packages "$YOLO_SOURCE_VENV"
fi
if ! grep -qi '^include-system-site-packages = true$' "$YOLO_SOURCE_VENV/pyvenv.cfg"; then
  echo "FAIL: $YOLO_SOURCE_VENV must use --system-site-packages for ROS cv_bridge/OpenCV" >&2
  exit 2
fi

YOLO_PY="$YOLO_SOURCE_VENV/bin/python"
"$YOLO_PY" -m pip install --no-cache-dir \
  'torch==2.10.0' 'torchvision==0.25.0' \
  'onnxruntime==1.24.2' 'onnx==1.17.0'
# Ultralytics depends on opencv-python, but the ROS node must keep using the
# system cv2 built for cv_bridge, so install it without its dependency set.
"$YOLO_PY" -m pip install --no-cache-dir --no-deps \
  'ultralytics==8.4.210' 'nvidia-ml-py>=12.0.0' 'ultralytics-platform>=0.1.102'
"$YOLO_PY" -m pip install --no-cache-dir 'polars' 'ultralytics-thop' 'zxing-cpp'

MODEL="$PROJECT_ROOT/lv2_module6/models/yolo/bee_yolo11n_v5.onnx"
if [ -f "$HERE/install/perception/share/perception/model/bee_yolo11n_v5.onnx" ]; then
  MODEL="$HERE/install/perception/share/perception/model/bee_yolo11n_v5.onnx"
fi
YOLO_SITE_PACKAGES="$YOLO_SOURCE_VENV/lib/python3.12/site-packages"
PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$YOLO_SITE_PACKAGES" MODEL="$MODEL" python3 - <<'PY'
import os
import time

import cv2
import numpy as np
import onnxruntime as ort
import torch
from ultralytics import YOLO

model_path = os.environ["MODEL"]
providers = ort.get_available_providers()
assert "CPUExecutionProvider" in providers, providers
print(f"NumPy {np.__version__} ({np.__file__}) · OpenCV {cv2.__version__} ({cv2.__file__})")
print(f"PyTorch {torch.__version__} · CUDA torch={torch.cuda.is_available()}")
print(f"ONNX Runtime {ort.__version__} · providers={providers}")
model = YOLO(model_path, task="detect")
image = np.zeros((360, 640, 3), dtype=np.uint8)
for frame in range(3):
    start = time.perf_counter()
    result = model.predict(image, imgsz=(384, 640), device="cpu", verbose=False)[0]
    elapsed = time.perf_counter() - start
    print(f"frame={frame + 1} · inference={elapsed:.3f}s · boxes={len(result.boxes)}")
print(f"YOLO ONNX smoke test OK · {len(result.names)} classes")
PY

echo "Done. ros2_ws/start.sh automatically adds $YOLO_SOURCE_VENV to PYTHONPATH."
