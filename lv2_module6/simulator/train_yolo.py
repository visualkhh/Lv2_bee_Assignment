#!/usr/bin/env python3
"""YOLO nano 학습 (호스트, MPS/GPU) — export_dataset.py 산출물로 학습.

  cd lv2_module6/simulator && .venv/bin/python train_yolo.py [--epochs 100] [--model yolo11n.pt]

산출물 runs/yolo11n-beebot/weights/best.pt → perception_yolo/model/best.pt 로 복사 후 커밋.
"""
import argparse
import shutil
from pathlib import Path

HERE = Path(__file__).resolve().parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(HERE / "dataset" / "data.yaml"))
    ap.add_argument("--model", default="yolo11n.pt")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", default="mps")
    ap.add_argument("--name", default="yolo11n-beebot")
    args = ap.parse_args()

    from ultralytics import YOLO

    try:
        model = YOLO(args.model)
    except Exception:  # 첫 실행에 다운로드 실패하면 CPU로 재시도
        model = YOLO(args.model)
    # project 고정: ultralytics 전역 settings의 runs_dir을 무시하고 저장소 안에 둔다
    model.train(data=args.data, epochs=args.epochs, imgsz=args.imgsz,
                device=args.device, name=args.name, patience=20,
                workers=4, batch=-1, project=str(HERE / "runs"))
    best = HERE / "runs" / args.name / "weights" / "best.pt"
    dst = HERE.parent / "ros2_ws" / "model" / "yolo" / "best.pt"
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy(best, dst)
    print(f"OK: {best} → {dst} ({dst.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
