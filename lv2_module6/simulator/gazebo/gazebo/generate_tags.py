#!/usr/bin/env python3
"""AprilTag 자산 생성 — 퍽 6면·벽 미션 태그용 PNG/OBJ/MTL.

- tag36h11 (AprilTag 3 계열, OpenCV 기본 사전)
- 퍽 색 타입: red=1, orange=2, green=3 — 같은 색 퍽은 6면 모두 같은 ID를 쓴다.
- 벽 미션: SORT_COLOR=10, MOVE_TO_B=11, STAND_UP=12, LAY_DOWN=13
- B 구역 표지: 20
- PNG는 태그 주변에 색 테두리를 둬서 카메라가 색 타입도 같이 구분할 수 있게 한다.
- OBJ/MTL은 Gazebo OGRE(1)에서도 확실히 보이는 텍스처 평면이다.
  (PBR albedo_map은 ogre2 전용이라 이 월드의 ogre 렌더러에서 안 보일 수 있음)

사용:
  python3 generate_tags.py [--out assets/apriltags] [--force]
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
DEFAULT_OUT = HERE / "assets" / "apriltags"
FAMILY = "tag36h11"
TAG_PIXELS = 512
CANVAS_PIXELS = 640
QUIET_PIXELS = 32

COLORS = {
    "red": {"tag_id": 1, "label": "빨강", "rgb": [0.82, 0.10, 0.10], "bgr": [34, 34, 210]},
    "orange": {"tag_id": 2, "label": "주황", "rgb": [0.95, 0.45, 0.05], "bgr": [20, 120, 245]},
    "green": {"tag_id": 3, "label": "초록", "rgb": [0.10, 0.65, 0.20], "bgr": [50, 165, 36]},
}
MISSIONS = {
    "SORT_COLOR": {
        "tag_id": 10, "label": "색깔별 정리",
        "text": "빨강·주황·초록 퍽을 색깔별 구역으로 정리",
        "target": "color_bins", "bgr": [120, 70, 180],
    },
    "MOVE_TO_B": {
        "tag_id": 11, "label": "B 구역으로 옮기기",
        "text": "퍽을 B 구역으로 옮겨 정리",
        "target": "b_zone", "bgr": [220, 120, 20],
    },
    "STAND_UP": {
        "tag_id": 12, "label": "누운 퍽 세우기",
        "text": "누워 있는 퍽을 모두 세우기",
        "target": "cluster", "bgr": [200, 180, 20],
    },
    "LAY_DOWN": {
        "tag_id": 13, "label": "퍽 눕히기",
        "text": "세워진 퍽을 모두 눕히기",
        "target": "cluster", "bgr": [40, 180, 200],
    },
}
ZONE_TAGS = {
    "B_ZONE": {"tag_id": 20, "label": "B 구역", "bgr": [220, 120, 20]},
}


def make_marker_image(tag_id, bgr):
    """검정/흰 태그 + 흰 quiet zone + 색 테두리."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.generateImageMarker(dictionary, int(tag_id), TAG_PIXELS, borderBits=1)
    marker = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

    canvas = np.empty((CANVAS_PIXELS, CANVAS_PIXELS, 3), np.uint8)
    canvas[:] = bgr
    # marker 자체에 1bit 흰 테두리가 있지만, 인쇄/검출 여유를 위해 흰 여백을 한 번 더 둔다.
    quiet = cv2.copyMakeBorder(marker, QUIET_PIXELS, QUIET_PIXELS, QUIET_PIXELS, QUIET_PIXELS,
                               cv2.BORDER_CONSTANT, value=(255, 255, 255))
    y = (CANVAS_PIXELS - quiet.shape[0]) // 2
    x = (CANVAS_PIXELS - quiet.shape[1]) // 2
    canvas[y:y + quiet.shape[0], x:x + quiet.shape[1]] = quiet
    return canvas


def _write_plane_obj(path, tag_name, mtl_name):
    # local XY 평면, 법선 +Z. SDF에서 scale로 실제 크기를 준다.
    path.write_text(
        f"mtllib {mtl_name}\n"
        f"o {tag_name}\n"
        "v -0.5 -0.5 0\n"
        "v 0.5 -0.5 0\n"
        "v 0.5 0.5 0\n"
        "v -0.5 0.5 0\n"
        "vt 0 0\n"
        "vt 1 0\n"
        "vt 1 1\n"
        "vt 0 1\n"
        "vn 0 0 1\n"
        "usemtl tag\n"
        "f 1/1/1 2/2/1 3/3/1 4/4/1\n"
    )


def _write_plane_mtl(path, png_path):
    # map_Kd는 절대경로 — start.sh가 어디서 실행돼도 같은 텍스처를 찾는다.
    path.write_text(
        "newmtl tag\n"
        "Ka 1 1 1\n"
        "Kd 1 1 1\n"
        "Ks 0 0 0\n"
        "d 1\n"
        "illum 1\n"
        f"map_Kd {png_path}\n"
    )


def ensure_assets(out_dir=DEFAULT_OUT, force=False):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    generated = {}
    all_tags = {}
    for name, info in COLORS.items():
        all_tags[info["tag_id"]] = {"kind": "puck_color", "name": name,
                                    "label": info["label"], "rgb": info["rgb"],
                                    "bgr": info["bgr"]}
    for name, info in MISSIONS.items():
        all_tags[info["tag_id"]] = {"kind": "mission", "name": name,
                                    "label": info["label"], "text": info["text"],
                                    "bgr": info["bgr"]}
    for name, info in ZONE_TAGS.items():
        all_tags[info["tag_id"]] = {"kind": "zone", "name": name,
                                    "label": info["label"], "bgr": info["bgr"]}

    for tag_id, info in sorted(all_tags.items()):
        stem = f"tag_{tag_id}"
        png = out_dir / f"{stem}.png"
        obj = out_dir / f"{stem}.obj"
        mtl = out_dir / f"{stem}.mtl"
        if force or not (png.is_file() and obj.is_file() and mtl.is_file()):
            image = make_marker_image(tag_id, info.get("bgr", [200, 200, 200]))
            if not cv2.imwrite(str(png), image):
                raise RuntimeError(f"PNG 저장 실패: {png}")
            _write_plane_obj(obj, stem, f"{stem}.mtl")
            _write_plane_mtl(mtl, png.resolve())
        generated[tag_id] = {"png": str(png), "obj": str(obj), "mtl": str(mtl)}

    metadata = {
        "family": FAMILY,
        "tag_pixels": TAG_PIXELS,
        "canvas_pixels": CANVAS_PIXELS,
        "quiet_pixels": QUIET_PIXELS,
        "colors": COLORS,
        "missions": MISSIONS,
        "zone_tags": ZONE_TAGS,
        "tags": {str(k): v for k, v in sorted(all_tags.items())},
    }
    (out_dir / "tags.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n")
    return generated, metadata


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()
    generated, _ = ensure_assets(args.out, force=args.force)
    print(f"[generate_tags] {len(generated)} tags → {args.out}")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # pragma: no cover - start.sh에서 원인을 보여주기
        print(f"FAIL: {exc}", file=sys.stderr)
        raise
