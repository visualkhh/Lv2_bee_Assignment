"""이미지 준비 — 원본 2장(pole·gripper) → 원본+마킹 4장으로 펼쳐 백엔드에 넘긴다.

요청: images[{data_b64, camera, format, width, height}]
  format "jpeg"(기본) 그대로 · "rgb8"이면 width×height raw RGB → JPEG 변환 (컨테이너 OpenCV 없이 보내기용).
마킹: objects[].px{camera: [u, v]} (컨테이너가 3D pose→픽셀 투영, map→카메라 tf) — 번호=id.
Pillow 없으면 마킹 없이 원본만 (서빙은 계속).
"""

import base64
import io

try:
    from PIL import Image, ImageDraw
except ImportError:
    Image = None

JPEG_Q = 70
MARK_R = 14


def to_jpeg(raw, fmt, width, height):
    if fmt == "rgb8":
        if Image is None or not isinstance(width, int) or not isinstance(height, int):
            return None
        if len(raw) != width * height * 3:
            return None
        img = Image.frombytes("RGB", (width, height), raw)
    else:
        if Image is None:
            return raw if raw.startswith(b"\xff\xd8\xff") else None
        try:
            img = Image.open(io.BytesIO(raw))
        except Exception:  # noqa: BLE001
            return None
    buf = io.BytesIO()
    img.convert("RGB").save(buf, "JPEG", quality=JPEG_Q)
    return buf.getvalue()


def mark(jpeg_bytes, markers):
    """markers [(id, u, v)] — 화면 안만. 번호=id. → (바이트, 그렸는지)."""
    if Image is None:
        return jpeg_bytes, False
    try:
        img = Image.open(io.BytesIO(jpeg_bytes)).convert("RGB")
    except Exception:  # noqa: BLE001
        return jpeg_bytes, False
    w, h = img.size
    pts = []
    for oid, u, v in markers:
        try:
            u, v = int(u), int(v)
        except (TypeError, ValueError):
            continue
        if 0 <= u < w and 0 <= v < h:
            pts.append((str(oid), u, v))
    if not pts:
        return jpeg_bytes, False
    d = ImageDraw.Draw(img)
    for label, u, v in pts:
        d.ellipse([u - MARK_R, v - MARK_R, u + MARK_R, v + MARK_R], outline=(255, 0, 0), width=3)
        d.text((u + MARK_R + 2, v - MARK_R - 8), label, fill=(255, 255, 0), stroke_width=2,
               stroke_fill=(0, 0, 0))
    buf = io.BytesIO()
    img.save(buf, "JPEG", quality=JPEG_Q)
    return buf.getvalue(), True


def to_png(raw, fmt, width, height):
    """로그용 — JPEG·rgb8 → PNG 바이트. 실패면 None."""
    if Image is None:
        return None
    try:
        if fmt == "rgb8" and isinstance(width, int) and isinstance(height, int) \
                and len(raw) == width * height * 3:
            img = Image.frombytes("RGB", (width, height), raw)
        else:
            img = Image.open(io.BytesIO(raw))
        buf = io.BytesIO()
        img.convert("RGB").save(buf, "PNG")
        return buf.getvalue()
    except Exception:  # noqa: BLE001
        return None


def prepare(req):
    """→ [{"camera", "kind"(raw|marked), "data_b64"(jpeg)}] — 최대 4장, 순서 pole→gripper."""
    out = []
    images = req.get("images", []) if isinstance(req, dict) else []
    objects = req.get("objects", []) if isinstance(req, dict) else []
    for im in images[:2]:
        if not isinstance(im, dict):
            continue
        try:
            raw = base64.b64decode(im.get("data_b64", ""), validate=True)
        except Exception:  # noqa: BLE001
            continue
        jpeg = to_jpeg(raw, im.get("format", "jpeg"), im.get("width"), im.get("height"))
        if jpeg is None:
            continue
        cam = im.get("camera", "")
        markers = []
        for o in objects:
            if not isinstance(o, dict):
                continue
            px = o.get("px", {})
            if isinstance(px, dict) and cam in px and isinstance(px[cam], (list, tuple)):
                markers.append((o.get("id", "?"), px[cam][0] if len(px[cam]) > 0 else -1,
                                px[cam][1] if len(px[cam]) > 1 else -1))
        out.append({"camera": cam, "kind": "raw",
                    "data_b64": base64.b64encode(jpeg).decode()})
        if markers:
            marked, drew = mark(jpeg, markers)
            if drew:
                out.append({"camera": cam, "kind": "marked",
                            "data_b64": base64.b64encode(marked).decode()})
    return out
