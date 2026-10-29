"""종이상자(shelf) 윗면·아랫면 중앙에 AprilTag를 테이프로 붙인 텍스처 생성.

원본 shelf 텍스처(box_basecolor / box_roughness / box_normal)는 읽기만 하고,
태그를 합성한 새 텍스처를 같은 폴더에 쓴다.

  box_tag_basecolor_L1.png  (ID 12)
  box_tag_basecolor_L2.png  (ID 13)
  box_tag_basecolor_L3.png  (ID 14)
  box_tag_roughness.png     (L1~L3 공통)
  box_tag_normal.png        (L1~L3 공통)

태그 규격 (실물 출력물 기준)
  - tag36h11, 검은 사각형 48 mm (셀 6 mm × 8), 흰 종이 60 × 60 mm
  - 비트 패턴은 OpenCV generateImageMarker 를 180° 돌린 것 = 실제 출력물과 같은 방향
  - 윗면: 태그 위쪽 = 상자 +Y / 아랫면(밖에서 볼 때): 태그 위쪽 = 상자 -Y
    (상자를 X축으로 뒤집으면 아랫면이 윗면과 같은 방향이 된다)

테이프 처리
  - 48 mm 투명 테이프 두 줄을 상자 X 방향으로 나란히 붙여 종이를 덮는다
    (두 줄이 가운데서 16 mm 겹쳐 전체 약 80 × 80 mm, 겹친 곳은 두 겹)
  - 테이프 조각의 모양(찢긴 끝)·주름(normal)·광택 얼룩(roughness)은
    ambientCG Tape004 (CC0, https://ambientcg.com/view?id=Tape004) 스캔에서 가져온다.
    색은 갈색 포장테이프라 쓰지 않고, 밝기 변화만 투명 테이프 얼룩으로 쓴다.
  - 조각 두 개를 잘라 펴서 source/texture/shelf/tape/ 에 캐시해 둔다 (--tape-src 로 재생성)

UV 매핑 (geo.usd 의 /shelf/geo 에서 fit, 오차 < 2e-6)
  윗면  u = 2.40133 x + 0.24854,  v =  2.40133 y + 0.24854
  아랫면 u = 2.40133 x + 0.74756,  v = -2.40133 y + 0.24854

usage:
  python make_shelf_tag_textures.py
  python make_shelf_tag_textures.py --tape-src <Tape004_4K-JPG 압축 푼 폴더>   # 테이프 조각 캐시 재생성
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

Image.MAX_IMAGE_PIXELS = None

TEX_DIR = Path(__file__).resolve().parents[2] / "source" / "texture" / "shelf"
TAPE_DIR = TEX_DIR / "tape"

TAGS = {"L1": 12, "L2": 13, "L3": 14}

UV_PER_M = 2.40133
FACES = {  # (u0, v0, v 부호) — 면 중심의 uv, 상자 y 와 v 의 방향
    "top": (0.24854, 0.24854, +1),
    "bottom": (0.74756, 0.24854, -1),
}

TAG_BLACK_MM = 48.0
PAPER_MM = 60.0
TAPE_WIDTH_MM = 48.0  # 테이프 한 줄 폭
TAPE_LENGTH_MM = 80.0  # 한 줄 길이 (상자 X 방향)
TAPE_SPAN_MM = 80.0  # 두 줄을 합친 폭 (상자 Y 방향) → 겹침 = 2 × 48 - 80 = 16 mm

# Tape004 4K 에서 쓸 조각 (opacity 연결요소 중심, px). 한쪽 끝이 구겨진 조각 + 깨끗한 조각
TAPE_SRC_STRIPS = {"a": (1469, 446), "b": (1407, 2561)}
TAPE_SPLICE_MM = 4.0  # 가운데를 잘라 낸 이음매 크로스페이드 폭
TAPE_PAD_MM = 3.0  # 조각 주변 여백 (부드러운 가장자리 보존)

PAPER_RGB = 238
INK_RGB = 14
TAPE_DARKEN = 0.93
TAPE_STAIN = 0.6  # 스캔 밝기 변화를 얼룩으로 얼마나 반영할지
TAPE_FLATTEN = 0.75  # 테이프 아래 골판지 노멀을 평평하게 누르는 정도
TAPE_NORMAL = 1.0  # 스캔 주름 노멀 세기

PAPER_THICK = 1.0  # 높이맵 상대값 (종이 단차)
TAPE_THICK = 0.5  # 테이프 가장자리 단차
EDGE_BLUR_MM = 0.4
NORMAL_STRENGTH = 6.0

SS = 4  # 태그 경계 안티에일리어싱 슈퍼샘플


def tag_bits(tag_id: int) -> np.ndarray:
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    m = cv2.aruco.generateImageMarker(d, tag_id, 8, borderBits=1)
    return np.rot90(m, 2) > 127  # True = 흰 셀


def px_per_mm(size: int) -> float:
    return size * UV_PER_M / 1000


def face_coords(size: int, face: str, ss: int = 1):
    """텍스처 픽셀 → 면 로컬 좌표 (mm). a: +u 방향, b: +v 방향(텍스처 위쪽)."""
    u0, v0, _ = FACES[face]
    n = size * ss
    t = (np.arange(n) + 0.5) / n
    a = (t - u0) / UV_PER_M * 1000.0
    b = ((1.0 - t) - v0) / UV_PER_M * 1000.0
    return a[None, :], b[:, None]


def tag_coverage(size: int, face: str, bits: np.ndarray):
    """면 전체 크기 배열로 종이 coverage, 잉크 coverage (0~1) 반환. 태그 근처만 슈퍼샘플."""
    u0, v0, _ = FACES[face]
    half = PAPER_MM / 2 / 1000 * UV_PER_M
    c0 = int((u0 - half) * size) - 4
    c1 = int((u0 + half) * size) + 4
    r0 = int((1 - v0 - half) * size) - 4
    r1 = int((1 - v0 + half) * size) + 4

    a_all, b_all = face_coords(size, face, SS)
    a = a_all[:, c0 * SS : c1 * SS]
    b = b_all[r0 * SS : r1 * SS, :]

    paper = (np.abs(a) <= PAPER_MM / 2) & (np.abs(b) <= PAPER_MM / 2)
    hb = TAG_BLACK_MM / 2
    inside = (np.abs(a) < hb) & (np.abs(b) < hb)
    cell = TAG_BLACK_MM / 8
    col = np.clip(((a + hb) // cell).astype(int), 0, 7)
    row = np.clip(((hb - b) // cell).astype(int), 0, 7)
    ink = inside & ~bits[row, col]

    def down(x):
        h, w = x.shape
        return x.reshape(h // SS, SS, w // SS, SS).mean(axis=(1, 3))

    paper_full = np.zeros((size, size), np.float32)
    ink_full = np.zeros((size, size), np.float32)
    paper_full[r0:r1, c0:c1] = down(paper.astype(np.float32))
    ink_full[r0:r1, c0:c1] = down(ink.astype(np.float32))
    return paper_full, ink_full


def blur_mm(x: np.ndarray, size: int) -> np.ndarray:
    return cv2.GaussianBlur(x, (0, 0), EDGE_BLUR_MM * px_per_mm(size))


# ---------------------------------------------------------------- 테이프 조각


def extract_tape_strips(src: Path):
    """Tape004 스캔에서 조각을 골라 수평으로 펴고, 가운데를 잘라 TAPE_LENGTH_MM 로 줄여 캐시."""
    op = cv2.imread(str(src / "Tape004_4K-JPG_Opacity.jpg"), cv2.IMREAD_GRAYSCALE)
    col = cv2.imread(str(src / "Tape004_4K-JPG_Color.jpg"), cv2.IMREAD_COLOR)
    nrm = cv2.imread(str(src / "Tape004_4K-JPG_NormalGL.jpg"), cv2.IMREAD_COLOR)[..., ::-1]
    rough = cv2.imread(str(src / "Tape004_4K-JPG_Roughness.jpg"), cv2.IMREAD_GRAYSCALE)
    lum = cv2.cvtColor(col, cv2.COLOR_BGR2GRAY)

    _, lab = cv2.connectedComponents((op > 128).astype(np.uint8))
    TAPE_DIR.mkdir(parents=True, exist_ok=True)
    for name, (cx, cy) in TAPE_SRC_STRIPS.items():
        comp = (lab == lab[cy, cx]).astype(np.uint8)
        cnt, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        (rx, ry), (w, h), ang = cv2.minAreaRect(max(cnt, key=cv2.contourArea))
        if w < h:  # 긴 변을 가로로
            w, h, ang = h, w, ang + 90
        ppm = h / TAPE_WIDTH_MM
        pad = int(TAPE_PAD_MM * ppm)
        out_w, out_h = int(w) + 2 * pad, int(h) + 2 * pad
        m = cv2.getRotationMatrix2D((rx, ry), ang, 1.0)
        m[:, 2] += (out_w / 2 - rx, out_h / 2 - ry)

        def warp(img, interp=cv2.INTER_LINEAR):
            return cv2.warpAffine(img, m, (out_w, out_h), flags=interp, borderMode=cv2.BORDER_REPLICATE)

        mask = cv2.dilate(warp(comp, cv2.INTER_NEAREST), np.ones((2 * pad + 1,) * 2, np.uint8))
        alpha = warp(op).astype(np.float32) / 255 * mask
        # 노멀 벡터도 이미지와 같이 회전 (GL: x 오른쪽, y 위쪽 → 화면 반시계 = 수학 반시계)
        n = warp(nrm).astype(np.float32) / 127.5 - 1
        t = np.deg2rad(ang)
        nx = n[..., 0] * np.cos(t) - n[..., 1] * np.sin(t)
        ny = n[..., 0] * np.sin(t) + n[..., 1] * np.cos(t)
        n[..., 0], n[..., 1] = nx, ny
        r = warp(rough).astype(np.float32)
        lm = warp(lum).astype(np.float32)

        # 양 끝(찢긴 모양)은 살리고 가운데를 잘라 길이를 맞춘다
        keep = int(TAPE_LENGTH_MM * ppm) + 2 * pad
        fade = int(TAPE_SPLICE_MM * ppm)
        half = keep // 2

        def splice(x):
            left, right = x[:, : half + fade], x[:, out_w - (keep - half) - fade :]
            wgt = np.linspace(0, 1, 2 * fade, dtype=np.float32)
            wgt = wgt.reshape(1, -1, *([1] * (x.ndim - 2)))
            mid = left[:, half - fade :] * (1 - wgt) + right[:, : 2 * fade] * wgt
            return np.concatenate([left[:, : half - fade], mid, right[:, 2 * fade :]], axis=1)

        alpha, n, r, lm = (splice(x) for x in (alpha, n, r, lm))
        n /= np.linalg.norm(n, axis=-1, keepdims=True)
        stain = lm / max(lm[alpha > 0.9].mean(), 1)  # 1 ± 밝기 변화

        def save(arr, suffix, mode):
            Image.fromarray(np.clip(arr + 0.5, 0, 255).astype(np.uint8), mode).save(
                TAPE_DIR / f"tape_strip_{name}_{suffix}.png"
            )

        save(alpha * 255, "opacity", "L")
        save((n + 1) * 127.5, "normal", "RGB")
        save(r, "roughness", "L")
        save(np.clip(stain * 128, 0, 255), "stain", "L")  # 128 = 평균 밝기
        print(f"tape_strip_{name}: {alpha.shape[1]}x{alpha.shape[0]} px, {ppm:.2f} px/mm")
        (TAPE_DIR / f"tape_strip_{name}_ppm.txt").write_text(f"{ppm:.6f}\n")


def load_tape_strip(name: str):
    def ld(suffix):
        return np.array(Image.open(TAPE_DIR / f"tape_strip_{name}_{suffix}.png")).astype(np.float32)

    ppm = float((TAPE_DIR / f"tape_strip_{name}_ppm.txt").read_text())
    return {
        "alpha": ld("opacity") / 255,
        "normal": ld("normal") / 127.5 - 1,
        "rough": ld("roughness"),
        "stain": ld("stain") / 128,
        "ppm": ppm,
    }


def place_strip(strip: dict, size: int, face: str, b_mm: float, rot180: bool):
    """조각을 면의 (a=0, b=b_mm) 중심에 놓은 텍스처 크기 배열들을 반환."""
    u0, v0, _ = FACES[face]
    s = px_per_mm(size) / strip["ppm"]
    h, w = strip["alpha"].shape
    small = {}
    for k in ("alpha", "normal", "rough", "stain"):
        x = cv2.resize(strip[k], (round(w * s), round(h * s)), interpolation=cv2.INTER_AREA)
        if rot180:
            x = x[::-1, ::-1].copy()
            if k == "normal":
                x[..., :2] *= -1
        small[k] = x
    sh, sw = small["alpha"].shape
    cc = u0 * size - 0.5
    cr = (1 - (v0 + b_mm / 1000 * UV_PER_M)) * size - 0.5
    m = np.float32([[1, 0, cc - (sw - 1) / 2], [0, 1, cr - (sh - 1) / 2]])
    out = {}
    for k, x in small.items():
        fill = {"alpha": 0, "normal": 0, "rough": 0, "stain": 1}[k]
        out[k] = cv2.warpAffine(
            x, m, (size, size), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=fill
        )
    return out


# ---------------------------------------------------------------- main


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tape-src", type=Path, help="ambientCG Tape004_4K-JPG 압축 푼 폴더")
    args = ap.parse_args()
    if args.tape_src:
        extract_tape_strips(args.tape_src)

    base = np.array(Image.open(TEX_DIR / "box_basecolor.png").convert("RGB")).astype(np.float32)
    rough = np.array(Image.open(TEX_DIR / "box_roughness.png")).astype(np.float32)
    normal = np.array(Image.open(TEX_DIR / "box_normal.png").convert("RGB")).astype(np.float32)
    size = base.shape[0]
    assert base.shape[:2] == (size, size) == rough.shape == normal.shape[:2]

    strips = {k: load_tape_strip(k) for k in TAPE_SRC_STRIPS}

    paper = np.zeros((size, size), np.float32)
    inks = {k: np.zeros((size, size), np.float32) for k in TAGS}
    for face in FACES:
        for k, tag_id in TAGS.items():
            p, ink = tag_coverage(size, face, tag_bits(tag_id))
            inks[k] = np.maximum(inks[k], ink)
        paper = np.maximum(paper, p)  # 종이 영역은 ID 와 무관

    # 테이프: 면마다 두 줄 (아래 줄 먼저, 위 줄이 덮음). 아랫면은 조각을 180° 돌려 변화를 준다
    off = (TAPE_SPAN_MM - TAPE_WIDTH_MM) / 2
    layers = np.zeros((size, size), np.float32)
    tape_xy = np.zeros((size, size, 2), np.float32)
    rough_out = rough.copy()
    shade = np.ones((size, size), np.float32)  # basecolor 곱
    for face in FACES:
        for name, b_mm in (("a", off), ("b", -off)):
            st = place_strip(strips[name], size, face, b_mm, rot180=(face == "bottom"))
            a = st["alpha"]
            layers += a
            tape_xy = tape_xy * (1 - a[..., None]) + st["normal"][..., :2] * a[..., None]
            rough_out = rough_out * (1 - a) + st["rough"] * a
            stain = 1 + (st["stain"] - 1) * TAPE_STAIN
            shade *= 1 - a * (1 - TAPE_DARKEN * stain)
    tape = np.minimum(layers, 1.0)

    # roughness
    Image.fromarray(np.clip(rough_out + 0.5, 0, 255).astype(np.uint8), "L").save(TEX_DIR / "box_tag_roughness.png")

    # normal: 골판지 결 누르기 → 종이 위는 완전 평면 → 테이프 주름 → 높이맵 단차
    flat = np.array([128.0, 128.0, 255.0])
    n = normal.copy()
    w = (tape * TAPE_FLATTEN)[..., None]
    n = n * (1 - w) + flat * w
    n = n * (1 - paper[..., None]) + flat * paper[..., None]
    nv = n / 127.5 - 1.0
    nv[..., :2] += tape_xy * TAPE_NORMAL

    height = blur_mm(paper * PAPER_THICK + layers * TAPE_THICK, size)
    dh_dc = cv2.Sobel(height, cv2.CV_32F, 1, 0, ksize=3) / 8
    dh_dr = cv2.Sobel(height, cv2.CV_32F, 0, 1, ksize=3) / 8
    # OpenGL 노멀 (G = +v = 이미지 위쪽)
    nv[..., 0] += -NORMAL_STRENGTH * dh_dc
    nv[..., 1] += NORMAL_STRENGTH * dh_dr
    nv /= np.linalg.norm(nv, axis=-1, keepdims=True)
    Image.fromarray(np.clip((nv + 1) * 127.5 + 0.5, 0, 255).astype(np.uint8), "RGB").save(
        TEX_DIR / "box_tag_normal.png"
    )

    # basecolor (종이 위 테이프는 거의 투명: 얼룩을 약하게)
    paper_shade = 1 - (1 - shade) * 0.2
    for k, ink in inks.items():
        tag_col = (PAPER_RGB * (1 - ink) + INK_RGB * ink) * paper_shade
        out = base * shade[..., None] * (1 - paper[..., None]) + tag_col[..., None] * paper[..., None]
        Image.fromarray(np.clip(out + 0.5, 0, 255).astype(np.uint8), "RGB").save(
            TEX_DIR / f"box_tag_basecolor_{k}.png"
        )
        print(f"box_tag_basecolor_{k}.png  (ID {TAGS[k]})")
    print("box_tag_roughness.png, box_tag_normal.png")


if __name__ == "__main__":
    main()
