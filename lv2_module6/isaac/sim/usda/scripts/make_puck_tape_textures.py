"""퍽(직육면체·원기둥) 태그에 짧은 테이프 조각 디테일을 입혀 퍽 텍스처를 덮어쓴다.

원래 퍽 텍스처는 테이프를 단순하게 표현했다 (옆면을 감는 긴 광택 띠, 윗면·아랫면 전체 광택).
이걸 걷어내고, 태그 종이마다 종이보다 조금 큰 짧은 테이프 조각을 붙인 모양으로 바꾼다.

  - 입력은 항상 git 의 고정 커밋(ORIG_COMMIT) 원본 → 여러 번 돌려도 결과가 같다
  - 덮어쓰는 파일: puck_{box,cyl}_roughness.png, puck_{box,cyl}_normal.png,
    puck_{box,cyl}_basecolor_id0~11.png (tagmask 는 그대로)

원래 테이프 걷어내기
  - 직육면체 옆면: roughness·normal 의 FDM 적층 줄무늬는 열(u) 방향 1D 주기 패턴이라,
    띠 바깥에서 한 주기 템플릿을 뽑아 띠 안을 다시 채운다
  - 윗면·아랫면(둘 다): 전체 광택 → 옆면 플라스틱 평균 roughness, normal 은 평면

테이프 조각
  - ambientCG Tape004 (CC0, https://ambientcg.com/view?id=Tape004) 스캔을 실제 크기 그대로 쓴다
    (스캔 48 mm 테이프 폭 기준 px/mm 로 텍스처 px/mm 에 맞춤, 늘이지 않음)
  - 조각마다 찢긴 양 끝 + 양쪽 공장 가장자리를 살리고 가운데를 잘라 PIECE_MM 크기로 만든다
  - 태그 종이(tagmask)마다 하나씩, 종이 방향에 맞춰 살짝 비뚤게 붙이고 면 밖으로는 잘라낸다
  - 색은 쓰지 않고 roughness(광택 얼룩)·normal(주름, 가장자리 단차)·종이 위 미세한 얼룩만

usage:
  python make_puck_tape_textures.py --tape-src <Tape004_4K-JPG 압축 푼 폴더>   # 처음 (조각 캐시 생성)
  python make_puck_tape_textures.py                                          # 이후
"""

from __future__ import annotations

import argparse
import io
import subprocess
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from pxr import Usd

Image.MAX_IMAGE_PIXELS = None

SIM_DIR = Path(__file__).resolve().parents[2]
TEX_DIR = SIM_DIR / "source" / "texture" / "puck"
TAPE_DIR = TEX_DIR / "tape"
ORIG_COMMIT = "9be67946b3c5589bd3d3d1d2fc11156753dca2cd"  # 원래 퍽 텍스처가 마지막으로 바뀐 커밋

SHAPES = {"box": SIM_DIR / "usda/puck/box/geo.usd", "cyl": SIM_DIR / "usda/puck/cyl/geo.usd"}
IDS = range(12)

# Tape004 4K 에서 조각을 뽑을 연결요소 중심 (px) — 서로 다른 주름이 나오도록 여러 개
TAPE_SRC_STRIPS = [(1469, 446), (1407, 2561), (3286, 2036), (1578, 3604), (1468, 1518)]
TAPE_WIDTH_MM = 48.0  # 스캔 테이프 폭 (px/mm 기준)
PIECE_MM = {"box": 27.0, "cyl": 27.0}  # 조각 한 변 (종이 약 20 mm + 여백)
SPLICE_MM = 2.0
PAD_MM = 1.5
JITTER_DEG = 4.0
JITTER_MM = 0.8

OLD_TAPE_ROUGH = 60  # 이 값보다 낮으면 원래 테이프 광택 영역
TAPE_FLATTEN = 0.6  # 테이프 아래 FDM 결을 누르는 정도
TAPE_NORMAL = 1.0  # 주름 노멀 세기
TAPE_NORMAL_ON_PAPER = 0.6
PAPER_STAIN = 0.25  # 종이 위 테이프 얼룩 세기 (basecolor)
TAPE_DARKEN = 0.97

NORMAL_MAP_SCALE = 2.0  # 퍽 usda 의 ND_normalmap_float inputs:scale (FDM 결 때문에 2 배 증폭)

PAPER_THICK = 1.0
TAPE_THICK = 0.5
EDGE_BLUR_MM = 0.15
NORMAL_STRENGTH = 4.0


# ---------------------------------------------------------------- io


def load_orig(name: str, mode: str) -> np.ndarray:
    blob = subprocess.run(
        ["git", "show", f"{ORIG_COMMIT}:./{name}"], cwd=TEX_DIR, check=True, capture_output=True
    ).stdout
    return np.array(Image.open(io.BytesIO(blob)).convert(mode)).astype(np.float32)


def save(arr: np.ndarray, path: Path, mode: str):
    Image.fromarray(np.clip(arr + 0.5, 0, 255).astype(np.uint8), mode).save(path)


# ---------------------------------------------------------------- 메시 UV


def face_groups(geo: Path, size: int):
    """평면 방향별 면 그룹 → (uv 래스터 마스크, 텍스처 px/mm). 곡면(원기둥 옆면·베벨)은 제외."""
    stage = Usd.Stage.Open(str(geo))
    mesh = next(p for p in stage.Traverse() if p.GetTypeName() == "Mesh" and p.GetName() == "geo")
    pts = np.array(mesh.GetAttribute("points").Get())
    fvc = np.array(mesh.GetAttribute("faceVertexCounts").Get())
    fvi = np.array(mesh.GetAttribute("faceVertexIndices").Get())
    st = np.array(mesh.GetAttribute("primvars:st").Get())
    pv = pts[fvi]
    polys = {}
    o = 0
    for c in fvc:
        idx = np.arange(o, o + c)
        o += c
        v = pv[idx]
        n = np.cross(v[1] - v[0], v[2] - v[0])
        n /= np.linalg.norm(n) + 1e-12
        if np.abs(n).max() < 0.999:
            continue
        polys.setdefault(tuple(np.round(n).astype(int)), []).append(idx)
    groups = []
    for key, faces in polys.items():
        mask = np.zeros((size, size), np.uint8)
        for idx in faces:
            uv = st[idx]
            xy = np.stack([uv[:, 0] * size, (1 - uv[:, 1]) * size], -1)
            cv2.fillPoly(mask, [np.round(xy * 16).astype(np.int32)], 1, lineType=cv2.LINE_8, shift=4)
        ii = np.concatenate(faces)
        m = np.c_[pv[ii], np.ones(len(ii))]
        a = np.linalg.lstsq(m, st[ii], rcond=None)[0]
        uv_per_m = np.linalg.norm(a[:3, 0])
        groups.append({"key": key, "mask": mask, "ppm": uv_per_m * size / 1000})
    return groups


# ---------------------------------------------------------------- 테이프 조각


def make_pieces(src: Path):
    """Tape004 스캔 → 조각들 (찢긴 양 끝 + 양쪽 가장자리 유지, 가운데 잘라 PIECE_MM 정사각형)."""
    op = cv2.imread(str(src / "Tape004_4K-JPG_Opacity.jpg"), cv2.IMREAD_GRAYSCALE)
    col = cv2.imread(str(src / "Tape004_4K-JPG_Color.jpg"), cv2.IMREAD_COLOR)
    nrm = cv2.imread(str(src / "Tape004_4K-JPG_NormalGL.jpg"), cv2.IMREAD_COLOR)[..., ::-1]
    rough = cv2.imread(str(src / "Tape004_4K-JPG_Roughness.jpg"), cv2.IMREAD_GRAYSCALE)
    lum = cv2.cvtColor(col, cv2.COLOR_BGR2GRAY)
    _, lab = cv2.connectedComponents((op > 128).astype(np.uint8))
    TAPE_DIR.mkdir(parents=True, exist_ok=True)

    size_mm = max(PIECE_MM.values())
    for i, (cx, cy) in enumerate(TAPE_SRC_STRIPS):
        comp = (lab == lab[cy, cx]).astype(np.uint8)
        cnt, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        (rx, ry), (w, h), ang = cv2.minAreaRect(max(cnt, key=cv2.contourArea))
        if w < h:
            w, h, ang = h, w, ang + 90
        ppm = h / TAPE_WIDTH_MM
        pad = int(PAD_MM * ppm)
        ow, oh = int(w) + 2 * pad, int(h) + 2 * pad
        m = cv2.getRotationMatrix2D((rx, ry), ang, 1.0)
        m[:, 2] += (ow / 2 - rx, oh / 2 - ry)

        def warp(img, interp=cv2.INTER_LINEAR):
            return cv2.warpAffine(img, m, (ow, oh), flags=interp, borderMode=cv2.BORDER_REPLICATE)

        mask = cv2.dilate(warp(comp, cv2.INTER_NEAREST), np.ones((2 * pad + 1,) * 2, np.uint8))
        maps = {
            "alpha": warp(op).astype(np.float32) / 255 * mask,
            "normal": warp(nrm).astype(np.float32) / 127.5 - 1,
            "rough": warp(rough).astype(np.float32),
            "lum": warp(lum).astype(np.float32),
        }
        t = np.deg2rad(ang)  # 노멀 xy 도 이미지와 같이 회전 (GL: y 위쪽)
        n = maps["normal"]
        n[..., 0], n[..., 1] = (
            n[..., 0] * np.cos(t) - n[..., 1] * np.sin(t),
            n[..., 0] * np.sin(t) + n[..., 1] * np.cos(t),
        )

        keep = int(size_mm * ppm) + 2 * pad
        fade = int(SPLICE_MM * ppm)

        def splice(x, axis, hard=False):
            x = np.moveaxis(x, axis, 0)
            total, half = x.shape[0], keep // 2
            a, b = x[: half + fade], x[total - (keep - half) - fade :]
            wgt = np.linspace(0, 1, 2 * fade, dtype=np.float32).reshape(-1, *([1] * (x.ndim - 1)))
            if hard:
                wgt = (wgt >= 0.5).astype(np.float32)
            mid = a[half - fade :] * (1 - wgt) + b[: 2 * fade] * wgt
            return np.moveaxis(np.concatenate([a[: half - fade], mid, b[2 * fade :]]), 0, axis)

        # 길이: 찢긴 끝 하나부터 연속으로 잘라 반대쪽은 가위로 자른 직선 끝 (조각마다 왼/오른 끝 번갈아)
        cover = maps["alpha"].mean(0) > 0.5 * maps["alpha"].mean(0).max()
        cols = np.flatnonzero(cover)
        start = max(cols[0] - pad, 0) if i % 2 == 0 else min(cols[-1] + pad + 1, ow) - keep
        maps = {k: v[:, start : start + keep] for k, v in maps.items()}
        if i % 2:  # 찢긴 끝이 항상 같은 쪽에 오지 않도록 180° 돌려 저장
            maps = {k: v[::-1, ::-1].copy() for k, v in maps.items()}
            maps["normal"][..., :2] *= -1
        # 폭: 양쪽 공장 가장자리를 살리고 가운데를 잘라 맞춘다
        #     (모양은 섞으면 찢긴 끝이 어긋난 곳이 흐려지므로 이음매에서 딱 끊는다)
        maps = {k: splice(v, 0, hard=(k == "alpha")) for k, v in maps.items()}
        n = maps["normal"]
        n /= np.linalg.norm(n, axis=-1, keepdims=True)
        a = maps["alpha"]
        stain = maps["lum"] / max(maps["lum"][a > 0.9].mean(), 1)

        save(a * 255, TAPE_DIR / f"piece{i}_opacity.png", "L")
        save((n + 1) * 127.5, TAPE_DIR / f"piece{i}_normal.png", "RGB")
        save(maps["rough"], TAPE_DIR / f"piece{i}_roughness.png", "L")
        save(np.clip(stain * 128, 0, 255), TAPE_DIR / f"piece{i}_stain.png", "L")
        (TAPE_DIR / f"piece{i}_ppm.txt").write_text(f"{ppm:.6f}\n")
        print(f"piece{i}: {a.shape[1]}x{a.shape[0]} px, {ppm:.2f} px/mm")


def load_piece(i: int):
    def ld(s):
        return np.array(Image.open(TAPE_DIR / f"piece{i}_{s}.png")).astype(np.float32)

    return {
        "alpha": ld("opacity") / 255,
        "normal": ld("normal") / 127.5 - 1,
        "rough": ld("roughness"),
        "stain": ld("stain") / 128,
        "ppm": float((TAPE_DIR / f"piece{i}_ppm.txt").read_text()),
    }


def place_piece(piece, size, center, angle_deg, ppm_tex, piece_mm, flip):
    """조각을 텍스처 px 좌표 center 에 angle 로 놓는다. 크기는 실제 mm 유지(가장자리만 piece_mm 로 자름)."""
    p = {k: (v[::-1, ::-1].copy() if flip else v) for k, v in piece.items() if k != "ppm"}
    if flip:
        p["normal"][..., :2] *= -1
    h, w = p["alpha"].shape
    s = ppm_tex / piece["ppm"]
    t = np.deg2rad(angle_deg)
    rot = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])  # 이미지 좌표 (y 아래)
    lin = rot * s
    off = np.asarray(center) - lin @ np.array([(w - 1) / 2, (h - 1) / 2])
    m = np.hstack([lin, off[:, None]]).astype(np.float32)
    out = {}
    for k, v in p.items():
        fill = 1.0 if k == "stain" else 0.0
        out[k] = cv2.warpAffine(v, m, (size, size), flags=cv2.INTER_LINEAR, borderValue=fill)
    # 노멀 xy 회전: GL(y 위) → 이미지(y 아래)에서 회전 → GL
    nx, ny = out["normal"][..., 0], -out["normal"][..., 1]
    out["normal"][..., 0] = rot[0, 0] * nx + rot[0, 1] * ny
    out["normal"][..., 1] = -(rot[1, 0] * nx + rot[1, 1] * ny)
    return out


# ---------------------------------------------------------------- 원래 테이프 걷어내기


def periodic_fill(rough, normal, face, bad):
    """face 영역 안 bad 픽셀을 열 방향 주기 템플릿으로 다시 채운다 (행 방향은 불변)."""
    inner = cv2.erode(face.astype(np.uint8), np.ones((17, 17), np.uint8)).astype(bool)  # 면 안쪽만
    goodc = np.flatnonzero((inner.sum(0) > 100) & ~(bad & inner).any(0))
    w = inner[:, goodc].astype(np.float32)
    cnt = w.sum(0)
    prof = np.concatenate([((rough[:, goodc] * w).sum(0) / cnt)[:, None],
                           (normal[:, goodc] * w[..., None]).sum(0) / cnt[:, None]], 1)
    q = prof[:, 0] - prof[:, 0].mean()
    best = None
    for period in np.arange(8.0, 12.0, 0.0005):
        ph = (goodc % period) / period
        b = np.minimum((ph * 64).astype(int), 63)
        cnt = np.bincount(b, minlength=64)
        mean = np.bincount(b, q, minlength=64) / np.maximum(cnt, 1)
        err = ((q - mean[b]) ** 2).mean()
        if best is None or err < best[0]:
            best = (err, period)
    period = best[1]
    nb = 64
    ph = (goodc % period) / period
    b = np.minimum((ph * nb).astype(int), nb - 1)
    cnt = np.maximum(np.bincount(b, minlength=nb), 1)
    tmpl = np.stack([np.bincount(b, prof[:, c], minlength=nb) / cnt for c in range(4)], 1)
    recon = tmpl[b]
    print(f"    주기 {period:.4f} px, 재구성 오차 rough {np.abs(recon[:, 0] - prof[:, 0]).mean():.2f}")
    fill = bad & face
    r, c = np.nonzero(fill)
    v = tmpl[np.minimum(((c % period) / period * nb).astype(int), nb - 1)]
    rough[r, c] = v[:, 0]
    normal[r, c] = v[:, 1:]


def strip_old_tape(shape, rough, normal, groups):
    old = rough < OLD_TAPE_ROUGH
    plastic = rough[~old & (rough > 100)].mean()
    flat = np.array([127.5, 127.5, 255.0])
    for g in groups:
        face = g["mask"].astype(bool)
        bad = old & face
        if not bad.any():
            continue
        if g["key"][2] == 0:  # 옆면 (직육면체)
            print(f"  {shape} 옆면 {g['key']}: 띠 {bad.sum()} px 재구성")
            periodic_fill(rough, normal, face, bad)
        else:  # 윗면·아랫면
            rough[bad] = plastic
            normal[bad] = flat
    # 면 경계(베벨 래스터 틈)에 남은 옛 광택
    rest = rough < OLD_TAPE_ROUGH
    if rest.any():
        grow = cv2.dilate(rest.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool)
        filled = cv2.inpaint(np.clip(rough, 0, 255).astype(np.uint8), rest.astype(np.uint8), 5, cv2.INPAINT_TELEA)
        rough[rest] = filled[rest]
        normal[rest & grow] = flat
    return plastic


# ---------------------------------------------------------------- main


def process(shape: str):
    print(f"[{shape}]")
    rough = load_orig(f"puck_{shape}_roughness.png", "L")
    normal = load_orig(f"puck_{shape}_normal.png", "RGB")
    tagmask = load_orig(f"puck_{shape}_tagmask.png", "L") / 255
    size = rough.shape[0]
    groups = face_groups(SHAPES[shape], size)
    strip_old_tape(shape, rough, normal, groups)

    pieces = [load_piece(i) for i in range(len(TAPE_SRC_STRIPS))]
    rng = np.random.default_rng({"box": 1, "cyl": 2}[shape])

    n_lab, lab, stats, cen = cv2.connectedComponentsWithStats((tagmask > 0.5).astype(np.uint8))
    layers = np.zeros((size, size), np.float32)
    tape_xy = np.zeros((size, size, 2), np.float32)
    shade = np.ones((size, size), np.float32)
    rough_tape = rough.copy()
    for k in range(1, n_lab):
        cnt, _ = cv2.findContours((lab == k).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        (cx, cy), _, ang = cv2.minAreaRect(cnt[0])
        g = next(g for g in groups if g["mask"][int(cy), int(cx)])
        ppm = g["ppm"]
        jit = rng.uniform(-1, 1, 3)
        center = (cx + jit[0] * JITTER_MM * ppm, cy + jit[1] * JITTER_MM * ppm)
        pi = (k - 1) % len(pieces)
        flip = bool(((k - 1) // len(pieces) + k) % 2)
        st = place_piece(pieces[pi], size, center, ang + jit[2] * JITTER_DEG, ppm, PIECE_MM[shape], flip)
        face = cv2.GaussianBlur(g["mask"].astype(np.float32), (0, 0), 1.0)
        a = st["alpha"] * face
        layers += a
        tape_xy = tape_xy * (1 - a[..., None]) + st["normal"][..., :2] * a[..., None]
        rough_tape = rough_tape * (1 - a) + st["rough"] * a
        shade *= 1 - a * (1 - TAPE_DARKEN * (1 + (st["stain"] - 1) * PAPER_STAIN))
        print(f"  tag {k}: 면 {g['key']}, piece{pi}{' (180°)' if flip else ''}, {ppm:.1f} px/mm")
    tape = np.minimum(layers, 1)

    save(rough_tape, TEX_DIR / f"puck_{shape}_roughness.png", "L")

    flat = np.array([127.5, 127.5, 255.0])
    w = (tape * TAPE_FLATTEN)[..., None]
    n = normal * (1 - w) + flat * w
    n = n * (1 - tagmask[..., None]) + flat * tagmask[..., None]  # 종이 위는 평면
    nv = n / 127.5 - 1
    # 테이프·종이로 더하는 기울기는 머티리얼에서 NORMAL_MAP_SCALE 배 증폭되므로 미리 나눠 둔다
    strength = TAPE_NORMAL * (1 - tagmask) + TAPE_NORMAL_ON_PAPER * tagmask
    delta = tape_xy * strength[..., None]
    ppm = groups[0]["ppm"]
    height = cv2.GaussianBlur(tagmask * PAPER_THICK + layers * TAPE_THICK, (0, 0), EDGE_BLUR_MM * ppm)
    delta[..., 0] += -NORMAL_STRENGTH * cv2.Sobel(height, cv2.CV_32F, 1, 0, ksize=3) / 8
    delta[..., 1] += NORMAL_STRENGTH * cv2.Sobel(height, cv2.CV_32F, 0, 1, ksize=3) / 8
    nv[..., :2] += delta / NORMAL_MAP_SCALE
    nv /= np.linalg.norm(nv, axis=-1, keepdims=True)
    save((nv + 1) * 127.5, TEX_DIR / f"puck_{shape}_normal.png", "RGB")

    for i in IDS:
        base = load_orig(f"puck_{shape}_basecolor_id{i}.png", "L")
        save(base * shade, TEX_DIR / f"puck_{shape}_basecolor_id{i}.png", "L")
    print(f"  puck_{shape}_roughness/normal/basecolor_id0~{IDS[-1]} 저장")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tape-src", type=Path, help="ambientCG Tape004_4K-JPG 압축 푼 폴더")
    args = ap.parse_args()
    if args.tape_src:
        make_pieces(args.tape_src)
    for shape in SHAPES:
        process(shape)


if __name__ == "__main__":
    main()
