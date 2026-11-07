#!/usr/bin/env python3
"""YOLO 데이터셋 자동 생성 (호스트 — 도커 불필요) — documents/mission/README.md 장면.

시뮬레이터의 미션 배치 코드(main.Sim: 미션 지역 · QR 판 · 누운 퍽 · B 지역 · 장애물 · 사람)로 장면을 깔고,
pole_cam(멀리 1~3m · 가까이 0.4~1m)과 gripper_cam(집기 직전 자세) 시점으로 렌더 → 정답 위치를 투영해 YOLO 라벨.
장면마다 바닥·벽·배경 텍스처, 조명(0.55~1.45배), 카메라 잡음을 무작위로 (도메인 무작위화).
클래스: 0 puck(서 있음) · 1 puck_lying(누움) · 2 goal_zone(B 지역) · 3 station_zone(미션 지역 판) · 4 qr_board · 5 obstacle · 6 person · 7 traffic_light
퍽 색 · 신호등 상태(빨강·노랑·초록·꺼짐)는 클래스로 나누지 않는다 — perception이 박스 안 HSV로 판별.

  cd lv2_module6/simulator && .venv/bin/python export_dataset.py --out dataset --n 1000 --views 3
  → dataset/{train,val}/{images,labels}/ + dataset/data.yaml (ultralytics용)

URDF: 테스트베드가 떠 있으면 /robot_description, 아니면 lv2_module6/urdf/beebot_mesh/urdf.xacro를 xacro로 펼침 (기본 모델).
"""
import argparse
import math
import random
import shutil
from pathlib import Path

import cv2
import numpy as np

import maze
import scene
from bridge import read_msg

HERE = Path(__file__).resolve().parent
CLASSES = ["puck", "puck_lying", "goal_zone", "station_zone", "qr_board", "obstacle", "person", "traffic_light"]
IMG_W, IMG_H = 640, 480  # fake_camera 출력과 동일


def corners_3d(p):
    """물체 점유 8점 (world). z0=바닥 띄움."""
    zo = float(p.get("z", 0.0))
    x, y, yaw = p["x"], p["y"], p.get("yaw", 0.0)
    cy, sy = math.cos(yaw), math.sin(yaw)
    if p.get("kind") == "traffic_light":  # 폴 + 헤드박스 — 렌더(scene._traffic_light)와 같은 치수, 바닥부터 헤드 위까지
        hsx, hsy, _, hz1 = scene.TL_HEAD
        pts = [(dx, dy, dz) for dx in (-hsx / 2, hsx / 2) for dy in (-hsy / 2, hsy / 2) for dz in (0, hz1)]
    elif p["shape"] == "sphere":
        r = p["size"][0]
        pts = [(dx, dy, dz) for dx in (-r, r) for dy in (-r, r) for dz in (0, 2 * r)]
    elif p["shape"] == "cylinder" and p.get("lying"):  # 누운 원기둥: 축이 yaw 방향 수평, 중심 높이 = 반지름
        r, h = p["size"][0], p["size"][1]
        pts = [(ax, s_ * r, r + c_ * r) for ax in (-h / 2, h / 2) for s_, c_ in ((1, 0), (-1, 0), (0, 1), (0, -1))]
    elif p["shape"] == "cylinder":
        r, h = p["size"][0], p["size"][1]
        pts = [(r * math.cos(a), r * math.sin(a), dz)
               for a in (0, math.pi / 2, math.pi, 3 * math.pi / 2) for dz in (0, h)]
    else:
        sx = p.get("sx", p["size"][0])
        syy = p.get("sy", p["size"][0])
        h = p["size"][1]
        pts = [(dx, dy, dz) for dx in (-sx / 2, sx / 2) for dy in (-syy / 2, syy / 2)
               for dz in (0, h)]
    R = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    out = []
    for dx, dy, dz in pts:
        w = R @ np.array([dx, dy, dz])
        out.append((x + w[0], y + w[1], zo + w[2]))
    return out


def project_box(p, T_cw, fx, cx, cy, W, H, depth):
    """→ (xc, yc, w, h) 정규화 bbox 또는 None (뒤·프레임밖·가려짐·너무작음 제외)."""
    pts = []
    for X, Y, Z in corners_3d(p):
        q = T_cw @ np.array([X, Y, Z, 1.0])
        if q[2] < 0.05:
            return None
        pts.append((q[0], q[1], q[2]))
    us = [fx * q[0] / q[2] + cx for q in pts]
    vs = [fx * q[1] / q[2] + cy for q in pts]
    x0, x1, y0, y1 = min(us), max(us), min(vs), max(vs)
    if x1 < 0 or x0 >= W or y1 < 0 or y0 >= H:
        return None
    x0c, x1c, y0c, y1c = max(x0, 0), min(x1, W - 1), max(y0, 0), min(y1, H - 1)
    # 가림 검사: 화면에 보이는 부분의 중심 픽셀 깊이가 물체보다 앞이면 벽·다른 물체 뒤 (잘린 물체도 보이는 쪽으로 판정)
    u, v = int((x0c + x1c) / 2), int((y0c + y1c) / 2)
    dm = depth[v, u] / 1000.0
    if dm > 0 and dm < min(q[2] for q in pts) - 0.08:
        return None
    if (x1c - x0c) * (y1c - y0c) < 64:
        return None
    return ((x0c + x1c) / 2 / W, (y0c + y1c) / 2 / H, (x1c - x0c) / W, (y1c - y0c) / H)


def scene_things(sim):
    """지금 장면의 (물체, 클래스) 목록 + 렌더할 물체 전부"""
    labeled = [(p, 1 if p.get("lying") else 0) for p in sim.pucks] + [(sim.goal, 2)]
    labeled += [(z, 3) for z in sim.stations]
    labeled += [(p, {"qr_board": 4, "obstacle": 5, "mover": 6, "traffic_light": 7}[p["kind"]]) for p in sim.props
                if p["kind"] in ("qr_board", "obstacle", "mover", "traffic_light")]
    things = [dict(p) for p in sim.pucks + sim.props] + [dict(sim.goal)] + [dict(z) for z in sim.stations]
    return labeled, things


def build_scene(sim, rng, bgs, floors, walls):
    """미션 장면 하나: 미션 지역 1~3곳(문제 무작위) · B 지역 위치 · 환경(장애물·사람) · B 근처 퍽 · 텍스처"""
    for z in list(sim.stations):
        sim.remove_station(z["name"])
    sim.clear_env()
    sim.pucks = []
    cells = list(sim.cells)
    rng.shuffle(cells)
    sim.move_goal(*cells[0])
    for c in cells[1:1 + rng.randint(1, 3)]:
        sim.add_station(*c, rng.choice(sorted(scene.PROBLEMS)))
    sim.setup_env(rng.randrange(1 << 30))
    sim.slip_zones = []
    for _ in range(rng.choice((0, 1, 1, 2))):  # 신호등 — 상태 무작위 (켜진 램프 색은 perception이 판별)
        sim.spawn_traffic()
    for p in sim.props:
        if p["kind"] == "traffic_light":
            p["state"] = rng.choice(["red", "yellow", "green", "off"])
            p["color"] = scene.TL_COLORS[p["state"]]
    for _ in range(rng.randint(0, 3)):  # B 지역에 이미 옮겨 놓은 퍽
        g = sim.goal
        sim.pucks.append({"id": 900 + len(sim.pucks), "shape": "cylinder", "color": rng.choice(scene.PUCK_COLORS),
                          "x": g["x"] + rng.uniform(-0.1, 0.1), "y": g["y"] + rng.uniform(-0.1, 0.1),
                          "yaw": rng.uniform(-math.pi, math.pi), "size": [0.025, 0.035]})
    sim.renderer.set_background(rng.choice(bgs + [None]))
    sim.renderer.set_floor(rng.choice(floors + [None]))
    sim.renderer.set_wall(rng.choice(walls + [None]))


def camera_views(sim, kin, labeled, rng, views):
    """시점 목록 [(T_world_cam, 관절값)] — pole_cam 먼 · 가까운, gripper_cam 집기 직전"""
    by_cls = {}
    for p, c in labeled:
        by_cls.setdefault(c, []).append(p)
    out = []
    for _ in range(views * 12):  # 벽에 막힌 자리는 건너뛰므로 넉넉히
        if len(out) >= views:
            break
        t = rng.choice(by_cls[rng.choice(sorted(by_cls))])  # 클래스를 먼저 고름 — 드문 클래스(신호등·누운 퍽)도 고르게
        kind = rng.choices(("far", "near", "grip"), weights=(4, 4, 2))[0]
        d = {"far": (1.0, 3.0), "near": (0.4, 1.0), "grip": (0.22, 0.32)}[kind]
        ang = rng.uniform(-math.pi, math.pi)
        dist = rng.uniform(*d)
        x, y = t["x"] + dist * math.cos(ang), t["y"] + dist * math.sin(ang)
        if sim.wall_hit(x, y, 0.18):
            continue
        yaw = math.atan2(t["y"] - y, t["x"] - x) + rng.uniform(-0.3, 0.3)
        T_wb = scene.transform(scene.rpy_matrix(0, 0, yaw), (x, y, 0))
        q = {}
        cam = "pole_cam"
        if kind == "grip":  # 팔을 앞으로 뻗어 그리퍼 카메라가 바닥 앞을 내려다보는 자세 (무작위)
            cam = "gripper_cam"
            q = {"joint1": rng.uniform(-0.3, 0.3), "joint2": rng.uniform(0.3, 1.0),
                 "joint3": rng.uniform(-0.5, 0.3), "joint4": rng.uniform(0.6, 1.5), "joint5": rng.uniform(-1.0, 1.0),
                 "gripper_left_joint": rng.uniform(0.0, 0.0265)}
        out.append((T_wb, T_wb @ kin.pose(f"{cam}_color_optical_frame", q), q, cam))
    return out


def main():
    import main as sim_main  # 시뮬레이터의 미션 배치 (창 없이)
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="dataset")
    ap.add_argument("--n", type=int, default=1000, help="장면 수 (=이미지 최대 n×views)")
    ap.add_argument("--views", type=int, default=3, help="장면당 시점 수")
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    desc, _ = read_msg("robot_description")
    if desc:
        urdf_xml = desc["data"]
    else:  # 테스트베드가 안 떠 있음 — xacro 직접 전개
        import xacro as xacro_lib
        urdf_xml = xacro_lib.process_file(
            str(HERE.parent / "urdf" / "beebot_mesh" / "urdf.xacro"),
            mappings={"mock_hardware": "true", "mesh_uri": "package://ros_system/urdf/beebot_mesh/meshes"}).toxml()
    kin = scene.Kinematics(urdf_xml)
    sim_main.send = lambda *a, **k: None  # 장면만 쓴다 — ROS로 아무것도 보내지 않음
    sim_main.send_service_request = lambda *a, **k: None
    sim = sim_main.Sim(maze.load())
    import robot_view
    body = scene.BodyModel(urdf_xml, resolve=robot_view.resolve_mesh, load_mesh=robot_view.camera_mesh,  # 팔·차체 메시 그대로
                           max_tris=192 if sim.renderer.dev.type == 'cpu' else 10 ** 6)
    renderer = sim.renderer
    fx = (scene.RENDER_W / 2) / math.tan(math.radians(scene.HFOV_DEG) / 2)
    cx, cy = scene.RENDER_W / 2, scene.RENDER_H / 2
    rng = random.Random(args.seed)
    random.seed(args.seed)
    img_dir = HERE / "images"
    bgs = [p for p in scene.backgrounds(img_dir / "background")]
    floors = [p for p in scene.backgrounds(img_dir / "floor") if p.suffix == ".jpg"]
    walls = [p for p in scene.backgrounds(img_dir / "wall") if p.suffix == ".jpg"]
    print(f"렌더 장치 {renderer.dev} · 문제 {sorted(scene.PROBLEMS)} · 배경 {len(bgs)} 바닥 {len(floors)} 벽 {len(walls)}")

    out = Path(args.out)
    imgdir, labdir = out / "images", out / "labels"
    for d in (imgdir, labdir):
        d.mkdir(parents=True, exist_ok=True)

    n_img = 0
    counts = [0] * len(CLASSES)
    for i in range(args.n):
        build_scene(sim, rng, bgs, floors, walls)
        labeled, things = scene_things(sim)
        boards = [p for p in things if p.get("kind") == "qr_board"]
        for v, (T_wb, T_wc, q, cam) in enumerate(camera_views(sim, kin, labeled, rng, args.views)):
            own = f"{cam}_link"
            boxes = [(T_wb @ kin.pose(link, q) @ To, sx, sy, sz, rgb)
                     for link, To, (sx, sy, sz), rgb in body.boxes if link != own]
            meshes = [(T_wb @ kin.pose(link, q) @ To, tris, rgb) for link, To, tris, rgb in body.meshes if link != own]
            bgr, depth = renderer.render(T_wc, things, boxes, meshes)
            for b in boards:
                bgr = scene.overlay_qr(bgr, depth, T_wc, b)
            gain, noise = rng.uniform(0.55, 1.45), rng.choice((0, 0, 4, 8, 15))
            f = bgr.astype(np.float32) * gain
            if noise:
                f += np.random.normal(0, noise, f.shape).astype(np.float32)
            bgr = np.clip(f, 0, 255).astype(np.uint8)
            T_cw = np.linalg.inv(T_wc)
            lines = []
            for p, cls in labeled:
                b = project_box(p, T_cw, fx, cx, cy, scene.RENDER_W, scene.RENDER_H, depth)
                if b:
                    lines.append(f"{cls} {b[0]:.6f} {b[1]:.6f} {b[2]:.6f} {b[3]:.6f}")
                    counts[cls] += 1
            if not lines:  # 빈 장면은 버림 (배경 편향 방지)
                continue
            name = f"{i:05d}_{v}"
            cv2.imwrite(str(imgdir / f"{name}.png"),
                        cv2.resize(bgr, (IMG_W, IMG_H), interpolation=cv2.INTER_LINEAR))
            (labdir / f"{name}.txt").write_text("\n".join(lines) + "\n")
            n_img += 1
        if (i + 1) % 50 == 0:
            print(f"{i + 1}/{args.n} 장면 · {n_img}장 · " + " ".join(f"{c}={n}" for c, n in zip(CLASSES, counts)), flush=True)
    # ultralytics용 분할 + data.yaml
    names = sorted(p.stem for p in imgdir.glob("*.png"))
    if not names:
        print("경고: 라벨 있는 장면 0장 — --n/--views를 늘리시오")
        (out / "data.yaml").write_text(
            f"path: {out.resolve()}\ntrain: train/images\nval: val/images\n"
            f"nc: {len(CLASSES)}\nnames: {CLASSES}\n")
        return
    rng.shuffle(names)
    n_val = max(1, len(names) // 10)
    val = set(names[:n_val])
    for split, sel in (("train", [n for n in names if n not in val]),
                       ("val", [n for n in names if n in val])):
        (out / split / "images").mkdir(parents=True, exist_ok=True)
        (out / split / "labels").mkdir(parents=True, exist_ok=True)
        for n in sel:
            shutil.move(str(imgdir / f"{n}.png"), str(out / split / "images" / f"{n}.png"))
            shutil.move(str(labdir / f"{n}.txt"), str(out / split / "labels" / f"{n}.txt"))
    imgdir.rmdir()
    labdir.rmdir()
    (out / "data.yaml").write_text(
        f"path: {out.resolve()}\ntrain: train/images\nval: val/images\n"
        f"nc: {len(CLASSES)}\nnames: {CLASSES}\n")
    print(f"OK: {n_img}장 → {out}/train {len(names) - n_val}장 · val {n_val}장")


if __name__ == "__main__":
    main()
