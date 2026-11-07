#!/usr/bin/env python3
"""Gazebo 월드 생성 — 분산 미션 + AprilTag 퍽 + 통행로 위 신호등.

기본 구성(퍽 총 9개, 네 미션에 3/2/2/2로 분배):
  · 상단 방 SORT_COLOR, 중간 위 MOVE_TO_B, 중간 아래 STAND_UP, 하단 LAY_DOWN
  · 각 미션은 자기 구역 근처 벽에 미션 태그(10~13)를 하나씩 가진다 → 지도에 분산
  · 퍽은 30×30×60 mm 상자, 4옆면 + 위/아래 = 6면에 AprilTag
  · 한 색의 퍽은 6면 모두 같은 tag ID (red=1, orange=2, green=3)
  · SORT_COLOR는 색별 정리 구역 3개, MOVE_TO_B는 B 구역 + B 태그(20)
  · STAND_UP 퍽은 눕고, LAY_DOWN 퍽은 서서 시작한다.
  · 상단 방 통행로를 가로지르는 갠트리에 매단 신호등 2개(빨강/주황/초록 렌즈)

무작위 장애물(기본 6 박스 + 6 실린더)은 네 미션 앞쪽/미션 태그/신호등 갠트리를 비워 두고
벽·로봇 스폰·다른 물체와 겹치지 않게 배치한다. 동적 장애물은 static 월드에
넣지 않고 dynamic_obstacles.py가 **로봇 이동 중에만** 생성/제거한다.

결과:
  --world-out : gz sim에 넣을 SDF (원본 월드에 모델 삽입)
  --state-out : 실제 스폰 pose/크기/태그/미션 JSON (publish_poses.py가 읽음)

환경변수:
  GZ_MISSION=random|SORT_COLOR|MOVE_TO_B|STAND_UP|LAY_DOWN
  GZ_PUCK_COUNT=9                # 네 미션에 나눠 배치하는 총 퍽 수
  GZ_PUCK_MM="30 30 60"         # 퍽 크기 (mm)
  GZ_TAG_SIZE_MM=28              # 퍽 태그 판 크기 (mm)
  GZ_MISSION_TAG_MM=120          # 벽 미션 태그 판 크기 (mm)
  GZ_MISSION_SITES=all|single    # all=분산(기본), single=활성 미션 한 곳
  GZ_MISSION_CENTER="0.35 2.75" # single일 때 퍽 군집 중심 (world, m)
  GZ_MISSION_WALL=north|south|east|west  # 미션 태그를 붙일 벽 방향 (기본 north)
  GZ_MISSION_SEED / GZ_OBSTACLE_SEED
  GZ_OBSTACLE_BOXES / GZ_OBSTACLE_CYLINDERS
  GZ_OBSTACLE_BOX_MM / GZ_OBSTACLE_CYLINDER_MM
  GZ_OBSTACLE_STATIC=1
  GZ_FLOOR_TEXTURE=floor_wood    # assets/environment/floor 선택 (기본 none)
  GZ_WALL_TEXTURE=wall_painted   # assets/environment/wall 선택 (기본 none)
  GZ_BACKGROUND=pano_wooden_lounge # assets/environment/background 선택 (기본 none)
  GZ_ENV_OUT=/tmp/beebot_environment
  GZ_TRAFFIC_INTERVAL=3.0        # 신호등 주기 (런타임)
  GZ_DYNAMIC_COUNT=3             # 동시 동적 장애물 수 (0=끔)
  GZ_DYNAMIC_LIFETIME=12 GZ_DYNAMIC_RESPAWN=1 GZ_DYNAMIC_SPEED=0.16
  GZ_DYNAMIC_MM="220 220 600"
"""
import argparse
import json
import math
import os
import random
import re
import sys
from pathlib import Path

import generate_tags

HERE = Path(__file__).resolve().parent
COURSE = HERE.parent.parent / "course"
DEFAULT_WORLD = COURSE / "worlds" / "twin_closed_maze.sdf"
DEFAULT_WALLS = COURSE / "twin_closed_maze" / "walls.json"
ASSET_DIR = HERE / "assets" / "apriltags"
ENV_ASSET_DIR = HERE / "assets" / "environment"

COLOR_ORDER = ("red", "orange", "green")
COLOR_RGB = {name: tuple(info["rgb"]) for name, info in generate_tags.COLORS.items()}
COLOR_TAG = {name: int(info["tag_id"]) for name, info in generate_tags.COLORS.items()}
MISSION_NAMES = tuple(generate_tags.MISSIONS)
MISSION_TAG = {name: int(info["tag_id"]) for name, info in generate_tags.MISSIONS.items()}
B_TAG_ID = int(generate_tags.ZONE_TAGS["B_ZONE"]["tag_id"])
WALL_HEIGHT = 0.39

# 도로형 신호등: 하우징 로컬 X축으로 3개 렌즈를 나란히 배치한다.
TRAFFIC_COLORS = {
    "red": (0.95, 0.04, 0.04),
    "orange": (1.00, 0.45, 0.00),
    "green": (0.04, 0.85, 0.16),
}
TRAFFIC_ORDER = ("red", "green", "orange")
TRAFFIC_INTERVAL = 3.0
TRAFFIC_LENS_X = (-0.11, 0.0, 0.11)
TRAFFIC_LENS_Z = 0.034
TRAFFIC_LIT_Z = 0.050
TRAFFIC_Z = 0.65               # 로봇 통행로 위, 자동차 신호등처럼 높게
TRAFFIC_GANTRY_WIDTH = 0.72    # 통행로를 가로지르는 상부 빔 폭
TRAFFIC_GANTRY_TOP = 0.24      # 하우징 중심에서 상부 빔까지
TRAFFIC_POST_BOTTOM = -0.47    # 하우징 중심에서 기둥 아래 끝까지


def env_int(name, default):
    value = os.environ.get(name)
    return default if value in (None, "") else int(value)


def env_floats(name, default):
    value = os.environ.get(name)
    if value in (None, ""):
        return list(default)
    parts = value.replace(",", " ").split()
    if len(parts) != len(default):
        raise ValueError(f"{name}: {len(default)}개 값이 필요합니다 (예: {' '.join(map(str, default))})")
    return [float(p) for p in parts]


def env_pair(name, default):
    value = os.environ.get(name)
    if value in (None, ""):
        return tuple(default)
    parts = value.replace(",", " ").split()
    if len(parts) != 2:
        raise ValueError(f"{name}: x y 두 값이 필요합니다")
    return float(parts[0]), float(parts[1])


def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def rpy_for_normal(normal, up=None):
    """로컬 XY 평면(법선 +Z)을 world의 normal 방향으로, 텍스처는 세워서.

    로컬 +Y(태그 위쪽)가 world +Z를 향하도록 정렬한다. 법선이 위/아래면
    +Y를 world +Y로 둔다. 이렇게 해야 6면 모두 태그 앞면이 바깥을 향한다.
    """
    nx, ny, nz = (float(v) for v in normal)
    nlen = math.sqrt(nx * nx + ny * ny + nz * nz)
    if nlen < 1e-12:
        raise ValueError("normal must be non-zero")
    nx, ny, nz = nx / nlen, ny / nlen, nz / nlen

    if up is None:
        up = (0.0, 1.0, 0.0) if abs(nz) > 0.5 else (0.0, 0.0, 1.0)
    ux, uy, uz = (float(v) for v in up)
    dot = ux * nx + uy * ny + uz * nz
    ux, uy, uz = ux - dot * nx, uy - dot * ny, uz - dot * nz
    ulen = math.sqrt(ux * ux + uy * uy + uz * uz)
    if ulen < 1e-12:
        # normal과 up이 평행한 경우: world X를 기준으로 다시 만든다.
        ux, uy, uz = 1.0, 0.0, 0.0
        dot = ux * nx + uy * ny + uz * nz
        ux, uy, uz = ux - dot * nx, uy - dot * ny, uz - dot * nz
        ulen = math.sqrt(ux * ux + uy * uy + uz * uz)
        if ulen < 1e-12:
            raise ValueError("cannot construct tag basis")
    ux, uy, uz = ux / ulen, uy / ulen, uz / ulen

    # local X = local Y × local Z (오른손 좌표계)
    xx, xy, xz = uy * nz - uz * ny, uz * nx - ux * nz, ux * ny - uy * nx
    # R = Rz(yaw) Ry(pitch) Rx(roll); columns are local axes in world.
    pitch = math.asin(max(-1.0, min(1.0, -xz)))
    roll = math.atan2(uz, nz)
    yaw = math.atan2(xy, xx)
    return roll, pitch, yaw


def pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    return {"x": round(float(x), 6), "y": round(float(y), 6), "z": round(float(z), 6),
            "roll": round(float(roll), 6), "pitch": round(float(pitch), 6),
            "yaw": round(float(yaw), 6)}


def pose_text(p):
    return (f"{p['x']:.9g} {p['y']:.9g} {p['z']:.9g} "
            f"{p['roll']:.9g} {p['pitch']:.9g} {p['yaw']:.9g}")


def box_inertia(mass, sx, sy, sz):
    return (mass * (sy * sy + sz * sz) / 12.0,
            mass * (sx * sx + sz * sz) / 12.0,
            mass * (sx * sx + sy * sy) / 12.0)


def cylinder_inertia(mass, radius, height):
    side = mass * (3.0 * radius * radius + height * height) / 12.0
    return side, side, mass * radius * radius / 2.0


def object_radius(obj):
    sx, sy, _ = obj["size"] if obj["shape"] == "box" else (obj["size"][0], obj["size"][0], 0.0)
    if obj["shape"] == "box":
        return math.hypot(sx, sy) / 2.0
    return sx / 2.0


def wall_bounds(walls):
    return (min(b["x"] - b["sx"] / 2.0 for b in walls),
            max(b["x"] + b["sx"] / 2.0 for b in walls),
            min(b["y"] - b["sy"] / 2.0 for b in walls),
            max(b["y"] + b["sy"] / 2.0 for b in walls))


def wall_hit(walls, x, y, radius):
    for b in walls:
        if (abs(x - b["x"]) < b["sx"] / 2.0 + radius + 0.008 and
                abs(y - b["y"]) < b["sy"] / 2.0 + radius + 0.008):
            return True
    return False


def clear_hit(clear_regions, reserved, x, y, radius):
    for clear in clear_regions:
        if clear[0] - radius < x < clear[2] + radius and clear[1] - radius < y < clear[3] + radius:
            return True
    for p in reserved:
        if math.hypot(x - p["x"], y - p["y"]) < radius + p["r"]:
            return True
    return False


def make_obstacle(name, shape, size, x, y, yaw, rng, static):
    """size: box=[sx,sy,sz] · cylinder=[지름,높이] (m)."""
    if shape == "box":
        sx, sy, sz = size
        z = sz / 2.0 + 0.001
        # 퍽 색(빨/주/초)과 헷갈리지 않는 중성 색만 쓴다.
        color = rng.choice(((0.42, 0.45, 0.48), (0.34, 0.40, 0.47),
                            (0.52, 0.42, 0.32), (0.38, 0.34, 0.46)))
    else:
        diameter, height = size
        z = height / 2.0 + 0.001
        color = rng.choice(((0.24, 0.50, 0.62), (0.45, 0.34, 0.56),
                            (0.52, 0.52, 0.28), (0.32, 0.36, 0.44)))
    return {
        "name": name,
        "kind": "obstacle",
        "shape": shape,
        "size": [round(float(v), 6) for v in size],
        "pose": pose(x, y, z, 0.0, 0.0, yaw),
        "color": [round(float(c), 4) for c in color],
        "static": bool(static),
    }


def make_puck(name, color_name, size, x, y, lying, yaw):
    sx, sy, sz = size
    z = (sy / 2.0 if lying else sz / 2.0) + 0.001
    roll = math.pi / 2.0 if lying else 0.0
    return {
        "name": name,
        "kind": "puck",
        "shape": "box",
        "size": [round(float(v), 6) for v in size],
        "pose": pose(x, y, z, roll, 0.0, yaw),
        "color": [round(float(c), 4) for c in COLOR_RGB[color_name]],
        "color_name": color_name,
        "tag_id": COLOR_TAG[color_name],
        "lying": bool(lying),
        "static": False,
    }


def make_zone(name, label, x, y, width, color, yaw=0.0):
    return {
        "name": name,
        "kind": "zone",
        "shape": "plane",
        "size": [round(width, 6), round(width, 6), 0.001],
        "pose": pose(x, y, 0.002, 0.0, 0.0, yaw),
        "color": [round(float(c), 4) for c in color],
        "label": label,
        "static": True,
    }


def make_tag_model(name, tag_id, x, y, z, normal, board_size, label, mission=None):
    roll, pitch, yaw = rpy_for_normal(normal)
    return {
        "name": name,
        "kind": "mission_tag" if mission else "zone_tag",
        "shape": "plane",
        "size": [round(board_size, 6), round(board_size, 6), 0.001],
        "pose": pose(x, y, z, roll, pitch, yaw),
        "color": [0.95, 0.95, 0.95],
        "tag_id": int(tag_id),
        "label": label,
        "mission": mission,
        "normal": list(normal),
        "static": True,
    }


def make_traffic_light(name, x, y, z, normal, label, index):
    """통행로 위를 가로지르는 갠트리에 매단 가로 3색 신호등.

    모델 원점은 하우징 중심이고, 로컬 X=도로 폭, 로컬 Y=위, 로컬 Z=렌즈가
    바라보는 진행 방향이다. 렌즈가 도로를 따라 로봇 쪽을 향한다.
    """
    roll, pitch, yaw = rpy_for_normal(normal)
    p = pose(x, y, z, roll, pitch, yaw)
    lenses = []
    for i, color in enumerate(("red", "orange", "green")):
        lx = TRAFFIC_LENS_X[i]
        # 로컬 (lx, 0, lit_z)를 world 오프셋으로 변환해 두면 런타임 노드가 바로 쓴다.
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        lx2, ly2, lz2 = lx, 0.0, TRAFFIC_LIT_Z
        x1 = lx2
        y1 = cr * ly2 - sr * lz2
        z1 = sr * ly2 + cr * lz2
        x2 = cp * x1 + sp * z1
        y2 = y1
        z2 = -sp * x1 + cp * z1
        wx = cy * x2 - sy * y2
        wy = sy * x2 + cy * y2
        wz = z2
        lenses.append({
            "color": color,
            "local": [round(lx, 6), 0.0, TRAFFIC_LENS_Z],
            "world_offset": [round(wx, 6), round(wy, 6), round(wz, 6)],
            "tag_visual": f"lens_{color}",
        })
    return {
        "name": name,
        "kind": "traffic_light",
        "shape": "box",
        "size": [0.38, 0.16, 0.06],
        "pose": p,
        "color": [0.09, 0.10, 0.12],
        "label": label,
        "index": int(index),
        "normal": list(normal),
        "lenses": lenses,
        "gantry_width": TRAFFIC_GANTRY_WIDTH,
        "gantry_top": TRAFFIC_GANTRY_TOP,
        "post_bottom": TRAFFIC_POST_BOTTOM,
        "static": True,
    }

def build_traffic_lights(args, walls):
    """상단 방의 동서 통행로를 따라, 위에서 내려다보이는 신호등 2개."""
    # 로봇이 서쪽 스폰에서 동쪽으로 지나가는 길 위에 차례로 세운다.
    specs = [
        ("traffic_light_1", -0.75, 2.75, "신호등 1"),
        ("traffic_light_2", 1.15, 2.75, "신호등 2"),
    ]
    lights = []
    for index, (name, x, y, label) in enumerate(specs):
        # 렌즈는 서쪽(스폰 방향)을 향해 로봇이 진행하며 정면으로 본다.
        lights.append(make_traffic_light(name, x, y, TRAFFIC_Z, (-1.0, 0.0, 0.0),
                                         label, index))
    return lights

def sdf_for_traffic_light(obj):
    p = obj["pose"]
    hw, hh, hd = obj["size"]
    r, g, b = obj["color"]
    gantry_w = float(obj.get("gantry_width", TRAFFIC_GANTRY_WIDTH))
    gantry_top = float(obj.get("gantry_top", TRAFFIC_GANTRY_TOP))
    post_bottom = float(obj.get("post_bottom", TRAFFIC_POST_BOTTOM))
    post_height = gantry_top - post_bottom
    post_mid = (gantry_top + post_bottom) / 2.0
    lens_visuals = []
    for lens in obj["lenses"]:
        lx = lens["local"][0]
        lz = lens["local"][2]
        cr, cg, cb = TRAFFIC_COLORS[lens["color"]]
        # 꺼진 렌즈는 어두운 색으로 두고, 런타임 노드가 앞에 밝은 구를 붙인다.
        dr, dg, db = cr * 0.18 + 0.015, cg * 0.18 + 0.015, cb * 0.18 + 0.015
        lens_visuals.append(f"""
      <visual name="{lens['tag_visual']}">
        <pose>{lx:.9g} 0 {lz:.9g} 0 0 0</pose>
        <geometry><cylinder><radius>0.034</radius><length>0.008</length></cylinder></geometry>
        <material><ambient>{dr:.4g} {dg:.4g} {db:.4g} 1</ambient>
          <diffuse>{dr:.4g} {dg:.4g} {db:.4g} 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular></material>
      </visual>""")
    return f"""
  <model name="{obj['name']}">
    <pose>{pose_text(p)}</pose>
    <static>true</static>
    <link name="link">
      <visual name="housing">
        <geometry><box><size>{hw:.9g} {hh:.9g} {hd:.9g}</size></box></geometry>
        <material><ambient>{r:.4g} {g:.4g} {b:.4g} 1</ambient>
          <diffuse>{r:.4g} {g:.4g} {b:.4g} 1</diffuse>
          <specular>0.10 0.10 0.10 1</specular></material>
      </visual>
      <visual name="bezel">
        <pose>0 0 0.031 0 0 0</pose>
        <geometry><box><size>{hw * 0.94:.9g} {hh * 0.76:.9g} 0.006</size></box></geometry>
        <material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>
      </visual>
      <visual name="gantry_beam">
        <pose>0 {gantry_top:.9g} 0 0 0 0</pose>
        <geometry><box><size>{gantry_w:.9g} 0.045 0.045</size></box></geometry>
        <material><ambient>0.08 0.08 0.09 1</ambient><diffuse>0.08 0.08 0.09 1</diffuse>
          <specular>0.10 0.10 0.10 1</specular></material>
      </visual>
      <visual name="gantry_post_left">
        <pose>{-gantry_w / 2.0:.9g} {post_mid:.9g} 0 0 0 0</pose>
        <geometry><box><size>0.035 {post_height:.9g} 0.035</size></box></geometry>
        <material><ambient>0.08 0.08 0.09 1</ambient><diffuse>0.08 0.08 0.09 1</diffuse></material>
      </visual>
      <visual name="gantry_post_right">
        <pose>{gantry_w / 2.0:.9g} {post_mid:.9g} 0 0 0 0</pose>
        <geometry><box><size>0.035 {post_height:.9g} 0.035</size></box></geometry>
        <material><ambient>0.08 0.08 0.09 1</ambient><diffuse>0.08 0.08 0.09 1</diffuse></material>
      </visual>
      <visual name="gantry_hanger">
        <pose>0 {gantry_top - 0.10:.9g} -0.02 0 0 0</pose>
        <geometry><box><size>0.035 0.20 0.035</size></box></geometry>
        <material><ambient>0.08 0.08 0.09 1</ambient><diffuse>0.08 0.08 0.09 1</diffuse></material>
      </visual>{''.join(lens_visuals)}
    </link>
  </model>"""


def build_dynamic_config(args, rng, walls, spawn, reserved, clear, objects):
    """동적 장애물이 나타날 수 있는 빈 지점을 미리 샘플링한다."""
    if args.dynamic_count <= 0:
        return {"enabled": False, "max_active": 0, "spawn_points": []}
    x_min, x_max, y_min, y_max = wall_bounds(walls)
    sx, sy, sz = args.dynamic_size
    radius = math.hypot(sx, sy) / 2.0
    points = []
    for _ in range(30000):
        if len(points) >= max(32, args.dynamic_count * 12):
            break
        x = rng.uniform(x_min + radius + 0.03, x_max - radius - 0.03)
        y = rng.uniform(y_min + radius + 0.03, y_max - radius - 0.03)
        if math.hypot(x - spawn[0], y - spawn[1]) < 0.45:
            continue
        if wall_hit(walls, x, y, radius + 0.02):
            continue
        if clear_hit(clear, reserved, x, y, radius + 0.02):
            continue
        if any(math.hypot(x - o["pose"]["x"], y - o["pose"]["y"]) <
               radius + object_radius(o) + 0.10 for o in objects
               if o.get("kind") in ("obstacle", "traffic_light")):
            continue
        if any(math.hypot(x - p["x"], y - p["y"]) < 0.22 for p in points):
            continue
        points.append({"x": round(x, 6), "y": round(y, 6)})
    return {
        "enabled": bool(points),
        "max_active": int(max(0, args.dynamic_count)),
        "lifetime_sec": float(args.dynamic_lifetime),
        "respawn_sec": float(args.dynamic_respawn),
        "speed_mps": float(args.dynamic_speed),
        "size": [round(float(v), 6) for v in args.dynamic_size],
        "spawn_points": points,
        "colors": [[0.82, 0.28, 0.14], [0.20, 0.56, 0.82], [0.62, 0.30, 0.70],
                   [0.86, 0.62, 0.12]],
    }




def wall_span(wall):
    if wall["sx"] >= wall["sy"]:
        return wall["x"] - wall["sx"] / 2.0, wall["x"] + wall["sx"] / 2.0
    return wall["y"] - wall["sy"] / 2.0, wall["y"] + wall["sy"] / 2.0


def choose_wall(walls, x, y, side):
    """원하는 방향의 긴 벽(패널)을 고른다. 없으면 None."""
    candidates = []
    for wall in walls:
        if max(wall["sx"], wall["sy"]) < 0.20:
            continue
        horizontal = wall["sx"] >= wall["sy"]
        if side in ("north", "south"):
            if not horizontal:
                continue
            inside = wall["y"] > y if side == "north" else wall["y"] < y
        else:
            if horizontal:
                continue
            inside = wall["x"] > x if side == "east" else wall["x"] < x
        if not inside:
            continue
        lo, hi = wall_span(wall)
        center_coord = x if side in ("north", "south") else y
        clamped = min(max(center_coord, lo + 0.05), hi - 0.05)
        distance = (abs(wall["y"] - y) if horizontal else abs(wall["x"] - x))
        candidates.append((distance + abs(clamped - center_coord), wall))
    return min(candidates, key=lambda item: item[0])[1] if candidates else None


def wall_tag_pose(walls, x, y, side, board_size):
    """벽 안쪽 면에 붙는 태그 중심 pose + normal."""
    wall = choose_wall(walls, x, y, side)
    if wall is None:
        # 마지막 fallback: 북쪽 벽이 없으면 월드 밖을 향하지 않게 north face로 둔다.
        return pose(x, y, 0.20), (0, -1, 0), None
    horizontal = wall["sx"] >= wall["sy"]
    gap = 0.002
    half = board_size / 2.0
    if horizontal:
        lo, hi = wall["x"] - wall["sx"] / 2.0, wall["x"] + wall["sx"] / 2.0
        tx = min(max(x, lo + half + 0.005), hi - half - 0.005)
        if side == "north" or (side not in ("south",) and y < wall["y"]):
            ty = wall["y"] - wall["sy"] / 2.0 - gap
            normal = (0, -1, 0)
        else:
            ty = wall["y"] + wall["sy"] / 2.0 + gap
            normal = (0, 1, 0)
    else:
        lo, hi = wall["y"] - wall["sy"] / 2.0, wall["y"] + wall["sy"] / 2.0
        ty = min(max(y, lo + half + 0.005), hi - half - 0.005)
        if side == "east" or (side not in ("west",) and x < wall["x"]):
            tx = wall["x"] - wall["sx"] / 2.0 - gap
            normal = (-1, 0, 0)
        else:
            tx = wall["x"] + wall["sx"] / 2.0 + gap
            normal = (1, 0, 0)
    return pose(tx, ty, 0.20), normal, wall.get("name")


STATION_ORDER = ("SORT_COLOR", "MOVE_TO_B", "STAND_UP", "LAY_DOWN")
STATION_SPECS = {
    # 미로의 위·중간·아래 방에 미션을 분산한다. 좌표는 world 기준.
    "SORT_COLOR": {"suffix": "sort", "center": (0.35, 2.75), "wall": "north"},
    "MOVE_TO_B": {"suffix": "move", "center": (-1.45, 1.97), "wall": "north"},
    "STAND_UP": {"suffix": "stand", "center": (1.45, 1.18), "wall": "south"},
    "LAY_DOWN": {"suffix": "lay", "center": (0.0, -2.75), "wall": "north"},
}


def station_counts(total):
    """퍽 총 개수를 네 미션에 최대한 균등하게 나눈다 (기본 9 → 3/2/2/2)."""
    total = max(0, int(total))
    base, rem = divmod(total, len(STATION_ORDER))
    counts = [base] * len(STATION_ORDER)
    for i in range(rem):
        counts[i] += 1
    return counts


def build_pucks(args, rng, mission, center, count, color_plan, prefix):
    count = max(0, int(count))
    sx, sy, sz = args.puck_size
    spacing = max(0.075, max(sx, sy, sz) * 1.35)
    cols = max(1, math.ceil(math.sqrt(count))) if count else 1
    colors = list(color_plan)
    while len(colors) < count:
        colors.append(COLOR_ORDER[len(colors) % len(COLOR_ORDER)])

    if mission == "STAND_UP":
        lying_indices = set(rng.sample(range(count), min(5, count)))
    elif mission == "LAY_DOWN":
        lying_indices = set()
    else:
        lying_indices = set()

    pucks = []
    color_counts = {c: 0 for c in COLOR_ORDER}
    for i in range(count):
        row, col = divmod(i, cols)
        x = center[0] + (col - (cols - 1) / 2.0) * spacing
        y = center[1] + (row - (math.ceil(count / cols) - 1) / 2.0) * spacing
        color = colors[i]
        name = f"puck_{prefix}_{color}_{color_counts[color]}"
        color_counts[color] += 1
        lying = i in lying_indices
        pucks.append(make_puck(name, color, args.puck_size, x, y, lying,
                               rng.uniform(-math.pi, math.pi)))
    return pucks


def build_station(args, rng, walls, mission, center, count, color_plan, index, active):
    info = generate_tags.MISSIONS[mission]
    cx, cy = center
    pucks = build_pucks(args, rng, mission, center, count, color_plan,
                        STATION_SPECS[mission]["suffix"])
    objects = list(pucks)
    zones = []
    zone_tag = None
    zone_specs = []

    if mission == "SORT_COLOR":
        colors = ((COLOR_ORDER[0], "red"), (COLOR_ORDER[1], "orange"), (COLOR_ORDER[2], "green"))
        for i, (color_name, _label) in enumerate(colors):
            zx = cx + 0.78 + 0.30 * i
            zone = make_zone(f"zone_{color_name}", f"{color_name.upper()} 정리",
                             zx, cy, 0.18, COLOR_RGB[color_name])
            zones.append(zone)
            objects.append(zone)
            zone_specs.append({"name": zone["name"], "label": zone["label"],
                               "x": zone["pose"]["x"], "y": zone["pose"]["y"],
                               "size": 0.18, "color": color_name})
    elif mission == "MOVE_TO_B":
        zx, zy = cx + 0.90, cy
        zone = make_zone("zone_B", "B 구역", zx, zy, 0.36, (0.10, 0.42, 0.92))
        zones.append(zone)
        objects.append(zone)
        zone_specs.append({"name": zone["name"], "label": "B", "x": zx, "y": zy,
                           "size": 0.36, "color": "blue"})
        tp, normal, wall_name = wall_tag_pose(walls, zx, zy, args.mission_wall,
                                              args.mission_tag_size * 0.72)
        zone_tag = make_tag_model("zone_tag_B", B_TAG_ID, tp["x"], tp["y"], tp["z"],
                                  normal, args.mission_tag_size * 0.72, "B 구역")
        objects.append(zone_tag)

    # 각 미션은 자기 구역 근처의 벽에 태그를 하나씩 가진다. 네 태그가
    # 지도 위·중간·아래에 분산되므로, 한 곳에 모인 보드가 아니다.
    wall_side = STATION_SPECS[mission].get("wall", args.mission_wall)
    tp, normal, tag_wall = wall_tag_pose(walls, cx, cy, wall_side,
                                         args.mission_tag_size)
    mission_tag = make_tag_model(f"mission_tag_{mission}", MISSION_TAG[mission],
                                 tp["x"], tp["y"], tp["z"], normal,
                                 args.mission_tag_size, info["label"], mission=mission)
    mission_tag["active"] = bool(active)
    objects.append(mission_tag)

    reserved = [{"x": cx, "y": cy, "r": 0.75}]  # 미션 앞쪽은 동적/정적 장애물 모두 비움
    for p in pucks:
        reserved.append({"x": p["pose"]["x"], "y": p["pose"]["y"], "r": 0.050})
    for zone in zones:
        reserved.append({"x": zone["pose"]["x"], "y": zone["pose"]["y"],
                         "r": max(zone["size"][0], zone["size"][1]) / 2.0 + 0.04})
    for tag in [mission_tag] + ([zone_tag] if zone_tag is not None else []):
        reserved.append({"x": tag["pose"]["x"], "y": tag["pose"]["y"], "r": 0.10})

    xs = [p["pose"]["x"] for p in pucks] + [z["pose"]["x"] for z in zones] + [cx]
    ys = [p["pose"]["y"] for p in pucks] + [z["pose"]["y"] for z in zones] + [cy]
    # 미션 앞쪽에는 동적 장애물이 나타나지 않도록 넉넉히 비운다.
    clear = (min(xs) - 0.45, min(ys) - 0.45, max(xs) + 0.45, max(ys) + 0.45)

    station_info = {
        "index": int(index),
        "name": mission,
        "suffix": STATION_SPECS[mission]["suffix"],
        "label": info["label"],
        "text": info["text"],
        "tag_id": MISSION_TAG[mission],
        "tag_name": mission_tag["name"],
        "active": bool(active),
        "pucks": len(pucks),
        "puck_names": [p["name"] for p in pucks],
        "colors": {c: sum(1 for p in pucks if p["color_name"] == c) for c in COLOR_ORDER},
        "lying": [p["name"] for p in pucks if p["lying"]],
        "target_laying": len(pucks) if mission == "LAY_DOWN" else 0,
        "cluster": {"x": round(cx, 6), "y": round(cy, 6)},
        "zones": zone_specs,
        "wall_tag": {
            "x": mission_tag["pose"]["x"], "y": mission_tag["pose"]["y"],
            "z": mission_tag["pose"]["z"], "normal": mission_tag["normal"],
            "wall": tag_wall,
        },
        "zone_tag": ({"x": zone_tag["pose"]["x"], "y": zone_tag["pose"]["y"],
                      "z": zone_tag["pose"]["z"], "tag_id": B_TAG_ID}
                     if zone_tag is not None else None),
    }
    return objects, station_info, reserved, clear, mission_tag


def build_missions(args, rng, walls):
    total = max(0, args.pucks)
    if args.mission_sites == "single":
        specs = [(args.mission, tuple(args.mission_center), total,
                  STATION_ORDER.index(args.mission), True)]
    else:
        counts = station_counts(total)
        specs = [(mission, STATION_SPECS[mission]["center"], counts[i], i,
                  mission == args.mission)
                 for i, mission in enumerate(STATION_ORDER)]

    objects = []
    stations = []
    reserved = []
    clears = []
    for mission, center, count, index, active in specs:
        color_plan = [COLOR_ORDER[(i + index) % len(COLOR_ORDER)] for i in range(count)]
        station_objects, info, station_reserved, clear, tag = build_station(
            args, rng, walls, mission, center, count, color_plan, index, active)
        objects.extend(station_objects)
        stations.append(info)
        reserved.extend(station_reserved)
        clears.append(clear)

    active_info = next((st for st in stations if st["active"]), stations[0])
    board = [{
        "name": st["name"], "label": st["label"], "tag_id": st["tag_id"],
        "tag_name": st["tag_name"], "active": st["active"],
        "x": st["wall_tag"]["x"], "y": st["wall_tag"]["y"],
        "z": st["wall_tag"]["z"], "normal": st["wall_tag"]["normal"],
        "wall": st["wall_tag"]["wall"],
    } for st in stations]
    all_pucks = [o for o in objects if o["kind"] == "puck"]
    mission_info = {
        "name": args.mission,
        "label": generate_tags.MISSIONS[args.mission]["label"],
        "text": generate_tags.MISSIONS[args.mission]["text"],
        "tag_id": MISSION_TAG[args.mission],
        "pucks": len(all_pucks),
        "active_pucks": active_info["pucks"],
        "puck_faces": 6,
        "tag_family": generate_tags.FAMILY,
        "colors": {c: sum(1 for p in all_pucks if p["color_name"] == c) for c in COLOR_ORDER},
        "lying": [p["name"] for p in all_pucks if p["lying"]],
        "target_laying": active_info["target_laying"],
        "cluster": dict(active_info["cluster"]),
        "zones": active_info["zones"],
        "stations": stations,
        "mission_board": board,
        "wall_tag": dict(active_info["wall_tag"]),
        "zone_tag": active_info["zone_tag"],
        "mission_sites": args.mission_sites,
    }
    return objects, mission_info, reserved, clears


def generate_obstacles(args, rng, walls, spawn, reserved, clear):
    x_min, x_max, y_min, y_max = wall_bounds(walls)
    placed = []
    shapes = ["box"] * args.boxes + ["cylinder"] * args.cylinders
    rng.shuffle(shapes)
    counts = {"box": 0, "cylinder": 0}
    for shape in shapes:
        size = args.box_size if shape == "box" else args.cylinder_size
        name = f"gz_{shape}_{counts[shape]}"
        counts[shape] += 1
        radius = math.hypot(size[0], size[1]) / 2.0 if shape == "box" else size[0] / 2.0
        for _ in range(8000):
            x = rng.uniform(x_min, x_max)
            y = rng.uniform(y_min, y_max)
            if math.hypot(x - spawn[0], y - spawn[1]) < 0.35:
                continue
            if wall_hit(walls, x, y, radius):
                continue
            if clear_hit(clear, reserved, x, y, radius):
                continue
            if any(math.hypot(x - p["pose"]["x"], y - p["pose"]["y"]) <
                   radius + object_radius(p) + 0.025 for p in placed):
                continue
            placed.append(make_obstacle(name, shape, size, x, y,
                                        rng.uniform(-math.pi, math.pi), rng, args.static))
            break
        else:
            print(f"WARN: {name} 자리를 찾지 못했습니다 (벽/물체가 너무 많음)", file=sys.stderr)
    return placed


def sdf_for_obstacle(obj):
    shape = obj["shape"]
    p = obj["pose"]
    if shape == "box":
        sx, sy, sz = obj["size"]
        volume = sx * sy * sz
        mass = max(0.03, volume * 650.0)
        ixx, iyy, izz = box_inertia(mass, sx, sy, sz)
        geometry = f"<box><size>{sx:.9g} {sy:.9g} {sz:.9g}</size></box>"
    else:
        diameter, height = obj["size"]
        radius = diameter / 2.0
        volume = math.pi * radius * radius * height
        mass = max(0.03, volume * 650.0)
        ixx, iyy, izz = cylinder_inertia(mass, radius, height)
        geometry = f"<cylinder><radius>{radius:.9g}</radius><length>{height:.9g}</length></cylinder>"
    r, g, b = obj["color"]
    return f"""
  <model name="{obj['name']}">
    <pose>{pose_text(p)}</pose>
    <static>{'true' if obj['static'] else 'false'}</static>
    <self_collide>false</self_collide>
    <link name="link">
      <inertial>
        <mass>{mass:.9g}</mass>
        <inertia>
          <ixx>{ixx:.9g}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy:.9g}</iyy><iyz>0</iyz><izz>{izz:.9g}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>{geometry}</geometry>
      </collision>
      <visual name="visual">
        <geometry>{geometry}</geometry>
        <material>
          <ambient>{r:.5g} {g:.5g} {b:.5g} 1</ambient>
          <diffuse>{r:.5g} {g:.5g} {b:.5g} 1</diffuse>
          <specular>0.08 0.08 0.08 1</specular>
        </material>
      </visual>
    </link>
  </model>"""


def write_spawn_models(args, tag_assets):
    """Gazebo GUI 플러그인이 런타임에 소환할 수 있는 모델 SDF를 준비한다."""
    out = Path(args.spawn_model_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    models = {}
    for color in COLOR_ORDER:
        name = f"puck_{color}"
        puck = make_puck(f"manual_puck_{color}", color, args.puck_size, 0.0, 0.0, False, 0.0)
        path = out / f"{name}.sdf"
        path.write_text('<?xml version="1.0"?>\n<sdf version="1.8">\n' +
                        sdf_for_puck(puck, tag_assets, args.tag_size) + '\n</sdf>\n')
        models[name] = {"file": str(path), "z": float(puck["pose"]["z"]),
                        "label": f"{color} 퍽", "name": puck["name"]}
    light = make_traffic_light("manual_traffic_light", 0.0, 0.0, TRAFFIC_Z,
                               (-1.0, 0.0, 0.0), "수동 신호등", 0)
    path = out / "traffic_light.sdf"
    body = sdf_for_traffic_light(light)
    # 수동 소환용 모델은 자동 점등 런타임에 연결되지 않으므로, 최소한 렌즈가
    # 켜져 보이도록 빨강 발광 구를 하나 포함한다. (자동 신호등은 traffic_lights.py가 담당)
    lit = (f'<visual name="manual_lit_red"><pose>0 0 {TRAFFIC_LIT_Z:.6g} 0 0 0</pose>'
           '<geometry><sphere><radius>0.032</radius></sphere></geometry>'
           '<material><ambient>0.95 0.04 0.04 1</ambient>'
           '<diffuse>0.95 0.04 0.04 1</diffuse>'
           '<emissive>0.95 0.04 0.04 1</emissive>'
           '<lighting>false</lighting></material></visual>')
    body = body.replace('</link>', lit + '</link>', 1)
    path.write_text('<?xml version="1.0"?>\n<sdf version="1.8">\n' + body + '\n</sdf>\n')
    models["traffic_light"] = {"file": str(path), "z": float(TRAFFIC_Z),
                               "label": "가로 신호등", "name": light["name"]}
    (out / "catalog.json").write_text(json.dumps(models, indent=2, ensure_ascii=False) + "\n")
    return models


def sdf_for_puck(obj, tag_assets, tag_size):
    sx, sy, sz = obj["size"]
    volume = sx * sy * sz
    mass = max(0.03, volume * 650.0)
    ixx, iyy, izz = box_inertia(mass, sx, sy, sz)
    r, g, b = obj["color"]
    tag = tag_assets[obj["tag_id"]]
    eps = 0.00035
    ts = tag_size
    faces = [
        ("px", (sx / 2.0 + eps, 0, 0), (1, 0, 0)),
        ("nx", (-sx / 2.0 - eps, 0, 0), (-1, 0, 0)),
        ("py", (0, sy / 2.0 + eps, 0), (0, 1, 0)),
        ("ny", (0, -sy / 2.0 - eps, 0), (0, -1, 0)),
        ("pz", (0, 0, sz / 2.0 + eps), (0, 0, 1)),
        ("nz", (0, 0, -sz / 2.0 - eps), (0, 0, -1)),
    ]
    assert len(faces) == 6, "퍽은 4옆면 + 위/아래 = 6면 태그가 필요합니다"
    tag_visuals = []
    for name, (tx, ty, tz), normal in faces:
        roll, pitch, yaw = rpy_for_normal(normal)
        tag_visuals.append(f"""
      <visual name="tag_{name}">
        <pose>{tx:.9g} {ty:.9g} {tz:.9g} {roll:.9g} {pitch:.9g} {yaw:.9g}</pose>
        <geometry><mesh><uri>file://{tag['obj']}</uri><scale>{ts} {ts} {ts}</scale></mesh></geometry>
        <cast_shadows>false</cast_shadows>
      </visual>""")
    return f"""
  <model name="{obj['name']}">
    <pose>{pose_text(obj['pose'])}</pose>
    <static>false</static>
    <self_collide>false</self_collide>
    <link name="link">
      <inertial>
        <mass>{mass:.9g}</mass>
        <inertia>
          <ixx>{ixx:.9g}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy:.9g}</iyy><iyz>0</iyz><izz>{izz:.9g}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{sx:.9g} {sy:.9g} {sz:.9g}</size></box></geometry>
      </collision>
      <visual name="body">
        <geometry><box><size>{sx:.9g} {sy:.9g} {sz:.9g}</size></box></geometry>
        <material>
          <ambient>{r:.5g} {g:.5g} {b:.5g} 1</ambient>
          <diffuse>{r:.5g} {g:.5g} {b:.5g} 1</diffuse>
          <specular>0.10 0.10 0.10 1</specular>
        </material>
      </visual>{''.join(tag_visuals)}
    </link>
  </model>"""


def sdf_for_tag_model(obj, tag_assets):
    tag = tag_assets[obj["tag_id"]]
    size = obj["size"][0]
    return f"""
  <model name="{obj['name']}">
    <pose>{pose_text(obj['pose'])}</pose>
    <static>true</static>
    <link name="link">
      <visual name="tag">
        <geometry><mesh><uri>file://{tag['obj']}</uri><scale>{size} {size} {size}</scale></mesh></geometry>
        <cast_shadows>false</cast_shadows>
      </visual>
    </link>
  </model>"""


def sdf_for_zone(obj):
    w, h, _ = obj["size"]
    r, g, b = obj["color"]
    p = obj["pose"]
    return f"""
  <model name="{obj['name']}">
    <pose>{pose_text(p)}</pose>
    <static>true</static>
    <link name="link">
      <visual name="border">
        <geometry><plane><normal>0 0 1</normal><size>{w + 0.012:.9g} {h + 0.012:.9g}</size></plane></geometry>
        <material><ambient>0.03 0.03 0.03 1</ambient><diffuse>0.03 0.03 0.03 1</diffuse></material>
      </visual>
      <visual name="fill">
        <pose>0 0 0.001 0 0 0</pose>
        <geometry><plane><normal>0 0 1</normal><size>{w:.9g} {h:.9g}</size></plane></geometry>
        <material><ambient>{r:.5g} {g:.5g} {b:.5g} 1</ambient><diffuse>{r:.5g} {g:.5g} {b:.5g} 1</diffuse></material>
      </visual>
    </link>
  </model>"""



# ---------------------------------------------------------------------------
# 선택형 환경 텍스처 (simulator/images → Gazebo mesh/MTL)
# ---------------------------------------------------------------------------

def _env_asset(kind, value):
    """'floor_wood', 'floor_wood.jpg', 'none' → 실제 파일 또는 None."""
    if value in (None, "", "none", "None", "off", "OFF"):
        return None
    name = str(value).strip()
    path = ENV_ASSET_DIR / kind / name
    if not path.is_file() and not path.suffix:
        path = ENV_ASSET_DIR / kind / (name + ".jpg")
    if not path.is_file():
        choices = ", ".join(p.name for p in (ENV_ASSET_DIR / kind).glob("*.jpg"))
        raise SystemExit(f"FAIL: {kind} 텍스처 없음: {value} (가능: {choices})")
    return path


def _obj_quad(verts, uvs, norms, faces, points, uv, normal):
    start = len(verts) + 1
    for i, p in enumerate(points):
        verts.append(p)
        uvs.append(uv[i])
        norms.append(normal)
    faces.append((start, start + 1, start + 2, start + 3))


def _write_mtl(path, material, image):
    path.write_text(
        f"newmtl {material}\n"
        "Ka 1 1 1\n"
        "Kd 1 1 1\n"
        "Ks 0 0 0\n"
        "d 1\n"
        "illum 1\n"
        f"map_Kd {Path(image).resolve()}\n")


def _write_obj(path, material, verts, uvs, norms, faces):
    lines = [f"mtllib {path.with_suffix('.mtl').name}", f"o {path.stem}"]
    lines += [f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in verts]
    lines += [f"vt {u:.6f} {v:.6f}" for u, v in uvs]
    lines += [f"vn {x:.6f} {y:.6f} {z:.6f}" for x, y, z in norms]
    lines += [f"usemtl {material}", "s off"]
    lines += [f"f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c} {d}/{d}/{d}"
              for a, b, c, d in faces]
    path.write_text("\n".join(lines) + "\n")


def _write_floor_assets(out_dir, image):
    obj = out_dir / "floor.obj"
    mtl = out_dir / "floor.mtl"
    _write_mtl(mtl, "Beebot/Floor", image)
    verts = [(-7.0, -7.0, 0.0), (7.0, -7.0, 0.0),
             (7.0, 7.0, 0.0), (-7.0, 7.0, 0.0)]
    uvs = [(0.0, 0.0), (14.0, 0.0), (14.0, 14.0), (0.0, 14.0)]
    norms = [(0.0, 0.0, 1.0)] * 4
    faces = [(1, 2, 3, 4)]
    _write_obj(obj, "Beebot/Floor", verts, uvs, norms, faces)
    return obj


def _write_maze_assets(out_dir, walls, image, height):
    obj = out_dir / "maze.obj"
    mtl = out_dir / "maze.mtl"
    _write_mtl(mtl, "Beebot/Wall", image)
    # 벽면은 법선/조명 방향에 따라 어둡게 나오지 않도록 unlit 텍스처로 둔다.
    mtl.write_text(mtl.read_text().replace("illum 1", "illum 0"))
    verts, uvs, norms, faces = [], [], [], []
    for wall in walls:
        x, y = float(wall["x"]), float(wall["y"])
        sx, sy = float(wall["sx"]), float(wall["sy"])
        x0, x1 = x - sx / 2.0, x + sx / 2.0
        y0, y1 = y - sy / 2.0, y + sy / 2.0
        z0, z1 = 0.0, height
        # sides: horizontal texture repeats every 0.5 m, vertical stretches to wall height
        ux, uy = max(0.1, sx / 0.5), max(0.1, sy / 0.5)
        # bottom / top
        _obj_quad(verts, uvs, norms, faces,
                  [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)],
                  [(0, 0), (ux, 0), (ux, uy), (0, uy)], (0, 0, -1))
        _obj_quad(verts, uvs, norms, faces,
                  [(x0, y0, z1), (x0, y1, z1), (x1, y1, z1), (x1, y0, z1)],
                  [(0, 0), (0, uy), (ux, uy), (ux, 0)], (0, 0, 1))
        # four sides
        _obj_quad(verts, uvs, norms, faces,
                  [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)],
                  [(0, 0), (ux, 0), (ux, 1), (0, 1)], (0, -1, 0))
        _obj_quad(verts, uvs, norms, faces,
                  [(x1, y1, z0), (x0, y1, z0), (x0, y1, z1), (x1, y1, z1)],
                  [(0, 0), (ux, 0), (ux, 1), (0, 1)], (0, 1, 0))
        _obj_quad(verts, uvs, norms, faces,
                  [(x0, y1, z0), (x0, y0, z0), (x0, y0, z1), (x0, y1, z1)],
                  [(0, 0), (uy, 0), (uy, 1), (0, 1)], (-1, 0, 0))
        _obj_quad(verts, uvs, norms, faces,
                  [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)],
                  [(0, 0), (uy, 0), (uy, 1), (0, 1)], (1, 0, 0))
    _write_obj(obj, "Beebot/Wall", verts, uvs, norms, faces)
    return obj


def _write_panorama_assets(out_dir, image, radius=9.0, lat_n=24, lon_n=48):
    obj = out_dir / "panorama.obj"
    mtl = out_dir / "panorama.mtl"
    _write_mtl(mtl, "Beebot/Panorama", image)
    # MTL illum 1 would require lights; use a flat visible texture
    text = mtl.read_text().replace("illum 1", "illum 0")
    mtl.write_text(text)
    verts, uvs, norms, faces = [], [], [], []
    for i in range(lat_n + 1):
        lat = math.pi / 2.0 - math.pi * i / lat_n
        for j in range(lon_n + 1):
            lon = -math.pi + 2.0 * math.pi * j / lon_n
            x = radius * math.cos(lat) * math.cos(lon)
            y = radius * math.cos(lat) * math.sin(lon)
            z = radius * math.sin(lat)
            verts.append((x, y, z))
            # OGRE의 OBJ V축은 아래가 0이라, 이미지 상하가 뒤집히지 않도록 뒤집는다.
            uvs.append((j / lon_n, 1.0 - i / lat_n))
            norms.append((-x / radius, -y / radius, -z / radius))
    for i in range(lat_n):
        for j in range(lon_n):
            a = i * (lon_n + 1) + j + 1
            b = a + 1
            c = a + (lon_n + 1)
            d = c + 1
            # inward winding so the user camera sees the inside of the sphere
            faces.append((a, b, c))
            faces.append((b, d, c))
    lines = [f"mtllib {obj.with_suffix('.mtl').name}", "o panorama"]
    lines += [f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in verts]
    lines += [f"vt {u:.6f} {v:.6f}" for u, v in uvs]
    lines += [f"vn {x:.6f} {y:.6f} {z:.6f}" for x, y, z in norms]
    lines += ["usemtl Beebot/Panorama", "s off"]
    lines += [f"f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}" for a, b, c in faces]
    obj.write_text("\n".join(lines) + "\n")
    return obj


def _mesh_uri(path):
    return f"file://{Path(path).resolve()}"


def build_environment(args, walls):
    """선택된 floor/wall/background를 Gazebo용 OBJ/MTL + SDF로 만든다."""
    floor_img = _env_asset("floor", args.floor_texture)
    wall_img = _env_asset("wall", args.wall_texture)
    pano_img = _env_asset("background", args.background)
    if not (floor_img or wall_img or pano_img):
        return "", "", "", {"floor": None, "wall": None, "background": None}
    out = Path(args.env_out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    floor_sdf = wall_sdf = pano_sdf = ""
    info = {"floor": None, "wall": None, "background": None}
    if floor_img:
        obj = _write_floor_assets(out, floor_img)
        floor_sdf = f"""
  <model name="textured_floor">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><plane><normal>0 0 1</normal><size>14 14</size></plane></geometry>
      </collision>
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>{_mesh_uri(obj)}</uri></mesh></geometry>
      </visual>
    </link>
  </model>"""
        info["floor"] = str(floor_img)
    if wall_img:
        obj = _write_maze_assets(out, walls, wall_img, WALL_HEIGHT)
        collisions = []
        for wall in walls:
            name = re.sub(r"[^A-Za-z0-9_]", "_", str(wall.get("name", "wall")))
            collisions.append(
                f'      <collision name="{name}"><pose>{wall["x"]:.9g} {wall["y"]:.9g} '
                f'{WALL_HEIGHT / 2.0:.9g} 0 0 0</pose><geometry><box>'
                f'<size>{wall["sx"]:.9g} {wall["sy"]:.9g} {WALL_HEIGHT:.9g}</size>'
                f'</box></geometry></collision>')
        wall_sdf = f"""
  <model name="twin_closed_maze">
    <static>true</static>
    <link name="maze">
      <visual name="walls">
        <cast_shadows>true</cast_shadows>
        <geometry><mesh><uri>{_mesh_uri(obj)}</uri></mesh></geometry>
      </visual>
{''.join(collisions)}
    </link>
  </model>"""
        info["wall"] = str(wall_img)
    if pano_img:
        obj = _write_panorama_assets(out, pano_img)
        pano_sdf = f"""
  <model name="panorama_background">
    <static>true</static>
    <link name="link">
      <visual name="sphere">
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>{_mesh_uri(obj)}</uri></mesh></geometry>
      </visual>
    </link>
  </model>"""
        info["background"] = str(pano_img)
    return floor_sdf, wall_sdf, pano_sdf, info




# ---------------------------------------------------------------------------
# GUI 환경 선택용 전체 모델 카탈로그
# ---------------------------------------------------------------------------

def _floor_model_sdf(obj_path):
    return f"""<sdf version="1.8">
  <model name="textured_floor">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><plane><normal>0 0 1</normal><size>14 14</size></plane></geometry>
      </collision>
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>{_mesh_uri(obj_path)}</uri></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>"""


def _floor_none_sdf():
    return """<sdf version="1.8">
  <model name="textured_floor">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><plane><normal>0 0 1</normal><size>14 14</size></plane></geometry>
      </collision>
      <visual name="visual">
        <geometry><plane><normal>0 0 1</normal><size>14 14</size></plane></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


def _maze_model_sdf(obj_path, walls):
    collisions = []
    for wall in walls:
        name = re.sub(r"[^A-Za-z0-9_]", "_", str(wall.get("name", "wall")))
        collisions.append(
            f'      <collision name="{name}"><pose>{wall["x"]:.9g} {wall["y"]:.9g} '
            f'{WALL_HEIGHT / 2.0:.9g} 0 0 0</pose><geometry><box>'
            f'<size>{wall["sx"]:.9g} {wall["sy"]:.9g} {WALL_HEIGHT:.9g}</size>'
            f'</box></geometry></collision>')
    return f"""<sdf version="1.8">
  <model name="twin_closed_maze">
    <static>true</static>
    <link name="maze">
      <visual name="walls">
        <cast_shadows>true</cast_shadows>
        <geometry><mesh><uri>{_mesh_uri(obj_path)}</uri></mesh></geometry>
      </visual>
{''.join(collisions)}
    </link>
  </model>
</sdf>"""


def _maze_none_sdf(walls):
    parts = []
    for wall in walls:
        name = re.sub(r"[^A-Za-z0-9_]", "_", str(wall.get("name", "wall")))
        x, y = float(wall["x"]), float(wall["y"])
        sx, sy = float(wall["sx"]), float(wall["sy"])
        parts.append(
            f'      <collision name="{name}"><pose>{x:.9g} {y:.9g} '
            f'{WALL_HEIGHT / 2.0:.9g} 0 0 0</pose><geometry><box>'
            f'<size>{sx:.9g} {sy:.9g} {WALL_HEIGHT:.9g}</size></box></geometry></collision>\n'
            f'      <visual name="{name}_v"><pose>{x:.9g} {y:.9g} '
            f'{WALL_HEIGHT / 2.0:.9g} 0 0 0</pose><geometry><box>'
            f'<size>{sx:.9g} {sy:.9g} {WALL_HEIGHT:.9g}</size></box></geometry>'
            '<material><ambient>0.65 0.72 0.8 1</ambient>'
            '<diffuse>0.65 0.72 0.8 1</diffuse></material></visual>')
    return '<sdf version="1.8">\n  <model name="twin_closed_maze">\n    <static>true</static>\n    <link name="maze">\n' +            '\n'.join(parts) + '\n    </link>\n  </model>\n</sdf>\n'


def _pano_model_sdf(obj_path):
    return f"""<sdf version="1.8">
  <model name="panorama_background">
    <static>true</static>
    <link name="link">
      <visual name="sphere">
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>{_mesh_uri(obj_path)}</uri></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>"""


def write_environment_templates(args, walls):
    """모든 바닥/벽/파노라마를 GUI에서 즉시 교체할 수 있도록 SDF로 준비한다."""
    base = Path(args.env_model_dir).resolve()
    base.mkdir(parents=True, exist_ok=True)
    catalog = {"floor": [], "wall": [], "background": []}

    for image in sorted((ENV_ASSET_DIR / "floor").glob("*.jpg")):
        sub = base / "floor" / image.stem
        sub.mkdir(parents=True, exist_ok=True)
        obj = _write_floor_assets(sub, image)
        (sub / "model.sdf").write_text(_floor_model_sdf(obj))
        catalog["floor"].append(image.stem)
    none_floor = base / "floor" / "none"
    none_floor.mkdir(parents=True, exist_ok=True)
    (none_floor / "model.sdf").write_text(_floor_none_sdf())
    catalog["floor"].append("none")

    for image in sorted((ENV_ASSET_DIR / "wall").glob("*.jpg")):
        sub = base / "wall" / image.stem
        sub.mkdir(parents=True, exist_ok=True)
        obj = _write_maze_assets(sub, walls, image, WALL_HEIGHT)
        (sub / "model.sdf").write_text(_maze_model_sdf(obj, walls))
        catalog["wall"].append(image.stem)
    none_wall = base / "wall" / "none"
    none_wall.mkdir(parents=True, exist_ok=True)
    (none_wall / "model.sdf").write_text(_maze_none_sdf(walls))
    catalog["wall"].append("none")

    for image in sorted((ENV_ASSET_DIR / "background").glob("*.jpg")):
        sub = base / "background" / image.stem
        sub.mkdir(parents=True, exist_ok=True)
        obj = _write_panorama_assets(sub, image)
        (sub / "model.sdf").write_text(_pano_model_sdf(obj))
        catalog["background"].append(image.stem)

    (base / "catalog.json").write_text(json.dumps(catalog, indent=2, ensure_ascii=False) + "\n")
    return catalog

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--world-in", default=str(DEFAULT_WORLD))
    ap.add_argument("--walls", default=str(DEFAULT_WALLS))
    ap.add_argument("--world-out", default="/tmp/beebot_world.sdf")
    ap.add_argument("--state-out", default="/tmp/beebot_scene.json")
    ap.add_argument("--asset-dir", default=str(ASSET_DIR))
    ap.add_argument("--mission", default=os.environ.get("GZ_MISSION", "random").upper())
    ap.add_argument("--mission-center", default=env_pair("GZ_MISSION_CENTER", (0.35, 2.75)),
                    metavar=("X", "Y"), nargs=2, type=float)
    ap.add_argument("--mission-wall", default=os.environ.get("GZ_MISSION_WALL", "north").lower(),
                    choices=("north", "south", "east", "west"))
    ap.add_argument("--mission-sites", default=os.environ.get("GZ_MISSION_SITES", "all").lower(),
                    choices=("all", "single"),
                    help="all=네 미션을 지도에 분산, single=활성 미션만 한 곳에 (기본 all)")
    ap.add_argument("--pucks", type=int, default=env_int("GZ_PUCK_COUNT", 9))
    ap.add_argument("--puck-mm", type=float, nargs=3,
                    default=env_floats("GZ_PUCK_MM", [30.0, 30.0, 60.0]))
    ap.add_argument("--tag-mm", type=float, default=float(os.environ.get("GZ_TAG_SIZE_MM", "28")))
    ap.add_argument("--mission-tag-mm", type=float,
                    default=float(os.environ.get("GZ_MISSION_TAG_MM", "120")))
    ap.add_argument("--boxes", type=int, default=env_int("GZ_OBSTACLE_BOXES", 6))
    ap.add_argument("--cylinders", type=int, default=env_int("GZ_OBSTACLE_CYLINDERS", 6))
    ap.add_argument("--box-mm", type=float, nargs=3,
                    default=env_floats("GZ_OBSTACLE_BOX_MM", [30.0, 30.0, 60.0]))
    ap.add_argument("--cylinder-mm", type=float, nargs=2,
                    default=env_floats("GZ_OBSTACLE_CYLINDER_MM", [30.0, 60.0]))
    ap.add_argument("--dynamic-count", type=int, default=env_int("GZ_DYNAMIC_COUNT", 3),
                    help="동시에 떠 있는 동적 장애물 수 (0이면 끔)")
    ap.add_argument("--dynamic-lifetime", type=float,
                    default=float(os.environ.get("GZ_DYNAMIC_LIFETIME", "12")))
    ap.add_argument("--dynamic-respawn", type=float,
                    default=float(os.environ.get("GZ_DYNAMIC_RESPAWN", "1")))
    ap.add_argument("--dynamic-speed", type=float,
                    default=float(os.environ.get("GZ_DYNAMIC_SPEED", "0.16")))
    ap.add_argument("--dynamic-mm", type=float, nargs=3,
                    default=env_floats("GZ_DYNAMIC_MM", [220.0, 220.0, 600.0]))
    ap.add_argument("--traffic-interval", type=float,
                    default=float(os.environ.get("GZ_TRAFFIC_INTERVAL", str(TRAFFIC_INTERVAL))))
    ap.add_argument("--spawn", default=os.environ.get("GZ_OBSTACLE_SPAWN", "-1.835 2.757 0.05 0"),
                    help="로봇 스폰 x y z yaw (물체를 이 자리 근처에 놓지 않음)")
    ap.add_argument("--seed", type=int,
                    default=(int(os.environ.get("GZ_MISSION_SEED") or os.environ["GZ_OBSTACLE_SEED"])
                             if (os.environ.get("GZ_MISSION_SEED") or os.environ.get("GZ_OBSTACLE_SEED"))
                             else None))
    ap.add_argument("--static", action="store_true",
                    default=os.environ.get("GZ_OBSTACLE_STATIC", "0") not in ("0", "", "false", "False"))
    ap.add_argument("--force-tags", action="store_true")
    ap.add_argument("--floor-texture", default=os.environ.get("GZ_FLOOR_TEXTURE", "none"),
                    help="assets/environment/floor 파일명 또는 none")
    ap.add_argument("--wall-texture", default=os.environ.get("GZ_WALL_TEXTURE", "none"),
                    help="assets/environment/wall 파일명 또는 none")
    ap.add_argument("--background", default=os.environ.get("GZ_BACKGROUND", "none"),
                    help="assets/environment/background 파노라마 또는 none")
    ap.add_argument("--env-out", default=os.environ.get("GZ_ENV_OUT", "/tmp/beebot_environment"),
                    help="생성한 환경 OBJ/MTL 디렉터리")
    ap.add_argument("--spawn-model-dir",
                    default=os.environ.get("BEEBOT_SPAWN_DIR", "/tmp/beebot_spawn_models"),
                    help="Gazebo GUI 플러그인이 소환할 모델 SDF 디렉터리")
    ap.add_argument("--env-model-dir",
                    default=os.environ.get("BEEBOT_ENV_MODEL_DIR", "/tmp/beebot_environment_models"),
                    help="GUI 환경 선택용 전체 바닥/벽/파노라마 모델 디렉터리")
    args = ap.parse_args()

    if args.mission == "RANDOM":
        args.mission = "random"
    if args.mission == "random":
        rng_for_mission = random.Random(args.seed)
        args.mission = rng_for_mission.choice(MISSION_NAMES)
        if args.seed is None:
            # 장애물 난수와 미션 난수를 분리하되, seed를 줬을 때는 재현 가능하게.
            args.seed = rng_for_mission.randrange(1 << 30)
    if args.mission not in MISSION_NAMES:
        sys.exit(f"FAIL: 알 수 없는 미션: {args.mission} (가능: {', '.join(MISSION_NAMES)})")
    if args.pucks < 0 or args.boxes < 0 or args.cylinders < 0:
        sys.exit("FAIL: 개수는 0 이상이어야 합니다")

    world_in = Path(args.world_in)
    walls_path = Path(args.walls)
    if not world_in.is_file():
        sys.exit(f"FAIL: 월드 없음: {world_in}")
    if not walls_path.is_file():
        sys.exit(f"FAIL: walls.json 없음: {walls_path}")

    walls = json.loads(walls_path.read_text())["boxes"]
    spawn = [float(v) for v in args.spawn.replace(",", " ").split()]
    if len(spawn) != 4:
        sys.exit("FAIL: --spawn 은 'x y z yaw' 네 값이어야 합니다")

    args.puck_size = [v / 1000.0 for v in args.puck_mm]
    args.box_size = [v / 1000.0 for v in args.box_mm]
    args.cylinder_size = [args.cylinder_mm[0] / 1000.0, args.cylinder_mm[1] / 1000.0]
    args.dynamic_size = [v / 1000.0 for v in args.dynamic_mm]
    args.mission = args.mission.upper()
    args.mission_sites = args.mission_sites.lower()
    args.mission_center = tuple(args.mission_center)
    if min(args.puck_size + args.box_size + args.cylinder_size) <= 0.0:
        sys.exit("FAIL: 물체 크기는 0보다 커야 합니다")
    if min(args.puck_size) <= 0 or args.tag_mm <= 0 or args.mission_tag_mm <= 0:
        sys.exit("FAIL: 퍽/태그 크기는 0보다 커야 합니다")
    if args.dynamic_count < 0 or min(args.dynamic_size) <= 0:
        sys.exit("FAIL: 동적 장애물 개수/크기를 확인하세요")
    if args.dynamic_lifetime <= 0 or args.dynamic_respawn < 0 or args.dynamic_speed < 0:
        sys.exit("FAIL: 동적 장애물 수명/재등장/속도는 0 이상이어야 합니다")
    if args.traffic_interval <= 0:
        sys.exit("FAIL: 신호등 주기는 0보다 커야 합니다")
    tag_size = min(args.tag_mm / 1000.0, min(args.puck_size) * 0.94)
    args.tag_size = tag_size
    args.mission_tag_size = args.mission_tag_mm / 1000.0
    if args.mission_tag_size > WALL_HEIGHT - 0.04:
        sys.exit(f"FAIL: 미션 태그가 벽 높이({WALL_HEIGHT} m)보다 큽니다")

    tag_assets, tag_metadata = generate_tags.ensure_assets(args.asset_dir,
                                                           force=args.force_tags)
    spawn_models = write_spawn_models(args, tag_assets)
    rng = random.Random(args.seed)

    env_floor, env_wall, env_pano, env_info = build_environment(args, walls)
    env_catalog = write_environment_templates(args, walls)

    mission_objects, mission_info, reserved, clears = build_missions(args, rng, walls)
    traffic_lights = build_traffic_lights(args, walls)
    for light in traffic_lights:
        # 갠트리 기둥까지 포함해 장애물/동적 물체가 신호등 밑에 붙지 않게 한다.
        reserved.append({"x": light["pose"]["x"], "y": light["pose"]["y"], "r": 0.55})
    obstacles = generate_obstacles(args, rng, walls, spawn, reserved, clears)
    dynamic = build_dynamic_config(args, rng, walls, spawn, reserved, clears,
                                   mission_objects + obstacles + traffic_lights)
    objects = mission_objects + obstacles + traffic_lights

    text = world_in.read_text()
    if "</world>" not in text:
        sys.exit(f"FAIL: </world>를 찾지 못했습니다: {world_in}")
    if env_floor:
        replaced, n = re.subn(r'<model name="ground">.*?</model>', env_floor, text,
                              count=1, flags=re.S)
        if n != 1:
            sys.exit("FAIL: 바닥 ground 모델을 찾지 못했습니다")
        text = replaced
    if env_wall:
        replaced, n = re.subn(r'<include>\s*<uri>model://twin_closed_maze</uri>\s*</include>',
                              '', text, count=1, flags=re.S)
        if n != 1:
            sys.exit("FAIL: twin_closed_maze include를 찾지 못했습니다")
        text = replaced

    chunks = []
    for obj in objects:
        if obj["kind"] == "puck":
            chunks.append(sdf_for_puck(obj, tag_assets, tag_size))
        elif obj["kind"] == "zone":
            chunks.append(sdf_for_zone(obj))
        elif obj["kind"] in ("mission_tag", "zone_tag"):
            chunks.append(sdf_for_tag_model(obj, tag_assets))
        elif obj["kind"] == "traffic_light":
            chunks.append(sdf_for_traffic_light(obj))
        else:
            chunks.append(sdf_for_obstacle(obj))
    if env_wall:
        chunks.append(env_wall)
    if env_pano:
        chunks.append(env_pano)
    text = text.replace("</world>", "\n".join(chunks) + "\n</world>", 1)

    Path(args.world_out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.world_out).write_text(text)
    traffic_config = {
        "enabled": bool(traffic_lights),
        "interval_sec": float(args.traffic_interval),
        "order": list(TRAFFIC_ORDER),
        "lights": [{
            "name": light["name"],
            "label": light["label"],
            "index": light["index"],
            "pose": light["pose"],
            "normal": light["normal"],
            "lenses": light["lenses"],
        } for light in traffic_lights],
    }
    state = {
        "world": "default",
        "frame_id": "world",
        "source_world": str(world_in),
        "spawn": {"x": spawn[0], "y": spawn[1], "z": spawn[2], "yaw": spawn[3]},
        "mission": mission_info,
        "tag_map": tag_metadata["tags"],
        "walls": walls,
        "dynamic": dynamic,
        "traffic": traffic_config,
        "environment": env_info,
        "environment_catalog": env_catalog,
        "spawn_models": spawn_models,
        "objects": objects,
    }
    Path(args.state_out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.state_out).write_text(json.dumps(state, indent=2, ensure_ascii=False) + "\n")

    counts = {kind: sum(1 for o in objects if o["kind"] == kind)
              for kind in ("obstacle", "puck", "zone", "mission_tag", "zone_tag", "traffic_light")}
    print(f"[generate_world] mission={args.mission} pucks={counts['puck']} (6-face tags) "
          f"zones={counts['zone']} tags={counts['mission_tag'] + counts['zone_tag']} "
          f"obstacles={counts['obstacle']} signals={counts['traffic_light']} "
          f"dynamic={len(dynamic['spawn_points'])}pts → {args.state_out}")
    cluster = mission_info["cluster"]
    print(f"  active_cluster=({cluster['x']:+.3f}, {cluster['y']:+.3f}) "
          f"wall_tag={mission_info['wall_tag']}")
    print(f"  stations={[m['name'] + ('*' if m['active'] else '') for m in mission_info['mission_board']]}")
    print(f"  traffic={[l['name'] for l in traffic_lights]} "
          f"interval={args.traffic_interval:.1f}s dynamic_count={args.dynamic_count} "
          f"lifetime={args.dynamic_lifetime:.1f}s")
    print(f"  environment={env_info}")
    for p in mission_info["lying"]:
        print(f"  lying: {p}")
    for o in obstacles:
        p = o["pose"]
        print(f"  {o['name']:14s} {o['shape']:8s} x={p['x']:+.3f} y={p['y']:+.3f} yaw={p['yaw']:+.2f}")


if __name__ == "__main__":
    main()
