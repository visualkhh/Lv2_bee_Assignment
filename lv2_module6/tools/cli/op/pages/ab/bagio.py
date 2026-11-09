"""rosbag2 읽기·궤적 계산 공통 (A/B 리포트가 쓴다).

  from bagio import read_bag, plan_points, project_to_polyline, series, yaw_of
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np


def storage_id(path: str) -> str:
    """metadata.yaml의 storage_identifier (mcap·sqlite3). 없으면 확장자로 추정."""
    meta = Path(path) / "metadata.yaml"
    if meta.is_file():
        for line in meta.read_text(errors="ignore").splitlines():
            s = line.strip()
            if s.startswith("storage_identifier:"):
                return s.split(":", 1)[1].strip().strip("\"'")
    if any(f.suffix == ".mcap" for f in Path(path).glob("*")):
        return "mcap"
    return "sqlite3"


def read_bag(path: str) -> dict[str, list[tuple[float, object]]]:
    """{토픽: [(stamp_s, msg), …]} — 메시지 타입은 실행 환경(ROS)에서 가져온다."""
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id=storage_id(path)),
                ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out: dict[str, list[tuple[float, object]]] = {name: [] for name in types}
    while reader.has_next():
        name, data, stamp = reader.read_next()
        typ = types.get(name)
        if typ is None:
            continue
        try:
            from rclpy.serialization import deserialize_message
            from rosidl_runtime_py.utilities import get_message
            msg = deserialize_message(data, get_message(typ))
        except Exception:
            continue
        out[name].append((stamp / 1e9, msg))
    return out


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    """[-π, π]로 감기 — 스칼라·numpy 배열 모두."""
    return np.arctan2(np.sin(a), np.cos(a))


def series(msgs, fn):
    t = np.array([s for s, _ in msgs], float)
    v = np.array([fn(m) for _, m in msgs], float)
    return t, v


def plan_points(msgs) -> np.ndarray:
    if not msgs:
        return np.zeros((0, 2))
    path = msgs[-1][1]  # 마지막 경로
    return np.array([[p.pose.position.x, p.pose.position.y] for p in path.poses], float)


def project_to_polyline(xy: np.ndarray, poly: np.ndarray):
    """각 점 → 폴리라인 최단거리(횡오차), 진행거리(s), 접선각."""
    if len(poly) < 2 or len(xy) == 0:
        return np.zeros(len(xy)), np.zeros(len(xy)), np.zeros(len(xy))
    seg = np.diff(poly, axis=0)
    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(seg, axis=1))])
    dist = np.full(len(xy), np.inf)
    s = np.zeros(len(xy))
    head = np.zeros(len(xy))
    for i in range(len(seg)):
        a, b = poly[i], poly[i + 1]
        ab = b - a
        L2 = float(ab @ ab)
        if L2 <= 1e-12:
            continue
        u = np.clip(((xy - a) @ ab) / L2, 0.0, 1.0)
        proj = a + u[:, None] * ab
        d = np.linalg.norm(xy - proj, axis=1)
        take = d < dist
        dist[take] = d[take]
        s[take] = cum[i] + u[take] * math.sqrt(L2)
        head[take] = math.atan2(ab[1], ab[0])
    return dist, s, head
