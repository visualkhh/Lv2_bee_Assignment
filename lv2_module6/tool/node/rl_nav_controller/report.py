#!/usr/bin/env python3
"""rl_nav_controller — rosbag2 A/B 리포트 (report.png).

  ./run report ~/bags/policy ~/bags/dwb --labels 정책 DWB --out ~/report.png

읽는 토픽: /cmd_vel_nav(컨트롤러 출력) · /odometry/filtered · /odom · /imu · /plan ·
          /controller_server/FollowPath/debug  [a0, a1, v_out, w_out, scan_min, v_odom, w_odom, on]
지표: 횡오차(RMSE·최대) · 헤딩 MAE · 평균속도·시간·거리 · 가상스캔 최소거리 · 정책 ON 비율
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from common.bagio import plan_points, project_to_polyline, read_bag, series, wrap, yaw_of  # noqa: E402

FONTS = ["/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
         "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"]
DBG = "/controller_server/FollowPath/debug"


def stats(bag: str, data: dict) -> dict:
    res: dict = {}
    odom = data.get("/odometry/filtered") or data.get("/odom") or []
    if odom:
        t, v = series(odom, lambda m: m.twist.twist.linear.x)
        xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for _, m in odom], float)
        yaw = np.array([yaw_of(m.pose.pose.orientation) for _, m in odom], float)
        res["duration"] = float(t[-1] - t[0]) if len(t) > 1 else 0.0
        res["length"] = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
        res["v_mean"] = float(np.mean(np.abs(v)))
        poly = plan_points(data.get("/plan", []))
        if len(poly) >= 2:
            d, s, head = project_to_polyline(xy, poly)
            res["lat_rmse"] = float(np.sqrt(np.mean(d ** 2)))
            res["lat_max"] = float(np.max(d))
            res["head_mae"] = float(np.mean(np.abs(wrap(yaw - head))))
            res["progress"] = float(s[-1]) if len(s) else 0.0
    dbg = data.get(DBG, [])
    if dbg and len(dbg[0][1].data) >= 8:
        res["scan_min"] = float(np.mean([m.data[4] for _, m in dbg]))
        res["on"] = float(np.mean([m.data[7] for _, m in dbg]))
    return res


def plot(runs, out: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import font_manager as fm
    for f in FONTS:
        if Path(f).is_file():
            try:
                fm.fontManager.addfont(f)
            except Exception:
                pass
    import matplotlib.pyplot as plt
    matplotlib.rcParams["font.family"] = ["Noto Sans CJK JP", "Noto Sans CJK KR", "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False

    colors = ["#1e88e5", "#e53935", "#43a047", "#8e24aa"]
    fig, axes = plt.subplots(2, 2, figsize=(13, 9.0))
    fig.subplots_adjust(left=0.06, right=0.98, top=0.92, bottom=0.17, hspace=0.32, wspace=0.22)
    fig.suptitle("rl_nav_controller — A/B report", fontsize=13, fontweight="bold")

    ax = axes[0, 0]
    for i, (label, d, _st) in enumerate(runs):
        c = colors[i % len(colors)]
        if d.get("/cmd_vel_nav"):
            t, v = series(d["/cmd_vel_nav"], lambda m: m.twist.linear.x)
            if t.size:
                ax.plot(t - t[0], v, c, lw=1.4, label=f"{label} cmd")
        od = d.get("/odometry/filtered") or d.get("/odom") or []
        if od:
            t, v = series(od, lambda m: m.twist.twist.linear.x)
            if t.size:
                ax.plot(t - t[0], v, c, lw=0.8, ls="--", alpha=0.65, label=f"{label} odom")
    ax.set_title("선속도 [m/s]")
    ax.set_xlabel("time [s]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, ncol=2)

    ax = axes[0, 1]
    poly = None
    for i, (label, d, _st) in enumerate(runs):
        c = colors[i % len(colors)]
        od = d.get("/odometry/filtered") or d.get("/odom") or []
        if od:
            xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for _, m in od])
            if len(xy):
                ax.plot(xy[:, 0], xy[:, 1], c, lw=1.5, label=label)
        if poly is None and d.get("/plan"):
            poly = plan_points(d["/plan"])
    if poly is not None and len(poly) >= 2:
        ax.plot(poly[:, 0], poly[:, 1], "k--", lw=1.0, alpha=0.6, label="plan")
    ax.set_title("XY 궤적 [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[1, 0]
    for i, (label, d, _st) in enumerate(runs):
        c = colors[i % len(colors)]
        od = d.get("/odometry/filtered") or d.get("/odom") or []
        if od and poly is not None and len(poly) >= 2:
            xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for _, m in od])
            dist, s, _ = project_to_polyline(xy, poly)
            ax.plot(s, dist * 100.0, c, lw=1.3, label=label)
    ax.set_title("경로 횡오차 [cm]")
    ax.set_xlabel("진행 거리 [m]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[1, 1]
    for i, (label, d, _st) in enumerate(runs):
        c = colors[i % len(colors)]
        dbg = d.get(DBG, [])
        if dbg and len(dbg[0][1].data) >= 6:
            t = np.array([s for s, _ in dbg])
            a0 = np.array([m.data[0] for _, m in dbg])
            a1 = np.array([m.data[1] for _, m in dbg])
            ax.plot(t - t[0], a0, c, lw=1.2, label=f"{label} a0 (v)")
            ax.plot(t - t[0], a1, c, lw=1.0, ls=":", label=f"{label} a1 (ω)")
    ax.set_title("정책 행동 [-1,1]")
    ax.set_xlabel("time [s]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, ncol=2)

    lines = []
    for label, _d, st in runs:
        parts = []
        if "lat_rmse" in st:
            parts.append(f"횡오차 RMSE {st['lat_rmse']:.3f} m / 최대 {st['lat_max']:.3f} m")
        if "head_mae" in st:
            parts.append(f"헤딩 MAE {math.degrees(st['head_mae']):.1f}°")
        if "v_mean" in st:
            parts.append(f"평균 {st['v_mean']:.3f} m/s")
        if "scan_min" in st:
            parts.append(f"가상스캔 최소 {st['scan_min']:.2f} m")
        if "duration" in st:
            parts.append(f"{st['duration']:.1f}s · {st['length']:.2f} m")
        if "on" in st:
            parts.append(f"정책 ON {st['on']:.0%}")
        lines.append(f"{label}: " + " · ".join(parts))
    fig.text(0.01, 0.02, "\n".join(lines), fontsize=8.5, va="bottom")
    fig.savefig(out, dpi=140)
    print(f"저장: {out}")
    for ln in lines:
        print(ln)


def main() -> int:
    ap = argparse.ArgumentParser(description="rl_nav_controller rosbag2 A/B report")
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--labels", nargs="*")
    ap.add_argument("--out", default="rl_nav_controller_report.png")
    a = ap.parse_args()
    labels = a.labels or (["정책", "DWB"] if len(a.bags) == 2 else [Path(b).name for b in a.bags])
    runs = []
    for i, bag in enumerate(a.bags):
        if not Path(bag).exists():
            print(f"FAIL: bag 없음: {bag}", file=sys.stderr)
            return 2
        data = read_bag(bag)
        label = labels[i] if i < len(labels) else Path(bag).name
        st = stats(bag, data)
        runs.append((label, data, st))
        print(f"[{label}] " + " · ".join(f"{k}={v:.4g}" for k, v in st.items()))
    plot(runs, Path(a.out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
