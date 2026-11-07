#!/usr/bin/env python3
"""rl_nav_controller — A/B 리포트 (leg report.png + compare.png).

  사용법:
    report.py --export <bagdir>              # bag → raw.csv + plan.csv (leg 폴더 안에)
    report.py <legdir> [--label A1] [--out report.png]          # leg 리포트
    report.py <legA> <legB> --labels A1 B1 --out compare.png   # A/B 비교 리포트

  bag 디렉터리를 직접 줘도 된다 — raw.csv가 없으면 먼저 export한다.
  raw.csv 열: t,cmd_v,cmd_w,act_v,act_w,odom_v,odom_w,odom_x,odom_y,odom_yaw,
              a0,a1,scan_min,dbg_v_odom,dbg_w_odom,on (20Hz 공통 격자, 빈칸=NaN)
  plan.csv 열: x,y (웨이포인트)
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
TOOL = HERE.parents[4]  # .../tool (ab/rl_nav_controller에서 4 위: ab·pages·op·cli)
sys.path.insert(0, str(TOOL / "node"))
from common.bagio import plan_points, project_to_polyline, read_bag, wrap, yaw_of  # noqa: E402

FONTS = ["/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
         "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"]
DBG = "/controller_server/FollowPath/debug"
DT = 0.05  # raw.csv 공통 격자 [s] = 20Hz

A_COLOR = "#1e88e5"  # 파랑 = A (정책 ON)
B_COLOR = "#e53935"  # 빨강 = B (inner DWB)


def _mpl():
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
    return plt


def _series(data: dict, topic: str, fn):
    msgs = data.get(topic) or []
    if not msgs:
        return np.array([]), np.array([])
    t = np.array([s for s, _ in msgs], float)
    v = np.array([fn(m) for _, m in msgs], float)
    return t, v


def _interp(t_grid, t, v):
    out = np.full_like(t_grid, np.nan, dtype=float)
    if len(t) < 1:
        return out
    lo, hi = t[0], t[-1]
    m = (t_grid >= lo) & (t_grid <= hi)
    if not np.any(m):
        return out
    if len(t) == 1:
        out[m] = v[0]
        return out
    out[m] = np.interp(t_grid[m], t, v)
    return out


def export_leg(bagdir: str) -> tuple[Path, Path]:
    """bag → legdir/raw.csv + plan.csv. 반환 (raw, plan)."""
    leg = Path(bagdir)
    data = read_bag(str(leg))
    od = data.get("/odometry/filtered") or data.get("/odom") or []
    t0 = t1 = None
    for topic in ("/cmd_vel_nav", "/cmd_vel", "/odometry/filtered", "/odom", DBG):
        for s, _ in (data.get(topic) or []):
            t0 = s if t0 is None else min(t0, s)
            t1 = s if t1 is None else max(t1, s)
    if t0 is None:
        raise RuntimeError(f"빈 bag: {leg}")
    grid = np.arange(t0, t1 + DT / 2, DT)

    def col(topic, fn):
        t, v = _series(data, topic, fn)
        return _interp(grid, t, v)

    cmd_v = col("/cmd_vel_nav", lambda m: m.twist.linear.x)
    cmd_w = col("/cmd_vel_nav", lambda m: m.twist.angular.z)
    act_v = col("/cmd_vel", lambda m: m.twist.linear.x)
    act_w = col("/cmd_vel", lambda m: m.twist.angular.z)
    odom_v = col("/odometry/filtered", lambda m: m.twist.twist.linear.x)
    if np.all(np.isnan(odom_v)):
        odom_v = col("/odom", lambda m: m.twist.twist.linear.x)
    odom_w = col("/odometry/filtered", lambda m: m.twist.twist.angular.z)
    if np.all(np.isnan(odom_w)):
        odom_w = col("/odom", lambda m: m.twist.twist.angular.z)
    ox = col("/odometry/filtered", lambda m: m.pose.pose.position.x)
    if np.all(np.isnan(ox)):
        ox = col("/odom", lambda m: m.pose.pose.position.x)
    oy = col("/odometry/filtered", lambda m: m.pose.pose.position.y)
    if np.all(np.isnan(oy)):
        oy = col("/odom", lambda m: m.pose.pose.position.y)
    oyaw = col("/odometry/filtered", lambda m: yaw_of(m.pose.pose.orientation))
    if np.all(np.isnan(oyaw)):
        oyaw = col("/odom", lambda m: yaw_of(m.pose.pose.orientation))
    a0 = col(DBG, lambda m: m.data[0] if len(m.data) > 0 else np.nan)
    a1 = col(DBG, lambda m: m.data[1] if len(m.data) > 1 else np.nan)
    scan = col(DBG, lambda m: m.data[4] if len(m.data) > 4 else np.nan)
    dv = col(DBG, lambda m: m.data[5] if len(m.data) > 5 else np.nan)
    dw = col(DBG, lambda m: m.data[6] if len(m.data) > 6 else np.nan)
    on = col(DBG, lambda m: m.data[7] if len(m.data) > 7 else np.nan)

    header = "t,cmd_v,cmd_w,act_v,act_w,odom_v,odom_w,odom_x,odom_y,odom_yaw,a0,a1,scan_min,dbg_v_odom,dbg_w_odom,on"
    mat = np.column_stack([grid - grid[0], cmd_v, cmd_w, act_v, act_w, odom_v, odom_w,
                           ox, oy, oyaw, a0, a1, scan, dv, dw, on])
    raw = leg / "raw.csv"
    np.savetxt(raw, mat, delimiter=",", header=header, comments="", fmt="%.6f")
    poly = plan_points(data.get("/plan", []))
    plan = leg / "plan.csv"
    if len(poly) >= 1:
        np.savetxt(plan, np.asarray(poly, float), delimiter=",", header="x,y", comments="", fmt="%.6f")
    print(f"export: {raw} ({len(grid)}행) + {plan if plan.exists() else '(plan 없음)'}")
    return raw, plan


def load_run(path: str) -> dict:
    """leg 폴더(raw.csv 우선) 또는 bag → numpy dict."""
    leg = Path(path)
    raw = leg / "raw.csv"
    if not raw.is_file():
        # bag 직접 지정 — export 후 읽기
        export_leg(str(leg))
    m = np.genfromtxt(raw, delimiter=",", names=True)
    if m.ndim == 0:
        m = m.reshape(1)
    d: dict = {k: np.atleast_1d(m[k].astype(float)) for k in m.dtype.names}
    plan = leg / "plan.csv"
    d["plan_xy"] = np.zeros((0, 2))
    if plan.is_file():
        try:
            p = np.genfromtxt(plan, delimiter=",", names=True)
            if p.ndim == 0:
                d["plan_xy"] = np.array([[p["x"], p["y"]]])
            elif p.ndim == 1 and "x" in p.dtype.names:
                d["plan_xy"] = np.column_stack([p["x"], p["y"]])
        except Exception:
            pass
    d["leg"] = leg.name
    return d


def _finite(*arrs):
    m = np.ones_like(arrs[0], dtype=bool)
    for a in arrs:
        m &= np.isfinite(a)
    return m


def stats_run(d: dict) -> dict:
    res: dict = {}
    t, x, y = d["t"], d["odom_x"], d["odom_y"]
    m = _finite(t, x, y)
    if np.sum(m) >= 2:
        res["duration"] = float(t[m][-1] - t[m][0])
        res["length"] = float(np.sum(np.hypot(np.diff(x[m]), np.diff(y[m]))))
    for k, nk in (("cmd_v", "cmd_v_mean"), ("act_v", "act_v_mean"), ("odom_v", "odom_v_mean"),
                   ("a0", "a0_mean"), ("a1", "a1_mean"), ("on", "on"), ("scan_min", "scan_min")):
        v = d.get(k, np.array([np.nan]))
        v = v[np.isfinite(v)]
        if len(v):
            res[nk] = float(np.mean(v))
    # 명령 적산 vs 실제 (유효 cmd 구간)
    cv, cw = d["cmd_v"], d["cmd_w"]
    m2 = _finite(d["t"], cv, cw)
    if np.sum(m2) >= 2:
        tt = d["t"][m2]
        dt = np.clip(np.diff(tt, prepend=tt[0]), 0.0, 1.0)
        yaw0 = float(d["odom_yaw"][m][0]) if np.sum(m) else 0.0
        yaw = yaw0 + np.cumsum(cw[m2] * dt)
        res["cmd_length"] = float(np.sum(np.abs(cv[m2] * dt)))
        res["cmd_end"] = (float(x[m][0] + np.sum(cv[m2] * np.cos(yaw) * dt)),
                          float(y[m][0] + np.sum(cv[m2] * np.sin(yaw) * dt)))
    poly = d.get("plan_xy", np.zeros((0, 2)))
    if len(poly) >= 2 and np.sum(m) >= 2:
        xy = np.column_stack([x[m], y[m]])
        dist, s, head = project_to_polyline(xy, np.asarray(poly, float))
        yawm = d["odom_yaw"][m]
        res["lat_rmse"] = float(np.sqrt(np.mean(dist ** 2)))
        res["lat_max"] = float(np.max(np.abs(dist)))
        res["head_mae"] = float(np.mean(np.abs(wrap(yawm - head))))
        res["progress"] = float(s[-1]) if len(s) else 0.0
    return res


def plot_single(d: dict, st: dict, label: str, color: str, out: Path, title: str) -> None:
    plt = _mpl()
    fig, axes = plt.subplots(2, 2, figsize=(13, 9.0))
    fig.subplots_adjust(left=0.06, right=0.98, top=0.90, bottom=0.18, hspace=0.32, wspace=0.22)
    fig.suptitle(title, fontsize=14, fontweight="bold", color=color)
    t = d["t"]

    ax = axes[0, 0]
    for k, ls, lw, lab in (("cmd_v", "-", 1.4, "명령(cmd_vel_nav)"), ("act_v", ":", 1.0, "실제(cmd_vel)"),
                           ("odom_v", "--", 0.9, "오돔")):
        v = d.get(k, np.array([]))
        m = _finite(t, v)
        if np.any(m):
            ax.plot(t[m] - t[m][0], v[m], color, lw=lw, ls=ls, alpha=0.9, label=lab)
    ax.set_title(f"{label} 선속도 [m/s] — 명령 vs 실제 vs 오돔")
    ax.set_xlabel("time [s]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[0, 1]
    m = _finite(t, d["odom_x"], d["odom_y"])
    if np.any(m):
        ax.plot(d["odom_x"][m], d["odom_y"][m], color, lw=2.0, label=f"{label} 실제(오돔)")
    poly = d.get("plan_xy", np.zeros((0, 2)))
    if len(poly) >= 2:
        ax.plot(poly[:, 0], poly[:, 1], "k--", lw=1.2, alpha=0.7, label="웨이포인트(plan)")
        ax.scatter(poly[:, 0], poly[:, 1], c="k", s=18, zorder=5)
    if "cmd_end" in st and np.any(m):
        ax.plot([d["odom_x"][m][0], st["cmd_end"][0]], [d["odom_y"][m][0], st["cmd_end"][1]],
                color, lw=1.0, ls=":", alpha=0.8, label="명령 적산")
    ax.set_title(f"{label} XY 궤적 [m] — 실제 vs 웨이포인트 vs 명령적산")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[1, 0]
    if "lat_rmse" in st and np.any(m):
        xy = np.column_stack([d["odom_x"][m], d["odom_y"][m]])
        dist, s, _ = project_to_polyline(xy, np.asarray(poly, float))
        ax.plot(s, dist * 100.0, color, lw=1.5)
        ax.axhline(0, c="k", lw=0.8, alpha=0.5)
    ax.set_title("경로 횡오차 [cm]")
    ax.set_xlabel("진행 거리 [m]")
    ax.grid(alpha=0.3)

    ax = axes[1, 1]
    has_dbg = False
    for k, ls, lab in (("a0", "-", "a0 (v)"), ("a1", ":", "a1 (ω)")):
        v = d.get(k, np.array([]))
        mm = _finite(t, v)
        if np.any(mm):
            ax.plot(t[mm] - t[mm][0], v[mm], color, lw=1.3, ls=ls, label=f"{label} {lab}")
            has_dbg = True
    ax.set_title("정책 행동 [-1,1]" + ("" if has_dbg else " (디버그 없음 — DWB)"))
    ax.set_xlabel("time [s]")
    ax.grid(alpha=0.3)
    if has_dbg:
        ax.legend(fontsize=8)

    lines = [f"{label}: " + " · ".join(
        f"{k}={v:.4g}" for k, v in st.items() if isinstance(v, float))]
    fig.text(0.01, 0.02, "\n".join(lines), fontsize=9, va="bottom")
    fig.savefig(out, dpi=140)
    print(f"저장: {out}")
    for ln in lines:
        print(ln)
    plt.close(fig)


def plot_compare(runs, out: Path) -> None:
    """runs = [(label, color, d, st)]. A/B 한눈에 — 큰 라벨·요약표."""
    plt = _mpl()
    fig, axes = plt.subplots(2, 2, figsize=(14, 9.5))
    fig.subplots_adjust(left=0.06, right=0.98, top=0.88, bottom=0.24, hspace=0.35, wspace=0.25)
    fig.suptitle("rl_nav_controller — A(정책 ON) vs B(DWB) 비교", fontsize=15, fontweight="bold")

    ax = axes[0, 0]
    for label, color, d, _st in runs:
        m = _finite(d["t"], d["cmd_v"])
        if np.any(m):
            ax.plot(d["t"][m] - d["t"][m][0], d["cmd_v"][m], color, lw=1.6, label=f"{label} 명령")
        m2 = _finite(d["t"], d["act_v"])
        if np.any(m2):
            ax.plot(d["t"][m2] - d["t"][m2][0], d["act_v"][m2], color, lw=1.0, ls=":", alpha=0.85,
                    label=f"{label} 실제")
    ax.set_title("A/B 선속도 [m/s] — 명령(실선) vs 실제(점선)")
    ax.set_xlabel("time [s]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9, loc="upper right")

    ax = axes[0, 1]
    for label, color, d, _st in runs:
        m = _finite(d["t"], d["odom_x"], d["odom_y"])
        if np.any(m):
            ax.plot(d["odom_x"][m], d["odom_y"][m], color, lw=2.2, label=f"{label} 실제")
    poly = runs[0][2].get("plan_xy", np.zeros((0, 2)))
    if len(poly) < 2 and len(runs) > 1:
        poly = runs[1][2].get("plan_xy", np.zeros((0, 2)))
    if len(poly) >= 2:
        ax.plot(poly[:, 0], poly[:, 1], "k--", lw=1.2, alpha=0.7, label="웨이포인트")
        ax.scatter(poly[:, 0], poly[:, 1], c="k", s=22, zorder=5)
    ax.set_title("A/B XY 궤적 [m] — 파랑=A 실제, 빨강=B 실제")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)

    ax = axes[1, 0]
    for label, color, d, _st in runs:
        m = _finite(d["t"], d["odom_x"], d["odom_y"])
        pp = d.get("plan_xy", np.zeros((0, 2)))
        if np.any(m) and len(pp) >= 2:
            xy = np.column_stack([d["odom_x"][m], d["odom_y"][m]])
            dist, s, _ = project_to_polyline(xy, np.asarray(pp, float))
            ax.plot(s, dist * 100.0, color, lw=1.6, label=label)
    ax.axhline(0, c="k", lw=0.8, alpha=0.5)
    ax.set_title("A/B 경로 횡오차 [cm]")
    ax.set_xlabel("진행 거리 [m]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)

    ax = axes[1, 1]
    ax.axis("off")
    rows = []
    for label, _c, _d, st in runs:
        rows.append([label,
                     f"{st.get('duration', float('nan')):.1f}s",
                     f"{st.get('length', float('nan')):.2f} m",
                     f"{st.get('cmd_length', float('nan')):.2f} m",
                     f"{st.get('lat_rmse', float('nan')) * 100:.1f} cm" if "lat_rmse" in st else "-",
                     f"{st.get('on', float('nan')):.0%}" if "on" in st else "- (DWB)"])
    tab = ax.table(cellText=rows,
                   colLabels=["leg", "시간", "실제 이동", "명령 적산", "횡오차 RMSE", "정책 ON"],
                   loc="center", colWidths=[0.12, 0.14, 0.2, 0.2, 0.2, 0.14])
    tab.auto_set_font_size(False)
    tab.set_fontsize(10)
    for (r, c), cell in tab.get_celld().items():
        if r == 0:
            cell.set_facecolor("#eeeeee")
        elif c == 0:
            cell.set_facecolor(runs[r - 1][1])
            cell.get_text().set_color("white")
            cell.get_text().set_fontweight("bold")
    ax.set_title("A/B 요약", pad=12)

    fig.savefig(out, dpi=140)
    print(f"저장: {out}")
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description="rl_nav_controller A/B 리포트")
    ap.add_argument("paths", nargs="*", help="bag/leg 폴더 (1개=leg 리포트, 2개=비교)")
    ap.add_argument("--labels", nargs="*")
    ap.add_argument("--out", default=None)
    ap.add_argument("--export", default=None, help="bag 폴더 → raw.csv + plan.csv만 만들고 끝")
    a = ap.parse_args()
    if a.export:
        export_leg(a.export)
        return 0
    if not a.paths:
        print("FAIL: bag/leg 폴더를 주세요", file=sys.stderr)
        return 2
    if len(a.paths) == 1:
        d = load_run(a.paths[0])
        st = stats_run(d)
        label = (a.labels[0] if a.labels else d.get("leg", "run"))
        color = A_COLOR if label.upper().startswith("A") else B_COLOR
        out = Path(a.out) if a.out else Path(a.paths[0]) / "report.png"
        plot_single(d, st, label, color, out, f"rl_nav_controller — {label} leg 리포트")
        return 0
    runs = []
    for i, p in enumerate(a.paths[:2]):
        d = load_run(p)
        st = stats_run(d)
        label = a.labels[i] if a.labels and i < len(a.labels) else d.get("leg", f"run{i}")
        color = (A_COLOR, B_COLOR)[i % 2]
        runs.append((label, color, d, st))
        print(f"[{label}] " + " · ".join(f"{k}={v:.4g}" for k, v in st.items() if isinstance(v, float)))
    out = Path(a.out) if a.out else Path(a.paths[0]).parent / "compare.png"
    plot_compare(runs, out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
