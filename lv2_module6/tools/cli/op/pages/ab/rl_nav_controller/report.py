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
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))   # ab/ (bagio.py)
from bagio import plan_points, project_to_polyline, read_bag, wrap, yaw_of  # noqa: E402

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


def _start_plan(msgs, route_csv: Path) -> np.ndarray:
    """route 구간마다 Nav2가 그 구간 출발점에서 만든 첫 전역 경로를 이어 붙인다 (출발→G1→G2…).
    로봇이 밀려난 뒤 다시 짠 경로(마지막 /plan)를 기준으로 쓰면 진행·횡오차가 틀린다.
    구간 i = route[i]에서 0.25 m 안에서 시작하고 route[i+1] 근처에서 끝나는 가장 이른 /plan. route가 없으면 첫 /plan."""
    paths = [np.array([[p.pose.position.x, p.pose.position.y] for p in m.poses], float) for _t, m in msgs]
    paths = [q for q in paths if len(q) >= 2]
    if not paths:
        return np.zeros((0, 2))
    try:
        route = np.atleast_2d(np.genfromtxt(route_csv, delimiter=",", skip_header=1))[:, :2]
    except Exception:
        return paths[0]
    out, k = [], 0
    for i in range(len(route) - 1):
        for j in range(k, len(paths)):
            q = paths[j]
            if np.hypot(*(q[0] - route[i])) <= 0.25 and np.hypot(*(q[-1] - route[i + 1])) <= 0.3:
                out.append(q)
                k = j + 1
                break
        else:
            break  # 그 구간 경로가 없음 = 앞 goal에서 실패 — 거기까지만
    return np.vstack(out) if out else paths[0]


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
    nav = _start_plan(data.get("/plan", []), leg / "plan.csv")
    if len(nav) >= 2:  # Nav2가 출발점에서 만든 전역 경로 — 횡오차·진행 기준 · 리포트 점선
        np.savetxt(leg / "nav_plan.csv", nav, delimiter=",", header="x,y", comments="", fmt="%.4f")
    plan = leg / "plan.csv"
    if plan.is_file():
        poly = []  # guide가 쓴 route(출발 + goal)가 정본 — bag의 Nav2 /plan(촘촘한 경로)으로 덮지 않는다
    if len(poly) >= 1:
        np.savetxt(plan, np.asarray(poly, float), delimiter=",", header="x,y", comments="", fmt="%.6f")
    print(f"export: {raw} ({len(grid)}행) + {plan if plan.exists() else '(plan 없음)'}")
    return raw, plan


def _load_map(folder: Path):
    """그룹 폴더의 map.pgm·map.yaml (guide가 /map을 저장) → (img[행 0 = 위], origin xy, res) 또는 None."""
    pgm, yml = folder / "map.pgm", folder / "map.yaml"
    if not (pgm.is_file() and yml.is_file()):
        return None
    try:
        raw = pgm.read_bytes()
        tok, pos = [], 0
        while len(tok) < 4:  # P5 · 폭 · 높이 · 최댓값 (주석 줄 건너뜀)
            while raw[pos:pos + 1].isspace():
                pos += 1
            if raw[pos:pos + 1] == b"#":
                pos = raw.index(b"\n", pos) + 1
                continue
            end = pos
            while not raw[end:end + 1].isspace():
                end += 1
            tok.append(raw[pos:end].decode())
            pos = end
        w, h = int(tok[1]), int(tok[2])
        img = np.frombuffer(raw[pos + 1:pos + 1 + w * h], np.uint8).reshape(h, w)
        meta = {}
        for line in yml.read_text().splitlines():
            if ":" in line and not line.lstrip().startswith("#"):
                k, v = line.split(":", 1)
                meta[k.strip()] = v.strip()
        org = [float(v) for v in meta["origin"].strip("[]").split(",")[:2]]
        return img, (org[0], org[1]), float(meta["resolution"])
    except Exception as exc:
        print(f"WARN: 지도 읽기 실패 ({exc}) — 지도 없이 그림", file=sys.stderr)
        return None


def load_run(path: str) -> dict:
    """leg 폴더(raw.csv 우선) 또는 bag → numpy dict. pose.csv(map 기준 자세)·result.json·그룹 지도도 같이."""
    leg = Path(path)
    raw = leg / "raw.csv"
    cols = ("t,cmd_v,cmd_w,act_v,act_w,odom_v,odom_w,odom_x,odom_y,odom_yaw,"
            "a0,a1,scan_min,dbg_v_odom,dbg_w_odom,on").split(",")
    d: dict = {k: np.array([np.nan]) for k in cols}  # bag이 비어도(바로 끝난 leg) 리포트는 나오게
    try:
        if not raw.is_file():
            export_leg(str(leg))  # bag 직접 지정 — export 후 읽기
        m = np.genfromtxt(raw, delimiter=",", names=True)
        if m.ndim == 0:
            m = m.reshape(1)
        d.update({k: np.atleast_1d(m[k].astype(float)) for k in m.dtype.names})
    except Exception as exc:
        print(f"WARN: {leg.name} 토픽 기록 없음 ({exc})", file=sys.stderr)
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
    # 궤적: map 기준 pose.csv(guide가 TF로 기록) 우선 — plan.csv와 같은 좌표계.
    #   odom은 시뮬 텔레포트·SLAM 보정으로 map과 어긋나 웨이포인트와 겹쳐 그리면 틀린다.
    d["xy_frame"] = "odom"
    d["xy"] = np.column_stack([d["odom_x"], d["odom_y"]])
    d["xy_yaw"] = d["odom_yaw"]
    pose = leg / "pose.csv"
    if pose.is_file():
        try:
            q = np.atleast_2d(np.genfromtxt(pose, delimiter=",", skip_header=1))
            if q.shape[0] >= 2 and q.shape[1] >= 4:
                d["xy"], d["xy_yaw"], d["xy_t"], d["xy_frame"] = q[:, 1:3], q[:, 3], q[:, 0], "map"
        except Exception:
            pass
    d["nav_xy"] = np.zeros((0, 2))
    nav = leg / "nav_plan.csv"
    if nav.is_file():
        try:
            d["nav_xy"] = np.atleast_2d(np.genfromtxt(nav, delimiter=",", skip_header=1))[:, :2]
        except Exception:
            pass
    d["leg"] = leg.name
    d["result"] = {}
    res = leg / "result.json"  # guide가 남긴 leg 판정 (도착·모드 검증) — 수동 녹화엔 없음
    if res.is_file():
        try:
            d["result"] = json.loads(res.read_text())
        except Exception:
            pass
    d["map"] = _load_map(leg.parent)
    return d


def _finite(*arrs):
    m = np.ones_like(arrs[0], dtype=bool)
    for a in arrs:
        m &= np.isfinite(a)
    return m


def _wall_clearance(xy: np.ndarray, mp) -> float | None:
    """궤적 점마다 가장 가까운 벽 칸까지 거리의 최솟값 [m] (지도 있을 때만)."""
    if mp is None or len(xy) == 0:
        return None
    img, (ox, oy), res = mp
    rr, cc = np.nonzero(img < 50)  # 점유 칸
    if len(rr) == 0:
        return None
    h = img.shape[0]
    walls = np.column_stack([ox + (cc + 0.5) * res, oy + (h - 1 - rr + 0.5) * res])
    best = np.inf
    for i in range(0, len(xy), max(1, len(xy) // 400)):
        best = min(best, float(np.min(np.hypot(walls[:, 0] - xy[i, 0], walls[:, 1] - xy[i, 1]))))
    return max(0.0, best - res / 2)  # 칸 중심 → 칸 가장자리


def _ref_path(d: dict) -> np.ndarray:
    """횡오차·진행 기준 경로 — Nav2가 실제로 만든 전역 경로(nav_plan.csv), 없으면 route 직선(plan.csv)."""
    nav = np.asarray(d.get("nav_xy", np.zeros((0, 2))), float)
    return nav if len(nav) >= 2 else np.asarray(d.get("plan_xy", np.zeros((0, 2))), float)


def stats_run(d: dict) -> dict:
    res: dict = {}
    xy = d["xy"]
    mk = _finite(xy[:, 0], xy[:, 1]) if len(xy) else np.zeros(0, bool)
    xy = xy[mk]
    t = d["t"]
    mt = _finite(t)
    if np.sum(mt) >= 2:
        res["duration"] = float(t[mt][-1] - t[mt][0])
    if len(xy) >= 2:
        res["length"] = float(np.sum(np.hypot(*np.diff(xy, axis=0).T)))
    for k, nk in (("cmd_v", "cmd_v_mean"), ("act_v", "act_v_mean"), ("odom_v", "odom_v_mean"),
                   ("a0", "a0_mean"), ("a1", "a1_mean"), ("on", "on"), ("scan_min", "scan_min")):
        v = d.get(k, np.array([np.nan]))
        v = v[np.isfinite(v)]
        if len(v):
            res[nk] = float(np.mean(v))
    v = d.get("odom_v", np.array([np.nan]))
    v = v[np.isfinite(v)]
    if len(v):
        res["reverse_ratio"] = float(np.mean(v < -0.01))  # 후진한 시간 비율
    cv = d["cmd_v"]
    m2 = _finite(d["t"], cv)
    if np.sum(m2) >= 2:
        tt = d["t"][m2]
        dt = np.clip(np.diff(tt, prepend=tt[0]), 0.0, 1.0)
        res["cmd_length"] = float(np.sum(np.abs(cv[m2] * dt)))
    poly = np.asarray(d.get("plan_xy", np.zeros((0, 2))), float)
    ref = _ref_path(d)
    if len(poly) >= 2 and len(xy) >= 2:
        dist, s, head = project_to_polyline(xy, ref)
        res["lat_rmse"] = float(np.sqrt(np.mean(dist ** 2)))
        res["lat_max"] = float(np.max(np.abs(dist)))
        res["progress"] = float(s[-1]) if len(s) else 0.0
        res["path_len"] = float(np.sum(np.hypot(*np.diff(ref, axis=0).T)))
        res["goal_err"] = float(np.hypot(*(xy[-1] - poly[-1])))
    clr = _wall_clearance(xy, d.get("map"))
    if clr is not None:
        res["clearance"] = clr
    return res


# ---------------------------------------------------------------- 그림 (A/B 비교 · leg 하나 공통)
INK, MUTED, GRID = "#263238", "#78909c", "#eceff1"
OK_C, FAIL_C, BAD_C = "#2e7d32", "#ef6c00", "#757575"


def _style(ax, title=None):
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color("#b0bec5")
    ax.tick_params(colors=MUTED, labelsize=8.5)
    ax.grid(color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    if title:
        ax.set_title(title, loc="left", fontsize=11, fontweight="bold", color=INK, pad=8)


def _verdict(d: dict) -> tuple[str, str]:
    res = d.get("result") or {}
    if not res:
        return "판정 없음 (수동 녹화)", BAD_C
    if res.get("stopped"):
        return "중지됨 — 사용자가 멈춤", BAD_C
    if not res.get("valid", True):
        return "무효 — 비교에서 제외", BAD_C
    return ("도착", OK_C) if res.get("arrived") else ("실패 — 목표 못 감", FAIL_C)


def _draw_map(ax, runs) -> None:
    mp = next((d["map"] for _l, _c, d, _s in runs if d.get("map") is not None), None)
    pts = [d["xy"][_finite(d["xy"][:, 0], d["xy"][:, 1])] for _l, _c, d, _s in runs if len(d["xy"])]
    poly = next((np.asarray(d["plan_xy"], float) for _l, _c, d, _s in runs if len(d.get("plan_xy", [])) >= 1),
                np.zeros((0, 2)))
    allp = np.vstack([p for p in pts if len(p)] + ([poly] if len(poly) else [])) if (pts or len(poly)) else np.zeros((0, 2))
    if mp is not None:
        img, (ox, oy), res = mp
        h, w = img.shape
        rgb = np.empty((h, w, 3))
        rgb[img >= 250] = (0.98, 0.98, 0.98)                     # 빈 곳
        rgb[(img > 50) & (img < 250)] = (0.88, 0.90, 0.91)      # 모름
        rgb[img <= 50] = (0.22, 0.28, 0.31)                     # 벽
        ax.imshow(rgb, extent=(ox, ox + w * res, oy, oy + h * res), origin="upper", interpolation="nearest", zorder=0)
    navs = [(label, color, np.asarray(d["nav_xy"], float)) for label, color, d, _s in runs if len(d.get("nav_xy", [])) >= 2]
    for label, color, nav in navs:  # leg마다 Nav2가 그 leg에서 만든 경로 (구간별로 이어 붙임) — leg 색 점선 + 경로 점
        ax.plot(nav[:, 0], nav[:, 1], color=color, lw=1.1, ls=(0, (4, 3)), alpha=0.55, zorder=2,
                label=f"{label} Nav2 전역 경로 (웨이포인트의 출처)")
        ax.scatter(nav[:, 0], nav[:, 1], s=7, color=color, alpha=0.6, lw=0, zorder=2.5)
        allp = np.vstack([allp, nav]) if len(allp) else nav
    if not navs and len(poly) >= 2:
        ax.plot(poly[:, 0], poly[:, 1], color=INK, lw=1.4, ls=(0, (4, 3)), alpha=0.75, zorder=2, label="출발→목표 (직선)")
    for label, color, d, _st in runs:
        xy = d["xy"][_finite(d["xy"][:, 0], d["xy"][:, 1])]
        if len(xy) < 2:
            continue
        ax.plot(xy[:, 0], xy[:, 1], color=color, lw=3.0, alpha=0.9, solid_capstyle="round", zorder=3, label=f"{label} 실제")
        step = max(1, len(xy) // 6)  # 진행 방향 화살표
        for i in range(step, len(xy) - 1, step):
            dx, dy = xy[i + 1] - xy[i]
            if math.hypot(dx, dy) > 1e-4:
                ax.annotate("", xy=xy[i + 1], xytext=xy[i], zorder=4,
                            arrowprops=dict(arrowstyle="-|>", color=color, lw=0, mutation_scale=13))
        ax.scatter(*xy[-1], s=70, color=color, edgecolor="white", lw=1.5, zorder=5)
        ax.annotate(f" {label} 끝", xy[-1], color=color, fontsize=9, fontweight="bold", zorder=6,
                    xytext=(4, 4), textcoords="offset points")
    if len(poly):
        ax.scatter(*poly[0], marker="^", s=130, color=OK_C, edgecolor="white", lw=1.5, zorder=6, label="출발")
        ax.scatter(poly[1:, 0], poly[1:, 1], marker="*", s=260, color="#fbc02d", edgecolor=INK, lw=0.8, zorder=6,
                   label="목표 (route)")
        for i, (gx, gy) in enumerate(poly[1:], 1):
            ax.annotate(f"G{i}", (gx, gy), xytext=(7, -13), textcoords="offset points", fontsize=10,
                        fontweight="bold", color=INK, zorder=7,
                        bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="#fbc02d", lw=1))
    if len(allp):  # 경로 주변으로 확대 (최소 2.4 m 폭)
        lo, hi = allp.min(0), allp.max(0)
        c, half = (lo + hi) / 2, max(1.2, float(np.max(hi - lo)) / 2 + 0.6)
        ax.set_xlim(c[0] - half, c[0] + half)
        ax.set_ylim(c[1] - half, c[1] + half)
    ax.set_aspect("equal")
    frame = runs[0][2].get("xy_frame", "odom")
    _style(ax, "지도 위 궤적" + ("" if mp is not None else " (지도 없음)") + ("" if frame == "map" else " · odom 좌표(지도와 어긋날 수 있음)"))
    ax.set_xlabel("x [m]", color=MUTED, fontsize=9)
    ax.set_ylabel("y [m]", color=MUTED, fontsize=9)
    ax.legend(loc="best", fontsize=8.5, framealpha=0.92, edgecolor="#cfd8dc")


def _time_panel(ax, runs, key_cmd, key_act, title, unit):
    for label, color, d, _st in runs:
        t = d["t"]
        m = _finite(t, d[key_cmd])
        if np.any(m):
            ax.plot(t[m] - t[m][0], d[key_cmd][m], color=color, lw=1.8, label=f"{label} 명령")
        m2 = _finite(t, d[key_act])
        if np.any(m2):
            ax.plot(t[m2] - t[m2][0], d[key_act][m2], color=color, lw=1.1, ls=":", alpha=0.9, label=f"{label} 실제")
    ax.axhline(0, color="#b0bec5", lw=0.8)
    _style(ax, title)
    ax.set_xlabel("시간 [s]", color=MUTED, fontsize=9)
    ax.set_ylabel(unit, color=MUTED, fontsize=9)
    ax.legend(fontsize=7.5, ncol=2, framealpha=0.9, edgecolor="#cfd8dc")


def _fmt(v, f, scale=1.0):
    return "-" if v is None or (isinstance(v, float) and not math.isfinite(v)) else f.format(v * scale)


def _kpi_table(ax, runs) -> None:
    ax.axis("off")
    rows = [  # (이름, 값 함수, 작을수록 좋음?)
        ("판정", lambda d, st: _verdict(d)[0], None),
        ("정책 실행", lambda d, st: _fmt((d.get("result") or {}).get("rl_ratio", st.get("on")), "{:.0%}"), None),
        ("도착한 목표", lambda d, st: (f"{d['result']['goals_reached']} / {d['result']['goals_total']}"
                                  if "goals_total" in (d.get("result") or {}) else "-"), None),
        ("걸린 시간", lambda d, st: _fmt(st.get("duration"), "{:.1f} s"), "duration"),
        ("주행 거리", lambda d, st: _fmt(st.get("length"), "{:.2f} m"), None),
        ("경로 진행", lambda d, st: (_fmt(st.get("progress"), "{:.2f}") + " / " + _fmt(st.get("path_len"), "{:.2f} m"))
         if "progress" in st else "-", None),
        ("도착 오차", lambda d, st: _fmt(st.get("goal_err"), "{:.0f} cm", 100), "goal_err"),
        ("횡오차 RMSE", lambda d, st: _fmt(st.get("lat_rmse"), "{:.1f} cm", 100), "lat_rmse"),
        ("횡오차 최대", lambda d, st: _fmt(st.get("lat_max"), "{:.1f} cm", 100), "lat_max"),
        ("벽 최소 거리", lambda d, st: _fmt(st.get("clearance"), "{:.0f} cm", 100), None),
        ("후진 시간", lambda d, st: _fmt(st.get("reverse_ratio"), "{:.0%}"), "reverse_ratio"),
    ]
    cells, colors = [], []
    valid_all = all((d.get("result") or {}).get("valid", True) and (d.get("result") or {}).get("arrived", True)
                    for _l, _c, d, _s in runs)
    for name, fn, better_key in rows:
        row = [name] + [fn(d, st) for _l, _c, d, st in runs]
        col = ["#f5f7f8"] + ["white"] * len(runs)
        if name == "판정":
            col = ["#f5f7f8"] + [_verdict(d)[1] for _l, _c, d, _s in runs]
        elif better_key and len(runs) == 2 and valid_all:  # 둘 다 유효·도착일 때만 더 나은 쪽 강조
            vals = [st.get(better_key) for _l, _c, _d, st in runs]
            if all(v is not None for v in vals) and vals[0] != vals[1]:
                col[1 + int(np.argmin(vals))] = "#e8f5e9"
        cells.append(row)
        colors.append(col)
    tab = ax.table(cellText=cells, cellColours=colors, colLabels=["지표"] + [l for l, _c, _d, _s in runs],
                   loc="upper center", cellLoc="center", colWidths=[0.30] + [0.70 / len(runs)] * len(runs))
    tab.auto_set_font_size(False)
    tab.set_fontsize(9.5)
    tab.scale(1, 1.32)
    for (r, c), cell in tab.get_celld().items():
        cell.set_edgecolor("#e0e6e9")
        if r == 0:
            cell.set_facecolor(INK if c == 0 else runs[c - 1][1])
            cell.get_text().set_color("white")
            cell.get_text().set_fontweight("bold")
        elif c == 0:
            cell.get_text().set_color(MUTED)
            cell.get_text().set_ha("left")
        elif r == 1:
            cell.get_text().set_color("white")
            cell.get_text().set_fontweight("bold")
    _style(ax, "핵심 지표")


def plot_report(runs, out: Path, title: str, subtitle: str = "") -> None:
    """runs = [(label, color, d, st)] — 1개면 leg 리포트, 2개면 A/B 비교. 지도·속도·각속도(또는 정책 행동)·횡오차·지표."""
    plt = _mpl()
    from matplotlib.gridspec import GridSpec
    fig = plt.figure(figsize=(16, 10.4), facecolor="white")
    gs = GridSpec(3, 3, figure=fig, left=0.05, right=0.98, top=0.85, bottom=0.06,
                  hspace=0.42, wspace=0.26, height_ratios=[1, 1, 1.05])
    fig.text(0.05, 0.955, title, fontsize=18, fontweight="bold", color=INK)
    if subtitle:
        fig.text(0.05, 0.925, subtitle, fontsize=10, color=MUTED)
    notes = []  # 무효·실패 이유 — 표만 보고 결과를 믿지 않게 제목 바로 아래
    for label, _c, d, _st in runs:
        res = d.get("result") or {}
        why = res.get("invalid_reason") or res.get("error")
        if why:
            notes.append(f"{label}: {why if len(why) <= 120 else why[:119] + '…'}")
    if notes:
        fig.text(0.05, 0.897, "⚠ " + "   ·   ".join(notes), fontsize=9, color="#b71c1c")
    x = 0.98  # 오른쪽 위: leg별 판정 배지
    for label, color, d, _st in reversed(runs):
        text, vc = _verdict(d)
        t = fig.text(x, 0.945, f"  {label} · {text}  ", ha="right", va="center", fontsize=11, fontweight="bold",
                     color="white", bbox=dict(boxstyle="round,pad=0.45", fc=vc, ec="none"))
        fig.canvas.draw()
        x = t.get_window_extent().transformed(fig.transFigure.inverted()).x0 - 0.016

    _draw_map(fig.add_subplot(gs[0:2, 0:2]), runs)
    _time_panel(fig.add_subplot(gs[0, 2]), runs, "cmd_v", "act_v", "선속도", "m/s")
    ax = fig.add_subplot(gs[1, 2])
    single_policy = len(runs) == 1 and np.any(np.isfinite(runs[0][2].get("a0", np.array([np.nan]))))
    if single_policy:  # leg 하나 + 정책이면 정책 행동 그대로
        label, color, d, _st = runs[0]
        t = d["t"]
        for k, ls, lab in (("a0", "-", "a0 → 선속도"), ("a1", ":", "a1 → 각속도")):
            m = _finite(t, d[k])
            if np.any(m):
                ax.plot(t[m] - t[m][0], d[k][m], color=color, lw=1.6, ls=ls, label=lab)
        ax.axhline(0, color="#b0bec5", lw=0.8)
        ax.set_ylim(-1.05, 1.05)
        _style(ax, "정책 행동 [-1, 1]")
        ax.set_xlabel("시간 [s]", color=MUTED, fontsize=9)
        ax.legend(fontsize=7.5, framealpha=0.9, edgecolor="#cfd8dc")
    else:
        _time_panel(ax, runs, "cmd_w", "act_w", "각속도", "rad/s")

    ax = fig.add_subplot(gs[2, 0])
    drew = False
    for label, color, d, _st in runs:
        xy = d["xy"][_finite(d["xy"][:, 0], d["xy"][:, 1])]
        poly = _ref_path(d)
        if len(xy) >= 2 and len(poly) >= 2:
            dist, s, _ = project_to_polyline(xy, poly)
            ax.plot(s, dist * 100.0, color=color, lw=1.8, label=label)
            drew = True
    ax.axhline(0, color="#b0bec5", lw=0.8)
    _style(ax, "경로 횡오차 (Nav2 전역 경로 기준 · 왼쪽 +)" if any(len(d.get("nav_xy", [])) >= 2 for _l, _c, d, _s in runs)
           else "경로 횡오차 (출발→목표 직선 기준 · 왼쪽 +)")
    ax.set_xlabel("경로 진행 [m]", color=MUTED, fontsize=9)
    ax.set_ylabel("cm", color=MUTED, fontsize=9)
    if drew:
        ax.legend(fontsize=8, framealpha=0.9, edgecolor="#cfd8dc")

    _kpi_table(fig.add_subplot(gs[2, 1:3]), runs)
    fig.savefig(out, dpi=130, facecolor="white")
    print(f"저장: {out}")
    plt.close(fig)


def _subtitle(paths) -> str:
    leg = Path(paths[0])
    plan = load_run(paths[0]).get("plan_xy", np.zeros((0, 2)))
    goals = " → ".join(f"G{i} ({x:.2f}, {y:.2f})" for i, (x, y) in enumerate(np.asarray(plan, float)[1:6], 1))
    return f"{leg.parent.name}  ·  출발 ({plan[0][0]:.2f}, {plan[0][1]:.2f}) → {goals}" if len(plan) else leg.parent.name


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
        mode = "정책" if label.upper().startswith("A") else "DWB"
        plot_report([(label, color, d, st)], out, f"rl_nav_controller · {label} ({mode})", _subtitle(a.paths))
        print(f"[{label}] " + " · ".join(f"{k}={v:.4g}" for k, v in st.items() if isinstance(v, float)))
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
    plot_report(runs, out, "rl_nav_controller A/B — 정책(A) vs DWB(B)", _subtitle(a.paths))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
