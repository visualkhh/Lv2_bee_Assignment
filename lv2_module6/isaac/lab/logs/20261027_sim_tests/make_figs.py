#!/usr/bin/env python3
"""Figures and tables for the 2026-10-27 sim test report (README.md next to this file).

  python3 make_figs.py            # reads data/*.npz and data/t1_*.json, writes fig_*.png and prints the tables
"""
import glob
import json
import math
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
D = HERE / "data"
# reference data-viz palette (light): categorical slots 1-3, blue ramp for the heatmap, neutral ink
S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, MUTED, GRID, AXIS, SURF = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#fcfcfb"
BLUES = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]

plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
    "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
    "text.color": INK, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 10, "axes.titlesize": 11,
    "axes.titleweight": "bold", "legend.frameon": False,
})


def load(name):
    return np.load(D / f"{name}.npz", allow_pickle=True)


def rate(names):
    """Success rate % per run for the given run names (missing runs skipped)."""
    out = []
    for n in names:
        if (D / f"{n}.npz").exists():
            out.append(100 * load(n)["ok"].mean())
    return np.array(out)


def save(fig, name):
    fig.tight_layout()
    fig.savefig(HERE / name, dpi=150)
    plt.close(fig)
    print(f"-> {name}")


# ---------- T1 env0 fall-through
def t1():
    b, a = json.loads((D / "t1_before.json").read_text()), json.loads((D / "t1_after.json").read_text())
    mb, ma = np.array(b["masses"]), np.array(a["masses"])
    runs = np.arange(1, len(mb) + 1)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 3.6))
    ax1.plot(runs - 0.12, mb[:, 0] * 1000, "o", ms=8, color=S2, label="before (Lab randomize_rigid_body_mass)")
    ax1.plot(runs + 0.12, ma[:, 0] * 1000, "o", ms=8, color=S1, label="after (randomize_mass_synced)")
    for r, f in zip(runs, b["fell"]):
        if f: ax1.annotate("fell", (r - 0.12, 0), textcoords="offset points", xytext=(0, 8), ha="center", fontsize=8, color=INK2)
    ax1.set(xlabel="run (16 envs each)", ylabel="env0 puck mass in PhysX [g]", title="env0 mass as PhysX sees it", xticks=runs)
    ax1.set_ylim(-3, 40)
    ax1.legend(loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=1, fontsize=8)
    tr = np.load(D / "t1_trace_before.npz")
    P, dt = tr["trace_puck"], float(tr["step_dt"])
    t = np.arange(len(P)) * dt
    n = min(len(P), int(1.0 / dt))
    ax2.plot(t[:n], P[:n, 0, 2] * 1000, lw=2, color=S2, label="env0 (mass 0)")
    ax2.plot(t[:n], P[:n, 1, 2] * 1000, lw=2, color=S1, label="env1")
    ax2.axhline(70, color=AXIS, lw=1)
    ax2.text(t[n - 1], 72, "shelf top 70 mm", ha="right", va="bottom", fontsize=8, color=INK2)
    ax2.axhline(0, color=AXIS, lw=1)
    ax2.text(t[n - 1], 2, "floor", ha="right", va="bottom", fontsize=8, color=INK2)
    ax2.set(xlabel="time after reset [s]", ylabel="puck bottom z [mm]", title="Puck height, first 1 s (before fix)")
    ax2.set_ylim(-400, 120)
    ax2.legend(loc="lower left", fontsize=8)
    save(fig, "fig_t1_env0.png")
    print(f"T1 before: fell {sum(b['fell'])}/{len(b['fell'])}, env0 mass 0 in {(mb[:, 0] == 0).sum()} runs; "
          f"after: env0 mass 0 in {(ma[:, 0] == 0).sum()}/{len(ma)} runs, min mass {ma.min() * 1000:.1f} g")


# ---------- T2 gripper force x torsion
F2, R2 = ["8.3", "25", "38"], [0, 5, 10, 15]


def t2():
    m = np.zeros((len(F2), len(R2)))
    rows = []
    for i, f in enumerate(F2):
        for j, r in enumerate(R2):
            v = rate([f"t2_f{f}_r{r}_s{s}" for s in (1, 2, 3)])
            m[i, j] = v.mean()
            rows.append((f, r, v))
    fig, ax = plt.subplots(figsize=(6, 3.4))
    lo = 90.0
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list("b", BLUES)
    im = ax.imshow(np.clip(m, lo, 100), cmap=cmap, vmin=lo, vmax=100, aspect="auto")
    for i in range(len(F2)):
        for j in range(len(R2)):
            ax.text(j, i, f"{m[i, j]:.1f}", ha="center", va="center", fontsize=10,
                    color="#ffffff" if m[i, j] > 97 else INK)
    ax.set_xticks(range(len(R2)), [f"{r} mm" for r in R2])
    ax.set_yticks(range(len(F2)), [f"{f} N" for f in F2])
    ax.set(xlabel="torsional patch radius", ylabel="squeeze per finger", title="Scripted pick success [%] (256 envs x 3 seeds)")
    ax.grid(False)
    cb = fig.colorbar(im, ax=ax, fraction=0.05)
    cb.set_label(f"success % (clipped at {lo:.0f})", color=INK2)
    save(fig, "fig_t2_grip.png")
    print("T2 force N | torsion mm | success % per seed | mean")
    for f, r, v in rows:
        print(f"  {f:>4} | {r:>2} | {' '.join(f'{x:.1f}' for x in v)} | {v.mean():.1f}")


# ---------- T3 pull-out
def slip_stats(name):
    z = load(name)
    ok = z["ok"].astype(bool)
    s = z["slip"][ok]
    cens = z["slip_censored"][ok]
    return s, cens, float(z["slip_top"])


def t3():
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(12, 3.6))
    print("T3 pull | grip N | torsion mm | slip median [p10, p90] | never slipped")
    for ax, d, unit in ((ax1, "down", "N"), (ax2, "out", "N")):
        for k, (f, c) in enumerate(zip(F2, (S1, S1, S1))):  # one series: one colour
            s, cens, top = slip_stats(f"t3_{d}_f{f}")
            q = np.percentile(s, [10, 50, 90])
            ax.errorbar(k, q[1], yerr=[[q[1] - q[0]], [q[2] - q[1]]], fmt="o", ms=8, color=c, capsize=4, lw=2)
            ax.annotate(f"{q[1]:.1f}", (k, q[1]), textcoords="offset points", xytext=(10, 0), va="center", fontsize=9, color=INK)
            print(f"  {d:5} | {f:>4} | 10 | {q[1]:.2f} [{q[0]:.2f}, {q[2]:.2f}] {unit} | {cens.sum()}/{len(s)} (top {top:.1f})")
        ax.set_xticks(range(3), [f"{f} N" for f in F2])
        ax.set_xlim(-0.5, 2.7)
        ax.set(xlabel="squeeze per finger", ylabel=f"slip force [{unit}]", title=f"Pull {d}: force at slip (median, p10-p90)")
    # twist: torque vs torsion radius, two grip forces
    for f, c, dx in (("8.3", S1, -0.25), ("38", S2, 0.25)):
        xs, med, lo_, hi_ = [], [], [], []
        for r in (0, 5, 10, 15):
            name = f"t3_twist_f{f}" if r == 10 else f"t3_twist_f{f}_r{r}"
            if not (D / f"{name}.npz").exists(): continue
            s, cens, top = slip_stats(name)
            q = np.percentile(s, [10, 50, 90])
            xs.append(r); med.append(q[1]); lo_.append(q[1] - q[0]); hi_.append(q[2] - q[1])
            print(f"  twist | {f:>4} | {r:>2} | {q[1]:.3f} [{q[0]:.3f}, {q[2]:.3f}] N m | {cens.sum()}/{len(s)} (top {top:.2f})")
        ax3.errorbar(np.array(xs) + dx, med, yerr=[lo_, hi_], fmt="-o", ms=8, color=c, capsize=4, lw=2, label=f"{f} N squeeze")
        ax3.annotate(f"{f} N", (xs[-1], med[-1]), textcoords="offset points", xytext=(8, 0), va="center", fontsize=9, color=INK)
    ax3.set(xlabel="torsional patch radius [mm]", ylabel="slip torque [N m]", title="Twist: torque at slip")
    ax3.set_xticks([0, 5, 10, 15])
    ax3.legend(loc="center left", bbox_to_anchor=(0.0, 0.62), fontsize=8)
    save(fig, "fig_t3_pull.png")


# ---------- T4 control rate x latency
def t4():
    conds = [("60 Hz", "t4_hz60"), ("60 Hz + latency", "t4_hz60_lat"), ("100 Hz", "t4_hz100"), ("100 Hz + latency", "t4_hz100_lat")]
    fig, ax = plt.subplots(figsize=(6.5, 3.4))
    print("T4 condition | success % per seed | mean")
    for k, (lab, pre) in enumerate(conds):
        v = rate([f"{pre}_s{s}" for s in (1, 2, 3)])
        # dot plot, not bars: the axis starts at 90 %, a truncated bar would exaggerate the gap
        ax.plot(np.full(len(v), k), v, "o", ms=6, color=MUTED, zorder=2)
        ax.plot(k, v.mean(), "D", ms=10, color=S1 if "60" in lab else S2, zorder=3)
        ax.annotate(f"{v.mean():.1f}", (k, v.mean()), textcoords="offset points", xytext=(0, 6), ha="center", fontsize=9)
        print(f"  {lab:16} | {' '.join(f'{x:.1f}' for x in v)} | {v.mean():.1f}")
    ax.set_xticks(range(4), [c[0] for c in conds])
    ax.set_ylim(90, 101)
    ax.set(ylabel="success [%]", title="Control rate and command latency (diamond = mean, dots = seeds)")
    save(fig, "fig_t4_rate.png")


# ---------- T5 randomization
def t5():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 3.8))
    gaps = [40, 50, 60, 70]
    print("T5 neighbours gap mm | open | success % | knocked % | neighbours on shelf (median)")
    for open_, suf, c in (("66 mm (sim limit)", "", S1), ("40 mm (real)", "_open40", S2)):
        ys = []
        for g in gaps:
            names = [f"t5_nb{g}{suf}_s{s}" for s in (1, 2)]
            v = rate(names)
            kn = np.mean([100 * load(n)["nb_moved"].mean() for n in names if (D / f"{n}.npz").exists()])
            onb = np.median(np.concatenate([load(n)["nb_placed"] for n in names if (D / f"{n}.npz").exists()]))
            ys.append(v.mean())
            print(f"  {g} | {open_:17} | {v.mean():.1f} | {kn:.1f} | {onb:.0f}")
        ax1.plot(gaps, ys, "-o", lw=2, ms=8, color=c, label=f"open {open_}")
    ax1.set(xlabel="neighbour centre-to-centre gap [mm] (puck 30 mm)", ylabel="success [%]", title="Neighbour pucks on the shelf grid")
    ax1.set_xticks(gaps)
    ax1.set_xlim(37, 80)
    ax1.legend(loc="lower right", fontsize=8)
    conds = [("baseline", [f"t4_hz60_s{s}" for s in (1, 2, 3)]),
             ("open 40 mm", [f"t5_open40_s{s}" for s in (1, 2, 3)]),
             ("latency + jitter 20 ms", [f"t5_jit20_s{s}" for s in (1, 2, 3)]),
             ("parking err 10 mm", [f"t5_park10_s{s}" for s in (1, 2)]),
             ("parking err 20 mm", [f"t5_park20_s{s}" for s in (1, 2)]),
             ("parking err 30 mm", [f"t5_park30_s{s}" for s in (1, 2)]),
             ("all, 8.3 N", [f"t5_all_f8.3_s{s}" for s in (1, 2, 3)]),
             ("all, 38 N", [f"t5_all_f38_s{s}" for s in (1, 2, 3)])]
    print("T5 condition | success % per seed | mean")
    for k, (lab, names) in enumerate(conds):
        v = rate(names)
        ax2.plot(v, np.full(len(v), k), "o", ms=5, color=MUTED, zorder=2)
        ax2.plot(v.mean(), k, "D", ms=9, color=S2 if lab.startswith("all") else S1, zorder=3)
        ax2.annotate(f"{v.mean():.1f}", (max(v.max(), v.mean()), k), textcoords="offset points", xytext=(8, 0), va="center", fontsize=9)
        print(f"  {lab:22} | {' '.join(f'{x:.1f}' for x in v)} | {v.mean():.1f}")
    ax2.set_yticks(range(len(conds)), [c[0] for c in conds])
    ax2.invert_yaxis()
    ax2.set_xlim(50, 106)
    ax2.set(xlabel="success [%]", title="One change at a time, then all (diamond = mean)")
    save(fig, "fig_t5_random.png")


# ---------- T6 diagonal grasp of cylinders
def kind_split(names):
    ok, k = [], []
    for n in names:
        z = load(n)
        ok.append(z["ok"]); k.append(z["kind"].astype(str))
    ok, k = np.concatenate(ok), np.concatenate(k)
    cyl = np.char.find(k, "cyl") >= 0
    return 100 * ok[~cyl].mean(), 100 * ok[cyl].mean(), 100 * ok.mean()


def t6():
    conds = [("neighbours 50 mm, open 40", [f"t5_nb50_open40_s{s}" for s in (1, 2)], [f"t6_nb50_open40_diag_s{s}" for s in (1, 2)]),
             ("neighbours 40 mm, open 40", [f"t5_nb40_open40_s{s}" for s in (1, 2)], [f"t6_nb40_open40_diag_s{s}" for s in (1, 2)]),
             ("all (100 Hz, latency, open 40, nb 50, park 10)", [f"t5_all_f8.3_s{s}" for s in (1, 2, 3)], [f"t6_all_diag_s{s}" for s in (1, 2, 3)])]
    fig, ax = plt.subplots(figsize=(8, 3.4))
    print("T6 condition | box before/after | cylinder before/after | all before/after")
    for k, (lab, before, after) in enumerate(conds):
        b0, c0, a0 = kind_split(before)
        b1, c1, a1 = kind_split(after)
        ax.plot([c0, c1], [k, k], color=AXIS, lw=2, zorder=1)
        ax.plot(c0, k, "o", ms=9, color=S2, zorder=2, label="cylinder, face rule" if k == 0 else None)
        ax.plot(c1, k, "o", ms=9, color=S1, zorder=2, label="cylinder, diagonal" if k == 0 else None)
        ax.plot(b0, k, "s", ms=7, color=MUTED, zorder=2, label="box (unchanged rule)" if k == 0 else None)
        ax.annotate(f"{c0:.1f} → {c1:.1f}", (max(c0, c1), k), textcoords="offset points", xytext=(10, 0), va="center", fontsize=9)
        print(f"  {lab:45} | {b0:.1f} / {b1:.1f} | {c0:.1f} / {c1:.1f} | {a0:.1f} / {a1:.1f}")
    ax.set_yticks(range(len(conds)), [c[0] for c in conds])
    ax.invert_yaxis()
    ax.set_xlim(20, 125)
    ax.set_xticks([20, 40, 60, 80, 100])
    ax.set(xlabel="success [%]", title="Cylinders: fingers along the grid diagonal")
    ax.legend(loc="lower left", fontsize=8)
    save(fig, "fig_t6_diag.png")


# ---------- T7 basket on the robot
def t7():
    z = load("t7_basket")
    sl, w, ok = z["slot"], z["wrist"], z["b_ok"]
    m = np.zeros((4, 12))
    for ww in range(4):
        for k in range(12):
            sel = (sl == k) & (w == ww)
            m[ww, k] = 100 * ok[sel].mean()
    fig, ax = plt.subplots(figsize=(9, 3.2))
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list("b", BLUES)
    im = ax.imshow(m, cmap=cmap, vmin=0, vmax=100, aspect="auto")
    for ww in range(4):
        for k in range(12):
            ax.text(k, ww, f"{m[ww, k]:.0f}", ha="center", va="center", fontsize=9, color="#ffffff" if m[ww, k] > 60 else INK)
    ax.set_xticks(range(12), [f"{k}" for k in range(12)])
    ax.set_yticks(range(4), ["+0", "+90", "+180", "+270"])
    for x in (3.5, 7.5):
        ax.axvline(x, color=SURF, lw=3)
    ax.set(xlabel="slot (0-3 pole side, 4-7 middle, 8-11 arm side)", ylabel="wrist option [deg]",
           title="Puck left upright in its basket slot [%] (4 envs per cell)")
    ax.grid(False)
    fig.colorbar(im, ax=ax, fraction=0.03).set_label("in slot %", color=INK2)
    save(fig, "fig_t7_basket.png")
    print("T7 row | wrist | in slot | joint5 at over_slot [deg]")
    for row in range(3):
        for ww in range(4):
            sel = (sl // 4 == row) & (w == ww)
            print(f"  {row} | +{90 * ww:3d} | {ok[sel].sum()}/{sel.sum()} | {np.degrees(z['b_q_over'][sel][0, 4]):.0f}")


# ---------- T8 box face pair chosen by neighbour room
def t8():
    conds = [("neighbours 50 mm", "t8_nb50_open40_diag", "t8_nb50_open40_diag_bc"),
             ("neighbours 40 mm", "t8_nb40_open40_diag", "t8_nb40_open40_diag_bc"),
             ("all", "t6_all_diag", "t8_all_diag_bc")]
    fig, ax = plt.subplots(figsize=(8, 3.0))
    print("T8 condition | box: face rule / box_clear | all: face rule / box_clear")
    for k, (lab, a, b) in enumerate(conds):
        b0, _, a0 = kind_split([f"{a}_s{s}" for s in (1, 2, 3)])
        b1, _, a1 = kind_split([f"{b}_s{s}" for s in (1, 2, 3)])
        ax.plot([b0, b1], [k, k], color=AXIS, lw=2, zorder=1)
        ax.plot(b0, k, "o", ms=9, color=S1, zorder=2, label="box, face closest to heading" if k == 0 else None)
        ax.plot(b1, k, "o", ms=9, color=S2, zorder=2, label="box, face with more room (--box_clear)" if k == 0 else None)
        ax.annotate(f"{b0:.1f} → {b1:.1f}", (max(b0, b1), k), textcoords="offset points", xytext=(10, 0), va="center", fontsize=9)
        print(f"  {lab:18} | {b0:.1f} / {b1:.1f} | {a0:.1f} / {a1:.1f}")
    ax.set_yticks(range(len(conds)), [c[0] for c in conds])
    ax.invert_yaxis()
    ax.set_xlim(20, 125)
    ax.set_xticks([20, 40, 60, 80, 100])
    ax.set(xlabel="box success [%] (256 envs x 3 seeds, open 40 mm, cylinders diagonal)", title="Boxes: switching to the face pair with more room")
    ax.legend(loc="lower left", fontsize=8)
    save(fig, "fig_t8_box.png")


# ---------- T9 basket: release and retreat variants
def t9():
    var = [("first version (section 7)", "t7_basket"), ("floor release + fold", "t9_basket_plain"),
           ("+ in-hand correction", "t9_basket_correct"), ("+ skip home pose", "t9_basket_skip"),
           ("+ correction + skip home", "t9_basket_skip_correct")]
    rows = ["pole side 0-3", "middle 4-7", "arm side 8-11"]
    fig, ax = plt.subplots(figsize=(8.5, 4.0))
    print("T9 variant | pole | middle | arm | total | in-hand offset p90 mm")
    for k, (lab, name) in enumerate(var):
        z = load(name)
        sl, ok = z["slot"], z["b_ok"]
        r = [100 * ok[sl // 4 == i].mean() for i in range(3)]
        for i, (v, c, mk) in enumerate(zip(r, (S1, S2, S3), ("o", "s", "D"))):
            ax.plot(v, k, mk, ms=8, color=c, zorder=2, label=rows[i] if k == 0 else None)
        tot = 100 * ok.mean()
        ax.plot(tot, k, "|", ms=18, mew=2.5, color=INK, zorder=3, label="all slots" if k == 0 else None)
        ax.annotate(f"{tot:.0f} %", (max(r + [tot]), k), textcoords="offset points", xytext=(10, 0), va="center", fontsize=9)
        off = (np.percentile(np.linalg.norm(z["puck_in_hand_ee"][:, :2], axis=1), 90) * 1000) if "puck_in_hand_ee" in z else float("nan")
        print(f"  {lab:28} | {r[0]:.0f} | {r[1]:.0f} | {r[2]:.0f} | {tot:.0f} ({ok.sum()}/{len(ok)}) | {off:.0f}")
    ax.set_yticks(range(len(var)), [v[0] for v in var])
    ax.invert_yaxis()
    ax.set_xlim(0, 110)
    ax.set(xlabel="puck upright in its slot [%] (192 envs, 64 per row, all 4 wrist options)", title="Basket: release and retreat")
    ax.legend(loc="upper center", bbox_to_anchor=(0.45, -0.22), ncol=4, fontsize=8)
    save(fig, "fig_t9_basket.png")


# ---------- T10 pick -> basket cycle: routes and continuous vs stop-and-go
ROUTES = [("direct", "c10_direct_h0"), ("direct, stop 0.3 s", "c10_direct_h0.3"), ("carry 150°", "c10_carry_h0"),
          ("carry 150°, stop 0.3 s", "c10_carry_h0.3"), ("home", "c10_home_h0"), ("home, stop 0.3 s", "c10_home_h0.3"),
          ("carry 90°", "c10_carry90_h0")]


def t10():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.0), gridspec_kw={"width_ratios": [1.1, 1]})
    print("T10 route | placed % (seed 1, 2) | rows pole/middle/arm | blocked % | cycle s median")
    for k, (lab, pre) in enumerate(ROUTES):
        pl, sl, bl, tc = [], [], [], []
        for s in (1, 2):
            z = load(f"{pre}_s{s}")
            pl.append(z["placed"][:, 0]); sl.append(z["slot_of"][:, 0]); bl.append(z["blocked"][:, 0]); tc.append(z["t_cycle"][:, 0])
        per_seed = [100 * x.mean() for x in pl]
        pl, sl, bl, tc = map(np.concatenate, (pl, sl, bl, tc))
        rate_, tmed = 100 * pl.mean(), np.median(tc)
        c = S2 if lab == "carry 90°" else (MUTED if "stop" in lab else S1)
        ax1.plot(tmed, rate_, "D" if lab == "carry 90°" else "o", ms=10, color=c, zorder=3)
        ax1.annotate(lab, (tmed, rate_), textcoords="offset points", xytext=(8, 4), fontsize=8, color=INK2)
        rows = [100 * pl[sl // 4 == r].mean() for r in range(3)]
        print(f"  {lab:24} | {per_seed[0]:.1f}, {per_seed[1]:.1f} | {' / '.join(f'{x:.0f}' for x in rows)} | "
              f"{100 * (bl > math.radians(10)).mean():.1f} | {tmed:.1f}")
    ax1.set(xlabel="cycle time, median [s] (vel scale 0.3)", ylabel="puck upright in its slot [%]",
            title="Route: speed vs placement (192 envs x 2 seeds)")
    ax1.set_xlim(13.5, 29)
    ax1.set_ylim(40, 75)
    z = load("c10_carry90_h0_s1")
    labs, secs = list(z["timeline_labels"]), np.median(z["timeline_s"], axis=1)
    keep = [(l, t) for l, t in zip(labs, secs) if l != "approach"]
    left = 0.0
    shades = {"descend": S1, "close": S1, "lift": S1, "carry": S2, "over_slot": S2, "lower": S3, "release": S3,
              "lift_out": S3, "carry_back": S2, "to_next": MUTED}
    for l, t in keep:
        ax2.barh(0, t, left=left, height=0.5, color=shades.get(l, MUTED), edgecolor=SURF, linewidth=2)
        ax2.text(left + t / 2, 0.38 if keep.index((l, t)) % 2 else -0.38, f"{l}\n{t:.1f}", ha="center", va="center", fontsize=7.5, color=INK2)
        left += t
    ax2.set_ylim(-0.8, 0.8)
    ax2.set_yticks([])
    ax2.set(xlabel="seconds (median per step)", title=f"carry 90°: one cycle = {left:.1f} s")
    ax2.grid(axis="y", visible=False)
    print("T10 carry 90° steps (median s): " + ", ".join(f"{l} {t:.2f}" for l, t in keep))
    save(fig, "fig_t10_routes.png")


# ---------- T11 release variants and per-slot placement (carry 90°, single puck)
def t11():
    var = [("put down on the floor (cycle default)", "c11_d1"), ("let go 3 mm above the floor", "c11_e_low3"),
           ("open to 66 mm (sim only)", "c11_e_gap66"), ("3 mm + 66 mm", "c11_e_low3_gap66"), ("3 mm + mirrored wrist", "c11_e_low3_mirror")]
    print("T11 variant | placed % | rows pole/middle/arm | arm-side fails stuck in the hand")
    for lab, name in var:
        z = load(name)
        sl, pl, stk = z["slot_of"][:, 0], z["placed"][:, 0], z["stuck"][:, 0]
        rows = [100 * pl[sl // 4 == r].mean() for r in range(3)]
        print(f"  {lab:38} | {100 * pl.mean():.0f} | {' / '.join(f'{x:.0f}' for x in rows)} | {int(((sl // 4 == 2) & (pl == 0) & (stk == 1)).sum())}")
    zs = [load(f"c11_f1_low3_s{s}") for s in (1, 2)]
    sl = np.concatenate([z["slot_of"][:, 0] for z in zs]); pl = np.concatenate([z["placed"][:, 0] for z in zs])
    m = np.array([[100 * pl[sl == r * 4 + c].mean() for c in range(4)] for r in range(3)])
    fig, ax = plt.subplots(figsize=(5.6, 3.4))
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list("b", BLUES)
    im = ax.imshow(m, cmap=cmap, vmin=0, vmax=100)
    for r in range(3):
        for c in range(4):
            ax.text(c, r, f"slot {r * 4 + c}\n{m[r, c]:.0f} %", ha="center", va="center", fontsize=9, color="#ffffff" if m[r, c] > 60 else INK)
    ax.set_xticks(range(4), ["y −97", "y −32", "y +32", "y +97"])
    ax.set_yticks(range(3), ["pole side", "middle", "arm side"])
    ax.set(title="Placed upright per slot (carry 90°, 3 mm, 32 envs per slot)")
    ax.grid(False)
    fig.colorbar(im, ax=ax, fraction=0.04).set_label("%", color=INK2)
    save(fig, "fig_t11_slots.png")
    print("T11 per slot % (rows pole/middle/arm): " + " | ".join(" ".join(f"{x:.0f}" for x in row) for row in m))


# ---------- T12 sequential loading (5 pucks on the shelf grid, carry 90°)
def t12():
    fig, ax = plt.subplots(figsize=(8, 3.6))
    print("T12 order | per load: picked % / placed % (n) | all picked | all placed | still in slot at the end | cycle s median")
    for o, c, mk in (("good_first", S2, "D"), ("back_first", S1, "o"), ("front_first", MUTED, "s")):
        zs = [load(f"c12_{o}_s{s}") for s in (1, 2)]
        pk = np.concatenate([z["picked"] for z in zs]); pl = np.concatenate([z["placed"] for z in zs])
        fin = np.concatenate([z["final_ok"] for z in zs]); tc = np.concatenate([z["t_cycle"] for z in zs])
        xs, ys, parts = [], [], []
        for k in range(pl.shape[1]):
            m = ~np.isnan(pl[:, k])
            xs.append(k + 1); ys.append(100 * pl[m, k].mean())
            parts.append(f"{100 * np.nanmean(pk[:, k]):.0f} / {100 * pl[m, k].mean():.0f} ({m.sum()})")
        ax.plot(xs, ys, "-" + mk, lw=2, ms=8, color=c, label=o.replace("_", " "))
        print(f"  {o:11} | {' | '.join(parts)} | {100 * np.nanmean(pk):.1f} | {100 * np.nanmean(pl):.1f} | {100 * np.nanmean(fin):.1f} | {np.nanmedian(tc):.1f}")
    ax.set(xlabel="puck (1 = target, 2-5 = grid neighbours +x, −x, +y, −y), loaded in this order", ylabel="placed upright [%]",
           title="Loading up to 5 pucks in a row (64 envs x 2 seeds)")
    ax.set_xticks([1, 2, 3, 4, 5])
    ax.set_ylim(0, 105)
    ax.legend(loc="lower left", fontsize=8)
    save(fig, "fig_t12_sequence.png")


if __name__ == "__main__":
    for f in (t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12):
        try:
            f()
        except FileNotFoundError as e:
            print(f"skip {f.__name__}: {e}")
