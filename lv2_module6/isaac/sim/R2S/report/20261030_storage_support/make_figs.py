#!/usr/bin/env python3
"""Figures and tables for the 2026-10-30 storage_support ring report (README.md next to this file).

  <IsaacLab>/.venv/bin/python make_figs.py     # reads data/*.npz (+ ../20261029_wrist_boxwall/data), writes fig_*.png, prints the tables

Run names (data/):
  ld_* / ul_*   cycle_sm.py loading / unloading a full basket, {L = slots 0,1,4,5,8,9, R = 2,3,6,7,10,11} x seed 1, 2
                supwall 34/25 | h38t25wall 38/25 | c36ch0wall 36/25 | c34ch{4,8,12}wall 34/25 chamfer 4/8/12 (all --box_wall all)
                fix 34/25 ch8 + pole slot fix | h34t30ch8fix 34/30 ch8 + fix | h34t35ch8fix 34/35 ch8 + fix (final)
                pole slot fix = --wall_all_slots 1,2 --unload_away_mm 8 --unload_dx_slots 1,2
  ul_dx*/ul_dy* pole slots 1 / 2 only, grasp offset along x / sideways away from the pole [mm]; ul_flip, ul_wallall: wrist variants
  tip2_<ring>_<spin>   tip_test.py --mode push (96 pucks, one per cell, kicked at <spin> rad/s)
  drv_<ring>_x1.0      drive_test.py, 12 pucks per env, real speed limits
  pl_* / plt_*  cycle_sm.py --place (unload, then dice cell): pl_none no ring | plt_{fix,nofix} 34/25 ch8 with / without
                --place_tilt_fix | pl_t35 34/35 ch8; _k<rad/s> = --unload_kick
  pldiag3_L, pltrace50c_L   far-row release diagnostics (puck in hand per stage; per-step trace with puck contact partners)
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
D = HERE / "data"
BASE = HERE.parent / "20261029_wrist_boxwall" / "data"
S1, S2, S3, S4 = "#2a78d6", "#eb6834", "#1baf7a", "#9b59d0"
INK, INK2, MUTED, GRID, AXIS, SURF = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#fcfcfb"
plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
    "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
    "text.color": INK, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 10, "axes.titlesize": 11,
    "axes.titleweight": "bold", "legend.frameon": False,
})
RINGS = [("no ring", BASE, "best"), ("34/25", D, "supwall"), ("36/25", D, "c36ch0wall"), ("38/25", D, "h38t25wall"),
         ("34/25 ch4", D, "c34ch4wall"), ("34/25 ch8", D, "c34ch8wall"), ("34/25 ch12", D, "c34ch12wall"),
         ("34/25 ch8 + fix", D, "fix"), ("34/30 ch8 + fix", D, "h34t30ch8fix"), ("34/35 ch8 + fix", D, "h34t35ch8fix")]


def load(f):
    return np.load(f, allow_pickle=True)


def col(d, k):
    v = d[k]
    return v[:, 0] if v.ndim > 1 else v


def is_box(d):
    return np.char.find(d["kinds"][0].astype(str), "box") >= 0


def basket(src, run, task):
    """(hits box, n box, hits cyl, n cyl, upright hits) over L/R x seed 1, 2; per slot arrays too."""
    key, sk = ("placed", "slot_of") if task == "ld" else ("out_ok", "slot")
    h, m, up = np.zeros((2, 12)), np.zeros((2, 12)), 0.0
    for s in (1, 2):
        for side in "LR":
            d = load(src / f"{task}_{run}_{side}_s{s}.npz")
            v, sl, box = col(d, key), col(d, sk), is_box(d)
            if task == "ld":
                up += np.nansum(v * (np.degrees(col(d, "tilt")) <= 5.0))
            for k, sel in enumerate((box, ~box)):
                for j in range(12):
                    c = sel & (sl == j)
                    h[k, j] += np.nansum(v[c]); m[k, j] += c.sum()
    # L runs hold only the -y slots, R runs only the +y slots (--slots), so the per-slot sums do not overlap
    return h, m, up


def tab_rings():
    print("\n## 1. ring | load % (box/cyl) | load upright <=5 deg % | unload % (box/cyl) | lowest slot,kind load / unload")
    out = {}
    for lab, src, run in RINGS:
        hl, ml, up = basket(src, run, "ld")
        hu, mu, _ = basket(src, run, "ul")
        pl, pu = 100 * hl / np.maximum(ml, 1), 100 * hu / np.maximum(mu, 1)
        lo_l, lo_u = np.unravel_index(np.argmin(np.where(ml > 0, pl, 999)), pl.shape), np.unravel_index(np.argmin(np.where(mu > 0, pu, 999)), pu.shape)
        kinds = ("box", "cyl")
        r = dict(ld=100 * hl.sum() / ml.sum(), ld_b=100 * hl[0].sum() / ml[0].sum(), ld_c=100 * hl[1].sum() / ml[1].sum(),
                 up=100 * up / ml.sum(), ul=100 * hu.sum() / mu.sum(), ul_b=100 * hu[0].sum() / mu[0].sum(), ul_c=100 * hu[1].sum() / mu[1].sum(), n=int(ml.sum()))
        out[lab] = r
        print(f"  {lab:16s} | {r['ld']:5.1f} ({r['ld_b']:.1f}/{r['ld_c']:.1f}) | {r['up']:5.1f} | {r['ul']:5.1f} ({r['ul_b']:.1f}/{r['ul_c']:.1f}) | "
              f"slot {lo_l[1]} {kinds[lo_l[0]]} {pl[lo_l]:.0f} / slot {lo_u[1]} {kinds[lo_u[0]]} {pu[lo_u]:.0f} | n {r['n']}")
    return out


def tab_tip():
    print("\n## 2. kicked puck in the cell (tip_test, 96 pucks): spin | leaning >= 20 deg in the cell | max tilt box / cyl in the cell | thrown out of the cell")
    rows = {}
    for lab, tag in (("no ring", "none"), ("34/25", "h34ch0"), ("36/25", "h36ch0"), ("38/25", "h38ch0"), ("34/25 ch4", "h34ch4"),
                     ("34/25 ch8", "h34ch8"), ("34/25 ch12", "h34ch12"), ("34/30 ch8", "h34t30ch8"), ("34/35 ch8", "h34t35ch8")):
        for sp in (20, 40, 60, 80, 120):
            f = D / f"tip2_{tag}_{sp}.npz"
            if not f.exists():
                continue
            d = load(f)
            # older runs did not save the kind: MultiUsdFileCfg deals the puck types round robin, env i -> PUCK_USDS[i % 4] (box, box, cyl, cyl)
            box = d["box"] if "box" in d.files else (np.arange(len(d["tilt"])) % 4) < 2
            t, inside = d["tilt"], d["off"] < 25
            # thrown out = centre more than 25 mm off the cell, or sitting on the ring top (centre > 38 mm above the floor)
            inside &= d["hz"] < 38
            mb = t[inside & box].max() if (inside & box).any() else 0.0
            mc = t[inside & ~box].max() if (inside & ~box).any() else 0.0
            rows[(lab, sp)] = (int(((t >= 20) & inside).sum()), mb, mc, int((~inside).sum()))
            print(f"  {lab:11s} {sp:3d} rad/s | {rows[(lab, sp)][0]:2d} | {mb:4.0f} / {mc:4.0f} | {rows[(lab, sp)][3]}")
    return rows


def tab_drive():
    print("\n## 3. driving with 12 loaded pucks (1152 per run): ring | upright at the end | max tilt during, box / cyl | worst profile")
    names = ["straight", "straight hard", "spin", "spin hard", "arc", "arc hard", "reverse hard", "zigzag"]
    out = {}
    for lab, tag in (("no ring", "none"), ("34/25", "h34ch0"), ("38/25", "h38ch0"), ("34/25 ch8", "h34ch8"), ("34/30 ch8", "h34t30ch8"), ("34/35 ch8", "h34t35ch8")):
        d = load(D / f"drv_{tag}_x1.0.npz")
        mt, box, pr = d["max_tilt"], d["box"], d["prof"]
        worst = max(range(8), key=lambda k: mt[pr == k].max())
        out[lab] = (int(d["upright"].sum()), d["upright"].size, mt[box].max(), mt[~box].max())
        print(f"  {lab:10s} | {out[lab][0]}/{out[lab][1]} | {out[lab][2]:.1f} / {out[lab][3]:.1f} | {names[worst]} (base tilt max {d['base_tilt'].max():.1f} deg)")
    return out


def pole(prefix, seeds):
    hb = nb = hc = nc = 0
    for s in seeds:
        for side in "LR":
            f = D / f"{prefix}_{side}_s{s}.npz"
            if not f.exists():
                continue
            d = load(f)
            key = "out_ok" if "out_ok" in d.files else "placed"
            v, box = col(d, key), is_box(d)
            hb += np.nansum(v[box]); nb += box.sum(); hc += np.nansum(v[~box]); nc += (~box).sum()
    return 100 * hb / max(nb, 1), 100 * hc / max(nc, 1), int(nb + nc)


def tab_pole():
    print("\n## 4. pole slots 1 / 2, unloading (34/25 ch8, gripper body along y): grasp offset | box % | cyl % | n")
    out = {}
    for lab, pre, seeds in (("centre, wrist as loaded (box wall rule)", None, None), ("wrist flipped (camera to the pole)", "ul_flip", (1, 2)),
                            ("body along y for both kinds", "ul_wallall", (1,))) + tuple(
            (f"x +{v} mm (towards the arm)", f"ul_dx{v}", (1,)) for v in (2, 4, 6, 8)) + tuple(
            (f"sideways {v} mm away from the pole", f"ul_dy{v}", (1, 2)) for v in (4, 6, 8, 10, 12)):
        if pre is None:
            h, m, _ = basket(D, "c34ch8wall", "ul")
            out[lab] = (100 * h[0, 1:3].sum() / m[0, 1:3].sum(), 100 * h[1, 1:3].sum() / m[1, 1:3].sum(), int(m[:, 1:3].sum()))
        else:
            out[lab] = pole(pre, seeds)
        print(f"  {lab:40s} | {out[lab][0]:5.1f} | {out[lab][1]:5.1f} | {out[lab][2]}")
    return out


def place_runs(pattern):
    d = [load(f) for f in sorted(D.glob(pattern))]
    g = lambda k: np.concatenate([x[k] for x in d])
    box = np.concatenate([is_box(x) for x in d])
    return g, box


def tab_place():
    print("\n## 5. unload -> dice cell, near + mid rows (far row 0 % in every run, section 6)")
    print("   run | start tilt in the cell | n | held % | upright on the cell %")
    out = {}
    for lab, pat in (("no ring, kick 0/40/60", "pl_none_k*_*.npz"), ("34/25 ch8, no tilt fix, kick 60", "plt_nofix_*.npz"),
                     ("34/25 ch8, tilt fix, kick 60", "plt_fix_*.npz"), ("34/35 ch8, kick 0/80", "pl_t35_k*_*.npz")):
        g, box = place_runs(pat)
        ok, held, st, cell = g("ok"), g("held0"), np.degrees(g("start_tilt")), g("cell")
        near = cell // 3 < 2
        for blab, m in (("< 5 deg", st < 5), ("5-20 deg", (st >= 5) & (st < 20)), ("20-26 deg", (st >= 20) & (st < 26)), (">= 26 deg", st >= 26)):
            s = m & near
            if s.sum() == 0:
                continue
            out[(lab, blab)] = (int(s.sum()), 100 * held[s].mean(), 100 * ok[s].mean())
            print(f"  {lab:32s} | {blab:27s} | {s.sum():3d} | {out[(lab, blab)][1]:5.1f} | {out[(lab, blab)][2]:5.1f}")
        print(f"  {lab:32s} | far row upright {int(ok[cell // 3 == 2].sum())}/{int((cell // 3 == 2).sum())}")
    return out


def tab_far():
    print("\n## 6. far row release (34/25 ch8, seed 1, L slots): puck bottom in the hand (mm, median) per stage")
    d = load(D / "pldiag3_L.npz")
    r = d["cell"] // 3
    rows = {}
    for row, name in ((0, "near"), (1, "mid"), (2, "far")):
        m = (r == row) & d["held0"]
        v = [np.median(d[k][m][:, 2]) * 1000 for k in ("puck_in_ee_carry", "puck_in_ee_above", "puck_in_ee_down", "puck_in_ee_release")]
        rows[name] = (int(d["ok"][m].sum()), int(m.sum()), *v, np.degrees(np.median(d["base_pitch_down"][m])))
        print(f"  {name:4s}: ok {rows[name][0]}/{rows[name][1]} | carry {v[0]:.1f} above {v[1]:.1f} down {v[2]:.1f} after open {v[3]:.1f} | base pitch at down {rows[name][-1]:+.2f} deg")
    t = load(D / "pltrace50c_L.npz")
    T, rr = t["trace_release"], t["cell"] // 3
    print("  trace (gap 50 mm): t s | puck z in the base frame above the box top mm | puck contact N: fingers L/R, shelf")
    for name, row in (("mid", 1), ("far", 2)):
        i = np.where((rr == row) & t["held0"])[0][0]
        z, F = T[:, i, 2] * 1000 - 60.0, T[:, i, 17:23]  # base_link sits 10 mm above the floor: box top at z 60 mm
        print(f"   {name}: " + "; ".join(f"{k * 0.01:.1f} s {z[k]:+.1f} mm F {F[k, 0]:.1f}/{F[k, 1]:.1f} shelf {F[k, 5]:.2f}" for k in (0, 10, 30, 50, 70)))
    return rows, T, rr, t["held0"]


def fig_rings(r):
    labs = [k for k in r]
    x = np.arange(len(labs))
    fig, ax = plt.subplots(figsize=(10, 3.8))
    w = 0.26
    for k, (key, c, name) in enumerate((("ld", S1, "load (in the cell, < 20 deg)"), ("up", S3, "load, upright < 5 deg"), ("ul", S2, "unload"))):
        ax.bar(x + (k - 1) * w, [r[l][key] for l in labs], w, color=c, label=name)
    ax.set_xticks(x, labs, rotation=25, ha="right")
    ax.set_ylim(40, 109); ax.set_ylabel("%"); ax.legend(loc="upper left", ncols=3)
    ax.set_title("Basket loading / unloading per ring (hole/height mm, chamfer mm), 384 runs each")
    fig.tight_layout(); fig.savefig(HERE / "fig_s1_rings.png", dpi=130); plt.close(fig)


def fig_tilt(tip, drv):
    fig, (a, b) = plt.subplots(1, 2, figsize=(10, 3.6))
    labs = [l for l in ("no ring", "34/25", "38/25", "34/25 ch8", "34/30 ch8", "34/35 ch8")]
    x = np.arange(len(labs))
    a.bar(x - 0.2, [tip[(l, 40)][1] for l in labs], 0.4, color=S1, label="box")
    a.bar(x + 0.2, [tip[(l, 40)][2] for l in labs], 0.4, color=S2, label="cylinder")
    a.set_xticks(x, labs, rotation=25, ha="right"); a.set_ylabel("max tilt in the cell [deg]"); a.legend()
    a.set_title("Kicked at 40 rad/s (96 pucks)")
    b.bar(x - 0.2, [drv[l][2] for l in labs], 0.4, color=S1, label="box")
    b.bar(x + 0.2, [drv[l][3] for l in labs], 0.4, color=S2, label="cylinder")
    b.set_xticks(x, labs, rotation=25, ha="right"); b.set_ylabel("max tilt while driving [deg]"); b.legend()
    b.set_title("Driving at the real limits (1152 pucks)")
    fig.tight_layout(); fig.savefig(HERE / "fig_s2_tilt.png", dpi=130); plt.close(fig)


def fig_pole(p):
    v = [4, 6, 8, 10, 12]
    fig, ax = plt.subplots(figsize=(6, 3.4))
    ax.plot([0] + v, [p["body along y for both kinds"][0]] + [p[f"sideways {k} mm away from the pole"][0] for k in v], "o-", color=S1, label="box")
    ax.plot([0] + v, [p["body along y for both kinds"][1]] + [p[f"sideways {k} mm away from the pole"][1] for k in v], "o-", color=S2, label="cylinder")
    ax.set_xlabel("grasp offset sideways, away from the pole [mm]"); ax.set_ylabel("unloaded %"); ax.set_ylim(-3, 103); ax.legend()
    ax.set_title("Pole slots 1 / 2: unloading vs grasp offset")
    fig.tight_layout(); fig.savefig(HERE / "fig_s3_pole_offset.png", dpi=130); plt.close(fig)


def fig_far(T, rr, held):
    fig, (a, b) = plt.subplots(1, 2, figsize=(10, 3.4))
    t = np.arange(len(T)) * 0.01
    for name, row, c in (("mid row", 1, S1), ("far row", 2, S2)):
        i = np.where((rr == row) & held)[0][0]
        a.plot(t, T[:, i, 2] * 1000 - 60.0, color=c, label=name)
        b.plot(t, T[:, i, 22], color=c, label=name)
    a.set_xlabel("time after the fingers start to open [s]"); a.set_ylabel("puck bottom above box top, robot frame [mm]"); a.legend()
    a.set_title("Puck seen from the robot")
    b.set_xlabel("time after the fingers start to open [s]"); b.set_ylabel("puck - shelf contact [N]"); b.set_ylim(0, 1); b.legend()
    b.set_title("Puck stays on the box (0.18 N = its weight)")
    fig.tight_layout(); fig.savefig(HERE / "fig_s4_far_release.png", dpi=130); plt.close(fig)


def main():
    r = tab_rings()
    tip = tab_tip()
    drv = tab_drive()
    p = tab_pole()
    tab_place()
    _, T, rr, held = tab_far()
    fig_rings(r); fig_tilt(tip, drv); fig_pole(p); fig_far(T, rr, held)
    print("\nfigures: fig_s1_rings.png fig_s2_tilt.png fig_s3_pole_offset.png fig_s4_far_release.png")


if __name__ == "__main__":
    main()
