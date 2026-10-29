#!/usr/bin/env python3
"""Figures and tables for the 2026-10-29 wrist / box-face report (README.md next to this file).

  python3 make_figs.py            # reads data/*.npz, writes fig_*.png and prints the tables

Runs: {ld = loading, ul = unloading a full basket} x {L = slots 0,1,4,5,8,9 (-y), R = slots 2,3,6,7,10,11 (+y)} x seed 1, 2.
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
D = HERE / "data"
S1, S2, S3, S4 = "#2a78d6", "#eb6834", "#1baf7a", "#9b59d0"
INK, INK2, MUTED, GRID, AXIS, SURF = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#fcfcfb"
CMAP = matplotlib.colors.LinearSegmentedColormap.from_list("b", ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"])
plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
    "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
    "text.color": INK, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 10, "axes.titlesize": 11,
    "axes.titleweight": "bold", "legend.frameon": False,
})
# configs as (label, -y side run, +y side run); the -y side wrist does not change with the +y mirror
CFG = [("now", "base_L", "base_R"), ("wrist mirrored on +y", "base_L", "mir_R"), ("box faces to walls, all rows", "wall_L", "wall_R"),
       ("mirror + faces, all rows", "wall_L", "mirwall_R"), ("mirror + faces, not pole row", "best_L", "best_R")]


def counts(task, run, seed=None):
    """(hits, n) per slot (12) and kind (box, cyl)."""
    key, sk = ("placed", "slot_of") if task == "ld" else ("out_ok", "slot")
    h, m = np.zeros((2, 12)), np.zeros((2, 12))
    for s in ([seed] if seed else (1, 2)):
        d = np.load(D / f"{task}_{run}_s{s}.npz", allow_pickle=True)
        v = d[key] if d[key].ndim == 1 else d[key][:, 0]
        sl = d[sk] if d[sk].ndim == 1 else d[sk][:, 0]
        box = np.char.find(d["kinds"][0].astype(str), "box") >= 0
        for k, sel in enumerate((box, ~box)):
            for j in range(12):
                c = sel & (sl == j)
                h[k, j] += np.nansum(v[c]); m[k, j] += c.sum()
    return h, m


def cfg_counts(task, l, r, seed=None):
    hl, ml = counts(task, l, seed); hr, mr = counts(task, r, seed)
    right = np.array([j % 4 >= 2 for j in range(12)])
    return np.where(right, hr, hl), np.where(right, mr, ml)


def heat(ax, m, title):
    ax.imshow(m, cmap=CMAP, vmin=0, vmax=100, aspect="auto")
    for i in range(m.shape[0]):
        for j in range(m.shape[1]):
            ax.text(j, i, f"{m[i, j]:.0f}", ha="center", va="center", fontsize=9, color="#ffffff" if m[i, j] > 70 else INK)
    ax.set_xticks(range(4), ["y -97", "y -32", "y +32", "y +97"])
    ax.set_yticks(range(3), ["pole side", "middle", "arm side"])
    ax.set_title(title, fontsize=9.5)
    ax.grid(False)


def main():
    print("config | task | box % | cyl % | all % | seed 1 / seed 2 all % | n per slot and kind")
    summ = {}
    for lab, l, r in CFG:
        for task in ("ld", "ul"):
            h, m = cfg_counts(task, l, r)
            per_seed = [100 * a.sum() / b.sum() for a, b in (cfg_counts(task, l, r, s) for s in (1, 2))]
            box, cyl, al = 100 * h[0].sum() / m[0].sum(), 100 * h[1].sum() / m[1].sum(), 100 * h.sum() / m.sum()
            summ[(lab, task)] = (box, cyl, al, per_seed)
            print(f"  {lab} | {task} | {box:.1f} | {cyl:.1f} | {al:.1f} | {per_seed[0]:.1f} / {per_seed[1]:.1f} | {int(m.min())}-{int(m.max())}")
    print("slot | load box/cyl now -> mirror+faces not pole | unload box/cyl now -> mirror+faces not pole")
    p = {k: [100 * h / np.maximum(m, 1) for h, m in [cfg_counts(k, *CFG[0][1:])]][0] for k in ("ld", "ul")}
    q = {k: [100 * h / np.maximum(m, 1) for h, m in [cfg_counts(k, *CFG[-1][1:])]][0] for k in ("ld", "ul")}
    for j in range(12):
        print(f"  {j:>2} | {p['ld'][0, j]:.0f}/{p['ld'][1, j]:.0f} -> {q['ld'][0, j]:.0f}/{q['ld'][1, j]:.0f} | "
              f"{p['ul'][0, j]:.0f}/{p['ul'][1, j]:.0f} -> {q['ul'][0, j]:.0f}/{q['ul'][1, j]:.0f}")

    # fig 1: overall bars
    fig, ax = plt.subplots(figsize=(9, 3.4))
    labs = [c[0] for c in CFG]
    x = np.arange(len(labs))
    for k, (task, name, col) in enumerate((("ld", "loading", S1), ("ul", "unloading full basket", S2))):
        v = [summ[(lb, task)][2] for lb in labs]
        sd = np.array([summ[(lb, task)][3] for lb in labs])
        ax.bar(x + (k - 0.5) * 0.38, v, 0.36, color=col, label=name)
        ax.errorbar(x + (k - 0.5) * 0.38, v, yerr=[np.array(v) - sd.min(1), sd.max(1) - np.array(v)], fmt="none", ecolor=INK2, capsize=3, lw=1)
        for xi, vi in zip(x, v):
            ax.text(xi + (k - 0.5) * 0.38, vi + 3, f"{vi:.0f}", ha="center", fontsize=8.5, color=INK2)
    ax.axhline(summ[("now", "ld")][2], color=S1, lw=0.8, ls="--")
    ax.set_xticks(x, labs, fontsize=8.5)
    ax.set_ylim(0, 105)
    ax.set_ylabel("success [%], 12 slots")
    ax.set_title("Basket loading / unloading, 12 slots (whisker = seed 1 / 2 range)")
    ax.legend(loc="lower right")
    fig.tight_layout(); fig.savefig(HERE / "fig_w1_overall.png", dpi=150); plt.close(fig)

    # fig 2: per-slot maps, now vs chosen
    fig, axs = plt.subplots(2, 4, figsize=(14, 5.2))
    for r, (lab, l, rr) in enumerate((("now", *CFG[0][1:]), ("chosen", *CFG[-1][1:]))):
        for c, (task, k, name) in enumerate((("ld", 0, "loading, box"), ("ld", 1, "loading, cylinder"), ("ul", 0, "unloading, box"), ("ul", 1, "unloading, cylinder"))):
            h, m = cfg_counts(task, l, rr)
            heat(axs[r, c], (100 * h[k] / np.maximum(m[k], 1)).reshape(3, 4), f"{lab}: {name} [%]")
    fig.tight_layout(); fig.savefig(HERE / "fig_w2_slots.png", dpi=150); plt.close(fig)

    # fig 3: box loading per slot, all 5 configs
    fig, axs = plt.subplots(1, 5, figsize=(17, 2.6))
    for ax, (lab, l, rr) in zip(axs, CFG):
        h, m = cfg_counts("ld", l, rr)
        heat(ax, (100 * h[0] / np.maximum(m[0], 1)).reshape(3, 4), lab)
    fig.suptitle("Box puck loading per slot [%]", fontweight="bold")
    fig.tight_layout(); fig.savefig(HERE / "fig_w3_box_loading.png", dpi=150); plt.close(fig)
    print("-> fig_w1_overall.png fig_w2_slots.png fig_w3_box_loading.png")


def grip():
    """Grip force 0.3-0.4 A (22-29 N) vs the 8.3 N sim default, chosen basket rule. Pick runs at 8.3-29 N other than 22 N are in
    ../20261028_retest/data (same command, 40 mm open)."""
    old = HERE.parent / "20261028_retest" / "data"
    print("pick (all randomization on, 256 envs x 3 seeds) | N | all % (seed range) | box | cyl")
    pk = {}
    for f, src in (("8.3", old), ("22", D), ("25", old), ("29", old), ("36", old)):
        rs = [np.load(src / f"g{f}_s{s}.npz", allow_pickle=True) for s in (1, 2, 3)]
        v = [100 * r["ok"].mean() for r in rs]
        k = np.concatenate([r["kind"] for r in rs]).astype(str); ok = np.concatenate([r["ok"] for r in rs])
        pk[f] = (np.mean(v), 100 * ok[np.char.find(k, "box") >= 0].mean(), 100 * ok[np.char.find(k, "cyl") >= 0].mean())
        print(f"  {f} | {np.mean(v):.1f} ({min(v):.1f}-{max(v):.1f}) | {pk[f][1]:.1f} | {pk[f][2]:.1f}")
    print("basket, chosen rule | N | task | all % | box | cyl | seed 1 / 2")
    bs = {}
    for f, tag in (("8.3", "best"), ("22", "g22"), ("29", "g29"), ("37", "g37")):
        for task in ("ld", "ul"):
            h, m = cfg_counts(task, f"{tag}_L", f"{tag}_R")
            sd = [100 * a.sum() / b.sum() for a, b in (cfg_counts(task, f"{tag}_L", f"{tag}_R", s) for s in (1, 2))]
            bs[(f, task)] = (100 * h.sum() / m.sum(), 100 * h[0].sum() / m[0].sum(), 100 * h[1].sum() / m[1].sum())
            print(f"  {f} | {task} | {bs[(f, task)][0]:.1f} | {bs[(f, task)][1]:.1f} | {bs[(f, task)][2]:.1f} | {sd[0]:.1f} / {sd[1]:.1f}")
    print("unload, -y slots, seed 1: puck in the hand after close / lift / carry back [%]")
    for f in ("8.3", "29"):
        d = np.load(D / f"ul_stage_g{f}.npz", allow_pickle=True)
        cyl = np.char.find(d["kinds"][0].astype(str), "cyl") >= 0
        for nm, msk in (("cyl", cyl), ("box", ~cyl)):
            print(f"  {f} N {nm} | " + " / ".join(f"{100 * d[k][msk].mean():.0f}" for k in ("held_close", "held_lift", "held")))
    print("unload 29 N, seed 1, gripper damping (default 10) | cyl % | box %")
    for dmp, pat in (("10", "ul_g29_{}_s1"), ("19", "ul_g29d19_{}_s1"), ("35", "ul_g29d35_{}_s1")):
        o = [np.load(D / f"{pat.format(sd)}.npz", allow_pickle=True) for sd in "LR"]
        c = np.concatenate([d["out_ok"][np.char.find(d["kinds"][0].astype(str), "cyl") >= 0] for d in o])
        b = np.concatenate([d["out_ok"][np.char.find(d["kinds"][0].astype(str), "box") >= 0] for d in o])
        print(f"  {dmp} | {100 * c.mean():.0f} | {100 * b.mean():.0f}")

    fig, axs = plt.subplots(1, 3, figsize=(12, 3.4), sharey=True)
    for ax, (title, get, xs) in zip(axs, (("Pick (start box grid)", lambda f, k: pk[f][k], ["8.3", "22", "25", "29", "36"]),
                                          ("Basket loading", lambda f, k: bs[(f, "ld")][k], ["8.3", "22", "29", "37"]),
                                          ("Unloading full basket", lambda f, k: bs[(f, "ul")][k], ["8.3", "22", "29", "37"]))):
        x = [float(f) for f in xs]
        for k, lab, col, mk in ((1, "box", S1, "o-"), (2, "cylinder", S2, "s-")):
            ax.plot(x, [get(f, k) for f in xs], mk, color=col, lw=2, label=lab)
        ax.axvspan(22, 29, color=AXIS, alpha=0.35, lw=0)
        ax.axvspan(29, 37, color=AXIS, alpha=0.15, lw=0)
        ax.text(25.5, 42, "0.3-0.4 A", ha="center", fontsize=8, color=INK2)
        ax.text(33, 42, "0.4-0.5 A", ha="center", fontsize=8, color=INK2)
        ax.set(title=title, xlabel="squeeze per finger [N]")
        ax.set_ylim(40, 102)
    axs[0].set_ylabel("success [%]")
    axs[0].legend(loc="lower left")
    fig.tight_layout(); fig.savefig(HERE / "fig_w4_grip.png", dpi=150); plt.close(fig)
    print("-> fig_w4_grip.png")


def contact():
    """Why cylinders leave the hand at 29 N: contact settings (unload, -y slots, seed 1) and the finger collider shape."""
    runs = [("base29", "29 N as is"), ("rest0", "puck restitution 0"), ("iters32", "solver iterations 32"), ("tors0", "no torsional patch"),
            ("hull", "cylinder as convex hull"), ("smooth29", "cylinder as custom geometry"), ("coff5", "puck contact offset 5 mm"),
            ("fx_coff_10_roff_0", "puck contact offset 10 mm"), ("ffric1", "finger friction 1.2"),
            ("fx_coff_5_roff_0_ffric_1.0", "offset 5 mm + friction 1.0"), ("pose8.3", "8.3 N")]
    print("contact run | cylinder in the hand after close / lift / carry [%] | box taken out [%]")
    for nm, lab in runs:
        d = np.load(D / f"ct_{nm}.npz", allow_pickle=True)
        cyl = np.char.find(d["kinds"][0].astype(str), "cyl") >= 0
        print(f"  {lab} | " + " / ".join(f"{100 * d[k][cyl].mean():.0f}" for k in ("held_close", "held_lift", "held")) + f" | {100 * d['out_ok'][~cyl].mean():.0f}")
    print("puck in the end effector frame after close (median): bottom z [mm], sideways |y| [mm], finger joint [mm] (8.3 = touching a 30 mm puck)")
    for nm, lab in (("pose8.3", "8.3 N"), ("pose29", "29 N")):
        d = np.load(D / f"ct_{nm}.npz", allow_pickle=True)
        cyl = np.char.find(d["kinds"][0].astype(str), "cyl") >= 0
        for g, m in (("held after lift", cyl & d["held_lift"]), ("lost on lift", cyl & ~d["held_lift"])):
            if m.sum():
                c = d["ee_close"][m]
                print(f"  {lab} {g} (n {m.sum()}) | {1000 * np.median(c[:, 2]):.1f} | {1000 * np.median(np.abs(c[:, 1])):.1f} | {1000 * np.median(c[:, 4]):.2f}")

    print("finger colliders as convex decomposition | cylinder / box in the hand after close / lift / carry [%] | finger joint at close [mm]")
    for nm, lab in (("pose8.3", "hull, 8.3 N"), ("pose29", "hull, 29 N"), ("dec8.3", "decomposition, old stop 8.3 mm: 6.2 N, open 44 mm"),
                    ("dec29", "decomposition, old stop 8.3 mm: 22 N, open 44 mm"), ("dec2_8.3", "decomposition, stop 6.2 mm: 8.3 N, open 40 mm"),
                    ("dec2_29", "decomposition, stop 6.2 mm: 29 N, open 40 mm"), ("dec2_29_o44", "decomposition, stop 6.2 mm: 29 N, open 44 mm")):
        d = np.load(D / f"ct_{nm}.npz", allow_pickle=True)
        cyl = np.char.find(d["kinds"][0].astype(str), "cyl") >= 0
        row = [" / ".join(f"{100 * d[k][m].mean():.0f}" for k in ("held_close", "held_lift", "held")) for m in (cyl, ~cyl)]
        print(f"  {lab} | {row[0]} | {row[1]} | {1000 * np.median(d['ee_close'][:, 4]):.2f}")

    from scipy.spatial import ConvexHull
    fig, ax = plt.subplots(figsize=(9, 4))
    for side, col in (("left", S1), ("right", S2)):
        g = np.load(D / f"finger_{side}_mesh_gripper_frame.npy") * 1000
        ax.scatter(g[:, 0], g[:, 1], s=0.3, color=col, alpha=0.4)
        h = ConvexHull(g[:, :2]); v = np.r_[h.vertices, h.vertices[0]]
        ax.plot(g[v, 0], g[v, 1], color=col, lw=1.4, label=f"{side} finger collider (convex hull)")
    ax.axvspan(109.6, 129.6, color=AXIS, alpha=0.4, lw=0)
    ax.text(119.6, -33, "puck contact\n(top 20 mm)", ha="center", fontsize=8, color=INK2)
    ax.set_aspect("equal")
    ax.set(xlabel="along the finger, towards the tips [mm]", ylabel="closing axis [mm]",
           title="Finger mesh (dots) and the convex hull PhysX collides with, fingers closed")
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=8)
    fig.tight_layout(); fig.savefig(HERE / "fig_w5_finger_hull.png", dpi=150); plt.close(fig)
    print("-> fig_w5_finger_hull.png")


if __name__ == "__main__":
    main()
    grip()
    contact()
