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


if __name__ == "__main__":
    main()
