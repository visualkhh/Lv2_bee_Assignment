#!/usr/bin/env python3
"""Figures and tables for the 2026-10-29 new-model retest (README.md next to this file).

  python3 make_figs.py            # reads data/*.npz (+ the two earlier reports' data), writes fig_*.png, prints the tables

New model = commit 830b724: battery_link on the rear pole, finger colliders convex decomposition (GRIP_STOP 6.2 mm).
Runs per grip force g (8.3, 22, 29 N per finger):
  pk_g{g}_s{1,2,3}   pick_sm, all randomization on (256 envs)
  sb_g{g}_s{1,2}     start box, 2 pucks side by side, perpendicular fingers (128 envs)
  ld/ul_g{g}_{L,R}_s{1,2}  basket load / unload of a full basket, chosen wrist + face rule (96 envs per side)
  pl_g{g}[_nb]       dice cell place after unloading, 40 mm gap, 3 mm (216 envs), _nb = other 8 cells full
Opening check at 22 N: sb_g22_o44_s{1,2}, ul_g22o44_{L,R}_s1 (fingers opened 44 mm instead of 40 mm).
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
D = HERE / "data"
OLD_R = HERE.parent / "20261028_retest" / "data"
OLD_W = HERE.parent / "20261029_wrist_boxwall" / "data"
FORCES = ("8.3", "22", "29")
S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, MUTED, GRID, AXIS, SURF = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#fcfcfb"
CMAP = matplotlib.colors.LinearSegmentedColormap.from_list("b", ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"])
plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
    "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
    "text.color": INK, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 10, "axes.titlesize": 11,
    "axes.titleweight": "bold", "legend.frameon": False,
})


def load(path):
    return np.load(path, allow_pickle=True)


def is_box(kinds):
    return np.char.find(np.asarray(kinds).astype(str), "box") >= 0


def save(fig, name):
    fig.tight_layout()
    fig.savefig(HERE / name, dpi=130)
    plt.close(fig)


# ---------- pick (pick_sm, all on)
def pick_rates(paths):
    runs = [load(p) for p in paths]
    v = np.array([100 * r["ok"].mean() for r in runs])
    ok = np.concatenate([r["ok"] for r in runs]); box = is_box(np.concatenate([r["kind"] for r in runs]))
    return v.mean(), v.min(), v.max(), 100 * ok[box].mean(), 100 * ok[~box].mean()


def pick():
    print("pick (all on, 256 x 3) | grip N | new all (seed range) / box / cyl | old all / box / cyl")
    old = {"8.3": OLD_R / "g8.3_s{}.npz", "22": OLD_W / "g22_s{}.npz", "29": OLD_R / "g29_s{}.npz"}
    new_m, old_m = [], []
    for f in FORCES:
        n = pick_rates([D / f"pk_g{f}_s{s}.npz" for s in (1, 2, 3)])
        o = pick_rates([Path(str(old[f]).format(s)) for s in (1, 2, 3)])
        new_m.append(n); old_m.append(o)
        print(f"  {f} | {n[0]:.1f} ({n[1]:.1f}~{n[2]:.1f}) / {n[3]:.1f} / {n[4]:.1f} | {o[0]:.1f} / {o[3]:.1f} / {o[4]:.1f}")
    return new_m, old_m


# ---------- start box (2 pucks side by side, perpendicular fingers)
def start_box():
    print("start box (128 x 2) | grip N | puck 1 pick / place / still in slot | puck 2 pick / place")
    out = {}
    for tag, files in [(f"new {f} N", [D / f"sb_g{f}_s{s}.npz" for s in (1, 2)]) for f in FORCES] + \
                      [("old 8.3 N", [OLD_R / f"s1_y_perp_s{s}.npz" for s in (1, 2)])]:
        ds = [load(p) for p in files]
        cat = lambda k: np.concatenate([d[k] for d in ds])
        n = len(cat("picked"))
        pk, pl, fo = (100 * np.nansum(cat(k), 0) / n for k in ("picked", "placed", "final_ok"))
        out[tag] = (pk, pl, fo)
        print(f"  {tag} | {pk[0]:.1f} / {pl[0]:.1f} / {fo[0]:.1f} | {pk[1]:.1f} / {pl[1]:.1f}")
    return out


# ---------- basket load / unload
def counts(task, files):
    """(hits, n) per kind (box, cyl) and slot (12)."""
    key, sk = ("placed", "slot_of") if task == "ld" else ("out_ok", "slot")
    h, m = np.zeros((2, 12)), np.zeros((2, 12))
    for p in files:
        d = load(p)
        v = d[key] if d[key].ndim == 1 else d[key][:, 0]
        sl = d[sk] if d[sk].ndim == 1 else d[sk][:, 0]
        box = is_box(d["kinds"][0])
        for k, sel in enumerate((box, ~box)):
            for j in range(12):
                c = sel & (sl == j)
                h[k, j] += np.nansum(v[c]); m[k, j] += c.sum()
    return h, m


def basket_files(root, task, run, seeds=(1, 2)):
    return [root / f"{task}_{run}_{side}_s{s}.npz" for side in ("L", "R") for s in seeds]


def basket():
    print("basket (96 x 2 sides x 2 seeds) | task | grip N | new all / box / cyl (seed 1 / 2) | old all / box / cyl")
    old_run = {"8.3": "best", "22": "g22", "29": "g29"}
    res = {}
    for task in ("ld", "ul"):
        for f in FORCES:
            h, m = counts(task, basket_files(D, task, f"g{f}"))
            ps = [100 * a.sum() / b.sum() for a, b in (counts(task, basket_files(D, task, f"g{f}", (s,))) for s in (1, 2))]
            oh, om = counts(task, basket_files(OLD_W, task, old_run[f]))
            res[(task, f)] = (h, m, oh, om)
            print(f"  {task} | {f} | {100 * h.sum() / m.sum():.1f} / {100 * h[0].sum() / m[0].sum():.1f} / {100 * h[1].sum() / m[1].sum():.1f}"
                  f" ({ps[0]:.1f} / {ps[1]:.1f}) | {100 * oh.sum() / om.sum():.1f} / {100 * oh[0].sum() / om[0].sum():.1f} / {100 * oh[1].sum() / om[1].sum():.1f}")
    print("slot | load box/cyl % per grip 8.3 | 22 | 29 | unload box/cyl % per grip 8.3 | 22 | 29")
    for j in range(12):
        cells = []
        for task in ("ld", "ul"):
            cells.append(" | ".join(f"{100 * res[(task, f)][0][0, j] / res[(task, f)][1][0, j]:.0f}/"
                                    f"{100 * res[(task, f)][0][1, j] / res[(task, f)][1][1, j]:.0f}" for f in FORCES))
        print(f"  {j:>2} | {cells[0]} | {cells[1]}")
    return res


def heat(ax, m, xt, yt, title):
    ax.imshow(m, cmap=CMAP, vmin=0, vmax=100, aspect="auto")
    for i in range(m.shape[0]):
        for j in range(m.shape[1]):
            ax.text(j, i, f"{m[i, j]:.0f}", ha="center", va="center", fontsize=8.5, color="#ffffff" if m[i, j] > 70 else INK)
    ax.set_xticks(range(len(xt)), xt, fontsize=8)
    ax.set_yticks(range(len(yt)), yt, fontsize=8.5)
    ax.set_title(title, fontsize=9.5)
    ax.grid(False)


def fig_basket(res):
    fig, axs = plt.subplots(1, 2, figsize=(12, 3.6))
    xt = [str(j) for j in range(12)]
    for ax, task, name in ((axs[0], "ld", "load"), (axs[1], "ul", "unload a full basket")):
        rows, labs = [], []
        for f in FORCES:
            h, m = res[(task, f)][:2]
            for k, kind in enumerate(("box", "cyl")):
                rows.append(100 * h[k] / np.maximum(m[k], 1)); labs.append(f"{f} N {kind}")
        heat(ax, np.array(rows), xt, labs, f"Basket {name}, new model, per slot [%]")
        ax.set_xlabel("slot (0-3 pole side, 4-7 middle, 8-11 arm side)")
    save(fig, "fig_n2_basket_slots.png")


# ---------- dice cell place
def place():
    runs = [(f"pl_g{f}{nb}", D / f"pl_g{f}{nb}.npz", f"new {f} N{', 8 neighbours' if nb else ''}") for nb in ("", "_nb") for f in FORCES]
    runs += [("old", OLD_R / "p_g40_h3.npz", "old 8.3 N"), ("old_nb", OLD_R / "p_g40_h3_nb.npz", "old 8.3 N, 8 neighbours")]
    print("place (216) | run | ok % of held | held at start | stuck in hand | neighbour moved | centre error mm median / p90 (near+mid) | per cell %")
    m, labs = [], []
    for _, p, lab in runs:
        d = load(p)
        h = d["held0"]
        cell = np.array([100 * d["ok"][h & (d["cell"] == c)].mean() for c in range(9)])
        e = 1000 * d["err"][h & (d["cell"] < 6)]
        print(f"  {lab} | {100 * d['ok'][h].mean():.1f} | {h.sum()}/{len(h)} | {d['in_hand'][h].sum()} | {d['nb_hit'][h].sum()} | "
              f"{np.nanmedian(e):.1f} / {np.nanpercentile(e, 90):.1f} | {' '.join(f'{v:.0f}' for v in cell)}")
        m.append(cell); labs.append(lab)
    fig, ax = plt.subplots(figsize=(8, 0.45 * len(m) + 1.5))
    heat(ax, np.array(m), ["near\n-y", "near\n0", "near\n+y", "mid\n-y", "mid\n0", "mid\n+y", "far\n-y", "far\n0", "far\n+y"],
         labs, "Dice cell: on target (< 10 mm, upright, still 2 s) [%]")
    ax.set_xlabel("cell (row from the robot, column)")
    save(fig, "fig_n3_place.png")


def fig_overview(pk, sb, res):
    """Success per step and grip force, new vs old model."""
    steps = ["pick\n(all on)", "start box\npuck 1", "start box\npuck 2", "basket\nload", "basket\nunload"]
    fig, axs = plt.subplots(1, 3, figsize=(12, 3.4), sharey=True)
    for ax, (i, f) in zip(axs, enumerate(FORCES)):
        new = [pk[0][i][0], sb[f"new {f} N"][0][0], sb[f"new {f} N"][0][1]] + \
              [100 * res[(t, f)][0].sum() / res[(t, f)][1].sum() for t in ("ld", "ul")]
        old = [pk[1][i][0]] + ([sb["old 8.3 N"][0][0], sb["old 8.3 N"][0][1]] if f == "8.3" else [np.nan, np.nan]) + \
              [100 * res[(t, f)][2].sum() / res[(t, f)][3].sum() for t in ("ld", "ul")]
        x = np.arange(len(steps))
        ax.bar(x - 0.2, old, 0.4, color=MUTED, label="before (hull fingers, no battery)")
        ax.bar(x + 0.2, new, 0.4, color=S1, label="new model")
        for xi, v in zip(x, new):
            ax.text(xi + 0.2, v + 1, f"{v:.0f}", ha="center", fontsize=8, color=INK2)
        ax.set_xticks(x, steps, fontsize=8)
        ax.set_title(f"{f} N per finger", fontsize=10)
        ax.set_ylim(0, 108)
    axs[0].set_ylabel("success [%]")
    axs[0].legend(loc="lower left", fontsize=8)
    save(fig, "fig_n1_overview.png")


def opening():
    print("opening 40 vs 44 mm at 22 N | start box puck 1 pick / puck 2 pick / puck 2 place | unload all / box / cyl (seed 1)")
    for gap, sb, ul in (("40", "sb_g22", "g22"), ("44", "sb_g22_o44", "g22o44")):
        ds = [load(D / f"{sb}_s{s}.npz") for s in (1, 2)]
        n = sum(len(d["picked"]) for d in ds)
        pk = 100 * sum(np.nansum(d["picked"], 0) for d in ds) / n
        pl = 100 * sum(np.nansum(d["placed"], 0) for d in ds) / n
        h, m = counts("ul", [D / f"ul_{ul}_{side}_s1.npz" for side in ("L", "R")])
        fails = np.concatenate([np.asarray(d["kinds"][1]).astype(str)[d["picked"][:, 1] == 0] for d in ds])
        print(f"  {gap} mm | {pk[0]:.1f} / {pk[1]:.1f} / {pl[1]:.1f} | {100 * h.sum() / m.sum():.1f} / {100 * h[0].sum() / m[0].sum():.1f} / "
              f"{100 * h[1].sum() / m[1].sum():.1f} | puck 2 pick fails: {len(fails)}, boxes {int(is_box(fails).sum()) if len(fails) else 0}")


if __name__ == "__main__":
    opening()
    pk = pick(); sb = start_box(); res = basket(); fig_basket(res); place(); fig_overview(pk, sb, res)
