#!/usr/bin/env python3
"""Figures and tables for the 2026-10-28 re-test report (README.md next to this file).

  python3 make_figs.py            # reads data/*.npz, writes fig_*.png and prints the tables
"""
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
D = HERE / "data"
S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, MUTED, GRID, AXIS, SURF = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#fcfcfb"
BLUES = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]
CMAP = matplotlib.colors.LinearSegmentedColormap.from_list("b", BLUES)

plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
    "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
    "text.color": INK, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 10, "axes.titlesize": 11,
    "axes.titleweight": "bold", "legend.frameon": False,
})


def load(name):
    return np.load(D / f"{name}.npz", allow_pickle=True)


def save(fig, name):
    fig.tight_layout()
    fig.savefig(HERE / name, dpi=150)
    plt.close(fig)
    print(f"-> {name}")


def heat(ax, m, xt, yt, title, lo=0.0, fmt="{:.0f}"):
    ax.imshow(np.clip(m, lo, 100), cmap=CMAP, vmin=lo, vmax=100, aspect="auto")
    for i in range(m.shape[0]):
        for j in range(m.shape[1]):
            ax.text(j, i, fmt.format(m[i, j]), ha="center", va="center", fontsize=9, color="#ffffff" if m[i, j] > 70 else INK)
    ax.set_xticks(range(len(xt)), xt)
    ax.set_yticks(range(len(yt)), yt)
    ax.set_title(title)
    ax.grid(False)


# ---------- grip force, new vs old robot model
FORCES = ["8.3", "12", "18", "25", "29", "36"]


def grip():
    fig, ax = plt.subplots(figsize=(7, 3.6))
    print("grip force N | new model all / box / cyl | old model all")
    rows = {}
    for tag, pre, col in (("new model", "", S1), ("before (COM -85 mm, shelf 207)", "oldcom_", MUTED)):
        mean, lo, hi, box, cyl = [], [], [], [], []
        for f in FORCES:
            runs = [load(f"{pre}g{f}_s{s}") for s in (1, 2, 3)]
            v = np.array([100 * r["ok"].mean() for r in runs])
            mean.append(v.mean()); lo.append(v.min()); hi.append(v.max())
            k = np.concatenate([r["kind"] for r in runs]); ok = np.concatenate([r["ok"] for r in runs])
            box.append(100 * ok[np.char.find(k.astype(str), "box") >= 0].mean())
            cyl.append(100 * ok[np.char.find(k.astype(str), "cyl") >= 0].mean())
        x = [float(f) for f in FORCES]
        ax.fill_between(x, lo, hi, color=col, alpha=0.15, lw=0)
        ax.plot(x, mean, "o-", color=col, lw=2, label=tag)
        rows[tag] = (mean, box, cyl)
    new, old = rows["new model"], rows["before (COM -85 mm, shelf 207)"]
    for i, f in enumerate(FORCES):
        print(f"  {f:>4} | {new[0][i]:.1f} / {new[1][i]:.1f} / {new[2][i]:.1f} | {old[0][i]:.1f}")
    ax.plot([float(f) for f in FORCES], new[2], "s--", color=S2, ms=4, lw=1, label="new model, cylinders only")
    for xa, lab in ((11.0, "0.15 A"), (14.7, "0.2 A"), (44.1, "0.6 A (now)")):  # x 73.5 N/A (KT 2.2 Nm/A measured)
        ax.axvline(xa, color=AXIS, lw=1, ls=":")
        ax.text(xa, 101.2, lab, ha="center", fontsize=8, color=INK2)
    ax.set(xlabel="squeeze per finger [N] (sim)", ylabel="pick success [%]", title="Pick success vs grip force, all randomization on (256 envs x 3 seeds)")
    ax.set_ylim(80, 102)
    ax.legend(loc="lower left", fontsize=8)
    save(fig, "fig_r1_grip.png")


def hz():
    print("hz | latency | success % per seed | mean")
    for h in (60, 100):
        for lat in ("", "_lat"):
            v = np.array([100 * load(f"t4_hz{h}{lat}_s{s}")["ok"].mean() for s in (1, 2, 3)])
            print(f"  {h} | {'on' if lat else 'off'} | {' '.join(f'{x:.1f}' for x in v)} | {v.mean():.1f}")


# ---------- start box
def start_box():
    labels, data = [], []
    print("layout | rule | puck 1 pick / place / still in slot | puck 2 pick / place | blocked deg median (puck 2)")
    for lay, lname in (("y", "side by side (±Y)"), ("x", "one behind (±X)")):
        for r in ("heading", "perp"):
            ds = [load(f"s1_{lay}_{r}_s{s}") for s in (1, 2)]
            cat = lambda k: np.concatenate([d[k] for d in ds])
            n = len(cat("picked"))
            pk, pl, fo = (100 * np.nansum(cat(k), 0) / n for k in ("picked", "placed", "final_ok"))
            bl = np.degrees(np.nanmedian(cat("blocked"), 0))
            print(f"  {lay} | {r} | {pk[0]:.1f} / {pl[0]:.1f} / {fo[0]:.1f} | {pk[1]:.1f} / {pl[1]:.1f} | {bl[1]:.1f}")
            labels.append(f"{lname}\n{'perpendicular' if r == 'perp' else 'heading rule'}")
            data.append((pk, pl, fo))
    fig, ax = plt.subplots(figsize=(9, 3.8))
    x = np.arange(len(labels))
    w = 0.2
    for k, (lab, col, idx, j) in enumerate((("puck 1 picked", S1, 0, 0), ("puck 2 picked", S3, 0, 1),
                                           ("puck 2 in slot", S2, 1, 1), ("puck 1 still in slot at the end", MUTED, 2, 0))):
        ax.bar(x + (k - 1.5) * w, [d[idx][j] for d in data], w, color=col, label=lab)
    ax.set_xticks(x, labels, fontsize=8)
    ax.set(ylabel="%", title="Start box: 2 pucks 80 mm apart (2 x 128 envs)")
    ax.set_ylim(0, 105)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.25), ncol=4, fontsize=8)
    save(fig, "fig_r2_start_box.png")


# ---------- basket slots
def slot_rates(names):
    pl = np.zeros(12); n = np.zeros(12)
    for nm in names:
        d = load(nm)
        sl = d["slot_of"][:, 0]
        for s in range(12):
            m = sl == s
            pl[s] += np.nansum(d["placed"][m, 0]); n[s] += m.sum()
    return 100 * pl / np.maximum(n, 1)


def by_kind(names, key, slot_of):
    """% per slot (12) for boxes and cylinders separately, pooled over runs."""
    acc = {"box": [np.zeros(12), np.zeros(12)], "cyl": [np.zeros(12), np.zeros(12)]}
    for nm in names:
        d = load(nm)
        kind = np.char.find(d["kinds"][0].astype(str), "box") >= 0
        sl, v = slot_of(d), d[key] if d[key].ndim == 1 else d[key][:, 0]
        for s in range(12):
            for k, m in (("box", kind), ("cyl", ~kind)):
                sel = (sl == s) & m
                acc[k][0][s] += np.nansum(v[sel]); acc[k][1][s] += sel.sum()
    return {k: 100 * a / np.maximum(n, 1) for k, (a, n) in acc.items()}


def slots_kind():
    ld = by_kind(["c12_slots_s1", "c12_slots_s2"], "placed", lambda d: d["slot_of"][:, 0])
    ul = by_kind(["u_dz0"], "out_ok", lambda d: d["slot"])
    fig, axs = plt.subplots(2, 2, figsize=(9, 5.2))
    cols = ["y -97", "y -32", "y +32", "y +97"]
    rows = ["pole side", "middle", "arm side"]
    for j, (k, lab) in enumerate((("box", "box pucks"), ("cyl", "cylinders"))):
        heat(axs[0, j], ld[k].reshape(3, 4), cols, rows, f"Loading, {lab} [%]")
        heat(axs[1, j], ul[k].reshape(3, 4), cols, rows, f"Unloading full basket, {lab} [%]")
    save(fig, "fig_r3b_slots_kind.png")
    print("slot | loading box / cyl % | unloading box / cyl %")
    for s in range(12):
        print(f"  {s:>2} | {ld['box'][s]:.0f} / {ld['cyl'][s]:.0f} | {ul['box'][s]:.0f} / {ul['cyl'][s]:.0f}")


def slots():
    base = slot_rates(["c12_slots_s1", "c12_slots_s2"])
    flip = slot_rates(["c12_flip2_s1", "c12_flip2_s2"])
    fig, axs = plt.subplots(1, 2, figsize=(9, 2.8))
    cols = ["y -97", "y -32", "y +32", "y +97"]
    rows = ["pole side", "middle", "arm side"]
    heat(axs[0], base.reshape(3, 4), cols, rows, "Slot placement [%], wrist as now")
    heat(axs[1], flip.reshape(3, 4), cols, rows, "Same, slot 2 wrist turned the other way")
    save(fig, "fig_r3_slots.png")
    print("slot | placed % now | slot 2 flipped")
    for s in range(12):
        print(f"  {s:>2} | {base[s]:.0f} | {flip[s]:.0f}")
    print(f"  mean | {base.mean():.1f} | {flip.mean():.1f}")
    for nm in ():
        d = load(nm)
        sl = d["slot_of"][:, 0]
        print(f"{nm}: slot | placed / n | stuck | puck bottom off centre dx, dy mm (median) | tilt deg")
        for s in (8, 9, 10, 11):
            m = sl == s
            print(f"  {s} | {np.nansum(d['placed'][m, 0]):.0f}/{m.sum()} | {np.nansum(d['stuck'][m, 0]):.0f} | "
                  f"{1000 * np.nanmedian(d['dx'][m, 0]):+.1f}, {1000 * np.nanmedian(d['dy'][m, 0]):+.1f} | {np.degrees(np.nanmedian(d['tilt'][m, 0])):.0f}")


def contacts():
    d = load("c12_contacts")
    sl = d["slot_of"][:, 0]
    bodies = list(d["contact_bodies"])
    print("contacts: slot | move | robot body with the largest median contact force [N] (wheels and fingers left out)")
    for s in (1, 2, 8, 9, 10, 11):
        m = (sl == s) & (d["picked"][:, 0] == 1)
        for mv in ("lower", "release", "lift_out"):
            c = np.median(d[f"contact_{mv}"][m], axis=0)
            top = [(bodies[j], c[j]) for j in np.argsort(-c) if "wheel" not in bodies[j] and "finger" not in bodies[j]][:2]
            print(f"  {s} | {mv} | " + ", ".join(f"{b} {v:.0f}" for b, v in top))


def sequence():
    print("5 pucks | puck pick % / place % (envs) | total placed | still in slot at the end")
    for tag in ("held", "noheld"):
        ds = [load(f"c13_{tag}_s{s}") for s in (1, 2)]
        cat = lambda k: np.concatenate([d[k] for d in ds])
        on = cat("on_shelf")
        pk, pl, fo = cat("picked"), cat("placed"), cat("final_ok")
        per = " · ".join(f"{100 * np.nansum(pk[:, j]) / on[:, j].sum():.0f}/{100 * np.nansum(pl[:, j]) / on[:, j].sum():.0f} ({on[:, j].sum()})"
                         for j in range(pk.shape[1]))
        print(f"  {tag} | {per} | {100 * np.nansum(pl) / on.sum():.1f} | {100 * np.nansum(fo) / on.sum():.1f}")


# ---------- unload from a full basket
def unload():
    runs = [("u_dz-5", "-5 mm"), ("u_dz-3", "-3 mm"), ("u_dz0", "0"), ("u_dz3", "+3 mm"), ("u_dz5", "+5 mm"),
            ("u_yaw10", "yaw 10°"), ("u_yaw20", "yaw 20°"), ("u_yaw30", "yaw 30°"), ("u_yaw45", "yaw 45°"), ("u_flip2", "0, slot 2 flipped")]
    m = np.zeros((len(runs), 12)); nb = np.zeros((len(runs), 12))
    print("unload run | out and held % | envs with a neighbour moved > 3 mm or tilted > 5 deg | per slot % (0..11)")
    for i, (nm, lab) in enumerate(runs):
        d = load(nm)
        hit = (d["nb_disp"] > 0.003) | (d["nb_dtilt"] > math.radians(5.0))
        for s in range(12):
            k = d["slot"] == s
            m[i, s] = 100 * d["out_ok"][k].mean()
            nb[i, s] = 100 * hit[k].any(axis=1).mean()
        print(f"  {lab} | {100 * d['out_ok'].mean():.1f} | {hit.any(axis=1).sum()}/{len(hit)} | {' '.join(f'{v:.0f}' for v in m[i])}")
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.2))
    heat(axs[0], m, [str(s) for s in range(12)], [lab for _, lab in runs], "Taken out and held [%] per slot")
    heat(axs[1], 100 - nb, [str(s) for s in range(12)], [lab for _, lab in runs], "Neighbours untouched [%] per slot")
    for ax in axs:
        ax.set_xlabel("slot (0-3 pole side, 4-7 middle, 8-11 arm side)")
    save(fig, "fig_r4_unload.png")


# ---------- dice cells
def place():
    runs = [("p_g36_h0", "36 mm, 0 mm"), ("p_g36_h3", "36 mm, 3 mm"), ("p_g40_h0", "40 mm, 0 mm"), ("p_g40_h3", "40 mm, 3 mm"),
            ("p_g40_h6", "40 mm, 6 mm"), ("p_g66_h3", "66 mm, 3 mm"), ("p_g40_h3_nonoise", "40 mm, 3 mm, no aim noise"),
            ("p_g36_h3_nb", "36 mm, 3 mm, 8 neighbours"), ("p_g40_h3_nb", "40 mm, 3 mm, 8 neighbours")]
    runs = [(nm, lab) for nm, lab in runs if (D / f"{nm}.npz").exists()]
    m = np.zeros((len(runs), 9))
    print("place run (gap, let go at) | ok % of held | stuck in hand | neighbour moved > 3 mm | centre error mm median / p90 (near+mid rows) | per cell %")
    for i, (nm, lab) in enumerate(runs):
        d = load(nm)
        h = d["held0"]
        for c in range(9):
            k = h & (d["cell"] == c)
            m[i, c] = 100 * d["ok"][k].mean()
        nm6 = h & (d["cell"] < 6)
        e = 1000 * d["err"][nm6]
        print(f"  {lab} | {100 * d['ok'][h].mean():.1f} | {d['in_hand'][h].sum()}/{h.sum()} | {d['nb_hit'][h].sum()} | "
              f"{np.median(e):.1f} / {np.percentile(e, 90):.1f} | {' '.join(f'{v:.0f}' for v in m[i])}")
    fig, ax = plt.subplots(figsize=(8, 0.5 * len(runs) + 1.5))
    heat(ax, m, ["near\n-y", "near\n0", "near\n+y", "mid\n-y", "mid\n0", "mid\n+y", "far\n-y", "far\n0", "far\n+y"],
         [lab for _, lab in runs], "Dice cell: on target (< 10 mm, upright, still 2 s) [%]")
    ax.set_xlabel("cell (row from the robot, column)")
    save(fig, "fig_r5_place.png")


if __name__ == "__main__":
    grip(); hz(); start_box(); slots(); slots_kind(); contacts(); sequence(); unload(); place()
