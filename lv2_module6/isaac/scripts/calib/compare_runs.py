#!/usr/bin/env python3
"""Compare two sin_test sessions (same sine.yaml) joint by joint (run plot_logs.py on both first, it writes analysis.yaml).

  python3 compare_runs.py 20261024 20261026      # date prefixes of the run folders in logs/

Per session the latest run of each joint is used. Also measures the state update interval (median time between
position changes in log.csv). Output: logs/compare_<a>_<b>.png, logs/compare_<a>_<b>.yaml.
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

from plot_logs import GRID, INK, MUTED, load, style

HERE = Path(__file__).resolve().parent
LOGS = HERE / 'logs'
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
OLD, NEW = '#a3a29e', '#2a78d6'  # earlier session recessive grey, newer session categorical slot 1
PANELS = [('lag [ms]', 'lag_ms', '{:.0f}'), ('state update interval [ms]', 'update_ms', '{:.0f}'),
          ('RMS error [deg]', 'rms_error', '{:.1f}'),
          ('RMS error, lag + offset removed [deg]', 'rms_error_lag_offset_removed', '{:.2f}'),
          ('peak error, lag removed [deg]', 'peak_error_lag_removed', '{:.1f}'), ('gain', 'gain', '{:.2f}')]


def session(prefix):
    out = {}
    for run in sorted(LOGS.glob(f'{prefix}_*')):  # sorted by time -> the last run of a joint wins
        if not (run / 'analysis.yaml').exists(): continue
        r = yaml.safe_load((run / 'analysis.yaml').read_text())
        _, a = load(run)
        ch = a['t'][1:][np.diff(a['pos']) != 0]
        r['update_ms'] = round(float(np.median(np.diff(ch)) * 1000), 1)
        r['run'] = run.name
        out[r['joint']] = r
    return out


def main():
    pa, pb = sys.argv[1:3]
    A, B = session(pa), session(pb)
    joints = [j for j in JOINTS if j in A and j in B]
    x = np.arange(len(joints))
    fig, axs = plt.subplots(2, 3, figsize=(14, 7.2))
    fig.patch.set_facecolor('#fcfcfb')
    for ax, (name, key, f) in zip(axs.flat, PANELS):
        ax.set_facecolor('#fcfcfb')
        for dx, s, c, lab in ((-0.19, A, OLD, pa), (0.19, B, NEW, pb)):
            v = [s[j][key] for j in joints]
            ax.bar(x + dx, v, width=0.38, color=c, edgecolor='#fcfcfb', linewidth=2, label=lab)
            for xi, vi in zip(x + dx, v): ax.text(xi, vi, f.format(vi), ha='center', va='bottom', fontsize=8, color=INK)
        if key == 'gain':
            ax.axhline(1.0, color=MUTED, linewidth=1, linestyle='--')
            ax.set_ylim(0.9, 1.02)
        ax.set_xticks(x, joints, fontsize=9)
        ax.set_title(name, fontsize=10, color=INK, loc='left')
        style(ax)
    fig.legend(*axs[0][0].get_legend_handles_labels(), frameon=False, fontsize=10, labelcolor=INK, loc='upper right', ncol=2)
    fig.suptitle(f'sin_test {pa} vs {pb} — same sine.yaml (full-amplitude part)', x=0.02, ha='left',
                 fontsize=13, color=INK, fontweight='bold')
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    name = f'compare_{pa}_{pb}'
    fig.savefig(LOGS / f'{name}.png', dpi=130)
    plt.close(fig)
    keys = ['run'] + [k for _, k, _ in PANELS] + ['offset_meas', 'max_abs_current_A']
    res = {j: {pa: {k: A[j][k] for k in keys}, pb: {k: B[j][k] for k in keys}} for j in joints}
    for j in sorted(set(B) - set(joints)): res[j] = {pb: {k: B[j][k] for k in keys if k in B[j]}}
    (LOGS / f'{name}.yaml').write_text(yaml.safe_dump(res, sort_keys=False, allow_unicode=True))
    for j in joints:
        print(f"{j}: lag {A[j]['lag_ms']:.0f} -> {B[j]['lag_ms']:.0f} ms, update {A[j]['update_ms']:.0f} -> {B[j]['update_ms']:.0f} ms, "
              f"rms {A[j]['rms_error']:.2f} -> {B[j]['rms_error']:.2f} deg")
    print(f'-> {LOGS / name}.png, {name}.yaml')


if __name__ == '__main__':
    main()
