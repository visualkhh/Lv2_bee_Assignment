#!/usr/bin/env python3
"""Frequency sweep summary (run plot_logs.py on logs/freq first, it writes each run's analysis.yaml).

  python3 freq_response.py

Per joint, the runs in logs/freq (same ±10 deg amplitude, period 0.5 / 1 / 2 / 4 s) give gain and lag vs period.
Lag is fitted as lag(T) = delay + phase/360 * T: a pure time delay (same at every period) plus a constant phase lag
(hysteresis: friction / backlash, grows in ms as the period gets longer). The large-amplitude 2 / 2.5 s runs in
logs/ are drawn as hollow markers (amplitude check). Output: logs/freq/freq_response.png, freq_response.yaml.
"""
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
FREQ = HERE / 'logs' / 'freq'
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
COLOR = dict(zip(JOINTS, ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4']))  # categorical slots 1-5
INK, MUTED, GRID = '#0b0b0b', '#52514e', '#e4e3df'


def runs(root):
    out = {}
    for f in sorted(root.glob('*/analysis.yaml')):  # sorted by time -> a repeated period keeps the last run
        r = yaml.safe_load(f.read_text())
        if r['joint'] in JOINTS: out.setdefault(r['joint'], {})[r['period_s']] = r
    return out


def main():
    sweep, big = runs(FREQ), runs(HERE / 'logs')
    res = {}
    fig, (ax_g, ax_l) = plt.subplots(1, 2, figsize=(12, 4.8))
    for j in JOINTS:
        if j not in sweep: continue
        T = np.array(sorted(sweep[j]))
        g = np.array([sweep[j][t]['gain'] for t in T])
        lag = np.array([sweep[j][t]['lag_ms'] for t in T])
        k, d = np.polyfit(T, lag, 1)  # lag_ms = d + k * T[s]
        res[j] = {'delay_ms': round(float(d), 1), 'hysteresis_phase_deg': round(float(k / 1000 * 360), 1),
                  'gain': {float(t): round(float(x), 3) for t, x in zip(T, g)},
                  'lag_ms': {float(t): round(float(x), 1) for t, x in zip(T, lag)}}
        c = COLOR[j]
        ax_g.plot(T, g, '-o', color=c, lw=2, ms=8, mec='white', mew=2)
        ax_l.plot(T, lag, 'o', color=c, ms=8, mec='white', mew=2)
        ts = np.linspace(0.4, 4.2, 50)
        ax_l.plot(ts, d + k * ts, '--', color=c, lw=1.5)
        for t, r in big.get(j, {}).items():
            ax_g.plot(t, r['gain'], 'o', mfc='white', mec=c, mew=2, ms=8)
            ax_l.plot(t, r['lag_ms'], 'o', mfc='white', mec=c, mew=2, ms=8)
        ax_g.annotate(j, (T[-1], g[-1]), xytext=(8, {'joint5': 5, 'joint1': -5}.get(j, 0)), textcoords='offset points', va='center', color=INK, fontsize=9)
        ax_l.annotate(f'{j}  {d:.0f} ms + {k / 1000 * 360:.1f}°', (4.2, d + k * 4.2), xytext=(6, 0),
                      textcoords='offset points', va='center', color=INK, fontsize=9)
    ax_g.set(title='Gain (measured / commanded amplitude)', xlabel='period [s]', ylabel='gain', xlim=(0.3, 4.7))
    ax_l.set(title='Lag  —  dashed: delay + constant phase fit', xlabel='period [s]', ylabel='lag [ms]', xlim=(0.3, 5.9))
    for ax in (ax_g, ax_l):
        ax.grid(color=GRID, lw=0.8)
        ax.set_axisbelow(True)
        for s in ('top', 'right'): ax.spines[s].set_visible(False)
        for s in ('left', 'bottom'): ax.spines[s].set_color(MUTED)
        ax.tick_params(colors=MUTED)
        ax.title.set_color(INK)
    ax_g.plot([], [], 'o', color=MUTED, ms=8, label='±10° sweep')
    ax_g.plot([], [], 'o', mfc='white', mec=MUTED, mew=2, ms=8, label='large amplitude (±22–54°)')
    ax_g.legend(frameon=False, loc='lower right', labelcolor=MUTED)
    fig.suptitle('Arm frequency response — same ±10° amplitude at every period', color=INK)
    fig.tight_layout()
    fig.savefig(FREQ / 'freq_response.png', dpi=130)
    (FREQ / 'freq_response.yaml').write_text(yaml.safe_dump(res, sort_keys=False))
    for j, r in res.items():
        print(f"{j}: delay {r['delay_ms']} ms + phase {r['hysteresis_phase_deg']} deg | gain {r['gain']}")


if __name__ == '__main__':
    main()
