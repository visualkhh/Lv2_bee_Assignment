"""Plot and analyse sin_test logs: one PNG per run + a summary of all runs.

  python3 plot_logs.py                      # every run in ./logs
  python3 plot_logs.py logs/2026..._joint3  # selected runs

Per run (<run>/plot.png): command vs measured position, error, servo current — one shared time axis.
<run>/plot_lag_removed.png: the same with the command delayed by the fitted lag (pure delay compensated), so what is
left is the servo's tracking error itself (gain, offset from gravity, encoder steps).
Analysis on the full-amplitude part (fade-in/out excluded): a sine at the test frequency is fitted to the
command and to the measurement -> gain (measured / commanded amplitude) and lag (phase as time), plus the
error that remains after shifting the command by that lag. Results: <run>/analysis.yaml, logs/summary.png,
logs/summary_lag_removed.png and logs/joints_lag_removed.png (all joints on one page). The summaries use the latest
run of each joint.
"""
import csv
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
CMD, MEAS, ERR, CUR = '#2a78d6', '#eb6834', '#52514e', '#52514e'  # categorical 1, 2; neutral ink
INK, MUTED, GRID = '#0b0b0b', '#52514e', '#e4e3df'


def load(run):
    meta = yaml.safe_load((run / 'meta.yaml').read_text())
    rows = list(csv.DictReader(open(run / 'log.csv')))
    a = {k: np.array([float(r[k]) for r in rows]) for k in ('t', 'cmd_pos', 'pos', 'effort_A')}
    return meta, a


def fit(t, y, w):
    """Least squares y = a sin(wt) + b cos(wt) + c -> amplitude, phase, offset."""
    m = np.column_stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)])
    (a, b, c), *_ = np.linalg.lstsq(m, y, rcond=None)
    return math.hypot(a, b), math.atan2(b, a), c


def analyse(meta, a):
    spec, sine = meta['spec'], meta['sine']
    period, total = spec['period'], sine['total_s']
    ramp = meta['config']['ramp_cycles'] * period
    w = 2 * math.pi / period
    lin = meta['joint'].startswith('gripper')
    k = 1000.0 if lin else 180 / math.pi
    steady = (a['t'] >= ramp) & (a['t'] <= total - ramp)
    whole = (a['t'] >= 0) & (a['t'] <= total)
    t = a['t'][steady]
    amp_c, ph_c, _ = fit(t, a['cmd_pos'][steady], w)
    amp_m, ph_m, off_m = fit(t, a['pos'][steady], w)
    lag = ((ph_c - ph_m) % (2 * math.pi)) / w  # s the measurement trails the command
    # error after removing the lag: compare with the command evaluated lag seconds earlier
    shifted = np.interp(t - lag, a['t'], a['cmd_pos'])
    err = (a['pos'] - a['cmd_pos'])[whole]
    res = a['pos'][steady] - shifted
    return {
        'joint': meta['joint'], 'unit': 'mm' if lin else 'deg', 'period_s': period,
        'amplitude_cmd': round(amp_c * k, 3), 'amplitude_meas': round(amp_m * k, 3),
        'gain': round(amp_m / amp_c, 4), 'lag_ms': round(lag * 1000, 1),
        'phase_deg': round(math.degrees(lag * w), 1),
        'offset_meas': round(off_m * k, 3),
        'peak_error': round(float(np.max(np.abs(err))) * k, 3), 'rms_error': round(float(np.sqrt(np.mean(err ** 2))) * k, 3),
        'rms_error_lag_removed': round(float(np.sqrt(np.mean(res ** 2))) * k, 3),
        'peak_error_lag_removed': round(float(np.max(np.abs(res))) * k, 3),
        'rms_error_lag_offset_removed': round(float(np.sqrt(np.mean((res - res.mean()) ** 2))) * k, 3),
        'max_abs_current_A': round(float(np.max(np.abs(a['effort_A'][whole]))), 3),
    }


def style(ax):
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for s in ('top', 'right'): ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'): ax.spines[s].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.yaxis.label.set_color(MUTED)
    ax.xaxis.label.set_color(MUTED)


def plot_run(run, meta, a, r, compensate=False):
    """compensate: draw the command delayed by the fitted lag (pure delay removed) -> plot_lag_removed.png."""
    k = 1000.0 if r['unit'] == 'mm' else 180 / math.pi
    lag = r['lag_ms'] / 1000 if compensate else 0.0
    cmd = np.interp(a['t'] - lag, a['t'], a['cmd_pos'], left=a['cmd_pos'][0]) if compensate else a['cmd_pos']
    u = r['unit']
    total = meta['sine']['total_s']
    ramp = meta['config']['ramp_cycles'] * r['period_s']
    fig, axs = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True, height_ratios=[3, 1.6, 1.2])
    fig.patch.set_facecolor('#fcfcfb')
    for ax in axs:
        ax.set_facecolor('#fcfcfb')
        ax.axvspan(a['t'].min(), ramp, color=GRID, alpha=0.45, linewidth=0)
        ax.axvspan(total - ramp, total, color=GRID, alpha=0.45, linewidth=0)
        ax.axvspan(total, a['t'].max(), color=GRID, alpha=0.25, linewidth=0)
        style(ax)
    ax = axs[0]
    ax.plot(a['t'], cmd * k, color=CMD, linewidth=2, label=f"command delayed {r['lag_ms']:.0f} ms" if compensate else 'command')
    ax.plot(a['t'], a['pos'] * k, color=MEAS, linewidth=2, label='measured')
    ax.set_ylabel(f'position [{u}]')
    ax.legend(loc='upper right', frameon=False, fontsize=9, labelcolor=INK)
    ax.text(ramp / 2, ax.get_ylim()[1], 'fade-in', ha='center', va='top', fontsize=8, color=MUTED)
    ax.text(total - ramp / 2, ax.get_ylim()[1], 'fade-out', ha='center', va='top', fontsize=8, color=MUTED)
    ax.text(total + (a['t'].max() - total) / 2, ax.get_ylim()[1], 'settle', ha='center', va='top', fontsize=8, color=MUTED)
    ax = axs[1]
    ax.axhline(0, color=MUTED, linewidth=0.8)
    ax.plot(a['t'], (a['pos'] - cmd) * k, color=ERR, linewidth=1.5)
    ax.set_ylabel(f'error [{u}]\nmeas − {"delayed cmd" if compensate else "cmd"}')
    ax = axs[2]
    ax.axhline(0, color=MUTED, linewidth=0.8)
    ax.plot(a['t'], a['effort_A'], color=CUR, linewidth=1.5)
    ax.set_ylabel('current [A]')
    ax.set_xlabel('time since sine start [s]')
    head = f"{r['joint']} — amplitude {r['amplitude_cmd']:.1f} {u}, period {r['period_s']} s"
    fig.suptitle(head + (f" · pure delay {r['lag_ms']:.0f} ms removed" if compensate else ''), x=0.06, ha='left',
                 fontsize=13, color=INK, fontweight='bold')
    if compensate:
        sub = (f"gain {r['gain']:.2f} · offset {r['offset_meas']:+.2f} {u} · full-amplitude part: RMS {r['rms_error_lag_removed']:.2f} {u} "
               f"(offset removed {r['rms_error_lag_offset_removed']:.2f}), peak {r['peak_error_lag_removed']:.2f} {u}  "
               f"[before: RMS {r['rms_error']:.2f}, peak {r['peak_error']:.2f}]")
    else:
        sub = (f"gain {r['gain']:.2f} · lag {r['lag_ms']:.0f} ms ({r['phase_deg']:.0f}°) · "
               f"RMS error {r['rms_error']:.2f} {u} → {r['rms_error_lag_removed']:.2f} {u} with lag removed · "
               f"peak {r['peak_error']:.2f} {u} · max |I| {r['max_abs_current_A']:.2f} A")
    fig.text(0.06, 0.925, sub, fontsize=9.5, color=MUTED, ha='left')
    fig.tight_layout(rect=(0, 0, 1, 0.91))
    fig.savefig(run / ('plot_lag_removed.png' if compensate else 'plot.png'), dpi=130)
    plt.close(fig)


SUMMARY = [('lag [ms]', 'lag_ms', '{:.0f}'), ('gain (meas / cmd amplitude)', 'gain', '{:.2f}'),
           ('RMS error [deg]', 'rms_error', '{:.1f}'), ('RMS error, lag removed [deg]', 'rms_error_lag_removed', '{:.1f}')]
SUMMARY_LAG_REMOVED = [('mean offset (meas − cmd) [deg]', 'offset_meas', '{:+.2f}'),
                       ('RMS error, lag removed [deg]', 'rms_error_lag_removed', '{:.2f}'),
                       ('RMS error, lag + offset removed [deg]', 'rms_error_lag_offset_removed', '{:.2f}'),
                       ('peak error, lag removed [deg]', 'peak_error_lag_removed', '{:.2f}')]


def plot_summary(results, out, panels=SUMMARY, title='sin_test summary (full-amplitude part)'):
    names = [r['joint'] for r in results]
    x = np.arange(len(names))
    fig, axs = plt.subplots(1, len(panels), figsize=(13, 3.6))
    fig.patch.set_facecolor('#fcfcfb')
    for ax, (name, key, f) in zip(axs, panels):
        ax.set_facecolor('#fcfcfb')
        v = [r[key] for r in results]
        ax.bar(x, v, width=0.6, color=CMD, edgecolor='#fcfcfb', linewidth=2)
        for xi, vi in zip(x, v): ax.text(xi, vi, f.format(vi), ha='center', va='bottom', fontsize=8.5, color=INK)
        if key == 'gain': ax.axhline(1.0, color=MUTED, linewidth=1, linestyle='--')
        if min(v) < 0: ax.axhline(0, color=MUTED, linewidth=0.8)
        ax.set_xticks(x, names, fontsize=9)
        ax.set_title(name, fontsize=10, color=INK, loc='left')
        style(ax)
    fig.suptitle(title, x=0.02, ha='left', fontsize=12, color=INK, fontweight='bold')
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def plot_joints(items, out):
    """One page: per joint (latest run) delayed command vs measured, and the remaining error."""
    fig, axs = plt.subplots(len(items), 2, figsize=(14, 2.6 * len(items)), width_ratios=[3, 2], squeeze=False)
    fig.patch.set_facecolor('#fcfcfb')
    for row, (meta, a, r) in zip(axs, items):
        k = 1000.0 if r['unit'] == 'mm' else 180 / math.pi
        u, lag = r['unit'], r['lag_ms'] / 1000
        total = meta['sine']['total_s']
        ramp = meta['config']['ramp_cycles'] * r['period_s']
        cmd = np.interp(a['t'] - lag, a['t'], a['cmd_pos'], left=a['cmd_pos'][0])
        for ax in row:
            ax.set_facecolor('#fcfcfb')
            ax.axvspan(a['t'].min(), ramp, color=GRID, alpha=0.45, linewidth=0)
            ax.axvspan(total - ramp, total, color=GRID, alpha=0.45, linewidth=0)
            ax.axvspan(total, a['t'].max(), color=GRID, alpha=0.25, linewidth=0)
            style(ax)
        ax = row[0]
        ax.plot(a['t'], cmd * k, color=CMD, linewidth=2, label='command (delayed)')
        ax.plot(a['t'], a['pos'] * k, color=MEAS, linewidth=1.6, label='measured')
        ax.set_ylabel(f'position [{u}]')
        ax.set_title(f"{r['joint']} — ±{r['amplitude_cmd']:.1f} {u}, period {r['period_s']} s · delay {r['lag_ms']:.0f} ms removed · "
                     f"gain {r['gain']:.2f}", fontsize=10.5, color=INK, loc='left', fontweight='bold')
        ax = row[1]
        ax.axhline(0, color=MUTED, linewidth=0.8)
        ax.plot(a['t'], (a['pos'] - cmd) * k, color=ERR, linewidth=1.2)
        ax.set_ylabel(f'meas − delayed cmd [{u}]')
        ax.set_title(f"offset {r['offset_meas']:+.2f} · RMS {r['rms_error_lag_removed']:.2f} "
                     f"(offset removed {r['rms_error_lag_offset_removed']:.2f}) · peak {r['peak_error_lag_removed']:.2f} {u}",
                     fontsize=9.5, color=MUTED, loc='left')
    axs[0][0].legend(loc='upper right', frameon=False, fontsize=9, labelcolor=INK)
    for ax in axs[-1]: ax.set_xlabel('time since sine start [s]')
    fig.suptitle('sin_test — command delayed by each joint\'s pure delay vs measured (grey: fade-in / fade-out / settle)',
                 x=0.01, ha='left', fontsize=13, color=INK, fontweight='bold')
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    fig.savefig(out, dpi=120)
    plt.close(fig)


def main():
    runs = [Path(p) for p in sys.argv[1:]] or sorted(p for p in (HERE / 'logs').iterdir() if (p / 'log.csv').exists())
    results, latest = [], {}
    for run in runs:
        meta, a = load(run)
        r = analyse(meta, a)
        plot_run(run, meta, a, r)
        plot_run(run, meta, a, r, compensate=True)
        r = {key: float(v) if isinstance(v, (float, np.floating)) else v for key, v in r.items()}
        (run / 'analysis.yaml').write_text(yaml.safe_dump(r, sort_keys=False))
        results.append(r)
        latest[r['joint']] = (meta, a, r)  # runs are sorted by time -> the last one wins
        print(f"{r['joint']:20s} gain {r['gain']:.3f}  lag {r['lag_ms']:6.1f} ms  rms {r['rms_error']:6.2f} -> {r['rms_error_lag_removed']:5.2f} {r['unit']}"
              f"  peak {r['peak_error']:6.2f}  offset {r['offset_meas']:+.2f}  max|I| {r['max_abs_current_A']:.2f} A  -> {run / 'plot.png'}")
    if len(latest) > 1:
        items = [latest[j] for j in sorted(latest)]
        res = [r for _, _, r in items]
        out = runs[0].parent
        plot_summary(res, out / 'summary.png')
        plot_summary(res, out / 'summary_lag_removed.png', SUMMARY_LAG_REMOVED,
                     'sin_test summary — pure delay removed (full-amplitude part)')
        plot_joints(items, out / 'joints_lag_removed.png')
        print(f'summary (latest run per joint) -> {out}/summary.png, summary_lag_removed.png, joints_lag_removed.png')


if __name__ == '__main__':
    main()
