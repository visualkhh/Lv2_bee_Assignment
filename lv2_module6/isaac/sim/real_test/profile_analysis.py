#!/usr/bin/env python3
"""Analyse profile_test logs (logs/profile): hold / ramp / step.

  python3 profile_analysis.py        # latest complete run per joint and mode

Per run: <run>/plot.png (command vs measured, error, current) and <run>/analysis.yaml.
  hold: per dwell angle, mean sag (measured - command) and current over the second half of the dwell, with the
        direction the angle was approached from (friction makes it matter).
  ramp: constant-speed parts only (2 s after each reversal skipped, 40 % of a shorter sweep). err+/err- = mean (measured - command) on the
        + / - sweep, I+/I- the mean current. hysteresis = err- - err+ (includes 2 * speed * delay), friction current
        = |I+ - I-| / 2, gravity current = (I+ + I-) / 2 (friction cancels). Gravity torque = current x KT and
        the same torque from the URDF masses (xacro) averaged over the swing.
  step: per command edge: peak servo speed, dead time (command edge -> 10 %), rise 10 -> 90 %, overshoot, final
        error (last 0.4 s). Times are coarse: the state only changes every ~60 ms.
Summary: logs/profile/profile_summary.yaml and logs/profile/ramp_hysteresis.png (error vs command, j2-4).
"""
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

from plot_logs import CMD, CUR, ERR, GRID, INK, MEAS, MUTED, style

HERE = Path(__file__).resolve().parent
ROOT = HERE / 'logs' / 'profile'
URDF = HERE.parents[2] / 'urdf' / 'beebot_mesh' / 'urdf.xacro'
KT = 4.1 / 2.3  # Nm/A, XM430-W350 stall torque / stall current at 12 V — only a rough current -> torque factor
DEG = 180 / math.pi
LINEAR = {'gripper_left_joint'}


def units(joint):
    """(scale from SI, unit name): arm deg, gripper mm. Config values are deg (arm) or m (gripper)."""
    return (1000.0, 'mm') if joint in LINEAR else (DEG, 'deg')


COLOR = {'joint2': '#eb6834', 'joint3': '#1baf7a', 'joint4': '#eda100'}  # same slots as freq_response.py


def load(run):
    meta = yaml.safe_load((run / 'meta.yaml').read_text())
    d = np.genfromtxt(run / 'log.csv', delimiter=',', names=True, dtype=None, encoding=None)
    a = {k: np.asarray(d[k], float) for k in ('t', 'cmd_pos', 'cmd_vel', 'pos', 'vel', 'effort_A')}
    return meta, a


# --- URDF gravity torque (joint1 = 0, the tested joint's axis is horizontal) ---
def urdf_gravity():
    """Returns tau(q, joint) -> gravity torque [Nm] about the joint axis (+ = pulls toward + angle), or None."""
    try:
        root = ET.fromstring(subprocess.run(['xacro', str(URDF)], capture_output=True, text=True, check=True).stdout)
    except (OSError, subprocess.CalledProcessError, ET.ParseError):
        return None
    links = {l.get('name'): l for l in root.iter('link')}
    kids = {}
    for j in root.iter('joint'):
        if j.find('parent') is not None: kids.setdefault(j.find('parent').get('link'), []).append(j)
    vec = lambda s: np.array([float(x) for x in s.split()])

    def rot(ax, t):
        c, s = math.cos(t), math.sin(t)
        if ax[1]: return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        if ax[0]: return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    def child(j, R, p, q):
        pj = p + R @ vec(j.find('origin').get('xyz'))
        if j.get('type') == 'revolute': R = R @ rot(vec(j.find('axis').get('xyz')), q.get(j.get('name'), 0.0))
        return R, pj

    def masses(link, R, p, q, out):
        i = links[link].find('inertial')
        if i is not None:
            o = i.find('origin')
            out.append((float(i.find('mass').get('value')), p + R @ (vec(o.get('xyz')) if o is not None else 0)))
        for j in kids.get(link, []):
            masses(j.find('child').get('link'), *child(j, R, p, q), q, out)

    def find(link, R, p, q, name):
        for j in kids.get(link, []):
            Rj, pj = child(j, R, p, q)
            if j.get('name') == name: return j, Rj, pj
            f = find(j.find('child').get('link'), Rj, pj, q, name)
            if f: return f

    def tau(q, name):
        j, R, p = find('arm_base_link', np.eye(3), np.zeros(3), q, name)
        out = []
        masses(j.find('child').get('link'), R, p, q, out)
        return float(9.81 * sum(m * (c[0] - p[0]) for m, c in out))  # rotation about +y: + torque leans forward
    return tau


# --- per mode ---
def segments(cmd):
    """Index ranges where the command is constant (dwell) — [(i0, i1)]."""
    edges = np.flatnonzero(np.abs(np.diff(cmd)) > 1e-7) + 1
    bounds = np.concatenate([[0], edges, [len(cmd)]])
    return [(int(i0), int(i1)) for i0, i1 in zip(bounds[:-1], bounds[1:])]


def hold(meta, a, tau):
    c = meta['sine']['center']
    dwell = meta['spec']['hold'].get('dwell_s', 2.0)
    k, u = units(meta['joint'])
    rows, prev = [], c
    for i0, i1 in segments(a['cmd_pos']):
        if a['t'][i1 - 1] - a['t'][i0] < 0.8 * dwell: continue  # moves and the final settle are shorter
        h = slice(i0 + (i1 - i0) // 2, i1)
        q = float(np.mean(a['cmd_pos'][h]))
        cur = float(np.mean(a['effort_A'][h]))
        r = {f'target_{u}': round(q * k, 2), 'approach': '+' if q > prev else '-',
             f'measured_{u}': round(float(np.mean(a['pos'][h])) * k, 3),
             f'sag_{u}': round(float(np.mean(a['pos'][h] - a['cmd_pos'][h])) * k, 3), 'current_A': round(cur, 4)}
        if meta['joint'] not in LINEAR: r['torque_Nm'] = round(-cur * KT, 3)
        if tau and meta['joint'] not in LINEAR: r['urdf_torque_Nm'] = round(tau({meta['joint']: q}, meta['joint']), 3)
        rows.append(r)
        prev = q
    return {'dwell': rows}


def ramp(meta, a, tau):
    k, u = units(meta['joint'])
    v = meta['spec']['ramp']['speed'] * (1.0 if meta['joint'] in LINEAR else 1 / DEG)  # SI
    m = a['t'] > 0
    out = {}
    for sgn, key in ((1, '+'), (-1, '-')):
        sel = m & (sgn * a['cmd_vel'] > 0.5 * v)
        idx = np.flatnonzero(sel)
        blocks = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
        skip = lambda b: min(2.0, 0.4 * (a['t'][b[-1]] - a['t'][b[0]]))  # settle after a reversal: 2 s, less for short sweeps
        keep = np.concatenate([b[a['t'][b] > a['t'][b[0]] + skip(b)] for b in blocks if len(b)])
        out[key] = (float(np.mean(a['pos'][keep] - a['cmd_pos'][keep])) * k, float(np.mean(a['effort_A'][keep])))
    (ep, ip), (en, i_n) = out['+'], out['-']
    g_cur = (ip + i_n) / 2
    r = {f'speed_{u}_s': round(v * k, 3), f'err_plus_{u}': round(ep, 3), f'err_minus_{u}': round(en, 3),
         f'hysteresis_{u}': round(en - ep, 3), f'delay_part_{u}': round(2 * v * k * 0.1, 3),
         f'mean_offset_{u}': round((ep + en) / 2, 3),
         'current_plus_A': round(ip, 4), 'current_minus_A': round(i_n, 4),
         'friction_current_A': round(abs(ip - i_n) / 2, 4)}
    if meta['joint'] not in LINEAR:
        r |= {'friction_torque_Nm': round(abs(ip - i_n) / 2 * KT, 3), 'gravity_current_A': round(g_cur, 4),
              'gravity_torque_Nm': round(-g_cur * KT, 3)}
    if tau and meta['joint'] not in LINEAR:
        c, amp = meta['sine']['center'], meta['spec']['ramp']['amplitude'] / DEG
        r['urdf_gravity_torque_Nm'] = round(float(np.mean([tau({meta['joint']: q}, meta['joint'])
                                                          for q in np.linspace(c - amp, c + amp, 31)])), 3)
        r['measured_over_urdf'] = round(r['gravity_torque_Nm'] / r['urdf_gravity_torque_Nm'], 3)
    return r


def step(meta, a, tau):
    k, u = units(meta['joint'])
    cmd, t, p = a['cmd_pos'], a['t'], a['pos']
    edges = np.flatnonzero(np.abs(np.diff(cmd)) > 1e-3) + 1
    rows = []
    for e, i in enumerate(edges):
        j = edges[e + 1] if e + 1 < len(edges) else len(cmd)
        p0, p1 = cmd[i - 1], cmd[i]
        n, tt = (p[i:j] - p0) / (p1 - p0), t[i:j] - t[i]
        hit = lambda x: float(tt[np.argmax(n >= x)]) if np.any(n >= x) else None
        t10, t90 = hit(0.1), hit(0.9)
        tail = tt > tt[-1] - 0.4
        rows.append({f'size_{u}': round((p1 - p0) * k, 2), f'peak_speed_{u}_s': round(float(np.max(np.abs(a['vel'][i:j]))) * k, 1),
                     'dead_ms': None if t10 is None else round(t10 * 1000), 'rise_10_90_ms': None if None in (t10, t90) else round((t90 - t10) * 1000),
                     f'overshoot_{u}': round(max(0.0, float(n.max()) - 1) * abs(p1 - p0) * k, 3),
                     f'final_error_{u}': round(float(np.mean(p[i:j][tail] - p1)) * k, 3)})
    size = max(abs(r[f'size_{u}']) for r in rows)  # the largest step size (30 deg / 10 mm)
    big = [r for r in rows if abs(r[f'size_{u}']) > 0.9 * size]
    return {'edges': rows, f'largest_step_{u}': round(size, 2), f'peak_speed_largest_{u}_s': max(r[f'peak_speed_{u}_s'] for r in big),
            f'overshoot_largest_{u}': max(r[f'overshoot_{u}'] for r in big)}


ANALYSE = {'hold': hold, 'ramp': ramp, 'step': step}


def plain(x):
    """numpy scalars -> python (yaml)."""
    if isinstance(x, dict): return {k: plain(v) for k, v in x.items()}
    if isinstance(x, list): return [plain(v) for v in x]
    return x.item() if isinstance(x, np.generic) else x


def headline(joint, mode, r):
    _, u = units(joint)
    if mode == 'ramp':
        s = (f"hysteresis {r[f'hysteresis_{u}']:.2f} {u} (delay part {r[f'delay_part_{u}']:.2f}) · friction {r['friction_current_A']:.3f} A")
        if joint in LINEAR: return s
        return (s + f" ≈ {r['friction_torque_Nm']:.2f} Nm · gravity {r['gravity_torque_Nm']:.2f} Nm"
                + (f" (URDF {r['urdf_gravity_torque_Nm']:.2f})" if 'urdf_gravity_torque_Nm' in r else ''))
    if mode == 'step':
        return (f"{r[f'largest_step_{u}']:g} {u} steps: peak speed {r[f'peak_speed_largest_{u}_s']:.0f} {u}/s · "
                f"overshoot up to {r[f'overshoot_largest_{u}']:.2f} {u}")
    return ' · '.join(f"{d[f'target_{u}']:+.1f} ({d['approach']}) sag {d[f'sag_{u}']:+.2f} {d['current_A']:+.2f} A" for d in r['dwell'])


def plot_run(run, meta, a, mode, r):
    fig, axs = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True, height_ratios=[3, 1.6, 1.2])
    fig.patch.set_facecolor('#fcfcfb')
    for ax in axs:
        ax.set_facecolor('#fcfcfb')
        style(ax)
    k, u = units(meta['joint'])
    axs[0].plot(a['t'], a['cmd_pos'] * k, color=CMD, lw=2, label='command')
    axs[0].plot(a['t'], a['pos'] * k, color=MEAS, lw=2, label='measured')
    axs[0].set_ylabel(f'position [{u}]')
    axs[0].legend(loc='upper right', frameon=False, fontsize=9, labelcolor=INK)
    for ax, y, lab in ((axs[1], (a['pos'] - a['cmd_pos']) * k, f'error [{u}]\nmeas − cmd'), (axs[2], a['effort_A'], 'current [A]')):
        ax.axhline(0, color=MUTED, lw=0.8)
        ax.plot(a['t'], y, color=ERR if ax is axs[1] else CUR, lw=1.5)
        ax.set_ylabel(lab)
    axs[2].set_xlabel('time since profile start [s]')
    fig.suptitle(f"{meta['joint']} — {mode}", x=0.06, ha='left', fontsize=13, color=INK, fontweight='bold')
    fig.text(0.06, 0.925, headline(meta['joint'], mode, r), fontsize=9, color=MUTED, ha='left', wrap=True)
    fig.tight_layout(rect=(0, 0, 1, 0.91))
    fig.savefig(run / 'plot.png', dpi=130)
    plt.close(fig)


def plot_hysteresis(items, out):
    """ramp: error vs command position — the loop width is friction/backlash (+ the small delay part)."""
    fig, ax = plt.subplots(figsize=(8, 5))
    fig.patch.set_facecolor('#fcfcfb')
    ax.set_facecolor('#fcfcfb')
    style(ax)
    for j, (meta, a, r) in items:
        m = a['t'] > 0
        ax.plot(a['cmd_pos'][m] * DEG, (a['pos'][m] - a['cmd_pos'][m]) * DEG, color=COLOR.get(j, INK), lw=1.5,
                label=f"{j}: width {r['hysteresis_deg']:.1f}°, friction {r['friction_current_A']:.2f} A")
    ax.set(xlabel='command [deg]', ylabel='measured − command [deg]')
    ax.legend(frameon=False, fontsize=9, labelcolor=INK, loc='upper left')
    ax.set_title(f"ramp {items[0][1][0]['spec']['ramp']['speed']}°/s — upper branch: moving −, lower: moving +",
                 color=INK, fontsize=11, loc='left')
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def main():
    tau = urdf_gravity()
    if tau is None: print('xacro not available (source ROS) — URDF gravity comparison skipped')
    latest = {}
    for run in sorted(p for p in ROOT.iterdir() if (p / 'meta.yaml').exists()):  # sorted by time -> last wins
        s = yaml.safe_load((run / 'summary.yaml').read_text()) if (run / 'summary.yaml').exists() else {}
        if s.get('interrupted', True) or not any(v for k, v in s.items() if k != 'interrupted'): continue  # stopped or empty
        meta = yaml.safe_load((run / 'meta.yaml').read_text())
        latest[(meta['joint'], meta['mode'] + (f"_{meta['config']['tag']}" if meta['config'].get('tag') else ''))] = run
    summary, ramps = {}, []
    for (j, name), run in sorted(latest.items(), key=lambda kv: (kv[0][1], kv[0][0])):
        meta, a = load(run)
        mode = meta['mode']
        r = plain(ANALYSE[mode](meta, a, tau))
        (run / 'analysis.yaml').write_text(yaml.safe_dump(r, sort_keys=False))
        plot_run(run, meta, a, mode, r)
        summary.setdefault(name, {})[j] = {'run': run.name} | ({k: v for k, v in r.items() if k != 'edges'})
        if mode == 'ramp' and j not in LINEAR: ramps.append((j, (meta, a, r)))
        print(f'{j} {name:4s} {headline(j, mode, r)}')
    (ROOT / 'profile_summary.yaml').write_text(yaml.safe_dump(summary, sort_keys=False, allow_unicode=True))
    if ramps: plot_hysteresis(ramps, ROOT / 'ramp_hysteresis.png')
    print(f'-> {ROOT}/profile_summary.yaml, ramp_hysteresis.png, <run>/plot.png + analysis.yaml')


if __name__ == '__main__':
    main()
