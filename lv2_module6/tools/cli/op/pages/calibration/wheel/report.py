#!/usr/bin/env python3
"""칼리브레이션 wheel — raw.csv 리포트/비교 (report.png).

  python3 report.py <raw.csv>                         # 한 번 측정 요약 + report.png
  python3 report.py <sim.csv> <real.csv> --labels SIM REAL --out compare.png
  ./run-cli calibrate report                          # 최근 raw.csv 1~2개 자동 선택

읽는 열: t · lin_raw · ang_raw · v_enc · w_gyro · wheel_l/r_vel · odom_x/y/yaw · t_ros/t_odom/t_imu/t_joint
지표: 선속도/각속도 RMSE·bias·p95·gain · 경로 길이 · 메시지 지연 · 두 측정의 궤적 평균 거리
"""
from __future__ import annotations

import argparse
import csv
import logging
import math
import sys
from pathlib import Path

import numpy as np

KOREAN_FONTS = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
]
NUM = ('t', 'lin_raw', 'ang_raw', 'lin_out', 'ang_out', 'odom_x', 'odom_y', 'odom_yaw',
       'v_enc', 'w_enc', 'w_gyro', 'wheel_l_vel', 'wheel_r_vel', 'wheel_l_pos', 'wheel_r_pos',
       't_ros', 't_odom', 't_imu', 't_joint')


def load(path: str) -> list[dict]:
    rows = []
    with open(path, newline='') as f:
        for raw in csv.DictReader(f):
            row = {}
            for key in NUM:
                try:
                    row[key] = float(raw.get(key, ''))
                except (TypeError, ValueError):
                    row[key] = float('nan')
            row['label'] = raw.get('label', '')
            rows.append(row)
    return [r for r in rows if math.isfinite(r['t'])]


def rms(vals) -> float:
    return float(np.sqrt(np.mean(np.square(vals)))) if len(vals) else float('nan')


def p95(vals) -> float:
    return float(np.percentile(np.abs(vals), 95)) if len(vals) else float('nan')


def gain(cmd, resp, floor: float) -> float:
    c = np.asarray([v for v in cmd if abs(v) >= floor], float)
    r = np.asarray([v for v, c0 in zip(resp, cmd) if abs(c0) >= floor], float)
    if not len(c) or float(np.dot(c, c)) == 0.0:
        return float('nan')
    return float(np.dot(c, r) / np.dot(c, c))


def median_age(rows, key: str) -> float:
    vals = [(r['t_ros'] - r[key]) * 1000.0 for r in rows
            if math.isfinite(r.get('t_ros', float('nan'))) and math.isfinite(r.get(key, float('nan')))]
    return float(np.median(vals)) if vals else float('nan')


def metrics(rows: list[dict]) -> dict:
    moving = [r for r in rows if abs(r['lin_raw']) > 1e-9 or abs(r['ang_raw']) > 1e-9]
    ev = [r['lin_raw'] - r['v_enc'] for r in moving if math.isfinite(r['v_enc'])]
    ew = [r['ang_raw'] - r['w_gyro'] for r in moving if math.isfinite(r['w_gyro'])]
    m = {
        'rows': len(rows),
        'duration': rows[-1]['t'] - rows[0]['t'] if rows else 0.0,
        'v_rms': rms(ev), 'v_bias': float(np.mean(ev)) if ev else float('nan'),
        'v_p95': p95(ev), 'v_gain': gain([r['lin_raw'] for r in moving],
                                         [r['v_enc'] for r in moving], 0.02),
        'w_rms': rms(ew), 'w_bias': float(np.mean(ew)) if ew else float('nan'),
        'w_p95': p95(ew), 'w_gain': gain([r['ang_raw'] for r in moving],
                                         [r['w_gyro'] for r in moving], 0.05),
        'age_odom': median_age(rows, 't_odom'),
        'age_imu': median_age(rows, 't_imu'),
        'age_joint': median_age(rows, 't_joint'),
    }
    x = np.asarray([r['odom_x'] for r in rows], float)
    y = np.asarray([r['odom_y'] for r in rows], float)
    if len(x) > 1:
        m['path_len'] = float(np.sum(np.linalg.norm(np.diff(np.column_stack([x, y]), axis=0), axis=1)))
    return m


def aligned_path(rows: list[dict]) -> tuple[np.ndarray, np.ndarray]:
    """시작 위치·yaw를 0으로 맞춘 odom 궤적 — 두 측정을 겹쳐 보기 위해."""
    if not rows:
        return np.zeros(0), np.zeros(0)
    x0, y0 = rows[0]['odom_x'], rows[0]['odom_y']
    yaw0 = rows[0]['odom_yaw']
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    xs, ys = [], []
    for r in rows:
        dx, dy = r['odom_x'] - x0, r['odom_y'] - y0
        xs.append(dx * c - dy * s)
        ys.append(dx * s + dy * c)
    return np.asarray(xs), np.asarray(ys)


def mean_dev(a: tuple[np.ndarray, np.ndarray], b: tuple[np.ndarray, np.ndarray]) -> float:
    n = min(len(a[0]), len(b[0]))
    if n == 0:
        return float('nan')
    return float(np.mean(np.hypot(a[0][:n] - b[0][:n], a[1][:n] - b[1][:n])))


def plot(runs, out: Path) -> None:
    import matplotlib
    matplotlib.use('Agg')
    logging.getLogger('matplotlib.font_manager').setLevel(logging.ERROR)
    from matplotlib import font_manager as fm
    for f in KOREAN_FONTS:
        if Path(f).is_file():
            try:
                fm.fontManager.addfont(f)
            except Exception:
                pass
    import matplotlib.pyplot as plt
    plt.rcParams['font.family'] = ['Noto Sans CJK JP', 'Noto Sans CJK KR', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    colors = ['#1e88e5', '#e53935', '#43a047', '#8e24aa']
    fig, axes = plt.subplots(3, 2, figsize=(13, 10))
    fig.subplots_adjust(left=0.07, right=0.98, top=0.93, bottom=0.14, hspace=0.35, wspace=0.22)
    fig.suptitle('wheel calibration — raw.csv' + (' 비교' if len(runs) > 1 else ''),
                 fontsize=13, fontweight='bold')
    ax_traj, ax_v, ax_w, ax_err, ax_wheel, ax_age = axes.ravel()

    for i, (label, rows) in enumerate(runs):
        c = colors[i % len(colors)]
        t = np.asarray([r['t'] for r in rows], float)
        ax_v.plot(t, [r['lin_raw'] for r in rows], c, ls='--', lw=1.0, alpha=0.8, label=f'{label} cmd')
        ax_v.plot(t, [r['v_enc'] for r in rows], c, lw=1.4, label=f'{label} v_enc')
        ax_w.plot(t, [r['ang_raw'] for r in rows], c, ls='--', lw=1.0, alpha=0.8, label=f'{label} cmd')
        ax_w.plot(t, [r['w_gyro'] for r in rows], c, lw=1.4, label=f'{label} w_gyro')
        ax_err.plot(t, [r['lin_raw'] - r['v_enc'] for r in rows], c, lw=1.0, label=f'{label} v 오차')
        ax_err.plot(t, [r['ang_raw'] - r['w_gyro'] for r in rows], c, ls=':', lw=1.0, label=f'{label} w 오차')
        ax_wheel.plot(t, [r['wheel_l_vel'] for r in rows], c, lw=1.0, label=f'{label} L')
        ax_wheel.plot(t, [r['wheel_r_vel'] for r in rows], c, ls='--', lw=1.0, label=f'{label} R')
        for key, ls in (('t_odom', '-'), ('t_imu', '--'), ('t_joint', ':')):
            age = [(r['t_ros'] - r[key]) * 1000.0 for r in rows]
            ax_age.plot(t, age, c, ls=ls, lw=1.0, label=f'{label} {key[2:]}')
        xs, ys = aligned_path(rows)
        if len(xs):
            ax_traj.plot(xs, ys, c, lw=1.4, label=label)
            ax_traj.plot(xs[0], ys[0], 'ko', ms=4)
            ax_traj.plot(xs[-1], ys[-1], 'r.', ms=8)

    ax_traj.set_title('odom 궤적 (시작 정렬) [m]')
    ax_traj.set_aspect('equal', adjustable='datalim')
    ax_traj.grid(alpha=0.3)
    ax_traj.legend(fontsize=8)
    ax_v.set_title('선속도: 명령 vs 엔코더 [m/s]')
    ax_w.set_title('각속도: 명령 vs 자이로 [rad/s]')
    ax_err.set_title('추종 오차 (명령 − 실측)')
    ax_err.axhline(0, color='k', lw=0.6)
    ax_wheel.set_title('바퀴 각속도 [rad/s]')
    ax_wheel.axhline(0, color='k', lw=0.6)
    ax_age.set_title('메시지 지연 [ms]')
    ax_age.axhline(0, color='k', lw=0.6)
    for ax in (ax_v, ax_w, ax_err, ax_wheel, ax_age):
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, ncol=2)
    ax_wheel.set_xlabel('시간 [s]')
    ax_age.set_xlabel('시간 [s]')

    lines = []
    for label, rows in runs:
        m = metrics(rows)
        lines.append(
            f'{label}: v {m["v_rms"]:.4f} m/s (bias {m["v_bias"]:+.4f}, gain {m["v_gain"]:.3f}) · '
            f'w {m["w_rms"]:.4f} rad/s (bias {m["w_bias"]:+.4f}, gain {m["w_gain"]:.3f}) · '
            f'{m["duration"]:.1f}s · {m.get("path_len", float("nan")):.2f} m · '
            f'지연 odom/imu/joint {m["age_odom"]:.0f}/{m["age_imu"]:.0f}/{m["age_joint"]:.0f} ms')
    if len(runs) == 2:
        d = mean_dev(aligned_path(runs[0][1]), aligned_path(runs[1][1]))
        lines.append(f'두 궤적 평균 거리: {d:.3f} m')
    fig.text(0.01, 0.02, '\n'.join(lines), fontsize=8.5, va='bottom')
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f'저장: {out}')
    for line in lines:
        print(line)


def main() -> int:
    ap = argparse.ArgumentParser(description='wheel calibration raw.csv 리포트/비교')
    ap.add_argument('csv', nargs='+', help='raw.csv 1개(요약) 또는 2개(비교)')
    ap.add_argument('--labels', nargs='*', help='범례 이름 (기본: 폴더 이름)')
    ap.add_argument('--out', default=None, help='출력 PNG (기본: 첫 CSV 옆 report_*.png)')
    a = ap.parse_args()
    if len(a.csv) > 2:
        print('FAIL: CSV는 1개 또는 2개만', file=sys.stderr)
        return 2
    runs = []
    for path in a.csv:
        if not Path(path).is_file():
            print(f'FAIL: CSV 없음: {path}', file=sys.stderr)
            return 2
        runs.append((path, load(path)))
    labels = a.labels or [Path(p).parent.name or Path(p).stem for p, _ in runs]
    named = [(labels[i] if i < len(labels) else Path(p).stem, rows)
             for i, (p, rows) in enumerate(runs)]
    out = Path(a.out).expanduser() if a.out else Path(a.csv[0]).parent / (
        'compare.png' if len(runs) == 2 else 'report.png')
    out.parent.mkdir(parents=True, exist_ok=True)
    plot(named, out)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
