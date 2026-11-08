#!/usr/bin/env python3
"""손 안 퍽 위치를 gripper_cam 영상에서 환산하는 보정표 — 정확도 측정 (pick_test.py --save-images 폴더).
  python3 held_puck_fit.py <폴더>   폴더 = samples.jsonl + seedNNNN.png (파지 직후 영상 · Isaac 참값)
  퍽을 색(빨강·초록)으로 분할 → 중심 (u, v) · 높이 · 폭 · 넓이 → 손 기준 오프셋 [mm]을 선형(릿지)으로 맞추고 leave-one-out 오차를 냄.
  들린 샘플(손 값 있음)만 쓴다. 분할이 안 되는 영상(퍽이 안 보임)은 따로 센다.
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

TARGETS = ('finger_off', 'close_off', 'face_above_bottom')  # 손가락 방향 · 닫힘 방향 · 손끝 끝면의 퍽 바닥 위 높이 (참값 m)
MIN_AREA_PX = 400


def segment(img, color):
    """퍽 색 마스크의 가장 큰 덩어리 → (u, v, w, h, area) 또는 None."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    if color == 'red':
        m = cv2.inRange(hsv, (0, 120, 50), (8, 255, 255)) | cv2.inRange(hsv, (172, 120, 50), (180, 255, 255))
    else:
        m = cv2.inRange(hsv, (45, 100, 40), (85, 255, 255))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, _, stats, cen = cv2.connectedComponentsWithStats(m)
    if n < 2:
        return None
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x, y, w, h, area = stats[k]
    return None if area < MIN_AREA_PX else (float(cen[k][0]), float(cen[k][1]), float(w), float(h), float(area))


def ridge_loo(X, Y, lam=1e-3):
    """leave-one-out 예측 오차 [Y와 같은 단위] — 입력은 표준화."""
    mu, sd = X.mean(0), X.std(0) + 1e-9
    Z = np.c_[(X - mu) / sd, np.ones(len(X))]
    err = []
    for i in range(len(X)):
        keep = np.arange(len(X)) != i
        A = Z[keep]
        W = np.linalg.solve(A.T @ A + lam * np.eye(A.shape[1]), A.T @ Y[keep])
        err.append(Z[i] @ W - Y[i])
    return np.array(err)


def main(folder):
    folder = Path(folder)
    rows = [json.loads(l) for l in open(folder / 'samples.jsonl')]
    feats, ys, miss, no_hand = [], [], 0, 0
    for r in rows:
        if r['lift'] < 0.02 or not r.get('hand'):
            no_hand += 1
            continue
        img = cv2.imread(str(folder / r['file']))
        f = segment(img, 'red' if '_red_' in r['target'] else 'green')
        if f is None:
            miss += 1
            continue
        feats.append(f)
        ys.append([1000 * r['hand'][t] for t in TARGETS])
    print(f'샘플 {len(rows)}: 들리지 않았거나 손 값 없음 {no_hand}, 퍽 분할 실패 {miss}, 사용 {len(feats)}')
    if not feats:
        return
    X, Y = np.array(feats), np.array(ys)
    print('특징 (u, v, w, h, 넓이) 평균', X.mean(0).round(1), ' 표준편차', X.std(0).round(1))
    for j, t in enumerate(TARGETS):
        print(f'{t}: 참값 평균 {Y[:, j].mean():.1f} mm, 표준편차 {Y[:, j].std():.1f} mm')
    if len(X) < 8:
        print('샘플이 8개 미만 — 맞추지 않음')
        return
    for j, t in enumerate(TARGETS):
        e = ridge_loo(X, Y[:, j])
        print(f'{t}: LOO 오차 평균절대 {np.abs(e).mean():.2f} mm · p90 {np.percentile(np.abs(e), 90):.2f} mm '
              f'(아무것도 안 쓴 기준 = 참값 표준편차 {Y[:, j].std():.2f} mm)')


if __name__ == '__main__':
    main(sys.argv[1])
