"""감지 물체 보기 창 — 두 카메라 융합 결과(/tracked_objects)와 융합 전 카메라별 검출(/perception/<cam>/detections)

  표: 융합 트랙 한 줄 + 그 트랙에 붙은 카메라별 검출(하위 줄) — 위치 · σ(공분산) · 융합 위치와의 차이
  확대도: 고른 트랙의 2σ 타원 — 융합(보라) · pole_cam(주황) · gripper_cam(분홍). 지도에선 몇 픽셀이라 여기서 cm 단위로 크게
  카메라: 두 카메라 ROS 영상 + 인지 박스 (메인 RGB-SEG 칸과 같은 그림)
지도·RGB-SEG에 그리는 것과 같은 파일(debug/output/topic)을 읽는다 — 0.5초마다 갱신.
"""
import base64
import math
import time
import tkinter as tk
from tkinter import ttk

import cv2
import numpy as np

from bridge import TOPIC_DIR, read_msg
from perception_view import FUSED_COLOR

REFRESH_MS = 500
ZOOM_PX = 300   # 확대도 한 변 [px]
CAM_TILE = (320, 240)


def pos_of(o):
    p = o['pose']['pose']['position']
    return np.array([p['x'], p['y'], p['z']], float)


def cov_of(o):
    """pose.covariance 위치 3×3 — 없거나 0이면 None (이전 perception은 비워 둠)"""
    try:
        c = np.array(o['pose']['covariance'], float).reshape(6, 6)[:3, :3]
    except (KeyError, TypeError, ValueError):
        return None
    return c if np.all(np.isfinite(c)) and c[0, 0] > 0 and c[1, 1] > 0 else None


def sigma_cm(c):
    return '-' if c is None else ' '.join(f'{math.sqrt(max(v, 0.0)) * 100:.1f}' for v in np.diag(c))


def ellipse(img, center_px, cov2, scale, color, thick=1):
    """xy 공분산 2×2 → 2σ 타원 (화면 y는 위아래 반대)"""
    w, v = np.linalg.eigh(cov2)
    r = np.sqrt(np.clip(w, 0, None)) * 2.0 * scale
    ang = -math.degrees(math.atan2(v[1, 1], v[0, 1]))
    cv2.ellipse(img, center_px, (max(1, int(r[1])), max(1, int(r[0]))), ang, 0, 360, color, thick)


class ObjectsWindow:
    def __init__(self, root, sim, overlay, camera_detections, cam_colors, fresh_s, localization_error=None):
        self.sim = sim
        self.overlay = overlay                  # main.overlay_tracked(rgb, sim, cam, t_world_cam)
        self.camera_detections = camera_detections
        self.cam_colors = cam_colors
        self.fresh_s = fresh_s
        self.localization_error = localization_error  # () → (dx, dy, dyaw) 로봇이 아는 위치 − 실제 위치, 없으면 None
        self.selected = None                    # 고른 융합 트랙 id (갱신해도 유지)
        self.photos = {}

        self.win = tk.Toplevel(root)
        self.win.title('감지 물체 보기')
        self.win.geometry('980x720')
        self.meta = tk.Label(self.win, text='', anchor=tk.W, justify=tk.LEFT)
        self.meta.pack(fill=tk.X, padx=4, pady=(4, 0))

        cols = ('class', 'color', 'cams', 'x', 'y', 'z', 'sigma', 'diff', 'size', 'conf', 'age')
        heads = ('클래스', '색', '카메라', 'x [m]', 'y [m]', 'z [m]', 'σ x·y·z [cm]', '융합과 차이 [cm]', '크기 [cm]',
                 '신뢰도', '최근 [s]')
        widths = (90, 60, 130, 60, 60, 60, 120, 110, 110, 55, 60)
        self.tree = ttk.Treeview(self.win, columns=cols, height=10)
        self.tree.heading('#0', text='id')
        self.tree.column('#0', width=80, stretch=False)
        for c, h, w in zip(cols, heads, widths):
            self.tree.heading(c, text=h)
            self.tree.column(c, width=w, anchor=tk.E if c in ('x', 'y', 'z', 'conf', 'age') else tk.W)
        self.tree.tag_configure('fused', foreground='#7b1fa2')
        self.tree.tag_configure('pole_cam', foreground='#e65100')
        self.tree.tag_configure('gripper_cam', foreground='#ad1457')
        self.tree.pack(fill=tk.X, padx=4, pady=4)
        self.tree.bind('<<TreeviewSelect>>', self.on_select)

        bottom = tk.Frame(self.win)
        bottom.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        zbox = tk.Frame(bottom)
        zbox.pack(side=tk.LEFT, anchor=tk.N)
        tk.Label(zbox, text='확대도 (위에서 본 x·y, 2σ 타원)').pack(anchor=tk.W)
        self.zoom_label = tk.Label(zbox)
        self.zoom_label.pack()
        self.cam_labels = {}
        for cam in cam_colors:
            box = tk.Frame(bottom)
            box.pack(side=tk.LEFT, anchor=tk.N, padx=(6, 0))
            tk.Label(box, text=f'{cam} (ROS 영상 + 인지 박스, F = 융합)').pack(anchor=tk.W)
            self.cam_labels[cam] = tk.Label(box)
            self.cam_labels[cam].pack()
        self.refresh()

    def on_select(self, _event):
        sel = self.tree.selection()
        if sel:
            iid = sel[0]
            self.selected = int((self.tree.parent(iid) or iid)[1:])  # 't<id>' 또는 그 하위 줄

    def refresh(self):
        if not self.win.winfo_exists():
            return
        try:
            self.update()
        except tk.TclError:
            return
        self.win.after(REFRESH_MS, self.refresh)

    def update(self):
        data, age = read_msg('tracked_objects')
        fresh = age is not None and age <= self.fresh_s
        tracks = (data or {}).get('objects', []) if fresh else []
        dets = self.camera_detections()
        by_id = {}
        for cam, objs in dets.items():
            for o in objs:
                by_id.setdefault(o.get('id'), []).append((cam, o))
        n_fused = sum(1 for o in tracks if len({c for c, _ in by_id.get(o['id'], [])}) > 1)
        self.meta.config(text=(
            f"/tracked_objects  {'%.1f초 전' % age if age is not None else '없음 — test-faker 실행 중인지 확인'}"
            f"  · 트랙 {len(tracks)} · 두 카메라 융합 {n_fused}"
            f"  · 카메라별 검출 " + ' · '.join(f'{c} {len(dets[c]) if c in dets else "-"}' for c in self.cam_colors)
            + ('\n/perception/<cam>/detections 없음 — 이전 perception (융합 전 검출 미발행)' if not dets else '')
            + self.loc_text()))
        self.fill_tree(tracks, by_id)
        sel = next((o for o in tracks if o['id'] == self.selected), None) or (tracks[0] if tracks else None)
        self.photos['zoom'] = self.to_photo(self.zoom_image(sel, by_id.get(sel['id'], []) if sel else []))
        self.zoom_label.config(image=self.photos['zoom'])
        for cam, lab in self.cam_labels.items():
            self.photos[cam] = self.to_photo(self.cam_image(cam))
            lab.config(image=self.photos[cam])

    def loc_text(self):
        """위치추정 오차 — 인지 좌표는 로봇이 아는 위치(EKF·RTAB-Map) 기준이라 이만큼 통째로 어긋나 지도에 그려진다"""
        e = self.localization_error() if self.localization_error else None
        if e is None:
            return ''
        d = math.hypot(e[0], e[1])
        warn = '  ← 감지 물체가 지도에서 이만큼 통째로 밀림 ([위치 초기화] 또는 위치추정 필요)' if d > 0.05 else ''
        return f'\n위치추정 오차 (로봇이 아는 위치 − 실제): dx {e[0] * 100:+.1f} dy {e[1] * 100:+.1f} cm · yaw {math.degrees(e[2]):+.1f}°{warn}'

    def fill_tree(self, tracks, by_id):
        open_ids = {iid for iid in self.tree.get_children() if self.tree.item(iid, 'open')}
        self.tree.delete(*self.tree.get_children())
        now = time.time()
        for o in tracks:
            p, c = pos_of(o), cov_of(o)
            cams = sorted({cam for cam, _ in by_id.get(o['id'], [])})
            ls = o.get('last_seen', {})
            seen_age = now - (ls.get('sec', 0) + ls.get('nanosec', 0) * 1e-9) if ls.get('sec') else None
            s = o.get('size', {})
            iid = f"t{o['id']}"
            self.tree.insert('', tk.END, iid=iid, text=f"#{o['id']}" + (' F' if len(cams) > 1 else ''),
                             open=iid in open_ids or len(cams) > 1, tags=('fused',),
                             values=(o.get('class_name', '?'), o.get('color', ''),
                                     ' + '.join(cams) if cams else f"({o.get('camera', '')})",
                                     f'{p[0]:.3f}', f'{p[1]:.3f}', f'{p[2]:.3f}', sigma_cm(c), '',
                                     f"{s.get('x', 0) * 100:.0f}×{s.get('y', 0) * 100:.0f}×{s.get('z', 0) * 100:.0f}",
                                     f"{o.get('confidence', 0):.2f}",
                                     f'{seen_age:.1f}' if seen_age is not None else '-'))
            for cam, d in sorted(by_id.get(o['id'], []), key=lambda x: x[0]):
                q = pos_of(d)
                self.tree.insert(iid, tk.END, text=f'└ {cam}', tags=(cam,),
                                 values=(d.get('class_name', '?'), d.get('color', ''), f"px {d.get('px', ['?', '?'])}",
                                         f'{q[0]:.3f}', f'{q[1]:.3f}', f'{q[2]:.3f}', sigma_cm(cov_of(d)),
                                         f'{np.linalg.norm(q - p) * 100:.1f}', '', f"{d.get('confidence', 0):.2f}", ''))
        if self.selected is not None and self.tree.exists(f't{self.selected}'):
            self.tree.selection_set(f't{self.selected}')

    def zoom_image(self, track, cam_dets):
        img = np.full((ZOOM_PX, ZOOM_PX, 3), 250, np.uint8)
        if track is None:
            cv2.putText(img, 'no tracks', (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 120, 120), 1)
            return img
        center = pos_of(track)
        items = [(FUSED_COLOR, pos_of(track), cov_of(track), 2)]
        items += [(self.cam_colors[cam], pos_of(d), cov_of(d), 1) for cam, d in cam_dets]
        # 배율: 가장 먼 점 + 가장 긴 2σ가 화면 40% 안에 들게 (최소 ±5cm)
        reach = 0.05
        for _, q, c, _ in items:
            r = 2.0 * math.sqrt(max(np.linalg.eigvalsh(c[:2, :2]).max(), 0.0)) if c is not None else 0.0
            reach = max(reach, float(np.linalg.norm(q[:2] - center[:2])) + r)
        scale = ZOOM_PX * 0.4 / reach  # px/m
        mid = ZOOM_PX // 2

        def px(q):
            return (int(mid + (q[0] - center[0]) * scale), int(mid - (q[1] - center[1]) * scale))

        step = 10 ** math.floor(math.log10(reach))  # 격자 간격 [m] (1cm · 10cm …)
        for k in range(-int(reach / step) - 1, int(reach / step) + 2):
            g = int(mid + k * step * scale)
            cv2.line(img, (g, 0), (g, ZOOM_PX), (225, 225, 225), 1)
            cv2.line(img, (0, g), (ZOOM_PX, g), (225, 225, 225), 1)
        for color, q, c, thick in items:
            if c is not None:
                ellipse(img, px(q), c[:2, :2], scale, color, thick)
        for color, q, _, thick in items[1:]:
            cv2.line(img, px(q), px(center), color, 1)
            cv2.drawMarker(img, px(q), color, cv2.MARKER_TILTED_CROSS, 10, 2)
        cv2.drawMarker(img, px(center), FUSED_COLOR, cv2.MARKER_CROSS, 12, 2)
        cv2.putText(img, f"{track.get('class_name', '?')}#{track['id']}", (6, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, FUSED_COLOR, 1)
        cv2.putText(img, f'grid {step * 100:g} cm   x right, y up (map)', (6, ZOOM_PX - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (90, 90, 90), 1)
        y = 34
        for name, color in [('fused', FUSED_COLOR)] + [(c, self.cam_colors[c]) for c, _ in cam_dets]:
            cv2.putText(img, name, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            y += 16
        return img

    def cam_image(self, cam):
        path = TOPIC_DIR / cam / 'color/image_raw/image.jpg'
        img = cv2.imread(str(path))
        if img is None:
            img = np.zeros((CAM_TILE[1], CAM_TILE[0], 3), np.uint8)
            cv2.putText(img, 'no image', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
            return img
        img = cv2.resize(img, CAM_TILE)
        cached = self.sim.renders.get(cam) if hasattr(self.sim, 'renders') else None
        self.overlay(img, self.sim, cam, cached[2] if cached is not None else None)
        return img

    @staticmethod
    def to_photo(img):
        ok, buf = cv2.imencode('.png', img)
        return tk.PhotoImage(data=base64.b64encode(buf.tobytes())) if ok else None
