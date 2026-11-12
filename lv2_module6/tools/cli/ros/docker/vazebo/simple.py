#!/usr/bin/env python3
"""도커 모드용 간단 SIL 뷰어 — 실환경 3D + 카메라 보내기 전/받은 후만 보여 준다.

vazebo/main.py의 조작·환경·수동주행 창은 빼고, 렌더·스트리밍 코어(Sim)만 재사용한다.
3D 화면은 moderngl(OpenGL) GPU 래스터라이저 — 실패하면 torch 광선추적으로 폴백.
자산은 cli/ros/docker/assets 사본을 쓴다 (BEEBOT_SIM_ASSETS).

  왼쪽: 실환경 3D — 마우스로 돌려 보는 추적 시점 (미로·로봇·퍽·장애물)
  오른쪽: 카메라 2대 × (SIM/ROS) × (RGB · DEPTH · POINT) 6칸
    SIM = 시뮬레이터가 보낸 원본 · ROS = fake_camera가 받아 ROS로 낸 것

3D 조작: WASD/방향키 이동 · LMB 드래그 회전 · RMB/MMB 드래그 이동 · 휠 확대/축소 · R 초기화 · F 로봇 따라가기

SIM 칸 = 시뮬레이터가 렌더해 debug/input/hardware/camera/에 쓴 RGB·DEPTH·POINT 원본
ROS 칸 = monitor_manager가 debug/output/topic/<카메라>/... 에 저장한 RGB·DEPTH·POINT
ROS 칸이 2초 넘게 낡으면 흐리게 + STALE로 표시한다.

  .venv/bin/python vazebo/simple.py                       # 개발용 (simulator 자산)
  BEEBOT_SIM_ASSETS=../tools/cli/ros/docker/assets .venv/bin/python vazebo/simple.py
  BEEBOT_SIM_ASSETS=../tools/cli/ros/docker/assets .venv/bin/python vazebo/simple.py --headless
  BEEBOT_SIM_ASSETS=../tools/cli/ros/docker/assets .venv/bin/python vazebo/simple.py --snapshot out.png
"""
from __future__ import annotations

import argparse
import atexit
import math
import os
import signal
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

import main as vazebo

try:
    from gl_world import WorldGL
except Exception:  # moderngl 없음/컨텍스트 실패 → torch 폴백
    WorldGL = None

# docker 모드는 cli/ros/docker/assets 사본을 쓴다 (원본 isaac/·simulators/ 경로 참조 금지).
ASSETS = Path(os.environ['BEEBOT_SIM_ASSETS']).resolve() if os.environ.get('BEEBOT_SIM_ASSETS') else None

TILE = (320, 240)
GAP = 8
HEADER_H = 26
MAX_W, MAX_H = 1500, 900
VIEW_W, VIEW_H = 640, 480

# 3D 뷰 로봇 색 (BGR) — 회색 바닥·벽과 구분되게 주황
ROBOT_TINT_BGR = (30, 90, 235)


def _tint_rgb(rgb):
    """URDF 링크 색(BGR)을 로봇 색 계열로 — 원래 밝기로 명암만 남긴다."""
    b, g, r = rgb
    lum = (0.114 * float(b) + 0.587 * float(g) + 0.299 * float(r)) / 255.0
    k = 0.55 + 0.65 * min(1.0, lum)
    return tuple(min(255, int(c * k)) for c in ROBOT_TINT_BGR)

# 뷰어가 실제로 읽는 토픽 — monitor_manager가 이 파일을 보고 동적으로 구독한다.
VIEW_TOPICS = (
    '/state/operation_mode', '/state/map_mode', '/state/map_name', '/state/map_list',
    '/state/devices', '/state/restartable', '/state/topics', '/state/restart', '/state/emergency',
    '/state/devices/pole_front_cam', '/state/devices/gripper_cam', '/state/devices/opencr',
    '/state/devices/imu', '/state/devices/wheels', '/state/devices/arm', '/state/devices/gripper',
    '/joint_states', '/odom', '/imu', '/robot_description',
    '/fake_opencr/ground_truth', '/tracked_objects',
    '/pole_front_cam/color/image_raw', '/pole_front_cam/color/camera_info',
    '/pole_front_cam/aligned_depth_to_color/image_raw', '/pole_front_cam/depth/color/points',
    '/gripper_cam/color/image_raw', '/gripper_cam/color/camera_info',
    '/gripper_cam/aligned_depth_to_color/image_raw', '/gripper_cam/depth/color/points',
)

_REQ_PATH = vazebo.DEBUG / 'input' / 'monitor' / f'viewer-{os.getpid()}.topics'


def _write_requests() -> None:
    try:
        _REQ_PATH.parent.mkdir(parents=True, exist_ok=True)
        _REQ_PATH.write_text('\n'.join(VIEW_TOPICS) + '\n')
    except OSError:
        pass


def _start_requests() -> None:
    _write_requests()
    atexit.register(lambda: _REQ_PATH.unlink(missing_ok=True))

    def beat():
        while True:
            _write_requests()
            time.sleep(2.0)

    threading.Thread(target=beat, daemon=True).start()


def _blank(text: str):
    img = np.zeros((TILE[1], TILE[0], 3), np.uint8)
    cv2.putText(img, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200, 200, 200), 1)
    return img


def _label(img, text: str):
    cv2.putText(img, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    return img


_TILE_CACHE = {}


def camera_tiles(sim, name: str):
    """vazebo.camera_tiles의 8장 중 SIM/ROS RGB·RGBD·POINT만 — 0.3초 캐시."""
    now = time.time()
    hit = _TILE_CACHE.get(name)
    if hit is not None and now - hit[0] < 0.3:
        return hit[1]
    tiles = vazebo.camera_tiles(sim, name, (0.6, -0.4))
    _TILE_CACHE[name] = (now, tiles)
    return tiles


def _cam_header(width: int):
    img = np.full((HEADER_H, width, 3), 30, np.uint8)
    labels = ('SIM RGB', 'SIM DEPTH', 'SIM POINT', 'ROS RGB', 'ROS DEPTH', 'ROS POINT')
    cw = vazebo.TILE[0]
    for i, text in enumerate(labels):
        cv2.putText(img, text, (i * (cw + GAP) + 6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    return img


def _header(width: int):
    img = np.full((HEADER_H, width, 3), 30, np.uint8)
    cv2.putText(img, 'SIM (before send)', (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1)
    cv2.putText(img, 'ROS (fake_camera received)', (TILE[0] + GAP + 8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1)
    return img


class OrbitCamera:
    """로봇 주변을 도는 3D 카메라 — Isaac Sim 뷰포트처럼 마우스로 조작."""

    def __init__(self):
        self.target = None
        self.pan = np.zeros(3)
        self.yaw = 0.0
        self.pitch = math.radians(38.0)
        self.distance = 3.2
        self.follow = True
        self.drag = None
        self.dirty = True
        self.last_render = 0.0
        self.cache = None
        self.pane_rect = (0, 0, VIEW_W, VIEW_H)

    def sync(self, sim):
        x, y, _, _ = sim.robot_xy_yaw()
        if self.target is None or self.follow:
            self.target = np.array([x, y, 0.3]) + self.pan

    def reset(self, sim):
        x, y, yaw, _ = sim.robot_xy_yaw()
        self.target = np.array([x, y, 0.3])
        self.pan[:] = 0.0
        self.yaw = yaw + math.pi          # 로봇 뒤
        self.pitch = math.radians(38.0)
        self.distance = 3.2
        self.follow = True
        self.dirty = True

    def eye(self):
        cp = math.cos(self.pitch)
        return self.target + self.distance * np.array(
            [math.cos(self.yaw) * cp, math.sin(self.yaw) * cp, math.sin(self.pitch)])

    def orbit(self, dx, dy):
        self.yaw -= dx * 0.008
        self.pitch = float(np.clip(self.pitch - dy * 0.006,
                                   math.radians(5.0), math.radians(85.0)))
        self.dirty = True

    def pan_by(self, dx, dy):
        fwd = self.target - self.eye()
        fwd /= max(1e-6, np.linalg.norm(fwd))
        right = np.cross(fwd, np.array([0.0, 0.0, 1.0]))
        right /= max(1e-6, np.linalg.norm(right))
        up = np.cross(right, fwd)
        move = (-dx * right + dy * up) * (self.distance * 0.0015)
        self.pan += move
        if not self.follow:
            self.target += move
        self.dirty = True

    def zoom(self, delta):
        self.distance = float(np.clip(self.distance * (0.9 if delta > 0 else 1.1), 1.0, 8.0))
        self.dirty = True

    def move(self, key: str) -> bool:
        """WASD/방향키로 시점을 옮긴다 (로봇 따라가기 중이면 로봇 기준 오프셋)."""
        fwd = self.target - self.eye()
        fwd[2] = 0.0
        n = np.linalg.norm(fwd)
        fwd = fwd / n if n > 1e-6 else np.array([1.0, 0.0, 0.0])
        right = np.cross(fwd, np.array([0.0, 0.0, 1.0]))
        right /= max(1e-6, np.linalg.norm(right))
        step = 0.15
        move = {
            'w': fwd * step, 'up': fwd * step,
            's': -fwd * step, 'down': -fwd * step,
            'd': right * step, 'right': right * step,
            'a': -right * step, 'left': -right * step,
        }.get(key)
        if move is None:
            return False
        self.pan += move
        if not self.follow:
            self.target += move
        self.dirty = True
        return True

    def toggle_follow(self):
        self.follow = not self.follow
        if self.follow:
            self.pan[:] = 0.0
        self.dirty = True


def _look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """optical 프레임(z 앞 · x 오른쪽 · y 아래)으로 보는 T_world_cam."""
    eye = np.asarray(eye, float)
    fwd = np.asarray(target, float) - eye
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, np.asarray(up, float))
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = right, down, fwd, eye
    return T


def _decorate_world(img):
    cv2.rectangle(img, (0, 0), (VIEW_W, 28), (25, 25, 25), -1)
    cv2.rectangle(img, (0, VIEW_H - 24), (VIEW_W, VIEW_H), (25, 25, 25), -1)
    cv2.putText(img, '3D world (GPU · orbit)', (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    cv2.putText(img, 'WASD/arrows move | LMB orbit | RMB pan | wheel zoom | R reset | F follow',
                (8, VIEW_H - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (220, 220, 220), 1)
    return img


_GL = None
_GL_FAILED = False


def world_view(sim, cam: OrbitCamera):
    """로봇 주변 3D — GPU 래스터라이저 우선, 실패하면 기존 torch 광선추적 폴백."""
    global _GL, _GL_FAILED
    cam.sync(sim)
    if WorldGL is not None and not _GL_FAILED:
        if _GL is None:
            try:
                _GL = WorldGL()
            except Exception as exc:  # noqa: BLE001
                print(f'[3d] GPU 뷰포트 실패 — torch 폴백: {exc}', file=sys.stderr)
                _GL_FAILED = True
        if _GL is not None:
            try:
                img = _decorate_world(_GL.render(sim, cam, robot_tint=ROBOT_TINT_BGR))
                cam.cache, cam.last_render, cam.dirty = img, time.time(), False
                return img
            except Exception as exc:  # noqa: BLE001
                print(f'[3d] GPU 렌더 실패 — torch 폴백: {exc}', file=sys.stderr)
                _GL_FAILED = True

    now = time.time()
    if not cam.dirty and cam.cache is not None and now - cam.last_render < 0.5:
        return cam.cache
    if sim.kin is None or sim.body is None:
        return _blank('3D world: waiting for /robot_description')
    x, y, yaw, q = sim.robot_xy_yaw()
    T = _look_at(cam.eye(), cam.target)
    twb = vazebo.scene.transform(vazebo.scene.rpy_matrix(0, 0, yaw), (x, y, 0))
    with sim.lock:
        things = ([dict(p) for p in sim.pucks + sim.props] + [dict(sim.goal)] +
                  [dict(z) for z in sim.stations])
    body = sim.body
    boxes = [(twb @ sim.kin.pose(link, q) @ To, sx, sy, sz, _tint_rgb(rgb))
             for link, To, (sx, sy, sz), rgb in body.boxes]
    meshes = [(twb @ sim.kin.pose(link, q) @ To, tris, _tint_rgb(rgb))
              for link, To, tris, rgb in body.meshes]
    joints = tuple(sorted((n, round(v, 2)) for n, v in q.items() if 'wheel' not in n))
    rel_key = tuple(np.round((np.linalg.inv(twb) @ T).ravel(), 3))
    body_key = ('world', joints, rel_key)
    bgr, _ = sim.renderer.render(T, things, boxes, meshes, body_key=body_key)
    bgr = cv2.convertScaleAbs(bgr, alpha=1.35, beta=30)
    img = _decorate_world(cv2.resize(bgr, (VIEW_W, VIEW_H), interpolation=cv2.INTER_LINEAR))
    cam.cache, cam.last_render, cam.dirty = img, now, False
    return img


def compose(sim, cam: OrbitCamera):
    rows = []
    cw, ch = vazebo.TILE
    for name in vazebo.CAMERAS:
        tiles = camera_tiles(sim, name)
        sel = [tiles[0], tiles[1], tiles[3], tiles[4], tiles[5], tiles[7]]
        parts = []
        for i, tile in enumerate(sel):
            if i:
                parts.append(np.full((ch, GAP, 3), 255, np.uint8))
            parts.append(tile)
        row = np.hstack(parts)
        label = np.full((20, row.shape[1], 3), 20, np.uint8)
        cv2.putText(label, name, (6, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        rows.append(np.vstack([label, row]))
    grid = np.vstack([rows[0], np.full((GAP, rows[0].shape[1], 3), 255, np.uint8), rows[1]])
    grid = np.vstack([_cam_header(grid.shape[1]), grid])

    m = world_view(sim, cam)
    h = max(m.shape[0], grid.shape[0])
    pad = lambda a: np.vstack([a, np.full((h - a.shape[0], a.shape[1], 3), 255, np.uint8)])
    out = np.hstack([pad(m), np.full((h, GAP, 3), 255, np.uint8), pad(grid)])

    scale = min(1.0, MAX_W / out.shape[1], MAX_H / out.shape[0])
    if scale < 1.0:
        out = cv2.resize(out, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    cam.pane_rect = (0, 0, int(VIEW_W * scale), int(VIEW_H * scale))
    return out


def _setup(args):
    spawn = tuple(float(v) for v in args.spawn.split(',')) if args.spawn else None
    course = args.course
    bg_dir, floor_dir, wall_dir = vazebo.BG_DIR, vazebo.FLOOR_DIR, vazebo.WALL_DIR
    if ASSETS is not None:   # 코스(실세계)는 cli/ros/fake/real/map — maze.load가 이름으로 찾는다
        bg_dir, floor_dir, wall_dir = ASSETS / 'background', ASSETS / 'floor', ASSETS / 'wall'
        if (ASSETS / 'dice').is_dir():
            vazebo.dice.PROBLEM_DIR = ASSETS / 'dice'
    sim = vazebo.Sim(vazebo.maze.load(course), spawn)
    if 'mission' in sim.course and args.dice:
        sim.load_dice(args.dice)
    if args.pucks:
        sim.randomize(args.pucks)
    bgs = vazebo.scene.backgrounds(bg_dir)
    sim.renderer.set_background(bgs[0] if bgs else None)
    floors = [p for p in vazebo.scene.backgrounds(floor_dir) if p.name != 'FLOOR_CREDITS.txt']
    sim.renderer.set_floor(floors[0] if floors else None)
    walls = [p for p in vazebo.scene.backgrounds(wall_dir) if p.suffix == '.jpg']
    sim.renderer.set_wall(walls[0] if walls else None)
    return sim


def _headless(sim):
    signal.signal(signal.SIGTERM, lambda *_: setattr(sim, 'running', False))
    print('SIL simple view headless — 카메라 →', vazebo.LIVE_DIR, flush=True)
    try:
        sim.render_loop()
    except KeyboardInterrupt:
        pass
    finally:
        sim.running = False
        sim.stop_streaming()


def _gui(sim):
    import tkinter as tk

    root = tk.Tk()
    root.title('lv2_module6 — docker world (3D / SIM → fake_camera → ROS)')
    label = tk.Label(root, bd=0)
    label.pack()
    root.focus_set()

    cam = OrbitCamera()
    cam.reset(sim)

    def inside(x, y):
        x0, y0, x1, y1 = cam.pane_rect
        return x0 <= x < x1 and y0 <= y < y1

    def press(e):
        if inside(e.x, e.y):
            cam.drag = (e.x, e.y, e.num)
            root.focus_set()

    def drag(e):
        if cam.drag is None:
            return
        x0, y0, btn = cam.drag
        dx, dy = e.x - x0, e.y - y0
        cam.drag = (e.x, e.y, btn)
        if btn == 1:
            cam.orbit(dx, dy)
        elif btn in (2, 3):
            cam.pan_by(dx, dy)

    def release(_e):
        cam.drag = None

    def wheel(e):
        if inside(e.x, e.y):
            cam.zoom(1 if e.delta > 0 else -1)

    def wheel_linux(e):
        if inside(e.x, e.y):
            cam.zoom(1 if e.num == 4 else -1)

    def key(e):
        k = e.keysym.lower()
        if k == 'r':
            cam.reset(sim)
        elif k == 'f':
            cam.toggle_follow()
        elif k in ('plus', 'equal'):
            cam.zoom(1)
        elif k == 'minus':
            cam.zoom(-1)
        else:
            cam.move(k)

    for seq, fn in (('<ButtonPress-1>', press), ('<B1-Motion>', drag), ('<ButtonRelease-1>', release),
                    ('<ButtonPress-2>', press), ('<B2-Motion>', drag), ('<ButtonRelease-2>', release),
                    ('<ButtonPress-3>', press), ('<B3-Motion>', drag), ('<ButtonRelease-3>', release),
                    ('<MouseWheel>', wheel), ('<Button-4>', wheel_linux), ('<Button-5>', wheel_linux),
                    ('<KeyPress>', key)):
        root.bind(seq, fn)

    def _stop(*_):
        sim.running = False
        sim.stop_streaming()   # 메타를 지워 fake_camera가 마지막 프레임을 계속 내지 않게
        root.quit()

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    def tick():
        if not root.winfo_exists():
            return
        photo = vazebo.to_photo(compose(sim, cam))
        if photo is not None:
            label.configure(image=photo)
            label.image = photo
        root.after(50, tick)

    tick()
    try:
        root.mainloop()
    finally:
        sim.running = False
        sim.stop_streaming()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--snapshot', help='창 없이 화면 한 장을 이 PNG로 저장하고 끝냄')
    parser.add_argument('--spawn', help='map 원점의 미로 좌표 "x,y" (기본: 코스 spawn)')
    parser.add_argument('--course', default='isaac_maze', help='코스(실세계) = cli/ros/fake/real/map/<이름> — isaac_maze(기본, USD) · dice_maze. faker course:=와 같게')
    parser.add_argument('--dice', default='q1', help='dice_maze 문제 = dice/<이름>.json (빈 값이면 안 깜)')
    parser.add_argument('--pucks', type=int, default=0, help='추가로 무작위 배치할 퍽 수')
    parser.add_argument('--headless', action='store_true', help='창 없이 카메라 송출만')
    args = parser.parse_args()

    sim = _setup(args)
    _start_requests()
    cam = OrbitCamera()
    cam.reset(sim)
    if args.snapshot:
        sim.step()
        cv2.imwrite(args.snapshot, compose(sim, cam))
        return
    if args.headless:
        _headless(sim)
        return
    threading.Thread(target=sim.render_loop, daemon=True).start()
    _gui(sim)


if __name__ == '__main__':
    main()
