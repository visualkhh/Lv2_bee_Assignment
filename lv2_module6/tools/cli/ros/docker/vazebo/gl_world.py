"""SIL 뷰어용 GPU 3D 뷰포트 — moderngl(OpenGL) 래스터라이저.

torch 광선추적(scene.Renderer)은 카메라 토픽(fake_camera)용으로 정확하지만,
사람이 돌려 보는 3D 화면은 몸 메시를 매 프레임 다시 쏘면 느리고 VIEW_RANGE 밖 벽이 사라진다.
이 모듈은 미로·퍽·로봇 몸을 삼각형으로 GPU에 올려 60fps로 그린다.

- 좌표: world z up · 카메라 optical 아님(뷰 행렬로 처리)
- 색: 장면 dict의 BGR/`scene.PALETTE`를 RGB로 변환
- 결과: 640×480 BGR numpy (fbo.read → flip)
"""
from __future__ import annotations

import math

import cv2
import moderngl
import numpy as np

import scene

W, H = 640, 480
NEAR, FAR = 0.05, 50.0
FOV_DEG = 60.0

_VERT = """
#version 330
uniform mat4 mvp;
uniform mat4 model;
in vec3 in_pos;
in vec3 in_nrm;
out vec3 v_nrm;
void main() {
    gl_Position = mvp * vec4(in_pos, 1.0);
    v_nrm = mat3(model) * in_nrm;
}
"""

_FRAG = """
#version 330
uniform vec3 color;
in vec3 v_nrm;
out vec4 f_color;
void main() {
    vec3 n = normalize(v_nrm);
    float l = 0.55 + 0.45 * max(dot(n, normalize(vec3(0.4, 0.35, 0.85))), 0.0);
    f_color = vec4(color * l, 1.0);
}
"""

_FLOOR_VERT = """
#version 330
uniform mat4 mvp;
in vec3 in_pos;
in vec3 in_nrm;
out vec3 v_pos;
void main() {
    gl_Position = mvp * vec4(in_pos, 1.0);
    v_pos = in_pos;
}
"""

_FLOOR_FRAG = """
#version 330
in vec3 v_pos;
out vec4 f_color;
void main() {
    float c = mod(floor(v_pos.x * 2.0) + floor(v_pos.y * 2.0), 2.0);
    vec3 base = mix(vec3(0.46, 0.48, 0.51), vec3(0.50, 0.52, 0.55), c);
    f_color = vec4(base, 1.0);
}
"""


def _perspective(fovy_deg, aspect, near, far):
    f = 1.0 / math.tan(math.radians(fovy_deg) / 2.0)
    return np.array([
        [f / aspect, 0, 0, 0],
        [0, f, 0, 0],
        [0, 0, (far + near) / (near - far), (2 * far * near) / (near - far)],
        [0, 0, -1, 0],
    ], np.float32)


def _look_at(eye, target, up=(0.0, 0.0, 1.0)):
    eye = np.asarray(eye, np.float32)
    fwd = np.asarray(target, np.float32) - eye
    fwd /= max(1e-6, np.linalg.norm(fwd))
    right = np.cross(fwd, np.asarray(up, np.float32))
    right /= max(1e-6, np.linalg.norm(right))
    upv = np.cross(right, fwd)
    M = np.eye(4, dtype=np.float32)
    M[0, :3], M[1, :3], M[2, :3] = right, upv, -fwd
    M[:3, 3] = -M[:3, :3] @ eye
    return M


def _model(x, y, z, sx=1.0, sy=1.0, sz=1.0, yaw=0.0, rx=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    if rx:
        c2, s2 = math.cos(rx), math.sin(rx)
        Rx = np.array([[1, 0, 0, 0], [0, c2, -s2, 0], [0, s2, c2, 0], [0, 0, 0, 1]], np.float32)
        Rz = Rz @ Rx
    S = np.diag([sx, sy, sz, 1.0]).astype(np.float32)
    T = np.eye(4, dtype=np.float32)
    T[:3, 3] = (x, y, z)
    return T @ Rz @ S


def _cube():
    faces = [
        ((1, 0, 0), [(1, -1, -1), (1, 1, -1), (1, 1, 1), (1, -1, 1)]),
        ((-1, 0, 0), [(-1, -1, 1), (-1, 1, 1), (-1, 1, -1), (-1, -1, -1)]),
        ((0, 1, 0), [(-1, 1, -1), (-1, 1, 1), (1, 1, 1), (1, 1, -1)]),
        ((0, -1, 0), [(-1, -1, 1), (-1, -1, -1), (1, -1, -1), (1, -1, 1)]),
        ((0, 0, 1), [(-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]),
        ((0, 0, -1), [(-1, 1, -1), (1, 1, -1), (1, -1, -1), (-1, -1, -1)]),
    ]
    pos, nrm, idx = [], [], []
    for n, quad in faces:
        base = len(pos)
        pos += quad
        nrm += [n] * 4
        idx += [base, base + 1, base + 2, base, base + 2, base + 3]
    return np.array(pos, np.float32), np.array(nrm, np.float32), np.array(idx, np.uint32)


def _cylinder(seg=24):
    pos, nrm, idx = [], [], []
    for i in range(seg):
        a0 = 2 * math.pi * i / seg
        a1 = 2 * math.pi * (i + 1) / seg
        c0, s0, c1, s1 = math.cos(a0), math.sin(a0), math.cos(a1), math.sin(a1)
        base = len(pos)
        pos += [(c0, s0, -0.5), (c1, s1, -0.5), (c1, s1, 0.5), (c0, s0, 0.5)]
        nrm += [(c0, s0, 0), (c1, s1, 0), (c1, s1, 0), (c0, s0, 0)]
        idx += [base, base + 1, base + 2, base, base + 2, base + 3]
    for z, n in ((-0.5, (0, 0, -1)), (0.5, (0, 0, 1))):
        base = len(pos)
        pos.append((0, 0, z)); nrm.append(n)
        for i in range(seg + 1):
            a = 2 * math.pi * i / seg
            pos.append((math.cos(a), math.sin(a), z)); nrm.append(n)
        for i in range(seg):
            idx += [base, base + 1 + i, base + 2 + i]
    return np.array(pos, np.float32), np.array(nrm, np.float32), np.array(idx, np.uint32)


def _sphere(seg=16, ring=10):
    pos, nrm, idx = [], [], []
    for j in range(ring + 1):
        v = math.pi * j / ring
        z, r = math.cos(v), math.sin(v)
        for i in range(seg + 1):
            u = 2 * math.pi * i / seg
            p = (r * math.cos(u), r * math.sin(u), z)
            pos.append(p); nrm.append(p)
    for j in range(ring):
        for i in range(seg):
            a = j * (seg + 1) + i
            b = a + seg + 1
            idx += [a, b, a + 1, a + 1, b, b + 1]
    return np.array(pos, np.float32), np.array(nrm, np.float32), np.array(idx, np.uint32)


def _plane(size):
    s = size / 2
    pos = [(-s, -s, 0), (s, -s, 0), (s, s, 0), (-s, s, 0)]
    nrm = [(0, 0, 1)] * 4
    idx = [0, 1, 2, 0, 2, 3]
    return np.array(pos, np.float32), np.array(nrm, np.float32), np.array(idx, np.uint32)


def _rgb(c):
    if isinstance(c, str):
        b, g, r = scene.PALETTE.get(c, (128, 128, 128))
    else:
        b, g, r = c
    return (r / 255.0, g / 255.0, b / 255.0)


def _robot_rgb(c, tint=None):
    """로봇 링크 색 — tint(BGR)가 있으면 그 색으로, 원래 밝기(luma)로 명암만 남긴다."""
    if tint is None:
        return _rgb(c)
    b, g, r = c if not isinstance(c, str) else scene.PALETTE.get(c, (128, 128, 128))
    lum = (0.114 * b + 0.587 * g + 0.299 * r) / 255.0
    k = 0.55 + 0.65 * min(1.0, lum)   # 0.55(어두운 부품) ~ 1.2(밝은 부품)
    tb, tg, tr = tint
    return tuple(min(1.0, (v / 255.0) * k) for v in (tr, tg, tb))   # 셰이더는 RGB


class WorldGL:
    def __init__(self):
        self.ctx = moderngl.create_standalone_context()
        self.ctx.enable(moderngl.DEPTH_TEST)
        self.ctx.disable(moderngl.CULL_FACE)   # winding 걱정 없이 (장면이 작아 비용 무시)
        self.prog = self.ctx.program(vertex_shader=_VERT, fragment_shader=_FRAG)
        self.floor_prog = self.ctx.program(vertex_shader=_FLOOR_VERT, fragment_shader=_FLOOR_FRAG)
        self.fbo = self.ctx.framebuffer(
            color_attachments=[self.ctx.texture((W, H), 3)],
            depth_attachment=self.ctx.depth_texture((W, H)))
        self.cube = self._vao(self.prog, *_cube())
        self.cyl = self._vao(self.prog, *_cylinder())
        self.sphere = self._vao(self.prog, *_sphere())
        fpos, _fnrm, fidx = _plane(14.0)
        self.floor = self._vao_floor(self.floor_prog, fpos, fidx)
        self.mesh_cache = {}
        self.proj = _perspective(FOV_DEG, W / H, NEAR, FAR)

    def _vao(self, prog, pos, nrm, idx, names=('in_pos', 'in_nrm')):
        data = np.hstack([pos, nrm]).astype('f4').tobytes()
        return self.ctx.vertex_array(
            prog, [(self.ctx.buffer(data), '3f 3f', *names)],
            index_buffer=self.ctx.buffer(idx.astype('i4').tobytes()))

    def _vao_floor(self, prog, pos, idx):
        data = np.asarray(pos, np.float32).tobytes()
        return self.ctx.vertex_array(
            prog, [(self.ctx.buffer(data), '3f', 'in_pos')],
            index_buffer=self.ctx.buffer(np.asarray(idx, np.uint32).astype('i4').tobytes()))

    def _robot_vao(self, tris):
        key = id(tris)
        hit = self.mesh_cache.get(key)
        if hit is not None:
            return hit
        tri = np.asarray(tris, np.float32)
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        ln = np.linalg.norm(n, axis=1)
        n = n / np.maximum(ln, 1e-9)[:, None]
        pos = tri.reshape(-1, 3)
        nrm = np.repeat(n, 3, axis=0)
        data = np.hstack([pos, nrm]).astype('f4').tobytes()
        vao = self.ctx.vertex_array(self.prog, [(self.ctx.buffer(data), '3f 3f', 'in_pos', 'in_nrm')])
        self.mesh_cache[key] = vao
        return vao

    def _draw(self, vao, mvp, model, color, prog=None):
        prog = prog or self.prog
        prog['mvp'].write(np.ascontiguousarray(mvp.T, dtype='f4').tobytes())
        prog['model'].write(np.ascontiguousarray(model.T, dtype='f4').tobytes())
        prog['color'].value = color
        vao.render()

    def render(self, sim, cam, debug_dir=None, robot_tint=None):
        """robot_tint = (B, G, R) 이면 로봇 링크를 그 색 계열로 칠한다 (원래 밝기로 명암 유지)."""
        cam.sync(sim)
        view = _look_at(cam.eye(), cam.target)
        eye = np.asarray(cam.eye(), np.float32)
        target = np.asarray(cam.target, np.float32)
        self.fbo.use()
        self.ctx.viewport = (0, 0, W, H)
        self.ctx.clear(0.62, 0.72, 0.82)

        # 바닥 — 체커 (모델 = 원점)
        I = np.eye(4, dtype=np.float32)
        self.floor_prog['mvp'].write(np.ascontiguousarray((self.proj @ view).T, dtype='f4').tobytes())
        self.floor.render()

        # 미로 벽 (축 정렬 상자)
        for cx, cy, cz, sx, sy, sz in sim.walls:
            m = _model(cx, cy, cz, sx / 2, sy / 2, sz / 2)
            self._draw(self.cube, self.proj @ view @ m, m, (0.68, 0.70, 0.73))

        # 퍽·장애물
        with sim.lock:
            things = [dict(p) for p in sim.pucks + sim.props]
        for p in things:
            shape = p.get('shape')
            if shape not in ('box', 'cylinder', 'sphere'):
                continue
            col = _rgb(p.get('color', (128, 128, 128)))
            x, y = float(p['x']), float(p['y'])
            z = float(p.get('z', 0.0))
            yaw = float(p.get('yaw', 0.0))
            size = p.get('size') or [0.015, 0.06]
            if shape == 'sphere':
                r = float(size[0])
                m = _model(x, y, z + r, r, r, r, yaw)
                self._draw(self.sphere, self.proj @ view @ m, m, col)
            elif shape == 'cylinder':
                r, h = float(size[0]), float(size[1])
                if p.get('lying'):
                    m = _model(x, y, z + r, r, r, h, yaw, rx=math.pi / 2)
                else:
                    m = _model(x, y, z + h / 2, r, r, h, yaw)
                self._draw(self.cyl, self.proj @ view @ m, m, col)
            else:
                sx = float(p.get('sx', size[0]))
                sy = float(p.get('sy', size[0]))
                h = float(size[1])
                m = _model(x, y, z + h / 2, sx / 2, sy / 2, h / 2, yaw)
                self._draw(self.cube, self.proj @ view @ m, m, col)

        # 로봇 몸 — 링크별 상자·메시
        if sim.kin is not None and sim.body is not None:
            x, y, yaw, q = sim.robot_xy_yaw()
            twb = scene.transform(scene.rpy_matrix(0, 0, yaw), (x, y, 0))
            for link, To, (sx, sy, sz), rgb in sim.body.boxes:
                T = twb @ sim.kin.pose(link, q) @ To
                m = T @ np.diag([sx / 2, sy / 2, sz / 2, 1.0]).astype(np.float32)
                self._draw(self.cube, self.proj @ view @ m, m, _robot_rgb(rgb, robot_tint))
            for link, To, tris, rgb in sim.body.meshes:
                T = twb @ sim.kin.pose(link, q) @ To
                self._draw(self._robot_vao(tris), self.proj @ view @ T, T, _robot_rgb(rgb, robot_tint))

        self.ctx.finish()
        raw = self.fbo.read(components=3, alignment=1)
        img = np.frombuffer(raw, np.uint8).reshape(H, W, 3)
        img = cv2.flip(img, 0)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
