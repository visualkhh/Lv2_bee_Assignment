"""로봇 3D 뷰 — URDF(로봇이 내보내는 /robot_description)의 visual 도형을 관절값대로 그린다 (OpenCV, 별도 3D 라이브러리 없음)

- RobotModel: URDF의 link visual(box · cylinder · sphere, origin, material 색)과 관절(종류 · 축 · 한계)을 읽는다.
  자세는 scene.Kinematics 순기구학.
- OrbitView: 궤도 카메라(드래그로 회전 · 휠로 줌). 면을 먼 것부터 칠하는 단순 렌더(painter's algorithm) + 조명.
- 팔 뷰: 관절 원점에 마커를 그리고, 마커를 잡아 위아래로 끌면 그 관절 목표값이 바뀐다 (수동 조작 창).
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np

import scene

URDF_DIR = Path(__file__).resolve().parent.parent / 'urdf'  # 소스 기준 (lv2_module6/urdf) — 모델 하나 = 폴더 하나 <이름>/urdf.xacro + meshes/
INSTALL_DIR = Path(__file__).resolve().parent.parent / 'debug' / 'install'  # 컨테이너 install 실파일 복사본 (docker/build-ws.sh)
CAMERA_MESH_CELL = 0.025  # 카메라 렌더용 간소화 격자 [m] — 광선 추적이라 미리보기보다 거칠게 (차체 7.8k → 약 3k 삼각형)
MESH_CELL = 0.016  # 메시 간소화 격자 [m] — 3D 미리보기용. 0.008의 1/3 삼각형·3배속, 320px 미리보기엔 충분
_mesh_cache = {}


def resolve_mesh(filename):
    """<mesh filename> → STL 실경로. package://<pkg>/<경로>는 debug/install/<pkg>/share/<pkg>/<경로>
    (빌드 뒤 컨테이너가 링크를 풀어 복사한 실파일) → 없으면 소스 (package://ros_system/urdf/<모델>/meshes/x → lv2_module6/urdf/<모델>/meshes/x).
    둘 다 없으면 None (뼈대 대체)"""
    if filename.startswith('package://'):
        pkg, _, rel = filename[len('package://'):].partition('/')
        for cand in (INSTALL_DIR / pkg / 'share' / pkg / rel, URDF_DIR / rel.removeprefix('urdf/')):
            if cand.is_file():
                return cand
        return None
    return None

# 메시 끔(use_mesh=False)일 때 대체 뼈대 — 구 URDF(primitive 시절) 치수 참조, visual 원점 기준 box
# (arm은 collision마저 메시라 여기서라도 살려야 팔·gripper_cam이 허공에 안 뜸. 카메라는 구 카메라 박스 0.025×0.090×0.025)
SKELETON_BOX = {
    'arm_base_link': (0.050, 0.045, 0.060),
    'shoulder_link': (0.030, 0.045, 0.060),
    'upper_arm_link': (0.030, 0.030, 0.130),
    'forearm_link': (0.124, 0.030, 0.030),
    'wrist_link': (0.040, 0.035, 0.035),
    'gripper_link': (0.070, 0.050, 0.035),
    'finger_left_link': (0.030, 0.004, 0.025),
    'finger_right_link': (0.030, 0.004, 0.025),
    'pole_cam_link': (0.025, 0.090, 0.025),
    'gripper_cam_link': (0.025, 0.090, 0.025),
}

LIGHT = np.array([0.4, 0.3, 0.85]) / np.linalg.norm([0.4, 0.3, 0.85])


def _box(sx, sy, sz):
    x, y, z = sx / 2, sy / 2, sz / 2
    v = np.array([[-x, -y, -z], [x, -y, -z], [x, y, -z], [-x, y, -z], [-x, -y, z], [x, -y, z], [x, y, z], [-x, y, z]])
    return [v[[0, 3, 2, 1]], v[[4, 5, 6, 7]], v[[0, 1, 5, 4]], v[[1, 2, 6, 5]], v[[2, 3, 7, 6]], v[[3, 0, 4, 7]]]


def _cylinder(r, h, n=14):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    ring = np.stack([r * np.cos(a), r * np.sin(a)], 1)
    bottom = np.column_stack([ring, np.full(n, -h / 2)])
    top = np.column_stack([ring, np.full(n, h / 2)])
    faces = [bottom[::-1], top]
    for i in range(n):
        j = (i + 1) % n
        faces.append(np.array([bottom[i], bottom[j], top[j], top[i]]))
    return faces


def _sphere(r, n=8):
    faces = []
    for i in range(n // 2):
        t0, t1 = math.pi * i / (n // 2) - math.pi / 2, math.pi * (i + 1) / (n // 2) - math.pi / 2
        for j in range(n):
            p0, p1 = 2 * math.pi * j / n, 2 * math.pi * (j + 1) / n
            pt = lambda t, p: [r * math.cos(t) * math.cos(p), r * math.cos(t) * math.sin(p), r * math.sin(t)]
            faces.append(np.array([pt(t0, p0), pt(t0, p1), pt(t1, p1), pt(t1, p0)]))
    return faces


def load_stl(path, scale, cell=MESH_CELL):
    """바이너리 STL → (N,3,3) 삼각형. 꼭짓점을 격자(cell, 기본 MESH_CELL)로 뭉쳐 간소화 · 같은 파일은 한 번만 읽는다.
    path는 resolve_mesh()가 준 실경로, scale은 축별(x y z) 배율"""
    key = (str(path), tuple(scale), cell)
    if key not in _mesh_cache:
        data = np.fromfile(path, np.uint8)
        n = int(np.frombuffer(data[80:84], np.uint32)[0])
        rec = np.frombuffer(data[84:84 + 50 * n], np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
        pts = rec['v'].astype(float).reshape(-1, 3) * np.asarray(scale, float)
        _, inv = np.unique(np.round(pts / cell).astype(np.int64), axis=0, return_inverse=True)
        inv = inv.reshape(-1)
        verts = np.zeros((inv.max() + 1, 3))
        np.add.at(verts, inv, pts)
        verts /= np.bincount(inv)[:, None]  # 뭉친 꼭짓점 = 격자 안 원래 꼭짓점의 평균
        idx = inv.reshape(-1, 3)
        idx = idx[(idx[:, 0] != idx[:, 1]) & (idx[:, 1] != idx[:, 2]) & (idx[:, 0] != idx[:, 2])]  # 찌그러진 삼각형 제거
        _, first = np.unique(np.sort(idx, axis=1), axis=0, return_index=True)  # 같은 삼각형 한 번만
        _mesh_cache[key] = verts[idx[np.sort(first)]]
    return _mesh_cache[key]


def primitive_faces(shape, a, h, b=None):
    """상태 뷰용 물체 원형 — a=반지름(원기둥·구) 또는 x폭(상자), h=높이, b=y폭(상자, 없으면 a)"""
    if shape == 'cylinder':
        return _cylinder(a, h)
    if shape == 'sphere':
        return _sphere(a)
    return _box(a, b if b else a, h)


COLLISION_BGR = (0, 140, 255)  # 충돌 모양 보기 — 한 가지 색 (주황)


class RobotModel:
    def __init__(self, urdf_xml, collision=False):
        """<mesh> visual은 항상 STL로 그림 (package:// → install share, 없으면 소스 meshes 폴백).
        파일이 없으면 그 링크만 뼈대(Skeleton 표 → collision 원시도형)로 대체.
        collision=True면 <visual> 대신 <collision> 모양 (메시 충돌체도 STL 그대로, 색은 COLLISION_BGR)"""
        root = ET.fromstring(urdf_xml)
        self.kin = scene.Kinematics(urdf_xml)
        colors = {m.get('name'): m.find('color').get('rgba') for m in root.findall('material') if m.find('color') is not None}
        self.visuals = []  # (link, T_origin, faces_local, BGR)
        self.missing = []  # 못 찾은 STL (파일명) — 상태 표시용
        for link in root.findall('link'):
            for vis in link.findall('collision' if collision else 'visual'):
                geom = vis.find('geometry')
                if geom is None or len(geom) == 0:
                    continue
                g = geom[0]
                if g.tag == 'box':
                    faces = _box(*[float(v) for v in g.get('size').split()])
                elif g.tag == 'cylinder':
                    faces = _cylinder(float(g.get('radius')), float(g.get('length')))
                elif g.tag == 'sphere':
                    faces = _sphere(float(g.get('radius')))
                elif g.tag == 'mesh':
                    stl = resolve_mesh(g.get('filename', ''))
                    if stl is None:
                        # 파일 없음 — 뼈대로 대체. 순서는 Skeleton 표 → collision 원시도형.
                        self.missing.append(Path(g.get('filename', '')).name)
                        if collision:
                            continue
                        origin = vis.find('origin')
                        xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
                        rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
                        mat = vis.find('material')
                        rgba = None
                        if mat is not None:
                            c = mat.find('color')
                            rgba = c.get('rgba') if c is not None else colors.get(mat.get('name'))
                        r, gg, b = ([float(v) for v in rgba.split()[:3]] if rgba else [0.7, 0.7, 0.7])
                        col = (int(b * 255), int(gg * 255), int(r * 255))
                        skel = SKELETON_BOX.get(link.get('name'))
                        if skel is not None:
                            self.visuals.append((link.get('name'), scene.transform(scene.rpy_matrix(*rpy), xyz),
                                                 _box(*skel), col))
                            continue
                        for col_ in link.findall('collision'):
                            cg = col_.find('geometry')
                            if cg is None or len(cg) == 0 or cg[0].tag == 'mesh':
                                continue
                            c = cg[0]
                            if c.tag == 'box':
                                cfaces = _box(*[float(v) for v in c.get('size').split()])
                            elif c.tag == 'cylinder':
                                cfaces = _cylinder(float(c.get('radius')), float(c.get('length')))
                            elif c.tag == 'sphere':
                                cfaces = _sphere(float(c.get('radius')))
                            else:
                                continue
                            corig = col_.find('origin')
                            cxyz = [float(v) for v in (corig.get('xyz', '0 0 0') if corig is not None else '0 0 0').split()]
                            crpy = [float(v) for v in (corig.get('rpy', '0 0 0') if corig is not None else '0 0 0').split()]
                            self.visuals.append((link.get('name'), scene.transform(scene.rpy_matrix(*crpy), cxyz),
                                                 cfaces, col))
                        continue  # 못 찾은 mesh visual은 건너뜀
                    sc = (g.get('scale', '1').split() * 3)[:3]
                    faces = load_stl(stl, [float(v) for v in sc])
                else:
                    continue
                origin = vis.find('origin')
                xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
                rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
                mat = vis.find('material')
                rgba = None
                if mat is not None:
                    c = mat.find('color')
                    rgba = c.get('rgba') if c is not None else colors.get(mat.get('name'))
                r, gg, b = ([float(v) for v in rgba.split()[:3]] if rgba else [0.7, 0.7, 0.7])
                col = COLLISION_BGR if collision else (int(b * 255), int(gg * 255), int(r * 255))
                self.visuals.append((link.get('name'), scene.transform(scene.rpy_matrix(*rpy), xyz), faces, col))
        self.joints = {}  # 움직이는 관절 이름 → (자식 링크, 종류, 축, 하한, 상한)
        for j in root.findall('joint'):
            if j.get('type') in ('revolute', 'continuous', 'prismatic') and j.find('mimic') is None:
                lim = j.find('limit')
                lo, hi = (float(lim.get('lower', -math.pi)), float(lim.get('upper', math.pi))) if lim is not None else (-math.pi, math.pi)
                axis = j.find('axis')
                self.joints[j.get('name')] = (j.find('child').get('link'), j.get('type'),
                                              [float(v) for v in axis.get('xyz').split()] if axis is not None else [1, 0, 0], lo, hi)

    def descendants(self, link):
        """link와 그 아래 링크 전부 (팔 뷰용)"""
        out, frontier = {link}, [link]
        while frontier:
            parent = frontier.pop()
            for child, j in self.kin.parent.items():
                if j['parent'] == parent and child not in out:
                    out.add(child)
                    frontier.append(child)
        return out

    def faces(self, q, links=None, tint=None):
        """월드(루트 링크) 기준 면 목록 [(꼭짓점 N×3, BGR)]"""
        out, cache = [], {}
        for link, T_o, faces, color in self.visuals:
            if links is not None and link not in links:
                continue
            if link not in cache:
                cache[link] = self.kin.pose(link, q)
            T = cache[link] @ T_o
            col = tint or color
            if isinstance(faces, np.ndarray):  # 메시: 삼각형 묶음 하나로 (render가 한꺼번에 처리)
                out.append((faces @ T[:3, :3].T + T[:3, 3], col))
                continue
            for f in faces:
                out.append((f @ T[:3, :3].T + T[:3, 3], col))
        return out

    def joint_frames(self, q):
        """관절 → (원점 xyz, 축 방향) 월드 기준"""
        out = {}
        for name, (child, _, axis, _, _) in self.joints.items():
            T = self.kin.pose(child, q)
            out[name] = (T[:3, 3], T[:3, :3] @ np.asarray(axis, float))
        return out


class OrbitView:
    def __init__(self, target=(0, 0, 0.2), dist=1.2, yaw=-2.3, pitch=0.45, fov_deg=45):
        self.target = np.array(target, float)
        self.dist, self.yaw, self.pitch, self.fov = dist, yaw, pitch, math.radians(fov_deg)

    def basis(self):
        cp, sp, cy, sy = math.cos(self.pitch), math.sin(self.pitch), math.cos(self.yaw), math.sin(self.yaw)
        eye = self.target + self.dist * np.array([cp * cy, cp * sy, sp])
        fwd = (self.target - eye) / np.linalg.norm(self.target - eye)
        right = np.cross(fwd, [0, 0, 1])
        right /= np.linalg.norm(right)
        up = np.cross(right, fwd)
        return eye, right, up, fwd

    def project(self, pts, size):
        eye, right, up, fwd = self.basis()
        rel = np.asarray(pts, float) - eye
        x, y, z = rel @ right, rel @ up, rel @ fwd
        f = size[1] / 2 / math.tan(self.fov / 2)
        with np.errstate(divide='ignore', invalid='ignore'):
            u = size[0] / 2 + f * x / z
            v = size[1] / 2 - f * y / z
        return np.stack([u, v], -1), z

    def render(self, faces, size, background=(40, 40, 46), grid=True):
        img = np.full((size[1], size[0], 3), background, np.uint8)
        if grid:  # 바닥 격자 (z=0, 0.1m)
            for k in np.arange(-1.0, 1.01, 0.1):
                for a, b in (([k, -1, 0], [k, 1, 0]), ([-1, k, 0], [1, k, 0])):
                    uv, z = self.project([a, b], size)
                    if (z > 0.05).all():
                        cv2.line(img, tuple(uv[0].astype(int)), tuple(uv[1].astype(int)), (70, 70, 78), 1, cv2.LINE_AA)
        eye = self.basis()[0]
        polys = []
        for verts, color in faces:
            if verts.ndim == 3:  # 삼각형 묶음 (N,3,3) — 뒷면 제거 · 투영을 한꺼번에
                n = np.cross(verts[:, 1] - verts[:, 0], verts[:, 2] - verts[:, 0])
                norm = np.linalg.norm(n, axis=1)
                n = n / np.maximum(norm, 1e-12)[:, None]
                ok = (norm > 1e-12) & (np.einsum('ij,ij->i', n, eye - verts.mean(1)) > 0)
                uv, z = self.project(verts.reshape(-1, 3), size)
                uv, z = uv.reshape(-1, 3, 2), z.reshape(-1, 3)
                ok &= (z >= 0.02).all(1)
                shade = 0.35 + 0.65 * np.maximum(0.0, n @ LIGHT)
                for i in np.flatnonzero(ok):
                    polys.append((float(z[i].mean()), uv[i].astype(np.int32), tuple(int(c * shade[i]) for c in color)))
                continue
            n = np.cross(verts[1] - verts[0], verts[2] - verts[0])
            norm = np.linalg.norm(n)
            if norm < 1e-12:
                continue
            n /= norm
            center = verts.mean(0)
            if n @ (eye - center) <= 0:  # 뒷면
                continue
            uv, z = self.project(verts, size)
            if (z < 0.02).any():
                continue
            shade = 0.35 + 0.65 * max(0.0, float(n @ LIGHT))
            polys.append((float(z.mean()), uv.astype(np.int32), tuple(int(c * shade) for c in color)))
        for _, uv, col in sorted(polys, key=lambda p: -p[0]):
            cv2.fillConvexPoly(img, uv, col, cv2.LINE_AA)
        return img

    def orbit(self, du, dv):
        self.yaw -= du * 0.01
        self.pitch = max(-0.2, min(1.5, self.pitch + dv * 0.01))

    def pan(self, du, dv):
        """화면 픽셀만큼 바라보는 점을 옆·위로 옮김 (Shift+드래그) — 멀리 볼수록 크게"""
        _, right, up, _ = self.basis()
        k = self.dist * 0.0015
        self.target = self.target - right * du * k + up * dv * k

    def zoom(self, steps):
        self.dist = max(0.25, min(4.0, self.dist * (0.9 ** steps)))


def camera_mesh(filename, scale):
    """카메라 렌더용 메시 로더 (scene.BodyModel load_mesh) — package:// 경로 → 간소화 삼각형 (N,3,3), 못 찾으면 None"""
    stl = resolve_mesh(filename)
    return None if stl is None else load_stl(stl, scale, CAMERA_MESH_CELL)


def draw_wheel_spin(img, model, view, size, q, wheel_vel, radius=0.033):
    """바퀴 회전 표시 — 바퀴 중심에서 림까지 스포크: 각도 = 바퀴 관절각(굴러간 만큼 돎), 색 = 회전 중 주황 · 정지 회색.
    바퀴 메시는 원기둥이라 돌아도 안 보여서 따로 그림 (로봇 3D · 조작 창 3D 공용)"""
    frames = model.joint_frames(q)
    for i, (joint, w) in enumerate((('wheel_left_joint', wheel_vel[0]), ('wheel_right_joint', wheel_vel[1]))):
        if joint not in frames or joint not in model.joints:
            continue
        origin, axis = frames[joint]
        axis = np.asarray(axis, float)
        axis = axis / max(np.linalg.norm(axis), 1e-9)
        p0 = np.cross(axis, [0.0, 0.0, 1.0])
        if np.linalg.norm(p0) < 1e-6:
            p0 = np.array([1.0, 0.0, 0.0])
        p0 = p0 / np.linalg.norm(p0)
        p1 = np.cross(axis, p0)
        ang = q.get(joint, 0.0)
        col = (0, 140, 255) if abs(w) > 0.05 else (150, 150, 150)
        rim = [origin + (math.cos(ang + k * math.pi / 2) * p0 + math.sin(ang + k * math.pi / 2) * p1) * radius
               for k in range(4)]  # 스포크 4개 — 한 개보다 도는 게 잘 보임
        uv, z = view.project([origin] + rim, size)
        if (z < 0.02).any():
            continue
        c = tuple(uv[0].astype(int))
        for k in range(4):
            r_ = tuple(uv[1 + k].astype(int))
            cv2.line(img, c, r_, col, 2 if k == 0 else 1, cv2.LINE_AA)
        cv2.circle(img, tuple(uv[1].astype(int)), 3, col, -1, cv2.LINE_AA)  # 0번 스포크 끝 = 회전 기준점
        cv2.putText(img, f'{"L" if i == 0 else "R"}{w:+.1f}', (c[0] + 8, c[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)
