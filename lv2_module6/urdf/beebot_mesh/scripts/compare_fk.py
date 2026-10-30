"""두 URDF가 물리적으로 같은 로봇인지 비교 — 링크 프레임을 다시 잡은 뒤(축 정렬 등) 검증용.
관절 각도 여러 세트에서 base_footprint 기준 world 자세를 비교한다:
  · 모든 링크의 visual·collision·inertial 원점 (메시·충돌·질량이 같은 곳에 있는지)
  · 질량·관성 행렬, 관절 종류·한계·mimic
  · 형상 없는 링크(end_effector_link, 카메라 광학 프레임 등)의 프레임
링크 프레임 자체는 비교하지 않는다(다시 잡은 것이므로).
사용: python3 compare_fk.py old.urdf new.urdf   (통과 0, 실패 1)
"""
import math, sys
import numpy as np
import xml.etree.ElementTree as ET

TOL = 1e-9  # m / rad


def rpyR(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def origin(e):
    T = np.eye(4)
    o = e.find('origin')
    if o is not None:
        T[:3, :3] = rpyR(*map(float, o.get('rpy', '0 0 0').split()))
        T[:3, 3] = list(map(float, o.get('xyz', '0 0 0').split()))
    return T


def motion(j, q):
    T = np.eye(4)
    a = np.array(list(map(float, j.find('axis').get('xyz').split()))) if j.find('axis') is not None else np.array([1., 0, 0])
    a = a / np.linalg.norm(a)
    if j.get('type') in ('revolute', 'continuous'):
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        T[:3, :3] = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K
    elif j.get('type') == 'prismatic':
        T[:3, 3] = a * q
    return T


class Model:
    def __init__(self, path):
        self.root = ET.parse(path).getroot()
        self.links = {l.get('name'): l for l in self.root.findall('link')}
        self.joints = {j.get('name'): j for j in self.root.findall('joint')}
        self.parent = {j.find('child').get('link'): j for j in self.joints.values()}

    def fk(self, q):
        q = dict(q)
        for j in self.joints.values():
            m = j.find('mimic')
            if m is not None:
                q[j.get('name')] = q.get(m.get('joint'), 0.0) * float(m.get('multiplier', 1)) + float(m.get('offset', 0))
        W = {}
        def f(l):
            if l not in W:
                j = self.parent.get(l)
                W[l] = np.eye(4) if j is None else f(j.find('parent').get('link')) @ origin(j) @ motion(j, q.get(j.get('name'), 0.0))
            return W[l]
        for l in self.links: f(l)
        return W

    def geometry(self, W):
        """(link, tag, index) -> world pose of that geometry/inertial origin; bare links -> their frame."""
        out = {}
        for n, l in self.links.items():
            tags = [(t, i, e) for t in ('inertial', 'visual', 'collision') for i, e in enumerate(l.findall(t))]
            for t, i, e in tags:
                out[(n, t, i)] = W[n] @ origin(e)
            if not tags:
                out[(n, 'frame', 0)] = W[n]
        return out


def pose_err(A, B):
    dp = np.linalg.norm(A[:3, 3] - B[:3, 3])
    return dp, np.linalg.norm(A[:3, :3] - B[:3, :3]) / math.sqrt(2)  # = 2 sin(angle/2) — acos(trace) loses precision near 0


a, b = Model(sys.argv[1]), Model(sys.argv[2])
ok = True
def check(cond, msg):
    global ok
    print(('PASS ' if cond else 'FAIL ') + msg); ok &= bool(cond)

check(set(a.links) == set(b.links) and set(a.joints) == set(b.joints), '링크·관절 이름 같음')
for n, ja in a.joints.items():
    jb = b.joints[n]
    same = ja.get('type') == jb.get('type') and ja.find('parent').get('link') == jb.find('parent').get('link')
    for tag in ('limit', 'mimic'):
        ea, eb = ja.find(tag), jb.find(tag)
        same &= (ea is None) == (eb is None) and (ea is None or ea.attrib == eb.attrib)
    if not same: check(False, f'{n} 종류·부모·limit·mimic 같음')
check(ok, '관절 종류·부모·limit·mimic 전부 같음')
for n, la in a.links.items():
    ia, ib = la.find('inertial'), b.links[n].find('inertial')
    if ia is None or ib is None: continue
    if ia.find('mass').attrib != ib.find('mass').attrib or ia.find('inertia').attrib != ib.find('inertia').attrib:
        check(False, f'{n} 질량·관성 행렬 같음')

rng = np.random.default_rng(0)
movable = [n for n, j in a.joints.items() if j.get('type') != 'fixed' and j.find('mimic') is None]
def rand_q():
    q = {}
    for n in movable:
        lim = a.joints[n].find('limit')
        lo, hi = (float(lim.get('lower', -math.pi)), float(lim.get('upper', math.pi))) if lim is not None and lim.get('lower') else (-math.pi, math.pi)
        q[n] = rng.uniform(lo, hi)
    return q
configs = [('zero', {})] + [(f'random{i}', rand_q()) for i in range(20)]
worst = (0.0, 0.0, '')
for name, q in configs:
    ga, gb = a.geometry(a.fk(q)), b.geometry(b.fk(q))
    if set(ga) != set(gb):
        check(False, f'{name}: 형상 목록 같음 ({set(ga) ^ set(gb)})'); continue
    for k in ga:
        dp, dr = pose_err(ga[k], gb[k])
        if max(dp, dr) > max(worst[:2]): worst = (dp, dr, f'{name} {k}')
check(max(worst[:2]) < TOL, f'{len(configs)}개 자세 × 형상 전부 일치, 최대 오차 위치 {worst[0]:.2e} m · 방향 {worst[1]:.2e} rad ({worst[2]})')
print('\nRESULT:', 'ALL PASS' if ok else 'FAIL')
raise SystemExit(0 if ok else 1)
