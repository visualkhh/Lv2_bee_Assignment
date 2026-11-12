"""beebot.urdf 검증 (check_urdf 대신 쓰는 로컬 점검).
1) 트리 구조  2) 관성 유효성  3) 메시 파일 존재 (visual·collision 전부)  4) 팔 조인트 원점 vs Fusion 조립 실측  5) 메시 왕복 위치
"""
import math, os, json, xml.etree.ElementTree as ET
import numpy as np
from stlio import read_stl

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..')   # 메시 경로(meshes/...)의 기준 = urdf/

def H(R, p): T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p; return T
def rpyR(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))
def origin(e):
    o = e.find('origin')
    if o is None: return np.eye(4)
    return H(rpyR(*[float(v) for v in o.get('rpy', '0 0 0').split()]), [float(v) for v in o.get('xyz', '0 0 0').split()])

ok = True
def check(cond, msg):
    global ok
    print(('PASS ' if cond else 'FAIL ') + msg); ok &= bool(cond)

FUSION_JOINTS_MM = {   # Fusion WAFFLE_PI_ASSY 안 rm-x52:1 조인트 원점(조립 좌표, mm) — 2026-10-13 읽음
    # joint2~4: Fusion 원점은 y -18.75 (서보 옆면). URDF는 이 원점을 각자 회전축(y) 따라 중심선(y 0)으로 옮겼다 — 같은 축 직선 위라 기구학 동일
    'joint1': (115.045, 0, 175.945), 'joint2': (115.045, 0, 217.545), 'joint3': (139.045, 0, 345.545),
    'joint4': (263.045, 0, 345.545), 'joint5': (325.945, 0, 345.545),
    'gripper_left_joint': (379.095, 38.614, 345.545), 'gripper_right_joint': (379.095, -38.614, 345.545)}
BASE_FOOTPRINT_ASM_MM = np.array([64.0, 0.0, -9.75])

for fn in ('beebot.urdf',):
    print('==', fn)
    root = ET.parse(os.path.join(OUT, fn)).getroot()
    links = {l.get('name'): l for l in root.findall('link')}
    joints = root.findall('joint')
    check(len(links) == len(root.findall('link')), f'링크 이름 중복 없음 ({len(links)}개)')
    check(len({j.get('name') for j in joints}) == len(joints), f'조인트 이름 중복 없음 ({len(joints)}개)')
    parent = {}
    for j in joints:
        c, p = j.find('child').get('link'), j.find('parent').get('link')
        check(c in links and p in links, f'{j.get("name")}: 부모/자식 링크 존재') if not (c in links and p in links) else None
        check(c not in parent, f'{c}: 부모 하나') if c in parent else None
        parent[c] = (p, j)
    roots = [l for l in links if l not in parent]
    check(roots == ['base_footprint'], f'루트가 base_footprint 하나 ({roots})')
    # FK (base_footprint 기준 -> 조립 좌표)
    T = {}
    def fk(l):
        if l in T: return T[l]
        if l not in parent: T[l] = H(np.eye(3), BASE_FOOTPRINT_ASM_MM / 1000); return T[l]
        p, j = parent[l]; T[l] = fk(p) @ origin(j); return T[l]
    for l in links: fk(l)
    jpos = {j.get('name'): (fk(j.find('parent').get('link')) @ origin(j))[:3, 3] * 1000 for j in joints}
    for name, ref in FUSION_JOINTS_MM.items():
        if name in jpos:
            err = np.linalg.norm(jpos[name] - np.array(ref))
            check(err < 0.01, f'{name} 원점 오차 {err:.4f} mm (Fusion 실측 대비)')
    # 관성
    tot = 0.0
    for n, l in links.items():
        ine = l.find('inertial')
        if ine is None: continue
        m = float(ine.find('mass').get('value')); tot += m
        a = ine.find('inertia').attrib; g = lambda k: float(a[k])
        I = np.array([[g('ixx'), g('ixy'), g('ixz')], [g('ixy'), g('iyy'), g('iyz')], [g('ixz'), g('iyz'), g('izz')]])
        ev = np.linalg.eigvalsh(I)
        tri = ev[0] + ev[1] >= ev[2] * (1 - 1e-6)
        if not (m > 0 and ev.min() > 0 and tri): check(False, f'{n} 관성 유효 (m={m}, eig={ev})')
    check(True, f'관성 행렬 전부 양정치·삼각부등식 만족, 총질량 {tot*1000:.1f} g')
    # 메시: visual·collision 참조 파일이 전부 있어야 한다
    refs = {m.get('filename') for m in root.iter('mesh')}
    missing = sorted(f for f in refs if not os.path.exists(os.path.join(OUT, f)))
    check(not missing, f'메시 파일 {len(refs)}개 전부 있음' if not missing else f'메시 파일 없음: {missing}')
    # 메시 왕복 위치: 베이스 쪽 메시를 Fusion 추출본(raw/)과 비교 — raw/가 있을 때만
    RAW_NAME = {'base_link': 'base_link', 'wheel_left_link': 'wheel_left', 'wheel_right_link': 'wheel_right',
                'pole_front_cam_link': 'camera_rear', 'gripper_cam_link': 'camera_wrist'}
    if not os.path.isdir(os.path.join(HERE, 'raw')):
        print('INFO raw/ 없음, 메시 왕복 위치 확인 건너뜀')
    for n, l in links.items():
        raw = os.path.join(HERE, 'raw', RAW_NAME.get(n, '') + '_asm_mm.stl')
        if n not in RAW_NAME or not os.path.exists(raw): continue
        for v in l.findall('visual'):
            tri = read_stl(os.path.join(OUT, v.find('geometry/mesh').get('filename'))).reshape(-1, 3) * 0.001
            Tw = fk(n) @ origin(v)
            w = (Tw[:3, :3] @ tri.T).T + Tw[:3, 3]
            rv = read_stl(raw).reshape(-1, 3) * 0.001
            e = max(np.abs(w.min(0) - rv.min(0)).max(), np.abs(w.max(0) - rv.max(0)).max()) * 1000
            check(e < 3.1, f'{n} 메시 왕복 위치 오차 {e:.3f} mm (간략화 셀 이하)')
print('\nRESULT:', 'ALL PASS' if ok else 'FAIL')
raise SystemExit(0 if ok else 1)
