"""BEE 팔 정적 중력 토크 점검 (beebot.urdf 기준).
J2~J5 가동 범위를 격자로 훑어 각 관절의 최대 중력 토크를 구하고, XM430-W350 정지 토크와 비교한다.
가속·마찰은 빼고 '멈춰서 버티는' 토크만 본다.
"""
import math, os, itertools, xml.etree.ElementTree as ET
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, '..', 'beebot.urdf')
G = np.array([0, 0, -9.81])
STALL_NM = 4.1                    # XM430-W350 정지 토크 (12 V)
PAYLOADS_G = [0, 50, 100, 200]    # 퍽 질량 후보 (미정)
GRIP_ASM_MM = np.array([435.0, 0.0, 345.545])   # 영점 자세에서 두 손가락 사이 집는 점 (끝 455.6 mm에서 20 mm 안쪽)
STEP = math.radians(10)

def H(R, p): T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p; return T
def rpyR(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))
def origin(e):
    o = e.find('origin')
    if o is None: return np.eye(4)
    return H(rpyR(*[float(v) for v in o.get('rpy', '0 0 0').split()]), [float(v) for v in o.get('xyz', '0 0 0').split()])
def axis_rot(ax, q):
    ax = ax / np.linalg.norm(ax); K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K

root = ET.parse(URDF).getroot()
links = {l.get('name'): l for l in root.findall('link')}
joints = {j.get('name'): j for j in root.findall('joint')}
parent = {j.find('child').get('link'): j for j in joints.values()}
inert = {}
for n, l in links.items():
    i = l.find('inertial')
    if i is not None: inert[n] = (float(i.find('mass').get('value')), origin(i)[:3, 3])

ARM_J = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
lim = {j: (float(joints[j].find('limit').get('lower')), float(joints[j].find('limit').get('upper'))) for j in ARM_J}

def fk(q):
    T = {}
    def f(l):
        if l in T: return T[l]
        if l not in parent: T[l] = H(np.eye(3), [0.064, 0, -0.00975]); return T[l]   # base_footprint 조립 좌표
        j = parent[l]; Tj = f(j.find('parent').get('link')) @ origin(j)
        if j.get('name') in q:
            ax = np.array([float(v) for v in j.find('axis').get('xyz').split()])
            Tj = Tj @ H(axis_rot(ax, q[j.get('name')]), [0, 0, 0])
        T[l] = Tj; return Tj
    for l in links: f(l)
    return T

def descendants(link):
    out = [link]
    for c, j in parent.items():
        if j.find('parent').get('link') == link: out += descendants(c)
    return out

# 각 관절의 자식 쪽 링크들, 관절 축(월드), 집는 점
child_of = {j: joints[j].find('child').get('link') for j in ARM_J}
desc = {j: descendants(child_of[j]) for j in ARM_J}
T0 = fk({})
grip_local = np.linalg.inv(T0['gripper_link']) @ np.append(GRIP_ASM_MM / 1000, 1)   # gripper_body 좌표로 고정

def torques(q, payload_kg):
    T = fk(q)
    out = {}
    for j in ARM_J:
        jj = joints[j]
        Tj = T[jj.find('parent').get('link')] @ origin(jj)
        p = Tj[:3, 3]; a = Tj[:3, :3] @ np.array([float(v) for v in jj.find('axis').get('xyz').split()])
        tau = 0.0
        for l in desc[j]:
            if l in inert:
                m, c = inert[l]; cw = T[l][:3, :3] @ c + T[l][:3, 3]
                tau += np.dot(a, np.cross(cw - p, m * G))
        gw = (T['gripper_link'] @ grip_local)[:3]
        tau += np.dot(a, np.cross(gw - p, payload_kg * G))
        out[j] = -tau          # 모터가 내야 하는 토크 = 중력 토크의 반대
    return out

def grid(j):
    lo, hi = lim[j]; return np.append(np.arange(lo, hi, STEP), hi)

# 가지 질량 확인
print('관절별로 들어 올리는 질량 (g):', {j: round(sum(inert[l][0] for l in desc[j] if l in inert) * 1000, 1) for j in ARM_J})
best = {}
for pay in PAYLOADS_G:
    worst = {j: (0.0, None) for j in ARM_J}
    for q2, q3, q4, q5 in itertools.product(grid('joint2'), grid('joint3'), grid('joint4'), np.radians([-90, 0, 90])):
        q = {'joint2': q2, 'joint3': q3, 'joint4': q4, 'joint5': q5}
        t = torques(q, pay / 1000)
        for j in ARM_J:
            if abs(t[j]) > abs(worst[j][0]): worst[j] = (t[j], q)
    best[pay] = worst
    print(f'\n== 퍽 {pay} g ==')
    for j in ARM_J[1:]:
        tau, q = worst[j]
        qs = ', '.join(f'{k.replace('joint', 'J')} {math.degrees(v):+.0f}°' for k, v in q.items()) if q else '-'
        print(f'  {j:15s} 최대 {abs(tau):5.2f} N·m = 정지토크의 {abs(tau)/STALL_NM*100:4.0f} %   자세: {qs}')

# 대표 자세: 팔을 앞으로 수평으로 쭉 뻗은 자세 찾기 (집는 점이 가장 멀리 + 같은 높이 근처)
print('\n== 수평으로 가장 멀리 뻗은 자세 ==')
cands = []
for q2, q3, q4 in itertools.product(grid('joint2'), grid('joint3'), grid('joint4')):
    q = {'joint2': q2, 'joint3': q3, 'joint4': q4, 'joint5': 0.0}
    T = fk(q); g = (T['gripper_link'] @ grip_local)[:3]; s = T['upper_arm_link'][:3, 3]
    cands.append((g[0] - s[0], abs(g[2] - s[2]), q, g))
far = max((c for c in cands if c[1] < 0.03), key=lambda c: c[0])
print(f'  어깨에서 집는 점까지 수평 거리 {far[0]*1000:.0f} mm, 높이 차 {far[1]*1000:.0f} mm, 자세',
      {k.replace('joint', 'J'): round(math.degrees(v)) for k, v in far[2].items()})
for pay in PAYLOADS_G:
    t = torques(far[2], pay / 1000)
    print(f'  퍽 {pay:3d} g: ' + '  '.join(f'{j.replace('joint', 'J')} {abs(t[j]):.2f} N·m ({abs(t[j])/STALL_NM*100:.0f}%)' for j in ARM_J[1:]))
