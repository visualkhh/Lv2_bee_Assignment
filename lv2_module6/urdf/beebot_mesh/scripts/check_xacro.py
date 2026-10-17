"""beebot.urdf.xacro(원본) 점검 — xacro로 펼쳐 beebot.urdf와 비교한다 (ROS 환경 필요). 어긋나면 make_urdf.sh로 beebot.urdf를 다시 만든다.
트리·이름, 그리퍼 + = 열림과 mimic, 손가락 좌우, end_effector 위치, 원본 beebot.urdf와 영점 형상 일치, ros2_control 조인트 확인.
"""
import math, os, subprocess, xml.etree.ElementTree as ET
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
XACRO = os.path.join(HERE, '..', 'beebot.urdf.xacro')
SRC = os.path.join(HERE, '..', 'beebot.urdf')
EXP = ET.fromstring(subprocess.run(['xacro', XACRO], check=True, capture_output=True, text=True).stdout
                   .replace('package://ros_system/urdf/meshes/', 'meshes/'))   # make_urdf.sh와 같은 치환
ok = True
def check(c, m):
    global ok
    print(('PASS ' if c else 'FAIL ') + m); ok &= bool(c)

def H(R, p): T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p; return T
def rpyR(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))
def origin(e):
    o = e.find('origin')
    if o is None: return np.eye(4)
    return H(rpyR(*[float(v) for v in o.get('rpy', '0 0 0').split()]), [float(v) for v in o.get('xyz', '0 0 0').split()])

def model(root):
    links = {l.get('name'): l for l in root.findall('link')}
    joints = {j.get('name'): j for j in root.findall('joint')}
    parent = {j.find('child').get('link'): j for j in joints.values()}
    return root, links, joints, parent

def fk(m, q):
    root, links, joints, parent = m
    qq = dict(q)
    for j in joints.values():               # mimic 전파
        mm = j.find('mimic')
        if mm is not None and mm.get('joint') in qq:
            qq[j.get('name')] = qq[mm.get('joint')] * float(mm.get('multiplier', 1)) + float(mm.get('offset', 0))
    T = {}
    def f(l):
        if l in T: return T[l]
        if l not in parent: T[l] = H(np.eye(3), [0.064, 0, -0.00975]); return T[l]
        j = parent[l]; Tj = f(j.find('parent').get('link')) @ origin(j)
        v = qq.get(j.get('name'), 0.0)
        if v:
            a = np.array([float(x) for x in j.find('axis').get('xyz').split()]); a /= np.linalg.norm(a)
            if j.get('type') == 'prismatic': Tj = Tj @ H(np.eye(3), a * v)
            else:
                K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
                Tj = Tj @ H(np.eye(3) + math.sin(v) * K + (1 - math.cos(v)) * K @ K, [0, 0, 0])
        T[l] = Tj; return Tj
    for l in links: f(l)
    return T

e = model(EXP); s = model(ET.parse(SRC).getroot())
root, links, joints, parent = e
roots = [l for l in links if l not in parent]
check(roots == ['base_footprint'], f'루트 = base_footprint ({roots})')
check(set(links) == set(s[1]) and set(joints) == set(s[2]), f'링크 {len(links)} / 조인트 {len(joints)}: beebot.urdf와 이름 집합 같음')
check(not any(n.endswith('_1') for n in links), '링크 이름에 _1 없음')
check(all(l.find('visual/geometry/mesh').get('filename').startswith('meshes/') for l in links.values() if l.find('visual') is not None), '메시 경로가 meshes/ 아래')

# 영점 형상: 이름 매핑 후 링크 프레임이 원본과 같아야 함
Te, Ts = fk(e, {}), fk(s, {})
err = max(np.abs(Te[n] - Ts[n]).max() for n in links if n in Ts)
check(err < 1e-9, f'영점 자세 링크 프레임이 원본 beebot.urdf와 같음 (최대차 {err:.1e})')

# 손가락 좌우와 + = 열림, mimic
yl, yr = Te['finger_left_link'][1, 3], Te['finger_right_link'][1, 3]
check(yl > 0 > yr, f'finger_left가 +Y(왼쪽) {yl*1000:.1f} mm, finger_right가 -Y {yr*1000:.1f} mm')
def gap(T): return np.linalg.norm(T['finger_left_link'][:3, 3] - T['finger_right_link'][:3, 3])
T1 = fk(e, {'gripper_left_joint': 0.0265})
check(gap(T1) - gap(Te) > 0.052, f'gripper_left_joint +26.5 mm → 손가락 간격 {gap(Te)*1000:.1f} → {gap(T1)*1000:.1f} mm (양쪽 열림, mimic 동작)')
gl = joints['gripper_left_joint'].find('limit'); ml = joints['gripper_right_joint'].find('mimic')
check(float(gl.get('lower')) == 0 and abs(float(gl.get('upper')) - 0.0265) < 1e-9, f'gripper 범위 {gl.get("lower")} ~ {gl.get("upper")} m')
check(ml is not None and ml.get('joint') == 'gripper_left_joint', 'gripper_right_joint → gripper_left_joint mimic')

# tcp
tcp = Te['end_effector_link'][:3, 3] * 1000
check(np.allclose(tcp, [435.0, 0.0, 345.545], atol=0.01), f'end_effector 위치 {tcp.round(2)} mm (집는 점)')
check(np.allclose(Te['end_effector_link'][:3, 2], [1, 0, 0], atol=1e-6), 'end_effector z축 = 접근 방향(전방 +X)')
Tr = fk(e, {'joint5': math.pi / 2})
check(np.linalg.norm(Tr['end_effector_link'][:3, 3] - Te['end_effector_link'][:3, 3]) < 0.002, 'joint5 롤 90°에 end_effector 위치 유지 (롤 축 위, 2 mm 이내)')

# optical frame
for cam in ('pole_cam', 'gripper_cam'):
    Tl, To = Te[f'{cam}_link'], Te[f'{cam}_color_optical_frame']
    check(np.allclose(To[:3, 2], Tl[:3, 0], atol=1e-6) and np.allclose(To[:3, 0], -Tl[:3, 1], atol=1e-6), f'{cam}_color_optical_frame: z=렌즈, x=오른쪽')

# limits / ros2_control
for n, j in joints.items():
    lim = j.find('limit')
    if lim is not None and float(lim.get('effort', 0)) >= 1e5: check(False, f'{n} effort 자리표시값 남음')
check(True, '자리표시 effort/velocity 없음')
rc = [x.get('name') for x in root.find('ros2_control').findall('joint')]
check(rc == ['wheel_left_joint', 'wheel_right_joint', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_left_joint'], f'ros2_control 조인트 {rc}')
# xacro 인자: launch가 넘기는 값이 ros2_control <param>에 들어가는지
hw = ET.fromstring(subprocess.run(['xacro', XACRO, 'mock_hardware:=true', 'mock_effort:=1.5', 'port:=/dev/ttyUSB9'],
                                  check=True, capture_output=True, text=True).stdout).find('ros2_control/hardware')
got = {p.get('name'): p.text for p in hw.findall('param')}
check(got.get('mock') == 'true' and got.get('mock_effort') == '1.5' and got.get('port') == '/dev/ttyUSB9',
      f'xacro 인자 mock_hardware·mock_effort·port 반영 ({got})')
print('\nRESULT:', 'ALL PASS' if ok else 'FAIL')
raise SystemExit(0 if ok else 1)
