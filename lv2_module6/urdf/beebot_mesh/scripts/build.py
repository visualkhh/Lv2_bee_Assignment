"""BEE 베이스 URDF 생성 — Fusion에서 뽑은 raw/bodies.json + raw/*_asm_mm.stl 로부터.
입력: raw/ (fusion_extract.py 결과), rm_x52.urdf (팔, 단위 5 산출물)
출력: out/bee_base.urdf, out/bee.urdf, out/meshes/*.stl, out/build_report.json
"""
import json, math, os, xml.etree.ElementTree as ET
import numpy as np
from stlio import read_stl, write_stl, cluster

HERE = os.path.dirname(os.path.abspath(__file__))
RAW, OUT = os.path.join(HERE, 'raw'), os.path.join(HERE, 'out')
os.makedirs(os.path.join(OUT, 'meshes'), exist_ok=True)

# ---------- 1. 질량 목표 (g, g/cm^3) ----------
WAFFLE_TARGET_G = 1800.0 - 114.0 - 2 * 28.5   # 공식 Waffle Pi 1.8 kg - LiDAR 114 g - 바퀴 2개(공식 URDF 28.5 g)
WHEEL_TARGET_G = 28.5                          # 공식 URDF 바퀴 1개
CAMERA_TARGET_G = 72.0                         # D435 (사용자 값)
POLE_TARGET_G = 225.0                          # 가정: 2020 알루미늄 프로파일 ~0.5 kg/m x 0.45 m
BASKET_DENSITY = 1.24 * 0.5                    # 가정: PLA 1.24 g/cm^3, 실효 50 %
BRACKET_DENSITY = 1.06                         # 출력물 (모델 재질 ABS 밀도 그대로)

d = json.load(open(os.path.join(RAW, 'bodies.json')))
recs = [r for r in d['recs'] if r['vol'] > 0]   # 서피스 바디(나사 등, 부피 없음)는 질량 0
vol = {}
for r in recs:
    vol[(r['link'], r['cat'])] = vol.get((r['link'], r['cat']), 0.0) + r['vol']


def density(r):
    k = (r['link'], r['cat'])
    if r['cat'] == 'waffle': return WAFFLE_TARGET_G / vol[k]
    if r['cat'] == 'wheel': return WHEEL_TARGET_G / vol[k]
    if r['cat'] == 'camera': return CAMERA_TARGET_G / vol[k]
    if r['cat'] == 'pole': return POLE_TARGET_G / vol[k]
    if r['cat'] == 'basket': return BASKET_DENSITY
    if r['cat'] == 'bracket': return BRACKET_DENSITY
    raise KeyError(k)


# ---------- 2. 링크별 질량·질량중심·관성 (조립 좌표, SI) ----------
# Fusion getXYZMomentsOfInertia = 월드 원점 기준 텐서 성분(곱 항 부호 포함), kg cm^2 — 주관성 모멘트와 대조해 확인함
links = {}
for r in recs:
    f = (density(r) * r['vol'] / 1000.0) / r['mass']          # 새 질량 / Fusion 질량 (관성도 같은 비율)
    L = links.setdefault(r['link'], dict(m=0.0, mc=np.zeros(3), Io=np.zeros((3, 3)), parts={}))
    m = r['mass'] * f
    xx, yy, zz, xy, yz, xz = [v * f * 1e-4 for v in r['I']]
    L['m'] += m
    L['mc'] += m * np.array(r['com']) * 0.01
    L['Io'] += np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
    L['parts'][r['cat']] = L['parts'].get(r['cat'], 0.0) + m
for L in links.values():
    c = L['mc'] / L['m']
    L['com'] = c
    L['Ic'] = L['Io'] - L['m'] * (np.dot(c, c) * np.eye(3) - np.outer(c, c))

# ---------- 3. 프레임 (조립 좌표, m) ----------
tb = d['tire_bb_mm']
t1, t2 = tb['PR16_A01_TIRE_2:1'], tb['PR16_A01_TIRE_2:2']    # :1 = -Y(오른쪽), :2 = +Y(왼쪽)
ground_z = min(t1[2], t2[2]) / 1000
wheel_x = (t1[0] + t1[3]) / 2000
wheel_z = (t1[2] + t1[5]) / 2000
wheel_r = (t1[5] - t1[2]) / 2000
wheel_w = (t1[4] - t1[1]) / 1000
wl = np.array([wheel_x, (t2[1] + t2[4]) / 2000, wheel_z])
wr = np.array([wheel_x, (t1[1] + t1[4]) / 2000, wheel_z])


def H(R=np.eye(3), p=(0, 0, 0)):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p; return T
def Rx(a): c, s = math.cos(a), math.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def Ry(a): c, s = math.cos(a), math.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def Rz(a): c, s = math.cos(a), math.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
def from_xyz_rpy(xyz, rpy): return H(Rz(rpy[2]) @ Ry(rpy[1]) @ Rx(rpy[0]), xyz)
def rpy_of(R):
    p = math.asin(max(-1, min(1, -R[2, 0])))
    if abs(math.cos(p)) > 1e-9:
        return (math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0]))
    return (math.atan2(-R[1, 2], R[1, 1]), p, 0.0)
def fus(name):  # Fusion transform2 (cm) -> 4x4 (m)
    a = np.array(d['transforms'][name], dtype=float).reshape(4, 4); a[:3, 3] *= 0.01; return a


T_fp = H(p=(wheel_x, 0, ground_z))                      # base_footprint: 바퀴축 아래 바닥
T_base = H(p=(wheel_x, 0, ground_z + 0.010))            # base_link: 공식 Waffle Pi처럼 바닥 +10 mm
T_wl, T_wr = H(p=wl), H(p=wr)
T_armbase = fus('rm-x52:1') @ H(Rx(-math.pi / 2))       # 원본 rm-x52(Y-up) -> 내보낸 팔(Z-up, 단위 5)
P_ros = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]], float)  # 열 = ROS x,y,z 를 카메라 로컬로: x=로컬 z(렌즈), y=로컬 x, z=로컬 y(위)
T_cam_rear = fus('RealSense:1') @ H(P_ros)
T_cam_wrist = fus('RealSense:2') @ H(P_ros)

# 팔 URDF 영점 순기구학 -> gripper_body_1 의 조립 좌표
arm = ET.parse(os.path.join(HERE, 'rm_x52.urdf')).getroot()
ajoints = {j.find('child').get('link'): j for j in arm.findall('joint')}
def arm_fk(link):
    T = np.eye(4)
    while link in ajoints:
        j = ajoints[link]; o = j.find('origin')
        T = from_xyz_rpy([float(v) for v in o.get('xyz').split()], [float(v) for v in o.get('rpy').split()]) @ T
        link = j.find('parent').get('link')
    return T
T_grip = T_armbase @ arm_fk('gripper_body_1')

frames = dict(base_footprint=T_fp, base_link=T_base, wheel_left_link=T_wl, wheel_right_link=T_wr,
              camera_rear_link=T_cam_rear, camera_wrist_link=T_cam_wrist)
srcname = dict(base_link='base_link', wheel_left_link='wheel_left', wheel_right_link='wheel_right',
               camera_rear_link='camera_rear', camera_wrist_link='camera_wrist')

# ---------- 4. 메시 (링크 좌표, mm, scale 0.001 — 팔과 같은 규칙) + 간략화 ----------
CELL = dict(base_link=3.0, wheel_left_link=1.0, wheel_right_link=1.0, camera_rear_link=0.0, camera_wrist_link=0.0)
mesh_info, local_bb = {}, {}
for ln, src in srcname.items():
    tri = read_stl(os.path.join(RAW, src + '_asm_mm.stl'))
    T = np.linalg.inv(frames[ln])
    v = tri.reshape(-1, 3) / 1000.0
    v = (T[:3, :3] @ v.T).T + T[:3, 3]
    tri_l = (v * 1000.0).reshape(-1, 3, 3)
    n0 = len(tri_l)
    if CELL[ln] > 0: tri_l = cluster(tri_l, CELL[ln])
    write_stl(os.path.join(OUT, 'meshes', ln + '.stl'), tri_l)
    vv = tri_l.reshape(-1, 3)
    local_bb[ln] = (vv.min(0) / 1000, vv.max(0) / 1000)
    mesh_info[ln] = dict(tri_in=n0, tri_out=len(tri_l), cell_mm=CELL[ln], bytes=os.path.getsize(os.path.join(OUT, 'meshes', ln + '.stl')))

# ---------- 5. URDF 쓰기 ----------
f6 = lambda x: f'{x:.6f}'
def xyz(v): return ' '.join(f6(a) for a in v)
def inertial(ln):
    L = links[srcname[ln]]; Tl = frames[ln]; R = Tl[:3, :3]
    c = R.T @ (L['com'] - Tl[:3, 3]); I = R.T @ L['Ic'] @ R
    return (f'    <inertial>\n      <origin xyz="{xyz(c)}" rpy="0 0 0"/>\n      <mass value="{L["m"]:.6f}"/>\n'
            f'      <inertia ixx="{I[0,0]:.4e}" ixy="{I[0,1]:.4e}" ixz="{I[0,2]:.4e}" iyy="{I[1,1]:.4e}" iyz="{I[1,2]:.4e}" izz="{I[2,2]:.4e}"/>\n    </inertial>\n')
def visual(ln):
    return (f'    <visual>\n      <origin xyz="0 0 0" rpy="0 0 0"/>\n      <geometry><mesh filename="meshes/{ln}.stl" scale="0.001 0.001 0.001"/></geometry>\n    </visual>\n')
def box(c, s, rpy=(0, 0, 0)):
    return (f'    <collision>\n      <origin xyz="{xyz(c)}" rpy="{xyz(rpy)}"/>\n      <geometry><box size="{xyz(s)}"/></geometry>\n    </collision>\n')
def to_base(p_asm_mm):
    return (np.linalg.inv(T_base) @ np.append(np.array(p_asm_mm) / 1000, 1))[:3]
def box_asm(lo, hi):  # 조립 좌표 mm 박스 -> base_link 좌표 콜리전
    lo, hi = to_base(lo), to_base(hi); return box((lo + hi) / 2, hi - lo)

g = ground_z * 1000
ball_r = 0.0047625                                                                        # 3/8" 강구
base_coll = (box_asm((-168.5, -132.8, g + 10), (132.8, 132.8, 91.0))                     # 와플 3층 상판 윗면(z 91)까지. 그 위 바구니 공간은 비움
             + box_asm((59.2, -68.8, 91.0), (132.8, 68.8, 126.6))                         # 4층 지지부 (판 아래)
             + box_asm((-155.4, -10.0, 91.0), (-135.4, 10.0, 126.6))                      # 기둥 아래 (z 91~126.6)
             + box_asm((59.2, -68.8, 126.6), (132.8, 68.8, 136.4))                        # FL4 팔 받침판
             + box_asm((-155.4, -10.0, 126.6), (-135.4, 10.0, 457.7)))                    # 기둥
# 저장 랙(바구니) 볼록 조각 9개 — Houdini 분해 결과 (meshes/storage_collision/). 홈 12개를 열어 둔다.
# 조각 STL(x 길이 195 · y 두께 35 · z 폭 ±130) -> base_link: x축 +90° 회전, x -64.0 mm, z -0.5 mm (base_link.stl 꼭짓점과 맞춘 값)
import shutil
shutil.copytree(os.path.join(HERE, '..', 'meshes', 'storage_collision'), os.path.join(OUT, 'meshes', 'storage_collision'), dirs_exist_ok=True)
for i in range(9):
    base_coll += (f'    <collision name="storage_collision_{i}">\n      <origin xyz="-0.064000 0.000000 -0.000500" rpy="1.570796 0.000000 0.000000"/>\n'
                  f'      <geometry><mesh filename="meshes/storage_collision/storage_{i}.stl" scale="0.001 0.001 0.001"/></geometry>\n    </collision>\n')
for y in (64.0, -64.0):                                                                   # 캐스터 볼: 바퀴 밑면 높이에 닿게
    c = to_base((-113.0, y, g + ball_r * 1000))
    base_coll += f'    <collision>\n      <origin xyz="{xyz(c)}" rpy="0 0 0"/>\n      <geometry><sphere radius="{ball_r:.6f}"/></geometry>\n    </collision>\n'
def wheel_coll():
    return (f'    <collision>\n      <origin xyz="0 0 0" rpy="1.570796 0 0"/>\n      <geometry><cylinder radius="{wheel_r:.6f}" length="{wheel_w:.6f}"/></geometry>\n    </collision>\n')
def cam_coll(ln):
    lo, hi = local_bb[ln]; return box((lo + hi) / 2, hi - lo)
def link(ln, coll):
    return f'  <link name="{ln}">\n' + inertial(ln) + visual(ln) + coll + '  </link>\n'
def joint(name, typ, parent, child, Tp, Tc, axis=None):
    T = np.linalg.inv(Tp) @ Tc
    s = (f'  <joint name="{name}" type="{typ}">\n    <parent link="{parent}"/>\n    <child link="{child}"/>\n'
         f'    <origin xyz="{xyz(T[:3,3])}" rpy="{xyz(rpy_of(T[:3,:3]))}"/>\n')
    if axis: s += f'    <axis xyz="{axis}"/>\n'
    return s + '  </joint>\n'

base_body = ('  <link name="base_footprint"/>\n'
             + joint('base_joint', 'fixed', 'base_footprint', 'base_link', T_fp, T_base)
             + link('base_link', base_coll)
             + joint('wheel_left_joint', 'continuous', 'base_link', 'wheel_left_link', T_base, T_wl, '0 1 0')
             + link('wheel_left_link', wheel_coll())
             + joint('wheel_right_joint', 'continuous', 'base_link', 'wheel_right_link', T_base, T_wr, '0 1 0')
             + link('wheel_right_link', wheel_coll())
             + joint('camera_rear_joint', 'fixed', 'base_link', 'camera_rear_link', T_base, T_cam_rear)
             + link('camera_rear_link', cam_coll('camera_rear_link')))
hdr = '<?xml version="1.0" encoding="utf-8"?>\n<!-- generated by build.py from WAFFLE_PI_ASSY (Fusion, read-only extraction) -->\n'
open(os.path.join(OUT, 'bee_base.urdf'), 'w', encoding='utf-8').write(hdr + '<robot name="bee_base">\n' + base_body + '</robot>\n')

arm_xml = ''
for e in arm:
    if e.tag in ('link', 'joint'):
        ET.indent(e, space='  ', level=1)
        arm_xml += '  ' + ET.tostring(e, encoding='unicode').strip() + '\n'
full = (hdr + '<robot name="bee">\n' + base_body
        + joint('arm_mount_joint', 'fixed', 'base_link', 'arm_base_1', T_base, T_armbase)
        + arm_xml
        + joint('camera_wrist_joint', 'fixed', 'gripper_body_1', 'camera_wrist_link', T_grip, T_cam_wrist)
        + link('camera_wrist_link', cam_coll('camera_wrist_link'))
        + '</robot>\n')
open(os.path.join(OUT, 'bee.urdf'), 'w', encoding='utf-8').write(full)

def rel(Ta, Tb):
    T = np.linalg.inv(Ta) @ Tb
    return dict(xyz=T[:3, 3].round(6).tolist(), rpy=[round(a, 6) for a in rpy_of(T[:3, :3])])
rep = dict(
    targets=dict(waffle_g=WAFFLE_TARGET_G, wheel_g=WHEEL_TARGET_G, camera_g=CAMERA_TARGET_G, pole_g=POLE_TARGET_G,
                 basket_density=BASKET_DENSITY, bracket_density=BRACKET_DENSITY),
    links={k: dict(mass_g=round(v['m'] * 1000, 1), parts_g={a: round(b * 1000, 1) for a, b in v['parts'].items()},
                   com_asm_mm=[round(x * 1000, 2) for x in v['com']]) for k, v in links.items()},
    wheel=dict(radius_mm=round(wheel_r * 1000, 3), width_mm=round(wheel_w * 1000, 3), left=(wl * 1000).round(3).tolist(),
               right=(wr * 1000).round(3).tolist(), ground_z_mm=round(ground_z * 1000, 3)),
    frames_asm_mm={k: dict(p=(v[:3, 3] * 1000).round(3).tolist(), rpy=[round(a, 6) for a in rpy_of(v[:3, :3])])
                   for k, v in dict(frames, arm_base_1=T_armbase, gripper_body_1=T_grip).items()},
    mount_base_to_arm=rel(T_base, T_armbase),
    camera_rear_in_base=rel(T_base, T_cam_rear),
    camera_wrist_in_gripper=rel(T_grip, T_cam_wrist),
    meshes=mesh_info)
json.dump(rep, open(os.path.join(OUT, 'build_report.json'), 'w', encoding='utf-8'), indent=1, ensure_ascii=False)
print(json.dumps(rep, indent=1, ensure_ascii=False))
