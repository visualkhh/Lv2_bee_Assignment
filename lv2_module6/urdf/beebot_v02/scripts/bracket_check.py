"""롤 브라켓 FR12_R101_ROLL_BRACKET 강도·강성 손계산 (보·판 이론, FEA 아님).
하중은 beebot.urdf(롤 브라켓 바깥쪽 질량)와 XM430 정지 토크에서 가져온다.
브라켓 좌표: rm-x52 원본 기준 x = 팔 방향, y = 위(롤 0°), z = 옆. 롤 축 (y 210.5, z 0).
"""
import math, os, xml.etree.ElementTree as ET
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, '..', 'beebot.urdf')
g = 9.81
ARM_DX = 115.045          # rm-x52 원본 x -> 조립 좌표 x (mm)
X_BACK = 212.0 + ARM_DX   # 뒷판 뒷면 (조립 x, mm)
X_WEB = 216.5 + ARM_DX    # 벽 중간
STALL = 4.1               # XM430 정지 토크 N·m (롤 모터가 막혔을 때 낼 수 있는 최대)
DYN = 2.0                 # 중력 하중 동적 계수 (가감속·충격 여유)
PUCK_G = [0, 100, 200]
GRIP_X = 435.0            # 집는 점 조립 x (mm)

# 재료 (FDM 출력 PLA는 층 방향에 따라 약함 → 설계 허용 응력을 낮게 잡는다)
MATS = {'PLA (FDM)': dict(E=3.5e3, allow=15.0), 'PETG (FDM)': dict(E=2.0e3, allow=12.0), 'Al 6061-T6': dict(E=69e3, allow=120.0)}   # MPa

# ---- 하중: 브라켓 바깥(롤 회전체) 질량과 무게중심 ----
root = ET.parse(URDF).getroot()
def origin(e):
    o = e.find('origin'); xyz = np.array([float(v) for v in o.get('xyz').split()]) if o is not None else np.zeros(3)
    r, p, y = [float(v) for v in o.get('rpy', '0 0 0').split()] if o is not None else (0, 0, 0)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    R = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = xyz; return T
links = {l.get('name'): l for l in root.findall('link')}
parent = {j.find('child').get('link'): j for j in root.findall('joint')}
T = {}
def fk(l):
    if l not in T:
        T[l] = np.diag([1, 1, 1, 1.0]) if l not in parent else fk(parent[l].find('parent').get('link')) @ origin(parent[l])
        if l not in parent: T[l][:3, 3] = [0.064, 0, -0.00975]
    return T[l]
DIST = ['gripper_link', 'finger_left_link', 'finger_right_link', 'gripper_cam_link']
m_tot, mx = 0.0, 0.0
for n in DIST:
    i = links[n].find('inertial'); m = float(i.find('mass').get('value'))
    c = (fk(n) @ origin(i))[:3, 3] * 1000
    m_tot += m; mx += m * c[0]
m_br = 15.1e-3                                       # 브라켓 자신은 하중에서 뺀다 (v24 모델 15.1 g)
x_com = (mx - m_br * (X_BACK + 5)) / (m_tot - m_br)
m_tot -= m_br
print(f'브라켓이 드는 질량 {m_tot*1000:.0f} g, 무게중심 x {x_com:.1f} mm (뒷판에서 {x_com - X_BACK:.1f} mm)')

def loads(puck_g):
    W = (m_tot + puck_g / 1000) * g
    a_back = ((m_tot * (x_com - X_BACK)) + puck_g / 1000 * (GRIP_X - X_BACK)) / (m_tot + puck_g / 1000) / 1000   # m
    a_web = a_back - (X_WEB - X_BACK) / 1000
    return W * DYN, W * DYN * a_back, W * DYN * a_web     # N, N·m (뒷판), N·m (벽)

# ---- 단면 ----
t_w, h_w, d_w = 2.78, 29.6, (11.52 + 14.3) / 2      # 벽 두께(z), 높이(y), 롤 축에서 벽 중심까지(z)
I_wz = 2 * t_w * h_w ** 3 / 12                       # 롤 0°: 중력이 -y → z축 굽힘
I_wy = 2 * (h_w * t_w ** 3 / 12 + h_w * t_w * d_w ** 2)   # 롤 90°: y축 굽힘
t_b = float(os.environ.get('T_B', 4.0))             # 뒷판 두께 (rm-x52 v24 = 4 mm, 처음 설계 2 mm)
r_bolt = 8.0                                         # 혼 볼트 원 반지름
# 뒷판 굽힘 모델: 위쪽 혼 볼트(y 218.5, z 0)는 두 벽 사이(안쪽 간격 23.04 mm)에 걸린 양단 고정 보, 폭 b_eff
span = 2 * 11.52; b_eff = 8.0
I_b = b_eff * t_b ** 3 / 12
# 아래 혼 볼트(y 202.5)는 롤 축 아래 혀 끝 → 벽 시작(y 208.5)까지 6 mm 외팔보
L_tab, b_tab = 6.0, float(os.environ.get('B_TAB', 10.0))   # 혀 폭 10 mm (rm-x52 v24: 조립용으로 좁은 혀 유지, 전체 폭이면 B_TAB=28.6)
I_tab = b_tab * t_b ** 3 / 12

print(f'\n벽 단면 I_z {I_wz:.0f} mm4 (롤 0°), I_y {I_wy:.0f} mm4 (롤 90°) / 뒷판 두께 {t_b} mm')
for puck in PUCK_G:
    V, M_back, M_web = loads(puck)
    print(f'\n== 퍽 {puck} g (중력 x{DYN} 동적 계수) : 전단 {V:.2f} N, 모멘트 뒷판 {M_back:.3f} / 벽 {M_web:.3f} N·m ==')
    s_wall0 = M_web * 1e3 * (h_w / 2) / I_wz                       # MPa
    s_wall90 = M_web * 1e3 * (d_w + t_w / 2) / I_wy
    F_cpl = M_back * 1e3 / (2 * r_bolt)                             # 혼 볼트 짝힘 (N)
    s_plate = (F_cpl * span / 8) / (b_eff * t_b ** 2 / 6)           # 양단 고정 보 중앙 하중 최대 모멘트 PL/8
    s_tab = (F_cpl * L_tab) / (b_tab * t_b ** 2 / 6)                # 외팔보
    print(f'  벽 굽힘 응력: 롤 0° {s_wall0:.2f} MPa, 롤 90° {s_wall90:.2f} MPa')
    print(f'  뒷판: 혼 볼트 짝힘 {F_cpl:.1f} N → 두 벽 사이 판 {s_plate:.1f} MPa, 아래 혀 {s_tab:.1f} MPa')
    for name, mt in MATS.items():
        worst = max(s_wall0, s_wall90, s_plate, s_tab)
        d_plate = F_cpl * span ** 3 / (192 * mt['E'] * I_b)          # mm
        d_tab = F_cpl * L_tab ** 3 / (3 * mt['E'] * I_tab)
        theta = (d_plate + d_tab) / (2 * r_bolt)                      # 뒷판이 휘어 그리퍼가 숙는 각 (rad)
        tip = theta * (GRIP_X - X_BACK)
        print(f'    {name:11s}: 최대 {worst:5.1f} MPa / 허용 {mt["allow"]:5.1f} → 안전율 {mt["allow"]/worst:4.1f} | 그리퍼 처짐각 {math.degrees(theta):.2f}°, 집는 점 {tip:.2f} mm')

# ---- 롤 토크 (모터가 막혔을 때 최대 4.1 N·m) ----
print(f'\n== 롤 토크 {STALL} N·m (모터 정지 토크, 손가락이 걸려 롤이 막힌 최악) ==')
F_bolt = STALL / (4 * r_bolt / 1000)
print(f'  혼 M2 볼트 1개 전단 {F_bolt:.0f} N → 볼트 전단 응력 {F_bolt/2.07:.0f} MPa (강 8.8급 허용 ~380), 판 구멍 지압 {F_bolt/(2*t_b):.0f} MPa')
F_wall = STALL * 1e3 / (2 * d_w)
print(f'  두 벽 짝힘 {F_wall:.0f} N → 벽 면내 전단 {1.5*F_wall/(t_w*h_w):.1f} MPa (작음, 면내 방향이라 강함)')
