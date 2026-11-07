"""Arm kinematics + gravity torque from the URDF (numpy only): FK, hand-down IK, joint gravity torques, and the
tip displacement a unit joint compliance would cause. Used by sag_measure.py (pose grid) and sag_fit.py (stiffness fit).

Model: joint i twists by d_i = tau_i / k_i under the gravity torque tau_i of everything beyond it, so the hand
moves by sum_i (a_i x (p_ee - p_i)) * d_i. Frames are base_footprint (z = floor) like the TF the coordinator uses.
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

URDF = Path(__file__).resolve().parents[3] / 'urdf' / 'beebot_mesh' / 'beebot.urdf'
ARM = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
G = 9.81
TIP = 0.0206  # [m] end_effector_link to the finger-tip face centre along the hand axis (plan/stand_plan.hpp FINGER_TIP)


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = _rpy(*rpy)
    T[:3, 3] = xyz
    return T


def _rot(axis, q):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K
    return T


def _vec(s):
    return np.array([float(x) for x in s.split()]) if s else np.zeros(3)


class Arm:
    def __init__(self, urdf=URDF):
        root = ET.parse(urdf).getroot()
        self.joints = {}
        for j in root.findall('joint'):
            o, a, lim = j.find('origin'), j.find('axis'), j.find('limit')
            self.joints[j.get('name')] = dict(
                type=j.get('type'), parent=j.find('parent').get('link'), child=j.find('child').get('link'),
                T=_T(_vec(o.get('xyz')) if o is not None else np.zeros(3), _vec(o.get('rpy')) if o is not None else np.zeros(3)),
                axis=_vec(a.get('xyz')) if a is not None else None,
                lim=(float(lim.get('lower')), float(lim.get('upper'))) if lim is not None and lim.get('lower') else None)
        self.mass, self.com = {}, {}
        for l in root.findall('link'):
            i = l.find('inertial')
            if i is not None:
                self.mass[l.get('name')] = float(i.find('mass').get('value'))
                self.com[l.get('name')] = _vec(i.find('origin').get('xyz')) if i.find('origin') is not None else np.zeros(3)
        self.by_parent = {}
        for n, j in self.joints.items():
            self.by_parent.setdefault(j['parent'], []).append(n)

    def fk_all(self, q, grip=0.0):
        """link frames in base_footprint. q = joint1..5 [rad], grip = gripper_left_joint [m] (right finger mirrors it)"""
        qs = dict(zip(ARM, q), gripper_left_joint=grip, gripper_right_joint=grip)
        frames, axes_w = {'base_footprint': np.eye(4)}, {}
        stack = ['base_footprint']
        while stack:
            p = stack.pop()
            for jn in self.by_parent.get(p, []):
                j = self.joints[jn]
                T = frames[p] @ j['T']
                if j['type'] == 'revolute' and jn in qs:
                    axes_w[jn] = (T[:3, :3] @ j['axis'] / np.linalg.norm(j['axis']), T[:3, 3].copy())
                    T = T @ _rot(j['axis'], qs[jn])
                elif j['type'] == 'prismatic' and jn in qs:
                    d = np.eye(4)
                    d[:3, 3] = j['axis'] * qs[jn]
                    T = T @ d
                frames[j['child']] = T
                stack.append(j['child'])
        return frames, axes_w

    def ee(self, q, grip=0.0):
        T = self.fk_all(q, grip)[0]['end_effector_link']
        return T[:3, 3], T[:3, 2]  # position, tool z axis (points down when the hand is down)

    def mark(self, q, grip=0.0):
        """finger-tip face centre (the point you put the tape on): end_effector_link + TIP along the hand axis [m]"""
        T = self.fk_all(q, grip)[0]['end_effector_link']
        return T[:3, 3] + TIP * T[:3, 2]

    def subtree(self, link):
        out, stack = [], [link]
        while stack:
            l = stack.pop()
            out.append(l)
            stack += [self.joints[j]['child'] for j in self.by_parent.get(l, [])]
        return out

    def gravity_and_lever(self, q, grip=0.0):
        """per arm joint i: gravity torque tau_i [Nm] about its axis, and dz_i = z of (a_i x (p_ee - p_i)) [m] (tip z per rad)"""
        frames, axes = self.fk_all(q, grip)
        pe = frames['end_effector_link'][:3, 3] + TIP * frames['end_effector_link'][:3, 2]  # the tape mark, see mark()
        tau, dz = [], []
        for jn in ARM:
            a, p = axes[jn]
            t = 0.0
            for l in self.subtree(self.joints[jn]['child']):
                if l in self.mass:
                    r = frames[l][:3, :3] @ self.com[l] + frames[l][:3, 3]
                    t += a @ np.cross(r - p, [0.0, 0.0, -self.mass[l] * G])
            tau.append(t)
            dz.append(np.cross(a, pe - p)[2])
        return np.array(tau), np.array(dz)

    def hand_ik(self, x, z, pitch_deg=0.0):
        """q2, q3, q4 [rad] with the face centre (mark) at (x, 0, z) and the hand pitched pitch_deg from straight down
        (q2 + q3 + q4 = 90 deg + pitch), joint1 = joint5 = 0, inside the URDF limits; None if there is no solution"""
        from scipy.optimize import least_squares

        def res(v):
            m = self.mark([0, v[0], v[1], v[2], 0])
            return [m[0] - x, m[2] - z, 2 * (v[0] + v[1] + v[2] - np.pi / 2 - np.radians(pitch_deg))]
        lims = [self.joints[n]['lim'] or (-np.pi, np.pi) for n in ('joint2', 'joint3', 'joint4')]
        best = None
        for s in ([0.5, -0.2, 1.2], [-0.4, 0.8, 1.15], [0.2, 0.2, 1.2], [-0.2, 0.4, 1.0], [0.3, 0.5, 0.8]):
            r = least_squares(res, s, bounds=([l[0] for l in lims], [l[1] for l in lims]))
            if np.max(np.abs(r.fun)) < 1e-4 and (best is None or r.cost < best.cost): best = r
        return None if best is None else best.x

    def hand_down_ik(self, x, z, seeds=None):
        """q2, q3, q4 [rad] for the hand pointing straight down at (x, 0, z) with joint1 = joint5 = 0, inside the URDF limits"""
        from scipy.optimize import least_squares

        def res(v):
            p, ax = self.ee([0, v[0], v[1], v[2], 0])
            return [p[0] - x, p[2] - z, 3 * (ax[2] + 1.0)]
        lims = [self.joints[n]['lim'] or (-np.pi, np.pi) for n in ('joint2', 'joint3', 'joint4')]
        best = None
        for s in (seeds or [[0.5, -0.2, 1.2], [-0.4, 0.8, 1.15], [0.2, 0.2, 1.2], [-0.2, 0.4, 1.0]]):
            r = least_squares(res, s, bounds=([l[0] for l in lims], [l[1] for l in lims]))
            if np.max(np.abs(r.fun)) < 1e-4 and (best is None or r.cost < best.cost):
                best = r
        return None if best is None else best.x


if __name__ == '__main__':  # check against the stand-run TF numbers (see sag_measure POSES)
    arm = Arm()
    for name, q in {'above': [0.49, 26.09, -23.96, 88.47, 0.34], 'grasp': [0.06, 29.78, -11.31, 71.52, 0.06],
                    'near_mid': [0.15, -22.68, 46.72, 65.96, 0.15], 'near_far': [0.11, -11.19, 35.15, 66.05, 0.12]}.items():
        p, ax = arm.ee(np.radians(q))
        tau, dz = arm.gravity_and_lever(np.radians(q))
        print(name, 'ee', np.round(p * 1000, 1), 'tool z', round(ax[2], 2), 'tau', np.round(tau, 3), 'dz', np.round(dz, 3))
    print('total hand-side mass [kg]:', sum(arm.mass.get(l, 0) for l in arm.subtree('gripper_link')))
