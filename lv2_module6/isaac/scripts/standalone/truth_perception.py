#!/usr/bin/env python3
"""인식 더미: YOLO 대신 Isaac 참값으로 /tracked_objects와 /perception/gripper_cam/detections를 냄 (순수 pick 시험 — pick_test.py).
  참값 = manipulator_env.py가 쓰는 /tmp/beebot_slot_pucks.json (퍽 위치·yaw·기울기, 5 s 넘게 안 갱신되면 빈 목록) + /tmp/beebot_trial.json (상자).
  서 있는 퍽은 puck, 누운(기울기 45° 넘음) 퍽은 puck_lying. 태그·모양·색은 처음부터 안다 (inspect 없이). 위치·yaw 오차는 옵션으로 (아래).
  오차 모델: 퍽마다 시행(seed)당 한 번 뽑는 편향(--bias-mm σ 양방향, --bias-x-mm 평균 x 이동 = 기둥 카메라 depth 축 편향, --yaw-bias-deg σ)
    + 프레임마다 새로 뽑는 잡음(--jitter-mm, --yaw-jitter-deg). 편향은 같은 seed면 같다 (재현). 모두 기본 0 = 참값.
  pick_load_demo.sh --truth-perception 이 YOLO perception_node를 끄고 이걸 띄운다.
"""
import argparse
import json
import math
import random
import re
import time

import rclpy
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Quaternion
from perception.msg import TrackedObject, TrackedObjectArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

SLOT_FILE, TRIAL_FILE = '/tmp/beebot_slot_pucks.json', '/tmp/beebot_trial.json'
PUCK_H, PUCK_W, BOX_TOP, BOX_SIZE = 0.06, 0.03, 0.07, (0.205, 0.205, 0.07)
BOX_X_DEFAULT = 0.30 + 0.1025   # constants.BOX_X (--random-layout 아니면 trial 파일이 없음)
NAME = re.compile(r'puck_(box|cyl)_(red|green)_(\d+)$')


def quat(yaw_rad):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw_rad / 2), w=math.cos(yaw_rad / 2))


def read(path, max_age=None):
    try:
        with open(path) as f:
            d = json.load(f)
    except (OSError, ValueError):
        return None
    return d if max_age is None or time.time() - d['t'] < max_age else None


class TruthPerception(Node):
    def __init__(self, opt):
        super().__init__('truth_perception')
        self.opt = opt
        self.bias = {}  # (seed, tag) → (dx, dy, dyaw)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(TrackedObjectArray, '/tracked_objects', qos)
        self.gripper_pub = self.create_publisher(TrackedObjectArray, '/perception/gripper_cam/detections', 5)
        self.create_timer(0.1, self.tick)

    def obj(self, oid, cls, x, y, z, yaw, size, color='', details=(), cams=('pole_front_cam',)):
        o = TrackedObject(id=oid, class_name=cls, confidence=1.0, color=color, cameras=list(cams), px=[0, 0])
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = x, y, z
        o.pose.pose.orientation = quat(yaw)
        o.pose.covariance[0] = o.pose.covariance[7] = o.pose.covariance[14] = 1e-6
        o.size.x, o.size.y, o.size.z = size
        o.last_seen = self.get_clock().now().to_msg()
        o.details = [KeyValue(key=k, value=str(v)) for k, v in details]
        return o

    def draw_bias(self, seed, tag):
        key = (seed, tag)
        if key not in self.bias:
            r, o = random.Random(f'{seed}/{tag}'), self.opt
            self.bias[key] = (r.gauss(0, o.bias_mm / 1000) + o.bias_x_mm / 1000, r.gauss(0, o.bias_mm / 1000), math.radians(r.gauss(0, o.yaw_bias_deg)))
        return self.bias[key]

    def tick(self):
        msg = TrackedObjectArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        slots = read(SLOT_FILE, 5.0)
        if slots:
            seed = (read(TRIAL_FILE) or {}).get('seed', 0)
            for name, r in slots['pucks'].items():
                m = NAME.match(name)
                if not m:
                    continue
                shape, color, tag = m.group(1), m.group(2), int(m.group(3))
                standing = r.get('up_tilt', 0.0) < 45.0
                bx, by, byaw = self.draw_bias(seed, tag)
                j = self.opt.jitter_mm / 1000
                x, y = r['xy'][0] + bx + random.gauss(0, j), r['xy'][1] + by + random.gauss(0, j)
                yaw = math.radians(r.get('yaw', 0.0)) + byaw + math.radians(random.gauss(0, self.opt.yaw_jitter_deg))
                z = r['z'] + (PUCK_H / 2 if standing else PUCK_W / 2)
                details = [('posture', 'standing' if standing else 'lying'), ('tag', tag), ('shape', shape),
                           ('distance', f'{math.hypot(x, y):.3f}')]
                if shape == 'box' and standing:
                    details.append(('yaw_period', 90))  # 원기둥·누운 퍽은 방향을 모름
                msg.objects.append(self.obj(100 + tag, 'puck' if standing else 'puck_lying', x, y, z, yaw,
                                            (PUCK_W, PUCK_W, PUCK_H), color, details, cams=('pole_front_cam', 'gripper_cam')))
            trial = read(TRIAL_FILE) or {}
            bx, by = trial.get('box', (BOX_X_DEFAULT, 0.0))
            msg.objects.append(self.obj(1, 'start_box', bx, by, BOX_TOP, math.radians(trial.get('box_yaw', 0.0)), BOX_SIZE,
                                        details=[('yaw_period', 90)]))
        self.pub.publish(msg)
        self.gripper_pub.publish(msg)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    for name, help_ in (('bias-mm', '퍽 위치 편향 σ [mm] (시행·퍽마다 한 번, x·y 각각)'), ('bias-x-mm', '모든 퍽 x에 더하는 평균 이동 [mm] (기둥 카메라 depth 축 편향)'),
                        ('jitter-mm', '프레임마다 새로 뽑는 위치 잡음 σ [mm]'), ('yaw-bias-deg', 'yaw 편향 σ [deg] (시행·퍽마다 한 번)'),
                        ('yaw-jitter-deg', '프레임마다 새로 뽑는 yaw 잡음 σ [deg]')):
        ap.add_argument('--' + name, type=float, default=0.0, help=help_)
    cli, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    rclpy.spin(TruthPerception(cli))
