#!/usr/bin/env python3
"""skill 시험 — coordinator에 skill을 하나씩(또는 묶어서) 보내고 결과·로봇 위치·주변 퍽을 기록한다 (mission_manager 없이)

  ros2 run plan skill_test go start_1            # 시작 박스 1 앞: navigate(standoff) → approach(approach_gap)
  ros2 run plan skill_test inspect start_1 -0.04  # 그 박스 위 퍽 자리(박스 중심 X−40mm) 내려다보기 → 근처 퍽 tag·색·모양
  ros2 run plan skill_test pick start_1 -0.04     # 그 자리 퍽(트랙 id)을 pick
  ros2 run plan skill_test place L1 4             # L1 칸 4에 place (z = 박스 윗면)
  ros2 run plan skill_test back                   # 마지막 go의 standoff로 똑바로 물러남
  ros2 run plan skill_test pose scan              # 팔 자세
  ros2 run plan skill_test raw "navigate:0.5,0,0" "approach:0.6,0,0"   # SubGoal 직접
  ros2 run plan skill_test cycle                  # start_1 퍽 하나를 L1 칸 4로: go → inspect → pick → back → go → place → inspect → back

좌표·박스는 plan.yaml의 mission_manager 값 (arena m) + start_pose_arena 파라미터(--start x,y,yaw, 기본 시뮬레이터 0.734,0.950,0)
박스 변은 로봇에서 가장 가까운 변 (--side W|E|S|N로 지정). 상태(마지막 standoff)는 /tmp/skill_test_state.json
"""
import argparse
import json
import math
import os
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from perception.msg import TrackedObjectArray
from plan.msg import SubGoal, SubGoalArray, TaskStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

STATE = Path("/tmp/skill_test_state.json")
LATCHED = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def mission_params():
    p = Path(get_package_share_directory("plan")) / "config" / "plan.yaml"
    m = yaml.safe_load(p.read_text())["mission_manager"]["ros__parameters"]
    c = yaml.safe_load(p.read_text())["coordinator"]["ros__parameters"]
    boxes = {}
    sb = m["start_boxes"]
    for i in range(0, len(sb), 2):
        boxes[f"start_{i // 2 + 1}"] = (sb[i], sb[i + 1], m["box_half"])
    lb, lo = m["location_boxes"], m["locations"]
    for k in range(3):
        boxes[f"L{k + 1}"] = (lb[3 * k], lb[3 * k + 1], lb[3 * k + 2])
        boxes[f"L{k + 1}_center"] = (lo[2 * k], lo[2 * k + 1])
    return m, c, boxes


class Tester(Node):
    def __init__(self, start):
        super().__init__("skill_test")
        self.start = start
        self.pub = self.create_publisher(SubGoalArray, "/task/sub_goals", LATCHED)
        self.status = {}
        self.create_subscription(TaskStatus, "/task/status", self.on_status, LATCHED)
        self.objects = None
        self.create_subscription(TrackedObjectArray, "/tracked_objects", lambda m: setattr(self, "objects", m), 5)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.plan_id = 2000000 + int(time.time()) % 100000 * 10  # mission_manager(1000000번대)와 안 겹치게

    def on_status(self, s):
        self.status[s.plan_id] = s

    # arena ↔ map (mission_solver arena_to_map과 같은 식)
    def to_map(self, x, y, yaw=0.0):
        x0, y0, a0 = self.start
        dx, dy = x - x0, y - y0
        c, s = math.cos(-a0), math.sin(-a0)
        return c * dx - s * dy, s * dx + c * dy, math.remainder(yaw - a0, 2 * math.pi)

    def to_arena(self, mx, my):
        x0, y0, a0 = self.start
        return x0 + math.cos(a0) * mx - math.sin(a0) * my, y0 + math.sin(a0) * mx + math.cos(a0) * my

    def robot(self):
        for _ in range(40):
            try:
                t = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
                q = t.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                return t.transform.translation.x, t.transform.translation.y, yaw
            except Exception:  # noqa: BLE001
                rclpy.spin_once(self, timeout_sec=0.1)
        return None

    def run(self, goals, timeout=240.0):
        """[(skill, target)] → (성공, 이유, 걸린 초)"""
        self.plan_id += 1
        msg = SubGoalArray()
        msg.plan_id = self.plan_id
        msg.goals = [SubGoal(step=i + 1, skill=s, target=t) for i, (s, t) in enumerate(goals)]
        t0 = time.time()
        self.pub.publish(msg)
        print(f"  → plan {self.plan_id}: " + " | ".join(f"{s}({t})" for s, t in goals), flush=True)
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            s = self.status.get(self.plan_id)
            if s is not None and s.state != TaskStatus.RUNNING:
                ok = s.state == TaskStatus.SUCCEEDED
                print(f"  ← {'성공' if ok else '실패'} step {s.step} {s.reason} ({time.time() - t0:.1f}s)", flush=True)
                return ok, s.reason, time.time() - t0
        print(f"  ← 시간 초과 ({timeout:.0f}s)", flush=True)
        return False, "timeout", timeout

    def pucks_near(self, mx, my, r=0.06, since=None):
        rclpy.spin_once(self, timeout_sec=0.3)
        out = []
        for o in (self.objects.objects if self.objects else []):
            if not o.class_name.startswith("puck"):
                continue
            px, py = o.pose.pose.position.x, o.pose.pose.position.y
            if math.hypot(px - mx, py - my) > r:
                continue
            seen = o.last_seen.sec + o.last_seen.nanosec * 1e-9
            if since is not None and seen < since:
                continue
            d = {kv.key: kv.value for kv in o.details}
            out.append((o.id, d.get("tag", "?"), o.color, d.get("shape", "?"), math.hypot(px - mx, py - my), px, py))
        return sorted(out, key=lambda t: t[4])


def side_poses(box, m, robot_arena):
    """박스 네 변 앞 standoff 자세 (arena) — 로봇에서 가까운 순"""
    bx, by, half = box
    d = half + m["standoff"]
    poses = {"W": (bx - d, by, 0.0), "E": (bx + d, by, math.pi), "S": (bx, by - d, math.pi / 2), "N": (bx, by + d, -math.pi / 2)}
    if robot_arena is None:
        return list(poses.items())
    return sorted(poses.items(), key=lambda kv: math.hypot(kv[1][0] - robot_arena[0], kv[1][1] - robot_arena[1]))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd")
    ap.add_argument("args", nargs="*")
    ap.add_argument("--start", default="0.734,0.950,0.0", help="로봇을 켠 arena 자세 (실기는 0.734,0.950,3.14159)")
    ap.add_argument("--side", default=None, help="박스 변 W|E|S|N (기본: 로봇에서 가까운 변)")
    a = ap.parse_args()
    m, c, boxes = mission_params()
    rclpy.init()
    t = Tester(tuple(float(v) for v in a.start.split(",")))
    state = json.loads(STATE.read_text()) if STATE.exists() else {}

    def fmt(p):
        return ",".join(f"{v:.4f}" for v in p)

    def where(label):
        r = t.robot()
        if r:
            ax, ay = t.to_arena(r[0], r[1])
            print(f"  · 로봇 {label}: map ({r[0]:.3f}, {r[1]:.3f}, {math.degrees(r[2]):.1f}°) · arena ({ax:.3f}, {ay:.3f})", flush=True)
        return r

    def go(name):
        r = where("출발")
        ra = t.to_arena(r[0], r[1]) if r else None
        sides = side_poses(boxes[name], m, ra)
        if a.side:
            sides = [kv for kv in sides if kv[0] == a.side.upper()]
        if state.get("back"):  # 이전 박스에 붙어 있으면 먼저 물러남
            ok, why, _ = t.run([("approach", state["back"])])
            state.pop("back")
            if not ok:
                return False
        push = m["standoff"] - m["approach_gap"]
        for side, (x, y, yaw) in sides[:3]:
            std = t.to_map(x, y, yaw)
            fin = t.to_map(x + push * math.cos(yaw), y + push * math.sin(yaw), yaw)
            print(f"  · {name} {side} 변", flush=True)
            ok, why, _ = t.run([("navigate", fmt(std)), ("approach", fmt(fin))])
            if ok:
                state["back"] = fmt(std)
                r = where("도착")
                if r:  # 박스 변까지 실제 간격 (base_footprint ~ 박스 변, 목표 approach_gap)
                    bx, by, half = boxes[name]
                    ax, ay = t.to_arena(r[0], r[1])
                    gap = {"W": bx - half - ax, "E": ax - (bx + half), "S": by - half - ay, "N": ay - (by + half)}[side]
                    print(f"  · 박스 변까지 {gap:.4f} m (목표 {m['approach_gap']})", flush=True)
                return True
            if why.startswith("approach"):
                state["back"] = fmt(std)  # Nav2는 도착했으니 거기로 물러날 수 있음
                return False
        return False

    def puck_point(name, dx):
        bx, by, _ = boxes[name]
        return bx + dx, by

    ok = True
    if a.cmd == "go":
        ok = go(a.args[0])
    elif a.cmd == "back":
        ok = t.run([("approach", state.pop("back"))])[0] if state.get("back") else True
    elif a.cmd == "pose":
        ok = t.run([("pose", a.args[0])])[0]
    elif a.cmd == "inspect":
        x, y = puck_point(a.args[0], float(a.args[1]) if len(a.args) > 1 else 0.0)
        mx, my, _ = t.to_map(x, y)
        t0 = time.time()
        ok = t.run([("inspect", fmt((mx, my, m["box_top_z"] + m["puck_height"])))])[0]
        for p in t.pucks_near(mx, my, 0.06, since=t0 - 0.5):
            print(f"  · 퍽 트랙 {p[0]}: tag {p[1]} · {p[2]} · {p[3]} · {p[4] * 1000:.0f} mm", flush=True)
    elif a.cmd == "pick":
        x, y = puck_point(a.args[0], float(a.args[1]) if len(a.args) > 1 else 0.0)
        mx, my, _ = t.to_map(x, y)
        near = t.pucks_near(mx, my, 0.05)
        if not near:
            print("  · 그 자리에 퍽 트랙 없음 — inspect 먼저", flush=True)
            ok = False
        else:
            ok = t.run([("pick", str(near[0][0]))])[0]
    elif a.cmd == "place":
        name, cell = a.args[0], int(a.args[1])
        cx, cy = boxes[f"{name}_center"]
        dx, dy = (cell % 3 - 1) * m["grid_step"], (1 - cell // 3) * m["grid_step"]
        mx, my, _ = t.to_map(cx + dx, cy + dy)
        ok = t.run([("place", fmt((mx, my, m["box_top_z"])))])[0]
    elif a.cmd == "raw":
        ok = t.run([tuple(g.split(":", 1)) for g in a.args])[0]
    elif a.cmd == "cycle":  # 퍽 하나를 시작 박스 → 장소 칸으로 (mission BUILD 한 번과 같은 순서)
        src, dx, dst, cell = (a.args + ["start_1", "-0.04", "L1", "4"][len(a.args):])[:4]
        steps = [("go", src), ("inspect", src, dx), ("pick", src, dx), ("go", dst), ("place", dst, cell),
                 ("inspect", f"{dst}_center"), ("back",)]
        for st in steps:
            print(f"■ {' '.join(st)}", flush=True)
            if st[0] == "go":
                ok = go(st[1])
            elif st[0] == "back":
                ok = t.run([("approach", state.pop("back"))])[0] if state.get("back") else True
            elif st[0] == "inspect" and st[1].endswith("_center"):
                cx, cy = boxes[st[1]]
                mx, my, _ = t.to_map(cx, cy)
                t0 = time.time()
                ok = t.run([("inspect", fmt((mx, my, m["box_top_z"] + m["puck_height"])))])[0]
                for p in t.pucks_near(mx, my, 0.15, since=t0 - 0.5):
                    ax, ay = t.to_arena(p[5], p[6])
                    print(f"  · 퍽 트랙 {p[0]}: tag {p[1]} · arena ({ax:.3f}, {ay:.3f})", flush=True)
            elif st[0] == "inspect":
                x, y = puck_point(st[1], float(st[2]))
                mx, my, _ = t.to_map(x, y)
                t0 = time.time()
                ok = t.run([("inspect", fmt((mx, my, m["box_top_z"] + m["puck_height"])))])[0]
                for p in t.pucks_near(mx, my, 0.06, since=t0 - 0.5):
                    print(f"  · 퍽 트랙 {p[0]}: tag {p[1]} · {p[2]} · {p[3]} · {p[4] * 1000:.0f} mm", flush=True)
            elif st[0] == "pick":
                x, y = puck_point(st[1], float(st[2]))
                mx, my, _ = t.to_map(x, y)
                near = t.pucks_near(mx, my, 0.05)
                ok = bool(near) and t.run([("pick", str(near[0][0]))])[0]
            elif st[0] == "place":
                cx, cy = boxes[f"{st[1]}_center"]
                cl = int(st[2])
                mx, my, _ = t.to_map(cx + (cl % 3 - 1) * m["grid_step"], cy + (1 - cl // 3) * m["grid_step"])
                ok = t.run([("place", fmt((mx, my, m["box_top_z"])))])[0]
            if not ok:
                print(f"■ 멈춤: {' '.join(st)} 실패", flush=True)
                break
    else:
        ap.error(f"모르는 명령 {a.cmd}")
    STATE.write_text(json.dumps(state))
    print("결과:", "성공" if ok else "실패", flush=True)
    rclpy.shutdown()
    os._exit(0 if ok else 1)


if __name__ == "__main__":
    main()
