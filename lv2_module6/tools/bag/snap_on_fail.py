#!/usr/bin/env python3
"""실패 순간 블랙박스 저장 — record.sh blackbox와 같이 켜 둠.

/task/status FAILED · /arm_safety_event · /state/emergency True · /mission/state 바뀜이 아니라 '실패'만 보고
/bee_blackbox/snapshot을 부름 (같은 실패가 몰려 오면 cooldown_s 안에는 한 번). 무엇 때문에 저장했는지는
~/bee_bags/snapshots.tsv에 한 줄씩 (시각 · 이유) — bag 안 시각과 맞춰 찾음.
  python3 snap_on_fail.py [--cooldown 10] [--delay 3]   # delay: 실패 뒤 몇 초까지 더 담고 저장
"""
import argparse
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool
from rosbag2_interfaces.srv import Snapshot

try:
    from plan.msg import TaskStatus
except ImportError:  # plan 빌드 없이도 나머지는 돎
    TaskStatus = None
try:
    from control.msg import ArmSafetyEvent
except ImportError:
    ArmSafetyEvent = None


class SnapOnFail(Node):
    def __init__(self, cooldown, delay, log):
        super().__init__("bee_snap_on_fail")
        self.cooldown, self.delay, self.log = cooldown, delay, log
        self.last = -1e9
        self.pending = None  # (저장할 시각, 이유)
        self.cli = self.create_client(Snapshot, "/bee_blackbox/snapshot")
        if TaskStatus:
            self.create_subscription(TaskStatus, "/task/status", self.on_task, 10)
        if ArmSafetyEvent:
            self.create_subscription(ArmSafetyEvent, "/arm_safety_event", lambda m: self.trigger(f"arm_safety {m.joint_name} {m.current_a:.2f}A"), 10)
        self.create_subscription(Bool, "/state/emergency", lambda m: m.data and self.trigger("emergency"), 10)
        self.create_timer(0.2, self.tick)
        self.get_logger().info("실패 기다림 → /bee_blackbox/snapshot")

    def on_task(self, m):
        if m.state == TaskStatus.FAILED:
            self.trigger(f"task_failed plan {m.plan_id} step {m.step} {m.reason}")

    def trigger(self, why):
        now = time.monotonic()
        if self.pending or now - self.last < self.cooldown:
            return
        self.pending = (now + self.delay, why)
        self.get_logger().warning(f"실패: {why} — {self.delay:.0f} s 뒤 저장")

    def tick(self):
        if not self.pending or time.monotonic() < self.pending[0]:
            return
        why = self.pending[1]
        self.pending, self.last = None, time.monotonic()
        if not self.cli.service_is_ready():
            self.get_logger().error("/bee_blackbox/snapshot 없음 — record.sh blackbox 켜졌나?")
            return
        self.cli.call_async(Snapshot.Request())
        with open(self.log, "a") as f:
            f.write(f"{time.strftime('%m-%d %H:%M:%S')}\t{why}\n")
        self.get_logger().info(f"저장함: {why}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cooldown", type=float, default=10.0)
    ap.add_argument("--delay", type=float, default=3.0)
    a = ap.parse_args()
    log = os.path.join(os.environ.get("BAG_DIR", os.path.expanduser("~/bee_bags")), "snapshots.tsv")
    os.makedirs(os.path.dirname(log), exist_ok=True)
    rclpy.init()
    n = SnapOnFail(a.cooldown, a.delay, log)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
