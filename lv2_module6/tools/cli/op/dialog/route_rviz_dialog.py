"""RouteRvizDialog — A/B 경로를 RViz 2D Goal Pose로 찍는 창 (Dialog 하위 클래스). pages/ab 경로 편집이 띄운다."""
from __future__ import annotations

import math

from .dialog import CONTINUE, Dialog

RVIZ_GOAL_TOPIC = '/ab/goal_pose'  # tools/rviz/ab.rviz의 2D Goal Pose — Nav2(/goal_pose)가 안 듣는 토픽이라 찍어도 로봇은 안 움직임


def fmt_pose(p: dict | None) -> str:
    return '지금 위치 (실행할 때)' if p is None else f'({p["x"]:+.2f}, {p["y"]:+.2f}) {p["yaw_deg"]:+.0f}°'


class RouteRvizDialog(Dialog):
    """RViz 2D Goal Pose(/ab/goal_pose)로 찍은 goal을 모으는 창 — Enter 저장 · Backspace 마지막 취소 · Esc 취소.
    run() → goal 목록(저장) 또는 None(취소)."""

    def __init__(self, ui, bus, start: dict | None):
        super().__init__(ui, 'A/B 경로 — RViz로 goal 찍기')
        from geometry_msgs.msg import PoseStamped
        self.bus, self.start = bus, start
        self.goals: list[dict] = []
        self._inbox: list = []
        self._sub = bus.node.create_subscription(PoseStamped, RVIZ_GOAL_TOPIC, self._inbox.append, 10)

    def content(self):
        body = ['RViz를 RVIZ_CONFIG=ab ./run-rviz 로 열고 2D Goal Pose로 goal을 순서대로 찍으세요.',
                '(이 설정의 2D Goal Pose는 Nav2가 안 듣는 토픽 — 찍어도 로봇은 안 움직임)', '',
                f'출발점: {fmt_pose(self.start)}']
        body += [f'  G{i}: {fmt_pose(g)}' for i, g in enumerate(self.goals, 1)] or ['  (아직 없음 — RViz에서 찍으세요)']
        return [self.title, ''] + body + ['', 'Enter 저장 · Backspace 마지막 goal 취소 · Esc 취소']

    def on_tick(self):
        self.bus._poll(0.05)
        while self._inbox:
            m = self._inbox.pop(0)
            if m.header.frame_id not in ('', 'map'):
                continue  # map 기준이 아닌 goal은 버림 (RViz Fixed Frame = map)
            q = m.pose.orientation
            yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            self.goals.append({'x': round(m.pose.position.x, 3), 'y': round(m.pose.position.y, 3),
                               'yaw_deg': round(yaw, 1)})
        self.ui.tick(0.0)

    def on_key(self, k):
        if k == 'enter' and self.goals:
            return list(self.goals)
        if k == 'backspace' and self.goals:
            self.goals.pop()
        elif k in ('esc', 'left', 'eof'):
            return None
        return CONTINUE

    def run(self):
        try:
            return super().run()
        finally:
            self.bus.node.destroy_subscription(self._sub)
