import os
import unittest

import prompt

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def load_image(name):
    with open(os.path.join(DATA, name), "rb") as f:
        return f.read()


class TestBuild(unittest.TestCase):
    def setUp(self):
        self.req = {"command": "빨간 퍽",
                    "objects": [{"id": 3, "class_name": "puck"}]}

    def test_no_history_unchanged(self):
        p = prompt.build(self.req)
        self.assertIn("명령: 빨간 퍽", p)
        self.assertIn("id=3 class=puck", p)
        self.assertNotIn("이전 대화", p)
        self.assertNotIn("시스템:", p)

    def test_system_prepended(self):
        p = prompt.build(self.req, "미션 설명")
        self.assertTrue(p.startswith("시스템: 미션 설명\n"))
        self.assertIn("명령: 빨간 퍽", p)

    def test_history_rendered(self):
        p = prompt.build(dict(self.req, history=[
            {"role": "user", "text": "파란 퍽은 놔둬"},
            {"role": "plan", "text": "navigate 3"}]))
        self.assertIn("  - user: 파란 퍽은 놔둬", p)
        self.assertIn("  - plan: navigate 3", p)

    def test_objects_with_pose(self):
        objs = [{"id": 3, "class_name": "puck", "position": [3.45, 0.9, 0.02],
                 "size": [0.1, 0.1, 0.05], "confidence": 0.92},
                {"id": 7, "class_name": "location"}]
        p = prompt.build({"command": "x", "objects": objs})
        self.assertIn("좌표=map 기준 m", p)
        self.assertIn("  - id=3 class=puck pos=(3.45,0.90,0.02) size=(0.10,0.10,0.05) conf=0.92", p)
        self.assertIn("  - id=7 class=location", p)  # 좌표 없어도 됨

    def test_history_caps(self):
        hist = [{"role": "user", "text": "x" * 600} for _ in range(25)]
        lines = [l for l in prompt.build(dict(self.req, history=hist)).splitlines()
                 if l.startswith("  - user: ")]
        self.assertEqual(len(lines), 20)  # 최근 20개
        self.assertEqual(len(lines[0]), len("  - user: ") + 500)  # 개당 500자

    def test_history_bad_shapes_ignored(self):
        for bad in ("문자열", [{"role": "user"}], [{"text": 1}]):
            p = prompt.build(dict(self.req, history=bad))
            self.assertIsInstance(p, str)

    def test_images_in_prompt(self):
        import base64
        # 실물: /pole_cam·/gripper_cam color/image_raw 토픽 저장본 그대로
        raw = [load_image("pole_cam.jpg"), load_image("gripper_cam.jpg")]
        for b in raw:
            self.assertTrue(b.startswith(b"\xff\xd8\xff"))  # JPEG
        req = dict(self.req,
                   images=[{"data_b64": base64.b64encode(b).decode()} for b in raw],
                   history=[{"role": "user", "text": "파란 퍽은 놔둬"}])
        p = prompt.build(req)
        self.assertIn("첨부 이미지: 2장 — 카메라별 원본·마킹(번호=id) 순으로 전달", p)
        self.assertIn("  - user: 파란 퍽은 놔둬", p)
        self.assertIn("JSON 배열로만 답하라", p)  # 계획 지시 유지
        for b in raw:  # base64 원문은 프롬프트에 안 박음 (용량) — 백엔드가 따로 첨부
            self.assertNotIn(base64.b64encode(b).decode()[:64], p)


if __name__ == "__main__":
    unittest.main()


class TestRobotAndColor(unittest.TestCase):
    """컨테이너 vlm_task_planner가 보내는 형식 그대로 — robot · color · px{카메라: [u, v]}"""

    def test_robot_distance_color(self):
        req = {"command": "앞에 있는 gray 물체를 집어", "robot": {"x": 0.24, "y": 0.07, "yaw": 0.0},
               "objects": [{"id": 13, "class_name": "puck", "position": [0.62, -0.117, 0.027], "size": [0.06, 0.07, 0.04],
                            "confidence": 0.83, "color": "black", "camera": "pole_cam", "px": {"pole_cam": [485, 420]}}]}
        text = prompt.build(req)
        self.assertIn("로봇 위치(map): (0.24,0.07)", text)
        self.assertIn("dist=0.42m", text)
        self.assertIn("color=black", text)
        self.assertIn("polepx=(485,420)", text)
        self.assertIn("navigate <같은 id>", text)

    def test_no_robot_no_dist(self):
        text = prompt.build({"command": "x", "objects": [{"id": 1, "class_name": "puck", "position": [1, 2, 0]}]})
        self.assertNotIn("dist=", text)
        self.assertNotIn("로봇 위치", text)


class TestMarkers(unittest.TestCase):
    """신호등 암호 미션 표식 — 퍽 tag · 장소 location tag · 바코드 content가 프롬프트에 실림, inspect 허용"""

    def test_tag_content_inspect(self):
        req = {"command": "P2-01", "objects": [
            {"id": 3, "class_name": "puck", "position": [0.5, 0.0, 0.03], "color": "red", "tag": "2"},
            {"id": 9, "class_name": "location", "position": [1.5, 0.0, 0.0], "tag": "13"},
            {"id": 10, "class_name": "barcode", "position": [1.5, 0.1, 0.05], "content": "P2-01"}]}
        text = prompt.build(req)
        self.assertIn("class=puck", text)
        self.assertIn("tag=2", text)
        self.assertIn("class=location", text)
        self.assertIn("tag=13", text)
        self.assertIn("content=P2-01", text)
        self.assertIn("inspect", text)
        self.assertNotIn("MOVE_TO_B", text)
