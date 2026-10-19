import base64
import json
import unittest
from unittest import mock

import server
from backends import ollama


GOOD = [
    {"step": 1, "skill": "navigate", "target": "3", "relation": "front", "offset_m": 0.2},
    {"step": 2, "skill": "pick", "target": "3", "relation": "", "offset_m": 0.0},
]
OBJS = [{"id": 3, "class_name": "puck"}, {"id": 7, "class_name": "goal_zone"}]


class TestValidate(unittest.TestCase):
    def test_ok(self):
        self.assertEqual(server.validate(GOOD, OBJS), "")

    def test_empty(self):
        self.assertEqual(server.validate([], OBJS), "empty")

    def test_bad_order(self):
        bad = [dict(GOOD[0], step=2), dict(GOOD[1], step=1)]
        self.assertIn("순서", server.validate(bad, OBJS))

    def test_bad_skill(self):
        self.assertIn("skill", server.validate([dict(GOOD[0], skill="dance")], OBJS))

    def test_unknown_id(self):
        self.assertIn("not tracked", server.validate([dict(GOOD[0], target="9")], OBJS))

    def test_say_text_target_ok(self):
        say = [{"step": 1, "skill": "say", "target": "퍽을 옮겼어요", "relation": "", "offset_m": 0.0}]
        self.assertEqual(server.validate(say, OBJS), "")


class TestCheckImages(unittest.TestCase):
    def test_ok(self):
        self.assertEqual(server.check_images([{"data_b64": base64.b64encode(b"x" * 100).decode()}]), "")

    def test_bad_base64(self):
        self.assertEqual(server.check_images([{"data_b64": "!!!"}]), "이미지 base64 오류")

    def test_oversize(self):
        big = base64.b64encode(b"x" * 600_000).decode()
        self.assertIn("500KB", server.check_images([{"data_b64": big}]))

    def test_empty_list_ok(self):
        self.assertEqual(server.check_images([]), "")


class TestLogPlan(unittest.TestCase):
    def test_per_request_dir(self):
        import base64
        import json
        import os
        import tempfile
        from pathlib import Path

        tmp = Path(tempfile.mkdtemp())
        data = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
        with open(os.path.join(data, "pole_cam.jpg"), "rb") as f:
            img = base64.b64encode(f.read()).decode()
        req = {"command": "빨간 퍽", "objects": [{"id": 3}],
               "images": [{"data_b64": img}, {"data_b64": img}]}
        with mock.patch.object(server, "HERE", tmp):
            server.log_plan(req, 200, {"ok": True, "goals": []}, raw="날출력")
        dirs = list((tmp / "log").iterdir())
        self.assertEqual(len(dirs), 1)
        d = dirs[0]
        self.assertEqual(
            json.loads((d / "request.json").read_text())["images"],
            [{"file": "request-image1.png", "bytes": len((d / "request-image1.png").read_bytes())},
             {"file": "request-image2.png", "bytes": len((d / "request-image2.png").read_bytes())}])
        self.assertEqual(json.loads((d / "response.json").read_text())["code"], 200)
        ai = json.loads((d / "ai.json").read_text())
        self.assertEqual(ai["backend"], "template")
        self.assertIn("명령: 빨간 퍽", ai["request"]["prompt"])
        self.assertEqual(ai["response"]["raw"], "날출력")
        self.assertIn("time", ai["request"])
        self.assertIn("time", ai["response"])
        self.assertIsInstance(ai["response"]["elapsed_s"], float)
        for i in (1, 2):
            self.assertTrue((d / f"request-image{i}.png").read_bytes().startswith(b"\x89PNG"))

    def test_no_images_no_files(self):
        import json
        import tempfile
        from pathlib import Path

        tmp = Path(tempfile.mkdtemp())
        with mock.patch.object(server, "HERE", tmp):
            server.log_plan({"command": "x"}, 400, {"ok": False})
        (d,) = list((tmp / "log").iterdir())
        ai = json.loads((d / "ai.json").read_text())
        self.assertEqual(json.loads((d / "request.json").read_text())["images"], [])
        self.assertEqual(ai["response"]["raw"], "")
        self.assertFalse(list(d.glob("*.png")))


class TestDispatch(unittest.TestCase):
    def test_template(self):
        with mock.patch.dict(server.CONFIG, {"ai": {"backend": "template"}}):
            goals, err, raw = server.dispatch({"objects": OBJS})
            self.assertIsNone(err)
            self.assertEqual(goals[0]["skill"], "navigate")

    def test_unknown_backend_falls_back(self):
        with mock.patch.dict(server.CONFIG, {"ai": {"backend": "뭐"}}):
            goals, err, raw = server.dispatch({"objects": OBJS})
            self.assertIsNone(err)
            self.assertTrue(goals)

    def test_ollama_reads_ai_section(self):
        seen = {}

        class FakeRes:
            def __enter__(self):
                return self
            def __exit__(self, *a):
                return False
            def read(self):
                return json.dumps({"message": {"content": "[]"}}).encode()

        def fake(urlopen_req, timeout=None):
            seen["url"] = urlopen_req.full_url
            seen["body"] = json.loads(urlopen_req.data)
            return FakeRes()

        with mock.patch.dict(server.CONFIG,
                              {"ai": {"backend": "ollama", "model": "qwen3", "timeout_s": 30}}), \
            mock.patch("backends.ollama.urllib.request.urlopen", fake):
            server.dispatch({"objects": []})
        self.assertEqual(seen["url"], "http://localhost:11434/api/chat")
        self.assertEqual(seen["body"]["model"], "qwen3")
        self.assertEqual(seen["body"]["options"], {"num_ctx": ollama.NUM_CTX})


if __name__ == "__main__":
    unittest.main()


class TestSystemText(unittest.TestCase):
    """시스템 프롬프트 = system_file 내용 + system (덧붙임)"""

    def test_file_and_extra(self):
        text = server.system_text({"system_file": "system_prompt.md", "system": "추가 지시"})
        self.assertIn("작업 계획자", text)
        self.assertTrue(text.endswith("추가 지시"))

    def test_missing_file_falls_back(self):
        self.assertEqual(server.system_text({"system_file": "없는파일.md", "system": "only"}), "only")
        self.assertEqual(server.system_text({}), "")
