import base64
import io
import os
import unittest

import images

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def load(name):
    with open(os.path.join(DATA, name), "rb") as f:
        return f.read()


def b64(b):
    return base64.b64encode(b).decode()


class TestPrepare(unittest.TestCase):
    def test_raw_only_without_px(self):
        req = {"objects": [{"id": 3, "class_name": "puck"}],
               "images": [{"data_b64": b64(load("pole_cam.jpg")), "camera": "pole_cam"}]}
        out = images.prepare(req)
        self.assertEqual([(o["camera"], o["kind"]) for o in out], [("pole_cam", "raw")])

    def test_marked_added_with_px(self):
        req = {"objects": [{"id": 3, "class_name": "puck",
                             "px": {"pole_cam": [160, 120]}}],
               "images": [{"data_b64": b64(load("pole_cam.jpg")), "camera": "pole_cam"},
                          {"data_b64": b64(load("gripper_cam.jpg")), "camera": "gripper_cam"}]}
        out = images.prepare(req)
        self.assertEqual([(o["camera"], o["kind"]) for o in out],
                         [("pole_cam", "raw"), ("pole_cam", "marked"),
                          ("gripper_cam", "raw")])  # gripper px 없음 → 원본만
        raw = base64.b64decode(out[0]["data_b64"])
        marked = base64.b64decode(out[1]["data_b64"])
        self.assertNotEqual(raw, marked)  # 마커가 그려짐
        for o in out:
            self.assertTrue(base64.b64decode(o["data_b64"]).startswith(b"\xff\xd8\xff"))

    def test_rgb8_conversion(self):
        from PIL import Image

        buf = io.BytesIO()
        Image.open(os.path.join(DATA, "pole_cam.jpg")).resize((32, 24)).save(buf, "JPEG")
        raw = Image.open(io.BytesIO(buf.getvalue())).convert("RGB").tobytes()
        req = {"objects": [],
               "images": [{"data_b64": b64(raw), "camera": "pole_cam",
                           "format": "rgb8", "width": 32, "height": 24}]}
        out = images.prepare(req)
        self.assertEqual(len(out), 1)
        self.assertTrue(base64.b64decode(out[0]["data_b64"]).startswith(b"\xff\xd8\xff"))

    def test_bad_image_skipped(self):
        req = {"objects": [],
               "images": [{"data_b64": "!!!", "camera": "pole_cam"}]}
        self.assertEqual(images.prepare(req), [])

    def test_out_of_frame_px_skipped(self):
        req = {"objects": [{"id": 3, "px": {"pole_cam": [99999, 99999]}}],
               "images": [{"data_b64": b64(load("pole_cam.jpg")), "camera": "pole_cam"}]}
        out = images.prepare(req)
        self.assertEqual([(o["camera"], o["kind"]) for o in out], [("pole_cam", "raw")])


if __name__ == "__main__":
    unittest.main()
