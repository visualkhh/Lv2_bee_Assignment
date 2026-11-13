"""Field-test bag presets — no ROS runtime is required."""
import runpy
import re
from pathlib import Path


TOOLS = Path(__file__).resolve().parents[2]
field_test = runpy.run_path(str(TOOLS / "field-test.py"))
bag_regex = field_test["bag_regex"]
as_int = field_test["_as_int"]


def test_camera_topics_are_opt_in_for_perception_debugging():
    default = re.compile(bag_regex())
    perception = re.compile(bag_regex(include_camera_data=True))

    assert default.fullmatch("/tracked_objects")
    assert not default.fullmatch("/pole_front_cam/color/image_raw")
    assert not default.fullmatch("/perception/pole_front_cam/yolo_detections")
    assert perception.fullmatch("/pole_front_cam/color/image_raw")
    assert perception.fullmatch("/gripper_cam/aligned_depth_to_color/image_raw")
    assert perception.fullmatch("/pole_front_cam/color/camera_info")
    assert perception.fullmatch("/perception/pole_front_cam/yolo_detections")
    assert perception.fullmatch("/perception/gripper_cam/debug_image")


def test_uint8_message_values_convert_from_bytes():
    assert as_int(b"\x02") == 2
    assert as_int(1) == 1


if __name__ == '__main__':
    test_camera_topics_are_opt_in_for_perception_debugging()
    test_uint8_message_values_convert_from_bytes()
    print('PASS test_field_test')
