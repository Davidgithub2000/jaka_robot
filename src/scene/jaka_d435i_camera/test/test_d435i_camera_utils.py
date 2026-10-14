#!/usr/bin/env python3

import os
import sys
import unittest


PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT_DIR = os.path.join(PACKAGE_DIR, "scripts")
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from d435i_camera_utils import camera_info_dict_from_intrinsics, parse_camera_config


class FakeIntrinsics:
    width = 1280
    height = 720
    fx = 910.0
    fy = 905.0
    ppx = 640.0
    ppy = 360.0
    coeffs = [0.1, -0.2, 0.0, 0.0, 0.01]


class D435iCameraUtilsTest(unittest.TestCase):
    def test_camera_info_dict_contains_ros_calibration_fields(self):
        info = camera_info_dict_from_intrinsics(FakeIntrinsics(), "camera_color_optical_frame")

        self.assertEqual(info["width"], 1280)
        self.assertEqual(info["height"], 720)
        self.assertEqual(info["frame_id"], "camera_color_optical_frame")
        self.assertEqual(info["k"], [910.0, 0.0, 640.0, 0.0, 905.0, 360.0, 0.0, 0.0, 1.0])
        self.assertEqual(info["d"], [0.1, -0.2, 0.0, 0.0, 0.01])

    def test_parse_camera_config_accepts_named_entry(self):
        config = {
            "cameras": {
                "left": {
                    "serial_no": "123",
                    "namespace": "/left_camera",
                    "frame_id": "left_camera_color_optical_frame",
                }
            }
        }

        parsed = parse_camera_config(config, "left")

        self.assertEqual(parsed["serial_no"], "123")
        self.assertEqual(parsed["namespace"], "/left_camera")
        self.assertEqual(parsed["frame_id"], "left_camera_color_optical_frame")


if __name__ == "__main__":
    unittest.main()
