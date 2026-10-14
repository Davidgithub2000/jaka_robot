#!/usr/bin/env python3

import os
import sys
import unittest

import numpy as np


PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT_DIR = os.path.join(PACKAGE_DIR, "scripts")
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from handeye_math import (
    create_chessboard_object_points,
    matrix_to_pose_dict,
    pose_dict_to_matrix,
)


class HandeyeMathTest(unittest.TestCase):
    def test_chessboard_object_points_use_inner_corners_and_meter_units(self):
        points = create_chessboard_object_points((3, 2), 0.02)

        self.assertEqual(points.shape, (6, 3))
        np.testing.assert_allclose(points[0], [0.0, 0.0, 0.0])
        np.testing.assert_allclose(points[1], [0.02, 0.0, 0.0])
        np.testing.assert_allclose(points[3], [0.0, 0.02, 0.0])
        np.testing.assert_allclose(points[-1], [0.04, 0.02, 0.0])

    def test_pose_dict_round_trip_preserves_transform(self):
        transform = np.eye(4)
        transform[:3, :3] = np.array(
            [
                [0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        transform[:3, 3] = [0.1, -0.2, 0.3]

        pose = matrix_to_pose_dict(transform)
        restored = pose_dict_to_matrix(pose)

        np.testing.assert_allclose(restored, transform, atol=1e-9)


if __name__ == "__main__":
    unittest.main()
