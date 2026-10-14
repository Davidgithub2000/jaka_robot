#!/usr/bin/env python3

import os
import select
import sys
import threading
import time

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import cv2
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty

from handeye_math import (
    create_chessboard_object_points,
    matrix_to_pose_dict,
    solve_pnp_target_to_camera,
    twist_stamped_to_matrix,
)


class HandeyeCaptureNode:
    def __init__(self):
        self.arm_name = rospy.get_param("~arm_name", "left")
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/color/camera_info")
        self.tool_position_topic = rospy.get_param("~tool_position_topic", "/jaka_driver/tool_position")
        self.samples_file = os.path.expandvars(os.path.expanduser(rospy.get_param("~samples_file")))
        self.pattern_size = tuple(rospy.get_param("~board/pattern_size", [11, 8]))
        self.square_size_m = float(rospy.get_param("~board/square_size_m", 0.020))
        self.pos_unit = rospy.get_param("~tool_position_pos_unit", "mm")
        self.rpy_unit = rospy.get_param("~tool_position_rpy_unit", "deg")
        self.rotation_type = rospy.get_param("~tool_position_rotation_type", "euler")
        self.euler_order = rospy.get_param("~tool_position_euler_order", "xyz")
        self.euler_intrinsic = bool(rospy.get_param("~tool_position_euler_intrinsic", False))

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_image = None
        self.latest_image_stamp = None
        self.latest_tool = None
        self.latest_tool_stamp = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.samples = []

        self.object_points = create_chessboard_object_points(self.pattern_size, self.square_size_m)
        self._load_existing_samples()

        rospy.Subscriber(self.image_topic, Image, self._image_cb, queue_size=1)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self._camera_info_cb, queue_size=1)
        rospy.Subscriber(self.tool_position_topic, rospy.AnyMsg, self._tool_any_cb, queue_size=1)
        rospy.Subscriber("~capture", Empty, self._capture_topic_cb, queue_size=1)

        self.stdin_thread = threading.Thread(target=self._stdin_loop)
        self.stdin_thread.daemon = True
        self.stdin_thread.start()

        rospy.loginfo(
            "[%s] handeye capture ready. Press 's' then Enter, or publish std_msgs/Empty to %s/capture.",
            self.arm_name,
            rospy.get_name(),
        )

    def _load_existing_samples(self):
        if not os.path.exists(self.samples_file):
            return
        with open(self.samples_file, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        self.samples = list(data.get("samples", []))
        rospy.loginfo("[%s] loaded %d existing samples", self.arm_name, len(self.samples))

    def _image_cb(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        with self.lock:
            self.latest_image = image
            self.latest_image_stamp = msg.header.stamp.to_sec() if msg.header.stamp else time.time()

    def _camera_info_cb(self, msg):
        with self.lock:
            self.camera_matrix = np.array(msg.K, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D, dtype=np.float64).reshape(-1, 1)

    def _tool_any_cb(self, any_msg):
        from geometry_msgs.msg import TwistStamped

        msg = TwistStamped().deserialize(any_msg._buff)
        transform = twist_stamped_to_matrix(
            msg,
            self.pos_unit,
            self.rpy_unit,
            self.rotation_type,
            self.euler_order,
            self.euler_intrinsic,
        )
        stamp = msg.header.stamp.to_sec() if msg.header.stamp else time.time()
        with self.lock:
            self.latest_tool = transform
            self.latest_tool_stamp = stamp

    def _capture_topic_cb(self, _msg):
        self.capture_one()

    def _stdin_loop(self):
        while not rospy.is_shutdown():
            try:
                if os.name == "nt":
                    line = sys.stdin.readline()
                else:
                    ready, _, _ = select.select([sys.stdin], [], [], 0.5)
                    if not ready:
                        continue
                    line = sys.stdin.readline()
            except Exception:
                return
            command = line.strip().lower()
            if command in ("s", "save", "capture"):
                self.capture_one()
            elif command in ("q", "quit", "exit"):
                rospy.signal_shutdown("user requested exit")

    def _find_corners(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ok, corners = cv2.findChessboardCorners(gray, self.pattern_size, flags)
        if not ok:
            return None
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        return corners.reshape(-1, 2)

    def capture_one(self):
        with self.lock:
            image = None if self.latest_image is None else self.latest_image.copy()
            camera_matrix = None if self.camera_matrix is None else self.camera_matrix.copy()
            dist_coeffs = None if self.dist_coeffs is None else self.dist_coeffs.copy()
            base_t_tool = None if self.latest_tool is None else self.latest_tool.copy()
            image_stamp = self.latest_image_stamp
            tool_stamp = self.latest_tool_stamp

        if image is None or camera_matrix is None or dist_coeffs is None or base_t_tool is None:
            rospy.logwarn("[%s] missing image, camera_info, or tool pose", self.arm_name)
            return False

        corners = self._find_corners(image)
        if corners is None:
            rospy.logwarn("[%s] chessboard not found", self.arm_name)
            return False

        camera_t_target = solve_pnp_target_to_camera(
            self.object_points,
            corners,
            camera_matrix,
            dist_coeffs,
        )
        sample = {
            "index": len(self.samples),
            "stamp": float(time.time()),
            "image_stamp": float(image_stamp or 0.0),
            "tool_stamp": float(tool_stamp or 0.0),
            "base_T_tool": matrix_to_pose_dict(base_t_tool),
            "camera_T_target": matrix_to_pose_dict(camera_t_target),
        }
        self.samples.append(sample)
        self._save()
        rospy.loginfo("[%s] saved sample %d to %s", self.arm_name, len(self.samples), self.samples_file)
        return True

    def _save(self):
        directory = os.path.dirname(self.samples_file)
        if directory:
            os.makedirs(directory, exist_ok=True)
        data = {
            "arm_name": self.arm_name,
            "board": {
                "pattern_size": list(self.pattern_size),
                "square_size_m": self.square_size_m,
            },
            "samples": self.samples,
        }
        with open(self.samples_file, "w", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream, default_flow_style=False, sort_keys=False)


def main():
    rospy.init_node("handeye_capture")
    HandeyeCaptureNode()
    rospy.spin()


if __name__ == "__main__":
    main()
