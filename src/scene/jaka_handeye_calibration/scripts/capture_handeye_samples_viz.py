#!/usr/bin/env python3

import os
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


class HandeyeCaptureVizNode:
    def __init__(self):
        self.arm_name = rospy.get_param("~arm_name", "left")
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/color/camera_info")
        self.tool_position_topic = rospy.get_param("~tool_position_topic", "/jaka_driver/tool_position")
        self.samples_file = os.path.expandvars(os.path.expanduser(rospy.get_param("~samples_file")))
        self.pattern_size = tuple(rospy.get_param("~board/pattern_size", [11, 8]))
        self.square_size_m = float(rospy.get_param("~board/square_size_m", 0.020))
        self.target_samples = int(rospy.get_param("~target_samples", 20))
        self.display_rate_hz = float(rospy.get_param("~display_rate_hz", 15.0))
        self.axis_length_m = float(rospy.get_param("~axis_length_m", 0.08))

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
        self.last_status = "Ready"
        self.last_status_time = 0.0
        self.last_reproj_error = None
        self.last_capture_index = None

        self.object_points = create_chessboard_object_points(self.pattern_size, self.square_size_m)
        self._load_existing_samples()

        rospy.Subscriber(self.image_topic, Image, self._image_cb, queue_size=1)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self._camera_info_cb, queue_size=1)
        rospy.Subscriber(self.tool_position_topic, rospy.AnyMsg, self._tool_any_cb, queue_size=1)
        rospy.Subscriber("~capture", Empty, self._capture_topic_cb, queue_size=1)

        self.window_name = "{} handeye capture".format(self.arm_name)
        rospy.loginfo(
            "[%s] visual handeye capture ready. Use the image window: Enter/s saves, q/Esc exits.",
            self.arm_name,
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

    def _snapshot(self):
        with self.lock:
            return {
                "image": None if self.latest_image is None else self.latest_image.copy(),
                "camera_matrix": None if self.camera_matrix is None else self.camera_matrix.copy(),
                "dist_coeffs": None if self.dist_coeffs is None else self.dist_coeffs.copy(),
                "base_t_tool": None if self.latest_tool is None else self.latest_tool.copy(),
                "image_stamp": self.latest_image_stamp,
                "tool_stamp": self.latest_tool_stamp,
            }

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

    def _estimate_target(self, corners, camera_matrix, dist_coeffs):
        camera_t_target = solve_pnp_target_to_camera(
            self.object_points,
            corners,
            camera_matrix,
            dist_coeffs,
        )
        rvec, _ = cv2.Rodrigues(camera_t_target[:3, :3])
        tvec = camera_t_target[:3, 3].reshape(3, 1)
        projected, _ = cv2.projectPoints(
            self.object_points,
            rvec,
            tvec,
            camera_matrix,
            dist_coeffs,
        )
        projected = projected.reshape(-1, 2)
        reproj_error = float(np.mean(np.linalg.norm(projected - corners, axis=1)))
        return camera_t_target, rvec, tvec, reproj_error

    def capture_one(self):
        data = self._snapshot()
        image = data["image"]
        camera_matrix = data["camera_matrix"]
        dist_coeffs = data["dist_coeffs"]
        base_t_tool = data["base_t_tool"]

        if image is None or camera_matrix is None or dist_coeffs is None or base_t_tool is None:
            self._set_status("FAIL: missing image/camera_info/tool pose", False)
            rospy.logwarn("[%s] missing image, camera_info, or tool pose", self.arm_name)
            return False

        corners = self._find_corners(image)
        if corners is None:
            self._set_status("FAIL: chessboard not found", False)
            rospy.logwarn("[%s] chessboard not found", self.arm_name)
            return False

        try:
            camera_t_target, _rvec, _tvec, reproj_error = self._estimate_target(
                corners,
                camera_matrix,
                dist_coeffs,
            )
        except Exception as exc:
            self._set_status("FAIL: PnP {}".format(str(exc)), False)
            rospy.logwarn("[%s] solvePnP failed: %s", self.arm_name, str(exc))
            return False

        sample = {
            "index": len(self.samples),
            "stamp": float(time.time()),
            "image_stamp": float(data["image_stamp"] or 0.0),
            "tool_stamp": float(data["tool_stamp"] or 0.0),
            "reprojection_error_px": reproj_error,
            "base_T_tool": matrix_to_pose_dict(base_t_tool),
            "camera_T_target": matrix_to_pose_dict(camera_t_target),
        }
        self.samples.append(sample)
        self._save()
        self.last_reproj_error = reproj_error
        self.last_capture_index = len(self.samples)
        self._set_status("SAVED sample {}  reproj={:.2f}px".format(len(self.samples), reproj_error), True)
        rospy.loginfo(
            "[%s] saved sample %d to %s, reprojection error %.3f px",
            self.arm_name,
            len(self.samples),
            self.samples_file,
            reproj_error,
        )
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

    def _set_status(self, text, ok):
        self.last_status = text
        self.last_status_ok = bool(ok)
        self.last_status_time = time.time()

    def _draw_chessboard_overlay(self, image, corners):
        cols, rows = self.pattern_size
        corners_i = np.round(corners).astype(int)
        for row in range(rows):
            color = tuple(int(x) for x in cv2.applyColorMap(
                np.array([[int(255 * row / max(1, rows - 1))]], dtype=np.uint8),
                cv2.COLORMAP_RAINBOW,
            )[0, 0])
            row_points = corners_i[row * cols:(row + 1) * cols]
            for p1, p2 in zip(row_points[:-1], row_points[1:]):
                cv2.line(image, tuple(p1), tuple(p2), color, 2, cv2.LINE_AA)

        for col in range(cols):
            col_points = corners_i[col::cols]
            for p1, p2 in zip(col_points[:-1], col_points[1:]):
                cv2.line(image, tuple(p1), tuple(p2), (0, 220, 255), 1, cv2.LINE_AA)

        for point in corners_i:
            cv2.circle(image, tuple(point), 4, (0, 0, 255), -1, cv2.LINE_AA)

    def _draw_info_panel(self, image, found, reproj_error, have_camera, have_tool):
        sample_text = "Samples: {}/{}".format(len(self.samples), self.target_samples)
        lines = [
            "{}  {}".format(self.arm_name.upper(), sample_text),
            "Board: {}x{} inner, {:.1f} mm".format(
                self.pattern_size[0],
                self.pattern_size[1],
                self.square_size_m * 1000.0,
            ),
            "Detection: {}".format("FOUND" if found else "NOT FOUND"),
            "PnP reproj: {}".format("{:.2f} px".format(reproj_error) if reproj_error is not None else "--"),
            "CameraInfo: {}   ToolPose: {}".format("OK" if have_camera else "WAIT", "OK" if have_tool else "WAIT"),
            "Last: {}".format(self.last_status),
            "Keys: Enter/s save, q/Esc exit",
        ]

        line_h = 24
        panel_h = 12 + line_h * len(lines)
        panel_w = 560
        overlay = image.copy()
        cv2.rectangle(overlay, (8, 8), (panel_w, panel_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.45, image, 0.55, 0, image)

        for index, line in enumerate(lines):
            y = 32 + index * line_h
            color = (255, 255, 255)
            if line.startswith("Detection:"):
                color = (80, 255, 80) if found else (80, 80, 255)
            elif line.startswith("Last:"):
                color = (80, 255, 80) if getattr(self, "last_status_ok", True) else (80, 80, 255)
            cv2.putText(image, line, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2, cv2.LINE_AA)

    def render(self):
        data = self._snapshot()
        image = data["image"]
        camera_matrix = data["camera_matrix"]
        dist_coeffs = data["dist_coeffs"]
        have_camera = camera_matrix is not None and dist_coeffs is not None
        have_tool = data["base_t_tool"] is not None

        if image is None:
            image = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.putText(image, "Waiting for image: {}".format(self.image_topic), (40, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
            self._draw_info_panel(image, False, None, have_camera, have_tool)
            return image

        display = image.copy()
        corners = self._find_corners(display)
        found = corners is not None
        reproj_error = None

        if found:
            self._draw_chessboard_overlay(display, corners)
            if have_camera:
                try:
                    _camera_t_target, rvec, tvec, reproj_error = self._estimate_target(
                        corners,
                        camera_matrix,
                        dist_coeffs,
                    )
                    cv2.drawFrameAxes(
                        display,
                        camera_matrix,
                        dist_coeffs,
                        rvec,
                        tvec,
                        self.axis_length_m,
                    )
                except Exception as exc:
                    rospy.logwarn_throttle(2.0, "[%s] visualization PnP failed: %s", self.arm_name, str(exc))

        self._draw_info_panel(display, found, reproj_error, have_camera, have_tool)
        return display

    def spin(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        rate = rospy.Rate(self.display_rate_hz)
        while not rospy.is_shutdown():
            frame = self.render()
            cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (10, 13, ord("s")):
                self.capture_one()
            elif key in (27, ord("q")):
                rospy.signal_shutdown("user closed visual capture")
                break
            rate.sleep()
        cv2.destroyWindow(self.window_name)


def main():
    rospy.init_node("handeye_capture_viz")
    node = HandeyeCaptureVizNode()
    node.spin()


if __name__ == "__main__":
    main()
