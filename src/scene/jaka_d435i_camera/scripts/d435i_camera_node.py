#!/usr/bin/env python3

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import cv2
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

from d435i_camera_utils import camera_info_dict_from_intrinsics, parse_camera_config

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


def _clean_namespace(namespace):
    namespace = str(namespace or "/camera").strip()
    if not namespace.startswith("/"):
        namespace = "/" + namespace
    return namespace.rstrip("/")


class D435iCameraNode:
    def __init__(self):
        if rs is None:
            raise RuntimeError("pyrealsense2 is not installed in this Python environment")

        self.camera_name = rospy.get_param("~camera_name", "single")
        config_file = rospy.get_param("~config_file", "")
        config_data = {}
        if config_file:
            config_file = os.path.expandvars(os.path.expanduser(config_file))
            with open(config_file, "r", encoding="utf-8") as stream:
                config_data = yaml.safe_load(stream) or {}

        defaults = parse_camera_config(config_data, self.camera_name) if config_data else {}
        self.serial_no = rospy.get_param("~serial_no", defaults.get("serial_no", ""))
        self.namespace = _clean_namespace(rospy.get_param("~namespace", defaults.get("namespace", "/camera")))
        self.frame_id = rospy.get_param("~frame_id", defaults.get("frame_id", "camera_color_optical_frame"))
        self.width = int(rospy.get_param("~width", defaults.get("width", 1280)))
        self.height = int(rospy.get_param("~height", defaults.get("height", 720)))
        self.fps = int(rospy.get_param("~fps", defaults.get("fps", 30)))
        self.show_window = bool(rospy.get_param("~show_window", False))
        self.publish_depth = bool(rospy.get_param("~publish_depth", False))
        self.enable_auto_exposure = bool(rospy.get_param("~enable_auto_exposure", True))

        self.bridge = CvBridge()
        self.pipeline = rs.pipeline()
        self.align = rs.align(rs.stream.color)
        self.camera_info_msg = None

        self.image_pub = rospy.Publisher(self.namespace + "/color/image_raw", Image, queue_size=1)
        self.info_pub = rospy.Publisher(self.namespace + "/color/camera_info", CameraInfo, queue_size=1)
        self.depth_pub = None
        if self.publish_depth:
            self.depth_pub = rospy.Publisher(self.namespace + "/aligned_depth_to_color/image_raw", Image, queue_size=1)

    def start(self):
        config = rs.config()
        if self.serial_no:
            config.enable_device(str(self.serial_no))
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        if self.publish_depth:
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)

        profile = self.pipeline.start(config)
        device = profile.get_device()
        actual_serial = device.get_info(rs.camera_info.serial_number)
        rospy.loginfo(
            "[%s] D435i started serial=%s namespace=%s frame_id=%s size=%dx%d fps=%d",
            self.camera_name,
            actual_serial,
            self.namespace,
            self.frame_id,
            self.width,
            self.height,
            self.fps,
        )

        for sensor in device.query_sensors():
            if sensor.supports(rs.option.enable_auto_exposure):
                sensor.set_option(rs.option.enable_auto_exposure, 1.0 if self.enable_auto_exposure else 0.0)

        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.camera_info_msg = self._make_camera_info(color_stream.get_intrinsics())

    def _make_camera_info(self, intrinsics):
        data = camera_info_dict_from_intrinsics(intrinsics, self.frame_id)
        msg = CameraInfo()
        msg.header.frame_id = data["frame_id"]
        msg.width = data["width"]
        msg.height = data["height"]
        msg.distortion_model = data["distortion_model"]
        msg.D = data["d"]
        msg.K = data["k"]
        msg.R = data["r"]
        msg.P = data["p"]
        return msg

    def spin(self):
        rate = rospy.Rate(self.fps)
        while not rospy.is_shutdown():
            frames = self.pipeline.wait_for_frames()
            if self.publish_depth:
                frames = self.align.process(frames)
                depth_frame = frames.get_depth_frame()
            else:
                depth_frame = None
            color_frame = frames.get_color_frame()
            if not color_frame:
                rospy.logwarn_throttle(2.0, "[%s] no color frame", self.camera_name)
                rate.sleep()
                continue

            stamp = rospy.Time.now()
            color_image = np.asanyarray(color_frame.get_data())
            image_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id
            self.image_pub.publish(image_msg)

            info_msg = self.camera_info_msg
            info_msg.header.stamp = stamp
            self.info_pub.publish(info_msg)

            if self.depth_pub is not None and depth_frame:
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
                depth_msg.header.stamp = stamp
                depth_msg.header.frame_id = self.frame_id
                self.depth_pub.publish(depth_msg)

            if self.show_window:
                cv2.imshow(self.camera_name, color_image)
                key = cv2.waitKey(1) & 0xFF
                if key == 27 or key == ord("q"):
                    rospy.signal_shutdown("user closed camera window")

            rate.sleep()

    def stop(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        if self.show_window:
            cv2.destroyAllWindows()


def main():
    rospy.init_node("d435i_camera_node")
    node = D435iCameraNode()
    rospy.on_shutdown(node.stop)
    node.start()
    node.spin()


if __name__ == "__main__":
    main()
