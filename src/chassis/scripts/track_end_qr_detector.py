#!/usr/bin/env python3
"""Detect QR code at track end and notify the base controller."""

import time

import cv2
import rospy
from std_msgs.msg import Bool, String


class TrackEndQrDetector:
    def __init__(self):
        # 现场换相机时，不需要改本脚本；改 launch 里的 track_end_qr_camera_device 参数即可。
        # 支持 0、1 这种相机序号，也支持 /dev/video2 或 /dev/chassis_qr_camera 这种设备路径。
        self.camera_device = rospy.get_param("~camera_device", 0)
        # required_text 为空表示任意二维码都触发；填写具体二维码内容则只认这一种二维码。
        self.required_text = rospy.get_param("~required_text", "")
        self.scan_rate_hz = float(rospy.get_param("~scan_rate_hz", 10.0))
        self.open_retry_s = float(rospy.get_param("~open_retry_s", 1.0))
        self.publish_false_on_idle = bool(rospy.get_param("~publish_false_on_idle", True))

        track_end_topic = rospy.get_param("~track_end_detected_topic", "/base/track_end_detected")
        qr_detected_topic = rospy.get_param("~qr_detected_topic", "/base/track_end_qr_detected")
        qr_text_topic = rospy.get_param("~qr_text_topic", "/base/track_end_qr_text")

        self.armed = False
        self.reported = False
        self.capture = None
        self.last_open_try = 0.0
        self.detector = cv2.QRCodeDetector()

        self.qr_pub = rospy.Publisher(qr_detected_topic, Bool, queue_size=5, latch=True)
        self.text_pub = rospy.Publisher(qr_text_topic, String, queue_size=5, latch=True)
        self.track_end_sub = rospy.Subscriber(track_end_topic, Bool, self.track_end_callback, queue_size=5)

        if self.publish_false_on_idle:
            self.qr_pub.publish(Bool(False))

        rospy.loginfo("track_end_qr_detector started. camera_device=%s required_text=%s",
                      str(self.camera_device), self.required_text or "<any>")

    # 收到 /base/track_end_detected=true 后才开始扫码，平时不占用相机。
    def track_end_callback(self, msg):
        if msg.data:
            self.armed = True
            self.reported = False
            # 每次重新进入扫码状态时先发布 false，清掉上一次 latch 的 true。
            if self.publish_false_on_idle:
                self.qr_pub.publish(Bool(False))
            rospy.logwarn("Track end received. Start QR scanning.")
        else:
            self.armed = False
            self.reported = False
            self.close_capture()
            if self.publish_false_on_idle:
                self.qr_pub.publish(Bool(False))

    def normalize_camera_device(self):
        # OpenCV 的 VideoCapture 可以接收整数序号，也可以接收 /dev/videoX 字符串。
        # 如果 launch 传入 "2"，这里转成整数 2；如果传入 /dev/video2，则保持字符串。
        if isinstance(self.camera_device, str):
            stripped = self.camera_device.strip()
            if stripped.isdigit():
                return int(stripped)
            return stripped
        return int(self.camera_device)

    # 相机可能被系统晚一点枚举出来，打开失败时按 open_retry_s 间隔重试。
    def ensure_capture(self):
        if self.capture is not None and self.capture.isOpened():
            return True

        now = time.time()
        if now - self.last_open_try < self.open_retry_s:
            return False

        self.last_open_try = now
        self.close_capture()
        device = self.normalize_camera_device()
        self.capture = cv2.VideoCapture(device)
        if not self.capture.isOpened():
            rospy.logwarn_throttle(5.0, "Cannot open QR camera: %s", str(device))
            return False

        rospy.loginfo("QR camera opened: %s", str(device))
        return True

    def close_capture(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None

    # 只在 armed 状态下读图识别；识别成功后本轮不再重复发布 true。
    def scan_once(self):
        if not self.armed or self.reported:
            return

        if not self.ensure_capture():
            return

        ok, frame = self.capture.read()
        if not ok or frame is None:
            rospy.logwarn_throttle(5.0, "QR camera read failed.")
            self.close_capture()
            return

        text, _, _ = self.detector.detectAndDecode(frame)
        if not text:
            return

        # 如果配置了 required_text，只接受内容完全一致的二维码，避免扫到无关二维码。
        if self.required_text and text != self.required_text:
            rospy.logwarn_throttle(2.0, "Ignore QR text: %s", text)
            return

        self.reported = True
        self.qr_pub.publish(Bool(True))
        self.text_pub.publish(String(text))
        rospy.logwarn("Track-end QR detected: %s", text)

    def spin(self):
        rate = rospy.Rate(max(1.0, self.scan_rate_hz))
        while not rospy.is_shutdown():
            self.scan_once()
            rate.sleep()
        self.close_capture()


if __name__ == "__main__":
    rospy.init_node("track_end_qr_detector")
    TrackEndQrDetector().spin()
