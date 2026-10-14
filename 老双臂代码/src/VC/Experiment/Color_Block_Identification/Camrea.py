#!/usr/bin/env python
# coding=utf-8
import pyrealsense2 as rs
import numpy as np
import cv2
class Camrea(object):
    def __init__(self, width=1280, height=720, fps=30):
        super(Camrea, self).__init__()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.pipeline.start(self.config)       # 开始连接相机
        self.align = rs.align(rs.stream.color)
    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        image = np.asanyarray(color_frame.get_data())
        return image
    def release(self):
        self.pipeline.stop()

if __name__ == '__main__':
    cam = Camrea()
    while True:
        color_image = cam.get_frame()
        cv2.imshow('camera image', color_image)
        cv2.waitKey(100)



