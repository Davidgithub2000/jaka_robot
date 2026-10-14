#!/usr/bin/env python
# coding=utf-8
import pyrealsense2 as rs
import numpy as np


class Camera(object):
    def __init__(self, width=1280, height=720, fps=30):
        self.width = width
        self.height = height
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, fps)
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



