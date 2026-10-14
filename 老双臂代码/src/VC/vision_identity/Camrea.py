import pyrealsense2 as rs
import numpy as np
import cv2

class Carmera(object):
    def __init__(self,width=1280,height=720,fps =15):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, fps)
        self.pipeline.start(self.config)
        sensor = self.pipeline.get_active_profile().get_device().query_sensors()[1]
        print(sensor.get_option(rs.option.exposure))
        self.align = rs.align(rs.stream.color)
    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        return color_frame,aligned_depth_frame
    def get_color_intrin_part(self,color_frame):
        color_profile = color_frame.get_profile()
        cvsprofile = rs.video_stream_profile(color_profile)
        color_intrin = cvsprofile.get_intrinsics()
        color_intrin_part = [color_intrin.ppx, color_intrin.ppy, color_intrin.fx, color_intrin.fy]
        return color_intrin_part
    def stop_pipline(self):
        self.pipeline.stop()
if __name__ == '__main__':
    cam = Carmera()
    while True:
       color_frame, aligned_depth_frame = cam.get_frame()
       image = np.asanyarray(color_frame.get_data())
       cv2.imshow('image', image)
       k = cv2.waitKey(1)
       if k == 27:
            cv2.destroyAllWindows()
            cam.stop_pipline()

