import numpy as np
import pyrealsense2 as rs
from typing import Optional, List
from concurrent import futures
import cv2


class Frame:
    def __init__(self):
        self.raw_color = None
        self.raw_depth = None
        self.depth = None
        self.color = None
        self.is_valid = self.is_valid()
        self.pcd = None

    def is_valid_c(self) -> bool:
        return self.color is not None

    def is_valid_d(self) -> bool:
        return self.depth is not None

    def is_valid(self) -> bool:
        return self.is_valid_c() and self.is_valid_d()


class Camera:
    def __init__(self, is_align: bool = True, width: int = 640, height: int = 480, fps: int = 60, custom_id: int = -1):
        self.pipeline = rs.pipeline()
        self.device_id: int = -1
        self.custom_id: int = custom_id
        self.is_align: bool = is_align
        self.align = rs.align(rs.stream.color) if self.is_align else None
        self.width: int = width
        self.height: int = height
        self.frame_rate: int = fps
        self.frame: Optional[Frame] = Frame()
        self.color_intrin_param: Optional[List[float]] = None
        self.WID = np.expand_dims(np.arange(self.width), 0).repeat(self.height, 0)
        self.HGT = np.expand_dims(np.arange(self.height), 1).repeat(self.width, 1)

    def enable_stream(self, config: rs.config) -> None:
        """
        注册数据流：深度图，彩色图，左右红外相机
        :param config:
        :return:
        """
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, self.frame_rate)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.frame_rate)
        config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, self.frame_rate)
        config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, self.frame_rate)

    def get_color_intrin_part(self):
        color_profile = self.frame.raw_color.get_profile()
        cvsprofile = rs.video_stream_profile(color_profile)
        color_intrin = cvsprofile.get_intrinsics()
        self.color_intrin_param = [color_intrin.ppx, color_intrin.ppy, color_intrin.fx, color_intrin.fy]


class CameraManager:
    def __init__(self, cameras: [Camera]):
        self.num: Optional[int] = None
        self.device = cameras
        self.configs = self.getconfig()
        self.executor = futures.ThreadPoolExecutor(max_workers=8)

    def getconfig(self):
        return [rs.config() for _ in range(len(self.device))]

    def register_stream(self) -> None:
        """
        1.对不同的相机设备分别注册数据流
        :return:
        """
        for camera, config in zip(self.device, self.configs):
            self._single_register_stream(camera, config)

    @staticmethod
    def _single_register_stream(camera: Camera, config: rs.config) -> None:
        """
        单个相近注册数据流
        :param camera: 相机实例
        :param config: 数据流参数实例
        """
        camera.enable_stream(config)

    def set_device(self):
        """
        2. 检查接入设备的设备名和序列号， 并赋予相机实例
        :return:
        """
        for i, d in enumerate(rs.context().devices):
            name = d.get_info(rs.camera_info.name)
            device_id = d.get_info(rs.camera_info.serial_number)
            self.device[i].custom_id = i
            self.device[i].device_id = device_id
            print(f'device : {name}, {device_id}')

    @staticmethod
    def _init_single_pipeline(camera: Camera, config: rs.config):
        """
        单相机建立设备管道
        :param camera:
        :param config:
        :return:
        """
        config.enable_device(camera.device_id)
        camera.pipeline.start(config)

    def init_pipeline(self):
        """
        3. 建立所有相机设备管道
        :return:
        """
        for camera, config in zip(self.device, self.configs):
            self._init_single_pipeline(camera, config)

    @staticmethod
    def _get_frame_single_device(camera: Camera):
        """
        获取单个相机的数据
        :param camera:
        :return:
        """
        frame = camera.pipeline.wait_for_frames()
        if camera.is_align:
            align_frame = camera.align.process(frame)
            camera.frame.raw_color = align_frame.get_color_frame()
            camera.frame.raw_depth = align_frame.get_depth_frame()
            camera.frame.color = np.asarray(camera.frame.raw_color.get_data())
            camera.frame.depth = np.asarray(camera.frame.raw_depth.get_data())
            camera.get_color_intrin_part()
        else:
            camera.frame.raw_color = frame.get_color_frame()
            camera.frame.raw_depth = frame.get_depth_frame()
            camera.frame.color = np.asarray(camera.frame.raw_color.get_data())
            camera.frame.depth = np.asarray(camera.frame.raw_depth.get_data())
            camera.get_color_intrin_part()
        if camera.frame.is_valid:
            return True
        else:
            return False

    def get_frame(self):
        future_cache = []
        camera_list = []
        for camera in self.device:
            future = self.executor.submit(self._get_frame_single_device, camera)
            if future:
                future_cache.append(future)
                camera_list.append(future.result())
        if len(future_cache):
            return True
        else:
            return False


if __name__ == '__main__':
    import time

    camera0 = Camera(is_align=True, width=1280, height=720, custom_id=0, fps=30)
    camera1 = Camera(is_align=True, width=1280, height=720, custom_id=1, fps=30)
    manager = CameraManager([camera0, camera1])

    manager.register_stream()
    manager.set_device()
    manager.init_pipeline()
    completed = False
    cv2.namedWindow('img')
    while not completed:
        t0 = time.time()
        manager.get_frame()
        t1 = time.time()
        print("FPS: {:.2f}".format(1 / (t1 - t0 + 0.00000001)))
        img = camera0.frame.color
        img1 = camera1.frame.color
        cv2.imshow('img', img)
        cv2.imshow('img1', img1)
        k = cv2.waitKey(10)
        if k == 27:
            cv2.destroyAllWindows()
            for i in manager.device:
                i.pipeline.stop()
            break
