import cv2
import numpy as np
import math
import rospy


def file_read(camera_intrinsic_calibration_path):  # 读取标定文件中保存的结果（相机内参矩阵和畸变系数）到对应的变量中
    fs = cv2.FileStorage(camera_intrinsic_calibration_path, cv2.FileStorage_READ)
    camera_matrix = fs.getNode('cameraMatrix').mat()
    camera_distort = fs.getNode('distCoeffs').mat()
    fs.release()
    return np.float32(camera_matrix), np.float32(camera_distort)


class MarkCamera:
    def __init__(self):
        self.w = 4
        self.h = 5
        self.b = 0.025 #0.015
        self.minArea = 300
        self.maxArea = 100000
        self.mark_points = []
        self.cam_corners = []
        try:
            self.camera_intrinsic_path = rospy.get_param('calibration_hand_eye/camera_intrinsic_path')
            self.camera_matrix, self.camera_distort = file_read(self.camera_intrinsic_path)
            print('Start Hand Eye Calibration...')
        except:
            self.camera_matrix, self.camera_distort = np.zeros((3,3), np.uint8), np.zeros((1, 5), np.uint8)
            print('Start Internal reference calibration...')
            print('If Calibration Done.. Please Check File Path!')

    def get_mark_points(self):
        """
        获取标定版为世界坐标系的圆心世界坐标
        :return: 圆心世界坐标
        """
        object_points = []
        for i in range(self.h):
            for j in range(self.w):
                object_points.append([(i % 2 + j * 2) * self.b, i * self.b, 0])
        return np.float32(object_points)

    def find_circles_grid(self, color_image):
        """
        查找圆心坐标
        :param color_image:
        :return: 是否找到True or False, 图像上的圆心坐标
        """
        params = cv2.SimpleBlobDetector_Params()
        params.maxArea = self.maxArea
        params.minArea = self.minArea
        blob_detector = cv2.SimpleBlobDetector_create(params)
        ret_find_grid, corners = cv2.findCirclesGrid(image=color_image, patternSize=(self.w, self.h),
                                           flags=cv2.CALIB_CB_ASYMMETRIC_GRID,
                                           blobDetector=blob_detector)
        self.cam_corners = corners
        return ret_find_grid

    def compute_mark2cam(self):
        """
        通过相机内参与畸变系数计算标定版坐标系到相机坐标系的转化关系
        :param color_image:
        :return:
        """
        self.mark_points = self.get_mark_points()
        ret_mark2cam, mark2cam_rotate_rodrigues, mark2cam_translation = cv2.solvePnP(self.mark_points, self.cam_corners,
                                                                            self.camera_matrix, self.camera_distort)
        return ret_mark2cam, mark2cam_rotate_rodrigues, mark2cam_translation

    def check_mark2cam(self, mark2cam_rotate_rodrigues, mark2cam_translation):
        """
        通过cv2.projectPoints进行反投影验证
        :return:
        """
        # if Ret is True:
        projectImagePoints, _ = cv2.projectPoints(self.mark_points, mark2cam_rotate_rodrigues, mark2cam_translation,
                                                  self.camera_matrix,
                                                  self.camera_distort)
        projectOffsetPowSum = 0
        for j in range(len(projectImagePoints)):
            # 使用opencv提供的相机外参计算函数，传入上面已经获取到的参数，计算标定板坐标系到相机坐标系的坐标变换，并存储到rotateRodriguesVec和translationVec
            error = cv2.norm(projectImagePoints[j], self.cam_corners[j], cv2.NORM_L2) / len(projectImagePoints)
            projectOffsetPowSum += error
        projectRMS = projectOffsetPowSum / math.sqrt(len(projectImagePoints))
        print("total error: {}".format(projectRMS))
        return projectRMS
        # if projectRMS < 1.0:
        #     print('重投影均方根误差在可接受范围内，本次采集为有效数据')

    def show(self, color_image, ret_find_grid):
        origin_color = color_image.copy()
        cv2.drawChessboardCorners(color_image, (self.w, self.h), self.cam_corners, ret_find_grid)
        inv_image = 255 - color_image
        cv2.imshow('camera image', inv_image)
        cv2.waitKey(1000)
        return inv_image, origin_color





