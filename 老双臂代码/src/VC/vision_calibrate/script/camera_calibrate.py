#!/home/base00302/vision_env/bin/python3
import cv2
import rospy
from mark_camera import MarkCamera
from Camera import Camera


class CameraCalibrate:
    def __init__(self):
        rospy.init_node('camera_calibrate')        
        self.mark2cam = MarkCamera()
        self.cam_plate_coners = []
        self.plate_point = []
        self.camera_intrinsic_path = rospy.get_param('camera_calibrate/camera_intrinsic_path')
        self.camera_intrinsic_images_path = rospy.get_param('camera_calibrate/camera_intrinsic_images_path')

    def main(self):
        image_id = 0
        while True:
            color_image = cam.get_frame()
            cv2.imshow('camera image', color_image)
            kvalue = cv2.waitKey(50) & 0xFF
            if kvalue == 27:
                break
            elif kvalue == 10 or kvalue == 13:
                ret_find_grid = self.mark2cam.find_circles_grid(color_image)
                if ret_find_grid:
                    inv_image, color_image = self.mark2cam.show(color_image, ret_find_grid)
                    cv2.imwrite(self.camera_intrinsic_images_path+'img_orgin_{}.png'.format(image_id), color_image)
                    cv2.imwrite(self.camera_intrinsic_images_path+'img_calib_{}.png'.format(image_id), inv_image)
                    image_id += 1
                    self.cam_plate_coners.append(self.mark2cam.cam_corners)
                    plate_point = self.mark2cam.get_mark_points()
                    self.plate_point.append(plate_point)
        cv2.destroyAllWindows()
        cam.release()  # 相机释放
        if len(self.cam_plate_coners) < 4:
            print('Camera Calibration Need More Data')
        else:
            ret, mtx, dist, revcs, tvecs = cv2.calibrateCamera(self.plate_point, self.cam_plate_coners,
                                                              (cam.width, cam.height), None, None )
            print('matrix :' + '\n', mtx)
            print('distort :' + '\n', dist)
            if ret < 1:
                fs = cv2.FileStorage(self.camera_intrinsic_path, cv2.FileStorage_WRITE)
                fs.write('cameraMatrix', mtx)
                fs.write('distCoeffs', dist)
                print('File Saved {}'.format(self.camera_intrinsic_path))



if __name__ == '__main__':
    cam = Camera()
    cam_cal = CameraCalibrate()
    cam_cal.main()
