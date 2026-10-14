#!/home/base00302/vision_env/bin/python3
import cv2
import numpy as np
import rospy
import transforms3d as tfs
from geometry_msgs.msg import Pose
from mark_camera import MarkCamera
from Camera import Camera


class Calibration:
    def __init__(self):
        rospy.init_node('calibration_hand_eye')
        self.isEyeInHand = False
        self.end_base_rot = []
        self.end_base_tr = []
        self.base_end_rot = []
        self.base_end_tr = []
        self.mark2cam = MarkCamera()
        self.mark_cameras_rot = []
        self.mark_cameras_tr = []
        self.eye_in_hand_path = rospy.get_param('calibration_hand_eye/eye_in_hand_path')
        self.eye_to_hand_path = rospy.get_param('calibration_hand_eye/eye_to_hand_path')
        self.camera_handeye_images_path = rospy.get_param('calibration_hand_eye/camera_handeye_images_path')
        self.isEyeInHand = rospy.get_param("calibration_hand_eye/isEyeInHand")

    def get_end2base(self):
        object_pose = rospy.wait_for_message("/tool_point", Pose, rospy.Duration(2))
        if object_pose is not None:
            x = object_pose.position.x
            y = object_pose.position.y
            z = object_pose.position.z
            Qx = object_pose.orientation.x
            Qy = object_pose.orientation.y
            Qz = object_pose.orientation.z
            Qw = object_pose.orientation.w
            Rq = [Qw, Qx, Qy, Qz]
            end2base_rotate = tfs.quaternions.quat2mat((Rq[0], Rq[1], Rq[2], Rq[3]))
            end2base_trans = np.array((x, y, z))
            print("end2base")
            print(end2base_rotate)
            print(end2base_trans)
            if self.isEyeInHand:
                self.end_base_rot.append(end2base_rotate)
                self.end_base_tr.append(end2base_trans)
            else:
                temp = tfs.affines.compose(np.squeeze(end2base_trans), end2base_rotate, [1, 1, 1])
                temp = np.linalg.inv(temp)
                base2end_trans = temp[0:3,3:4]
                base2end_rotate = temp[0:3,0:3]
                self.base_end_rot.append(base2end_rotate)
                self.base_end_tr.append(base2end_trans)

    def compute_calibration(self):
        if self.isEyeInHand:
            camera_end_rot, camera_end_tr = cv2.calibrateHandEye(self.end_base_rot, self.end_base_tr,
                                                                 self.mark_cameras_rot,
                                                                 self.mark_cameras_tr, method=cv2.CALIB_HAND_EYE_TSAI)

            fs_mechbase = cv2.FileStorage(self.eye_in_hand_path, cv2.FileStorage_WRITE)
            fs_mechbase.write('Rotate', camera_end_rot)
            fs_mechbase.write('Translation', camera_end_tr)
            print('Calibrate Eye In Hand result' + '\n', fs_mechbase)
            print('File Saved {}'.format(self.eye_in_hand_path))
        else:
            camera_base_rot, camera_base__tr = cv2.calibrateHandEye(self.base_end_rot, self.base_end_tr,
                                                                    self.mark_cameras_rot,
                                                                    self.mark_cameras_tr, method=cv2.CALIB_HAND_EYE_TSAI)
            fs_mechbase = cv2.FileStorage(self.eye_to_hand_path, cv2.FileStorage_WRITE)
            fs_mechbase.write('Rotate', camera_base_rot)
            fs_mechbase.write('Translation', camera_base__tr)
            print('To Hand Calibrate Result: ' + '\n', fs_mechbase)
            print('File Saved {}'.format(self.eye_to_hand_path))

    def main(self):
        image_id = 0
        while True:
            color_image = cam.get_frame()
            cv2.imshow('camera image', color_image)
            k_value = cv2.waitKey(50) & 0xFF
            if k_value == 27:
                break
            elif k_value == 10 or k_value == 13:
                ret_find_grid = self.mark2cam.find_circles_grid(color_image)
                if ret_find_grid:
                    inv_image, color_image = self.mark2cam.show(color_image, ret_find_grid)
                    ret_mark2cam, mark2cam_rotate_rodrigues, mark2cam_translation = self.mark2cam.compute_mark2cam()
                    if ret_mark2cam:
                        project_rms = self.mark2cam.check_mark2cam(mark2cam_rotate_rodrigues, mark2cam_translation)
                        if project_rms < 1.0:
                            print('重投影均方根误差在可接受范围内，本次采集为有效数据')
                            cv2.imwrite(self.camera_handeye_images_path+'img_orgin_{}.png'.format(image_id), color_image)
                            cv2.imwrite(self.camera_handeye_images_path+'img_calib_{}.png'.format(image_id), inv_image)
                            image_id += 1
                            self.get_end2base()
                            mark2cam_rotate, _ = cv2.Rodrigues(mark2cam_rotate_rodrigues)
                            self.mark_cameras_rot.append(mark2cam_rotate)
                            self.mark_cameras_tr.append(mark2cam_translation)
        cv2.destroyAllWindows()
        cam.release()
        print(self.mark_cameras_tr, self.mark_cameras_rot)
        print(len(self.mark_cameras_tr))
        print('Begin Calculate Hand Eye Calibration')
        if len(self.mark_cameras_rot) < 4:
            print('Hand Eye Calibration Need More Data')
        else:
            self.compute_calibration()

        
if __name__ == '__main__':
    cam = Camera()
    calibration = Calibration()
    calibration.main()






