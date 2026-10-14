from detect_new import YoloV5Detect
# from Camrea import Carmera
from realsense import Camera, CameraManager
from util import *
import numpy as np
import rospy
from scipy.spatial.transform import Rotation as R
from yolov5_ROS import Vision
from geometry_msgs.msg import Pose


class DataInference:
    def __init__(self):
        self.cam0 = Camera(is_align=True, width=1280, height=720, custom_id=0, fps=30)
        #self.cam1 = Camera(is_align=True, width=1280, height=720, custom_id=1, fps=30)
        #self.manager = CameraManager([self.cam0, self.cam1])
        self.manager = CameraManager([self.cam0])
        self.manager.register_stream()
        self.manager.set_device()
        self.manager.init_pipeline()
        self.Vision0 = Vision("vision_target0")
        #self.Vision1 = Vision("vision_target1")
        self.detect = YoloV5Detect()
        self.cam0_id = '213622073349'
        #self.cam1_id = '108322072281'
        self.isCorrect = True
        if self.cam0_id != self.cam0.device_id:
            self.isCorrect = False
        print("newarmcam-0, oldarmcam-1", self.isCorrect)
        # self.boxes, self.scores, self.classes, self.images = None, None, None, None
        # self.aligned_depth_frame = None
        # self.color_frame = None
        # self.color_intrin_part = None
        # self.NoSystemSend = True
        
    def detect_once(self):
        boxes0, scores0, classes0, im0s_0 = self.detect.run(self.cam0.frame.color)
        #boxes1, scores1, classes1, im0s_1 = self.detect.run(self.cam1.frame.color)
        #return [boxes0, scores0, classes0, im0s_0], [boxes1, scores1, classes1, im0s_1]
        return [boxes0, scores0, classes0, im0s_0]
        
    def getWorldXYZ(self, object_classify):
        shape = [self.cam0.frame.color.shape[0], self.cam0.frame.color.shape[1]]
        #cam0DetectResult, cam1DetectResult = self.detect_once()
        cam0DetectResult = self.detect_once()
        boxes0 = np.array(cam0DetectResult[0], dtype=int)
        #boxes1 = np.array(cam1DetectResult[0], dtype=int)
        #print("0")
        
        target_xyz_trues0, track_cls0 = get_aim_(boxes0, 
                                                 cam0DetectResult[2], 
                                                 self.cam0.color_intrin_param, 
                                                 self.cam0.frame.raw_depth, 
                                                 shape, 
                                                 object_classify)
        #target_xyz_trues1, track_cls1 = get_aim_(boxes1, 
                                                 #cam1DetectResult[2], 
                                                 #self.cam1.color_intrin_param, 
                                                 #self.cam1.frame.raw_depth, 
                                                 #shape, 
                                                 #object_classify)       
        datas0, outputs0 = imagexyz2wordxyz(target_xyz_trues0, track_cls0,
                                          '/home/base00302/catkin_ws/src/VC/vision_identity/eyeInHand_calibration.yaml')  
        #datas1, outputs1 = imagexyz2wordxyz(target_xyz_trues1, track_cls1,
        #                                  '/home/base00302/catkin_ws/src/VC/vision_identity/eyeInHand_calibration.yaml')      
        
        is_eye_in_hand = True
        if is_eye_in_hand:
            try:
                #if self.isCorrect:
                #    object_pose0 = rospy.wait_for_message("/tool_point0", Pose, timeout=0.5)  
                    #object_pose1 = rospy.wait_for_message("/tool_point", Pose, timeout=0.5)
                #else:
                #    object_pose0 = rospy.wait_for_message("/tool_point", Pose, timeout=0.5)  
                    #object_pose1 = rospy.wait_for_message("/tool_point0", Pose, timeout=0.5)  
                object_pose0 = rospy.wait_for_message("/tool_point", Pose, timeout=0.5) 
                #print("object_pose0: ", object_pose0)
                #print("object_pose1: ", object_pose1)
                
                robot0CurrentPoseMatrix = self.set_arm_pose(object_pose0)
                #robot1CurrentPoseMatrix = self.set_arm_pose(object_pose1)
                
                matrix0 = robot0CurrentPoseMatrix.dot(outputs0)
                #matrix1 = robot1CurrentPoseMatrix.dot(outputs1)
                
                XYZ0 = [[datas0[0][0], matrix0[0, 3], matrix0[1, 3], matrix0[2, 3]]]
                #XYZ1 = [[datas1[0][0], matrix1[0, 3], matrix1[1, 3], matrix1[2, 3]]]
                print("Is eye in hand, target position: ", XYZ0)
                return XYZ0
                #if self.isCorrect:
                #    return XYZ0
                #else:
                #    return XYZ1
            except:
                return [[4, 0, 0, 0]]

        else:
            return datas0             
                
    def set_arm_pose(self, msg_arm):
        x = msg_arm.position.x
        y = msg_arm.position.y
        z = msg_arm.position.z
        Qx = msg_arm.orientation.x
        Qy = msg_arm.orientation.y
        Qz = msg_arm.orientation.z
        Qw = msg_arm.orientation.w
        Rq = [Qx, Qy, Qz, Qw]
        r = R.from_quat(Rq)
        euler_cur = r.as_euler('xyz', degrees=True)
        print('euler_cur: ', euler_cur)
        robotCurrentMatrix = r.as_matrix()
        print('robotCurrentMatrix : ', robotCurrentMatrix)
        robotCurrentMatrix = np.concatenate([robotCurrentMatrix, np.array([x, y, z]).reshape(3,1)], 1)
        robotCurrentPoseMatrix = np.concatenate([robotCurrentMatrix, np.array([0, 0, 0, 1]).reshape(1, 4)], 0)    
        return robotCurrentPoseMatrix                      
                          
        
    """
    def output(self, image, aligned_depth_frame, object_classify):
        shape = [image.shape[0], image.shape[1]]
        boxes, scores, classes, im0s = self.detect.run(image)
        #print(type(boxes))
        self.boxes, self.scores, self.classes, self.images, self.aligned_depth_frame = boxes, scores, classes,\
                                                                                       im0s, aligned_depth_frame
        
        boxes = np.asarray(self.boxes, dtype=int)
        #print("boxes",self.boxes)
        target_xyz_trues, track_cls = get_aim_(boxes, self.classes, self.color_intrin_part,
                                               self.aligned_depth_frame, shape, object_classify)
        print("target_xyz_trues:", target_xyz_trues)
        #datas, outputs = imagexyz2wordxyz(target_xyz_trues, track_cls,
        #                                  '/home/jetson/nky/catkin_ws/src/VC/vision_identity/eyeToHand_calibration.yaml')
        datas, outputs = imagexyz2wordxyz_eyeinhand(target_xyz_trues, track_cls,
                                          '/home/base00302/catkin_ws/src/VC/vision_identity/eyeToHand_calibration.yaml')
        is_eye_in_hand = rospy.get_param('IsEyeInHand')
        is_eye_in_hand = 1
        if is_eye_in_hand:
            try:
                object_pose = rospy.wait_for_message("/tool_point", Pose, timeout=0.5)
                x = object_pose.position.x
                y = object_pose.position.y
                z = object_pose.position.z
                Qx = object_pose.orientation.x
                Qy = object_pose.orientation.y
                Qz = object_pose.orientation.z
                Qw = object_pose.orientation.w
                Rq = [Qx, Qy, Qz, Qw]
                r = R.from_quat(Rq)
                euler_cur = r.as_euler('xyz', degrees=True)
                print('euler_cur: ', euler_cur)
                robotCurrentMatrix = r.as_matrix()
                print('robotCurrentMatrix : ', robotCurrentMatrix)
                robotCurrentMatrix = np.concatenate([robotCurrentMatrix, np.array([x, y, z]).reshape(3,1)], 1)
                robotCurrentPoseMatrix = np.concatenate([robotCurrentMatrix, np.array([0, 0, 0, 1]).reshape(1, 4)], 0)
                matrix = robotCurrentPoseMatrix.dot(outputs)
                print('eye2base : ', matrix)
                XYZ = [[datas[0][0], matrix[0, 3], matrix[1, 3], matrix[2, 3]]]
                print("Is eye in hand, target position: ", XYZ)
                return XYZ
            except:
                return [[4, 0, 0, 0]]

        else:
            return datas

    def show(self):
        cv2.imshow('image', self.images)
        k = cv2.waitKey(1)
        if k == 27:
            cv2.destroyAllWindows()
            self.cam.stop_pipline()"""
            
            
    def main(self):
        while True:
            try:
                vision_enable = rospy.get_param('vision_enable')
            except:
                vision_enable = False
            try:
                object_classify = rospy.get_param('object_classify')
            except:
                object_classify = 0
            self.manager.get_frame()
            data0 = self.getWorldXYZ(object_classify)
            #re = self.getWorldXYZ()
            #print(re[0], re[1])
            print(data0)
            if vision_enable:
                self.Vision0.msg_publish(data0[0])
                #self.Vision1.msg_publish(data1[0])
            cv2.imshow('img', self.cam0.frame.color)
            #cv2.imshow('img1', self.cam1.frame.color)
            k = cv2.waitKey(10)
            if k == 27:
                cv2.destroyAllWindows()
                for i in self.manager.device:
                    i.pipeline.stop()
                break
            
    """
    def main(self):
        while True:
            try:
                vision_enable = rospy.get_param('vision_enable')
            except:
                vision_enable = False
            try:
                object_classify = rospy.get_param('object_classify')
            except:
                object_classify = 0

            color_frame, aligned_depth_frame = self.cam.get_frame()
            self.color_intrin_part = self.cam.get_color_intrin_part(color_frame)
            image = np.asanyarray(color_frame.get_data())
            image[:,:400,:]=0
            data = self.output(image, aligned_depth_frame, object_classify)
            print(data[0])
            if vision_enable:
                self.Vision.msg_publish(data[0])
            self.show()"""


if __name__ == '__main__':
    #print('0: rect','\n','1: red','\n','2: blue')
    print('0: apple','\n','1: pear','\n','2: green apple', '\n', '3: orange')
    M = DataInference()
    M.main()






