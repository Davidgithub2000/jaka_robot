#!/usr/bin/env python
# -*- coding: utf_8 -*-

from __future__ import print_function
import rospy
import socket
import math
import pickle
import numpy as np
from std_msgs.msg import Int8
from g3p_msgs.msg import Gripper
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time
from realman_msgs.srv import *
from threading import Thread
import struct

class RobotVisionWrapper:
    def __init__(self):
        rospy.loginfo("======>START Robot Vision Wrapper, press [Ctrl+C]: exit<=====")
        rospy.on_shutdown(self.cleanUp)

        self.visionSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.visionSocket.settimeout(5)
        self.gripperSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.gripperSocket.settimeout(5)
        self.visionIsConnected = False
        self.gripperIsConnected = False
        self.isEyeInHand = False

        self.inputResponding = False
        self.cancelResponding = False
        self.robotStatus = True
        self.objectCompensationXYZ = None
        self.useInputRPY = None
        self.objectRotateRPY = None
        self.objectPrePoseOffsetXYZRPY = None
        self.autoPlace = None
        self.pickPrePose1Joint=None
        self.pickPrePose2Joint=None
        self.pickLatePose1Joint=None
        self.pickLatePose2Joint=None
        self.placeLatePose1Joint=None
        self.placeLatePose2Joint=None
        self.goalData=[0,0,0]
        self.jaka_joint=[0,0,0,0,0,0]
        self.jaka_pose=[0,0,0,0,0,0]
        rospy.set_param("/vision_enable",True)
        self.wrapperInputSub = rospy.Subscriber("/vision/input",Int8,self.inputCallback,queue_size=1)
        self.robotFeedbackSub = rospy.Subscriber("/robot_driver/robot_state",Bool,self.updateRobotStatus)
        self.robotFeedbackSubjoint = rospy.Subscriber("/robot_driver/joint_states",JointState,self.updateRobotStatusjoint)
        self.robotFeedbackSubpose = rospy.Subscriber("/robot_driver/endpose",Pose,self.updateRobotStatuspose)
        self.robotGripper3Pub = rospy.Publisher("/g3p_msgs/gripper",Gripper,queue_size=5)
        self.sub_goal = rospy.Subscriber("vision_target", Pose, self.goalCB)
        self.client = rospy.ServiceProxy("/robot_driver/move_line",Move)
        self.client_j = rospy.ServiceProxy("/robot_driver/move_joint",Move)
        


    def cleanUp(self):
        rospy.loginfo("==========>Robot Vision Wrapper STOPPED!<==========")
        # rospy.sleep(5)

    def initParam(self, isEyeInHand, objectCompensationXYZ, useInputRPY, objectRotateRPY, objectPrePoseOffsetXYZRPY, autoPlace,
                    pickPrePose1Joint, pickPrePose2Joint, pickLatePose1Joint, pickLatePose2Joint):
        self.isEyeInHand = isEyeInHand
        self.objectCompensationXYZ = tuple(map(float,objectCompensationXYZ.split(",")))
        self.useInputRPY = useInputRPY
        self.objectRotateRPY = tuple(map(float,objectRotateRPY.split(",")))
        self.objectPrePoseOffsetXYZRPY = tuple(map(float,objectPrePoseOffsetXYZRPY.split(",")))
        self.autoPlace = autoPlace
        self.pickPrePose1Joint = tuple(map(float,pickPrePose1Joint.split(",")))
        self.pickPrePose2Joint = tuple(map(float,pickPrePose2Joint.split(",")))
        #self.pickLatePose1Joint = tuple(map(float,pickLatePose1Joint.split(",")))
        self.pickLatePose1Joint = pickLatePose1Joint
        self.pickLatePose2Joint = tuple(map(float,pickLatePose2Joint.split(",")))
        self.placeLatePose1Joint = tuple(map(float,placeLatePose1Joint.split(",")))
        self.placeLatePose2Joint = tuple(map(float,placeLatePose2Joint.split(",")))
        
    def updateRobotStatusjoint(self, msg):
        self.jaka_joint[0] = msg.position[0]
        self.jaka_joint[1] = msg.position[1]
        self.jaka_joint[2] = msg.position[2]
        self.jaka_joint[3] = msg.position[3]
        self.jaka_joint[4] = msg.position[4]
        self.jaka_joint[5] = msg.position[5]

    def updateRobotStatuspose(self, msg):
        self.jaka_pose[0] = msg.position.x
        self.jaka_pose[1] = msg.position.y
        self.jaka_pose[2] = msg.position.z
        self.jaka_pose[3] = msg.orientation.x
        self.jaka_pose[4] = msg.orientation.y
        self.jaka_pose[5] = msg.orientation.z

    def CompareEndpose(self, msg):
        num = 0
        while True:
            if(abs(msg[0] - self.jaka_pose[0]) < 1  and abs(msg[1] - self.jaka_pose[1]) < 1 and abs(msg[2] - self.jaka_pose[2]) < 1 and abs(msg[3] - self.jaka_pose[3]) < 1 and abs(msg[4] - self.jaka_pose[4]) < 1 and abs(msg[5] - self.jaka_pose[5]) < 1):
                rospy.loginfo("endpose get.")
                break
            num = num + 1
            if num > 50:
                num = 0
                #self.robotPlace()
                break;
            rospy.sleep(0.1)

    def CompareJoint(self, joints):
        while True:
            #print("joint:%f\n",joints[5],self.jaka_joint[5],abs(joints[0] - self.jaka_joint[0]))
            if(abs(joints[0] - self.jaka_joint[0]) < 1  and abs(joints[1] - self.jaka_joint[1]) < 1 and abs(joints[2] - self.jaka_joint[2]) < 1 and abs(joints[3] - self.jaka_joint[3]) < 1 and abs(joints[4] - self.jaka_joint[4]) < 1 and abs(joints[5] - self.jaka_joint[5]) < 1):
                rospy.loginfo("joints get.")
                break
            rospy.sleep(0.1)

    def goalCB(self,msg):
        self.visionIsConnected = True
        self.goalData[0]=msg.position.x * 1000
        self.goalData[1]=msg.position.y * 1000
        self.goalData[2]=msg.position.z * 1000
        bfb = (self.goalData[0] + 130)/490 * 10
        self.goalData[2] = self.goalData[2] + bfb + (530 - self.goalData[1]) * 0.9
        self.goalData[1] = self.goalData[1] + bfb
        
        '''
        if self.goalData[0] > 0 or self.goalData[0] == 0:
            self.goalData[1]=msg.position.y * 1000 + self.goalData[0] * 0.3
            self.goalData[2]=msg.position.z * 1000 + self.goalData[0] * 0.3
        if self.goalData[0] < 0:
            self.goalData[1]=msg.position.y * 1000 + self.goalData[0] * 0.1
            self.goalData[2]=msg.position.z * 1000 + self.goalData[0] * 0.1
        '''
        #if self.goalData[0] > -500 and self.goalData[0] < 400 and self.goalData[1] > 500 and self.goalData[1] < 900 and self.goalData[2] < 1400 and self.goalData[2] > 900:
            #rospy.set_param("usr_task",6)
        '''if(self.goalData[1] > 300 and self.goalData[1] < 800):
            rospy.loginfo("usr_task -> 3 yes")
            rospy.set_param("usr_task",3)
        elif(self.goalData[1] == 0 or self.goalData[1] > 800):
            rospy.loginfo("usr_task -> 4 no")
            rospy.set_param("usr_task",4)'''
        

    def inputCallback(self, msg):
        # self.wrapperInputSub.unregister()
        # self.wrapperInputSub = rospy.Subscriber("/vision_rocr6/input",Int8,self.inputCallback,queue_size=1)

        if msg.data == 0:
            self.cancelResponding = True
            self.inputResponding = False
            self.robotJointMove(0, -1.57, 2.65, 0.5, 0, 1.57)
            rospy.sleep(1)
            while self.robotStatus != True and not rospy.is_shutdown():
                rospy.sleep(1)
            rospy.sleep(2)
            self.cancelResponding = False
            return
            
        if not self.inputResponding:
            self.inputResponding = True
            if msg.data == 99:
                self.robotPlace()
                rospy.set_param("/station_2/task",0)
            else:
                taskFinished = False
                while not taskFinished:
                    if self.visionIsConnected:
                        print("vision start...")
                        matrix = self.goalData
                        rospy.loginfo("data: %f,%f,%f", matrix[0], matrix[1],matrix[2])
                        if matrix[0] == 0 and matrix[1] == 0 and matrix[2] == 0:
                            rospy.logwarn("The target object is not detected !")
                            taskFinished = True
                            rospy.set_param("usr_task",4)
                            rospy.set_param("/station_1/task",0)
                            rospy.set_param("/station_2/task",1)
                        elif(matrix[1] < 400 or matrix[1] > 900 or matrix[0] < -140 or matrix[0] > 350 or matrix[2] < 850 ):
                            taskFinished = True
                            rospy.loginfo("位置不可到达")
                            rospy.set_param("usr_task",4)
                        else:
                            self.robotPick(matrix)
                            taskFinished = True
                            # rospy.set_param("/station_1/task",0)
                            # rospy.set_param("/station_2/task",1)
                            if self.autoPlace:
                                self.robotPlace()
                    else:
                        rospy.logfatal("Need to connect the VisionServer first, please try to start VisionServer !")
                        taskFinished = True
            self.inputResponding = False
        else:
            rospy.logwarn("Input is responding, please wait until the response is over !")

    def updateRobotStatus(self, msg):
        self.robotStatus = msg.data
        # if self.robotStatus :
        #     self.inputResponding = True

    def visionAction(self,commandString):
        self.visionSocket.sendall(commandString.encode("ascii"))
        recvData=self.visionSocket.recv(1024).decode("ascii")
        rospy.loginfo("tx:[%s]    rx:[%s]",commandString,recvData)
        return recvData

    def gripperAction(self,commandString):
        self.gripperSocket.sendall(commandString.encode("ascii"))
        recvData=self.gripperSocket.recv(1024).decode("ascii")
        rospy.loginfo("tx:[%s]    rx:[%s]",commandString,recvData)
        return recvData

    def startGripper_1(self):
        rospy.loginfo(self.gripperAction("start"))

    def startVision(self):
        rospy.loginfo(self.visionAction("start"))

    def stopVision(self):
        rospy.loginfo(self.visionAction("stop"))

    def robotGripperChangeState(self, state):
        self.robotGripperPub.publish(state)
        time.sleep(2)

    def robotGripper3ChangeState(self, state):
        gripper3State = Gripper()
        gripper3State.state = state
        self.robotGripper3Pub.publish(gripper3State)
        rospy.loginfo("gripper3ChangeState: %f",gripper3State.state)

    def visionObjectRequest(self):
        objectNo = "r," + self.pickLatePose1Joint
        rospy.loginfo("objectNo:[%s]", objectNo)
        self.visionSocket.sendall(objectNo.encode("ascii"))
        # test=pickle.dumps(matrix,protocol=0)
        rospy.sleep(5)
        matrix = self.visionSocket.recv(1024)
        mat = matrix.split(",")
        m_data = [0,0,0,0,0,0]
        if mat[0] == "1001":
            m_data[0] = float(mat[1])*1000
            m_data[1] = float(mat[2])*1000
            m_data[2] = float(mat[3])*1000
            m_data[3] = float(mat[4])*1000
            m_data[4] = float(mat[5])*1000
            m_data[5] = float(mat[6])*1000
        return m_data

    def robotJointMove(self, j1, j2, j3, j4, j5, j6):  
        pos = MoveRequest()
        pos.pose = [j1, j2, j3, j4, j5, j6]
        pos.mvvelo = 360.0
        self.client_j.wait_for_service()
        resp = self.client_j.call(pos)
        rospy.loginfo("robotJointMoveTo: %f,%f,%f,%f,%f,%f",j1, j2, j3, j4, j5, j6)

    def robotJointMoveL(self, valueList):
        while self.robotStatus != True and not rospy.is_shutdown():
            rospy.sleep(1)
        if self.robotStatus == True and valueList != (-1,-1,-1,-1,-1,-1) and not self.cancelResponding:
            self.robotJointMove(valueList[0], valueList[1], valueList[2], valueList[3], valueList[4], valueList[5])
            rospy.sleep(1)

    def robotCartesianMove(self, x, y, z, Rr, Rp, Ry): 
        pos = MoveRequest()
        pos.pose = [x, y, z, Rr, Rp, Ry]
        pos.mvvelo = 360.0
        self.client.wait_for_service()
        resp = self.client.call(pos)
        rospy.loginfo("robotCartesianMoveTo: %f,%f,%f,%f,%f,%f",x, y, z, Rr, Rp, Ry)

    def robotCartesianMoveL(self, valueList):
        while self.robotStatus != True and not rospy.is_shutdown():
            rospy.sleep(1)
        if self.robotStatus == True and valueList != (-1,-1,-1,-1,-1,-1) and not self.cancelResponding:
            self.robotCartesianMove(valueList[0], valueList[1], valueList[2], valueList[3], valueList[4], valueList[5])
            rospy.sleep(1)

    def robotMatrix2Pose(self, matrix):
        x = matrix[0] + self.objectCompensationXYZ[0]
        y = matrix[1] + self.objectCompensationXYZ[1]
        z = matrix[2] + self.objectCompensationXYZ[2]
        #Rr = 8.4
        #Rp = -18.8
        #Ry = -127.3
        Rr = 44.0
        Rp = -26.5
        Ry = -157.83
        return (x, y, z, Rr, Rp, Ry)

    def robotMatrix2PrePose(self, pose):
        return (pose[0]+self.objectPrePoseOffsetXYZRPY[0], pose[1]+self.objectPrePoseOffsetXYZRPY[1],
                pose[2]+self.objectPrePoseOffsetXYZRPY[2], pose[3]+self.objectPrePoseOffsetXYZRPY[3],
                pose[4]+self.objectPrePoseOffsetXYZRPY[4], pose[5]+self.objectPrePoseOffsetXYZRPY[5])

    def robotGripper3ChangeStateL(self, state):
        while self.robotStatus != True and not rospy.is_shutdown():
            rospy.sleep(1)
        if self.robotStatus == True and not self.cancelResponding:
            self.robotGripper3ChangeState(state)
            #rospy.sleep(3)

    def robotPick(self, matrix):
        pose = self.robotMatrix2Pose(matrix)
        #1
        #self.robotJointMoveL(self.placeLatePose1Joint)

	#2
        self.robotJointMoveL(self.pickPrePose2Joint)

        self.robotJointMoveL(self.pickPrePose1Joint)

	#3
        self.robotCartesianMoveL(self.robotMatrix2PrePose(pose))

	#4
        self.robotCartesianMoveL(pose)
        #self.CompareEndpose(pose)
        rospy.sleep(2)
        self.robotGripper3ChangeStateL(1)
        rospy.sleep(2)

        self.robotCartesianMoveL(self.robotMatrix2PrePose(pose))


        self.robotJointMoveL(self.pickPrePose1Joint)

        self.robotJointMoveL(self.pickPrePose2Joint)
        
        #self.robotJointMoveL(self.placeLatePose1Joint)

        self.robotJointMoveL(self.placeLatePose2Joint)
        #self.CompareJoint(self.placeLatePose2Joint)
        rospy.sleep(2)
        self.robotGripper3ChangeStateL(0)
        rospy.sleep(1.5)
        rospy.set_param("usr_task",4)
        

    def robotPlace(self):
        self.robotJointMoveL(self.pickPrePose1Joint)
        self.CompareJoint(self.pickPrePose1Joint)
        self.robotJointMoveL(self.pickPrePose2Joint)
        self.CompareJoint(self.pickPrePose2Joint)
        
        self.robotJointMoveL(self.placeLatePose1Joint)
        self.CompareJoint(self.placeLatePose1Joint)

        self.robotJointMoveL(self.placeLatePose2Joint)
        self.CompareJoint(self.placeLatePose2Joint)
        rospy.set_param("usr_task",4)
        #self.robotCartesianMoveL(self.placeLatePose2Joint)


def wrapperThread():
    inputPub = rospy.Publisher("/vision/input",Int8,queue_size=1)
    inputMsg = Int8()

    old_docked = 0
    while not rospy.is_shutdown():
        docked = rospy.get_param("/zeus_paths_multiple/docked", 0)
        #print("赋值前：oldDocked:{}, docked{}",old_docked,docked)
        if docked != old_docked:
            if old_docked == 0 and docked == 1:
                task = rospy.get_param("/station_1/task", 0)
                # print("station 1 task:{}",task)
                if task > 0:
                    inputMsg.data = task
                    inputPub.publish(inputMsg)
                    # print("station 1 pub:{}",inputMsg.data)
            if old_docked ==0 and docked == 2:
                task = rospy.get_param("/station_2/task", 0)
                # print("station 2 task:{}",task)
                if task == 1:
                    inputMsg.data = 99
                    inputPub.publish(inputMsg)
                    # print("station 2 pub:{}",inputMsg.data)
            old_docked = docked
            # print("赋值后：oldDocked:{}, docked{}",old_docked,docked)
            rospy.sleep(1)
        else:
            pass


if __name__ == "__main__":
    rospy.init_node("robot_vision_wrapper")

    visionServerIP = rospy.get_param("robot_vision_wrapper/vision_server_ip")
    visionServerPort = rospy.get_param("robot_vision_wrapper/vision_server_port")
    gripperServerIP = rospy.get_param("robot_vision_wrapper/gripper_server_ip")
    gripperServerPort = rospy.get_param("robot_vision_wrapper/gripper_server_port")
    isEyeInHand = rospy.get_param("robot_vision_wrapper/isEyeInHand")
    objectCompensationXYZ = rospy.get_param("robot_vision_wrapper/object_compensation_XYZ")
    useInputRPY = rospy.get_param("robot_vision_wrapper/use_input_RPY")
    objectRotateRPY = rospy.get_param("robot_vision_wrapper/object_rotate_RPY")
    objectPrePoseOffsetXYZRPY = rospy.get_param("robot_vision_wrapper/object_pre_pose_offset_XYZRPY")
    autoPlace = rospy.get_param("robot_vision_wrapper/auto_place")
    pickPrePose1Joint = rospy.get_param("robot_vision_wrapper/pick_pre_pose_1_joint")
    pickPrePose2Joint = rospy.get_param("robot_vision_wrapper/pick_pre_pose_2_joint")
    pickLatePose1Joint = rospy.get_param("robot_vision_wrapper/pick_late_pose_1_joint")
    pickLatePose2Joint = rospy.get_param("robot_vision_wrapper/pick_late_pose_2_joint")
    placeLatePose1Joint = rospy.get_param("robot_vision_wrapper/place_late_pose_1_joint")
    placeLatePose2Joint = rospy.get_param("robot_vision_wrapper/place_late_pose_2_joint")

    wrapper = RobotVisionWrapper()
    wrapper.initParam(isEyeInHand, objectCompensationXYZ, useInputRPY, objectRotateRPY, objectPrePoseOffsetXYZRPY, autoPlace,
                    pickPrePose1Joint, pickPrePose2Joint, pickLatePose1Joint, pickLatePose2Joint)

    rospy.sleep(10)

    thread = Thread(target=wrapperThread)
    thread.start()

    rospy.spin()
