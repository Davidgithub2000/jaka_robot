#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_vision_wrapper_main.py

唯一主程序入口。

运行方式示例：
    rosrun <your_package> robot_vision_wrapper_main.py

拆分后的模块：
1. robot_vision_config.py      参数、单位、常量；
2. robot_motion_control.py     IK、运动服务、ready 等待、运动失败检查；
3. robot_vision_auto.py        视觉目标、自动采摘状态机、抓取流程；
4. robot_vision_wrapper_main.py 当前文件，负责 ROS 初始化和入口。
"""

import os
import socket
import sys

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rospy
from geometry_msgs.msg import Pose, TwistStamped
from jaka_msgs.msg import RobotMsg
from realman_msgs.srv import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, Int8, String

from robot_vision_config import LEFT_SIDE, RIGHT_SIDE
from robot_vision_config import RobotVisionConfigMixin
from robot_motion_control import RobotMotionMixin
from robot_vision_auto import RobotVisionAutoMixin


class RobotVisionWrapper(RobotVisionConfigMixin, RobotMotionMixin, RobotVisionAutoMixin):
    def __init__(self):
        """初始化机器人视觉包装器。"""
        rospy.loginfo("======>START Robot Vision Wrapper, press [Ctrl+C]: exit<=====")
        rospy.on_shutdown(self.cleanUp)

        self._init_parameters()
        self._init_network()
        self._init_ros_communication()
        self._init_data_recording()

        # 新版采用消息触发：/custom_arm_data 每个目标只发布一次，收到后即入队处理。
        # use_usr_task_gate=False 时，不再受 /usr_task==4 影响；如现场仍需任务门控，可在参数中打开。
        initial_task = self._get_usr_task()
        initial_enable = self._auto_pick_task_enabled()
        self.vision_accepting = True
        rospy.set_param(self.vision_enable_param, True)
        self.auto_pick_enabled_last = initial_enable
        rospy.loginfo(
            "初始 %s=%s，use_usr_task_gate=%s，消息触发采摘%s",
            self.task_param_name,
            initial_task,
            self.use_usr_task_gate,
            "开启" if initial_enable else "暂停",
        )

    def _init_network(self):
        """初始化网络连接对象，保留原变量，当前程序不主动连接。"""
        self.vision_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.vision_socket.settimeout(5)
        self.gripper_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.gripper_socket.settimeout(5)

    def _init_ros_communication(self):
        """初始化 ROS 订阅、发布和服务。"""
        # 保留 /vision/input：用于人工切换 87/88/89；自动采摘不再依赖该话题触发。
        self.wrapper_input_sub = rospy.Subscriber(
            self.wrapper_input_topic, Int8, self.inputCallback, queue_size=1
        )
        self.robot_state_sub = rospy.Subscriber(
            self.robot_states_topic, RobotMsg, self.updateRobotStatus, queue_size=1
        )
        self.joint_state_sub = rospy.Subscriber(
            self.joint_state_topic, JointState, self.updateJointStatus, queue_size=1
        )
        # 订阅 JAKA 驱动反馈的当前 TCP 位姿，用于将视觉 TCP 坐标转换到机械臂基坐标系。
        self.tool_position_sub = rospy.Subscriber(
            self.tool_position_topic,
            TwistStamped,
            self.updateToolPosition,
            queue_size=1,
        )
        # 总控节点已经完成视觉合理性判断，每个目标只发布一次。
        # 本节点订阅 Float32MultiArray 类型 /custom_arm_data，收到后立即入队并触发采摘。
        # data 格式默认：[p1x, p1y, p1z, p2x, p2y, p2z]，两个点均为当前 TCP 坐标系下坐标。
        self.vision_target_sub = rospy.Subscriber(
            self.custom_arm_data_topic, Float32MultiArray, self.goalCB, queue_size=10
        )
        self.pick_result_topic = rospy.get_param(
            "robot_vision_wrapper/pick_result_topic", "target_result"
        )
        self.pick_result_pub = rospy.Publisher(
            self.pick_result_topic, String, queue_size=20
        )

        self._init_gripper_feedback_subscriber()

        #self.gripper_pub = rospy.Publisher("/gripper/grip_cmd", Bool, queue_size=5)
        #self.robotGripperPub = self.gripper_pub

        self.move_line_client = rospy.ServiceProxy(self.linear_move_service, Move)
        self.move_line_tol_client = rospy.ServiceProxy(self.linear_move_tol_service, Move)
        self.move_joint_client = rospy.ServiceProxy(self.joint_move_service, Move)
        self.move_joint_tol_client = rospy.ServiceProxy(self.joint_move_tol_service, Move)
        self.ik_client = rospy.ServiceProxy(self.ik_service, GetIK)
        self.fk_client = rospy.ServiceProxy(self.fk_service, GetFK)

    def cleanUp(self):
        """节点退出清理。"""
        rospy.loginfo("==========>Robot Vision Wrapper STOPPED!<==========")
        for sock in (getattr(self, "vision_socket", None), getattr(self, "gripper_socket", None)):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass


# -----------------------------------------------------------------------------
# 保留原 wrapperThread 函数。
# 自动采摘模式下 main 不启动它，避免重复触发。
# -----------------------------------------------------------------------------
def wrapperThread():
    inputPub = rospy.Publisher("/vision/input", Int8, queue_size=1)
    inputMsg = Int8()

    old_docked = 0
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        docked = rospy.get_param("/zeus_paths_multiple/docked", 0)

        if docked != old_docked:
            if old_docked == 0 and docked == 1:
                task = rospy.get_param("/station_1/task", 0)
                if task > 0:
                    inputMsg.data = int(task)
                    inputPub.publish(inputMsg)

            if old_docked == 0 and docked == 2:
                task = rospy.get_param("/station_2/task", 0)
                if task == 1:
                    inputMsg.data = 99
                    inputPub.publish(inputMsg)

            old_docked = docked
            rospy.sleep(1)

        rate.sleep()


def _get_param(name, default=None):
    """读取 ROS 参数；default 为 None 时保持原行为，缺参直接抛异常。"""
    if default is None:
        return rospy.get_param(name)
    return rospy.get_param(name, default)


def main():
    """ROS 节点唯一入口。"""
    rospy.init_node("robot_vision_wrapper")
    print("init_node success")

    # 保留原变量读取；网络参数当前未主动使用，但不删除，方便兼容已有 launch/yaml。
    visionServerIP = _get_param("robot_vision_wrapper/vision_server_ip", "")
    visionServerPort = _get_param("robot_vision_wrapper/vision_server_port", 0)
    gripperServerIP = _get_param("robot_vision_wrapper/gripper_server_ip", "")
    gripperServerPort = _get_param("robot_vision_wrapper/gripper_server_port", 0)

    isEyeInHand = _get_param("robot_vision_wrapper/isEyeInHand")
    objectCompensationXYZ = _get_param("robot_vision_wrapper/object_compensation_XYZ")
    useInputRPY = _get_param("robot_vision_wrapper/use_input_RPY")
    objectRotateRPY = _get_param("robot_vision_wrapper/object_rotate_RPY")
    objectPrePoseOffsetXYZRPY = _get_param("robot_vision_wrapper/object_pre_pose_offset_XYZRPY")
    autoPlace = _get_param("robot_vision_wrapper/auto_place")
    refJointLeft = _get_param("robot_vision_wrapper/ref_joint_left")
    refJointRight = _get_param("robot_vision_wrapper/ref_joint_right")
    placeJointRight = _get_param("robot_vision_wrapper/place_joint_right")
    placeJointLeft = _get_param("robot_vision_wrapper/place_joint_left")
    initJointPosRight = _get_param("robot_vision_wrapper/init_joint_pos_right")
    initJointPosLeft = _get_param("robot_vision_wrapper/init_joint_pos_left")
    midJointTrans = _get_param("robot_vision_wrapper/mid_joint_trans")


    rospy.loginfo(
        "Configured vision server %s:%s, gripper server %s:%s",
        visionServerIP,
        visionServerPort,
        gripperServerIP,
        gripperServerPort,
    )

    wrapper = RobotVisionWrapper()
    wrapper.initParam(
        isEyeInHand,
        objectCompensationXYZ,
        useInputRPY,
        objectRotateRPY,
        objectPrePoseOffsetXYZRPY,
        autoPlace,
        refJointLeft,
        refJointRight,
        placeJointRight,
        placeJointLeft,
        initJointPosRight,
        initJointPosLeft,
        midJointTrans,
    )

    # 等待 ROS 订阅和服务代理建立。此处不是机械臂运动到位等待。
    rospy.sleep(2)

    # 自动采摘模式：
    # 直接订阅 /vision_target0；
    # 不启动 wrapperThread；
    # 不依赖 /vision/input 触发采摘；
    # 机械臂运动等待全部采用 /jaka_driver/robot_states 状态判断；
    # 自动采摘流程由 /usr_task 控制，/usr_task != 4 时开启。
    wrapper.start_auto_pick()

    rospy.spin()


if __name__ == "__main__":
    main()
