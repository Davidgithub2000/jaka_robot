#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_motion_control.py

机械臂运动与 robot_states 状态等待模块。

本模块包含：
1. 左/右 IK 参考关节区分；
2. 所有 Move 服务调用检查返回值；
3. 关节运动统一使用 rad；
4. _wait_robot_ready() 默认带超时，避免永久阻塞；
5. 机械臂运动完成不再采用固定 sleep，而是根据 /jaka_driver/robot_states 判断：
   - motion_state: Stop=0, Pause=1, EmeStop=2, Running=3, Error=4；
   - power_state: 上电=1；
   - servo_state: 伺服使能=1；
   - collision_state: 碰撞报警=1；
6. 多臂夹爪控制改为发布 z_efg_ros/GripperCmd：
   - 话题默认 /multi_gripper/cmd；
   - JAKA 机械臂对应手爪 id 默认 1；
   - grip: 0 打开，1 闭合。

说明：
- 运动指令下发后，程序等待 robot_states 中出现指令之后的新状态；
- 若检测到 Running=3，则继续等待 Stop=0；
- 若运动很短没有捕获到 Running，也允许在指令之后连续收到若干帧 Stop=0 后判定完成；
- 若出现 EmeStop、Error、未上电、伺服未使能、碰撞报警或状态超时，则立即返回失败。
"""

import csv
import math
import os
import time
from threading import Lock

import rospy
from realman_msgs.srv import *

try:
    from z_efg_ros.msg import GripperCmd
except ImportError:
    GripperCmd = None

try:
    from z_efg_ros.msg import InterpretedState
except ImportError:
    InterpretedState = None

from robot_vision_config import INVALID_6D_VALUE


MOTION_STOP = 0
MOTION_PAUSE = 1
MOTION_EME_STOP = 2
MOTION_RUNNING = 3
MOTION_ERROR = 4


class RobotMotionMixin:
    def _init_data_recording(self):
        """初始化关节数据记录功能。"""
        self.record_dir = os.path.expanduser("~/robot_joint_records")
        try:
            os.makedirs(self.record_dir, exist_ok=True)
            rospy.loginfo("Joint records directory: %s", self.record_dir)
        except OSError as exc:
            rospy.logerr("Failed to create directory: %s", str(exc))
            self.record_dir = os.getcwd()

        self.recording_enabled = True
        self.joint_records = []
        self.record_lock = Lock()
        self.record_start_time = 0.0
        self.current_operation = ""

        rospy.loginfo(
            "Joint data recording initialized. Records will be saved to: %s",
            self.record_dir,
        )

    def _start_recording(self, operation_name):
        """开始记录关节数据。"""
        if not self.recording_enabled:
            return

        self.current_operation = str(operation_name)
        self.record_start_time = time.time()
        with self.record_lock:
            self.joint_records = []

        rospy.loginfo("Started recording joint data for operation: %s", operation_name)

    def _stop_and_save_recording(self, filename_prefix):
        """停止并保存关节数据。"""
        if not self.recording_enabled or self.record_start_time == 0:
            return

        self.record_start_time = 0.0
        with self.record_lock:
            records = list(self.joint_records)
            self.joint_records = []

        if not records:
            rospy.logwarn("No joint records to save for operation: %s", self.current_operation)
            return

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = "{}_{}_{}.csv".format(filename_prefix, self.current_operation, timestamp)
        filepath = os.path.join(self.record_dir, filename)

        try:
            with open(filepath, "w", newline="", encoding="utf-8") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow([
                    "time(s)", "operation",
                    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
                ])
                for record in records:
                    joints = list(record.get("joints", []))[:6]
                    if len(joints) < 6:
                        continue
                    writer.writerow([
                        "{:.3f}".format(float(record.get("time", 0.0))),
                        record.get("operation", ""),
                        *["{:.6f}".format(float(j)) for j in joints],
                    ])
            rospy.loginfo("Saved %d joint records to %s", len(records), filepath)
        except (OSError, ValueError) as exc:
            rospy.logerr("Failed to save joint records: %s", str(exc))

    # ------------------------------------------------------------------
    # robot_states 状态处理
    # ------------------------------------------------------------------
    def _motion_state_name(self, motion_state):
        names = {
            MOTION_STOP: "Stop",
            MOTION_PAUSE: "Pause",
            MOTION_EME_STOP: "EmeStop",
            MOTION_RUNNING: "Running",
            MOTION_ERROR: "Error",
        }
        return names.get(int(motion_state), "Unknown({})".format(motion_state))

    def updateRobotStatus(self, msg):
        """
        更新 /jaka_driver/robot_states 状态。

        robot_states 字段定义：
        - motion_state: Stop=0, Pause=1, EmeStop=2, Running=3, Error=4；
        - power_state: 上电=1，未上电=0；
        - servo_state: 伺服使能=1，未使能=0；
        - collision_state: 碰撞报警=1，未碰撞=0。
        """
        now = time.time()
        lock = getattr(self, "robot_state_lock", None)
        if lock is None:
            self.motion_state = int(getattr(msg, "motion_state", MOTION_ERROR))
            self.power_state = int(getattr(msg, "power_state", 0))
            self.servo_state = int(getattr(msg, "servo_state", 0))
            self.collision_state = int(getattr(msg, "collision_state", 0))
            self._motion_state_stamp = now
            self.robot_state_received = True
            self.robot_status = self._robot_ready_to_accept_motion_unlocked()
            return

        with lock:
            self.motion_state = int(getattr(msg, "motion_state", MOTION_ERROR))
            self.power_state = int(getattr(msg, "power_state", 0))
            self.servo_state = int(getattr(msg, "servo_state", 0))
            self.collision_state = int(getattr(msg, "collision_state", 0))
            self._motion_state_stamp = now
            self.robot_state_received = True
            self.robot_status = self._robot_ready_to_accept_motion_unlocked()

    def _get_robot_state_snapshot(self):
        """读取 robot_states 快照。"""
        lock = getattr(self, "robot_state_lock", None)
        if lock is None:
            return {
                "received": bool(getattr(self, "robot_state_received", False)),
                "stamp": float(getattr(self, "_motion_state_stamp", 0.0)),
                "motion_state": int(getattr(self, "motion_state", MOTION_ERROR)),
                "power_state": int(getattr(self, "power_state", 0)),
                "servo_state": int(getattr(self, "servo_state", 0)),
                "collision_state": int(getattr(self, "collision_state", 0)),
            }

        with lock:
            return {
                "received": bool(getattr(self, "robot_state_received", False)),
                "stamp": float(getattr(self, "_motion_state_stamp", 0.0)),
                "motion_state": int(getattr(self, "motion_state", MOTION_ERROR)),
                "power_state": int(getattr(self, "power_state", 0)),
                "servo_state": int(getattr(self, "servo_state", 0)),
                "collision_state": int(getattr(self, "collision_state", 0)),
            }

    def _robot_ready_to_accept_motion_unlocked(self):
        """在 robot_state_lock 内部使用：判断是否可以接收新的运动指令。"""
        return (
            bool(getattr(self, "robot_state_received", False))
            and int(getattr(self, "motion_state", MOTION_ERROR)) == MOTION_STOP
            and int(getattr(self, "power_state", 0)) == 1
            and int(getattr(self, "servo_state", 0)) == 1
            and int(getattr(self, "collision_state", 0)) == 0
            and not rospy.is_shutdown()
        )

    def _robot_state_fresh(self, state):
        if not state["received"]:
            return False
        max_age = float(getattr(self, "robot_state_timeout_s", 1.5))
        return (time.time() - state["stamp"]) <= max_age

    def _robot_fault_reason(self, state):
        """返回 None 表示没有故障；返回字符串表示当前状态禁止继续运动。"""
        if not state["received"]:
            return "尚未收到 /jaka_driver/robot_states"
        if not self._robot_state_fresh(state):
            return "robot_states 超时 {:.2f}s".format(time.time() - state["stamp"])
        if state["power_state"] != 1:
            return "机器人未上电 power_state={}".format(state["power_state"])
        if state["servo_state"] != 1:
            return "伺服未使能 servo_state={}".format(state["servo_state"])
        if state["collision_state"] == 1:
            return "碰撞报警 collision_state=1"
        if state["motion_state"] == MOTION_EME_STOP:
            return "机器人急停 motion_state=EmeStop"
        if state["motion_state"] == MOTION_ERROR:
            return "机器人错误 motion_state=Error"
        if state["motion_state"] not in (MOTION_STOP, MOTION_PAUSE, MOTION_RUNNING):
            return "未知运动状态 motion_state={}".format(state["motion_state"])
        return None

    def _robot_ready_reason(self, state):
        """返回 None 表示可接收运动指令，否则返回不可运动原因。"""
        fault = self._robot_fault_reason(state)
        if fault is not None:
            return fault
        if state["motion_state"] != MOTION_STOP:
            return "机器人未停止，motion_state={}".format(self._motion_state_name(state["motion_state"]))
        return None

    def _wait_until(self, cond_fn, timeout_s, sleep_s=0.02):
        """通用轮询等待。只用于等待状态变化，不再用于固定运动时长等待。"""
        start = time.time()
        timeout_s = float(timeout_s)
        while not rospy.is_shutdown() and (time.time() - start) < timeout_s:
            try:
                if cond_fn():
                    return True
            except Exception as exc:
                rospy.logwarn("wait condition exception: %s", str(exc))
                return False
            rospy.sleep(sleep_s)
        return False

    def _sleep_with_shutdown_check(self, sleep_s, log_text=""):
        """
        仅保留给夹爪发布等待、主循环节拍等没有机器人状态反馈的场景。
        机械臂运动完成判断不得再调用此函数。
        """
        sleep_s = max(0.0, float(sleep_s))

        if sleep_s <= 0.0:
            return not rospy.is_shutdown()

        if log_text:
            rospy.loginfo("%s，等待 %.2f s", log_text, sleep_s)

        start = time.time()
        while not rospy.is_shutdown() and (time.time() - start) < sleep_s:
            rospy.sleep(0.05)

        return not rospy.is_shutdown()

    def _wait_robot_ready(self, timeout_s=None):
        """
        等待机械臂处于可接收指令状态，并带超时保护。

        可接收指令条件：
        - 已收到新鲜 robot_states；
        - power_state == 1；
        - servo_state == 1；
        - collision_state == 0；
        - motion_state == Stop(0)。
        """
        if timeout_s is None:
            timeout_s = self.robot_ready_timeout_s

        timeout_s = float(timeout_s)
        start = time.time()
        last_reason = ""

        while not rospy.is_shutdown() and (time.time() - start) < timeout_s:
            state = self._get_robot_state_snapshot()
            reason = self._robot_ready_reason(state)
            if reason is None:
                self.robot_status = True
                return True
            last_reason = reason
            rospy.loginfo_throttle(2.0, "waiting robot ready: %s", reason)
            rospy.sleep(float(getattr(self, "motion_state_poll_s", 0.02)))

        self.robot_status = False
        rospy.logerr("等待机器人 ready 超时 %.2f s：%s", timeout_s, last_reason)
        return False

    def _wait_stop_state_confirmed(self, timeout_s=None, after_stamp=0.0, log_name="机械臂停止确认"):
        """
        等待 robot_states 连续若干帧处于 Stop。

        after_stamp > 0 时，只统计该时间之后到达的 robot_states，避免把运动指令之前的旧 Stop
        当成本次运动完成。
        """
        if timeout_s is None:
            timeout_s = self.motion_done_timeout_s

        timeout_s = float(timeout_s)
        confirm_count_target = max(1, int(getattr(self, "motion_stop_confirm_count", 3)))
        poll_s = float(getattr(self, "motion_state_poll_s", 0.02))
        start = time.time()
        stop_count = 0
        last_counted_stamp = 0.0
        last_reason = ""

        while not rospy.is_shutdown() and (time.time() - start) < timeout_s:
            state = self._get_robot_state_snapshot()
            fault = self._robot_fault_reason(state)
            if fault is not None:
                # 没收到状态或状态短暂超时可以继续等；明确故障立即失败。
                if state["received"] and self._robot_state_fresh(state):
                    rospy.logerr("%s失败：%s", log_name, fault)
                    return False
                last_reason = fault
                rospy.loginfo_throttle(2.0, "%s等待中：%s", log_name, fault)
                rospy.sleep(poll_s)
                continue

            if state["stamp"] <= after_stamp:
                last_reason = "等待运动指令之后的新 robot_states"
                rospy.sleep(poll_s)
                continue

            motion_state = state["motion_state"]
            if motion_state == MOTION_STOP:
                if state["stamp"] != last_counted_stamp:
                    stop_count += 1
                    last_counted_stamp = state["stamp"]
                if stop_count >= confirm_count_target:
                    self.last_side_stable_time = time.time()
                    rospy.loginfo(
                        "%s完成：连续 %d 帧 Stop，motion_state=%s, power=%s, servo=%s, collision=%s",
                        log_name,
                        stop_count,
                        self._motion_state_name(motion_state),
                        state["power_state"],
                        state["servo_state"],
                        state["collision_state"],
                    )
                    return True
            elif motion_state == MOTION_RUNNING:
                stop_count = 0
                last_reason = "机器人仍在运行 Running"
            elif motion_state == MOTION_PAUSE:
                stop_count = 0
                last_reason = "机器人暂停 Pause"
            else:
                rospy.logerr("%s失败：异常 motion_state=%s", log_name, self._motion_state_name(motion_state))
                return False

            rospy.loginfo_throttle(2.0, "%s等待中：%s", log_name, last_reason)
            rospy.sleep(poll_s)

        rospy.logerr("%s超时 %.2f s：%s", log_name, timeout_s, last_reason)
        return False

    def _wait_motion_complete_after_command(self, command_stamp, timeout_s=None, log_name="机械臂运动"):
        """
        根据 robot_states 等待一次运动指令完成。

        判定规则：
        1. 只接受 command_stamp 之后到达的 robot_states；
        2. 优先等待 Running=3，然后等待 Stop=0；
        3. 若运动很短没有捕获 Running，则在 motion_start_grace_s 之后，
           连续若干帧 Stop=0 也判定完成；
        4. EmeStop/Error/未上电/未使能/碰撞立即失败。
        """
        if command_stamp is None or command_stamp <= 0:
            command_stamp = time.time()

        if timeout_s is None:
            timeout_s = self.motion_done_timeout_s

        timeout_s = float(timeout_s)
        confirm_count_target = max(1, int(getattr(self, "motion_stop_confirm_count", 3)))
        poll_s = float(getattr(self, "motion_state_poll_s", 0.02))
        start_grace_s = max(0.0, float(getattr(self, "motion_start_grace_s", 0.3)))

        start = time.time()
        saw_running = False
        stop_count = 0
        last_counted_stamp = 0.0
        last_reason = "等待运动指令之后的新 robot_states"

        rospy.loginfo("%s：开始等待 robot_states 判定运动完成", log_name)

        while not rospy.is_shutdown() and (time.time() - start) < timeout_s:
            state = self._get_robot_state_snapshot()
            fault = self._robot_fault_reason(state)
            if fault is not None:
                if state["received"] and self._robot_state_fresh(state):
                    rospy.logerr("%s失败：%s", log_name, fault)
                    return False
                last_reason = fault
                rospy.loginfo_throttle(2.0, "%s等待中：%s", log_name, fault)
                rospy.sleep(poll_s)
                continue

            if state["stamp"] <= command_stamp:
                last_reason = "等待运动指令之后的新 robot_states"
                rospy.sleep(poll_s)
                continue

            motion_state = state["motion_state"]
            if motion_state == MOTION_RUNNING:
                saw_running = True
                stop_count = 0
                last_reason = "已检测到 Running，等待回到 Stop"
            elif motion_state == MOTION_STOP:
                # 若还没有捕获 Running，先给驱动状态切换留出短暂窗口，
                # 避免在服务刚返回、运动尚未置 Running 前误判完成。
                if not saw_running and (time.time() - command_stamp) < start_grace_s:
                    last_reason = "尚未检测到 Running，等待启动宽限窗口结束"
                    rospy.sleep(poll_s)
                    continue

                if state["stamp"] != last_counted_stamp:
                    stop_count += 1
                    last_counted_stamp = state["stamp"]

                if stop_count >= confirm_count_target:
                    self.last_side_stable_time = time.time()
                    rospy.loginfo(
                        "%s完成：saw_running=%s，连续 %d 帧 Stop",
                        log_name,
                        saw_running,
                        stop_count,
                    )
                    return True
                last_reason = "等待 Stop 连续确认 {}/{}".format(stop_count, confirm_count_target)
            elif motion_state == MOTION_PAUSE:
                stop_count = 0
                last_reason = "机器人暂停 Pause"
            else:
                rospy.logerr("%s失败：异常 motion_state=%s", log_name, self._motion_state_name(motion_state))
                return False

            rospy.loginfo_throttle(2.0, "%s等待中：%s", log_name, last_reason)
            rospy.sleep(poll_s)

        rospy.logerr("%s超时 %.2f s：%s", log_name, timeout_s, last_reason)
        return False

    def wait_arm_motion_stable(self, hold_s=None, timeout_s=None, command_stamp=None, log_name="机械臂状态稳定"):
        """
        使用 robot_states 判断机械臂是否已经停止，不再使用固定 sleep。

        参数 hold_s 仅为兼容旧调用保留，不再代表固定等待时间。
        """
        if timeout_s is None:
            timeout_s = self.motion_done_timeout_s

        if command_stamp is not None:
            return self._wait_motion_complete_after_command(
                command_stamp=command_stamp,
                timeout_s=timeout_s,
                log_name=log_name,
            )

        return self._wait_stop_state_confirmed(
            timeout_s=timeout_s,
            after_stamp=0.0,
            log_name=log_name,
        )

    def wait_joint_reached(self, target_joint_rad, tol_rad=0.1, timeout_s=3.0):
        """
        保留旧函数名。
        当前流程使用 robot_states 判断运动完成，不使用 joint_states 做到位判断。
        """
        rospy.logwarn("wait_joint_reached is not used; motion completion is judged by robot_states")
        return self.wait_arm_motion_stable(timeout_s=timeout_s, log_name="wait_joint_reached")

    def wait_tcp_reached(self, target_pose_mm_rad, pos_tol_mm=5.0, ang_tol_rad=0.1, timeout_s=5.0):
        """
        保留旧函数名。
        当前流程使用 robot_states 判断运动完成，不使用 TCP 位姿做到位判断。
        """
        rospy.logwarn("wait_tcp_reached is not used; motion completion is judged by robot_states")
        return self.wait_arm_motion_stable(timeout_s=timeout_s, log_name="wait_tcp_reached")

    def degJoint2radJoint(self, deg_joint_list):
        return [math.radians(float(deg_joint)) for deg_joint in deg_joint_list]

    def radJoint2degJoint(self, rad_joint_list):
        return [math.degrees(float(rad_joint)) for rad_joint in rad_joint_list]

    def updateJointStatus(self, msg):
        """更新关节状态。记录用，不用于本流程的运动完成判断。"""
        self.joint_positions = list(msg.position[:6])
        self.joint_state_stamp = time.time()

        if self.recording_enabled and self.record_start_time > 0:
            with self.record_lock:
                self.joint_records.append({
                    "time": time.time() - self.record_start_time,
                    "joints": self.joint_positions[:],
                    "operation": self.current_operation,
                })

    def _get_joint_state_snapshot(self):
        return (
            list(getattr(self, "joint_positions", [0.0] * 6)[:6]),
            float(getattr(self, "joint_state_stamp", 0.0)),
        )

    def joints_close_to(self, target_joints, tolerance_rad=None):
        if target_joints is None or len(target_joints) < 6:
            return False, "invalid target joints"

        current, stamp = self._get_joint_state_snapshot()
        if len(current) < 6:
            return False, "invalid current joints"

        max_age = float(getattr(self, "joint_state_max_age_s", 1.0))
        if stamp <= 0.0:
            return False, "no joint_state received"
        age = time.time() - stamp
        if age > max_age:
            return False, "joint_state stale {:.3f}s > {:.3f}s".format(age, max_age)

        tol = float(
            tolerance_rad
            if tolerance_rad is not None
            else getattr(self, "arm_safe_joint_tolerance_rad", 0.08)
        )
        diffs = [abs(float(current[i]) - float(target_joints[i])) for i in range(6)]
        max_diff = max(diffs)
        if max_diff > tol:
            return False, "joint diff {:.6f}rad > {:.6f}rad, diffs={}".format(
                max_diff,
                tol,
                ["{:.6f}".format(v) for v in diffs],
            )

        return True, "joint diff {:.6f}rad <= {:.6f}rad".format(max_diff, tol)

    def updateTcpPose(self, msg):
        """
        更新末端工具位姿。记录用，不用于本流程的运动完成判断。
        当前驱动中的 endpose.position 按 mm 反馈，orientation.x/y/z 按角度反馈，
        内部统一保存为 m + rad。
        """
        self.end_effector_pose = [
            msg.position.x / 1000.0,
            msg.position.y / 1000.0,
            msg.position.z / 1000.0,
            math.radians(msg.orientation.x),
            math.radians(msg.orientation.y),
            math.radians(msg.orientation.z),
        ]

    def updateToolPosition(self, msg):
        """
        更新 /jaka_driver/tool_position 当前 TCP 位姿。

        订阅消息类型：geometry_msgs/TwistStamped

        驱动实际单位：
            twist.linear  : mm
            twist.angular : degree

        本函数内部统一保存为：
            位置 m，姿态 rad
            [x_m, y_m, z_m, rx_rad, ry_rad, rz_rad]
        """
        try:
            linear = msg.twist.linear
            angular = msg.twist.angular

            # -----------------------------
            # 1. 位置统一转为 m
            # -----------------------------
            pos = [float(linear.x), float(linear.y), float(linear.z)]
            pos_unit = str(getattr(self, "tool_position_pos_unit", "mm")).strip().lower()

            if pos_unit in ("mm", "millimeter", "millimeters", "millimetre", "millimetres"):
                pos = [v / 1000.0 for v in pos]
            elif pos_unit in ("m", "meter", "meters", "metre", "metres"):
                pass
            else:
                rospy.logwarn_throttle(
                    5.0,
                    "未知 tool_position_pos_unit=%s，按 mm 处理",
                    pos_unit,
                )
                pos = [v / 1000.0 for v in pos]

            # -----------------------------
            # 2. 姿态统一转为 rad
            # -----------------------------
            rpy = [float(angular.x), float(angular.y), float(angular.z)]
            rpy_unit = str(getattr(self, "tool_position_rpy_unit", "deg")).strip().lower()

            if rpy_unit in ("deg", "degree", "degrees", "角度"):
                rpy = [math.radians(v) for v in rpy]
            elif rpy_unit in ("rad", "radian", "radians", "弧度"):
                pass
            else:
                rospy.logwarn_throttle(
                    5.0,
                    "未知 tool_position_rpy_unit=%s，按 degree 处理",
                    rpy_unit,
                )
                rpy = [math.radians(v) for v in rpy]

            stamp = time.time()
            frame_id = ""
            if hasattr(msg, "header"):
                frame_id = str(getattr(msg.header, "frame_id", ""))

            lock = getattr(self, "tool_position_lock", None)
            if lock is None:
                self.tool_position_pose = pos + rpy
                self.tool_position_stamp = stamp
                self.tool_position_frame_id = frame_id
                self.tool_position_received = True
                return

            with lock:
                self.tool_position_pose = pos + rpy
                self.tool_position_stamp = stamp
                self.tool_position_frame_id = frame_id
                self.tool_position_received = True

        except Exception as exc:
            rospy.logerr("updateToolPosition failed: %s", str(exc))

    def _get_tool_pose_snapshot(self):
        """读取当前 TCP 位姿快照，返回 received, stamp, pose, frame_id。"""
        lock = getattr(self, "tool_position_lock", None)
        if lock is None:
            return (
                bool(getattr(self, "tool_position_received", False)),
                float(getattr(self, "tool_position_stamp", 0.0)),
                list(getattr(self, "tool_position_pose", [0.0] * 6)),
                str(getattr(self, "tool_position_frame_id", "")),
            )

        with lock:
            return (
                bool(getattr(self, "tool_position_received", False)),
                float(getattr(self, "tool_position_stamp", 0.0)),
                list(getattr(self, "tool_position_pose", [0.0] * 6)),
                str(getattr(self, "tool_position_frame_id", "")),
            )

    # ------------------------------------------------------------------
    # IK 与运动服务封装
    # ------------------------------------------------------------------
    def solve_ik_for_pose(self, target_pose, ref_joints):
        """使用逆运动学求解目标位姿对应的关节角度，返回 rad 列表；失败返回空列表。"""
        target_joint = []
        try:
            req = GetIKRequest()

            # 左侧 IK 使用左侧参考关节，右侧 IK 使用右侧参考关节。
            side_ref_joints = self.ref_joint_right if self.isRightPos else self.ref_joint_left
            req.ref_joint = list(ref_joints) if ref_joints else list(side_ref_joints)

            # cartesian_pose: [x, y, z, rx, ry, rz]，位置 mm，姿态 rad。
            req.cartesian_pose = list(target_pose)

            rospy.loginfo("cartesian_pose : %s", req.cartesian_pose)
            rospy.loginfo("IK ref_joint(rad): %s", req.ref_joint)
            self.ik_client.wait_for_service(timeout=1.0)
            res = self.ik_client.call(req)

            if len(res.joint):
                target_joint = list(res.joint)
                rospy.loginfo("target_joint(rad) : %s", target_joint)
                rospy.loginfo("target_joint(deg) : %s", self.radJoint2degJoint(target_joint))
            else:
                rospy.logerr("solve_ik_for_pose failed: %s", getattr(res, "message", "empty joint result"))
        except Exception as exc:
            rospy.logerr("solve_ik_for_pose failed: %s", str(exc))

        return target_joint

    def _response_indicates_success(self, response):
        """
        判断 Move.srv 响应是否表示运动指令已被驱动接受。

        当前 JAKA /jaka_driver/joint_move 和 /jaka_driver/joint_move_tol
        实测：
            ret: 1
            message: "joint_move has been executed"

        因此本驱动中 ret=1 表示成功。
        """

        if response is None:
            return True, "no response object"

        bool_fields = ("success", "is_success", "result")
        for field in bool_fields:
            if hasattr(response, field):
                value = getattr(response, field)
                if isinstance(value, bool):
                    return value, "{}={}".format(field, value)

        if hasattr(response, "ret"):
            ret_value = getattr(response, "ret")
            message = str(getattr(response, "message", ""))

            try:
                ret_int = int(ret_value)
            except (TypeError, ValueError):
                return False, "ret={} cannot convert to int, message={}".format(
                    ret_value,
                    message,
                )

            if ret_int == 1:
                return True, "ret=1, message={}".format(message)

            return False, "ret={}, message={}".format(ret_int, message)

        code_fields = ("code", "errcode", "error_code")
        for field in code_fields:
            if hasattr(response, field):
                value = getattr(response, field)
                message = ""
                if hasattr(response, "message"):
                    message = ", message={}".format(getattr(response, "message"))
                try:
                    return int(value) == 0, "{}={}{}".format(field, value, message)
                except (TypeError, ValueError):
                    return bool(value), "{}={}{}".format(field, value, message)

        if hasattr(response, "message"):
            rospy.logwarn(
                "Move 服务响应只有 message 字段，未发现 success/ret/code 字段: %s",
                response.message,
            )
        else:
            rospy.logwarn(
                "Move 服务响应未发现 success/ret/code 字段，暂按调用成功处理: %s",
                type(response),
            )

        return True, "no explicit status field"

    def _call_move_service(self, client, pose, mvvelo, mvacc, log_name, wait_timeout=None):
        """
        统一运动服务调用封装。

        返回 True 只表示服务接受了运动指令；是否真正运动完成由后续 robot_states 判断。
        """
        try:
            if wait_timeout is None:
                wait_timeout = self.motion_service_timeout_s

            req = MoveRequest()
            req.pose = list(pose)
            req.mvvelo = mvvelo
            req.mvacc = mvacc
            client.wait_for_service(timeout=wait_timeout)

            # 记录服务调用前时间。后续等待 robot_states 时，只接受该时间之后的新状态。
            self._last_motion_command_stamp = time.time()
            rospy.loginfo(
                "%s request: pose=%s, mvvelo=%s, mvacc=%s",
                log_name,
                req.pose,
                req.mvvelo,
                req.mvacc,
            )
            res = client.call(req)
            ok, detail = self._response_indicates_success(res)
            if not ok:
                rospy.logerr("%s service returned failure: %s", log_name, detail)
                return False
            rospy.loginfo("%s service accepted: %s", log_name, detail)
            return True
        except Exception as exc:
            self._last_motion_command_stamp = 0.0
            rospy.logerr("%s failed: %s", log_name, str(exc))
            return False

    def _coerce_joint_pose_rad(self, joints, unit="rad"):
        """将关节角列表转换为 rad。"""
        unit = self._normalize_joint_unit_name(unit)
        pose = [float(j) for j in list(joints)[:6]]
        if unit == "degree":
            pose = self.degJoint2radJoint(pose)
        return pose

    def _valid_6d_value(self, valueList):
        return valueList is not None and len(valueList) >= 6 and tuple(valueList[:6]) != INVALID_6D_VALUE

    def _motion_request_allowed(self, valueList, log_name, require_ready=True):
        """
        检查运动前置条件。

        require_ready=True：用于单条运动，要求 robot_states 已经 Stop，可接收新指令。
        require_ready=False：用于连续 tol 轨迹段，只检查状态新鲜和安全故障，
        不要求上一条运动已经 Stop，避免连续下发 robotJointMoveTolL 时在中间等待到位。
        """
        if not self._valid_6d_value(valueList):
            rospy.logerr("%s rejected: invalid 6D pose/joint list: %s", log_name, valueList)
            return False
        if self.cancel_responding:
            rospy.logerr("%s rejected: cancel_responding=True", log_name)
            return False

        state = self._get_robot_state_snapshot()
        if require_ready:
            reason = self._robot_ready_reason(state)
        else:
            reason = self._robot_fault_reason(state)
            if reason is None and state["motion_state"] == MOTION_PAUSE:
                reason = "机器人暂停 Pause，不允许连续下发运动指令"

        if reason is not None:
            rospy.logerr("%s rejected: %s", log_name, reason)
            return False
        return True

    def robotJointMove(self, j1, j2, j3, j4, j5, j6, unit="rad"):
        """
        关节空间运动。

        单位约定：默认输入 rad；若外部仍传 degree，必须显式 unit='degree'。
        下发给 Move 服务前统一转换为 rad。
        """
        pose = self._coerce_joint_pose_rad([j1, j2, j3, j4, j5, j6], unit=unit)
        ok = self._call_move_service(
            self.move_joint_client,
            pose,
            math.radians(180.0),
            math.radians(360.0),
            "robotJointMoveTo",
            wait_timeout=self.motion_service_timeout_s,
        )
        if ok:
            rospy.loginfo("robotJointMoveTo rad: %s", pose)
            rospy.loginfo("robotJointMoveTo deg: %s", self.radJoint2degJoint(pose))
        return ok

    def robotJointMoveL(self, valueList, wait=True, timeout_s=None, sleep_s=None, unit="rad"):
        """
        普通关节运动封装。

        wait=True 时根据 robot_states 等待运动完成，不再固定 sleep。
        sleep_s 参数仅兼容旧调用，当前不使用。
        """
        rospy.loginfo("come in robotJointMoveL")

        if not self._wait_robot_ready(timeout_s=self.robot_ready_timeout_s):
            rospy.logerr("robotJointMoveL aborted: robot not ready")
            return False
        if not self._motion_request_allowed(valueList, "robotJointMoveL"):
            return False

        ok = self.robotJointMove(
            valueList[0], valueList[1], valueList[2],
            valueList[3], valueList[4], valueList[5],
            unit=unit,
        )

        if wait and ok:
            ok = self.wait_arm_motion_stable(
                timeout_s=timeout_s or self.motion_done_timeout_s,
                command_stamp=self._last_motion_command_stamp,
                log_name="关节运动完成等待",
            )

        rospy.loginfo("out robotJointMoveL, ok=%s", ok)
        return ok

    def robotJointMoveTol(self, j1, j2, j3, j4, j5, j6, unit="rad"):
        """关节空间 tol 运动。默认输入 rad。"""
        pose = self._coerce_joint_pose_rad([j1, j2, j3, j4, j5, j6], unit=unit)
        ok = self._call_move_service(
            self.move_joint_tol_client,
            pose,
            math.radians(180.0),
            math.radians(360.0),
            "robotJointMoveTolTo",
            wait_timeout=self.motion_service_timeout_s,
        )
        if ok:
            rospy.loginfo("robotJointMoveTolTo rad: %s", pose)
            rospy.loginfo("robotJointMoveTolTo deg: %s", self.radJoint2degJoint(pose))
        return ok

    def robotJointMoveTolL(
        self,
        valueList,
        wait=False,
        timeout_s=None,
        tol_rad=0.1,
        sleep_s=None,
        unit="rad",
        check_ready=True,
    ):
        """
        关节空间 tol 运动封装。

        wait=True 时根据 robot_states 等待运动完成，不再固定 sleep。
        check_ready=True 时在下发前等待并确认机器人 Stop。
        check_ready=False 仅用于连续 tol 轨迹内部点，不在两条连续指令之间检查到位。
        sleep_s/tol_rad 参数仅兼容旧调用，当前不用于运动等待。
        """
        rospy.loginfo("come in robotJointMoveTolL")

        if check_ready and not self._wait_robot_ready(timeout_s=self.robot_ready_timeout_s):
            rospy.logerr("robotJointMoveTolL aborted: robot not ready")
            return False
        if not self._motion_request_allowed(
            valueList,
            "robotJointMoveTolL",
            require_ready=check_ready,
        ):
            return False

        ok = self.robotJointMoveTol(
            valueList[0], valueList[1], valueList[2],
            valueList[3], valueList[4], valueList[5],
            unit=unit,
        )

        if wait and ok:
            ok = self.wait_arm_motion_stable(
                timeout_s=timeout_s or self.motion_done_timeout_s,
                command_stamp=self._last_motion_command_stamp,
                log_name="关节 tol 运动完成等待",
            )

        rospy.loginfo("out robotJointMoveTolL, ok=%s", ok)
        return ok

    def robotJointMoveTolSequenceL(
        self,
        value_lists,
        wait=True,
        timeout_s=None,
        unit="rad",
    ):
        """
        连续关节 tol 轨迹封装。

        设计目的：
        - 只在连续段开始前等待一次 robot ready；
        - 中间点只检查安全故障，不要求 motion_state 回到 Stop；
        - 连续段全部指令下发完成后，只等待最后一条指令完成。

        适用于“过渡点 -> 目标点”“预抓取点 -> 抓取点”等需要平滑衔接的场景。
        不适用于中间需要夹爪动作、视觉刷新或人工安全确认的场景。
        """
        targets = [list(item) for item in list(value_lists or [])]
        if not targets:
            rospy.logerr("robotJointMoveTolSequenceL rejected: empty target list")
            return False

        rospy.loginfo("come in robotJointMoveTolSequenceL, count=%d", len(targets))

        if not self._wait_robot_ready(timeout_s=self.robot_ready_timeout_s):
            rospy.logerr("robotJointMoveTolSequenceL aborted: robot not ready")
            return False

        last_command_stamp = 0.0
        for index, target in enumerate(targets, start=1):
            log_name = "robotJointMoveTolSequenceL[{}/{}]".format(index, len(targets))

            # 第 1 个点要求 ready；后续点不要求上一点已经到位，
            # 只检查 power/servo/collision/EmeStop/Error/状态超时等安全故障。
            if not self._motion_request_allowed(
                target,
                log_name,
                require_ready=(index == 1),
            ):
                return False

            ok = self.robotJointMoveTol(
                target[0], target[1], target[2],
                target[3], target[4], target[5],
                unit=unit,
            )
            if not ok:
                rospy.logerr("%s service call failed", log_name)
                return False

            last_command_stamp = self._last_motion_command_stamp
            rospy.loginfo("%s accepted, no intermediate arrival wait", log_name)

        ok = True
        if wait:
            ok = self.wait_arm_motion_stable(
                timeout_s=timeout_s or self.motion_done_timeout_s,
                command_stamp=last_command_stamp,
                log_name="连续关节 tol 运动完成等待",
            )

        rospy.loginfo("out robotJointMoveTolSequenceL, ok=%s", ok)
        return ok

    def robotCartesianMove(self, x, y, z, rx, ry, rz):
        """笛卡尔直线运动。位置单位：mm；姿态单位：rad。"""
        ok = self._call_move_service(
            self.move_line_client,
            [x, y, z, rx, ry, rz],
            200,
            400,
            "robotCartesianMoveTo",
            wait_timeout=self.motion_service_timeout_s,
        )
        if ok:
            rospy.loginfo(
                "robotCartesianMoveTo: %f,%f,%f,%f,%f,%f",
                x, y, z, math.degrees(rx), math.degrees(ry), math.degrees(rz),
            )
        return ok

    def robotCartesianMoveL(self, valueList, wait=True, timeout_s=None, sleep_s=None):
        """
        普通笛卡尔运动封装。

        wait=True 时根据 robot_states 等待运动完成，不再固定 sleep。
        """
        rospy.loginfo("come in robotCartesianMoveL")

        if not self._wait_robot_ready(timeout_s=self.robot_ready_timeout_s):
            rospy.logerr("robotCartesianMoveL aborted: robot not ready")
            return False
        if not self._motion_request_allowed(valueList, "robotCartesianMoveL"):
            return False

        ok = self.robotCartesianMove(
            valueList[0], valueList[1], valueList[2],
            valueList[3], valueList[4], valueList[5],
        )

        if wait and ok:
            ok = self.wait_arm_motion_stable(
                timeout_s=timeout_s or self.motion_done_timeout_s,
                command_stamp=self._last_motion_command_stamp,
                log_name="笛卡尔运动完成等待",
            )

        rospy.loginfo("out robotCartesianMoveL, ok=%s", ok)
        return ok

    def robotCartesianMoveTol(self, x, y, z, rx, ry, rz):
        """笛卡尔直线运动 tol。位置单位：mm；姿态单位：rad。"""
        ok = self._call_move_service(
            self.move_line_tol_client,
            [x, y, z, rx, ry, rz],
            400,
            760,
            "robotCartesianMoveTolTo",
            wait_timeout=self.motion_service_timeout_s,
        )
        if ok:
            rospy.loginfo(
                "robotCartesianMoveTolTo: %f,%f,%f,%f,%f,%f",
                x, y, z, math.degrees(rx), math.degrees(ry), math.degrees(rz),
            )
        return ok

    def robotCartesianMoveTolL(
        self,
        valueList,
        wait=True,
        timeout_s=None,
        sleep_s=None,
    ):
        """
        笛卡尔直线 tol 运动封装。

        wait=True 时根据 robot_states 等待运动完成，不再固定 sleep。
        """
        rospy.loginfo("come in robotCartesianMoveTolL")

        if not self._wait_robot_ready(timeout_s=self.robot_ready_timeout_s):
            rospy.logerr("robotCartesianMoveTolL aborted: robot not ready")
            return False
        if not self._motion_request_allowed(valueList, "robotCartesianMoveTolL"):
            return False

        ok = self.robotCartesianMoveTol(
            valueList[0], valueList[1], valueList[2],
            valueList[3], valueList[4], valueList[5],
        )

        if wait and ok:
            ok = self.wait_arm_motion_stable(
                timeout_s=timeout_s or self.motion_done_timeout_s,
                command_stamp=self._last_motion_command_stamp,
                log_name="笛卡尔 tol 运动完成等待",
            )

        rospy.loginfo("out robotCartesianMoveTolL, ok=%s", ok)
        return ok

    # ------------------------------------------------------------------
    # 多臂夹爪控制
    # ------------------------------------------------------------------
    def _normalize_grip_value(self, state):
        """
        将输入状态转换为多臂夹爪 grip 字段。

        约定：
        - 0 / False：打开夹爪；
        - 1 / True ：闭合夹爪。
        """
        if isinstance(state, bool):
            return 1 if state else 0

        try:
            grip_value = int(state)
        except (TypeError, ValueError):
            raise ValueError("invalid gripper state: {}, expected 0/open or 1/close".format(state))

        if grip_value not in (0, 1):
            raise ValueError("invalid gripper state: {}, expected 0/open or 1/close".format(state))

        return grip_value

    def _ensure_multi_gripper_publisher(self):
        """
        确保当前 self.gripper_pub 为多臂夹爪命令发布器。

        这样即使其它初始化文件里还保留旧的 Bool 发布器，本函数第一次调用时也会
        在 robot_motion_control.py 内部切换到 /multi_gripper/cmd + GripperCmd，
        不需要新增文件。
        """
        topic = rospy.get_param(
            "robot_vision_wrapper/gripper_cmd_topic",
            getattr(self, "gripper_cmd_topic", "/multi_gripper/cmd"),
        )
        gripper_id = rospy.get_param(
            "robot_vision_wrapper/jaka_gripper_id",
            getattr(self, "jaka_gripper_id", 1),
        )

        self.gripper_cmd_topic = str(topic)
        self.jaka_gripper_id = int(gripper_id)

        if GripperCmd is None:
            rospy.logerr(
                "z_efg_ros/GripperCmd is not available; cannot publish gripper command"
            )
            return False

        if (
            getattr(self, "_multi_gripper_pub_ready", False)
            and getattr(self, "_multi_gripper_pub_topic", None) == self.gripper_cmd_topic
        ):
            return True

        self.gripper_pub = rospy.Publisher(
            self.gripper_cmd_topic,
            GripperCmd,
            queue_size=5,
        )
        self.robotGripperPub = self.gripper_pub
        self._multi_gripper_pub_ready = True
        self._multi_gripper_pub_topic = self.gripper_cmd_topic

        rospy.loginfo(
            "multi gripper publisher initialized: topic=%s, msg=z_efg_ros/GripperCmd, jaka_gripper_id=%d",
            self.gripper_cmd_topic,
            self.jaka_gripper_id,
        )
        return True

    def _wait_for_gripper_subscriber(self):
        """Wait until the gripper command topic has a real subscriber."""
        require_subscriber = bool(
            rospy.get_param(
                "robot_vision_wrapper/require_gripper_subscriber",
                getattr(self, "require_gripper_subscriber", True),
            )
        )
        if not require_subscriber:
            return True

        timeout_s = max(
            0.0,
            float(
                rospy.get_param(
                    "robot_vision_wrapper/gripper_wait_subscriber_timeout_s",
                    getattr(self, "gripper_wait_subscriber_timeout_s", 1.0),
                )
            ),
        )
        start = time.time()
        while not rospy.is_shutdown():
            if self.gripper_pub.get_num_connections() > 0:
                return True
            if (time.time() - start) >= timeout_s:
                rospy.logerr(
                    "no subscriber connected on gripper topic %s after %.2fs",
                    getattr(self, "gripper_cmd_topic", "/multi_gripper/cmd"),
                    timeout_s,
                )
                return False
            rospy.sleep(0.02)

        return False

    def _init_gripper_feedback_subscriber(self):
        """Subscribe to interpreted gripper feedback for command/result linkage."""
        if not getattr(self, "use_gripper_feedback_wait", True):
            rospy.loginfo("gripper feedback wait disabled")
            return

        if InterpretedState is None:
            rospy.logerr(
                "z_efg_ros/InterpretedState is not available; cannot use gripper feedback"
            )
            self.gripper_feedback_sub = None
            return

        if getattr(self, "gripper_feedback_lock", None) is None:
            self.gripper_feedback_lock = Lock()
        if not hasattr(self, "gripper_feedback_state_by_id"):
            self.gripper_feedback_state_by_id = {}
        if not hasattr(self, "gripper_feedback_stamp_by_id"):
            self.gripper_feedback_stamp_by_id = {}

        topic = str(
            rospy.get_param(
                "robot_vision_wrapper/gripper_feedback_topic",
                getattr(self, "gripper_feedback_topic", "/multi_gripper/interpreted_state"),
            )
        )
        self.gripper_feedback_topic = topic
        self.gripper_feedback_sub = rospy.Subscriber(
            topic,
            InterpretedState,
            self._update_gripper_interpreted_state,
            queue_size=20,
        )
        rospy.loginfo(
            "gripper feedback subscriber initialized: topic=%s, msg=z_efg_ros/InterpretedState, gripper_id=%d",
            topic,
            int(getattr(self, "jaka_gripper_id", 1)),
        )

    def _update_gripper_interpreted_state(self, msg):
        gripper_id = int(msg.id)
        state = str(msg.interpreted_state).strip().lower()
        now = time.time()
        with self.gripper_feedback_lock:
            self.gripper_feedback_state_by_id[gripper_id] = state
            self.gripper_feedback_stamp_by_id[gripper_id] = now
        self.gripper_is_connected = True

    def _parse_gripper_state_names(self, value, default_states):
        if isinstance(value, str):
            raw_items = value.replace(";", ",").split(",")
        elif isinstance(value, (list, tuple, set)):
            raw_items = list(value)
        else:
            raw_items = list(default_states)

        states = set()
        for item in raw_items:
            text = str(item).strip().lower()
            if text:
                states.add(text)

        if not states:
            states = set(default_states)
        return states

    def _accepted_gripper_states_for_command(self, state):
        grip_value = self._normalize_grip_value(state)
        if grip_value:
            configured = rospy.get_param(
                "robot_vision_wrapper/gripper_close_success_states",
                getattr(self, "gripper_close_success_states", "close"),
            )
            return self._parse_gripper_state_names(configured, {"close"})

        configured = rospy.get_param(
            "robot_vision_wrapper/gripper_open_success_states",
            getattr(self, "gripper_open_success_states", "open"),
        )
        return self._parse_gripper_state_names(configured, {"open"})

    def wait_for_gripper_command_result(self, state):
        """Wait for interpreted feedback after the last gripper command."""
        use_feedback = bool(
            rospy.get_param(
                "robot_vision_wrapper/use_gripper_feedback_wait",
                getattr(self, "use_gripper_feedback_wait", True),
            )
        )
        if not use_feedback:
            return self._sleep_with_shutdown_check(
                getattr(self, "gripper_action_wait_s", 1.0),
                "夹爪动作等待",
            )

        if InterpretedState is None or getattr(self, "gripper_feedback_sub", None) is None:
            rospy.logerr("gripper feedback is enabled but subscriber is not initialized")
            return False

        gripper_id = int(getattr(self, "jaka_gripper_id", 1))
        accepted_states = self._accepted_gripper_states_for_command(state)
        command_stamp = float(getattr(self, "_last_gripper_command_stamp", 0.0))
        timeout_s = max(
            0.0,
            float(
                rospy.get_param(
                    "robot_vision_wrapper/gripper_feedback_timeout_s",
                    getattr(self, "gripper_feedback_timeout_s", 3.0),
                )
            ),
        )
        poll_s = max(
            0.005,
            float(
                rospy.get_param(
                    "robot_vision_wrapper/gripper_feedback_poll_s",
                    getattr(self, "gripper_feedback_poll_s", 0.02),
                )
            ),
        )

        start = time.time()
        last_state = None
        last_stamp = 0.0
        while not rospy.is_shutdown():
            with self.gripper_feedback_lock:
                last_state = self.gripper_feedback_state_by_id.get(gripper_id)
                last_stamp = float(self.gripper_feedback_stamp_by_id.get(gripper_id, 0.0))

            if last_stamp >= command_stamp and last_state in accepted_states:
                rospy.loginfo(
                    "gripper feedback reached: id=%d, state=%s, accepted=%s",
                    gripper_id,
                    last_state,
                    sorted(accepted_states),
                )
                return True

            if (time.time() - start) >= timeout_s:
                rospy.logerr(
                    "gripper feedback timeout: id=%d, last_state=%s, accepted=%s, last_age=%.3fs",
                    gripper_id,
                    last_state,
                    sorted(accepted_states),
                    time.time() - last_stamp if last_stamp > 0.0 else float("inf"),
                )
                return False

            rospy.sleep(poll_s)

        return False

    def robotGripperChangeState(self, state):
        """
        兼容旧函数名。

        现在不再发布 std_msgs/Bool，而是统一走 robotGripper3ChangeState()，发布：
            z_efg_ros/GripperCmd {id: 1, grip: 0/1}
        """
        ok = self.robotGripper3ChangeState(state)
        if not ok:
            return False
        return self.wait_for_gripper_command_result(state)

    def robotGripper3ChangeState(self, state):
        """
        多臂夹爪控制函数。

        参数：
            state = 0 / False：打开夹爪；
            state = 1 / True ：闭合夹爪。

        发布话题：
            默认 /multi_gripper/cmd，可通过 robot_vision_wrapper/gripper_cmd_topic 修改。

        消息类型：
            z_efg_ros/GripperCmd

        JAKA 手爪：
            默认 id = 1，可通过 robot_vision_wrapper/jaka_gripper_id 修改。

        等价测试命令：
            rostopic pub /multi_gripper/cmd z_efg_ros/GripperCmd "{id: 1, grip: 1}" -1
            rostopic pub /multi_gripper/cmd z_efg_ros/GripperCmd "{id: 1, grip: 0}" -1
        """
        try:
            grip_value = self._normalize_grip_value(state)
            if not self._ensure_multi_gripper_publisher():
                return False

            if not self._wait_for_gripper_subscriber():
                return False

            cmd = GripperCmd()
            cmd.id = int(getattr(self, "jaka_gripper_id", 1))
            cmd.grip = int(grip_value)
            self._last_gripper_command_stamp = time.time()
            self._last_gripper_command_id = cmd.id
            self._last_gripper_command_grip = cmd.grip

            repeat_count = max(
                1,
                int(
                    rospy.get_param(
                        "robot_vision_wrapper/gripper_cmd_repeat",
                        getattr(self, "gripper_cmd_repeat", 1),
                    )
                ),
            )
            repeat_interval_s = max(
                0.0,
                float(
                    rospy.get_param(
                        "robot_vision_wrapper/gripper_cmd_repeat_interval_s",
                        getattr(self, "gripper_cmd_repeat_interval_s", 0.05),
                    )
                ),
            )

            for index in range(repeat_count):
                self.gripper_pub.publish(cmd)
                if index + 1 < repeat_count:
                    rospy.sleep(repeat_interval_s)

            rospy.loginfo(
                "gripper3ChangeState: topic=%s, id=%d, grip=%d",
                getattr(self, "gripper_cmd_topic", "/multi_gripper/cmd"),
                cmd.id,
                cmd.grip,
            )
            return True

        except Exception as exc:
            rospy.logerr("robotGripper3ChangeState failed: %s", str(exc))
            return False
