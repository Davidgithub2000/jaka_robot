#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_vision_auto.py

视觉目标、自动采摘状态机和抓取流程模块。

说明：
- 本版按需求停用视觉多帧缓冲和稳定目标确认；
- 自动采摘由 /custom_arm_data 单次目标消息触发，收到后立即转换坐标并入队；
- 仍不删除 calculate_grasp_pose() 中的固定测试坐标覆盖，方便保留调试行为；
- 机械臂运动等待方式仍采用 robot_states 状态判断，不使用固定 sleep 判断到位；
- 夹爪当前没有状态反馈，仍保留 gripper_action_wait_s 短暂等待；
- 夹爪闭合后先退回采摘前置点作为过渡点，再运动到放果位置。
"""

import json
import math
import time
from threading import Thread

import numpy as np
import rospy
from scipy.spatial.transform import Rotation as R

from robot_vision_config import LEFT_SIDE, RIGHT_SIDE


class RobotVisionAutoMixin:
    @staticmethod
    def _rotation_to_matrix(rotation):
        if hasattr(rotation, "as_matrix"):
            return rotation.as_matrix()
        return rotation.as_dcm()

    @staticmethod
    def _rotation_from_matrix(matrix):
        if hasattr(R, "from_matrix"):
            return R.from_matrix(matrix)
        return R.from_dcm(matrix)

    def start_auto_pick(self):
        """启动自动采摘控制线程。"""
        if self.auto_pick_thread is not None and self.auto_pick_thread.is_alive():
            rospy.logwarn("自动采摘控制线程已经在运行")
            return

        self.auto_pick_thread = Thread(target=self._auto_pick_loop)
        self.auto_pick_thread.daemon = True
        self.auto_pick_thread.start()
        rospy.loginfo("消息触发采摘线程已启动，订阅目标话题: %s", getattr(self, "custom_arm_data_topic", "/custom_arm_data"))

    def _publish_pick_target_result(self, target_job, ok, reason="", arm_safe=True, arm_safe_reason="not_executed"):
        """Publish one result message for the target popped from /custom_arm_data."""
        pub = getattr(self, "pick_result_pub", None)
        if pub is None:
            return

        payload = {
            "stamp": time.time(),
            "node": rospy.get_name(),
            "namespace": rospy.get_namespace(),
            "success": bool(ok),
            "reason": str(reason or ""),
            "arm_safe": bool(arm_safe),
            "arm_safe_reason": str(arm_safe_reason or ""),
            "target_side": "right" if target_job.get("target_side", False) else "left",
            "p1_base_mm": [float(v) for v in target_job.get("p1_base", [])[:3]],
            "p2_base_mm": [float(v) for v in target_job.get("p2_base", [])[:3]],
            "p1_base_m": [float(v) for v in target_job.get("p1_base_m", [])[:3]],
            "p2_base_m": [float(v) for v in target_job.get("p2_base_m", [])[:3]],
            "processed_target_count": int(getattr(self, "processed_target_count", 0)),
            "pending_target_count": int(self._pending_target_count()),
        }

        try:
            pub.publish(json.dumps(payload, ensure_ascii=False))
        except Exception as exc:
            rospy.logwarn("发布 target_result 失败: %s", str(exc))

    def _current_init_joints(self):
        return self.init_joint_right if self.isRightPos else self.init_joint_left

    def _check_arm_safe_at_init(self):
        side = "right" if self.isRightPos else "left"
        init_joints = self._current_init_joints()

        if not self.wait_arm_motion_stable(
            timeout_s=min(float(getattr(self, "motion_done_timeout_s", 30.0)), 3.0),
            log_name="arm_safe Stop confirm",
        ):
            return False, "{} arm is not stable Stop".format(side)

        close_ok, close_reason = self.joints_close_to(
            init_joints,
            tolerance_rad=float(getattr(self, "arm_safe_joint_tolerance_rad", 0.08)),
        )
        if not close_ok:
            return False, "{} arm not at init: {}".format(side, close_reason)

        return True, "{} arm at init: {}".format(side, close_reason)

    def _recover_arm_to_init_for_safety(self):
        side = "right" if self.isRightPos else "left"
        init_joints = self._current_init_joints()

        safe, reason = self._check_arm_safe_at_init()
        if safe:
            return True, reason

        if not getattr(self, "recover_to_init_on_pick_failure", True):
            return False, "recover disabled; {}".format(reason)

        rospy.logwarn("%s arm not safe after pick result, trying to return init: %s", side, reason)
        if not self.robotJointMoveTolL(
            init_joints,
            wait=True,
            timeout_s=self.motion_done_timeout_s,
        ):
            return False, "{} arm failed to move init for safety".format(side)

        rospy.set_param(self.arm_is_right_pos_param, 1 if self.isRightPos else 0)
        return self._check_arm_safe_at_init()

    def _set_vision_accepting(self, enable):
        """
        控制本程序是否接收视觉目标。
        同时设置视觉使能参数，如果视觉节点支持该参数，则视觉节点也可以暂停识别。
        """
        self.vision_accepting = bool(enable)

        try:
            rospy.set_param(self.vision_enable_param, bool(enable))
        except Exception as exc:
            rospy.logwarn("set %s failed: %s", self.vision_enable_param, str(exc))

        rospy.loginfo("vision_accepting = %s", self.vision_accepting)

    def goalCB(self, msg):
        """
        /custom_arm_data 回调。

        新流程：
        1. 总控节点已完成视觉合理性判断，每个目标只发布一次 Float32MultiArray；
        2. 本回调必须立即解析并保存目标，不能因为运动中或 vision_accepting=False 而丢弃；
        3. 视觉点是当前 TCP 坐标系下坐标，回调中使用“收到消息瞬间”的 tool_position
           快照转换到机械臂基坐标系；
        4. 转换后的基坐标目标入队，由自动采摘线程逐个执行。
        """
        if self._is_empty_custom_target_msg(msg):
            rospy.loginfo_throttle(5.0, "忽略 /custom_arm_data 空目标占位")
            return

        try:
            p1_tcp, p2_tcp = self._extract_vision_points_from_msg(msg)
            target_job = self._build_target_job_from_tcp_points(p1_tcp, p2_tcp)
        except Exception as exc:
            self.dropped_target_count = int(getattr(self, "dropped_target_count", 0)) + 1
            rospy.logerr("解析/转换 /custom_arm_data 目标失败，已丢弃: %s", str(exc))
            self._publish_pick_target_result(
                {"target_side": bool(getattr(self, "isRightPos", False))},
                False,
                "target_parse_or_transform_failed: {}".format(str(exc)),
            )
            return

        if not self._push_pick_target(target_job):
            self._publish_pick_target_result(target_job, False, "target_queue_rejected")
            return

        with self.vision_lock:
            self.vision_seq += 1
            seq = self.vision_seq
            self.p1_tcp = list(target_job["p1_tcp"])
            self.p2_tcp = list(target_job["p2_tcp"])
            self.p1_base = list(target_job["p1_base"])
            self.p2_base = list(target_job["p2_base"])
            self.goal_data = list(target_job["p1_base"])
            self.vision_is_connected = True
            self.vision_target_stamp = float(target_job["stamp"])
            self.vision_target_buffer = []

        rospy.loginfo(
            "收到 /custom_arm_data 单次目标并入队: seq=%s, queue=%s, side=%s, "
            "p1_tcp_m=%s, p2_tcp_m=%s, p1_base_m=%s, p2_base_m=%s, p1_base_mm=%s, p2_base_mm=%s",
            seq,
            self._pending_target_count(),
            "右" if target_job["target_side"] else "左",
            [round(float(v), 6) for v in target_job["p1_tcp"]],
            [round(float(v), 6) for v in target_job["p2_tcp"]],
            [round(float(v), 6) for v in target_job.get("p1_base_m", [0.0, 0.0, 0.0])],
            [round(float(v), 6) for v in target_job.get("p2_base_m", [0.0, 0.0, 0.0])],
            [round(float(v), 3) for v in target_job["p1_base"]],
            [round(float(v), 3) for v in target_job["p2_base"]],
        )

    def _is_empty_custom_target_msg(self, msg):
        """Return True for the no-target placeholder published by the vision node."""
        if not hasattr(msg, "data"):
            return False

        data = list(msg.data)
        if len(data) < 6:
            return False

        epsilon = float(getattr(self, "target_empty_epsilon", 1e-6))
        try:
            return all(abs(float(value)) <= epsilon for value in data[:6])
        except (TypeError, ValueError):
            return False
    def _extract_vision_points_from_msg(self, msg):
        """
        从视觉消息中提取两个 TCP 坐标系下的点。

        推荐消息格式：std_msgs/Float32MultiArray，data 长度至少 6：
            [p1x, p1y, p1z, p2x, p2y, p2z]
        兼容格式：geometry_msgs/Pose：position 作为 p1，orientation.x/y/z 作为 p2。
        """
        if hasattr(msg, "data"):
            data = list(msg.data)
            if len(data) < 6:
                raise ValueError("Float32MultiArray.data 长度不足 6，无法解析两个三维点")
            p1 = [float(data[0]), float(data[1]), float(data[2])]
            p2 = [float(data[3]), float(data[4]), float(data[5])]
            return p1, p2

        if hasattr(msg, "position") and hasattr(msg, "orientation"):
            p1 = [float(msg.position.x), float(msg.position.y), float(msg.position.z)]
            p2 = [float(msg.orientation.x), float(msg.orientation.y), float(msg.orientation.z)]
            return p1, p2

        raise ValueError("不支持的视觉消息格式；需要 data[0:6] 或 Pose(position/orientation.xyz)")


    def _build_target_job_from_tcp_points(self, p1_tcp, p2_tcp):
        """
        将 /custom_arm_data 中的一次目标消息封装成采摘任务。

        单位约定：
        1. /custom_arm_data 目标点单位为 m；
        2. /jaka_driver/tool_position 在 updateToolPosition() 中统一为 m + rad；
        3. TCP->base 坐标变换中间过程统一使用 m + rad；
        4. 原有 check_reachable()、calculate_grasp_pose()、IK 服务仍按 mm + rad，
           因此进入采摘动作前将 base 坐标从 m 转为 mm。
        """
        if not self._is_finite_point(p1_tcp) or not self._is_finite_point(p2_tcp):
            raise ValueError("目标点不是有限数")
        if self._is_zero_point(p1_tcp) or self._is_zero_point(p2_tcp):
            raise ValueError("目标点为空目标")

        tool_snapshot = self._wait_for_fresh_tool_position()

        # TCP -> base，返回 m。
        p1_base_m = self._tcp_point_to_base_with_snapshot(p1_tcp, tool_snapshot)
        p2_base_m = self._tcp_point_to_base_with_snapshot(p2_tcp, tool_snapshot)

        # 兼容原有工作空间判断和 IK：m -> mm。
        p1_base_mm = self._distance_vector_m_to_mm(p1_base_m)
        p2_base_mm = self._distance_vector_m_to_mm(p2_base_m)

        target_side = self._infer_target_side_from_base_points(p1_base_mm, p2_base_mm)
        if not self._points_reachable_for_side(p1_base_mm, p2_base_mm, target_side):
            side_name = "右" if target_side else "左"
            raise ValueError(
                "目标转换到基坐标后不在{}侧可达工作空间内: "
                "p1_base_m={}, p2_base_m={}, p1_base_mm={}, p2_base_mm={}".format(
                    side_name,
                    [round(float(v), 6) for v in p1_base_m],
                    [round(float(v), 6) for v in p2_base_m],
                    [round(float(v), 3) for v in p1_base_mm],
                    [round(float(v), 3) for v in p2_base_mm],
                )
            )

        return {
            "stamp": time.time(),
            "target_side": bool(target_side),

            # 原始 TCP 坐标，单位 m。
            "p1_tcp": [float(v) for v in p1_tcp[:3]],
            "p2_tcp": [float(v) for v in p2_tcp[:3]],

            # 保存 base 米制坐标，方便日志和调试。
            "p1_base_m": [float(v) for v in p1_base_m[:3]],
            "p2_base_m": [float(v) for v in p2_base_m[:3]],

            # 原有采摘流程使用 mm，所以 p1_base / p2_base 仍给 mm。
            "p1_base": [float(v) for v in p1_base_mm[:3]],
            "p2_base": [float(v) for v in p2_base_mm[:3]],

            # tool_pose 现在为 m + rad。
            "tool_pose": list(tool_snapshot[2]),
            "tool_frame_id": str(tool_snapshot[3]),
        }

    def _push_pick_target(self, target_job):
        """将单次目标加入采摘队列；队列满时丢弃最旧目标，保证最新目标不丢。"""
        max_len = max(1, int(getattr(self, "custom_target_queue_max_len", 10)))
        lock = getattr(self, "target_queue_lock", None)
        if lock is None:
            self.pending_pick_targets = getattr(self, "pending_pick_targets", [])[-(max_len - 1):]
            self.pending_pick_targets.append(target_job)
            return True

        with lock:
            queue = list(getattr(self, "pending_pick_targets", []))
            if len(queue) >= max_len:
                dropped = queue.pop(0)
                self.dropped_target_count = int(getattr(self, "dropped_target_count", 0)) + 1
                rospy.logwarn(
                    "采摘目标队列已满，丢弃最旧目标: stamp=%.3f, p1_base=%s",
                    float(dropped.get("stamp", 0.0)),
                    dropped.get("p1_base", []),
                )
            queue.append(target_job)
            self.pending_pick_targets = queue
        return True

    def _pop_pick_target(self):
        """取出一个待采摘目标；没有目标时返回 None。"""
        lock = getattr(self, "target_queue_lock", None)
        if lock is None:
            queue = getattr(self, "pending_pick_targets", [])
            if not queue:
                return None
            target = queue[0]
            self.pending_pick_targets = queue[1:]
            return target

        with lock:
            queue = list(getattr(self, "pending_pick_targets", []))
            if not queue:
                return None
            target = queue.pop(0)
            self.pending_pick_targets = queue
            return target

    def _pending_target_count(self):
        lock = getattr(self, "target_queue_lock", None)
        if lock is None:
            return len(getattr(self, "pending_pick_targets", []))
        with lock:
            return len(getattr(self, "pending_pick_targets", []))

    def _clear_pending_targets(self):
        lock = getattr(self, "target_queue_lock", None)
        if lock is None:
            self.pending_pick_targets = []
            return
        with lock:
            self.pending_pick_targets = []

    def _infer_target_side_from_base_points(self, p1_base, p2_base):
        """根据基坐标 Y 值推断目标属于左/右侧；必要时也可关闭该推断，沿用当前机位。"""
        if not getattr(self, "infer_target_side_from_base_y", True):
            return bool(self.isRightPos)
        mid_y = (float(p1_base[1]) + float(p2_base[1])) / 2.0
        return RIGHT_SIDE if mid_y >= 0.0 else LEFT_SIDE

    def _points_reachable_in_custom_workspace(self, matrix, matrix2):
        """按 launch 配置的工作空间检查两个目标点。"""
        if matrix is None or matrix2 is None or len(matrix) < 3 or len(matrix2) < 3:
            return False

        def _one_point_reachable(point):
            return (
                self.workspace_x_min <= float(point[0]) <= self.workspace_x_max
                and self.workspace_y_min <= float(point[1]) <= self.workspace_y_max
                and self.workspace_z_min <= float(point[2]) <= self.workspace_z_max
            )

        return _one_point_reachable(matrix) and _one_point_reachable(matrix2)

    def _points_reachable_for_side(self, matrix, matrix2, right_side):
        """检查目标位置是否在指定机位工作空间内。"""
        if matrix is None or matrix2 is None or len(matrix) < 3 or len(matrix2) < 3:
            return False

        if getattr(self, "use_custom_workspace", False):
            return self._points_reachable_in_custom_workspace(matrix, matrix2)

        if not right_side:
            return (
                -100 <= matrix[0] <= 600 and -820 <= matrix[1] <= -400 and 80 <= matrix[2] <= 810
                and -100 <= matrix2[0] <= 600 and -820 <= matrix2[1] <= -400 and 80 <= matrix2[2] <= 810
            )

        return (
            -100 <= matrix[0] <= 600 and 400 <= matrix[1] <= 820 and 80 <= matrix[2] <= 810
            and -100 <= matrix2[0] <= 600 and 400 <= matrix2[1] <= 820 and 80 <= matrix2[2] <= 810
        )

    def _clear_vision_target(self):
        """清空最新视觉目标，避免切换机位或采摘后复用旧目标。"""
        with self.vision_lock:
            self.p1_tcp = [0.0] * 3
            self.p2_tcp = [0.0] * 3
            self.p1_base = [0.0] * 3
            self.p2_base = [0.0] * 3
            self.goal_data = [0.0] * 3
            self.vision_is_connected = False
            self.vision_target_stamp = 0.0
            self.vision_target_buffer = []

    def _prune_vision_target_buffer_locked(self, now=None):
        """多帧视觉缓冲裁剪已停用；保留空函数兼容旧代码引用。"""
        # 原多帧缓冲裁剪逻辑已按需求注释停用。
        return

    def _get_vision_snapshot(self):
        """读取最新视觉帧快照。p1/p2 为 TCP 坐标系下的原始视觉点。"""
        with self.vision_lock:
            return (
                self.vision_seq,
                self.vision_is_connected,
                self.vision_target_stamp,
                list(getattr(self, "p1_tcp", [0.0, 0.0, 0.0])),
                list(getattr(self, "p2_tcp", [0.0, 0.0, 0.0])),
            )

    def _get_vision_buffer_snapshot(self):
        """多帧视觉缓冲已停用，始终返回空列表。"""
        # 原多帧缓冲快照逻辑已按需求注释停用。
        return []

    def _is_zero_point(self, p):
        return len(p) >= 3 and all(abs(float(v)) <= self.target_empty_epsilon for v in p[:3])

    def _is_finite_point(self, p):
        if p is None or len(p) < 3:
            return False
        try:
            return bool(np.all(np.isfinite(np.array(p[:3], dtype=float))))
        except (TypeError, ValueError):
            return False

    def _distance_vector_to_m(self, point, unit_name):
        """将三维点按指定单位转换为 m。"""
        arr = np.array(point, dtype=float).reshape(-1)
        if arr.size < 3:
            raise ValueError("point dimension is less than 3")

        arr = arr[:3]
        unit = str(unit_name).strip().lower()

        if unit in ("m", "meter", "meters", "metre", "metres"):
            pass
        elif unit in ("mm", "millimeter", "millimeters", "millimetre", "millimetres"):
            arr = arr / 1000.0
        else:
            rospy.logwarn_throttle(
                5.0,
                "未知视觉点单位 vision_target_unit=%s，按 m 处理",
                unit,
            )

        return arr

    def _distance_vector_m_to_mm(self, point_m):
        """m -> mm，兼容原有 IK / 工作空间判断。"""
        arr = np.array(point_m, dtype=float).reshape(-1)
        if arr.size < 3:
            raise ValueError("point dimension is less than 3")
        return arr[:3] * 1000.0

    def _distance_vector_to_mm(self, point, unit_name):
        """
        将三维点按指定单位转换为 mm。

        保留该函数用于兼容旧代码；新 TCP->base 坐标变换内部统一使用 m。
        """
        return self._distance_vector_m_to_mm(
            self._distance_vector_to_m(point, unit_name)
        )

    def _tcp_point_to_base(self, point_tcp):
        """
        将视觉点从当前 TCP 坐标系转换到机械臂基坐标系。

        返回单位：mm。

        该函数用于兼容旧逻辑；/custom_arm_data 单次目标处理应优先使用
        _tcp_point_to_base_with_snapshot() 得到 m，再显式转为 mm。
        """
        p_base_m = self._tcp_point_to_base_with_snapshot(
            point_tcp,
            self._get_tool_pose_snapshot(),
        )
        return [float(v) for v in self._distance_vector_m_to_mm(p_base_m)]

    def _tool_position_age_s(self, tool_snapshot):
        """返回 tool_position 快照年龄；未收到时返回 inf。"""
        received, stamp, _, _ = tool_snapshot
        if not received or float(stamp) <= 0.0:
            return float("inf")
        return time.time() - float(stamp)

    def _wait_for_fresh_tool_position(self):
        """
        等待一帧新鲜 /jaka_driver/tool_position。

        /custom_arm_data 每个目标只发布一次，不能因为启动瞬间 tool_position
        暂时没来就立即丢弃目标；但也不能等待太久，否则目标对应的 TCP 位姿会失真。
        """
        max_age = float(getattr(self, "tool_position_max_age_s", 1.0))
        timeout_s = max(0.0, float(getattr(self, "tool_position_wait_timeout_s", 0.5)))
        poll_s = max(0.001, float(getattr(self, "tool_position_wait_poll_s", 0.01)))

        start = time.time()
        last_snapshot = self._get_tool_pose_snapshot()

        while not rospy.is_shutdown():
            snapshot = self._get_tool_pose_snapshot()
            if self._tool_position_age_s(snapshot) <= max_age:
                return snapshot

            last_snapshot = snapshot
            if (time.time() - start) >= timeout_s:
                break

            rospy.sleep(poll_s)

        received, stamp, _, _ = last_snapshot
        if not received:
            raise ValueError("尚未收到 /jaka_driver/tool_position，无法做 TCP->base 坐标变换")

        age = self._tool_position_age_s(last_snapshot)
        raise ValueError(
            "/jaka_driver/tool_position 超时 {:.3f}s > {:.3f}s，等待 {:.3f}s 后仍无新鲜 TCP 位姿".format(
                age,
                max_age,
                timeout_s,
            )
        )

    def _tool_rotation_matrix_from_pose(self, tcp_pose):
        """
        根据 tool_position 中的姿态部分生成 R_base_tcp。

        默认假设 tcp_pose[3:6] 为 JAKA 常见 [rx, ry, rz] 欧拉角，单位已在
        updateToolPosition() 中统一为 rad。
        """
        rot_values = np.array(tcp_pose[3:6], dtype=float)
        if rot_values.size < 3 or not np.all(np.isfinite(rot_values[:3])):
            raise ValueError("tool_position 姿态不是有效三维数值: {}".format(tcp_pose[3:6]))

        rotation_type = str(getattr(self, "tool_position_rotation_type", "euler")).strip().lower()

        if rotation_type in ("rotvec", "rotation_vector", "rotation-vector", "axis_angle", "axis-angle"):
            # 旋转向量：方向为旋转轴，模长为旋转角 rad。
            return self._rotation_to_matrix(R.from_rotvec(rot_values[:3]))

        if rotation_type not in ("euler", "rpy", "euler_rpy"):
            rospy.logwarn_throttle(
                5.0,
                "未知 tool_position_rotation_type=%s，按 euler 处理",
                rotation_type,
            )

        euler_order = str(getattr(self, "tool_position_euler_order", "xyz")).strip()
        euler_order = "".join([ch for ch in euler_order if ch.lower() in ("x", "y", "z")])
        if len(euler_order) != 3 or len(set(euler_order.lower())) != 3:
            rospy.logwarn_throttle(
                5.0,
                "未知 tool_position_euler_order=%s，按 xyz 处理",
                getattr(self, "tool_position_euler_order", "xyz"),
            )
            euler_order = "xyz"

        # SciPy: 小写为固定轴/外旋，常用于机器人 RPY；大写为动轴/内旋。
        if bool(getattr(self, "tool_position_euler_intrinsic", False)):
            euler_order = euler_order.upper()
        else:
            euler_order = euler_order.lower()

        return self._rotation_to_matrix(R.from_euler(euler_order, rot_values[:3], degrees=False))

    def _tcp_point_to_base_with_snapshot(self, point_tcp, tool_snapshot):
        """
        使用指定 tool_position 快照完成 TCP 坐标系点到机械臂基坐标系的变换。

        单位约定：
            point_tcp  : /custom_arm_data 传入，单位由 vision_target_unit 指定，当前为 m；
            tool_pose  : updateToolPosition() 内部已统一为 m + rad；
            返回值     : p_base_m，单位 m。

        变换关系：
            P_base_m = T_base_tcp_m + R_base_tcp @ P_tcp_m

        tool_snapshot 格式来自 _get_tool_pose_snapshot():
            received, stamp, [x_m, y_m, z_m, rx_rad, ry_rad, rz_rad], frame_id
        """
        received, stamp, tcp_pose, frame_id = tool_snapshot
        if not received:
            raise ValueError("尚未收到 /jaka_driver/tool_position，无法做 TCP->base 坐标变换")

        max_age = float(getattr(self, "tool_position_max_age_s", 1.0))
        age = time.time() - float(stamp)
        if age > max_age:
            raise ValueError(
                "/jaka_driver/tool_position 超时 {:.3f}s > {:.3f}s，拒绝使用旧 TCP 位姿".format(
                    age,
                    max_age,
                )
            )

        # 1. 视觉 TCP 点统一转为 m。
        p_tcp_m = self._distance_vector_to_m(
            point_tcp,
            getattr(self, "vision_target_unit", "m"),
        )
        if not np.all(np.isfinite(p_tcp_m)):
            raise ValueError("视觉 TCP 点不是有限数: {}".format(point_tcp))

        # 2. tool_position 已在 updateToolPosition() 中统一为 m + rad。
        t_base_tcp_m = np.array(tcp_pose[:3], dtype=float)
        rpy_rad = np.array(tcp_pose[3:6], dtype=float)

        if not np.all(np.isfinite(t_base_tcp_m)):
            raise ValueError("tool_position 位置不是有效三维数值: {}".format(tcp_pose[:3]))

        if not np.all(np.isfinite(rpy_rad)):
            raise ValueError("tool_position 姿态不是有效三维数值: {}".format(tcp_pose[3:6]))

        # 3. 姿态 rad 生成旋转矩阵。
        r_base_tcp = self._tool_rotation_matrix_from_pose(tcp_pose)

        # 4. TCP 坐标系点转换到 base 坐标系，单位 m。
        p_base_m = t_base_tcp_m[:3] + np.dot(r_base_tcp, p_tcp_m[:3])

        rospy.loginfo(
            "TCP->base(m/rad): frame=%s, age=%.3fs, rot_type=%s, euler_order=%s, intrinsic=%s, "
            "tcp_pose_m_rad=%s, p_tcp_m=%s, p_base_m=%s",
            frame_id,
            age,
            getattr(self, "tool_position_rotation_type", "euler"),
            getattr(self, "tool_position_euler_order", "xyz"),
            getattr(self, "tool_position_euler_intrinsic", False),
            [round(float(v), 6) for v in tcp_pose],
            [round(float(v), 6) for v in p_tcp_m],
            [round(float(v), 6) for v in p_base_m],
        )
        return [float(v) for v in p_base_m]

    def _vision_sample_valid(self, sample):
        """多帧样本有效性判断已停用；保留空实现兼容旧代码引用。"""
        # 原逻辑：检查样本机位、时间戳、有效性、可达性。
        # 新逻辑：不再处理历史样本，只在 _current_target_snapshot() 中检查最新帧。
        return False, "多帧样本确认已停用"

    def _cluster_valid_vision_samples(self, valid_samples):
        """多帧聚类功能已停用；保留空实现兼容旧代码引用。"""
        # 原逻辑：按番茄串中点进行空间聚类。
        return []

    def _select_stable_target_from_buffer(self):
        """多帧稳定目标选择已停用，改为返回最新视觉帧的目标快照。"""
        # 原逻辑：从 vision_target_buffer 中筛选、聚类并取中位数。
        # 新逻辑：只取最新帧，并在采摘前做 TCP->base 转换。
        return self._current_target_snapshot()

    def _accumulate_vision_before_decision(self):
        """
        多帧累计功能已按需求停用。
        当前流程只取最新一帧视觉反馈；保留函数名是为了兼容旧调用。
        """
        # 原多帧累计逻辑已注释停用：
        # window_s = max(0.0, float(getattr(self, "vision_accumulate_window_s", 1.0)))
        # ... 循环等待多个视觉帧 ...
        return True

    def _wait_for_vision_feedback(self, timeout_s):
        """等待任意视觉反馈。已有新鲜反馈时直接返回 True。"""
        seq, connected, stamp, _, _ = self._get_vision_snapshot()
        if connected and stamp > 0 and (time.time() - stamp) <= self.vision_feedback_max_age_s:
            return True

        start_seq = seq
        return self._wait_until(
            lambda: self._get_vision_snapshot()[0] > start_seq,
            timeout_s=timeout_s,
            sleep_s=0.05,
        )

    def _wait_for_new_vision(self, old_seq, timeout_s):
        """等待序号大于 old_seq 的新视觉帧。"""
        return self._wait_until(
            lambda: self._get_vision_snapshot()[0] > old_seq,
            timeout_s=timeout_s,
            sleep_s=0.05,
        )

    def _wait_for_new_vision_after_stable(self, old_seq, stable_time, timeout_s):
        """
        只等待机械臂稳定后的最新一帧视觉数据。
        多帧缓冲/稳定聚类已停用，不再等待累计窗口。
        """
        start_time = time.time()
        poll_s = max(0.005, float(getattr(self, "vision_sample_poll_s", 0.02)))
        last_reason = "尚未收到机械臂稳定后的视觉帧"

        while not rospy.is_shutdown() and (time.time() - start_time) < timeout_s:
            seq, connected, stamp, p1_tcp, p2_tcp = self._get_vision_snapshot()
            if seq > old_seq and connected and stamp >= stable_time:
                has_target, reason, _, p1_base, p2_base = self._current_target_snapshot()
                rospy.loginfo(
                    "收到稳定后的最新视觉帧: seq=%s, stamp=%.3f, p1_tcp=%s, p2_tcp=%s, valid=%s, reason=%s, p1_base=%s, p2_base=%s",
                    seq,
                    stamp,
                    p1_tcp,
                    p2_tcp,
                    has_target,
                    reason,
                    p1_base,
                    p2_base,
                )
                return True
            rospy.sleep(poll_s)

        rospy.logwarn("等待稳定后的最新视觉帧超时: %s", last_reason)
        return False

    def _current_target_snapshot(self):
        """
        判断当前是否有最新可采摘目标。

        消息触发模式下，goalCB 已经在收到 /custom_arm_data 时完成 TCP->base 转换，
        因此这里优先返回缓存的基坐标点，避免后续机械臂运动后重复使用新的 TCP 位姿重算。
        """
        with self.vision_lock:
            seq = int(self.vision_seq)
            connected = bool(self.vision_is_connected)
            stamp = float(self.vision_target_stamp)
            p1_tcp = list(getattr(self, "p1_tcp", [0.0, 0.0, 0.0]))
            p2_tcp = list(getattr(self, "p2_tcp", [0.0, 0.0, 0.0]))
            p1_base_cached = list(getattr(self, "p1_base", [0.0, 0.0, 0.0]))
            p2_base_cached = list(getattr(self, "p2_base", [0.0, 0.0, 0.0]))

        if not connected or stamp <= 0:
            return False, "尚未收到 /custom_arm_data 目标", seq, [0.0] * 3, [0.0] * 3

        if self._is_finite_point(p1_base_cached) and self._is_finite_point(p2_base_cached):
            if not self._is_zero_point(p1_base_cached) and not self._is_zero_point(p2_base_cached):
                target_side = self._infer_target_side_from_base_points(p1_base_cached, p2_base_cached)
                if self._points_reachable_for_side(p1_base_cached, p2_base_cached, target_side):
                    return (
                        True,
                        "使用已缓存的 /custom_arm_data 基坐标目标：p1_base={}，p2_base={}".format(
                            [round(float(v), 3) for v in p1_base_cached],
                            [round(float(v), 3) for v in p2_base_cached],
                        ),
                        seq,
                        p1_base_cached,
                        p2_base_cached,
                    )

        # 兼容旧逻辑：若只有 TCP 点缓存，则尝试使用当前 tool_position 重算。
        if not self._is_finite_point(p1_tcp) or not self._is_finite_point(p2_tcp):
            return False, "最新视觉点不是有限数", seq, p1_tcp, p2_tcp
        if self._is_zero_point(p1_tcp) or self._is_zero_point(p2_tcp):
            return False, "最新视觉点为空目标", seq, p1_tcp, p2_tcp

        try:
            p1_base = self._tcp_point_to_base(p1_tcp)
            p2_base = self._tcp_point_to_base(p2_tcp)
        except Exception as exc:
            return False, "TCP->base 坐标变换失败: {}".format(str(exc)), seq, p1_tcp, p2_tcp

        target_side = self._infer_target_side_from_base_points(p1_base, p2_base)
        if not self._points_reachable_for_side(p1_base, p2_base, target_side):
            return False, "最新目标转换到基坐标后不在可达工作空间内", seq, p1_base, p2_base

        with self.vision_lock:
            self.p1_base = list(p1_base)
            self.p2_base = list(p2_base)
            self.goal_data = list(p1_base)

        return (
            True,
            "最新视觉目标有效：p1_tcp={}，p2_tcp={}，p1_base={}，p2_base={}".format(
                p1_tcp,
                p2_tcp,
                [round(float(v), 3) for v in p1_base],
                [round(float(v), 3) for v in p2_base],
            ),
            seq,
            p1_base,
            p2_base,
        )

    def calculate_grasp_pose(self, p1_base, p2_base):
        """
        纯空间几何解算：输入基坐标系下两个视觉点，输出目标末端姿态。

        注意：这里输入必须已经是基坐标系坐标，单位为 mm；
        TCP 坐标到基坐标的转换在 goalCB() / _build_target_job_from_tcp_points() 中完成。

        新规则：
        1. 果梗方向由点位顺序严格确定：
            第二点 -> 第一点，即 p1 - p2
        2. 果梗方向严格对应机械臂末端坐标系 +Y 轴；
        3. 有有效视觉点时，使用视觉点；
        4. 无有效视觉点且 allow_pick_without_confirmed_target=True 时，
        保留并使用原有左右两侧固定测试点位。
        """

        def _to_point3(p):
            arr = np.array(p, dtype=float).reshape(-1)
            if arr.size < 3:
                raise ValueError("point dimension is less than 3")
            return arr[:3]

        def _is_valid_point(p):
            return np.all(np.isfinite(p)) and np.linalg.norm(p) > 1e-6

        p1 = _to_point3(p1_base)
        p2 = _to_point3(p2_base)

        input_valid = (
            _is_valid_point(p1)
            and _is_valid_point(p2)
            and np.linalg.norm(p1 - p2) > 1e-6
        )
        '''
        if not input_valid:
            if not getattr(self, "allow_pick_without_confirmed_target", False):
                raise ValueError(
                    "invalid p1/p2 from vision and fixed test point mode is disabled"
                )

            rospy.logwarn(
                "视觉输入点无效，但 allow_pick_without_confirmed_target=True，使用原有固定测试点位"
            )

            if not self.isRightPos:
                # 左侧原有固定测试点位
                p1 = np.array(
                    [102.715370, -487.775798, 533.487671],
                    dtype=float,
                )
                p2 = np.array(
                    [142.715370, -497.775798, 513.487671],
                    dtype=float,
                )
            else:
                # 右侧原有固定测试点位
                # 注意：这里不再交换 p1/p2。
                # 果梗方向统一由 p2 -> p1，即 p1 - p2 决定。
                p1 = np.array(
                    [102.715370, 487.775798, 533.487671],
                    dtype=float,
                )
                p2 = np.array(
                    [142.715370, 497.775798, 513.487671],
                    dtype=float,
                )
        '''
        rospy.loginfo("p1 : %f,%f,%f", p1[0], p1[1], p1[2])
        rospy.loginfo("p2 : %f,%f,%f", p2[0], p2[1], p2[2])

        p_mid = (p1 + p2) / 2.0

        # ============================================================
        # 核心要求：
        # 果梗方向 = 第二点指向第一点 = p1 - p2
        # 该方向严格对应机械臂末端坐标系 +Y 轴。
        # ============================================================
        stem_raw = p1 - p2
        stem_norm = np.linalg.norm(stem_raw)
        if stem_norm < 1e-6:
            raise ValueError("p1 and p2 are too close, cannot calculate stem direction")

        y_axis = stem_raw / stem_norm

        # 使用基坐标原点指向目标中点的方向构造接近方向。
        # 然后将其投影到与 y_axis 垂直的平面内，作为工具 Z 轴方向。
        cam_center = np.array([0.0, 0.0, 0.0], dtype=float)
        v_cam_to_stem = p_mid - cam_center

        z_axis = v_cam_to_stem - np.dot(v_cam_to_stem, y_axis) * y_axis
        z_norm = np.linalg.norm(z_axis)

        # 极端情况下，若目标方向与果梗方向几乎平行，则使用备用方向。
        if z_norm < 1e-6:
            fallback_dirs = [
                np.array([0.0, 0.0, 1.0], dtype=float),
                np.array([1.0, 0.0, 0.0], dtype=float),
                np.array([0.0, 1.0, 0.0], dtype=float),
            ]

            best_dir = None
            best_dot = None

            for candidate in fallback_dirs:
                dot_val = abs(float(np.dot(candidate, y_axis)))
                if best_dot is None or dot_val < best_dot:
                    best_dot = dot_val
                    best_dir = candidate

            z_axis = best_dir - np.dot(best_dir, y_axis) * y_axis
            z_norm = np.linalg.norm(z_axis)

            if z_norm < 1e-6:
                raise ValueError("cannot calculate valid approach direction")

        z_axis = z_axis / z_norm

        # 构造右手坐标系：
        # 已知 +Y = 果梗方向
        # 已知 +Z = 接近方向
        # 则 +X = +Y × +Z
        x_axis = np.cross(y_axis, z_axis)
        x_norm = np.linalg.norm(x_axis)
        if x_norm < 1e-6:
            raise ValueError("cannot calculate valid x axis")

        x_axis = x_axis / x_norm

        # 重新正交化 Z 轴，保证旋转矩阵严格正交。
        z_axis = np.cross(x_axis, y_axis)
        z_axis = z_axis / np.linalg.norm(z_axis)

        # R_target 的三列分别是末端坐标系 X/Y/Z 轴在基坐标系下的方向。
        # 第二列 R_target[:, 1] 严格等于果梗方向 y_axis。
        R_target = np.column_stack((x_axis, y_axis, z_axis))

        det_r = np.linalg.det(R_target)
        if det_r < 0.0:
            raise ValueError("invalid rotation matrix, determinant < 0")

        euler_target = self._rotation_from_matrix(R_target).as_euler("xyz", degrees=False)

        rospy.loginfo("p_mid : %f,%f,%f", p_mid[0], p_mid[1], p_mid[2])

        rospy.loginfo(
            "stem direction p2_to_p1 / tool +Y : %f,%f,%f",
            y_axis[0],
            y_axis[1],
            y_axis[2],
        )

        rospy.loginfo(
            "tool x_axis : %f,%f,%f",
            x_axis[0],
            x_axis[1],
            x_axis[2],
        )

        rospy.loginfo(
            "tool y_axis : %f,%f,%f",
            R_target[0, 1],
            R_target[1, 1],
            R_target[2, 1],
        )

        rospy.loginfo(
            "tool z_axis : %f,%f,%f",
            z_axis[0],
            z_axis[1],
            z_axis[2],
        )

        rospy.loginfo(
            "tool +Y and stem direction dot : %.6f",
            float(np.dot(R_target[:, 1], y_axis)),
        )

        rospy.loginfo(
            "euler_target : %f,%f,%f",
            math.degrees(euler_target[0]),
            math.degrees(euler_target[1]),
            math.degrees(euler_target[2]),
        )

        return p_mid, euler_target, z_axis

    def _switch_to_side(self, right_side, refresh_vision=True):
        """
        robot_states 严格握手式切换机位。

        流程：
        1. 关闭视觉接收；
        2. 清空旧视觉目标；
        3. 下发机位切换运动；
        4. 根据 /jaka_driver/robot_states 等待 motion_state 回到 Stop；
        5. 稳定后再次清空视觉目标；
        6. 打开视觉接收；
        7. 等待稳定后的新视觉帧。
        """
        with self.control_lock:
            target_side_name = "右" if right_side else "左"

            rospy.loginfo("准备切换/确认%s侧机位，关闭视觉接收", target_side_name)

            # 运动前关闭视觉，避免运动过程帧进入目标缓存。
            self._set_vision_accepting(False)
            self._clear_vision_target()

            # 已经在目标侧：不再固定 sleep，只确认 robot_states 已经处于 Stop。
            if right_side == self.isRightPos:
                rospy.loginfo("当前已经在%s侧机位，仅确认机器人状态", target_side_name)
                rospy.set_param(self.arm_is_right_pos_param, 1 if right_side else 0)

                if not self.wait_arm_motion_stable(
                    timeout_s=self.motion_done_timeout_s,
                    log_name="当前机位 Stop 状态确认",
                ):
                    rospy.logerr("当前%s侧机位 Stop 状态确认失败", target_side_name)
                    self._clear_vision_target()
                    self._set_vision_accepting(True)
                    return False

                self._clear_vision_target()
                rospy.loginfo("机械臂已在%s侧稳定，重新打开目标接收", target_side_name)
                self._set_vision_accepting(True)

                if refresh_vision:
                    old_seq = self._get_vision_snapshot()[0]
                    stable_time = self.last_side_stable_time or time.time()
                    self._wait_for_new_vision_after_stable(
                        old_seq,
                        stable_time,
                        self.vision_refresh_timeout,
                    )
                return True

            if right_side:
                rospy.loginfo("切换到右侧机位...")
                move_ok = self.robotJointMoveTolSequenceL(
                    [self.mid_joint_trans, self.init_joint_right],
                    wait=True,
                    timeout_s=self.motion_done_timeout_s,
                )
                if move_ok:
                    self.isRightPos = True
                    rospy.set_param(self.arm_is_right_pos_param, 1)
            else:
                rospy.loginfo("切换到左侧机位...")
                move_ok = self.robotJointMoveTolSequenceL(
                    [self.mid_joint_trans, self.init_joint_left],
                    wait=True,
                    timeout_s=self.motion_done_timeout_s,
                )
                if move_ok:
                    self.isRightPos = False
                    rospy.set_param(self.arm_is_right_pos_param, 0)

            if not move_ok:
                rospy.logwarn("切换到%s侧机位失败", target_side_name)
                self._clear_vision_target()
                self._set_vision_accepting(True)
                return False

            # robotJointMoveTolSequenceL(wait=True) 已经只在连续段末尾根据 robot_states 等到 Stop。
            # 这里再做一次 Stop 连续帧确认，替代原来的运动后固定 sleep。
            if not self.wait_arm_motion_stable(
                timeout_s=self.motion_done_timeout_s,
                log_name="机位切换后 Stop 状态确认",
            ):
                rospy.logerr("切换到%s侧后 Stop 状态确认失败", target_side_name)
                self._clear_vision_target()
                self._set_vision_accepting(True)
                return False

            # 关键：稳定后再次清空旧缓存。消息触发模式下不等待新的视觉帧。
            self._clear_vision_target()

            rospy.loginfo("机械臂已完成%s侧切换并稳定，重新打开目标接收", target_side_name)
            self._set_vision_accepting(True)

            if refresh_vision:
                old_seq = self._get_vision_snapshot()[0]
                stable_time = self.last_side_stable_time or time.time()
                # 兼容旧人工切换流程；消息触发采摘调用 refresh_vision=False，不会等待新视觉帧。
                self._wait_for_new_vision_after_stable(
                    old_seq,
                    stable_time,
                    self.vision_refresh_timeout,
                )
            return True

    def inputCallback(self, msg):
        """保留人工切换机位功能；自动采摘由 _auto_pick_loop 执行。"""
        if msg.data == 89:
            self._switch_to_side(LEFT_SIDE)
            return
        if msg.data == 88:
            self._switch_to_side(RIGHT_SIDE)
            return
        if msg.data == 87:
            self._switch_to_side(not self.isRightPos)
            return

        rospy.loginfo("自动采摘模式已启用，忽略 /vision/input 采摘触发: %s", msg.data)

    def _try_pick_current_side(self):
        """尝试采摘当前机位目标。成功采摘返回 True。"""
        if self.input_responding:
            rospy.logwarn("当前已有采摘任务执行中")
            return False

        if not self._auto_pick_task_enabled():
            rospy.loginfo(
                "当前 %s=%s，自动采摘未开启，跳过采摘",
                self.task_param_name,
                self._get_usr_task(),
            )
            return False
        
        side = "右" if self.isRightPos else "左"

        # 按需求：不再累计多帧，只读取最新一帧视觉目标，并立即做 TCP->base 坐标转换。
        has_target, reason, seq, p1, p2 = self._current_target_snapshot()
        rospy.loginfo("%s侧最新目标检查: %s", side, reason)

        if not has_target:
            rospy.loginfo("%s侧最新视觉目标无效，本轮不采摘", side)
            return False

        self.input_responding = True
        try:
            rospy.loginfo("开始执行%s侧采摘流程", side)

            # 执行采摘期间关闭视觉接收，避免运动过程帧污染下一轮判断。
            self._set_vision_accepting(False)

            ok = self.robotPickJoint(p1, p2)

            # 抓取流程结束后，根据 robot_states 再确认一次 Stop，替代原固定 sleep。
            if ok:
                ok = self.wait_arm_motion_stable(
                    timeout_s=self.motion_done_timeout_s,
                    log_name="采摘结束后 Stop 状态确认",
                )

            '''
            # 稳定后清空运动期间的视觉缓存。
            self._clear_vision_target()
            old_seq = self._get_vision_snapshot()[0]
            stable_time = self.last_side_stable_time or time.time()

            self._set_vision_accepting(True)

            if ok:
                self._wait_for_new_vision_after_stable(
                    old_seq,
                    stable_time,
                    self.vision_refresh_timeout,
                )
            '''
            return ok

        except Exception as exc:
            rospy.logerr("%s侧采摘流程异常: %s", side, str(exc))
            self._clear_vision_target()
            self._set_vision_accepting(True)
            return False
        finally:
            self.input_responding = False

    def _get_usr_task(self):
        """
        读取 /usr_task。

        默认值为 4：
        如果参数不存在或值非法，则默认认为当前不应自动采摘，避免节点启动后误动作。
        """
        try:
            return int(rospy.get_param(self.task_param_name, self.task_finish_value))
        except Exception as exc:
            rospy.logwarn_throttle(
                5.0,
                "读取 %s 失败或值非法，按 %s 处理: %s",
                self.task_param_name,
                self.task_finish_value,
                str(exc),
            )
            return self.task_finish_value

    def _auto_pick_task_enabled(self):
        """
        判断自动采摘是否开启。

        按需求：
        /usr_task != 4 时开启自动采摘；
        /usr_task == 4 时暂停自动采摘。
        """
        if not getattr(self, "use_usr_task_gate", False):
            return True
        return self._get_usr_task() != self.task_finish_value

    def _handle_auto_pick_disabled_by_task(self):
        """
        /usr_task == 4 时的暂停处理。

        暂停状态下：
        1. 清空旧视觉目标；
        2. 关闭本节点视觉接收；
        3. 设置 /vision_enable=False，让支持该参数的视觉节点也停止输出。
        """
        task = self._get_usr_task()

        if self.auto_pick_enabled_last is not False:
            rospy.loginfo(
                "检测到 %s=%s，自动采摘暂停，关闭视觉接收并清空目标",
                self.task_param_name,
                task,
            )
            self._clear_vision_target()
            self._set_vision_accepting(False)
            self.auto_pick_enabled_last = False
        else:
            rospy.loginfo_throttle(
                10.0,
                "自动采摘暂停中：%s=%s",
                self.task_param_name,
                task,
            )

    def _handle_auto_pick_enabled_by_task(self):
        """
        /usr_task != 4 时的开启或恢复处理。

        从暂停切换到开启时：
        1. 清除 station_finished 状态；
        2. 清空旧视觉目标；
        3. 根据 robot_states 确认机械臂处于 Stop；
        4. 打开视觉接收。
        """
        task = self._get_usr_task()

        if self.auto_pick_enabled_last is not True:
            rospy.loginfo(
                "检测到 %s=%s != %s，自动采摘开启/恢复",
                self.task_param_name,
                task,
                self.task_finish_value,
            )

            self.station_finished = False
            self._clear_vision_target()

            # 不再使用开启前固定 sleep；改为确认 robot_states 为 Stop。
            if not self.wait_arm_motion_stable(
                timeout_s=self.robot_ready_timeout_s,
                log_name="开启自动采摘前 Stop 状态确认",
            ):
                rospy.logwarn("开启自动采摘前未确认机器人 Stop，本轮暂不打开视觉")
                self.auto_pick_enabled_last = False
                return

            old_seq = self._get_vision_snapshot()[0]
            stable_time = self.last_side_stable_time or time.time()

            self._set_vision_accepting(True)
            self.auto_pick_enabled_last = True

            '''
            self._wait_for_new_vision_after_stable(
                old_seq,
                stable_time,
                self.vision_refresh_timeout,
            )
            '''

    def _finish_current_station(self):
        """两侧都没有可采摘果实时，下发当前工位完成。"""
        docked = rospy.get_param("/zeus_paths_multiple/docked", 0)
        seq = self._get_vision_snapshot()[0]

        rospy.loginfo(
            "两侧机位均无可采摘果实，当前工位采摘任务完成，设置 %s = %s",
            self.task_param_name,
            self.task_finish_value,
        )
        rospy.set_param(self.task_param_name, self.task_finish_value)

        self.station_finished = True
        self.finished_docked = docked
        self.finished_vision_seq = seq
        self.finished_time = time.time()

    def _station_can_resume(self):
        """
        判断是否允许自动采摘继续运行。

        新需求下不再根据 docked 或新视觉帧自动恢复；
        只有 /usr_task != 4 时才允许自动采摘流程运行。
        """
        if self._auto_pick_task_enabled():
            if self.station_finished:
                rospy.loginfo(
                    "检测到 %s != %s，清除当前工位完成状态，允许自动采摘恢复",
                    self.task_param_name,
                    self.task_finish_value,
                )
                self.station_finished = False
            return True

        return False

    def _execute_pick_target_job(self, target_job):
        """执行一个由 /custom_arm_data 触发的采摘任务。"""
        side = "右" if target_job.get("target_side", False) else "左"
        p1 = list(target_job.get("p1_base", [0.0, 0.0, 0.0]))
        p2 = list(target_job.get("p2_base", [0.0, 0.0, 0.0]))

        if self.input_responding:
            rospy.logwarn("当前已有采摘任务执行中，本目标稍后处理")
            self._publish_pick_target_result(target_job, False, "picker_busy")
            return False

        if not self._auto_pick_task_enabled():
            rospy.logwarn(
                "收到目标但任务门控未开启：%s=%s，目标暂不执行",
                self.task_param_name,
                self._get_usr_task(),
            )
            self._publish_pick_target_result(target_job, False, "task_gate_disabled")
            return False

        self.input_responding = True
        try:
            rospy.loginfo(
                "开始执行 /custom_arm_data 目标采摘: side=%s, "
                "p1_base_m=%s, p2_base_m=%s, p1_base_mm=%s, p2_base_mm=%s, tool_frame=%s",
                side,
                [round(float(v), 6) for v in target_job.get("p1_base_m", [0.0, 0.0, 0.0])],
                [round(float(v), 6) for v in target_job.get("p2_base_m", [0.0, 0.0, 0.0])],
                [round(float(v), 3) for v in p1],
                [round(float(v), 3) for v in p2],
                target_job.get("tool_frame_id", ""),
            )

            target_side = bool(target_job.get("target_side", self.isRightPos))
            if target_side != bool(self.isRightPos):
                if getattr(self, "auto_switch_side_for_target", False):
                    rospy.loginfo("目标属于%s侧，当前机位不同，先切换机位", side)
                    if not self._switch_to_side(target_side, refresh_vision=False):
                        rospy.logerr("切换到目标侧失败，放弃本目标")
                        arm_safe, arm_safe_reason = self._recover_arm_to_init_for_safety()
                        self._publish_pick_target_result(
                            target_job,
                            False,
                            "switch_side_failed",
                            arm_safe=arm_safe,
                            arm_safe_reason=arm_safe_reason,
                        )
                        return False
                else:
                    rospy.logwarn(
                        "目标推断为%s侧，但当前机械臂在%s侧；auto_switch_side_for_target=False，仍按当前机位尝试采摘",
                        side,
                        "右" if self.isRightPos else "左",
                    )

            # 执行采摘期间不再清空队列，也不等待新的视觉帧；队列中的其他目标会在本目标完成后继续处理。
            self._set_vision_accepting(False)
            pick_ok = self.robotPickJoint(p1, p2)

            motion_ok = True
            if pick_ok:
                motion_ok = self.wait_arm_motion_stable(
                    timeout_s=self.motion_done_timeout_s,
                    log_name="单目标采摘结束后 Stop 状态确认",
                )
            ok = bool(pick_ok and motion_ok)

            arm_safe, arm_safe_reason = self._recover_arm_to_init_for_safety()
            result_ok = bool(ok and arm_safe)

            if result_ok:
                self.processed_target_count = int(getattr(self, "processed_target_count", 0)) + 1
                rospy.loginfo(
                    "单目标采摘完成，累计完成=%s，剩余队列=%s",
                    self.processed_target_count,
                    self._pending_target_count(),
                )
            else:
                rospy.logerr(
                    "单目标采摘失败或机械臂未安全回位，pick_ok=%s, motion_ok=%s, arm_safe=%s, reason=%s, 剩余队列=%s",
                    pick_ok,
                    motion_ok,
                    arm_safe,
                    arm_safe_reason,
                    self._pending_target_count(),
                )

            self._publish_pick_target_result(
                target_job,
                result_ok,
                "pick_done" if result_ok else "pick_failed_or_arm_not_safe",
                arm_safe=arm_safe,
                arm_safe_reason=arm_safe_reason,
            )
            return result_ok

        except Exception as exc:
            rospy.logerr("执行 /custom_arm_data 目标采摘异常: %s", str(exc))
            arm_safe, arm_safe_reason = self._recover_arm_to_init_for_safety()
            self._publish_pick_target_result(
                target_job,
                False,
                "exception: {}".format(str(exc)),
                arm_safe=arm_safe,
                arm_safe_reason=arm_safe_reason,
            )
            return False
        finally:
            self.input_responding = False
            self._clear_vision_target()
            self._set_vision_accepting(True)

    def _auto_pick_loop(self):
        """
        消息触发采摘主循环。

        /custom_arm_data 每个目标只发布一次，不能再采用旧逻辑中的“当前侧无果->切换另一侧->等待新视觉帧”。
        本循环只做一件事：从队列取出已经在回调中完成 TCP->base 转换的目标，并逐个执行采摘。
        """
        hz = max(1.0, float(getattr(self, "custom_target_process_rate_hz", 20.0)))
        rate = rospy.Rate(hz)
        rospy.loginfo(
            "消息触发采摘主循环启动：topic=%s, use_usr_task_gate=%s, queue_max=%s",
            getattr(self, "custom_arm_data_topic", "/custom_arm_data"),
            getattr(self, "use_usr_task_gate", False),
            getattr(self, "custom_target_queue_max_len", 10),
        )

        while not rospy.is_shutdown():
            if getattr(self, "use_usr_task_gate", False) and not self._auto_pick_task_enabled():
                self._handle_auto_pick_disabled_by_task()
                rate.sleep()
                continue

            target_job = self._pop_pick_target()
            if target_job is None:
                if getattr(self, "use_usr_task_gate", False):
                    self._handle_auto_pick_enabled_by_task()
                rospy.loginfo_throttle(
                    10.0,
                    "等待 /custom_arm_data 目标消息，已完成=%s，已丢弃=%s",
                    getattr(self, "processed_target_count", 0),
                    getattr(self, "dropped_target_count", 0),
                )
                rate.sleep()
                continue

            self._execute_pick_target_job(target_job)
            rate.sleep()

    def check_reachable(self, matrix, matrix2):
        """检查目标位置是否在当前机位工作空间内。"""
        if matrix is None or matrix2 is None or len(matrix) < 3 or len(matrix2) < 3:
            return False

        if getattr(self, "use_custom_workspace", False):
            return self._points_reachable_in_custom_workspace(matrix, matrix2)

        if not self.isRightPos:
            return (
                -100 <= matrix[0] <= 600 and -820 <= matrix[1] <= -400 and 80 <= matrix[2] <= 810
                and -100 <= matrix2[0] <= 600 and -820 <= matrix2[1] <= -400 and 80 <= matrix2[2] <= 810
            )

        return (
            -100 <= matrix[0] <= 600 and 400 <= matrix[1] <= 820 and 80 <= matrix[2] <= 810
            and -100 <= matrix2[0] <= 600 and 400 <= matrix2[1] <= 820 and 80 <= matrix2[2] <= 810
        )

    def robotMatrix2Pose(self, matrix):
        target = list(matrix[:3]) if matrix is not None and len(matrix) >= 3 else list(self.goal_data[:3])
        rospy.loginfo("matrix2: %f,%f,%f", target[0], target[1], target[2])

        if self.object_compensation_xyz is None:
            self.object_compensation_xyz = (0.0, 0.0, 0.0)

        x = target[0] + self.object_compensation_xyz[0]
        y = target[1] + self.object_compensation_xyz[1]
        z = target[2] + self.object_compensation_xyz[2]

        RX = math.radians(31)
        RY = math.radians(36)
        RZ = math.radians(-22)
        rospy.loginfo("target pose : %f,%f,%f,%f,%f,%f", x, y, z, RX, RY, RZ)
        return x, y, z, RX, RY, RZ

    def robotMatrix2PrePose(self, pose):
        """根据 object_pre_pose_offset 生成预抓取位姿。保留原 robotPick 调用依赖。"""
        if self.object_pre_pose_offset is None:
            self.object_pre_pose_offset = (0.0, 0.0, 100.0, 0.0, 0.0, 0.0)
        return tuple(float(pose[i]) + float(self.object_pre_pose_offset[i]) for i in range(6))

    def cal_target_pose_array(self, input_pose):
        x, y, z, roll_input, pitch_input, yaw_input = input_pose
        angles = self.generate_sorted_angles(math.radians(-15), math.radians(15), math.radians(1.5))
        target_poses = []

        for axis in ["x"]:
            for angle in angles:
                if axis == "x":
                    target_poses.append((x, y, z, roll_input + angle, pitch_input, yaw_input))
                else:
                    raise ValueError("Invalid axis. Axis must be 'x', 'y', or 'z'.")

        return target_poses

    def generate_sorted_angles(self, start, end, step):
        """生成 0, +step, -step, +2step, -2step ... 排序的角度数组。"""
        if step == 0:
            raise ValueError("step cannot be zero")

        base_angles = np.arange(start, end + step / 2.0, step)
        zero = [0.0] if np.any(np.isclose(base_angles, 0.0, atol=abs(step) / 10.0)) else []
        positives = sorted([float(x) for x in base_angles if x > 0], key=abs)
        negatives = sorted([float(x) for x in base_angles if x < 0], key=abs)

        sorted_angles = list(zero)
        count = max(len(positives), len(negatives))
        for i in range(count):
            if i < len(positives):
                sorted_angles.append(positives[i])
            if i < len(negatives):
                sorted_angles.append(negatives[i])

        return np.array(sorted_angles, dtype=float)

    def robotGripper3ChangeStateL(self, state):
        """
        夹爪动作封装。

        说明：夹爪当前只有发布型指令，没有类似 robot_states 的到位反馈，
        因此这里仍保留 gripper_action_wait_s 的短暂等待。
        """
        rospy.loginfo("come in robotGripper#ChangeStateL")
        if self.cancel_responding:
            rospy.logerr("夹爪动作被 cancel_responding 阻止")
            return False
        if not self.robotGripper3ChangeState(state):
            rospy.logerr("夹爪命令发布失败")
            return False
        ok = self.wait_for_gripper_command_result(state)
        rospy.loginfo("out robotGripper#ChangeStateL, ok=%s", ok)
        return ok

    def robotPickJoint(self, matrix, matrix2):
        """
        IK 关节空间抓取流程。

        修改点：
        - 每一次机械臂运动都使用 robot_states 判断 motion_state 是否回到 Stop；
        - 不再给机械臂运动步骤附加固定 sleep；
        - 任一运动或夹爪动作失败，立即 return False，停止后续流程。
        """
        side = "右" if self.isRightPos else "左"
        rospy.loginfo("开始%s侧简化抓取流程", side)

        try:
            p_mid, euler_target, z_axis = self.calculate_grasp_pose(matrix, matrix2)
        except ValueError as exc:
            rospy.logerr("calculate_grasp_pose failed: %s", str(exc))
            return False

        # p_mid 是两个视觉点的中点，即期望“末端执行器末端/工具尖端”到达的目标点。
        # 由于工具 +Z 轴沿末端指向外侧，工具尖端 = TCP 原点 + L * tool_z_axis。
        # 因此下发给机械臂的 TCP 原点目标应为：target_tcp = target_tip - L * tool_z_axis。
        tip_target_pos = np.array(p_mid, dtype=float)
        z_axis_np = np.array(z_axis, dtype=float)
        tool_length_mm = float(getattr(self, "end_effector_length_mm", 165.0))
        pre_offset_mm = float(getattr(self, "pre_grasp_offset_mm", 100.0))
        tcp_target_pos = tip_target_pos - tool_length_mm * z_axis_np

        pose = [
            float(tcp_target_pos[0]), float(tcp_target_pos[1]), float(tcp_target_pos[2]),
            float(euler_target[0]), float(euler_target[1]), float(euler_target[2]),
        ]

        rospy.loginfo("tip_target_pos(base, mm): %s", [float(v) for v in tip_target_pos])
        rospy.loginfo("tool_z_axis(base): %s", [float(v) for v in z_axis_np])
        rospy.loginfo("end_effector_length_mm: %.3f", tool_length_mm)
        rospy.loginfo("tcp_target_pose: %s", pose)

        target_pose_lists = self.cal_target_pose_array(pose)
        ref_j = []
        target_joints = self.solve_ik_for_pose(pose, ref_j)
        pre_target_joints = []
        final_pose = pose

        if len(target_joints):
            pre_pos = tcp_target_pos - pre_offset_mm * z_axis_np
            pre_pose = list(pre_pos) + list(euler_target)
            rospy.loginfo("pre_pose: %s", pre_pose)
            rospy.loginfo("pose: %s", pose)
            pre_target_joints = self.solve_ik_for_pose(pre_pose, ref_j)
        else:
            for index, target_pose in enumerate(target_pose_lists, start=1):
                target_joints = self.solve_ik_for_pose(target_pose, ref_j)
                if len(target_joints):
                    final_pose = list(target_pose)
                    pre_pos = np.array(target_pose[:3], dtype=float) - pre_offset_mm * z_axis_np
                    pre_pose = list(pre_pos) + list(target_pose[3:6])
                    rospy.loginfo("pre_pose: %s", pre_pose)
                    rospy.loginfo("pose: %s", final_pose)
                    pre_target_joints = self.solve_ik_for_pose(pre_pose, ref_j)
                    rospy.loginfo("逆运动学求解成功, target_pose: %s", target_pose)
                    break
                rospy.loginfo("无法求解目标位姿 i=%s 的逆运动学", index)

        if not (len(pre_target_joints) and len(target_joints)):
            rospy.logerr("%s侧目标逆运动学求解失败，无法采摘", side)
            return False

        # self._start_recording("pick_joint_{}".format("right" if self.isRightPos else "left"))

        try:
            if not self.robotJointMoveTolSequenceL(
                [pre_target_joints, target_joints],
                wait=True,
                timeout_s=self.motion_done_timeout_s,
            ):
                rospy.logerr("%s侧连续运动到预抓取点/抓取点失败，停止抓取流程", side)
                return False

            rospy.loginfo("start close")
            if not self.robotGripper3ChangeStateL(1):
                rospy.logerr("%s侧夹爪闭合失败或被中断，停止抓取流程", side)
                return False
            rospy.loginfo("close over")

            # ============================================================
            # 新增过渡点：
            # 机械臂到达采摘点并夹爪闭合后，不再直接去放果位置；
            # 先退回采摘点的前置点 pre_target_joints，作为避障/退果过渡点；
            # 到达该过渡点后，再运动到果实放置位置。
            # ============================================================
            rospy.loginfo("%s侧夹爪闭合后，先退回采摘前置过渡点...", side)
            if not self.robotJointMoveTolL(
                pre_target_joints,
                wait=True,
                timeout_s=self.motion_done_timeout_s,
            ):
                rospy.logerr("%s侧退回采摘前置过渡点失败，停止抓取流程", side)
                return False
            rospy.loginfo("%s侧已到达采摘前置过渡点，准备运动到放果位置", side)

            if self.isRightPos:
                rospy.loginfo("右侧放果位置...")
                if not self.robotJointMoveTolL(
                    self.place_joint_right,
                    wait=True,
                    timeout_s=self.motion_done_timeout_s,
                ):
                    rospy.logerr("右侧运动到放果位置失败，停止抓取流程")
                    return False
            else:
                rospy.loginfo("左侧放果位置...")
                if not self.robotJointMoveTolL(
                    self.place_joint_left,
                    wait=True,
                    timeout_s=self.motion_done_timeout_s,
                ):
                    rospy.logerr("左侧运动到放果位置失败，停止抓取流程")
                    return False

            rospy.loginfo("张开夹爪...")
            if not self.robotGripper3ChangeStateL(0):
                rospy.logerr("%s侧夹爪张开失败或被中断，停止抓取流程", side)
                return False

            if self.isRightPos:
                rospy.loginfo("回右侧初始位置...")
                if not self.robotJointMoveTolL(
                    self.init_joint_right,
                    wait=True,
                    timeout_s=self.motion_done_timeout_s,
                ):
                    rospy.logerr("回右侧初始位置失败，停止抓取流程")
                    return False
                rospy.set_param(self.arm_is_right_pos_param, 1)
            else:
                rospy.loginfo("回左侧初始位置...")
                if not self.robotJointMoveTolL(
                    self.init_joint_left,
                    wait=True,
                    timeout_s=self.motion_done_timeout_s,
                ):
                    rospy.logerr("回左侧初始位置失败，停止抓取流程")
                    return False
                rospy.set_param(self.arm_is_right_pos_param, 0)

            if not self.wait_arm_motion_stable(
                timeout_s=self.motion_done_timeout_s,
                log_name="抓取流程结束 Stop 状态确认",
            ):
                rospy.logerr("%s侧抓取结束后的 Stop 状态确认失败", side)
                return False

            rospy.loginfo("%s侧抓取流程完成，final_pose=%s", side, final_pose)
            return True

        finally:
            self._stop_and_save_recording("joint_record")
