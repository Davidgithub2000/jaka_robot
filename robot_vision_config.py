#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_vision_config.py

配置和参数解析模块。

本模块只负责：
1. 初始化成员变量；
2. 解析 ROS/YAML 参数；
3. 明确关节角单位，并把预设关节角统一转换为 rad。
"""

import os
import time
from threading import Lock

import rospy


INVALID_6D_VALUE = (-1, -1, -1, -1, -1, -1)
RIGHT_SIDE = True
LEFT_SIDE = False


class RobotVisionConfigMixin:
    def _join_ros_name(self, prefix, suffix):
        """Join a ROS namespace/prefix and a relative resource name."""
        prefix = str(prefix or "").strip().rstrip("/")
        suffix = str(suffix or "").strip().lstrip("/")
        if not prefix:
            return "/" + suffix
        return prefix + "/" + suffix

    def _init_parameters(self):
        """初始化类成员变量。"""
        self.vision_is_connected = False
        self.gripper_is_connected = False
        self.is_eye_in_hand = False
        self.isEyeInHand = False
        self.input_responding = False
        self.cancel_responding = False

        # robot_states 状态缓存。
        # motion_state: Stop=0, Pause=1, EmeStop=2, Running=3, Error=4。
        # power_state: 上电=1；servo_state: 伺服使能=1；collision_state: 碰撞报警=1。
        self.robot_state_lock = Lock()
        self.robot_state_received = False
        self.robot_status = False
        self.motion_state = 0
        self.power_state = 0
        self.servo_state = 0
        self.collision_state = 0
        self._motion_state_stamp = 0.0
        self._last_motion_command_stamp = 0.0

        self.driver_ns = str(
            rospy.get_param("robot_vision_wrapper/driver_ns", "/jaka_driver")
        ).strip().rstrip("/")
        if not self.driver_ns:
            self.driver_ns = "/jaka_driver"
        self.wrapper_input_topic = rospy.get_param(
            "robot_vision_wrapper/wrapper_input_topic", "/vision/input"
        )
        self.robot_states_topic = rospy.get_param(
            "robot_vision_wrapper/robot_states_topic",
            self._join_ros_name(self.driver_ns, "robot_states"),
        )
        self.joint_state_topic = rospy.get_param(
            "robot_vision_wrapper/joint_state_topic",
            self._join_ros_name(self.driver_ns, "joint_state"),
        )
        self.linear_move_service = rospy.get_param(
            "robot_vision_wrapper/linear_move_service",
            self._join_ros_name(self.driver_ns, "linear_move"),
        )
        self.linear_move_tol_service = rospy.get_param(
            "robot_vision_wrapper/linear_move_tol_service",
            self._join_ros_name(self.driver_ns, "linear_move_tol"),
        )
        self.joint_move_service = rospy.get_param(
            "robot_vision_wrapper/joint_move_service",
            self._join_ros_name(self.driver_ns, "joint_move"),
        )
        self.joint_move_tol_service = rospy.get_param(
            "robot_vision_wrapper/joint_move_tol_service",
            self._join_ros_name(self.driver_ns, "joint_move_tol"),
        )
        self.ik_service = rospy.get_param(
            "robot_vision_wrapper/ik_service",
            self._join_ros_name(self.driver_ns, "get_ik"),
        )
        self.fk_service = rospy.get_param(
            "robot_vision_wrapper/fk_service",
            self._join_ros_name(self.driver_ns, "get_fk"),
        )

        # 自动采摘控制相关
        self.auto_pick_thread = None
        self.control_lock = Lock()
        self.vision_lock = Lock()
        self.vision_seq = 0
        self.vision_target_stamp = 0.0
        self.vision_wait_timeout = 2.0
        self.vision_refresh_timeout = 4.0
        self.vision_feedback_max_age_s = 6.0
        self.target_empty_epsilon = 1e-6

        # 消息触发模式：总控节点每个目标只发布一次 /custom_arm_data。
        # 本节点必须在回调中立即保存目标，不能依赖后续视觉帧。
        self.custom_arm_data_topic = rospy.get_param(
            "robot_vision_wrapper/custom_arm_data_topic", "/custom_arm_data"
        )
        self.custom_target_queue_max_len = int(
            rospy.get_param("robot_vision_wrapper/custom_target_queue_max_len", 10)
        )
        self.custom_target_process_rate_hz = float(
            rospy.get_param("robot_vision_wrapper/custom_target_process_rate_hz", 20.0)
        )
        self.use_usr_task_gate = self._parse_bool_param(
            "robot_vision_wrapper/use_usr_task_gate", False
        )
        self.infer_target_side_from_base_y = self._parse_bool_param(
            "robot_vision_wrapper/infer_target_side_from_base_y", True
        )
        self.auto_switch_side_for_target = self._parse_bool_param(
            "robot_vision_wrapper/auto_switch_side_for_target", False
        )
        self.use_custom_workspace = self._parse_bool_param(
            "robot_vision_wrapper/use_custom_workspace", False
        )
        self.workspace_x_min = float(
            rospy.get_param("robot_vision_wrapper/workspace_x_min", -100.0)
        )
        self.workspace_x_max = float(
            rospy.get_param("robot_vision_wrapper/workspace_x_max", 600.0)
        )
        self.workspace_y_min = float(
            rospy.get_param("robot_vision_wrapper/workspace_y_min", -820.0)
        )
        self.workspace_y_max = float(
            rospy.get_param("robot_vision_wrapper/workspace_y_max", 820.0)
        )
        self.workspace_z_min = float(
            rospy.get_param("robot_vision_wrapper/workspace_z_min", 80.0)
        )
        self.workspace_z_max = float(
            rospy.get_param("robot_vision_wrapper/workspace_z_max", 810.0)
        )
        self.target_queue_lock = Lock()
        self.pending_pick_targets = []
        self.processed_target_count = 0
        self.dropped_target_count = 0

        # 视觉目标和 TCP 位姿单位配置。
        # 总控节点 /custom_arm_data 发布的目标点单位为 m。
        # 中间坐标变换统一使用 m + rad；进入原有运动/IK流程前再转换为 mm + rad。
        self.vision_target_topic = rospy.get_param(
            "robot_vision_wrapper/vision_target_topic", self.custom_arm_data_topic
        )
        self.vision_target_unit = str(
            rospy.get_param("robot_vision_wrapper/vision_target_unit", "m")
        ).strip().lower()

        # JAKA 驱动当前 TCP 位姿反馈：/jaka_driver/tool_position，TwistStamped。
        # twist.linear 为 TCP 在基坐标系下的位置；twist.angular 为 TCP 姿态欧拉角。
        self.tool_position_topic = rospy.get_param(
            "robot_vision_wrapper/tool_position_topic",
            self._join_ros_name(self.driver_ns, "tool_position"),
        )
        self.tool_position_pos_unit = str(
            rospy.get_param("robot_vision_wrapper/tool_position_pos_unit", "mm")
        ).strip().lower()
        self.tool_position_rpy_unit = str(
            rospy.get_param("robot_vision_wrapper/tool_position_rpy_unit", "deg")
        ).strip().lower()
        self.tool_position_euler_order = str(
            rospy.get_param("robot_vision_wrapper/tool_position_euler_order", "xyz")
        ).strip().lower()
        # tool_position.twist.angular 的旋转表示方式：
        # - euler：按欧拉角/固定轴 RPY 处理，默认与 JAKA Move/GetIK 的 [rx, ry, rz] 保持一致；
        # - rotvec：按旋转向量处理，方向为旋转轴，模长为旋转角 rad。
        self.tool_position_rotation_type = str(
            rospy.get_param("robot_vision_wrapper/tool_position_rotation_type", "euler")
        ).strip().lower()

        # SciPy 约定：小写轴序列表示固定轴/外旋，常用于机器人 RPY；
        # 大写轴序列表示动轴/内旋。如现场驱动采用内旋，可置 true。
        self.tool_position_euler_intrinsic = self._parse_bool_param(
            "robot_vision_wrapper/tool_position_euler_intrinsic", False
        )

        self.tool_position_max_age_s = float(
            rospy.get_param("robot_vision_wrapper/tool_position_max_age_s", 1.0)
        )
        # /custom_arm_data 每个目标只发一次；若回调瞬间还没有新鲜 TCP 位姿，
        # 允许短暂等待 tool_position 刷新，避免直接丢弃目标。
        self.tool_position_wait_timeout_s = float(
            rospy.get_param("robot_vision_wrapper/tool_position_wait_timeout_s", 0.5)
        )
        self.tool_position_wait_poll_s = float(
            rospy.get_param("robot_vision_wrapper/tool_position_wait_poll_s", 0.01)
        )

        self.tool_position_lock = Lock()
        self.tool_position_received = False
        self.tool_position_stamp = 0.0
        self.tool_position_frame_id = ""
        # 内部统一保存为 [x_m, y_m, z_m, rx_rad, ry_rad, rz_rad]。
        self.tool_position_pose = [0.0] * 6

        # 末端执行器长度：16.5 cm = 165 mm。
        # 约定：工具坐标系 +Z 轴沿末端向外，执行器末端点 = TCP 原点 + L * tool_z_axis。
        self.end_effector_length_mm = float(
            rospy.get_param("robot_vision_wrapper/end_effector_length_mm", 165.0)
        )
        # 预抓取点沿 -tool_z_axis 后退距离，保留原代码 100 mm 行为并参数化。
        self.pre_grasp_offset_mm = float(
            rospy.get_param("robot_vision_wrapper/pre_grasp_offset_mm", 100.0)
        )

        # ------------------------------------------------------------------
        # 视觉多帧缓冲与稳定目标确认参数【已停用】。
        # 保留这些参数是为了兼容旧 launch/yaml 和旧函数引用；主流程不再使用。
        # ------------------------------------------------------------------
        # 多帧累计窗口已停用，主流程只使用最新一帧视觉目标。
        self.vision_accumulate_window_s = float(
            rospy.get_param("robot_vision_wrapper/vision_accumulate_window_s", 1.0)
        )

        # 旧目标缓冲区中样本的最大有效时间【已停用】。
        self.vision_target_buffer_max_age_s = float(
            rospy.get_param("robot_vision_wrapper/vision_target_buffer_max_age_s", 2.0)
        )

        # 旧目标缓冲区最大样本数【已停用】。
        self.vision_target_buffer_max_len = int(
            rospy.get_param("robot_vision_wrapper/vision_target_buffer_max_len", 50)
        )

        # 旧多帧聚类确认帧数【已停用】。
        self.vision_target_min_confirm_frames = int(
            rospy.get_param("robot_vision_wrapper/vision_target_min_confirm_frames", 3)
        )

        # 旧聚类半径，单位 mm【已停用】。
        self.vision_target_cluster_radius_mm = float(
            rospy.get_param("robot_vision_wrapper/vision_target_cluster_radius_mm", 60.0)
        )

        # 等待最新视觉帧时的轮询间隔。
        self.vision_sample_poll_s = float(
            rospy.get_param("robot_vision_wrapper/vision_sample_poll_s", 0.02)
        )

        # 调试开关保留：当前版本最新视觉目标无效时不执行采摘。
        self.allow_pick_without_confirmed_target = self._parse_bool_param(
            "robot_vision_wrapper/allow_pick_without_confirmed_target", False
        )

        # 旧视觉目标多帧缓冲区【已停用】。
        self.vision_target_buffer = []

        # 可选任务门控：默认关闭。
        # use_usr_task_gate=False：收到 /custom_arm_data 即可触发采摘；
        # use_usr_task_gate=True：仍沿用 /usr_task != 4 才允许采摘。
        self.task_param_name = rospy.get_param(
            "robot_vision_wrapper/task_param_name", "/usr_task"
        )
        self.task_finish_value = int(
            rospy.get_param("robot_vision_wrapper/task_finish_value", 4)
        )
        self.vision_enable_param = rospy.get_param(
            "robot_vision_wrapper/vision_enable_param", "/vision_enable"
        )
        self.arm_is_right_pos_param = rospy.get_param(
            "robot_vision_wrapper/arm_is_right_pos_param", "/ArmIsRightPos"
        )
        self.auto_pick_enabled_last = None

        # 消息触发模式下不能因为机械臂运动关闭接收而丢掉单次目标消息；
        # 因此 goalCB 不再用 vision_accepting 作为丢弃条件。该变量仅保留兼容旧逻辑/日志。
        self.vision_accepting = True
        self.last_side_stable_time = 0.0

        # ------------------------------------------------------------------
        # robot_states 到位判断模式：
        # 机械臂运动完成不再使用固定 sleep，而是根据 /jaka_driver/robot_states
        # 中的 motion_state/power_state/servo_state/collision_state 判断。
        # ------------------------------------------------------------------
        self.use_sleep_motion_wait = False

        # 以下 sleep_s 参数仅为兼容旧函数签名保留，机械臂运动等待不再使用。
        self.default_joint_move_sleep_s = 0.0
        self.default_cartesian_move_sleep_s = 0.0
        self.switch_step_sleep_s = 0.0
        self.pick_step_sleep_s = 0.0
        self.place_step_sleep_s = 0.0
        self.arm_settle_sleep_s = 0.0
        self.vision_enable_delay_s = 0.0

        # 等待机器人 ready 的超时，防止 robot_states 异常时永久阻塞。
        self.robot_ready_timeout_s = float(
            rospy.get_param("robot_vision_wrapper/robot_ready_timeout_s", 3.0)
        )

        # robot_states 状态消息最大允许间隔；超过该时间认为状态失效。
        self.robot_state_timeout_s = float(
            rospy.get_param("robot_vision_wrapper/robot_state_timeout_s", 1.5)
        )

        # 运动指令下发后，等待 motion_state 回到 Stop 的最大时间。
        self.motion_done_timeout_s = float(
            rospy.get_param("robot_vision_wrapper/motion_done_timeout_s", 30.0)
        )

        # 连续收到多少帧 Stop 后判定机械臂已经稳定停止。
        self.motion_stop_confirm_count = int(
            rospy.get_param("robot_vision_wrapper/motion_stop_confirm_count", 3)
        )

        # 运动指令下发后，在这段时间内优先等待 Running，避免把指令前后的旧 Stop 误判为完成。
        self.motion_start_grace_s = float(
            rospy.get_param("robot_vision_wrapper/motion_start_grace_s", 0.3)
        )

        # robot_states 轮询间隔。该 sleep 只用于等待状态回调刷新，不是固定运动等待时间。
        self.motion_state_poll_s = float(
            rospy.get_param("robot_vision_wrapper/motion_state_poll_s", 0.02)
        )

        # 等待运动服务出现的超时时间。
        self.motion_service_timeout_s = float(
            rospy.get_param("robot_vision_wrapper/motion_service_timeout_s", 1.0)
        )

        # 夹爪命令发布后优先等待 /multi_gripper/interpreted_state 反馈；可关闭反馈等待退回固定等待。
        self.gripper_action_wait_s = float(
            rospy.get_param("robot_vision_wrapper/gripper_action_wait_s", 1.0)
        )
        self.gripper_cmd_topic = rospy.get_param(
            "robot_vision_wrapper/gripper_cmd_topic", "/multi_gripper/cmd"
        )
        self.jaka_gripper_id = int(
            rospy.get_param("robot_vision_wrapper/jaka_gripper_id", 1)
        )
        self.require_gripper_subscriber = self._parse_bool_param(
            "robot_vision_wrapper/require_gripper_subscriber", True
        )
        self.gripper_wait_subscriber_timeout_s = float(
            rospy.get_param("robot_vision_wrapper/gripper_wait_subscriber_timeout_s", 1.0)
        )
        self.gripper_cmd_repeat = int(
            rospy.get_param("robot_vision_wrapper/gripper_cmd_repeat", 1)
        )
        self.gripper_cmd_repeat_interval_s = float(
            rospy.get_param("robot_vision_wrapper/gripper_cmd_repeat_interval_s", 0.05)
        )
        self.gripper_feedback_topic = rospy.get_param(
            "robot_vision_wrapper/gripper_feedback_topic",
            "/multi_gripper/interpreted_state",
        )
        self.use_gripper_feedback_wait = self._parse_bool_param(
            "robot_vision_wrapper/use_gripper_feedback_wait", True
        )
        self.gripper_feedback_timeout_s = float(
            rospy.get_param("robot_vision_wrapper/gripper_feedback_timeout_s", 3.0)
        )
        self.gripper_feedback_poll_s = float(
            rospy.get_param("robot_vision_wrapper/gripper_feedback_poll_s", 0.02)
        )
        self.gripper_close_success_states = rospy.get_param(
            "robot_vision_wrapper/gripper_close_success_states", "close"
        )
        self.gripper_open_success_states = rospy.get_param(
            "robot_vision_wrapper/gripper_open_success_states", "open"
        )
        self.gripper_feedback_lock = Lock()
        self.gripper_feedback_state_by_id = {}
        self.gripper_feedback_stamp_by_id = {}

        # 关节角单位统一说明：
        # 1. 程序内部、IK 返回值、下发给 JAKA Move 服务的关节角一律使用 rad；
        # 2. YAML/launch 中的 ref/init/place 预设关节角默认按 rad 读取，保持旧程序“原样下发”的行为；
        # 3. 如果参数文件中写的是 degree，请增加：robot_vision_wrapper/joint_param_unit: degree。
        self.joint_param_unit = self._normalize_joint_unit_name(
            rospy.get_param("robot_vision_wrapper/joint_param_unit", "rad")
        )

        # 保留原参数名，避免其他位置引用时报错；当前由 robot_states 等待替代固定 sleep。
        self.arm_settle_hold_s = self.arm_settle_sleep_s
        self.arm_settle_timeout_s = self.motion_done_timeout_s

        self.station_finished = False
        self.finished_docked = None
        self.finished_vision_seq = 0
        self.finished_time = 0.0
        self.station_finish_hold_s = 5.0

        # 机器人状态
        self.joint_positions = [0.0] * 6
        self.joint_state_stamp = 0.0
        self.joint_state_max_age_s = float(
            rospy.get_param("robot_vision_wrapper/joint_state_max_age_s", 1.0)
        )
        self.arm_safe_joint_tolerance_rad = float(
            rospy.get_param("robot_vision_wrapper/arm_safe_joint_tolerance_rad", 0.08)
        )
        self.recover_to_init_on_pick_failure = self._parse_bool_param(
            "robot_vision_wrapper/recover_to_init_on_pick_failure", True
        )
        self.end_effector_pose = [0.0] * 6
        # 视觉节点传来的两点是 TCP 坐标系下的点；采摘前再转换到基坐标系。
        self.p1_tcp = [0.0] * 3
        self.p2_tcp = [0.0] * 3
        self.p1_base = [0.0] * 3
        self.p2_base = [0.0] * 3
        self.goal_data = [0.0] * 3

        # 运动参数
        self.object_compensation_xyz = None
        self.use_input_rpy = None
        self.object_rotate_rpy = None
        self.object_pre_pose_offset = None
        self.auto_place = None
        self.initial_is_right_pos = self._parse_bool_param(
            "robot_vision_wrapper/initial_is_right_pos", False
        )
        self.isRightPos = self.initial_is_right_pos  # True: 右侧机位, False: 左侧机位

        # 是否在节点启动时强制把 /usr_task 置为完成/暂停。
        # 消息触发模式默认不再强制置 4，避免总控只发一次目标时被任务门控阻断。
        if self._parse_bool_param("robot_vision_wrapper/set_usr_task_pause_on_start", False):
            rospy.set_param(self.task_param_name, self.task_finish_value)
        rospy.set_param(self.arm_is_right_pos_param, 1 if self.isRightPos else 0)

        # 预设位姿。注意：这些关节列表在 initParam() 中会被统一转换为 rad。
        self.ref_joint_left = None
        self.ref_joint_right = None
        self.place_joint_right = None
        self.place_joint_left = None
        self.init_joint_right = None
        self.init_joint_left = None
        self.mid_joint_trans = None


        # 数据记录变量
        self.recording_enabled = False
        self.joint_records = []
        self.record_lock = Lock()
        self.record_start_time = 0.0
        self.current_operation = ""
        self.record_dir = os.getcwd()

    def initParam(
        self,
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
    ):
        """初始化运动参数。"""
        self.isEyeInHand = isEyeInHand
        self.is_eye_in_hand = bool(isEyeInHand)
        self.object_compensation_xyz = self._parse_float_tuple(objectCompensationXYZ, 3)
        self.use_input_rpy = useInputRPY
        self.object_rotate_rpy = self._parse_float_tuple(objectRotateRPY, 3)
        self.object_pre_pose_offset = self._parse_float_tuple(objectPrePoseOffsetXYZRPY, 6)
        self.auto_place = autoPlace

        # 预设关节角进入程序后全部转换为 rad。
        # 默认 joint_param_unit=rad：兼容旧代码“原样下发”的配置。
        # 若 YAML 中写的是角度值，请设置 robot_vision_wrapper/joint_param_unit: degree。
        self.ref_joint_left = self._parse_joint_tuple(refJointLeft, 6)
        self.ref_joint_right = self._parse_joint_tuple(refJointRight, 6)
        self.place_joint_left = self._parse_joint_tuple(placeJointLeft, 6)
        self.place_joint_right = self._parse_joint_tuple(placeJointRight, 6)
        self.init_joint_left = self._parse_joint_tuple(initJointPosLeft, 6)
        self.init_joint_right = self._parse_joint_tuple(initJointPosRight, 6)
        self.mid_joint_trans = self._parse_joint_tuple(midJointTrans, 6)

        rospy.loginfo(
            "关节角单位已统一：内部和运动服务下发均为 rad；参数 joint_param_unit=%s",
            self.joint_param_unit,
        )

    def _parse_float_tuple(self, value, expected_len=None):
        """解析 ROS 参数中的逗号字符串或 list/tuple。"""
        if isinstance(value, str):
            items = [item.strip() for item in value.split(",") if item.strip() != ""]
        elif isinstance(value, (list, tuple)):
            items = list(value)
        else:
            raise ValueError("Unsupported parameter type: {}".format(type(value)))

        result = tuple(float(item) for item in items)
        if expected_len is not None and len(result) != expected_len:
            raise ValueError("Expected {} values, got {}: {}".format(expected_len, len(result), value))
        return result

    def _parse_bool_param(self, name, default=False):
        """读取 bool 型 ROS 参数，兼容 true/false、1/0、yes/no 字符串。"""
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)

        text = str(value).strip().lower()
        if text in ("true", "1", "yes", "y", "on", "是", "开", "开启"):
            return True
        if text in ("false", "0", "no", "n", "off", "否", "关", "关闭"):
            return False

        rospy.logwarn("参数 %s=%s 无法识别为 bool，按默认值 %s 处理", name, value, default)
        return bool(default)

    def _normalize_joint_unit_name(self, unit):
        """规范化关节角单位名称。"""
        text = str(unit).strip().lower()
        if text in ("deg", "degree", "degrees", "角度"):
            return "degree"
        if text in ("rad", "radian", "radians", "弧度"):
            return "rad"
        rospy.logwarn("未知 joint_param_unit=%s，按 rad 处理", unit)
        return "rad"

    def _parse_joint_tuple(self, value, expected_len=6):
        """
        解析预设关节角，并统一转换为 rad。

        说明：
        - IK 求解结果通常为 rad；
        - JAKA Move 服务下发关节角按 rad；
        - 预设关节参数可通过 robot_vision_wrapper/joint_param_unit 指定 rad/degree。
        """
        joints = list(self._parse_float_tuple(value, expected_len))
        if self.joint_param_unit == "degree":
            joints = self.degJoint2radJoint(joints)
        return tuple(float(j) for j in joints)
