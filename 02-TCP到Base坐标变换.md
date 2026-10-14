# 第 02 课：视觉点怎样从 TCP 坐标变到 Base 坐标

**状态：已展开**

**源码边界：** 本课按当前正式脚本 `src/scene/realman_scene/script/` 展开。结论来自静态源码；坐标精度、消息延迟和手眼标定误差尚未由本课做实机验证。

## 1. 场景与学习目标

视觉端向每条机械臂发布 6 个数：

```text
[p1_x, p1_y, p1_z, p2_x, p2_y, p2_z]
```

它们是同一个目标上的两个三维点，坐标系是“机械臂当前 TCP 坐标系”，默认单位为 m。逆运动学服务需要的目标却在机械臂 Base 坐标系下，因此 wrapper 必须同时取得当前 TCP 在 Base 下的位置和姿态，完成一次刚体变换。

学完本课，你应能回答三个问题：

1. 为什么只有视觉点、没有 `tool_position` 时不能计算 Base 点？
2. m/mm、degree/rad 分别在哪一层转换？
3. 为什么“成功入队”仍不等于视觉与机械臂已经严格时间同步？

## 2. 主调用链

```text
/<arm>/custom_arm_data: Float32MultiArray，两个 TCP 点（m）
  -> robot_vision_wrapper_main.py::_init_ros_communication()
  -> robot_vision_auto.py::goalCB()
  -> _extract_vision_points_from_msg()
  -> _build_target_job_from_tcp_points()
  -> _wait_for_fresh_tool_position()
  -> _tcp_point_to_base_with_snapshot()
  -> p_base = t_base_tcp + R_base_tcp @ p_tcp
  -> Base(m) × 1000 = Base(mm)
  -> 工作空间检查 -> 去重 -> 队列
```

另一条输入链负责持续刷新 TCP 位姿：

```text
/<arm>/jaka_driver/tool_position: TwistStamped（默认 mm + degree）
  -> robot_motion_control.py::updateToolPosition()
  -> [x_m, y_m, z_m, rx_rad, ry_rad, rz_rad] + 接收时间戳
```

## 3. 建议源码阅读顺序

1. `robot_vision_wrapper_main.py::_init_ros_communication()`：先确认目标话题绑定 `goalCB()`，工具位姿话题绑定 `updateToolPosition()`。
2. `robot_motion_control.py::updateToolPosition()`：看反馈单位如何归一化、时间戳如何缓存。
3. `robot_vision_auto.py::goalCB()`：看消息入口、错误处理、去重和入队顺序。
4. 同文件 `_build_target_job_from_tcp_points()`、`_wait_for_fresh_tool_position()`：看变换前置条件。
5. 同文件 `_tool_rotation_matrix_from_pose()`、`_tcp_point_to_base_with_snapshot()`：看公式落地。
6. 最后读 `_points_reachable_in_custom_workspace()`、`_is_duplicate_pick_target()`、`_push_pick_target()`：理解为什么变换成功后仍可能不执行。

## 4. 沿数据流逐步理解

### 4.1 目标消息先做格式与空值检查

`goalCB()` 先把消息解析为 `p1_tcp` 和 `p2_tcp`。当前主格式是 `Float32MultiArray`，源码还保留了对 `Pose` 类输入的兼容分支。全零占位目标会被忽略，非有限数或退化输入会在后续构造任务时被拒绝。

这里的点不是 Base 点，也不是相机光学坐标点；正式 wrapper 把它们解释为当前 TCP 坐标系下的点，单位 m。

### 4.2 工具反馈先统一单位

驱动的 `tool_position` 默认给出：

- 平移：mm；
- 欧拉角：degree。

`updateToolPosition()` 把它统一成内部格式：

```text
[x_m, y_m, z_m, rx_rad, ry_rad, rz_rad]
```

这样，矩阵乘法中的平移和视觉点都使用 m，旋转构造使用 rad。直到得到 Base 点以后，任务才额外保存一份 mm 数据，供抓取位姿和 JAKA SDK 接口使用。

### 4.3 新鲜度是变换的硬前提

`_wait_for_fresh_tool_position()` 会等待一份已收到且未过期的 TCP 位姿。当前配置默认最大年龄约为 `1.0 s`，等待超时约为 `0.5 s`，轮询间隔约为 `0.01 s`。没有工具位姿或位姿过期时，目标不会被“猜着变换”，而是返回失败结果。

取得快照后，同一目标的 p1、p2 共用这份快照，避免两点分别读取到不同机械臂姿态。

### 4.4 刚体变换公式

设：

- `p_tcp`：TCP 坐标系中的目标点；
- `R_base_tcp`：把 TCP 向量旋转到 Base 的旋转矩阵；
- `t_base_tcp`：TCP 原点在 Base 中的位置。

则：

```text
p_base = t_base_tcp + R_base_tcp @ p_tcp
```

`_tool_rotation_matrix_from_pose()` 根据配置把姿态解释为欧拉角或旋转向量。默认是 Euler `xyz`、小写轴序，对应 SciPy 的外旋/固定轴约定；若改参数，必须同步确认视觉和驱动所采用的姿态约定。

### 4.5 变换后还有三道门

1. **工作空间门：** p1、p2 都必须落在该 wrapper 配置的 Base 工作空间内。当前左右 launch 都配置了负 Y 区域；这只是当前参数事实，是否完全符合真实安装位置需要实机复核。
2. **重复目标门：** 以两点中点、机械臂侧别和时间窗判断重复。当前默认半径约 `30 mm`、TTL 约 `60 s`。
3. **队列容量门：** 队列上限默认 10；满时会丢掉最旧项、保留最新项。

`vision_accepting=False` 并不会让 `goalCB()` 丢弃新目标，目标仍可进入队列。这是为了避免机械臂动作期间漏掉一次性视觉发布。

## 5. 一个数值例子

为便于手算，假设当前 TCP 相对 Base 没有旋转：

```text
R_base_tcp = I
t_base_tcp = [0.3, -0.5, 0.4] m
p_tcp      = [0.1,  0.0, 0.0] m
```

那么：

```text
p_base = [0.3, -0.5, 0.4] + I @ [0.1, 0, 0]
       = [0.4, -0.5, 0.4] m
       = [400, -500, 400] mm
```

若 TCP 有旋转，就应先由 `R_base_tcp @ p_tcp` 改变点相对 TCP 原点的方向，再加平移，不能只把两个坐标直接相加。

## 6. 关键边界与风险

- `Float32MultiArray` 本身没有采集时间戳。代码使用的是 `goalCB()` 执行时能取得的最新 TCP 快照，不是与相机曝光硬同步的位姿。
- 工具反馈虽保存了 `frame_id`，当前变换直接使用数值位姿，没有调用 TF 查找变换。
- “位姿未超过 1 s”只说明未过期，不代表动态抓拍误差足够小。机械臂运动中采集目标时，应额外评估视觉时间戳、通信延迟和速度造成的空间误差。
- 工作空间检查是长方体边界，不等价于完整可达域、关节限位检查或碰撞检查。

## 7. 本课小结

本课真正实现的是“使用一份新鲜 TCP 快照，把两个 TCP 系点变到 Base 系，并进行上层过滤”。单位契约是：视觉点 m、工具反馈默认 mm+degree、内部变换 m+rad、抓取与 SDK 目标 mm+rad。它保证了计算链条自洽，但不自动保证视觉与机械臂的严格时间同步，也不等于目标一定可逆解、无碰撞。

**你的复述任务：** 不看正文，用自己的话从 `/left_arm/custom_arm_data` 开始，依次说出目标解析、TCP 快照、新鲜度、刚体变换、mm 转换、工作空间、去重和入队八个环节，并解释为什么消息没有时间戳会留下动态误差。
