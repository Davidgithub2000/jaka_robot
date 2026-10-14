# 第 08 课：MoveIt、RRTConnect 与 ServoJ 为什么不是当前主链

**状态：已展开**

**源码边界：** 仓库同时包含正式双臂采摘链和 MoveIt 示例/备用链。判断某算法是否“项目正在使用”，必须同时看到启动入口、运行节点和实际调用链，不能只凭配置文件或函数存在。

## 1. 场景与学习目标

仓库里能找到 OMPL、RRT/RRTConnect、KDL、FollowJointTrajectory 和 ServoJ。它们都是真实存在的能力，但当前 `dual_jaka_pick_bringup.launch` 启动的正式采摘链并没有经过这些组件。

学完本课，你应能用源码证据区分“已配置”“示例可用”“正式链已调用”，并能说清若要把正式采摘升级到 MoveIt，需要补齐哪些系统环节。

## 2. 两条链对照

### 2.1 当前正式采摘链

```text
dual_jaka_pick_bringup.launch
  -> dual_jaka_driver.launch
  -> dual_jaka_wrapper.launch
  -> dual_pick_coordinator.py
  -> Python 几何构姿
  -> JAKA SDK get_ik
  -> joint_move / joint_move_tol
  -> robot_states 完成监督
```

这个 launch 没有启动 `move_group`、规划场景或 `FollowJointTrajectory` action client。

### 2.2 MoveIt 示例链

```text
MoveIt launch / 独立示例脚本
  -> move_group
  -> KDL 运动学插件
  -> OMPL（RRT 或脚本显式 RRTConnect）
  -> FollowJointTrajectory
  -> jaka_planner/moveit_server.cpp
  -> SDK servo_j
```

这条链说明仓库具备实验桥接代码，不说明正式双臂采摘已经在线使用它。

## 3. MoveIt 配置分别证明什么

### 3.1 `ompl_planning.yaml`

`jaka_zu5` 的默认规划器配置为 `RRT`，同时列出了 `RRTConnect`、RRTstar、PRM 等多种 OMPL 规划器。它证明 MoveIt 配置包允许选择这些规划器。

独立脚本 `point_motion_adaptive.py` 显式调用 `set_planner_id("RRTConnect")`，并配置速度缩放、加速度缩放、规划时间和尝试次数。这只证明该脚本路径选择了 RRTConnect，不能反推正式 wrapper 使用了它。

### 3.2 `kinematics.yaml`

MoveIt 组 `jaka_zu5` 配置的是：

```text
kdl_kinematics_plugin/KDLKinematicsPlugin
```

搜索分辨率和超时也在该文件中配置。它与正式 wrapper 调用 JAKA SDK `kine_inverse()` 是两套不同 IK 路径。

### 3.3 `ros_controllers.yaml`

MoveIt 控制器列表声明 `jaka_zu5_controller/follow_joint_trajectory`，类型为 `FollowJointTrajectory`，并列出六个关节。该文件是控制器映射配置；只有相关 controller/action server 实际启动并连接后，轨迹才会执行。

## 4. `moveit_server.cpp` 怎样桥接 ServoJ

`goalCb()` 接收 `FollowJointTrajectoryGoal` 后：

1. 开启 SDK servo mode；
2. 从第 2 个轨迹点开始遍历；
3. 读取相邻点 `time_from_start` 差值 `dt`；
4. 计算 `step_num = int(dt / 0.008)`；
5. 将该点 6 个关节角交给 `robot.servo_j(..., ABS, step_num)`；
6. 轨迹点发完后，循环检查最终关节角是否都在约 `±0.2 degree` 范围；
7. 到位或收到 preempt 后退出并关闭 servo mode。

这是一座“MoveIt 离散轨迹点 → JAKA ServoJ”的桥。它不属于 `robotPickJoint()` 的调用链；正式抓取调用的是 joint move 服务。

## 5. 这条示例桥的工程风险

源码还暴露出若干需要在正式接入前处理的问题：

- `dt < 0.008 s` 时 `step_num` 可能为 0，当前未见显式校验；
- `servo_j` 返回非零只记录日志，没有立即中止并把 action 置为 aborted；
- 最终到位循环没有显式总超时，反馈异常时可能长期等待；
- preempt 时虽调用 `motion_abort()`，函数末尾仍统一 `setSucceeded()`，action 语义不正确；
- 没有在本文件中看到完整的轨迹容差、控制周期抖动和通信丢包处理。

所以即使把节点启动起来，也仍需修复 action 结果语义并完成实机节拍与安全验证，才能作为正式执行器。

## 6. 为什么“仓库里有”不等于“主链在用”

判断标准应分三级：

1. **能力存在：** 有源码、配置或接口；
2. **路径可启动：** 有 launch/脚本能把组件运行起来；
3. **正式任务调用：** 正式入口启动该组件，业务调用链确实经过它，运行话题/action 也能观测到。

MoveIt/RRTConnect/ServoJ 在本仓库满足前两级的部分证据，但当前正式 `dual_jaka_pick_bringup.launch` 不满足第三级。因此项目陈述应写成“仓库包含 MoveIt/ServoJ 示例与升级基础”，不能写成“当前双臂采摘使用 RRTConnect 在线避障和 ServoJ 伺服控制”。

## 7. 若升级到 MoveIt，需要补齐什么

不是把规划器名称改成 RRTConnect 就完成升级。至少需要：

1. 校验左右臂 URDF/SRDF、关节限位和零位；
2. 建立双臂及环境规划场景、夹爪和果筐碰撞体；
3. 解决左右命名空间、joint_states 和 controller/action 映射；
4. 明确 KDL IK 与 SDK IK 的使用边界；
5. 做轨迹时间参数化与速度/加速度限制；
6. 修复 ServoJ action server 的失败、抢占和超时语义；
7. 建立急停、碰撞、掉线与恢复策略；
8. 先仿真、再单臂空载、再双臂低速、最后带真实目标验收。

## 8. 运行时怎样证明到底走哪条链

仅查看文件不够时，可收集以下运行证据：

- `rosnode list` 中是否存在 `move_group` 与 MoveIt 执行节点；
- `rostopic info` / action 工具是否显示 `follow_joint_trajectory` 客户端与服务端连接；
- 正式 wrapper 的日志是否仍在调用 `get_ik`、`joint_move_tol`；
- 驱动侧是否实际收到 ServoJ 请求；
- 规划场景中是否存在机器人与环境碰撞体。

如果只有 MoveIt 配置文件而没有节点、连接和请求，就只能认定为离线能力配置。

## 9. 本课小结

当前正式采摘是“几何构姿 + 厂商 SDK IK + 关节关键点运动 + 状态监督”；MoveIt 示例链则是“KDL/OMPL 规划 + FollowJointTrajectory + ServoJ 桥接”。两者共存于仓库，但没有自动合并。准确描述边界，比把所有出现过的技术名词都写成已用能力更重要。

**你的复述任务：** 分别画出正式链和 MoveIt 示例链，至少列出四个不同节点/接口；然后用“能力存在—路径可启动—正式任务调用”三级证据判断 RRTConnect 和 ServoJ 的当前状态。
