# `ws_double_jaka` 机械臂运控源码课程

## 项目用途与本次学习目标

这个工作区面向双 JAKA 机械臂采摘：两套机械臂驱动在 ROS1 命名空间下独立运行，上层 wrapper 接收视觉目标，完成坐标变换、抓取姿态构造、IK、关键关节点运动、夹爪动作、回位与结果上报。

本课程不先背“机械臂算法名词”，而是沿当前正式启动链回答八个项目问题：

1. 一条视觉目标如何走到两台真实机械臂中的某一台？
2. TCP 坐标系中的两个点如何变到机械臂基坐标系？
3. 两个点如何确定六自由度抓取位姿？
4. 位姿如何变成可执行的六关节角，失败时怎样换姿态再求解？
5. 预抓取、抓取、退回、放置和回位是怎样编排的？
6. 程序怎样判断“指令被接受”和“机械臂真的停下”是两回事？
7. 双臂怎样隔离、怎样在任务层汇合，以及当前安全边界在哪里？
8. 仓库中的 MoveIt、RRTConnect 和 ServoJ 与正式采摘主链是什么关系？

## 源码版本与结论边界

- 本地目录没有可用的 Git 仓库元数据，无法给出 commit，也无法区分哪些内容是未提交修改。本课程以 2026-09-16 读取到的当前文件内容为准。
- 关键快照：
  - `robot_vision_auto.py`：SHA-256 前缀 `D7F75E5D993E`，文件时间 2026-07-15 17:33:17。
  - `robot_motion_control.py`：SHA-256 前缀 `1A7E477B8618`，文件时间 2026-07-07 15:18:16。
- 结论来自静态源码与 launch 配置，没有连接实机验证 IP、SDK 行为、IK 耗时、运动轨迹、夹爪反馈和碰撞状态。
- 当前正式主链使用 JAKA SDK 的 `kine_inverse`、`joint_move`；SDK 内部的 IK 方法、插补、速度规划和底层伺服闭环不可见，不能说成项目自研。

## 前置基础

- ROS1：launch、namespace、topic、service、callback。
- 刚体变换：旋转矩阵、欧拉角、`P_base = t_base_tcp + R_base_tcp P_tcp`。
- 机械臂基础：正/逆运动学、关节空间与笛卡尔空间、参考关节解。
- Python/Numpy：向量归一化、投影、叉乘；C++ 只需能读 ROS 服务回调。
- 单位意识：m/mm、degree/rad 是本项目的高风险交界面。

## 本次覆盖与暂不覆盖

覆盖当前 `dual_jaka_pick_bringup.launch` 能启动的双臂采摘运控主链、驱动服务、状态反馈、双臂任务协调和假目标验证入口。

暂不展开视觉检测网络、相机内参/手眼标定求解、底盘导航、Modbus 夹爪协议细节、旧代码和备份目录。MoveIt 只作为“仓库存在但正式主链未接入”的对照课程，不把示例能力当成当前生产能力。

## 先建立一张总图

代表性端到端流程：

```text
/left_arm/custom_arm_data 或 /right_arm/custom_arm_data
  -> RobotVisionAutoMixin.goalCB()
  -> 获取同一时刻附近的 tool_position
  -> TCP(m) -> Base(m) -> Base(mm)
  -> 工作空间与重复目标检查 -> 任务入队
  -> _auto_pick_loop() -> _execute_pick_target_job()
  -> robotPickJoint()
       -> calculate_grasp_pose()
       -> 工具长度/预抓取偏移
       -> solve_ik_for_pose() -> /<arm>/jaka_driver/get_ik
       -> robotJointMoveTolSequenceL()
       -> /<arm>/jaka_driver/joint_move_tol
  -> JAKA SDK kine_inverse()/joint_move()
  -> /<arm>/jaka_driver/robot_states
  -> Running -> Stop（或短运动宽限后连续 Stop）
  -> 夹爪、退回、放置、回初始位
  -> /left_picker/target_result 或 /right_picker/target_result
  -> dual_pick_coordinator -> /dual_picker/station_done
```

一句话定位：当前代码是“目标几何 + SDK IK + 关节关键点编排 + 状态监督”，不是自研连续轨迹优化器，也不是正式主链中的 MoveIt 在线避碰规划。

---

## 第 01 课：正式启动链怎样把两台机械臂隔离开

**状态：已展开（独立文件）**

**完整课程：** [01-双臂启动链与命名空间隔离.md](01-双臂启动链与命名空间隔离.md)

### 1. 场景与目标

触发场景是执行：

```bash
roslaunch realman_scene dual_jaka_pick_bringup.launch
```

读完本课，你应能不看文档解释三件事：

1. 为什么同名的两个 `jaka_driver` 节点能够连接不同 IP，并生成两套不同服务；
2. 为什么同一份 `robot_vision_wrapper_main.py` 能分别订阅左右目标，并只调用对应机械臂；
3. 为什么这里应称为“两套单臂控制实例”，而不是“一个双臂同步控制器”。

只需先理解三个 ROS1 概念：

- **节点（node）**：运行中的进程实例；同一份可执行文件可以启动多次。
- **命名空间（namespace）**：给节点、话题、服务和参数增加路径前缀。
- **参数注入（parameter injection）**：launch 在启动每个实例时给它不同配置，同一份代码因此表现不同。

### 2. 先给结论

左右臂不会串线，不是因为项目写了两套算法，而是因为同一套代码被实例化两次，并被绑定到了两套不同的全限定 ROS 名称：

| 层次 | 左侧实例 | 右侧实例 |
| --- | --- | --- |
| 驱动节点 | `/left_arm/jaka_driver` | `/right_arm/jaka_driver` |
| 机械臂 IP | `192.168.1.181` | `192.168.1.188` |
| wrapper 节点 | `/left_picker/robot_vision_wrapper` | `/right_picker/robot_vision_wrapper` |
| wrapper 绑定的 driver | `/left_arm/jaka_driver` | `/right_arm/jaka_driver` |
| 视觉目标 | `/left_arm/custom_arm_data` | `/right_arm/custom_arm_data` |
| 单目标结果 | `/left_picker/target_result` | `/right_picker/target_result` |
| 夹爪 ID | `1` | `2` |

真正起隔离作用的是 `/left_arm`、`/right_arm`、`/left_picker`、`/right_picker` 这些前缀，以及 wrapper 中显式传入的 `driver_ns`。`left`、`right` 字样本身没有魔法，只是 ROS 名字的一部分。

### 3. 完整启动主链

```text
dual_jaka_pick_bringup.launch
  ├─ include dual_jaka_driver.launch
  │    ├─ /left_arm/jaka_driver  -> 登录左臂 IP
  │    └─ /right_arm/jaka_driver -> 登录右臂 IP
  │
  ├─ include dual_jaka_wrapper.launch
  │    ├─ /left_picker/robot_vision_wrapper
  │    │    └─ driver_ns=/left_arm/jaka_driver
  │    └─ /right_picker/robot_vision_wrapper
  │         └─ driver_ns=/right_arm/jaka_driver
  │
  └─ /dual_pick_coordinator
       └─ 汇总左右目标和结果，不生成机械臂轨迹
```

启动之后，每个 wrapper 内部又执行：

```text
main()
  -> rospy.init_node("robot_vision_wrapper")
  -> RobotVisionWrapper()
       -> _init_parameters()
       -> _init_network()
       -> _init_ros_communication()
       -> _init_data_recording()
  -> initParam(...)
  -> rospy.sleep(2)
  -> start_auto_pick()
       -> 后台线程执行 _auto_pick_loop()
  -> rospy.spin()
       -> 主线程持续处理 ROS 回调
```

### 4. 源码阅读顺序

1. `src/scene/realman_scene/launch/dual_jaka_pick_bringup.launch:1`
   - 看顶层参数、两个 `include` 和协调器节点。
2. `src/arm/jaka_robot_v2.2/src/jaka_driver/launch/dual_jaka_driver.launch:1`
   - 看同一个 driver 如何在两个 namespace 中启动。
3. `src/arm/jaka_robot_v2.2/src/jaka_driver/src/jaka_driver.cpp::main()`
   - 看 `ip` 参数怎样进入 SDK，以及相对服务名怎样被发布。
4. `src/scene/realman_scene/launch/dual_jaka_wrapper.launch:1`
   - 对照左右 wrapper 的相同项和差异项。
5. `src/scene/realman_scene/script/robot_vision_wrapper_main.py::main()`、`RobotVisionWrapper.__init__()`、`_init_ros_communication()`
   - 看 Python 节点怎样加载参数、绑定回调和创建服务代理。
6. `src/scene/realman_scene/script/robot_vision_config.py::_init_parameters()`
   - 看 `driver_ns` 如何派生出完整 topic/service 名称。
7. `src/scene/realman_scene/script/robot_vision_auto.py::start_auto_pick()`
   - 看异步采摘线程在哪里真正启动。

选读：`DUAL_JAKA_OPERATION.md` 中的节点、话题、服务检查命令。

暂缓：`dual_pick_coordinator.py` 内部完成条件放到第 7 课；旧代码、`*_bak*` 和其他备份脚本不进入本课主链。

### 5. 沿启动流程带读

#### 5.1 顶层 launch 只做三件事

`dual_jaka_pick_bringup.launch` 首先定义左右 IP 和左右目标话题：

```xml
<arg name="left_ip" default="192.168.1.181"/>
<arg name="right_ip" default="192.168.1.188"/>
<arg name="left_target_topic" default="/left_arm/custom_arm_data"/>
<arg name="right_target_topic" default="/right_arm/custom_arm_data"/>
```

然后它分别包含 driver launch 和 wrapper launch。注意参数的流向不同：

- `left_ip/right_ip` 只传给 `dual_jaka_driver.launch`；
- `left_target_topic/right_target_topic` 只传给 `dual_jaka_wrapper.launch`；
- 协调器同时订阅左右目标和左右结果，负责工位级汇总。

所以顶层 launch 是装配入口，不包含 IK、轨迹或控制算法。

#### 5.2 同一份 driver 被启动两次

`dual_jaka_driver.launch` 的关键结构是：

```xml
<group ns="left_arm">
  <param name="ip" value="$(arg left_ip)"/>
  <node pkg="jaka_driver" type="jaka_driver" name="jaka_driver"/>
</group>
```

右臂完全同构，只把 namespace 换成 `right_arm`、参数换成 `right_ip`。

于是两个节点的完整名字分别是：

```text
/left_arm/jaka_driver
/right_arm/jaka_driver
```

参数 `ip` 也分别解析为：

```text
/left_arm/ip  = 192.168.1.181
/right_arm/ip = 192.168.1.188
```

C++ driver 中使用默认 `ros::NodeHandle nh`，再读取相对参数名：

```cpp
string robot_ip = nh.param("ip", default_ip);
robot.login_in(robot_ip.c_str());
```

因此左实例读取 `/left_arm/ip`，右实例读取 `/right_arm/ip`。同一份二进制最终连接两个不同地址。

#### 5.3 相对服务名在 namespace 下自然分裂

driver 发布服务时没有写前导 `/`：

```cpp
nh.advertiseService("jaka_driver/joint_move", joint_move_callback);
nh.advertiseService("jaka_driver/get_ik", get_ik_callback);
```

这类名字属于相对名，会继承当前节点所在 namespace。因此它们分别解析为：

```text
/left_arm/jaka_driver/joint_move
/left_arm/jaka_driver/get_ik

/right_arm/jaka_driver/joint_move
/right_arm/jaka_driver/get_ik
```

`tool_position`、`joint_state`、`robot_states` 也是同样的规则。这里没有“收到请求后再判断左右臂”的 `if`；在请求到达回调以前，ROS 名称已经把它路由到了对应 driver 进程。

#### 5.4 driver 启动具有真实硬件动作

`jaka_driver.cpp::main()` 不只是注册 ROS 接口。它还会：

1. 使用配置 IP 调用 `robot.login_in()`；
2. 检查机械臂是否上电，未上电则调用 `power_on()`；
3. 检查是否使能，未使能则调用 `enable_robot()`；
4. 调用 `set_rapidrate(0.3)`；
5. 以 `20 Hz` 发布 TCP、关节和机器人状态。

这意味着不能把上述 roslaunch 命令当成无害的“查看节点”命令。课程只做静态阅读；实机启动前必须确认急停、工作区、两台 IP、机械臂周围人员和初始姿态。

#### 5.5 同一份 wrapper 也被启动两次

`dual_jaka_wrapper.launch` 使用两个 picker namespace：

```text
/left_picker
/right_picker
```

两边的节点都叫 `robot_vision_wrapper`，但完整节点名不同，所以不会重名：

```text
/left_picker/robot_vision_wrapper
/right_picker/robot_vision_wrapper
```

最关键的不是节点名字，而是各自注入的 `driver_ns`：

```text
左 wrapper: /left_arm/jaka_driver
右 wrapper: /right_arm/jaka_driver
```

除此之外，左右实例还分别配置目标话题、结果话题、任务参数、初始位、参考关节、放置位和夹爪 ID。这些都是实例数据；Python 算法代码仍是同一份。

#### 5.6 `driver_ns` 派生出一整套接口

`RobotVisionConfigMixin._init_parameters()` 先读取 `driver_ns`，然后用 `_join_ros_name()` 拼接资源名：

```python
self.robot_states_topic = self._join_ros_name(self.driver_ns, "robot_states")
self.joint_move_service = self._join_ros_name(self.driver_ns, "joint_move")
self.ik_service = self._join_ros_name(self.driver_ns, "get_ik")
```

以左 wrapper 为例，结果是：

```text
driver_ns                          /left_arm/jaka_driver
robot_states_topic                /left_arm/jaka_driver/robot_states
joint_state_topic                 /left_arm/jaka_driver/joint_state
tool_position_topic               /left_arm/jaka_driver/tool_position
joint_move_service                /left_arm/jaka_driver/joint_move
joint_move_tol_service            /left_arm/jaka_driver/joint_move_tol
ik_service                        /left_arm/jaka_driver/get_ik
fk_service                        /left_arm/jaka_driver/get_fk
```

右 wrapper 只需更换 `driver_ns`，整组接口便全部切到 `/right_arm/jaka_driver/...`。这就是项目中最核心的依赖注入点。

#### 5.7 wrapper 把名字绑定到回调和服务代理

`RobotVisionWrapper` 通过多继承组合三个职责：

```python
class RobotVisionWrapper(
    RobotVisionConfigMixin,
    RobotMotionMixin,
    RobotVisionAutoMixin,
):
    ...
```

- `RobotVisionConfigMixin`：参数、接口名、单位和共享状态；
- `RobotMotionMixin`：IK、运动服务、状态等待和夹爪封装；
- `RobotVisionAutoMixin`：目标处理、抓取姿态和任务状态机；
- `RobotVisionWrapper`：把三者装配成 ROS 节点。

初始化顺序是参数 → 网络对象 → ROS 通信 → 数据记录。`_init_ros_communication()` 随后完成两类绑定：

```text
订阅：
robot_states  -> updateRobotStatus()
joint_state   -> updateJointStatus()
tool_position -> updateToolPosition()
custom_arm_data -> goalCB()

服务代理：
joint_move      -> move_joint_client
joint_move_tol  -> move_joint_tol_client
get_ik          -> ik_client
get_fk          -> fk_client
```

因此，左目标进入左 wrapper 后，后续 `ik_client` 和 `move_joint_client` 已经绑定左 driver；右侧同理。正常调用链中不需要每次再传“arm_id”。

#### 5.8 回调线程与采摘线程是两个异步边界

`main()` 创建 wrapper 后先固定等待 2 秒，然后调用 `start_auto_pick()`：

```python
self.auto_pick_thread = Thread(target=self._auto_pick_loop)
self.auto_pick_thread.daemon = True
self.auto_pick_thread.start()
```

随后主线程进入 `rospy.spin()`，负责 ROS 回调；后台线程运行 `_auto_pick_loop()`，负责从目标队列取任务并执行。

这里要区分：

- `Subscriber(...)` 和 `ServiceProxy(...)` 创建成功，只表示本地绑定对象已经建立；
- 它们不证明对端 driver、网络连接或机械臂已经可用；
- 真正的服务可用性在后续调用时通过 `wait_for_service()` 检查；
- 自动线程启动后还要检查关节反馈和初始位，才会允许采摘。

最后两点分别在第 4～6 课和第 7 课展开。

### 6. 哪些资源隔离，哪些资源共享

每臂隔离的资源：

- driver 进程和机械臂 IP；
- `joint_move/get_ik` 等服务；
- `tool_position/joint_state/robot_states` 等反馈；
- 视觉目标话题和采摘结果话题；
- 参考关节、初始位、放置位和本地目标队列。

跨双臂共享的资源：

- `/multi_gripper/cmd` 与 `/multi_gripper/interpreted_state`，靠夹爪 ID `1/2` 区分；
- `/dual_picker/*` 工位级信号；
- 同一份 Python/C++ 源码和算法逻辑。

所以“使用 namespace”不等于所有资源都自动隔离。必须逐个检查绝对名字、相对名字和共享总线上的设备 ID。

### 7. 重要分支与边界

#### 分支 A：关闭协调器

若 `start_pick_coordinator=false`，两个 driver 和两个 wrapper 仍会启动并可各自处理目标，但不会由本 launch 启动 `/dual_pick_coordinator`，因此没有本项目的工位完成汇总链。

#### 分支 B：覆盖 IP 或目标话题

launch 参数可以覆盖默认 IP 和目标话题。IP 只改变 driver 连接目标；目标话题只改变 wrapper/协调器的数据路由。这两类参数不能互相替代。

#### 分支 C：同名 driver 目录

`src/jaka_driver_2.2.7/` 下也有同名 `jaka_driver`，但该目录根部存在 `CATKIN_IGNORE`。当前课程追踪的是：

```text
src/arm/jaka_robot_v2.2/src/jaka_driver/
```

#### 边界 A：不是底层同步双臂控制

左右 driver 没有共同的轨迹时钟，也没有一个函数同时向两臂下发成对关节指令。协调器只汇总目标与结果。因此当前架构是任务级双臂协调，不是严格同步运动控制。

#### 边界 B：配置存在不等于功能正在使用

wrapper 会创建 `vision_socket` 和 `gripper_socket`，launch 也传入相应 IP/端口；但当前 `_init_network()` 只创建 socket，没有主动 `connect()`。不能据此声称当前主链通过这两个 socket 完成视觉或夹爪通信。

#### 边界 C：静态源码不能证明实机可达

本课确认了名称解析和代码连接关系，但没有证明：

- `192.168.1.181/188` 当前可达；
- JAKA SDK 登录成功；
- 两个 ROS 服务端都启动成功；
- 左右目标实际由正确视觉节点发布；
- 机械臂启动上电、使能和速率设置在现场是安全的。

### 8. 用一次默认启动把全流程串起来

假设使用默认参数启动，但不考虑真实硬件是否在线：

1. 顶层 launch 把 `192.168.1.181` 传给左 driver，把 `192.168.1.188` 传给右 driver。
2. `dual_jaka_driver.launch` 在 `/left_arm` 和 `/right_arm` 中分别启动同一个 `jaka_driver` 可执行文件。
3. 两个 C++ 进程各自读取本 namespace 下的 `ip`，尝试登录对应机械臂，并发布带 namespace 的服务与反馈话题。
4. `dual_jaka_wrapper.launch` 在 `/left_picker`、`/right_picker` 中分别启动同一份 Python wrapper。
5. 左 wrapper 读取 `driver_ns=/left_arm/jaka_driver`，派生所有左臂接口；右 wrapper 同理派生右臂接口。
6. 两个 wrapper 分别注册目标回调、状态回调和 IK/运动服务代理。
7. 每个 wrapper 等待 2 秒后启动自己的后台采摘线程，主线程进入 ROS 回调循环。
8. 协调器订阅左右目标与结果，等待后续任务事件；它不会介入单条 IK 或关节运动。

如果左 driver 启动失败，左 wrapper 的本地 `ServiceProxy` 仍可能已经创建，但真正调用左侧服务时会等待或失败；右链是否仍可工作取决于右 driver 和共享外围设备，源码没有全局“一臂失败则全部退出”的启动事务。

### 9. 本课回顾

请把整课压缩成一句话：

> 顶层 launch 将同一套 driver 和 wrapper 各启动两次，通过 ROS namespace 与 `driver_ns` 参数把左右 topic/service 绑定到不同机械臂；两臂在单臂运动层独立，只在协调器处做任务结果汇总。

**你的复述任务：** 不看上面的表，自己说出“一个左臂目标为什么最终只会调用 `/left_arm/jaka_driver/get_ik`，而不会调用右臂 IK”。复述时必须依次包含：目标话题、左 wrapper、`driver_ns`、`_join_ros_name()`、`ServiceProxy` 五个环节。

---

## 第 2 课：视觉点怎样从 TCP 坐标变到 Base 坐标

**状态：已展开（独立文件）**

**完整课程：** [02-TCP到Base坐标变换.md](02-TCP到Base坐标变换.md)

**读完能回答：** `/custom_arm_data` 的六个数是什么，为什么必须同时拿到新鲜的 `tool_position`，m/mm 和 degree/rad 在哪里转换？

**主链：**

```text
Float32MultiArray[p1_tcp, p2_tcp]（m）
  -> goalCB()
  -> _extract_vision_points_from_msg()
  -> _wait_for_fresh_tool_position()
  -> updateToolPosition() 已把 mm+degree 归一为 m+rad
  -> _tcp_point_to_base_with_snapshot()
  -> P_base = t_base_tcp + R_base_tcp @ P_tcp
  -> Base(m) 转 Base(mm)
  -> 工作空间/去重检查 -> 入队
```

**必读路径与符号：**

1. `src/scene/realman_scene/script/robot_vision_wrapper_main.py::_init_ros_communication()`
   - 输入：`/<arm>/custom_arm_data` 与 `/<arm>/jaka_driver/tool_position`。
   - 下一步：分别绑定 `goalCB()` 和 `updateToolPosition()`。
   - 输出：目标与当前 TCP 位姿两路数据进入 wrapper。
2. `src/scene/realman_scene/script/robot_motion_control.py::updateToolPosition()`
   - 输入：驱动 `TwistStamped`，默认位置 mm、姿态 degree。
   - 下一步：归一为 `[x_m,y_m,z_m,rx_rad,ry_rad,rz_rad]` 并带时间戳缓存。
   - 输出：坐标变换使用的 TCP 快照。
3. `src/scene/realman_scene/script/robot_vision_auto.py::goalCB()`、`_build_target_job_from_tcp_points()`
   - 输入：两个 TCP 点，单位 m。
   - 下一步：空目标、有限数、新鲜度、工作空间和重复目标检查。
   - 输出：保存 TCP(m)、Base(m)、Base(mm) 的 `target_job`。
4. `src/scene/realman_scene/script/robot_vision_auto.py::_tool_rotation_matrix_from_pose()`、`_tcp_point_to_base_with_snapshot()`
   - 输入：TCP 位姿与 TCP 坐标点。
   - 下一步：按欧拉角或旋转向量生成 `R_base_tcp`，再做刚体变换。
   - 输出：基坐标系点，单位 m。

**重要分支：** `tool_position` 未收到或超过 `tool_position_max_age_s` 时拒绝变换；旋转表示可配置为 Euler/rotvec，欧拉角轴序和内外旋也可配置。

**结果：** 后续姿态计算和 IK 不再依赖运动过程中变化的 TCP，而使用目标到达回调时冻结的基坐标点。

**选读验证：** `src/pei_test/publish_right_fake_target_once` 中 Base 假目标反算 TCP 目标的过程。

**暂缓阅读：** 手眼标定矩阵的求解来源不在本课；本课只读运行时坐标变换契约。

---

## 第 3 课：两个视觉点怎样构造六自由度抓取姿态

**状态：已展开（独立文件）**

**完整课程：** [03-双点构造六自由度抓取姿态.md](03-双点构造六自由度抓取姿态.md)

**读完能回答：** 为什么两个点除了给出位置，还能确定工具朝向？工具坐标系的 X/Y/Z 三轴是怎样构造出来的？

**主链：**

```text
p1_base、p2_base（mm）
  -> p_mid = (p1+p2)/2
  -> tool +Y = normalize(p1-p2)
  -> 候选 +Z = 目标中点方向在 +Y 法平面上的投影
  -> tool +X = +Y × +Z
  -> 重新正交化 +Z
  -> R_target=[X Y Z]
  -> XYZ Euler(rad)
```

**必读路径与符号：**

1. `src/scene/realman_scene/script/robot_vision_auto.py::calculate_grasp_pose()`
   - 输入：Base 坐标系下两个点，单位 mm。
   - 下一步：验证点、计算中点和果梗方向。
   - 输出：`p_mid`、`euler_target`、`z_axis`。
2. 同函数中的投影与叉乘段
   - 输入：`y_axis` 与基座原点到目标中点的方向。
   - 下一步：投影、归一化、叉乘、行列式检查。
   - 输出：右手正交旋转矩阵。
3. `src/scene/realman_scene/script/robot_vision_auto.py::robotPickJoint()` 的工具补偿段
   - 输入：工具尖端目标 `p_mid` 和工具 `+Z`。
   - 下一步：`tcp_target = tip_target - 165 mm * z_axis`；预抓取点再沿 `-Z` 偏移 `pre_grasp_offset_mm`。
   - 输出：给 IK 的抓取位姿与预抓取位姿。

**重要分支：** 当目标中点方向几乎与果梗方向平行时，从 Base Z/X/Y 中选最不平行方向作为备用，再投影构造 `+Z`。

**结果：** 项目自己实现的是“由双点几何构造目标位姿”；它不是 IK，也不是路径规划。

**选读验证：** 检查 `R_target.T @ R_target ≈ I`、`det(R_target)≈1`、`R_target[:,1]·normalize(p1-p2)≈1`。

**暂缓阅读：** 欧拉角奇异性、四元数连续性和姿态滤波属于后续改进；当前实现直接输出 XYZ Euler。

---

## 第 4 课：目标位姿怎样通过 SDK IK 变成六关节角

**状态：已展开（独立文件）**

**完整课程：** [04-SDK逆解与候选姿态搜索.md](04-SDK逆解与候选姿态搜索.md)

**读完能回答：** IK 是项目自己解的吗？参考关节有什么作用？初始姿态无解时程序具体搜索哪些候选？

**主链：**

```text
抓取位姿 [x,y,z,rx,ry,rz]（mm+rad）
  -> solve_ik_for_pose()
  -> 选择 left/right ref_joint（rad）
  -> /<arm>/jaka_driver/get_ik
  -> C++ get_ik_callback()
  -> JAKA SDK robot.kine_inverse()
  -> 六关节角（rad）
  -> 同时对预抓取位姿求 IK
```

**必读路径与符号：**

1. `src/scene/realman_scene/script/robot_motion_control.py::solve_ik_for_pose()`
   - 输入：目标笛卡尔位姿、可选参考关节。
   - 下一步：无显式参考时使用当前侧 `ref_joint_left/right`，调用 `get_ik`。
   - 输出：六关节角 rad，异常时空列表。
2. `src/arm/jaka_robot_v2.2/src/jaka_driver/src/jaka_driver.cpp::get_ik_callback()`
   - 输入：参考关节与笛卡尔位姿。
   - 下一步：直接调用 `robot.kine_inverse()`。
   - 输出：SDK 返回的一个关节解，或 ROS 服务失败。
3. `src/scene/realman_scene/script/robot_vision_auto.py::cal_target_pose_array()`、`generate_sorted_angles()`、`robotPickJoint()`
   - 输入：初始抓取位姿。
   - 下一步：若无解，绕工具姿态 X 分量按 `0,+1.5°,-1.5°,...,+15°,-15°` 搜索；每个成功抓取姿态还要验证预抓取姿态。
   - 输出：成对可解的预抓取/抓取关节目标。

**重要分支：** 原始位姿有解时直接求预抓取解；原始位姿无解时才遍历候选。搜索是串行服务调用，不是一次批量 IK 优化。

**结果：** 项目负责候选姿态搜索和参考解选择；真正的逆运动学算法在闭源 JAKA SDK 中，源码无法证明它是解析法、雅可比迭代或其他方法。

**选读验证：** 对每个 IK 结果做 FK 回代并检查位置/姿态误差；当前正式 Python 主链没有做这一步。

**暂缓阅读：** IK 搜索去重、总耗时预算、关节限位评分和奇异性指标属于优化课，不先混入当前行为。

---

## 第 5 课：关键点运动怎样下发，SDK 与项目各负责什么

**状态：已展开（独立文件）**

**完整课程：** [05-关节关键点运动编排.md](05-关节关键点运动编排.md)

**读完能回答：** “预抓取到抓取是一条轨迹”在当前代码里到底意味着什么？项目有没有自己生成每个控制周期的关节轨迹？

**主链：**

```text
[pre_target_joints, target_joints]
  -> robotJointMoveTolSequenceL()
  -> 第一点要求 Stop/ready
  -> 两个 joint_move_tol 服务顺序下发，中间不等待到位
  -> C++ joint_move_tol_callback()
  -> SDK joint_move(..., is_block=false, tol=0.5)
  -> 只等待最后一条命令对应的运动完成
```

随后按以下离散关键点编排：

```text
预抓取 -> 抓取 -> 闭夹爪 -> 退回预抓取 -> 放置位 -> 开夹爪 -> 初始位
```

**必读路径与符号：**

1. `src/scene/realman_scene/script/robot_vision_auto.py::robotPickJoint()`
   - 输入：成对 IK 关节解、放置关节位、初始关节位。
   - 下一步：按抓取状态顺序调用运动与夹爪封装，任一步失败立即退出。
   - 输出：单目标抓取成功/失败。
2. `src/scene/realman_scene/script/robot_motion_control.py::robotJointMoveTolSequenceL()`
   - 输入：多个关节关键点。
   - 下一步：首点做 ready 检查，中间点只做故障检查，不做中间到位等待。
   - 输出：所有命令被接受且末条运动完成。
3. `src/scene/realman_scene/script/robot_motion_control.py::robotJointMoveTol()`、`_call_move_service()`
   - 输入：关节角 rad、速度 `180°/s`、加速度 `360°/s²`。
   - 下一步：调用 ROS `Move` 服务。
   - 输出：只表示驱动接受/拒绝命令。
4. `src/arm/jaka_robot_v2.2/src/jaka_driver/src/jaka_driver.cpp::joint_move_tol_callback()`
   - 输入：ROS 运动请求。
   - 下一步：调用 SDK 非阻塞 `joint_move`。
   - 输出：`ret=1` 表示 SDK 接受。

**重要分支：** 普通 `joint_move` 使用阻塞调用；`joint_move_tol` 使用非阻塞调用。正式抓取主链选用后者串联关键点。

**结果：** 项目编排的是离散关节关键点；连续插补、速度曲线和底层伺服由 JAKA SDK/控制器完成。当前主链没有自写五次多项式、S 曲线、PID、MPC 或逐周期 ServoJ。

**选读验证：** 读取 `joint_state` 记录，画出时间—关节角/速度曲线，确认 SDK 实际过点和停靠行为；源码本身不能证明“平滑”效果。

**暂缓阅读：** `linear_move`/笛卡尔直线运动封装不是当前 `robotPickJoint()` 主路径；ServoJ 放到第 8 课。

---

## 第 6 课：怎样用 `robot_states` 做运动完成监督

**状态：已展开（独立文件）**

**完整课程：** [06-robot_states运动完成监督.md](06-robot_states运动完成监督.md)

**读完能回答：** 为什么 ROS 服务返回成功不等于机械臂已经到位？程序怎样避免把运动前的旧 Stop 状态误判成本次完成？

**主链：**

```text
运动服务调用前记录 command_stamp
  -> SDK 接受命令
  -> 驱动以约 20 Hz 发布 robot_states
  -> 只接收 command_stamp 之后的新状态
  -> 优先观察 Running
  -> Running -> Stop
  -> 连续 3 个不同时间戳的 Stop
  -> 判定运动完成
```

**必读路径与符号：**

1. `src/scene/realman_scene/script/robot_motion_control.py::updateRobotStatus()`、`_robot_fault_reason()`、`_robot_ready_reason()`
   - 输入：motion/power/servo/collision 状态。
   - 下一步：缓存时间戳并把状态转换为“允许运动”或明确故障原因。
   - 输出：运动前置条件。
2. 同文件 `_call_move_service()`
   - 输入：运动请求。
   - 下一步：在调用前记录 `_last_motion_command_stamp`。
   - 输出：指令是否被服务接受。
3. 同文件 `_wait_motion_complete_after_command()`、`_wait_stop_state_confirmed()`
   - 输入：命令时间戳和超时。
   - 下一步：新鲜度检查、Running/Stop 状态机、短运动宽限、连续 Stop 计数。
   - 输出：完成、明确故障或超时。
4. `src/arm/jaka_robot_v2.2/src/jaka_driver/src/jaka_driver.cpp::robot_states_callback()`
   - 输入：SDK `is_in_pos` 与 `get_robot_status`。
   - 下一步：映射为 Stop/Pause/EmeStop/Running/Error。
   - 输出：`jaka_msgs/RobotMsg`。

**重要分支：** 短运动可能来不及观察到 Running；程序在 `motion_start_grace_s` 后允许连续 Stop 判定完成。状态未收到/过期可以等待，明确急停、错误、掉电或失能立即失败。

**结果：** 这是上层监督闭环，不是电机位置/速度/力矩控制闭环；它确认控制器报告的整体状态，没有逐关节到位误差闭环。

**关键风险：** 当前 C++ 驱动把 `robot_states.collision_state` 直接写为 `0`，真实碰撞查询被注释，因此 Python 中的 collision 检查目前没有有效碰撞反馈来源。

**选读验证：** `rostopic echo /<arm>/jaka_driver/robot_states` 与运动日志时间戳对齐；再用 `joint_state` 独立验证最终关节误差。

**暂缓阅读：** `wait_joint_reached()` 和 `wait_tcp_reached()` 只是兼容函数，当前仍转到 `robot_states` 判定，不应误认为已做位置误差闭环。

---

## 第 7 课：双臂如何在任务层协调，并怎样验证整条链

**状态：已展开（独立文件）**

**完整课程：** [07-双臂任务协调与链路验证.md](07-双臂任务协调与链路验证.md)

**读完能回答：** 两条单臂链什么时候汇合？怎样用假目标逐层判断故障在输入、变换、IK、运动还是工位完成逻辑？

**主链：**

```text
左右目标分别进入各自队列
  -> 各自 wrapper 独立执行
  -> /left_picker/target_result、/right_picker/target_result
  -> dual_pick_coordinator 统计目标/结果/arm_safe/vision_done
  -> 条件满足后发布 /dual_picker/station_done
```

**必读路径与符号：**

1. `src/scene/realman_scene/script/robot_vision_auto.py::_auto_pick_loop()`、`_execute_pick_target_job()`
   - 输入：已变换并入队的目标。
   - 下一步：初始位检查、逐目标执行、失败恢复、结果上报。
   - 输出：每臂目标级结果与 `arm_safe`。
2. `src/scene/realman_scene/script/dual_pick_coordinator.py::_target_cb()`、`_result_cb()`、`_completion_blocker_locked()`、`_maybe_publish_station_done()`
   - 输入：左右目标、左右结果、视觉完成信号。
   - 下一步：统计并检查视觉完成、结果齐全、机械臂安全、静默期等门槛。
   - 输出：工位状态和工位完成信号。
3. `src/pei_test/publish_right_fake_target_once`
   - 输入：当前右臂 TCP 反馈。
   - 下一步：构造 Base 安全假目标并反算为 TCP m 目标。
   - 输出：一次 `/right_arm/custom_arm_data`。
4. `src/pei_test/publish_left_fake_target_once`
   - 输入/输出与右臂脚本同构，仅切换命名空间与目标侧。

**重要分支：** wrapper 失败后会尝试回初始位；结果是否成功同时取决于抓取动作和 `arm_safe`。协调器是任务完成判定，不生成双臂同步轨迹，也不做双臂间碰撞规划。

**结果：** 当前“双臂协同”是并行单臂链的任务级汇总，不是共享时基的低层双臂协同控制。

**选读验证：** `DUAL_JAKA_FAKE_TARGET_TEST.md`、`DUAL_PICK_COORDINATOR.md`；按“话题存在 → 服务存在 → 反馈新鲜 → 单臂假目标 → 左右结果 → station_done”分层验证。

**暂缓阅读：** 假目标硬件在环可以覆盖接口、变换、IK、运动、夹爪与回位，但不能验证视觉精度、手眼标定精度或真实抓取质量。

---

## 第 8 课：MoveIt、RRTConnect 与 ServoJ 为什么不是当前主链

**状态：已展开（独立文件）**

**完整课程：** [08-MoveIt与ServoJ边界.md](08-MoveIt与ServoJ边界.md)

**读完能回答：** 仓库里明明有 OMPL、RRTConnect、FollowJointTrajectory 和 ServoJ，为什么不能说正式双臂采摘用了这些算法？

**主链对照：**

```text
正式启动链：dual_jaka_pick_bringup.launch
  -> jaka_driver + Python wrapper + coordinator
  -> 没有 move_group、规划场景或 FollowJointTrajectory action

MoveIt 示例链：MoveIt demo/脚本
  -> move_group + OMPL
  -> 默认 RRT / 脚本显式 RRTConnect
  -> FollowJointTrajectory
  -> moveit_server.cpp 逐点调用 SDK servo_j
```

**必读路径与符号：**

1. `src/scene/realman_scene/launch/dual_jaka_pick_bringup.launch`
   - 证据：正式启动项中没有 MoveIt 节点。
2. `src/arm/jaka_robot_v2.2/src/jaka_zu5_moveit_config/config/ompl_planning.yaml`
   - 输入：MoveIt 规划器配置。
   - 输出：可用 OMPL 规划器集合；这是能力配置，不是主链运行证据。
3. `src/arm/jaka_robot_v2.2/src/jaka_zu5_moveit_config/scripts/point_motion_adaptive.py`
   - 证据：示例显式选择 `RRTConnect`。
4. `src/arm/jaka_robot_v2.2/src/jaka_planner/src/moveit_server.cpp::goalCb()`
   - 输入：`FollowJointTrajectoryGoal`。
   - 下一步：按轨迹点调用 `servo_j`。
   - 输出：MoveIt 到 JAKA ServoJ 的实验桥接。
5. `src/arm/jaka_robot_v2.2/src/jaka_driver/src/jaka_driver.cpp::servo_j_callback()`
   - 证据：驱动提供 ServoJ 服务，但正式 `robotPickJoint()` 没有调用它。

**重要分支：** “仓库里有”只说明可研究或备用；“launch 启动 + 调用链到达”才能说明正式路径在用。

**结果：** 可以把 MoveIt/RRTConnect/ServoJ 作为后续升级方向，不能把它们写成当前采摘主链已经实现的在线避碰、轨迹规划或伺服控制能力。

**选读验证：** 运行时用 `rosnode list`、`rostopic info`、`rostopic echo` 证明 `move_group`、trajectory action 和 ServoJ 请求是否真实存在。

**暂缓阅读：** 真正升级到 MoveIt 前要先补齐双臂 URDF/SRDF、规划场景、碰撞体、控制器映射、时间参数化和实机安全验证，不能只改规划器名称。

---

## 学习顺序与阶段目标

1. 第 1～2 课：先能画出 ROS 拓扑，讲清坐标系、消息格式和单位。
2. 第 3～4 课：再吃透“目标几何 → 抓取位姿 → SDK IK”。
3. 第 5～6 课：理解项目上层编排、SDK 边界和状态监督，不混淆成底层控制算法。
4. 第 7 课：用假目标把单臂到双臂工位完成链走通。
5. 第 8 课：最后对照 MoveIt/ServoJ，判断现状与升级路径。

完成这八课后，你应该能用源码证据回答：当前项目真正实现了哪些运控逻辑、哪些由厂商 SDK 完成、哪些只是示例或待接入能力，以及怎样分层验证实机行为。
