# 双 JAKA 机械臂视觉采摘（ROS1）

这个仓库整理自 `ws_double_jaka` catkin 工作区。正式双臂入口是
`src/scene/realman_scene/launch/dual_jaka_pick_bringup.launch`：分别启动左右 JAKA
驱动、视觉采摘 wrapper 和工位协调节点。视觉目标经过 TCP 到基坐标系变换、双点抓取
姿态构造和 JAKA SDK 逆解，再由关节运动服务执行预抓取、抓取、放置和回位。

## 代码位置

- `src/scene/realman_scene/script/`：视觉目标、运动控制与双臂任务协调。
- `src/scene/realman_scene/launch/`：左右臂 wrapper 和一键启动配置。
- `src/arm/jaka_robot_v2.2/src/jaka_driver/`：ROS 与 JAKA SDK 的服务接口。
- `src/arm/jaka_robot_v2.2/src/jaka_msgs/`：驱动消息和服务定义。
- `src/z_efg_ros/`：双夹爪节点。
- `src/scene/jaka_handeye_calibration/`、`src/scene/jaka_d435i_camera/`：标定与相机工具。
- `tutorial.md` 及 `01-` 至 `08-` 文档：当前运控主链的源码导学。

## 部署入口

运行环境是 Linux + ROS1 Noetic。安装 JAKA SDK 及项目需要的 ROS 依赖并构建工作区后，
先启动 `roscore`，再启动 `roslaunch z_efg_ros multi_gripper.launch`，最后使用
`roslaunch realman_scene dual_jaka_pick_bringup.launch`。真实视觉脚本在原部署环境的
`~/yolo`，不属于这个工作区；目标接口是左右独立的 `custom_arm_data` 话题，消息为
`std_msgs/Float32MultiArray` 的六个 TCP 坐标值，单位米。

驱动启动时可能自动上电、使能；wrapper 启动时可能自动回初始位。实际运行前需核对
设备 IP、TCP、关节初始位和现场运动空间。详见 `DUAL_JAKA_OPERATION.md` 与
`DUAL_JAKA_REAL_VISION_STARTUP.md`。

## 快照范围

本次同步保留项目可阅读的控制源码、ROS 包、launch 和说明。未提交本地 IDE 缓存、
Python 缓存、历史压缩包及厂商 SDK 二进制 `.so`。原工作区还包含大量厂商模型、
MoveIt 示例和停用的 `jaka_driver_2.2.7` 副本；这些文件不属于当前双臂采摘启动链。
此仓库未在实机上验证编译或运行，构建驱动前需要按设备 SDK 的授权方式补齐库文件。

## 历史双臂代码

`老双臂代码/` 保存 2024-12-15 归档工作区中的源码快照，与上面的当前 `src/`
主链分开存放。历史代码的目录说明和未同步文件类型见
[`老双臂代码/README.md`](老双臂代码/README.md)。
