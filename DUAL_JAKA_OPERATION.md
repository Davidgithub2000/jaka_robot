# 双 JAKA 采摘系统启动与验证流程

本文档用于工控机 `~/ws_double_jaka` 工作区的双臂启动、状态验证、wrapper 验证和视觉接入前检查。

当前阶段目标：

1. 启动两台 JAKA driver。
2. 确认左右机械臂状态、关节、TCP 位姿正常。
3. 启动两个 wrapper。
4. 确认 wrapper 分别绑定左右机械臂。
5. 视觉暂时只接入话题，不直接连续触发采摘。

## 0. 启动前安全检查

每次启动前先确认：

- 急停按钮可用。
- 两台机械臂周围无人。
- JAKA App/控制柜状态正常。
- 工控机网口和两台机械臂在同一网段。
- 左臂 IP：`192.168.1.181`
- 右臂 IP：`192.168.1.188`
- 不要先发布 `/left_arm/custom_arm_data` 或 `/right_arm/custom_arm_data`。

如果刚改过代码，先编译：

```bash
cd ~/ws_double_jaka
conda deactivate
rm -rf build devel
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

如果只是重新启动，不需要每次重新编译。

## 1. 终端 1：启动 roscore

```bash
cd ~/ws_double_jaka
conda deactivate
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roscore
```

这个终端保持打开，不要关闭。

## 2. 终端 2：检查网络

```bash
cd ~/ws_double_jaka
conda deactivate
source /opt/ros/noetic/setup.bash
source devel/setup.bash
ping 192.168.1.181
ping 192.168.1.188
```

两个 IP 都能 ping 通后，按 `Ctrl+C` 停止 ping。

## 3. 终端 2：启动双 JAKA driver

继续在终端 2 执行：

```bash
roslaunch jaka_driver dual_jaka_driver.launch
```

该 launch 会启动：

```text
/left_arm/jaka_driver
/right_arm/jaka_driver
```

当前 driver 启动时会自动尝试上电和使能机械臂。  
如果 JAKA App 已经下电/下使能，启动 driver 后等待十几秒，再检查状态。

## 4. 终端 3：验证 driver

新开终端 3：

```bash
cd ~/ws_double_jaka
conda deactivate
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

检查节点：

```bash
rosnode list | grep jaka_driver
```

期望看到：

```text
/left_arm/jaka_driver
/right_arm/jaka_driver
```

检查服务：

```bash
rosservice list | grep left_arm
rosservice list | grep right_arm
```

至少应看到：

```text
/left_arm/jaka_driver/joint_move
/left_arm/jaka_driver/linear_move
/left_arm/jaka_driver/stop_move
/right_arm/jaka_driver/joint_move
/right_arm/jaka_driver/linear_move
/right_arm/jaka_driver/stop_move
```

检查机械臂状态：

```bash
rostopic echo -n 1 /left_arm/jaka_driver/robot_states
rostopic echo -n 1 /right_arm/jaka_driver/robot_states
```

期望：

```text
motion_state: 0
power_state: 1
servo_state: 1
collision_state: 0
```

含义：

- `motion_state: 0`：停止状态。
- `power_state: 1`：已上电。
- `servo_state: 1`：已使能。
- `collision_state: 0`：无碰撞报警。

检查关节状态：

```bash
rostopic echo -n 1 /left_arm/jaka_driver/joint_state
rostopic echo -n 1 /right_arm/jaka_driver/joint_state
```

期望两边都有 6 个 joint：

```text
joint_1
joint_2
joint_3
joint_4
joint_5
joint_6
```

检查 TCP 位姿：

```bash
rostopic echo -n 1 /left_arm/jaka_driver/tool_position
rostopic echo -n 1 /right_arm/jaka_driver/tool_position
```

期望两边都有 `linear` 和 `angular` 数据。

## 5. 终端 4：启动双 wrapper

确认 driver 正常后，新开终端 4：

```bash
cd ~/ws_double_jaka
conda deactivate
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch realman_scene dual_jaka_wrapper.launch
```

这个终端保持打开。  
正常时会看到两个 wrapper 启动，并持续提示等待 `/custom_arm_data` 目标消息。

## 6. 终端 3：验证 wrapper

回到终端 3 执行：

```bash
rosnode list | grep robot_vision_wrapper
```

期望看到：

```text
/left_picker/robot_vision_wrapper
/right_picker/robot_vision_wrapper
```

检查左 wrapper 绑定左 driver：

```bash
rosparam get /left_picker/robot_vision_wrapper/driver_ns
```

期望：

```text
/left_arm/jaka_driver
```

检查右 wrapper 绑定右 driver：

```bash
rosparam get /right_picker/robot_vision_wrapper/driver_ns
```

期望：

```text
/right_arm/jaka_driver
```

检查目标话题订阅：

```bash
rostopic info /left_arm/custom_arm_data
rostopic info /right_arm/custom_arm_data
```

期望：

```text
Type: std_msgs/Float32MultiArray

Subscribers:
 * /left_picker/robot_vision_wrapper
```

以及：

```text
Type: std_msgs/Float32MultiArray

Subscribers:
 * /right_picker/robot_vision_wrapper
```

检查 wrapper 是否订阅对应 TCP 位姿：

```bash
rostopic info /left_arm/jaka_driver/tool_position
rostopic info /right_arm/jaka_driver/tool_position
```

期望：

```text
Subscribers:
 * /left_picker/robot_vision_wrapper
```

以及：

```text
Subscribers:
 * /right_picker/robot_vision_wrapper
```

到这里，双 driver 和双 wrapper 就算启动验证完成。

## 7. 一键启动方式

只有在上面的分步启动都确认正常后，才使用一键启动。

先在终端 2 和终端 4 按 `Ctrl+C` 停掉 driver 和 wrapper，然后执行：

```bash
cd ~/ws_double_jaka
conda deactivate
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch realman_scene dual_jaka_pick_bringup.launch
```

一键启动会同时启动：

```text
/left_arm/jaka_driver
/right_arm/jaka_driver
/left_picker/robot_vision_wrapper
/right_picker/robot_vision_wrapper
```

## 8. 视觉接入前检查

wrapper 目前等待两个视觉目标话题：

```text
/left_arm/custom_arm_data
/right_arm/custom_arm_data
```

消息类型必须是：

```text
std_msgs/Float32MultiArray
```

数据格式必须是：

```text
data: [p1x, p1y, p1z, p2x, p2y, p2z]
```

当前约定：

- `p1` 和 `p2` 是当前 TCP 坐标系下的两个点。
- 默认单位是 `m`。
- 如果视觉输出单位是 `mm`，需要在 `dual_jaka_wrapper.launch` 中把左右 wrapper 的 `vision_target_unit` 改成 `mm`。
- 视觉节点应该把左臂目标发到 `/left_arm/custom_arm_data`。
- 视觉节点应该把右臂目标发到 `/right_arm/custom_arm_data`。
- 不要两个臂同时收到同一个目标。

检查视觉是否有发布者：

```bash
rostopic info /left_arm/custom_arm_data
rostopic info /right_arm/custom_arm_data
```

检查视觉消息内容：

```bash
rostopic echo -n 1 /left_arm/custom_arm_data
rostopic echo -n 1 /right_arm/custom_arm_data
```

注意：当前 `use_usr_task_gate=False`，收到有效目标后 wrapper 会进入采摘流程。  
第一次测试视觉时，不要连续发布目标；先单次发布、低速、小范围验证。

## 9. 停机流程

当前 ROS driver 没有自动下使能/下电逻辑，也没有暴露下使能/下电 service。

推荐停机顺序：

1. 停 wrapper：在终端 4 按 `Ctrl+C`。
2. 停 driver：在终端 2 按 `Ctrl+C`。
3. 用 JAKA App/示教器下使能。
4. 用 JAKA App/示教器下电。
5. 工控机正常关机。

工控机正常关机命令：

```bash
sudo shutdown now
```

如果只是停止当前运动，可以执行：

```bash
rosservice call /left_arm/jaka_driver/stop_move "{}"
rosservice call /right_arm/jaka_driver/stop_move "{}"
```

`stop_move` 只停止运动，不等于下使能或下电。

## 10. 常见异常

### wrapper 没有启动

检查：

```bash
rosnode list | grep robot_vision_wrapper
```

如果没有输出，查看 wrapper 终端是否有 Python 报错。

### driver 状态不是 1

检查：

```bash
rostopic echo -n 1 /left_arm/jaka_driver/robot_states
rostopic echo -n 1 /right_arm/jaka_driver/robot_states
```

如果：

```text
power_state: 0
servo_state: 0
```

说明机械臂未上电或未使能。可以先在 JAKA App 中处理，再重启 driver。

### ping 不通

检查：

- 工控机 IP。
- 网线。
- 交换机。
- JAKA App 是否能发现机械臂。
- 两台 JAKA IP 是否仍是 `192.168.1.181` 和 `192.168.1.188`。

