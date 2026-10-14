# 双臂采摘启动流程简版

## 1. 启动前检查

1. 两台 JAKA 机械臂上电、使能、无急停、无碰撞报警。
2. IP 确认：
   - 左臂 181：`192.168.1.181`
   - 右臂 188：`192.168.1.188`
3. 如果终端在 conda 环境内，先退出：

```bash
conda deactivate
```

## 2. 启动 ROS

终端 1：

```bash
roscore
```

终端 2：

```bash
cd ~/ws_double_jaka
source devel/setup.bash
roslaunch realman_scene dual_jaka_pick_bringup.launch
```

这个 launch 会启动：

- 左右 JAKA driver
- 左右采摘 wrapper
- 双臂采摘协调节点 `dual_pick_coordinator`

## 3. 检查话题

```bash
rostopic list | grep -E "dual_picker|target_result|custom_arm_data"
```

应看到：

```bash
/left_arm/custom_arm_data
/right_arm/custom_arm_data
/left_picker/target_result
/right_picker/target_result
/dual_picker/station_done
/dual_picker/station_status
/dual_picker/station_start
/dual_picker/vision_done
```

检查订阅关系：

```bash
rostopic info /left_arm/custom_arm_data
rostopic info /right_arm/custom_arm_data
rostopic info /dual_picker/station_done
```

## 4. 视觉发布目标

左臂目标话题：

```bash
/left_arm/custom_arm_data
```

右臂目标话题：

```bash
/right_arm/custom_arm_data
```

消息类型：

```bash
std_msgs/Float32MultiArray
```

格式：

```bash
data: [p1_x, p1_y, p1_z, p2_x, p2_y, p2_z]
```

要求：

- 坐标系：对应机械臂当前 TCP 坐标系
- 单位：m
- `p1`、`p2` 是同一个目标上的两个三维点
- 果梗方向按 `p2 -> p1` 计算

## 5. 假目标测试

右臂 188：

```bash
bash ~/ws_double_jaka/src/pei_test/publish_right_fake_target_once
```

左臂 181：

```bash
bash ~/ws_double_jaka/src/pei_test/publish_left_fake_target_once
```

建议左右分开测试，不要同时发两个假目标。

## 6. 底盘联动

底盘到达工位后，可以发布一次工位开始信号，用于重置协调节点状态：

```bash
rostopic pub -1 /dual_picker/station_start std_msgs/Bool "data: true"
```

视觉发布目标后，机械臂完成每个目标会发布：

```bash
/left_picker/target_result
/right_picker/target_result
```

协调节点判断当前工位完成后发布：

```bash
/dual_picker/station_done
```

类型：

```bash
std_msgs/Bool
```

底盘订阅这个话题。收到：

```bash
data: true
```

表示当前工位采摘完成，可以去下一个工位。

查看工位状态：

```bash
rostopic echo /dual_picker/station_status
```

查看完成信号：

```bash
rostopic echo /dual_picker/station_done
```

如果视觉能明确判断当前工位没有目标了，可以额外发布：

```bash
rostopic pub -1 /dual_picker/vision_done std_msgs/Bool "data: true"
```

## 7. 常用状态检查

```bash
rostopic echo -n 1 /left_arm/jaka_driver/robot_states
rostopic echo -n 1 /right_arm/jaka_driver/robot_states

rostopic echo -n 1 /left_arm/jaka_driver/tool_position
rostopic echo -n 1 /right_arm/jaka_driver/tool_position
```

## 8. 停止

按 `Ctrl+C` 依次停止：

1. `dual_jaka_pick_bringup.launch`
2. `roscore`

