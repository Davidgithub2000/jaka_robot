# 双臂假目标采摘测试流程

## 1. 启动 roscore

终端 1：

```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source ~/ws_double_jaka/devel/setup.bash
roscore
```

## 2. 启动双夹爪

终端 2：

```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source ~/ws_double_jaka/devel/setup.bash
roslaunch z_efg_ros multi_gripper.launch
```

正常日志应包含：

```text
Gripper 1 initialized and enabled
Gripper 2 initialized and enabled
Multi-gripper ready
```

## 3. 启动双臂采摘系统

终端 3：

```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source ~/ws_double_jaka/devel/setup.bash
roslaunch realman_scene dual_jaka_pick_bringup.launch
```

正常日志应包含：

```text
等待 /custom_arm_data 目标消息
```

## 4. 单独测试夹爪

终端 4：

```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source ~/ws_double_jaka/devel/setup.bash
```

右夹爪 ID 2：

```bash
rostopic pub -1 /multi_gripper/cmd z_efg_ros/GripperCmd "{id: 2, grip: true}"
sleep 2
rostopic pub -1 /multi_gripper/cmd z_efg_ros/GripperCmd "{id: 2, grip: false}"
```

左夹爪 ID 1：

```bash
rostopic pub -1 /multi_gripper/cmd z_efg_ros/GripperCmd "{id: 1, grip: true}"
sleep 2
rostopic pub -1 /multi_gripper/cmd z_efg_ros/GripperCmd "{id: 1, grip: false}"
```

如果夹爪不动，先不要发假目标，检查终端 2 的夹爪日志。

## 5. 发布右臂假目标

终端 5：

```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source ~/ws_double_jaka/devel/setup.bash
bash ~/ws_double_jaka/src/pei_test/publish_right_fake_target_once
```

右臂正常流程：

```text
收到右臂目标
运动到预抓取点
运动到抓取点
右夹爪 ID 2 闭合
退回预抓取点
运动到右侧放果位置
右夹爪 ID 2 张开
回右侧初始位置
单目标采摘完成
```

## 6. 发布左臂假目标

确认右臂已经回到初始位置后，在终端 5 执行：

```bash
bash ~/ws_double_jaka/src/pei_test/publish_left_fake_target_once
```

左臂正常流程：

```text
收到左臂目标
运动到预抓取点
运动到抓取点
左夹爪 ID 1 闭合
退回预抓取点
运动到左侧放果位置
左夹爪 ID 1 张开
回左侧初始位置
单目标采摘完成
```

## 7. 常用检查命令

检查目标话题：

```bash
rostopic echo /right_arm/custom_arm_data
rostopic echo /left_arm/custom_arm_data
```

检查夹爪命令：

```bash
rostopic echo /multi_gripper/cmd
```

检查夹爪状态：

```bash
rostopic echo /multi_gripper/state
```

检查采摘结果：

```bash
rostopic echo /right_picker/target_result
rostopic echo /left_picker/target_result
```

检查工位完成：

```bash
rostopic echo /dual_picker/station_done
rostopic echo /dual_picker/station_status
```

## 8. 正常判断标准

右臂应看到：

```text
gripper3ChangeState: topic=/multi_gripper/cmd, id=2, grip=1
gripper3ChangeState: topic=/multi_gripper/cmd, id=2, grip=0
单目标采摘完成
```

左臂应看到：

```text
gripper3ChangeState: topic=/multi_gripper/cmd, id=1, grip=1
gripper3ChangeState: topic=/multi_gripper/cmd, id=1, grip=0
单目标采摘完成
```

协调器应看到：

```text
Station ... done
```

