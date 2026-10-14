# 双臂真实视觉采摘启动流程

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

## 4. 启动真实视觉节点

终端 4：

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate yolo
cd ~/yolo
python vision_inference_v11_rotated_caculateInerP_1215best.py
```

如果视觉节点找不到 ROS 包，再改用：

```bash
source /opt/ros/noetic/setup.bash
source ~/ws_double_jaka/devel/setup.bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate yolo
cd ~/yolo
python vision_inference_v11_rotated_caculateInerP_1215best.py
```

## 5. 视觉发布要求

左臂目标话题：

```text
/left_arm/custom_arm_data
```

右臂目标话题：

```text
/right_arm/custom_arm_data
```

消息类型：

```text
std_msgs/Float32MultiArray
```

数据格式：

```text
data: [p1_x, p1_y, p1_z, p2_x, p2_y, p2_z]
```

要求：

```text
单位：m
坐标系：对应机械臂当前 TCP 坐标系
p2 -> p1 表示果梗方向
左相机只发左臂话题
右相机只发右臂话题
```

## 6. 常用检查命令

新开终端时先执行：

```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source ~/ws_double_jaka/devel/setup.bash
```

检查视觉目标：

```bash
rostopic echo /left_arm/custom_arm_data
rostopic echo /right_arm/custom_arm_data
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
rostopic echo /left_picker/target_result
rostopic echo /right_picker/target_result
```

检查工位完成信号：

```bash
rostopic echo /dual_picker/station_done
rostopic echo /dual_picker/station_status
```

## 7. 判断是否正常

收到视觉目标后，采摘系统日志应出现：

```text
收到 /custom_arm_data 单次目标并入队
TCP->base
p1_base_mm=...
p2_base_mm=...
```

目标转换到 base 后，两个臂当前都应在负 Y 工作空间内：

```text
x: -100 ~ 600 mm
y: -820 ~ -400 mm
z: 80 ~ 810 mm
```

夹爪正常时应看到：

```text
gripper3ChangeState: topic=/multi_gripper/cmd, id=1 或 2, grip=1
gripper3ChangeState: topic=/multi_gripper/cmd, id=1 或 2, grip=0
```

