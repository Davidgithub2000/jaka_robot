# 双臂 IK 与运动效率优化建议

## 当前情况

- 左右臂分别运行独立采摘节点，可以并行处理目标。
- 预抓取点到抓取点采用连续关节指令，只在末端等待机器人停止。
- 夹爪根据反馈状态判断完成，不再依赖固定等待时间。
- 当前关节运动速度约为 `180 deg/s`，不建议直接继续提高全局速度。

## 优先优化项

### 1. 调整 IK 解算顺序

当前流程先解抓取点，再解预抓取点，并使用固定参考关节。

建议改为：

```text
当前关节 -> 解算预抓取点 -> 使用预抓取关节作为参考解算抓取点
```

作用：减少腕部绕圈、关节跳变和无效绕行。

修改位置：

```text
src/scene/realman_scene/script/robot_vision_auto.py
robotPickJoint()
```

### 2. 减少无效 IK 尝试

当前姿态搜索范围为 `0 deg、+/-1.5 deg ... +/-15 deg`，最多约 21 次，并且 `0 deg` 会重复计算。

建议改为：

```text
+/-3 deg、+/-6 deg、+/-9 deg、+/-12 deg、+/-15 deg
```

作用：缩短不可解目标的等待时间。

修改位置：

```text
robot_vision_auto.py
cal_target_pose_array()
```

### 3. 增加 IK 结果保护与评分

对候选关节结果检查：

- 与当前关节的总距离
- J6 变化量
- 是否接近关节限位
- 是否进入机械臂自身或车体危险姿态

优先选择关节变化小、路径短且安全的解。

主要修改位置：

```text
src/scene/realman_scene/script/robot_motion_control.py
solve_ik_for_pose()
```

### 4. 删除重复停止确认

返回初始位时，`robotJointMoveTolL(wait=True)` 已经等待机器人停止，后续再次调用 `wait_arm_motion_stable()` 属于重复确认。

修改位置：

```text
robot_vision_auto.py
robotPickJoint() 返回初始位之后
```

### 5. 分阶段设置运动速度

建议设置不同速度：

| 运动阶段 | 建议 |
| --- | --- |
| 初始位到预抓取点 | 较快 |
| 预抓取点到抓取点 | 较慢 |
| 抓取后退 | 中速 |
| 回收位和返回初始位 | 较快 |

当前速度设置位置：

```text
src/scene/realman_scene/script/robot_motion_control.py
robotJointMoveTol()
```

## 推荐实施顺序

1. 预抓取点先解算，抓取点使用预抓取关节作为参考。
2. 增加 J6 跳变保护和关节距离检查。
3. 减少 IK 姿态搜索次数。
4. 删除重复停止确认。
5. 实测记录各阶段耗时后，再调整分阶段速度。

安全测试时应先使用低速和假目标，确认左右臂路径、夹爪反馈和回初始位均正常后，再逐步提高速度。
