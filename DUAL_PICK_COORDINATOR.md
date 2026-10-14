# 双臂采摘协调器说明

## 协调器作用

节点名称：

```text
/dual_pick_coordinator
```

代码文件：

```text
src/scene/realman_scene/script/dual_pick_coordinator.py
```

启动文件：

```text
src/scene/realman_scene/launch/dual_jaka_pick_bringup.launch
```

协调器负责统计左右臂目标数量、采摘结果、机械臂安全状态和视觉完成状态，然后向底盘发布当前工位完成信号。

协调器不直接控制机械臂、夹爪或底盘运动。

## 涉及节点

| 节点 | 功能 |
| --- | --- |
| 左侧视觉节点 | 发布左臂目标 |
| 右侧视觉节点 | 发布右臂目标 |
| `/left_picker/robot_vision_wrapper` | 执行左臂采摘并发布结果 |
| `/right_picker/robot_vision_wrapper` | 执行右臂采摘并发布结果 |
| `/dual_pick_coordinator` | 统计双臂状态并判断工位是否完成 |
| 底盘或任务管理节点 | 发布工位开始，接收工位完成 |

JAKA驱动和夹爪节点由左右采摘节点直接控制，不与协调器直接通信。

## 订阅话题

| 话题 | 消息类型 | 作用 |
| --- | --- | --- |
| `/left_arm/custom_arm_data` | `std_msgs/Float32MultiArray` | 统计左臂视觉目标 |
| `/right_arm/custom_arm_data` | `std_msgs/Float32MultiArray` | 统计右臂视觉目标 |
| `/left_picker/target_result` | `std_msgs/String` | 接收左臂采摘结果和安全状态 |
| `/right_picker/target_result` | `std_msgs/String` | 接收右臂采摘结果和安全状态 |
| `/dual_picker/station_start` | `std_msgs/Bool` | 开始或重置一个工位任务 |
| `/dual_picker/vision_done` | `std_msgs/Bool` | 视觉确认当前工位没有更多目标 |

## 发布话题

| 话题 | 消息类型 | 作用 |
| --- | --- | --- |
| `/dual_picker/station_done` | `std_msgs/Bool` | 通知底盘当前工位已完成 |
| `/dual_picker/station_status` | `std_msgs/String` | 发布JSON格式调试状态 |

## 工位完成条件

当前配置下，需要同时满足：

1. 已收到工位开始信号，或收到第一个目标后自动开始。
2. 视觉发布 `/dual_picker/vision_done=true`。
3. 左右臂收到的目标都已有对应结果。
4. 机械臂结果中的 `arm_safe=true`。
5. 等待约 `0.5 s` 后发布 `/dual_picker/station_done=true`。

当前 `done_requires_all_success=false`，所以采摘失败不会阻止工位完成，但机械臂必须安全回位。

## 联动流程

```text
底盘到达工位
    -> 发布 station_start=true
    -> 视觉发布左右目标
    -> 左右采摘节点执行机械臂和夹爪动作
    -> 左右采摘节点发布 target_result
    -> 视觉确认无剩余目标并发布 vision_done=true
    -> 协调器确认所有目标已有结果且机械臂安全
    -> 发布 station_done=true
    -> 底盘前往下一工位
```
