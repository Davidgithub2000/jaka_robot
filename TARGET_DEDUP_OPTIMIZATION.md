# 双臂采摘目标去重优化建议

## 当前逻辑

机械臂将视觉目标转换到基坐标系，计算 `p1` 和 `p2` 的中点。如果新目标与历史目标的中点距离不超过 `30 mm`，并且历史记录未超过 `60 s`，则判定为重复目标。

当前参数：

```xml
<param name="enable_target_dedup" type="bool" value="true"/>
<param name="target_dedup_radius_mm" type="double" value="30.0"/>
<param name="target_dedup_ttl_s" type="double" value="60.0"/>
<param name="target_dedup_max_len" type="int" value="100"/>
```

主要文件：

```text
src/scene/realman_scene/script/robot_vision_auto.py
src/scene/realman_scene/script/robot_vision_config.py
src/scene/realman_scene/script/dual_pick_coordinator.py
src/scene/realman_scene/launch/dual_jaka_wrapper.launch
```

## 主要问题

1. 新工位中的目标可能与上一工位目标具有相似基坐标，从而被错误去重。
2. `QUEUED`、`DONE` 和 `FAILED` 使用相同的去重时间，失败目标无法合理重试。
3. 只比较目标中点，两个靠得很近的番茄可能被误认为同一目标。
4. 去重检查和历史登记是两个操作，多个消息同时到达时存在重复入队风险。
5. 队列拒绝或挤出的目标可能仍保留为 `QUEUED`，导致后续无法重新发布。
6. 重复目标当前以 `success=false` 发布结果，会被协调器计入失败次数。

## 优化方案

### 1. 按工位管理去重历史

- 新工位开始时清空上一工位的去重记录。
- `DONE` 目标在当前工位内始终保持去重。
- 工位结束后统一清理记录。

建议使用 `station_id` 或工位开始话题区分不同工位。

### 2. 根据目标状态使用不同规则

| 状态 | 建议处理 |
| --- | --- |
| `QUEUED/EXECUTING` | 严格禁止重复入队 |
| `DONE` | 当前工位内永久去重 |
| `FAILED` | 冷却约5秒后允许重试，最多1至2次 |
| `DROPPED` | 删除记录或立即允许重新发布 |

### 3. 增加目标几何特征比较

建议同时满足以下条件才判定为同一目标：

```text
中点距离 <= 30 mm
果梗方向夹角 <= 20 deg
p1-p2 长度差 <= 15~20 mm
```

这样可以减少相邻番茄之间的误判。

### 4. 原子化检查和登记

将以下两个过程合并到同一个锁内：

```text
检查是否重复 -> 预占目标 -> 写入 QUEUED 状态
```

避免两个同时到达的相同目标都进入采摘队列。

### 5. 修正队列失败状态

目标只有成功进入队列后才能保持 `QUEUED`。如果队列拒绝、队列满被挤出或任务门控关闭，应将记录改为 `DROPPED` 或直接删除。

### 6. 协调器区分重复目标

重复目标不应作为采摘失败统计，建议结果增加：

```text
status=ignored_duplicate
completed=true
arm_safe=true
```

协调器将其计入“已处理/已忽略”，但不计入 `failed`，避免以后启用 `done_requires_all_success=true` 时阻止工位完成。

### 7. 视觉提供目标ID

长期建议使用自定义消息，增加：

```text
station_id
target_id
p1
p2
confidence
```

机械臂优先使用 `station_id + target_id` 去重，空间位置只作为目标ID异常时的备用判断。

## 推荐初始参数

```text
空间去重半径：30 mm
FAILED冷却时间：5 s
FAILED最大重试次数：2
果梗方向阈值：20 deg
果梗长度差阈值：20 mm
当前工位DONE记录：工位结束前不超时
```

## 推荐实施顺序

1. 工位切换时清空去重历史。
2. `FAILED` 目标允许有限次数重试。
3. 重复目标不计入协调器失败次数。
4. 合并去重检查和目标预占操作。
5. 增加果梗方向和长度判断。
6. 最后升级为带 `station_id` 和 `target_id` 的自定义视觉消息。
