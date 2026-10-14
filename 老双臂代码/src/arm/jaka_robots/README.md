# jaka_robots

[![License](https://img.shields.io/badge/license-Apache2.0-blue)](./LICENSE)
[![Release](https://img.shields.io/badge/release-master-blue)](https://github.com/cartographer-project)
[![Author](https://img.shields.io/badge/Author-Boen-blue)](https://gitee.com/zhwl_1981)

#### 介绍
JAKA机械臂ROS驱控程序，兼容部分原厂话题和服务接口

#### 软件架构
基于JAKA的TCP通信协议接口开发，传输的数据采用JSON格式

#### 使用说明
1.  设置机械臂IP地址： ${project}/jaka_driver/jaka_bringup/config/jaka.yaml  ---(ipv4:XXX.XXX.XXX.XXX)
2.  系统启动命令：roslaunch jaka_bringup start.launch 

#### 数据输入
1.  支持关节角度控制：/robot_driver/move_joint   --数据为角度
2.  支持空间线性控制：/robot_driver/move_line    --数据为毫米和角度
3.  支持气泵控制：/gripper                       --气泵控制[0，1]

#### 数据输出
1.  反馈机械臂位姿：/robot_driver/tool_point
2.  反馈机械臂关节角度：/robot_driver/joint_states
3.  反馈机械臂运动状态：/robot_driver/robot_state

