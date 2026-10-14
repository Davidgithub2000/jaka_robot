# g3p

[![License](https://img.shields.io/badge/license-Apache2.0-blue)](./LICENSE)
[![Release](https://img.shields.io/badge/release-master-blue)](https://github.com/cartographer-project)
[![Author](https://img.shields.io/badge/Author-Boen-blue)](https://gitee.com/zhwl_1981)

#### 介绍
三指柔性夹爪

#### 软件架构
平台版本：ubuntu18.04
ROS系统：melodic

#### 安装依赖
sudo apt install ros-melodic-serial 


#### 使用说明
1.  直接驱动测试
    加载驱动： 
    roslaunch g3p_driver hw_link.launch
    
    测试命令：（到达11cm行程）
    rostopic pub -1 /g3p_msgs/gripper g3p_msgs/Gripper -- '11'

2. 可视化驱动测试
    加载驱动：
    roslaunch g3p_driver hw_link.launch

    加载可视化：(RVIZ)
    roslaunch g3p_description view_arm.launch
  
    测试命令：（到达11cm行程）
    rostopic pub -1 /g3p_msgs/gripper g3p_msgs/Gripper -- '11'

** 测试前请将夹爪恢复到初始位置，夹爪螺杆行程0～11厘米

** 支持的驱动变更成DRV8825，如果需要使用旧版驱动，需要在代码中取消该宏定义

