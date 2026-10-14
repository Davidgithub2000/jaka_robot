# vision_calibrate

#### 介绍
新版基于realsense相机的内参及手眼标定
其中手眼部分直接适用于以/tool_point话题发布机械臂位置与四元数的标定

#### 安装教程

1.  将vision_calibrate复制到catkin_ws/src下进行编译即可


#### 使用说明

1.  内参标定：roslaunch vision_calibrate vision_camera_calibration.launch
2.  手眼标定：roslaunch vision_calibrate vision_camera_calibration_handeye.launch
3.  其中相关内参及手眼标定保存路径见上述launch文件

#### 注意事项
由于使用的是python3版本，因此calibration_hand_eye.py 和 camera_calibrate.py 脚本的Shebang，
需修改为对应的python环境中：
如：#!/home/ros/QRcode_env/bin/python3

