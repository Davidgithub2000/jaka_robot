# 老双臂代码（2024-12-15 归档）

此目录整理自原始 `老双臂代码/src_dual_arm——20241215/src_dual_arm` 工作区。
`src/` 保留原有 ROS 包层级，包含视觉识别与标定、JAKA 机械臂、夹爪、手眼标定、
场景和串口代码。它是供回顾的历史快照，与仓库根目录的当前 `src/` 分开；
不要将两套同名 ROS 包同时加入一个 catkin 工作区。

本次同步了源码、配置、launch、机器人描述和必要网格文件。
原目录中的训练权重、训练运行记录、采集/标定图像与标签、Python 缓存、IDE 与嵌套 Git
元数据、压缩包、厂商手册、工程图和 MATLAB 数据文件未纳入代码快照。
原目录有三个指向旧 Linux 路径的符号链接，目标在当前电脑上不可用，因此没有复制：
`src/CMakeLists.txt`、`src/VC/vision_identity/eyeInHand_calibration.yaml`、
`src/VC/vision_identity/eyeToHand_calibration.yaml`。

此快照没有经过编译或实机验证。若要单独构建，应在 ROS 环境中检查依赖、
重新建立 catkin 顶层 `src/CMakeLists.txt`，并按现场配置处理上述标定文件。
