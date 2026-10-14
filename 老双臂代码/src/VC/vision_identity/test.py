import pyrealsense2 as rs
import numpy as np
import cv2
 
# 初始化深度相机
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)
 
# 创建深度图像的伪彩色映射器
colorizer = rs.colorizer()
 
# 回调函数，处理鼠标点击事件
def on_mouse_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:  # 当左键被点击时
        distance = depth_frame.get_distance(x, y)  # 获取鼠标点击点的距离
        distance_in_meters = round(distance, 2)
        cv2.putText(combined_image, f"Distance: {distance_in_meters} m", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        print(f"Distance at ({x}, {y}): {distance_in_meters}m")  # 打印距离信息到终端
        cv2.imshow("Combined Image", combined_image)
        cv2.waitKey(100)   # 显示距离文本的时间
 
# 创建一个窗口并设置鼠标回调函数
cv2.namedWindow("Combined Image")
cv2.setMouseCallback("Combined Image", on_mouse_click)
 
try:
    while True:
        try:
            # 等待深度数据帧和RGB数据帧，设置等待时间为10秒
            frames = pipeline.wait_for_frames(timeout_ms=10000)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
 
            if not depth_frame or not color_frame:
                continue
 
            color_data = np.asanyarray(color_frame.get_data())
 
            depth_colormap = np.asanyarray(colorizer.colorize(depth_frame).get_data())
 
            combined_image = np.hstack((color_data, depth_colormap))
            cv2.imshow("Combined Image", combined_image)
 
            key = cv2.waitKey(1)
            if key == 27:
                break
        except RuntimeError as e:
            print(f"等待帧时发生错误: {e}")
 
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
