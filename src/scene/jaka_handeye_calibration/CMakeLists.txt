cmake_minimum_required(VERSION 3.0.2)
project(jaka_handeye_calibration)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/capture_handeye_samples.py
  scripts/capture_handeye_samples_viz.py
  scripts/solve_handeye.py
  scripts/verify_handeye.py
  scripts/publish_handeye_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
