cmake_minimum_required(VERSION 3.0.2)
project(chassis)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(on_track_station_control on_track_station_control_v2.cpp)
target_link_libraries(on_track_station_control ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/track_end_qr_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS on_track_station_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES on_track_station_control_v2.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)