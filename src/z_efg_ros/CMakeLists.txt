cmake_minimum_required(VERSION 3.0.2)
project(z_efg_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  message_generation
)

add_message_files(
  FILES
  GripperCmd.msg
  GripperState.msg
    InterpretedState.msg

)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

# 单夹爪节点
add_executable(single_gripper_node src/single_gripper_node.cpp)
target_link_libraries(single_gripper_node ${catkin_LIBRARIES} pthread)

# 多夹爪节点
add_executable(multi_gripper_node src/multi_gripper_node.cpp)
add_dependencies(multi_gripper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_gripper_node ${catkin_LIBRARIES} pthread)

install(TARGETS single_gripper_node multi_gripper_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
