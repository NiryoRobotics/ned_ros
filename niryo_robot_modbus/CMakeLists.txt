cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_modbus)

find_package(catkin REQUIRED COMPONENTS
  niryo_robot_msgs
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  niryo_robot_msgs
  std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
    PROGRAMS scripts/modbus_server_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)