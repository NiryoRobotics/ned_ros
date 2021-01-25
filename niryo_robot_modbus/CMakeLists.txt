cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_modbus)

find_package(catkin REQUIRED COMPONENTS
  niryo_robot_msgs
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
