cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_unit_tests)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rostest
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
