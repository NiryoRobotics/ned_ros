cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_status)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  niryo_robot_msgs
)

add_message_files(
  FILES
  RobotStatus.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  niryo_robot_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  niryo_robot_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

