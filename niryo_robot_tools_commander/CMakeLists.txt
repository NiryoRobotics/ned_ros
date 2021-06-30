cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_tools_commander)

find_package(catkin REQUIRED
  COMPONENTS
  actionlib_msgs
  message_generation
  std_msgs
  geometry_msgs
  niryo_robot_msgs
)

add_message_files(
  FILES
  TCP.msg
  ToolCommand.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  SetTCP.srv
)

add_action_files(
  DIRECTORY action
  FILES
  Tool.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
  niryo_robot_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
