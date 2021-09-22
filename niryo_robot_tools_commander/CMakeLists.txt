cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_tools_commander)

find_package(catkin REQUIRED
  COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  niryo_robot_msgs
  std_msgs
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
  geometry_msgs
  niryo_robot_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  actionlib_msgs
  geometry_msgs
  message_runtime
  niryo_robot_msgs
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
    PROGRAMS scripts/tool_commander_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)