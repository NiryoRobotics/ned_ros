cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_arm_commander)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  niryo_robot_msgs
)

add_message_files(
  FILES
  ArmMoveCommand.msg
  PausePlanExecution.msg
  ShiftPose.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  GetFK.srv
  GetIK.srv
  JogShift.srv
)

add_action_files(
  DIRECTORY action
  FILES
  RobotMove.action
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  niryo_robot_msgs
)

catkin_package(
  CATKIN_DEPENDS
  actionlib_msgs
  geometry_msgs
  message_runtime
  niryo_robot_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
    PROGRAMS scripts/robot_commander_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

