cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_commander)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  niryo_robot_msgs
  niryo_robot_tools
)

add_message_files(
  FILES
  ArmMoveCommand.msg
  PausePlanExecution.msg
  RobotCommand.msg
  ShiftPose.msg
  CommandJog.msg
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
  geometry_msgs
  niryo_robot_msgs
  niryo_robot_tools
)

catkin_package(
  CATKIN_DEPENDS
  actionlib_msgs
  geometry_msgs
  niryo_robot_msgs
  niryo_robot_tools
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

