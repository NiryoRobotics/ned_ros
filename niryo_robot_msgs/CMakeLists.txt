cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  BusState.msg
  CommandJog.msg
  CommandStatus.msg
  HardwareStatus.msg
  MotorHeader.msg
  ObjectPose.msg
  RPY.msg
  RobotState.msg
  SoftwareVersion.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  GetBool.srv
  GetInt.srv
  GetNameDescriptionList.srv
  GetStringList.srv
  SetBool.srv
  SetInt.srv
  SetString.srv
  Trigger.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
