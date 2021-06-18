message("\n## Reading niryo_robot_msgs/CMakeLists.txt...")

cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  BusState.msg
  CommandStatus.msg
  HardwareStatus.msg
  MotorHeader.msg
  ObjectPose.msg
  RobotState.msg
  RPY.msg
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

#############
## Install ##
#############

# no scripts to install


if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/doc)
  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
  )
endif()