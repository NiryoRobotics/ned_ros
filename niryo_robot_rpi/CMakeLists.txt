cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_rpi)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  niryo_robot_msgs
  rospy
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  DigitalIOState.msg
  LogStatus.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  ChangeMotorConfig.srv
  GetDigitalIO.srv
  LedBlinker.srv
  SetDigitalIO.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  niryo_robot_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  actionlib_msgs
  message_runtime
  niryo_robot_msgs
  sensor_msgs
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
    PROGRAMS scripts/fake_rpi_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
    PROGRAMS scripts/rpi_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
