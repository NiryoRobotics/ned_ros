cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_rpi)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  sensor_msgs
  niryo_robot_msgs
  rospy
  roscpp
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
  niryo_robot_msgs
  std_msgs
  actionlib_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
