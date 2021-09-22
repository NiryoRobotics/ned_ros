cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_vision)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  niryo_robot_msgs
  sensor_msgs
)

add_message_files(
  FILES
  ImageParameters.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  DebugColorDetection.srv
  DebugMarkers.srv
  ObjDetection.srv
  SetImageParameter.srv
  TakePicture.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  niryo_robot_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  niryo_robot_msgs
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
    PROGRAMS scripts/camera_publisher_and_services.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
