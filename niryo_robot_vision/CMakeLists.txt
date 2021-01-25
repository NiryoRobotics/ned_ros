cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_vision)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  niryo_robot_msgs
  sensor_msgs
)

add_service_files(
  DIRECTORY srv
  FILES
  DebugColorDetection.srv
  DebugMarkers.srv
  ObjDetection.srv
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
  niryo_robot_msgs
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
