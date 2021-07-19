cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_credentials)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

add_service_files(
  DIRECTORY srv
  FILES
  GetSerial.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
