cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_serial_number)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
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

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
    PROGRAMS scripts/serial_number_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
