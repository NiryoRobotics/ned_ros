message("\n## Reading niryo_robot_vision/CMakeLists.txt...")

cmake_minimum_required(VERSION 3.0.2)
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

# enable python module support
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

#############
## Install ##
#############

catkin_install_python(PROGRAMS scripts/camera_publisher_and_services.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark other files for installation
install(
  DIRECTORY doc/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
