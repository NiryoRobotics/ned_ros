message("\n## Reading niryo_robot_gazebo/CMakeLists.txt...")

cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_gazebo)

find_package(catkin COMPONENTS
  gazebo_ros
)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

# Mark other files for installation
install(
  DIRECTORY demo/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/demo
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media
)

install(
  DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

install(
  DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)

if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/doc/${PROJECT_NAME})
  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
  )
endif()