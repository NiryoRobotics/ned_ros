message("\n## Reading niryo_robot_description/CMakeLists.txt...")

cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_description)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

# Mark other files for installation

install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)

install(
  DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)