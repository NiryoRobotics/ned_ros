message("\n## Reading niryo_ned_moveit_config_w_gripper1/CMakeLists.txt...")

cmake_minimum_required(VERSION 3.0.2)
project(niryo_ned_moveit_config_w_gripper1)

find_package(catkin REQUIRED)

catkin_package()

#############
##   Doc   ##
#############

#############
## Testing ##
#############

#############
## Install ##
#############

# no scripts to install

install(
  DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE
  )

install(
  DIRECTORY config 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/doc)
  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
  )
endif()