message("\n## Reading niryo_robot_system_api_client/CMakeLists.txt...")

cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_system_api_client)

find_package(catkin REQUIRED COMPONENTS
)

# enable python module support
catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS 
    scripts/system_api_client_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark other files for installation
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/doc/${PROJECT_NAME})
  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
  )
endif()