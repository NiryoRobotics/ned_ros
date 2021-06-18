message("\n## Reading niryo_robot_user_interface/CMakeLists.txt...")

cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_user_interface)

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
    scripts/user_interface_node.py
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
