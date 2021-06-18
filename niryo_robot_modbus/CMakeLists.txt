message("\n## Reading niryo_robot_modbus/CMakeLists.txt...")

cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_modbus)

find_package(catkin REQUIRED COMPONENTS
  niryo_robot_msgs
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS 
    scripts/coil_data_block.py
    scripts/data_block.py
    scripts/discrete_input_data_block.py
    scripts/holding_register_data_block.py
    scripts/input_register_data_block.py
    scripts/modbus_internal_test.py
    scripts/modbus_server_node.py
    scripts/modbus_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

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
  DIRECTORY examples/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples
)

if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/doc)
  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
  )
endif()