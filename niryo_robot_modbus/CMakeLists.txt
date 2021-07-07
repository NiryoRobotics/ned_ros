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
##   Doc   ##
#############

if(BUILD_DOCS)
  find_program(rosdoc_lite_FOUND rosdoc_lite)
  if(rosdoc_lite_FOUND)
      add_custom_target (${PROJECT_NAME}_doc ALL
          rosdoc_lite -o ${CMAKE_CURRENT_BINARY_DIR}/doc ${CMAKE_CURRENT_LIST_DIR}
      )
  else()
    message(WARNING "rosdoc_lite not found. Skipping documentation building")
  endif()

endif(BUILD_DOCS)

#############
## Testing ##
#############

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

##########################
## Static code analysis ##
##########################
find_package(roslint)

if(roslint_FOUND)
    roslint_python()
endif()
