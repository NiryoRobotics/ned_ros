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

# no testings : external lib

#############
## Install ##
#############

catkin_install_python(PROGRAMS 
    scripts/jog_interface/client_jog_interface_example.py
    scripts/jog_interface/client_jog_interface_mouse.py
    scripts/user_interface_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark other files for installation
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
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
