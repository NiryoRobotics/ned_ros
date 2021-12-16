cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_status)
message( STATUS "\n## Reading ${PROJECT_NAME}/CMakeLists.txt...")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  niryo_robot_msgs
  std_msgs
)

add_message_files(
  FILES
  RobotStatus.msg
)

# enable python module support
catkin_python_setup()

generate_messages(
  DEPENDENCIES
  niryo_robot_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  niryo_robot_msgs
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
##   Doc   ##
#############

find_program(rosdoc_lite_FOUND rosdoc_lite)
if(rosdoc_lite_FOUND)
    add_custom_target (${PROJECT_NAME}_doc ALL
        rosdoc_lite -o ${CMAKE_CURRENT_BINARY_DIR}/doc ${CMAKE_CURRENT_LIST_DIR}
    )
else()
  message(WARNING "rosdoc_lite not found. Skipping documentation building")
endif()

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

  ##########################
  ## Static code analysis ##
  ##########################
  find_package(roslint QUIET)

  if(roslint_FOUND)
      set(ROSLINT_PYTHON_OPTS "1>&2")
      roslint_python()
  else()
      message(WARNING "roslint not found. Skipping roslint target building")
  endif()
endif()

#############
## Install ##
#############

catkin_install_python(PROGRAMS
    scripts/robot_status_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark other files for installation

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

if (rosdoc_lite_FOUND)
  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
  )
endif()

