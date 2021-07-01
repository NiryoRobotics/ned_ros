message("\n## Reading niryo_robot_tools_commander/CMakeLists.txt...")

cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_tools_commander)

find_package(catkin REQUIRED
  COMPONENTS
  actionlib_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  ToolCommand.msg
)

add_action_files(
  DIRECTORY action
  FILES
  Tool.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime)

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
    scripts/tool_commander_node.py
    scripts/tool_ros_command_interface.py
    scripts/tools_classes.py
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
