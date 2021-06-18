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
## Install ##
#############

catkin_install_python(PROGRAMS 
    scripts/tool_commander_node.py
    scripts/tool_ros_commander_interface.py
    scripts/tools_classes.py
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
