cmake_minimum_required(VERSION 2.8.3)
project(niryo_ned_moveit_config_standalone)

find_package(catkin REQUIRED COMPONENTS
  niryo_robot_description
)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
