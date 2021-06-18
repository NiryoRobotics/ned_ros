message("\n## Reading niryo_ned_moveit_config_w_gripper1/CMakeLists.txt...")

cmake_minimum_required(VERSION 3.0.2)
project(niryo_ned_moveit_config_w_gripper1)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
