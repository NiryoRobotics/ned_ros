cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_user_interface)

find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
    PROGRAMS scripts/user_interface_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
    PROGRAMS scripts/jog_interface/client_jog_interface_mouse.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
