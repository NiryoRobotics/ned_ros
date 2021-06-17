cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_user_interface)

find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
