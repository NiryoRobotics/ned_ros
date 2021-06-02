cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_user_interface)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(
  FILES
  MatlabMoveResult.msg
)

catkin_python_setup()

generate_messages()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
