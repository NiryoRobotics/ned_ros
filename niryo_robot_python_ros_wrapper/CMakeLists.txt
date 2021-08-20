cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_python_ros_wrapper)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  niryo_robot_arm_commander
  niryo_robot_msgs
  niryo_robot_tools_commander
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  actionlib_msgs
  geometry_msgs
  niryo_robot_arm_commander
  niryo_robot_msgs
  niryo_robot_tools_commander
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
    PROGRAMS src/${PROJECT_NAME}/ros_wrapper.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(launch/rpi_python_wrapper.test)
  add_rostest(launch/simulation_python_wrapper.test)
endif()

