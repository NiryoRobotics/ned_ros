message("\n## Reading niryo_robot_rpi/CMakeLists.txt...")

cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_rpi)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  sensor_msgs
  niryo_robot_msgs
  rospy
  roscpp
  std_msgs
)

add_message_files(
  FILES
  DigitalIOState.msg
  LogStatus.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  ChangeMotorConfig.srv
  GetDigitalIO.srv
  LedBlinker.srv
  SetDigitalIO.srv
)

# enable python module support
catkin_python_setup()

generate_messages(
  DEPENDENCIES
  niryo_robot_msgs
  std_msgs
  actionlib_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

catkin_install_python(PROGRAMS 
    scripts/button_manager.py
    scripts/digital_io_panel.py
    scripts/fake_rpi_node.py
    scripts/led_manager.py
    scripts/ros_log_manager.py
    scripts/rpi_node.py
    scripts/shutdown_manager.py
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
find_package(roslint QUIET)

if(roslint_FOUND)
    roslint_python()
endif()
