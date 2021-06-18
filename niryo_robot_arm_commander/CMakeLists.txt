message("\n## Reading niryo_robot_arm_commander/CMakeLists.txt...")

cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_arm_commander)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  niryo_robot_msgs
)

add_message_files(
  FILES
  ArmMoveCommand.msg
  PausePlanExecution.msg
  ShiftPose.msg
  CommandJog.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  GetFK.srv
  GetIK.srv
  JogShift.srv
)

add_action_files(
  DIRECTORY action
  FILES
  RobotMove.action
)

# enable python module support
catkin_python_setup()

generate_messages(
  DEPENDENCIES
  geometry_msgs
  actionlib_msgs
  niryo_robot_msgs
)

catkin_package(
  CATKIN_DEPENDS
  actionlib_msgs
  geometry_msgs
  niryo_robot_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS 
    scripts/arm_commander.py
    scripts/ArmParametersValidator.py
    scripts/jog_controller.py
    scripts/motor_debug.py
    scripts/robot_commander_node.py
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
