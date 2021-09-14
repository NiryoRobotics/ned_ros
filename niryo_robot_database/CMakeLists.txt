cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_database)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
)

add_message_files(
  FILES
  Log.msg
  Metric.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  AddLog.srv
  GetAllLogs.srv
  GetAllMetrics.srv
  GetSettings.srv
  RmLogSinceDate.srv
  SetMetric.srv
  SetSettings.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
    PROGRAMS scripts/database_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
