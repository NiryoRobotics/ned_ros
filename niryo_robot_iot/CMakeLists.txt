cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_iot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

add_service_files(
  DIRECTORY srv
  FILES
  GetSerial.srv
  MqttPublish.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
