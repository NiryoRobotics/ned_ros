message("\n## Reading niryo_robot_iot/CMakeLists.txt...")

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

# enable python module support
catkin_python_setup()

generate_messages(
  DEPENDENCIES
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS 
    scripts/mqtt_client/MqttClient.py
    scripts/mqtt_client/MqttPublisher.py
    scripts/mqtt_client/MqttSubscriber.py
    scripts/mqtt_client/S3Client.py
    scripts/mqtt_client_node.py
    scripts/serial_number_node.py
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