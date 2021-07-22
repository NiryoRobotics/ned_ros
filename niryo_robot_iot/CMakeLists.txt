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
##   Doc   ##
#############

if(BUILD_DOCS)
  find_program(rosdoc_lite_FOUND rosdoc_lite)
  if(rosdoc_lite_FOUND)
      add_custom_target (${PROJECT_NAME}_doc ALL
          rosdoc_lite -o ${CMAKE_CURRENT_BINARY_DIR}/doc ${CMAKE_CURRENT_LIST_DIR}
      )
  else()
    message(WARNING "rosdoc_lite not found. Skipping documentation building")
  endif()

endif(BUILD_DOCS)

#############
## Testing ##
#############

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

##########################
## Static code analysis ##
##########################
find_package(roslint QUIET)

if(roslint_FOUND)
    roslint_python()
else()
    message(WARNING "roslint not found. Skipping roslint target building")
endif()