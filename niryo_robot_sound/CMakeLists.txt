cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_sound)
message( STATUS "\n## Reading ${PROJECT_NAME}/CMakeLists.txt...")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  niryo_robot_msgs
  std_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  SoundList.msg
  SoundObject.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  ManageSound.srv
  PlaySound.srv
  TextToSpeech.srv
)

generate_messages(
  DEPENDENCIES
  niryo_robot_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  niryo_robot_msgs
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
##   Doc   ##
#############

find_program(rosdoc_lite_FOUND rosdoc_lite)
if(rosdoc_lite_FOUND)
    add_custom_target (${PROJECT_NAME}_doc ALL
        rosdoc_lite -o ${CMAKE_CURRENT_BINARY_DIR}/doc ${CMAKE_CURRENT_LIST_DIR}
    )
else()
  message(WARNING "rosdoc_lite not found. Skipping documentation building")
endif()

##########################
## Static code analysis ##
##########################
find_package(roslint QUIET)

if(roslint_FOUND)
    set(ROSLINT_PYTHON_OPTS "1>&2")
    roslint_python()
else()
    message(WARNING "roslint not found. Skipping roslint target building")
endif()


#############
## Install ##
#############

catkin_install_python(PROGRAMS
    scripts/sound_interface_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Mark other files for installation
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY niryo_robot_state_sounds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/niryo_robot_state_sounds
)

install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

if (rosdoc_lite_FOUND)
  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
  )
endif()
