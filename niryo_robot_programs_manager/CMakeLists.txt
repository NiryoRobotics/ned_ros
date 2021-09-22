cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_programs_manager)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(
  FILES
  ProgramLanguage.msg
  ProgramLanguageList.msg
  ProgramList.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  ExecuteProgram.srv
  GetProgram.srv
  GetProgramAutorunInfos.srv
  GetProgramList.srv
  ManageProgram.srv
  SetProgramAutorun.srv
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
    PROGRAMS scripts/programs_manager_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(launch/rpi_programs_manager.test)
  add_rostest(launch/simulation_programs_manager.test)
endif()
