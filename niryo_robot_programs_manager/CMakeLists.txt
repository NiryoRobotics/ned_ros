message("\n## Reading niryo_robot_programs_manager/CMakeLists.txt...")

cmake_minimum_required(VERSION 3.0.2)
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
## Testing ##
#############
find_package(rostest REQUIRED)

execute_process( COMMAND
            uname -m COMMAND tr -d '\n'
        OUTPUT_VARIABLE
            ARCHITECTURE
    )
    
if (${ARCHITECTURE} MATCHES "^(arm.*|aarch64.*|arm64.*)$")
  add_rostest(test/rpi_programs_manager.test)
else()
  add_rostest(test/simulation_programs_manager.test)
endif()


#############
## Install ##
#############

catkin_install_python(PROGRAMS 
    scripts/programs_manager_node.py
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
