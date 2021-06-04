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