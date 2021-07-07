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

if(CATKIN_ENABLE_TESTING)

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
endif(CATKIN_ENABLE_TESTING)

#############
## Install ##
#############

catkin_install_python(PROGRAMS 
    scripts/programs_manager_node.py
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
find_package(roslint)

if(roslint_FOUND)
    roslint_python()
endif()
