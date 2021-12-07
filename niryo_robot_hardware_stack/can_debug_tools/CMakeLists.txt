cmake_minimum_required(VERSION 3.0.2)
project(can_debug_tools)
message( STATUS "\n## Reading ${PROJECT_NAME}/CMakeLists.txt...")

#retrieve architecture
execute_process( COMMAND
            uname -m COMMAND tr -d '\n'
        OUTPUT_VARIABLE
            ARCHITECTURE
    )

## c++ options
## Compile as C++14, supported in ROS Melodic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall  -Werror)

set(HARDWARE_VERSION "ned2" CACHE STRING "Robot model target")
# for preprocessor directives
if ("${HARDWARE_VERSION}" STREQUAL "one")
    add_definitions(-DNIRYO_ONE)
elseif ("${HARDWARE_VERSION}" STREQUAL "ned")
    add_definitions(-DNIRYO_NED)
elseif ("${HARDWARE_VERSION}" STREQUAL "ned2")
    add_definitions(-DNIRYO_NED2)
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED
    COMPONENTS
      mcp_can_rpi
)
 
find_package(Boost REQUIRED
    COMPONENTS
        program_options
)

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  append_coverage_compiler_flags()
endif()

###############################################
# Declare ROS messages, services and actions ##
###############################################

###################################
## catkin specific configuration ##
###################################

## INCLUDE_DIRS: if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS 
    mcp_can_rpi
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

## Declare libs and execs
add_library(${PROJECT_NAME}_core
    src/can_tools.cpp)

add_executable(${PROJECT_NAME}
    src/can_debug.cpp)

target_link_libraries(${PROJECT_NAME} 
    ${PROJECT_NAME}_core
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    pthread
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

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  if ("${HARDWARE_VERSION}" STREQUAL "ned" OR "${HARDWARE_VERSION}" STREQUAL "one")
    if ("${ARCHITECTURE}" MATCHES "^(arm.*|aarch64.*|arm64.*)$")
      catkin_add_gtest(
        ${PROJECT_NAME}_unit_tests
        test/can_tools_unit_tests.cpp
      )
    endif()

    if(TARGET ${PROJECT_NAME}_unit_tests)
      target_link_libraries(${PROJECT_NAME}_unit_tests
          ${catkin_LIBRARIES}
          ${PROJECT_NAME}_core
      )

      # Create a target ${PROJECT_NAME}_coverage_report
      if(ENABLE_COVERAGE_TESTING)
        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
        add_code_coverage(
          NAME ${PROJECT_NAME}_coverage_report
          DEPENDENCIES tests
        )
      endif()
    endif()
  endif()


  ##########################
  ## Static code analysis ##
  ##########################
  find_package(roslint QUIET)

  if(roslint_FOUND)
      roslint_cpp()
  else()
      message(WARNING "roslint not found. Skipping roslint target building")
  endif()
endif()

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if (rosdoc_lite_FOUND)
  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
  )
endif()
