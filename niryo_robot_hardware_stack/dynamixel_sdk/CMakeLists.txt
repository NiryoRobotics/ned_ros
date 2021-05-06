message("\n## Reading mcp_can_rpi/CMakeLists.txt...")

cmake_minimum_required(VERSION 3.0.2)
project(dynamixel_sdk)

option(BUILD_DOCS "Build documentation" OFF)
option(BUILD_UNIT_TESTS "Build the unit tests" OFF)
option(BUILD_INTEGRATION_TESTS "Build the integration tests" OFF)

## c++ options
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall  -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
    COMPONENTS
      roscpp
)

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
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare libs and execs
add_library(dynamixel_sdk
  src/packet_handler.cpp
  src/protocol1_packet_handler.cpp
  src/protocol2_packet_handler.cpp
  src/group_sync_read.cpp
  src/group_sync_write.cpp
  src/group_bulk_read.cpp
  src/group_bulk_write.cpp
  src/port_handler.cpp
  src/port_handler_linux.cpp
)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(dynamixel_sdk
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link executable targets against

# wiringPi should be installed only on a Raspberry Pi board
if(CMAKE_CROSSCOMPILING)
    message(STATUS "Cross compiling : wiringPi library is required")
    target_link_libraries(dynamixel_sdk
        ${catkin_LIBRARIES}
        -lwiringPi -lrt
    )
else()
    execute_process( COMMAND
            uname -m COMMAND tr -d '\n'
        OUTPUT_VARIABLE
            ARCHITECTURE
    )

    if (${ARCHITECTURE} MATCHES "^(arm.*|aarch64.*|arm64.*)$")
        message(STATUS "wiringPi library is required - arm processor")
        target_link_libraries(dynamixel_sdk
            ${catkin_LIBRARIES}
            -lwiringPi -lrt
        )
    else()
        message(STATUS "wiringPi library not required")
        target_link_libraries(dynamixel_sdk
            ${catkin_LIBRARIES}
        )
    endif()
endif()

#############
##   Doc   ##
#############

if (BUILD_DOCS)
    add_custom_target (dynamixel_sdk_doc ALL
        rosdoc_lite -o ${CATKIN_DEVEL_PREFIX}/doc/${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}
    )

endif()

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  if(BUILD_UNIT_TESTS)
    catkin_add_gtest(
      ${PROJECT_NAME}_unit_tests
      test/unit_tests.cpp
    )
    if(TARGET ${PROJECT_NAME}_unit_tests)
      target_link_libraries(
        ${PROJECT_NAME}_unit_tests
        ${PROJECT_NAME}
      )
    endif()
  endif(BUILD_UNIT_TESTS)

  if(BUILD_INTEGRATION_TESTS)
      find_package(catkin REQUIRED rostest)

      add_rostest_gtest(
        ${PROJECT_NAME}_integration_tests
        test/test_dynamixel_sdk_node.test
        test/test_dynamixel_sdk_node.cpp
      )
      if(TARGET ${PROJECT_NAME}_integration_tests)
        target_link_libraries(
          ${PROJECT_NAME}_integration_tests
          ${PROJECT_NAME}
        )
      endif()
  endif(BUILD_INTEGRATION_TESTS)

endif(CATKIN_ENABLE_TESTING)

#############
## Install ##
#############

install(TARGETS dynamixel_sdk
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY doc/
  DESTINATION ${CATKIN_DEVEL_PREFIX}/doc/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

##########################
## Static code analysis ##
##########################
find_package(roslint QUIET)

if(roslint_FOUND)
    roslint_cpp()
endif()
