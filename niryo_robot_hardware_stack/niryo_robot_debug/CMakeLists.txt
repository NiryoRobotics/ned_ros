message("\n## Reading niryo_robot_debug/CMakeLists.txt...")

cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_debug)

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
        rospy
        std_msgs
        niryo_robot_msgs
        mcp_can_rpi
        dynamixel_sdk
        roslint
)
 
find_package(Boost REQUIRED
    COMPONENTS
        system
        program_options
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
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    niryo_robot_msgs
    mcp_can_rpi
    dynamixel_sdk
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
add_executable(dxl_debug_tools
    src/dxl_debug.cpp
    src/dxl_tools.cpp
)

#
# wiringPi should be installed only on a Raspberry Pi board
#

execute_process( COMMAND
        uname -m COMMAND tr -d '\n'
    OUTPUT_VARIABLE
        ARCHITECTURE
)

if (${ARCHITECTURE} MATCHES "^(arm.*|aarch64.*|arm64.*)$")
    message(STATUS "wiringPi library is required - arm processor")
    target_link_libraries(dxl_debug_tools
        ${catkin_LIBRARIES} 
        ${Boost_LIBRARIES}
        -lwiringPi -lrt
    )
else()
    message(STATUS "wiringPi library not required")
    target_link_libraries(dxl_debug_tools
        ${catkin_LIBRARIES} 
        ${Boost_LIBRARIES}
    )
endif()

#############
##   Doc   ##
#############

if (BUILD_DOCS)
    add_custom_target (niryo_robot_debug_doc ALL
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
        ${PROJECT_NAME}_core
      )
    endif()
  endif(BUILD_UNIT_TESTS)

  if(BUILD_INTEGRATION_TESTS)
    find_package(catkin REQUIRED rostest)

    add_rostest_gtest(
      ${PROJECT_NAME}_integration_tests
      test/test_niryo_robot_debug_node.test
      test/test_niryo_robot_debug_node.cpp
    )
    if(TARGET ${PROJECT_NAME}_integration_tests)
      target_link_libraries(
        ${PROJECT_NAME}_integration_tests
        ${PROJECT_NAME}_core
      )
    endif()
  endif(BUILD_INTEGRATION_TESTS)

endif(CATKIN_ENABLE_TESTING)

#############
## Install ##
#############

## nothing for now


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

roslint_cpp()
