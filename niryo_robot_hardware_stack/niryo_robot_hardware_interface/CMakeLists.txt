message("\n## Reading niryo_robot_hardware_interface/CMakeLists.txt...")

cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_hardware_interface)

option(BUILD_DOCS "Build documentation" OFF)
option(BUILD_UNIT_TESTS "Build the unit tests" OFF)
option(BUILD_INTEGRATION_TESTS "Build the integration tests" OFF)

## c++ options
## Compile as C++14, supported in ROS Melodic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall  -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        tools_interface
        joints_interface
        conveyor_interface
        cpu_interface
        fake_interface
        message_generation
        niryo_robot_msgs
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
    CATKIN_DEPENDS
        roscpp
        tools_interface
        joints_interface
        conveyor_interface
        cpu_interface
        fake_interface
        message_runtime
        niryo_robot_msgs
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
add_library(${PROJECT_NAME}
    src/hardware_interface.cpp
)

add_executable(${PROJECT_NAME}_node
  src/hardware_interface_node.cpp
)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}
    conveyor_interface_generate_messages_cpp
)

add_dependencies(${PROJECT_NAME}
    tools_interface_generate_messages_cpp
)
add_dependencies(${PROJECT_NAME}
    niryo_robot_msgs_generate_messages_cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

#############
##   Doc   ##
#############

if (BUILD_DOCS)
    add_custom_target (${PROJECT_NAME}_doc ALL
        rosdoc_lite -o ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc ${CMAKE_CURRENT_LIST_DIR}
    )

endif()

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  if(BUILD_INTEGRATION_TESTS)
    find_package(catkin REQUIRED rostest)

    add_rostest_gtest(${PROJECT_NAME}_integration_tests
      test/hardware_interface_service_client.test
      test/hardware_interface_service_client.cpp
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

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY doc/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
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
