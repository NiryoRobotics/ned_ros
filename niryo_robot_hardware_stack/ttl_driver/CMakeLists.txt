message("\n## Reading ttl_driver/CMakeLists.txt...")

cmake_minimum_required(VERSION 3.0.2)
project(ttl_driver)

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
        dynamixel_sdk
        niryo_robot_msgs
        message_generation
        common
)

###############################################
# Declare ROS messages, services and actions ##
###############################################

add_message_files(
  FILES
    DxlArrayMotorHardwareStatus.msg
    DxlMotorCommand.msg
    DxlMotorHardwareStatus.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    SendCustomDxlValue.srv
    ReadCustomDxlValue.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    niryo_robot_msgs
)

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
        dynamixel_sdk
        niryo_robot_msgs
        common
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
  src/ttl_driver.cpp
  src/ttl_driver_core.cpp
  src/abstract_motor_driver.cpp
  src/stepper_driver.cpp
)

add_executable(${PROJECT_NAME}_node
  src/ttl_driver_node.cpp
)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}
    niryo_robot_msgs_generate_messages_cpp
    ttl_driver_generate_messages_cpp
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

find_program(rosdoc_lite_FOUND rosdoc_lite)
if(BUILD_DOCS AND rosdoc_lite_FOUND)
    add_custom_target (${PROJECT_NAME}_doc ALL
        rosdoc_lite -o ${CMAKE_CURRENT_BINARY_DIR}/doc ${CMAKE_CURRENT_LIST_DIR}
    )

endif()

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  if(BUILD_UNIT_TESTS)
  endif(BUILD_UNIT_TESTS)

  if(BUILD_INTEGRATION_TESTS)
    find_package(catkin REQUIRED rostest)

    add_rostest_gtest(${PROJECT_NAME}_unit_tests
      test/ttl_driver_unit_tests.test
      test/ttl_driver_unit_tests.cpp
    )
    if(TARGET ${PROJECT_NAME}_unit_tests)
      target_link_libraries(${PROJECT_NAME}_unit_tests
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
      )
    endif()

#    add_rostest_gtest(${PROJECT_NAME}_services_tests
#       test/ttl_driver_service_client.test
#       test/ttl_driver_service_client.cpp
#    )
#    if(TARGET ${PROJECT_NAME}_integration_tests)
#      target_link_libraries(${PROJECT_NAME}_integration_tests
#        ${PROJECT_NAME}
#      )
#    endif()

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


if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/doc/)
  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
  )
endif()

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

##########################
## Static code analysis ##
##########################
find_package(roslint QUIET)

if(roslint_FOUND)
    roslint_cpp()
endif()