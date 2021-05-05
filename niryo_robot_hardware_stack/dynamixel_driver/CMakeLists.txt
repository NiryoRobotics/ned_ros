message("\n## Reading dynamixel_driver/CMakeLists.txt...")

cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_driver)

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
        dynamixel_sdk
        niryo_robot_msgs
        message_generation
        common
        roslint
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
        ${PROJECT_NAME}_core
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
    src/xdriver.cpp
    src/xl320_driver.cpp
    src/xl330_driver.cpp
    src/xl430_driver.cpp
    src/xc430_driver.cpp
)

add_library(${PROJECT_NAME}_core
  src/dxl_driver_core.cpp
  src/dxl_driver.cpp
)

add_executable(${PROJECT_NAME}_node
  src/dxl_driver_node.cpp
)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}
    niryo_robot_msgs_generate_messages_cpp
)

add_dependencies(${PROJECT_NAME}
    dynamixel_driver_generate_messages_cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_core
  ${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
)

#############
##   Doc   ##
#############

if (BUILD_DOCS)
    add_custom_target (dynamixel_doc ALL
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
      test/test_dynamixel_driver_node.test
      test/test_dynamixel_driver_node.cpp
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

install(
  TARGETS ${PROJECT_NAME}_core
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
