cmake_minimum_required(VERSION 2.8.3)
project(stepper_driver)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        mcp_can_rpi
        niryo_robot_msgs
        message_generation
        common
)

add_message_files(
  FILES
    StepperArrayMotorHardwareStatus.msg
    StepperMotorCommand.msg
    StepperMotorHardwareStatus.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    StepperCmd.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    niryo_robot_msgs
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        mcp_can_rpi
        niryo_robot_msgs
        common
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/stepper_driver_core.cpp
    src/stepper_driver.cpp
)

add_executable(${PROJECT_NAME}_node
  src/stepper_driver_node.cpp
)

add_executable(${PROJECT_NAME}_test
  test/stepper_driver_test.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)
#
# wiringPi should be installed only on a Raspberry Pi board
#

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )

if (${ARCHITECTURE} MATCHES "^(arm.*|aarch64.*|arm64.*)$")
    message(STATUS "wiringPi library is required - arm processor")
    target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES} 
        -lwiringPi -lrt)
else()
    message(STATUS "wiringPi library not required")
    target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES} 
    )
endif()

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_dependencies(${PROJECT_NAME} stepper_driver_generate_messages_cpp)
