cmake_minimum_required(VERSION 2.8.3)
project(stepper_driver)
message("\n## Reading ${PROJECT_NAME}/CMakeLists.txt...")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
        mcp_can_rpi
        message_generation
        niryo_robot_msgs
        roscpp
        std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

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
    niryo_robot_msgs
    std_msgs
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
        ${PROJECT_NAME}_core
    CATKIN_DEPENDS
        mcp_can_rpi
        message_runtime
        niryo_robot_msgs
        roscpp
        std_msgs
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

add_library(${PROJECT_NAME}
    src/conveyor_state.cpp
    src/stepper_motor_cmd.cpp
    src/stepper_motor_state.cpp
)

add_library(${PROJECT_NAME}_core
    src/${PROJECT_NAME}.cpp
    src/${PROJECT_NAME}_core.cpp
)

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)

add_executable(${PROJECT_NAME}_test
test/${PROJECT_NAME}_test.cpp
)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME} 
  ${PROJECT_NAME}_generate_messages_cpp
)

## Specify libraries to link executable targets against
execute_process( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )

# wiringPi should be installed only on a Raspberry Pi board
if(CMAKE_CROSSCOMPILING)
    message(STATUS "Cross compiling : wiringPi library is required")
    target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        -lwiringPi -lrt -lcrypt
    )
else()

    if ("${ARCHITECTURE}" MATCHES "^(arm.*|aarch64.*|arm64.*)$")
        message(STATUS "wiringdxl_debug_toolsPi library is required for ${PROJECT_NAME} (ARM processor)")
        target_link_libraries(${PROJECT_NAME}
            ${catkin_LIBRARIES}
            -lwiringPi -lrt -lcrypt
        )
    else()
        message(STATUS "wiringPi library not required")
        target_link_libraries(${PROJECT_NAME}
            ${catkin_LIBRARIES}
        )
    endif()
endif()

target_link_libraries(${PROJECT_NAME}_core
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
)

target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}_core
)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# Mark other files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

##########################
## Static code analysis ##
##########################

find_package(roslint QUIET)

if(roslint_FOUND)
    roslint_cpp()
else()
    message(WARNING "roslint not found. Skipping roslint target building")
endif()
