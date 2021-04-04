cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        dynamixel_sdk
        niryo_robot_msgs
        message_generation
        utils
)

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
        utils
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/xdriver.cpp
    src/xl320_driver.cpp
    src/xl330_driver.cpp
    src/xl430_driver.cpp
    src/xc430_driver.cpp
    src/dxl_motor_state.cpp
    src/single_motor_cmd.cpp
    src/synchronize_motor_cmd.cpp
)

add_library(${PROJECT_NAME}_core
  src/dxl_driver_core.cpp
  src/dxl_driver.cpp
)

add_executable(${PROJECT_NAME}_node
  src/dxl_driver_node.cpp
)

add_executable(${PROJECT_NAME}_test
  test/test_dynamixel_driver_node.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_core
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_dependencies(${PROJECT_NAME} niryo_robot_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} dynamixel_driver_generate_messages_cpp)
