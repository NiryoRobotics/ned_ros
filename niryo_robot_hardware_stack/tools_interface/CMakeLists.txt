cmake_minimum_required(VERSION 2.8.3)
project(tools_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        std_msgs
        dynamixel_driver
        message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

add_service_files(
  DIRECTORY
    srv
  FILES
    CloseGripper.srv
    OpenGripper.srv
    PingDxlTool.srv
    PullAirVacuumPump.srv
    PushAirVacuumPump.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        std_msgs
        dynamixel_driver
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/tools_interface_core.cpp
    src/tool_state.cpp
)

add_executable(${PROJECT_NAME}_node
  src/tools_interface_node.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
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

add_dependencies(${PROJECT_NAME} tools_interface_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} dynamixel_driver_generate_messages_cpp)