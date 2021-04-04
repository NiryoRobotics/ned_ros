cmake_minimum_required(VERSION 2.8.3)
project(fake_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        niryo_robot_msgs
        std_msgs
        hardware_interface
        controller_manager
        tools_interface
        joints_interface
        conveyor_interface
        actionlib
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        niryo_robot_msgs
        std_msgs
        hardware_interface
        controller_manager
        tools_interface
        joints_interface
        conveyor_interface
        actionlib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/fake_interface_core.cpp
    src/FakeJointHardwareInterface.cpp
)

add_executable(${PROJECT_NAME}_node
  src/fake_interface_node.cpp
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

add_dependencies(${PROJECT_NAME} niryo_robot_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} conveyor_interface_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} tools_interface_generate_messages_cpp)
