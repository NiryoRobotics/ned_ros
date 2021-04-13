cmake_minimum_required(VERSION 2.8.3)
project(joints_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        hardware_interface
        controller_manager
        niryo_robot_msgs
        std_msgs
        sensor_msgs
        dynamixel_driver
        stepper_driver
        actionlib
        common
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
        ${PROJECT_NAME}_core
    CATKIN_DEPENDS
        roscpp
        hardware_interface
        controller_manager
        niryo_robot_msgs
        dynamixel_driver
        stepper_driver
        actionlib
        common
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/calibration_interface.cpp
)

add_library(${PROJECT_NAME}_core
    src/joints_interface_core.cpp
    src/JointHardwareInterface.cpp
)

add_executable(${PROJECT_NAME}_node
  src/joints_interface_node.cpp
)

# add_executable(${PROJECT_NAME}_test
#   test/joints_interface_test.cpp
# )

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

# target_link_libraries(${PROJECT_NAME}_test
#   ${PROJECT_NAME}_core
#   ${catkin_LIBRARIES}
# )

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
