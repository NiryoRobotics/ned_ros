cmake_minimum_required(VERSION 2.8.3)
project(common)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/model/abstract_motor_state.cpp
    src/model/conveyor_state.cpp
    src/model/dxl_motor_state.cpp
    src/model/joint_state.cpp
    src/model/tool_state.cpp
    src/model/stepper_motor_state.cpp
    src/model/motor_type_enum.cpp
    src/model/dxl_command_type_enum.cpp
    src/model/stepper_command_type_enum.cpp
    src/model/single_motor_cmd.cpp
    src/model/synchronize_motor_cmd.cpp
    src/model/stepper_motor_cmd.cpp
    src/model/synchronize_stepper_motor_cmd.cpp
    src/model/stepper_calibration_status_enum.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
